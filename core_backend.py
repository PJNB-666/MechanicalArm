import re
import sys
import time
import socket
import pyaubo_sdk
import config
import utils


# ---------- 运动学常量 ----------
ARRIVAL_TOLERANCE_MM = 1.5      # 到位判定阈值
ARRIVAL_POLL_INTERVAL = 0.1     # 到位轮询周期
ARRIVAL_TIMEOUT_S = 30          # 到位最大等待
SETTLE_DELAY = 0.2              # 到位后伺服稳定延时

# moveJoint 默认速度/加速度
JOINT_VEL = 0.4
JOINT_ACC = 0.4

# moveLine 默认速度/加速度（低速，用于精准下探）
LINE_VEL = 0.15
LINE_ACC = 0.2

# 夹爪 Modbus 输出值
GRIPPER_OPEN_VAL = 1000
GRIPPER_CLOSE_VAL = 0
GRIPPER_ACTION_DELAY = 1.0

# 堆叠层高（mm）
STACK_HEIGHT_PER_LAYER = 12.0


class VisionProcessor:
    """解析 VisionMaster 透传的像素坐标，转换为机械臂基坐标系坐标。"""

    def __init__(self):
        self.transform_engine = utils.NPointTransform()

    def parse_and_transform(self, raw_data):
        try:
            valid_nums = []
            for p in raw_data.split(','):
                found = re.findall(r"[-+]?\d*\.\d+|\d+", p)
                if found:
                    valid_nums.append(float(found[0]))

            if len(valid_nums) < 2:
                return None, None

            u, v = valid_nums[0], valid_nums[1]
            print(f"视觉原始坐标 -> U:{u}, V:{v}")

            tx, ty = self.transform_engine.pixel_to_robot_base(u, v)
            if tx is None or ty is None:
                return None, None

            # 静态偏移补偿：解决实验室昼夜光照导致的视觉漂移
            tx += config.VISION_OFFSET_X
            ty += config.VISION_OFFSET_Y
            print(f"补偿后坐标 -> X:{tx:.1f}, Y:{ty:.1f}")
            return tx, ty
        except Exception:
            return None, None


class AuboController:
    """AUBO 机械臂 RPC 控制封装。"""

    def __init__(self):
        self.rpc = pyaubo_sdk.RpcClient()
        self.robot = None

    # ---------- 连接 ----------
    def connect(self):
        self.rpc.connect(config.ROBOT_IP, config.ROBOT_PORT)
        if not self.rpc.hasConnected():
            return False

        try:
            self.rpc.login("aubo", "<your_password>")
        except Exception:
            pass

        names = self.rpc.getRobotNames()
        robot_name = names[0] if names else "rob1"
        self.robot = self.rpc.getRobotInterface(robot_name)
        return True

    def disconnect(self):
        self.rpc.disconnect()

    # ---------- 运动控制 ----------
    def _build_pose(self, x, y, z):
        """将 mm 坐标 + 固定姿态打包为 SDK 所需的位姿向量（m + rad）。"""
        return [
            float(x) / 1000.0,
            float(y) / 1000.0,
            float(z) / 1000.0,
            float(config.FIXED_RX),
            float(config.FIXED_RY),
            float(config.FIXED_RZ),
        ]

    def get_optimal_joints(self, target_xyz):
        """对目标位姿求逆解，从多组 IK 解中选择关节变化量最小的一组。

        优势：避免机械臂大幅翻转，规避奇异点，提升运动平滑性。
        """
        if not self.robot:
            return None

        target_pose = self._build_pose(*target_xyz)

        try:
            algo = self.robot.getRobotAlgorithm()
            curr_joints = self.robot.getRobotState().getJointPositions()
            ik_solutions = algo.inverseKinematics(curr_joints, target_pose)

            if isinstance(ik_solutions, int) or not ik_solutions:
                return None

            best_solution = None
            min_delta = float('inf')
            for sol in ik_solutions:
                if not isinstance(sol, (list, tuple)):
                    continue
                delta = sum(abs(s - c) for s, c in zip(sol, curr_joints))
                if delta < min_delta:
                    min_delta = delta
                    best_solution = sol
            return best_solution
        except Exception:
            return None

    def move_joint_to(self, target_xyz):
        """关节运动：用于大跨度移动，规避笛卡尔空间下的奇异点。

        若 IK 失败则降级为直线运动作为兜底。
        """
        joints = self.get_optimal_joints(target_xyz)
        if joints:
            self.robot.getMotionControl().moveJoint(joints, JOINT_ACC, JOINT_VEL, 0, 0)
            self.wait_arrival(target_xyz)
        else:
            self.move_line_to(*target_xyz)

    def move_line_to(self, x, y, z, vel=LINE_VEL, acc=LINE_ACC):
        """直线运动：用于精准下探和拔起的短距离运动。"""
        if not self.robot:
            return
        target_pose = self._build_pose(x, y, z)
        print(f"直线运动 -> X:{x:.1f}, Y:{y:.1f}, Z:{z:.1f} (vel={vel})")
        self.robot.getMotionControl().moveLine(target_pose, acc, vel, 0, 0)
        self.wait_arrival([x, y, z])

    def wait_arrival(self, target_xyz, timeout=ARRIVAL_TIMEOUT_S):
        """轮询 TCP 位姿，基于欧氏距离判定到位。

        机械臂运动指令是异步的，必须通过位置反馈同步软件状态与物理状态，
        否则会出现 "运动未结束即触发下一指令" 的严重问题。
        """
        start = time.time()
        print(f"等待到位，目标 Z={target_xyz[2]}")
        while time.time() - start < timeout:
            try:
                curr_pose = self.robot.getRobotState().getTcpPose()
                curr_xyz = [curr_pose[i] * 1000 for i in range(3)]
                dist = sum((a - b) ** 2 for a, b in zip(curr_xyz, target_xyz)) ** 0.5
                if dist < ARRIVAL_TOLERANCE_MM:
                    print(f"到位，偏差 {dist:.2f}mm")
                    break
            except Exception:
                pass
            time.sleep(ARRIVAL_POLL_INTERVAL)
        time.sleep(SETTLE_DELAY)

    # ---------- IO 控制 ----------
    def gripper_control(self, action):
        """夹爪开合，走 Modbus 输出信号。"""
        value = GRIPPER_OPEN_VAL if action == "OPEN" else GRIPPER_CLOSE_VAL
        print(f"夹爪 -> {action}")
        try:
            reg = self.rpc.getRegisterControl()
            reg.modbusSetOutputSignal(config.GRIPPER_ADDR, int(value))
        except Exception:
            pass
        time.sleep(GRIPPER_ACTION_DELAY)

    def conveyor_control(self, status):
        """传送带 IO 控制。

        调试阶段对 AUBO SDK 的 IO 接口进行多路径探测，
        最终确认 setStandardDigitalOutput 可用，其余路径作为兼容兜底保留。
        """
        is_on = bool(status)
        val = 1 if is_on else 0
        print(f"传送带 -> {'启动' if is_on else '停止'}")
        if not self.robot:
            return

        # 优先路径
        try:
            self.robot.getIoControl().setStandardDigitalOutput(0, is_on)
            return
        except Exception:
            pass

        # 兼容路径（不同 SDK 版本接口差异）
        try:
            self.robot.getIoControl().setConfigurableDigitalOutput(0, is_on)
            return
        except Exception:
            pass
        try:
            self.robot.getBoardIO().setBoardUserDO(0, val)
            return
        except Exception:
            pass
        try:
            self.rpc.getRegisterControl().setBoolOutput("U_DO_00", is_on)
            return
        except Exception:
            pass

        print("warning: 传送带 IO 指令未命中任何已知接口")

    def stop(self):
        if self.robot:
            self.robot.getMotionControl().stopMove(True, True)


class StageTimer:
    """简易阶段计时器，用于分析 Cycle Time 组成。"""

    def __init__(self):
        self.cycle_start = None

    def start_cycle(self):
        self.cycle_start = time.time()

    def stage(self, name):
        return _StageContext(name)

    def end_cycle(self):
        if self.cycle_start is None:
            return
        print(f"[Cycle] 总耗时 {time.time() - self.cycle_start:.2f}s")


class _StageContext:
    def __init__(self, name):
        self.name = name
        self.t0 = None

    def __enter__(self):
        self.t0 = time.time()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        print(f"[Stage] {self.name} 耗时 {time.time() - self.t0:.2f}s")
        return False


class AutomationSystem:
    """主流程：视觉触发 -> 抓取 -> 传送带接驳 -> 阵列装配 -> 回航。"""

    def __init__(self):
        self.arm = AuboController()
        self.vision = VisionProcessor()
        self.timer = StageTimer()
        self.assembly_count = 0

    # ---------- 子流程 ----------
    def _grasp_from_workbench(self, tx, ty):
        """从物料台精准抓取。"""
        self.arm.move_joint_to([tx, ty, config.SAFE_Z_HEIGHT])
        self.arm.move_line_to(tx, ty, config.GRASP_Z_WORKBENCH)
        time.sleep(1.0)
        self.arm.gripper_control("CLOSE")
        time.sleep(1.5)
        self.arm.move_line_to(tx, ty, config.SAFE_Z_HEIGHT)

    def _place_on_conveyor_and_run(self):
        """放料到传送带左端并启动传送。"""
        x, y = config.PLACE_X_CONVEYOR, config.PLACE_Y_CONVEYOR
        self.arm.move_joint_to([x, y, config.SAFE_Z_HEIGHT])
        self.arm.move_line_to(x, y, config.PLACE_Z_CONVEYOR)
        time.sleep(1.0)
        self.arm.gripper_control("OPEN")
        self.arm.move_line_to(x, y, config.SAFE_Z_HEIGHT)

        self.arm.conveyor_control(1)
        time.sleep(3.0)
        self.arm.conveyor_control(0)

    def _blind_pick_from_conveyor(self):
        """传送带末端定点盲抓（位置固定，无视觉）。"""
        x, y = config.PLACE_X_CONVEYOR, config.PICK_Y_CONVEYOR
        self.arm.move_joint_to([x, y, config.SAFE_Z_HEIGHT])
        self.arm.move_line_to(x, y, config.PLACE_Z_CONVEYOR)
        time.sleep(1.0)
        self.arm.gripper_control("CLOSE")
        time.sleep(0.5)
        self.arm.move_line_to(x, y, config.SAFE_Z_HEIGHT)

    def _assemble_to_peg(self):
        """3D 阵列装配：基于已装配计数动态计算目标铁杆与堆叠高度。"""
        total_pegs = len(config.PEG_POSITIONS)
        peg_index = self.assembly_count % total_pegs
        stack_level = self.assembly_count // total_pegs
        target_peg = config.PEG_POSITIONS[peg_index]
        drop_z = target_peg[2] + stack_level * STACK_HEIGHT_PER_LAYER

        print(f"装配 #{self.assembly_count + 1} -> 铁杆{peg_index + 1} 层{stack_level} Z={drop_z:.1f}")

        self.arm.move_joint_to([target_peg[0], target_peg[1], config.SAFE_Z_HEIGHT])
        self.arm.move_line_to(target_peg[0], target_peg[1], drop_z)
        time.sleep(0.5)
        self.arm.gripper_control("OPEN")
        self.arm.move_line_to(target_peg[0], target_peg[1], config.SAFE_Z_HEIGHT)

        self.assembly_count += 1

    def _return_to_scan(self):
        """大跨度回航：经过渡点回到拍照点，避免奇异点。"""
        via_xyz = [config.PLACE_X_CONVEYOR, config.PICK_Y_CONVEYOR, config.SAFE_Z_HEIGHT]
        scan_xyz = [config.SCAN_X, config.SCAN_Y, config.SCAN_Z]
        self.arm.move_joint_to(via_xyz)
        self.arm.move_joint_to(scan_xyz)

    def _drain_socket(self, conn):
        """清空连接缓冲区，丢弃运动期间累积的过期视觉数据。"""
        conn.setblocking(False)
        try:
            while conn.recv(1024):
                pass
        except Exception:
            pass
        conn.setblocking(True)

    # ---------- 单轮 Cycle ----------
    def _run_one_cycle(self, conn, tx, ty):
        self.timer.start_cycle()

        with self.timer.stage("抓取物料台"):
            self._grasp_from_workbench(tx, ty)

        with self.timer.stage("传送带接驳"):
            self._place_on_conveyor_and_run()

        with self.timer.stage("传送带末端盲抓"):
            self._blind_pick_from_conveyor()

        with self.timer.stage("3D阵列装配"):
            self._assemble_to_peg()

        with self.timer.stage("回航"):
            self._return_to_scan()
            self._drain_socket(conn)

        self.timer.end_cycle()

    # ---------- 主循环 ----------
    def run(self):
        if not self.arm.connect():
            sys.exit(1)

        self.arm.gripper_control("OPEN")
        self.arm.move_line_to(config.SCAN_X, config.SCAN_Y, config.SCAN_Z)

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((config.VISION_HOST, config.VISION_PORT))
        server.listen(1)

        try:
            while True:
                print("等待 VisionMaster 连接...")
                conn, addr = server.accept()
                print(f"VisionMaster 已连接 ({addr})")

                try:
                    while True:
                        data = conn.recv(1024).decode('utf-8', errors='ignore')
                        if not data:
                            print("VisionMaster 断开")
                            break

                        tx, ty = self.vision.parse_and_transform(data)
                        if tx is None:
                            continue

                        self._run_one_cycle(conn, tx, ty)
                except ConnectionResetError:
                    pass
                finally:
                    conn.close()

        except KeyboardInterrupt:
            print("急停")
            self.arm.stop()
            self.arm.conveyor_control(0)
        finally:
            server.close()
            self.arm.disconnect()


if __name__ == "__main__":
    AutomationSystem().run()