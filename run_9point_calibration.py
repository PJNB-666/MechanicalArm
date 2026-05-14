# run_9point_calibration.py
import numpy as np
import cv2


def main():
    # 1. 填入采集的 9 个像素坐标 (匹配点X, 匹配点Y)
    pixel_points = [
        (1206.157, 835.443), (1513.98, 888.532), (1823.413, 941.379),
        (1153.686, 1144.346), (1462.324, 1196.695), (1770.011, 1250.091),
        (1102.733, 1451.743), (1410.225, 1503.874), (1717.721, 1555.635)
    ]

    # 2. 填入采集的 9 个机械臂物理坐标 (Base X, Base Y)
    robot_points = [
        (32.28, -370.4), (32.28, -408.26), (32.28, -447.12),
        (-6.39, -372.04), (-7.9, -410.18), (-7.91, -447.12),
        (-45.41, -368.5), (-45.42, -407.81), (-45.42, -446.13)
    ]

    pts_src = np.array(pixel_points).astype(np.float32)
    pts_dst = np.array(robot_points).astype(np.float32)

    # 计算仿射变换矩阵
    matrix, inliers = cv2.estimateAffine2D(pts_src, pts_dst)

    if matrix is not None:
        np.save('n_point_matrix.npy', matrix)
        print("✅ 矩阵已生成并保存为 n_point_matrix.npy")
        print("矩阵内容:\n", matrix)

        # 验证一下第一个点
        test_pixel = np.array([[[1206.157, 835.443]]], dtype=np.float32)
        res = cv2.transform(test_pixel, matrix)

        calc_x, calc_y = res[0][0][0], res[0][0][1]
        print(f"\n验证点1: 像素(1206.157, 835.443) -> 机械臂 [{calc_x:.3f}, {calc_y:.3f}]")
        print(f"预期结果: [32.280, -370.400]")

        # 计算误差
        err_x = abs(calc_x - 32.28)
        err_y = abs(calc_y - (-370.4))
        print(f"📐 坐标系换算误差 -> X: {err_x:.3f} mm, Y: {err_y:.3f} mm")
    else:
        print("❌ 计算失败，请检查数据。")


if __name__ == "__main__":
    main()