import math
import csv
import os
from datetime import datetime
import numpy as np
import cv2




# ================= N点标定（仿射变换）引擎 =================
class NPointTransform:
    def __init__(self, matrix_path='n_point_matrix.npy'):
        """
        初始化 N 点标定引擎，加载算好的矩阵
        """
        try:
            self.matrix = np.load(matrix_path)
        except FileNotFoundError:
            self.matrix = None
            # print("没找到 N 点标定矩阵，请先运行标定脚本生成 n_point_matrix.npy")

    def train_matrix(self, pixel_points, robot_points, save_path='n_point_matrix.npy'):
        """
        传入 N 个像素点和对应的 N 个机械臂坐标，算出矩阵并保存
        pixel_points: [(u1,v1), (u2,v2)...]
        robot_points: [(x1,y1), (x2,y2)...]
        """
        pts_src = np.array(pixel_points).astype(np.float32)
        pts_dst = np.array(robot_points).astype(np.float32)

        # 计算 2x3 的仿射变换矩阵
        matrix, inliers = cv2.estimateAffine2D(pts_src, pts_dst)

        if matrix is not None:
            self.matrix = matrix
            np.save(save_path, matrix)
            print(f"✅ N点标定矩阵计算成功！已保存至 {save_path}")
            print("矩阵内容:\n", matrix)
        else:
            print("❌ 矩阵计算失败，请检查输入的坐标点是否共线或格式错误！")

        return matrix

    def pixel_to_robot_base(self, u, v):
        """
        使用算好的矩阵，将单个像素坐标 (u, v) 直接转为机械臂基座物理坐标 (X, Y)
        """
        if self.matrix is None:
            print("❌ 矩阵未加载，无法转换坐标！")
            return None, None

        # OpenCV 的 transform 函数需要特定的三维数组形状: (1, 1, 2)
        pixel_pos = np.array([[[u, v]]], dtype=np.float32)
        robot_pos = cv2.transform(pixel_pos, self.matrix)

        # 返回解算出的 X, Y 坐标 (Z轴由 config.GRASP_Z_HEIGHT 决定)
        return robot_pos[0][0][0], robot_pos[0][0][1]