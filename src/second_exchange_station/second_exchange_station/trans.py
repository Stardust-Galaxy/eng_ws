import numpy as np
import math

class Quadrilateral:
    def __init__(self):
        pass


    def init(self, x0, y0, z0, a,yaw_angle, pitch_angle, roll_angle):
        self.x0 = x0
        self.y0 = y0
        self.z0 = z0
        self.a = a
        self.yaw_angle = yaw_angle
        self.pitch_angle = pitch_angle
        self.roll_angle = roll_angle
    
    def rotate_point_around_origin(self, point):
        # 将��度转换为弧度
        yaw = self.yaw_angle
        pitch = self.pitch_angle
        roll = self.roll_angle
        
        # 构建旋转矩阵
        rotation_matrix_yaw = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                                         [math.sin(yaw), math.cos(yaw), 0],
                                         [0, 0, 1]])
        
        rotation_matrix_pitch = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                                           [0, 1, 0],
                                           [-math.sin(pitch), 0, math.cos(pitch)]])
        
        rotation_matrix_roll = np.array([[1, 0, 0],
                                          [0, math.cos(roll), -math.sin(roll)],
                                          [0, math.sin(roll), math.cos(roll)]])
        
        # 将点与旋转矩阵相乘
        rotated_point = np.dot(rotation_matrix_yaw, point)
        rotated_point = np.dot(rotation_matrix_pitch, rotated_point)
        rotated_point = np.dot(rotation_matrix_roll, rotated_point)
        
        return rotated_point
    
    def calculate_center_point_left(self):
        # 定义四个顶点的坐标（假设起始点为左上角）
        points = [(self.x0, self.y0, self.z0),
                  (self.x0 + self.a, self.y0, self.z0),
                  (self.x0 + self.a, self.y0 - self.a, self.z0),
                  (self.x0, self.y0 - self.a, self.z0)]
        
        # 将四个顶点绕原点旋转
        rotated_points = [self.rotate_point_around_origin(np.array(point)) for point in points]
        
        # 计算顶点坐标之和并取平均得到中心点坐标
        center_point = np.mean(rotated_points, axis=0)
        
        return center_point
    
    def calculate_center_point_right(self):
        # 定义四个顶点的坐标（假设起始点为右上角）
        points = [(self.x0, self.y0, self.z0),
                (self.x0 - self.a, self.y0, self.z0),
                (self.x0 - self.a, self.y0 - self.a, self.z0),
                (self.x0, self.y0 - self.a, self.z0)]
            
        # 将四个顶点绕原点旋转
        rotated_points = [self.rotate_point_around_origin(np.array(point)) for point in points]
            
        # 计算顶点坐标之和并取平均得到中心点坐标
        center_point = np.mean(rotated_points, axis=0)
            
        return center_point
