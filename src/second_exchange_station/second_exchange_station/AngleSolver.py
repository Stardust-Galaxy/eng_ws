import numpy as np
import cv2

class AngleSolver:
    def __init__(self, K, D):
        self.K = K  # 相机内参数矩阵
        self.D = D

    def set_points(self, points):
        self.points = points

    def center_distance(self,point_set1, point_set2):
        # 计算第一个点集的中心点
        center1 = np.mean(point_set1, axis=0)
        # 计算第二个点集的中心点
        center2 = np.mean(point_set2, axis=0)
        # 计算两个中心点之间的距离
        distance = np.linalg.norm(center2 - center1)
        return distance


    def get_mode(self):
        if len(self.points) < 3 and cv2.contourArea(self.points[-1]) > 35:
            tmp = [self.points[0]]
            #print("tmp ", len(tmp))
            return True, tmp
        elif len(self.points) >= 3:
            tmp = [self.points[-1], self.points[-2], self.points[-3]]
            #print("tmp ", len(tmp))
            return False, tmp
        else:
            #print(str(cv2.contourArea(self.points[-1])) + " !!!area")
            tmp = np.array([])
            #print("tmp ", len(tmp))
            return False, tmp

    def find_extreme_points_left(self, points):
        if not points:
            return None
        # Convert points list to NumPy array
        points_array = np.array(points[0])
        # Extract coordinates of all points
        x_values = points_array[:, 0, 0]  # x coordinates of all points
        y_values = points_array[:, 0, 1]  # y coordinates of all points
        # Find top-left point
        right_top_indices = np.where((x_values < np.mean(x_values)) & (y_values < np.mean(y_values)))
        right_top_points = points_array[right_top_indices]
        if len(right_top_points) == 0:
            # Set a default farthest point to continue code execution
            default_point = np.array([[[0, 0]]])
            return default_point
        # Calculate center coordinates
        center_x = np.mean(x_values)
        center_y = np.mean(y_values)
        # Calculate distances to center
        distances_to_center = np.sqrt(
            (right_top_points[:, 0, 0] - center_x) ** 2 + (right_top_points[:, 0, 1] - center_y) ** 2)
        # Find index of farthest point
        farthest_index = np.argmax(distances_to_center)
        # Get farthest point
        farthest_point = right_top_points[farthest_index]
        # Find index of rightmost point
        rightmost_index = np.argmax(x_values)
        # Find index of bottommost point
        bottommost_index = np.argmax(y_values)
        # Get rightmost and bottommost points
        bottom_right_point = points_array[bottommost_index]
        rightmost_point = points_array[rightmost_index]
        final_points = np.array([bottom_right_point, farthest_point, rightmost_point])

        mid_point=(bottom_right_point+rightmost_point)/2
        top_left_point = None
        min_k = 100
        for point in points[0]:
            x, y = point[0]  # 提取点的坐标
            k = ((y-farthest_point[0][1])/(x-farthest_point[0][0]))/((mid_point[0][1]-y)/(mid_point[0][0]-x))
            if abs(k-1)<min_k:
                min_k = abs(k-1)
                top_left_point = point



        return final_points,top_left_point

    def calculate_square_midpoints(self, points):
        point1 = np.mean(points[0], axis=0)
        point2 = np.mean(points[1], axis=0)
        point3 = np.mean(points[2], axis=0)
        return point1, point2,point3

    def find_corner_points(self, points):
        # Convert points list to NumPy array
        points_array = np.array(points)
        # 下面被我改了 不知道points啥情况 猜测这样才是x，y
        center_x = np.mean(points_array[:,:, 0])
        center_y = np.mean(points_array[:,:, 1])
        corner_points = []
        for point in points_array:
            flattened_list = [item for sublist in point for item in sublist]
        # Check if the x coordinate is greater than or equal to the center_x
            x, y = flattened_list
            if x >= center_x:
            # Check if the y coordinate is greater than or equal to the center_y
                if y >= center_y:
                # If both conditions are met, it's a corner point
                    corner_points.append(flattened_list)
        # Calculate distances to center
        distances_to_center = np.sqrt((points_array[:, :,0] - center_x) ** 2 + (points_array[:, :,1] - center_y) ** 2)
        # Find farthest point
        farthest_index = np.argmax(distances_to_center)
        farthest_point = points_array[farthest_index]
        return farthest_point
    
    def sort_points_clockwise(self, point1, point2, point3):
        if point2[0][0]>point3[0][0]:
            sorted_points = np.array([point3,point1,point2])
        else:
            sorted_points = np.array([ point2, point1,point3])
        return sorted_points

    def find_extreme_points_right(self, points):
        if not points:
            return None,None
        # Calculate midpoint1 and midpoint2
        midpoint1, midpoint2,midpoint3 = self.calculate_square_midpoints(points[0:3])
        # Find corner points
        # corner_points = self.find_corner_points(points[-1])
        # Sort points clockwise
        # mid = (midpoint1+midpoint2+corner_points)/3
        """
        2----1
             |
             3
        """
        
        sorted_points = self.sort_points_clockwise(midpoint3, midpoint2, midpoint1)
        # 初始化右上角点集
        upper_right_points = []
        # 提取给定点的坐标

        given_x, given_y = midpoint3[0]
        
        # 遍历所有点，找出在给定点右上方的点
        for point in points[-1]:
            x, y = point[0]  # 提取点的坐标
            if x > given_x and y < given_y:
                upper_right_points.append((x, y))
        
        # 如果没有找到右上角的点，返回空值
        if not upper_right_points:
            return None,None
        
        # 计算右上角点集中距离给定点最远的点
        farthest_point = None
        max_distance = -1
        
        for point in upper_right_points:
            x, y = point
            distance = np.sqrt((x - given_x)**2 + (y - given_y)**2)
            if distance > max_distance:
                max_distance = distance
                farthest_point = point
        
        return sorted_points,farthest_point
    
    def get_corner(self,mode,points):
        if mode:
            corner,forth_point = self.find_extreme_points_left(points)
            #print("left!")
        else:
            corner,forth_point = self.find_extreme_points_right(points)
            #print("right!")
        return corner,forth_point



    def find_fourth_point(self, image_points,forth_point):
        
        if len(image_points) < 3:
            #print("不足三个点的坐标！")
            return None
        if self.center_distance(image_points[0],image_points[1])>75 or \
        self.center_distance(image_points[1],image_points[2])>75 or \
        self.center_distance(image_points[0],image_points[2])>75:
            #print("太近了！笨蛋，都你害的！！")
            return None
        if self.center_distance(image_points[0],image_points[1])*self.center_distance(image_points[1],image_points[2])<150:
            #print("太小了！笨蛋，都你害的！！")
            return None
        
        return forth_point

    def solve_p3p(self, mode, image_points_,forth_point):
        if mode:
            object_points = np.array([[-19.5, 25, 0],
                                      [-25, -25, 0],
                                      [25, -19.5, 0],
                                      [-14, -14, 0]])
        else:
            object_points = np.array([[-19.5, -19.5, 0],
                                      [14, -14, 0],
                                      [19.5, 19.5, 0],
                                      [25, -25, 0]])
        image_points = image_points_.reshape(-1, 2)

        # Find the coordinates of the fourth point
        fourth_point = self.find_fourth_point(image_points,forth_point)
        if np.array_equal(fourth_point, np.array([0, 0])) or fourth_point is None:
            rvecs = np.array([[0], [0], [0]])  # Rotation vector
            tvecs = np.array([[0], [0], [0]])  # Translation vector
            return rvecs, tvecs
        # Add the coordinates of the fourth point to image_points
        # print("fourth point")
        # print(fourth_point)
        image_points = np.vstack([image_points, fourth_point])
        object_points = object_points.astype(np.float32)
        image_points = image_points.astype(np.float32)

        _, rvecs, tvecs, _ = cv2.solvePnPRansac(object_points, image_points, self.K, self.D)
        return rvecs, tvecs

    def rotation_matrix_to_euler_angles(self,rvec):
        rvec = np.float32(rvec)
        rmat, _ = cv2.Rodrigues(rvec)
        sy = np.sqrt(rmat[0, 0]**2 + rmat[1, 0]**2)
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(rmat[2, 1], rmat[2, 2])  # Roll
            y = np.arctan2(-rmat[2, 0], sy)          # Pitch
            z = np.arctan2(rmat[1, 0], rmat[0, 0])   # Yaw
        else:
            x = np.arctan2(-rmat[1, 2], rmat[1, 1])
            y = np.arctan2(-rmat[2, 0], sy)
            z = 0
        return np.array([x, y, z])

    def angle_solver_main(self, points, cv_image):
        self.set_points(points)
        mode, arr = self.get_mode()
        if len(arr) == 0:
            #print("len(arr)==0!")
            rvecs = np.array([[0], [0], [0]])  # Rotation vector
            tvecs = np.array([[0], [0], [0]])  # Translation vector
            return rvecs, tvecs,mode
        object_points,forth_point = self.get_corner(mode, arr)

        if object_points is None or forth_point is None:
            #print("len(object_points)==0!")
            rvecs = np.array([[0], [0], [0]])  # 旋转向量
            tvecs = np.array([[0], [0], [0]])  # 平移向量
            return rvecs, tvecs,mode
# 绘制每个点
        if(object_points.size!=0):
            i=1
            for point in object_points:
                point_ = (int(point[0][0]), int(point[0][1]))  # 将坐标转换为整数类型
                if(i==1):
                    cv2.circle(cv_image, point_, 5, (255, 0, 0), -1)  # 绘制红色点，半径为5
                elif(i==2):
                    cv2.circle(cv_image, point_, 5, (0, 255, 0), -1)
                elif(i==3):
                    cv2.circle(cv_image, point_, 5, (0, 0, 255), -1)
                i+=1
            if mode:
                point_4 = (int(forth_point[0][0]), int(forth_point[0][1])) 
            else:
                point_4 = (int(forth_point[0]), int(forth_point[1])) 
            cv2.circle(cv_image, point_4, 5, (0, 255, 255), -1)
            #cv2.imshow("image", cv_image)
        rvecs, tvecs = self.solve_p3p(mode, object_points,forth_point)
        #print("旋转向量:", rvecs)
        #print("平移向量:", tvecs)
        rvec = np.float32(rvecs)
        # Convert rotation vector to rotation matrix
        rotationMatrix, _ = cv2.Rodrigues(rvec)

        # Decompose rotation matrix into Euler angles
        eulerAngles = cv2.RQDecomp3x3(rotationMatrix)[0]
        #print("EulerAngle(pitch, yaw, roll):", eulerAngles)
        # 如果需要将欧拉角从弧度转换为度数
        #euler_angles_deg = np.degrees(rvecs)
        #roll, pitch, yaw = euler_angles_deg
        #print("Roll:", roll, "degrees")
        #print("Pitch:", pitch, "degrees")
        #print("Yaw:", yaw, "degrees")
        return rvecs, tvecs ,mode