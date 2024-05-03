import cv2
import numpy as np

class TriangleDetector:
    def __init__(self):
        cv2.namedWindow("Detected Triangles", cv2.WINDOW_NORMAL)
        pass

    def detect_edges(self, image):
        """
        边缘检测
        """
        edges = cv2.Canny(image, 50, 150)
        return edges


    def detect_triangles(self,image, edges, min_contour_area=30, max_aspect_ratio=2, max_angle=40):
        """
        检测边缘点的最小外接直角三角形
        """
        # 寻找轮廓
        contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

        # 存储直角三角形
        triangles = []
        points = []

        # 遍历轮廓
        for contour in contours:
            # 计算轮廓的面积
            contour_area = cv2.contourArea(contour)

            # 如果轮廓面积小于阈值，则跳过该轮廓
            if contour_area < min_contour_area:
                print("area!")
                continue

            rect = cv2.minAreaRect(contour)
            """ corner_points = []
            for point in contour:
                flattened_list = [item for sublist in point for item in sublist]
                # Check if the x coordinate is greater than or equal to the center_x
                x, y = flattened_list
                corner_points.append(flattened_list)
            
            corner_points_umat = cv2.UMat(corner_points)
            print("tri ",corner_points_umat)
            triangle = cv2.minEnclosingTriangle(corner_points_umat)
            for i in range(3):
                cv2.line(image, tuple(triangle[i][0]), tuple(triangle[(i + 1) % 3][0]), (0, 255, 0), 2)

            # 显示结果
            cv2.imshow('Min Enclosing Triangle', colimage)
            cv2.waitKey(30)
"""
            # 提取矩形的四个顶点
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # 计算矩形的对角线长度
            d1 = np.linalg.norm(box[0] - box[1])
            d2 = np.linalg.norm(box[1] - box[2])

            # 计算长宽比和倾斜角度
            aspect_ratio = max(d1, d2) / min(d1, d2)
            angle = rect[2]

            # 检查长宽比和倾斜角度是否满足条件
            if aspect_ratio < max_aspect_ratio and ((abs(angle) < max_angle)or(abs(angle) > 90 - max_angle)):
                triangles.append(box)
                points.append(contour)

            if aspect_ratio > max_aspect_ratio :
                print(str(aspect_ratio)+"  ratio!")
            if not((abs(angle) < max_angle)or(abs(angle) > 90 - max_angle)):
                print(str(angle)+"angle!")

        return triangles,points

    def draw_triangles(self, image, triangles):
        """
        在图像上绘制检测到的三角形
        """
        for triangle in triangles:
            cv2.drawContours(image, [triangle], -1, (0, 255, 0), 2)

    def sort_points_by_triangles(self,points, triangles):
        # 使用矩形的面积排序三角形的索引
        sorted_indices = sorted(range(len(triangles)), key=lambda i: cv2.contourArea(triangles[i]), reverse=True)

        # 根据排序后的三角形索引重新排序点集
        sorted_points = [points[i] for i in sorted_indices]

        return sorted_points

    def detector_main(self,image,origin):
        # 边缘检测
        edges = self.detect_edges(image)

        # 检测直角三角形
        triangles,points = self.detect_triangles(image,edges)
        # 绘制
        self.draw_triangles(origin, triangles)
        # 面积越大的矩形排在前面，面积越小的排在后面
        sorted_points = self.sort_points_by_triangles(points, triangles)
        # 在图像上显示结果
        #cv2.imshow("Detected Triangles", origin)
        #cv2.waitKey(30)
        # cv2.destroyAllWindows()
        return sorted_points
