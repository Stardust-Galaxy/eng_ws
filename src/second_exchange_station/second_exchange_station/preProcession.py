import cv2
import numpy as np

class ImagePreprocessor:
    def __init__(self,if_red):
        image_shape = (200, 300, 3)  # 图像尺寸 (height, width, channels)
        self.image = np.zeros(image_shape)
        self.r = np.zeros(image_shape)
        self.b = np.zeros(image_shape)
        self.g = np.zeros(image_shape)
        self.if_red = if_red

    def set_image(self,image):
        self.image = image
        self.r = image[:, :, 2]
        self.g = image[:, :, 1]
        self.b = image[:, :, 0]
        return image

    def binarize(self,if_red , threshold=110):
        """
        将灰度图二值化
        """
        if if_red:
            _, binary_image = cv2.threshold(self.r, threshold, 255, cv2.THRESH_BINARY)
            
        else:
            _, binary_image = cv2.threshold(self.b, threshold, 255, cv2.THRESH_BINARY)

        kernel = np.ones((5, 5), np.uint8)  # 这是一个5x5的正方形内核，你可以根据需要调整大小
        
        # 腐蚀膨胀操作
        eroded_image = cv2.erode(binary_image, kernel, iterations=1)
        dilated_image = cv2.dilate(eroded_image, kernel, iterations=1)

        cv2.imshow("binary_image", dilated_image)
        return dilated_image

    def preProcession_main(self):
        # 二值化图像
        binary_image = self.binarize(self.if_red)
       
        return binary_image

