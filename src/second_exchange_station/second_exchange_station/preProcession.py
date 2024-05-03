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
        return image

    def split_channels(self):
        """
        将图片分成三个通道（BGR）
        """
        b, g, r =cv2.split(self.image)
        self.r = r
        self.g = g
        self.b = b
        return b, g, r

    def binarize(self,if_red , threshold=80):
        """
        将灰度图二值化
        """
        _, binary_image_g = cv2.threshold(self.g, threshold, 255, cv2.THRESH_BINARY)
        if if_red:
            _, binary_image = cv2.threshold(self.g, threshold, 255, cv2.THRESH_BINARY)
            
        else:
            _, binary_image = cv2.threshold(self.g, threshold, 255, cv2.THRESH_BINARY)

        kernel = np.ones((5, 5), np.uint8)  # 这是一个5x5的正方形内核，你可以根据需要调整大小

        # 腐蚀操作
        eroded_image = cv2.erode(binary_image, kernel, iterations=1)

        # 膨胀操作
        dilated_image = cv2.dilate(eroded_image, kernel, iterations=1)

        return dilated_image

    def preProcession_main(self):
        # 分通道
        b_channel, g_channel, r_channel = self.split_channels()
        """
        # 显示各通道
        cv2.imshow("Blue Channel", b_channel)
        cv2.imshow("Green Channel", g_channel)
        cv2.imshow("Red Channel", r_channel)
        """
        # 二值化图像
        binary_image = self.binarize(self.if_red)
        

        # cv2.waitKey(30)
        
        # cv2.destroyAllWindows()
        return binary_image

