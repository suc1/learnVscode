#importing some useful packages
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2


# 感兴趣区域选择: 截取
def region_of_interest(img, vertices):
    #定义一个和输入图像同样大小的全黑图像mask，这个mask也称掩膜
    #掩膜的介绍，可参考：https://www.cnblogs.com/skyfsm/p/6894685.html
    mask = np.zeros_like(img)   
 
    #根据输入图像的通道数，忽略的像素点是多通道的白色，还是单通道的白色
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count      # (255,) * 3 = (255, 255, 255)
    else:
        ignore_mask_color = 255

    #[vertices]中的点组成了多边形，将在多边形内的mask像素点保留，
    cv2.fillPoly(mask, [vertices], ignore_mask_color)
 
    #与mask做"与"操作，即仅留下多边形部分的图像
    masked_image = cv2.bitwise_and(img, mask)

    return masked_image

def draw_lines(img, lines, color=[255, 0, 0], thickness=2):
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness) # 将线段绘制在img上


# 数据后处理: 1. 计算左右车道线的直线方程  2. 计算左右车道线的上下边界
def draw_lines2(img, lines, color=[255, 0, 0], thickness=2):
    left_lines_x = []
    left_lines_y = []
    right_lines_x = []
    right_lines_y = []
    line_y_max = 0
    line_y_min = 999
    
    for line in lines:
        for x1,y1,x2,y2 in line:
            if y1 > line_y_max:
                line_y_max = y1
            if y2 > line_y_max:
                line_y_max = y2
            if y1 < line_y_min:
                line_y_min = y1
            if y2 < line_y_min:
                line_y_min = y2
            k = (y2 - y1)/(x2 - x1)
            if k < -0.3:
                left_lines_x.append(x1)
                left_lines_y.append(y1)
                left_lines_x.append(x2)
                left_lines_y.append(y2)
            elif k > 0.3:
                right_lines_x.append(x1)
                right_lines_y.append(y1)
                right_lines_x.append(x2)
                right_lines_y.append(y2)
    #最小二乘直线拟合
    left_line_k, left_line_b = np.polyfit(left_lines_x, left_lines_y, 1)
    right_line_k, right_line_b = np.polyfit(right_lines_x, right_lines_y, 1)
 
    #根据直线方程和最大、最小的y值反算对应的x
    cv2.line(img,
             (int((line_y_max - left_line_b)/left_line_k), line_y_max),
             (int((line_y_min - left_line_b)/left_line_k), line_y_min),
             color, thickness)
    cv2.line(img,
             (int((line_y_max - right_line_b)/right_line_k), line_y_max),
             (int((line_y_min - right_line_b)/right_line_k), line_y_min),
             color, thickness)


if __name__ == "__main__":
    img = cv2.imread('whiteCarLaneSwitch.jpg')      #BGR()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 由于使用cv2.imread()读到的img的数据排列为BGR，因此这里的参数为BGR2GRAY

    # 边缘提取, 有Canny算法和Sobel算法
    low_threshold = 40
    high_threshold = 150
    canny_image = cv2.Canny(gray, low_threshold, high_threshold)

    #图像像素行数 rows = canny_image.shape[0]  540行
    #图像像素列数 cols = canny_image.shape[1]  960列
    left_bottom = [0, canny_image.shape[0]]
    right_bottom = [canny_image.shape[1], canny_image.shape[0]]
    apex = [canny_image.shape[1]/2, 310]
    vertices = np.array([ left_bottom, right_bottom, apex ], np.int32)
    roi_image = region_of_interest(canny_image, vertices)

    # 使用霍夫变换来提取图像中的直线（段）
    rho = 2                 # distance resolution in pixels of the Hough grid
    theta = np.pi/180       # angular resolution in radians of the Hough grid
    threshold = 15          # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 40    #minimum number of pixels making up a line
    max_line_gap = 20       # maximum gap in pixels between connectable line segments
    # Hough Transform 检测线段，线段两个端点的坐标存在lines中
    lines = cv2.HoughLinesP(roi_image, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)

    line_image = np.copy(img) # 复制一份原图，将线段绘制在这幅图上
    draw_lines(line_image, lines, [255, 0, 0], 6)
    cv2.imshow('image', line_image)
    cv2.waitKey(0)

    # 数据后处理
