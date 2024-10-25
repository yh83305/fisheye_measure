import cv2
import numpy as np

# 读取图片
image = cv2.imread('1.png')
if image is None:
    print("无法读取图片，请检查文件路径")
    exit()

# 转换为灰度图像
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 使用高斯模糊来减少噪声
gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)

# 检测霍夫圆
circles = cv2.HoughCircles(
    gray_blurred,
    cv2.HOUGH_GRADIENT,
    dp=1,
    minDist=30,
    param1=50,
    param2=30,
    minRadius=10,
    maxRadius=0
)

# 如果检测到圆
if circles is not None:
    circles = np.uint16(np.around(circles))
    for i in circles[0, :]:
        # 绘制外圆
        cv2.circle(image, (i[0], i[1]), i[2], (0, 255, 0), 2)
        # 绘制圆心
        cv2.circle(image, (i[0], i[1]), 2, (0, 0, 255), 3)

# 显示结果
cv2.imshow("Detected Circles", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
