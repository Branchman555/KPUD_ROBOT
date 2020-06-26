import cv2

img = cv2.imread("bg_0.jpg")
count = 100
while count > 0:
	cv2.imshow("WIN", img)
	cv2.waitKey(30)
	count -= 1

print("end")
