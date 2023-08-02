import cv2

img1 = cv2.imread("./img/frame1.jpg")
img2 = cv2.imread("./img/frame2.jpg")

img1_Gauss = cv2.GaussianBlur(img1, (3, 3), 1)
img2_Gauss = cv2.GaussianBlur(img2, (3, 3), 1)
img1_blur = cv2.blur(img1_Gauss, (5, 5))
img2_blur = cv2.blur(img2_Gauss, (5, 5))

out = cv2.subtract(img2_blur,img1_blur)
cv2.imshow("img2_blur", img2_blur)
cv2.imshow("img1_blur", img1_blur)
cv2.imshow("out", out)
cv2.imshow("out-0", out[:,:,0])
cv2.imshow("out-1", out[:,:,1])
cv2.imshow("out-2", out[:,:,2])
cv2.waitKey(0)


