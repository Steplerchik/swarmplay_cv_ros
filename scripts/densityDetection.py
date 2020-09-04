import cv2
import pytesseract
import numpy as np

videoCaptureObject = cv2.VideoCapture(0)
kernel = np.ones((5, 5), np.uint8)
bordersize = 20
desk_img = [None] * 9
desk_status = ['_'] * 9

while True:
    ret, img = videoCaptureObject.read()
    grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    (thresh, img) = cv2.threshold(grayImage, 220, 255, cv2.THRESH_BINARY)
    img = cv2.erode(img, kernel, iterations=1)

    img1 = img[293:414, 240:400]
    img1 = cv2.bitwise_not(img1)
    contours, _ = cv2.findContours(img1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(img1, contours, -1, (255, 255, 255), thickness=-1)
    count = cv2.countNonZero(img1)
    print('Density nonzero:', count / img1.size)
    print('Img size;', img1.size)

    desk_img[0] = img[15:134, 75:220]
    desk_img[1] = img[20:134, 240:390]
    desk_img[2] = img[25:134, 410:560]

    desk_img[3] = img[158:274, 75:230]
    desk_img[4] = img[158:274, 240:400]
    desk_img[5] = img[158:274, 410:565]

    desk_img[6] = img[293:414, 75:230]
    desk_img[7] = img[293:414, 240:400]
    desk_img[8] = img[293:414, 410:570]

    for i in range(len(desk_img)):
        cell = desk_img[i]
        cell = cv2.bitwise_not(cell)
        contours, _ = cv2.findContours(cell, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(cell, contours, -1, (255, 255, 255), thickness=-1)
        white_density = cv2.countNonZero(cell) / cell.size

        if white_density > 0.30:
            desk_status[i] = '0'
        elif white_density > 0.08:
            desk_status[i] = 'x'
        else:
            desk_status[i] = '_'

    output = ''.join(desk_status)
    print('Current desk status:', output)

    cv2.imshow('Result', img1)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        videoCaptureObject.release()
        cv2.destroyAllWindows()
