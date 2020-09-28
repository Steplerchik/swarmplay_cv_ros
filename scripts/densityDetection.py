import cv2
import pytesseract
import numpy as np


# def adjust_gamma(image, gamma=1.0):
#     # build a lookup table mapping the pixel values [0, 255] to
#     # their adjusted gamma values
#     invGamma = 1.0 / gamma
#     table = np.array([((j / 255.0) ** invGamma) * 255
#                       for j in np.arange(0, 256)]).astype("uint8")
#     # apply gamma correction using the lookup table
#     return cv2.LUT(image, table)


videoCaptureObject = cv2.VideoCapture(2)
videoCaptureObject.set(cv2.CAP_PROP_ZOOM, 253)
videoCaptureObject.set(cv2.CAP_PROP_TILT, 7200)

kernel = np.ones((5, 5), np.uint8)
bordersize = 20
desk_img = [None] * 9
desk_status = ['_'] * 9

while True:
    ret, img = videoCaptureObject.read()
    grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    (thresh, img) = cv2.threshold(grayImage, 190, 255, cv2.THRESH_BINARY)
    img = cv2.erode(img, kernel, iterations=1)

    img1 = img


    contours, _ = cv2.findContours(img1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(img1, contours, -1, (255, 255, 255), thickness=-1)
    count = cv2.countNonZero(img1)
    print('Density nonzero:', count / img1.size)
    print('Img size;', img1.size)

    cv2.imshow('Result', img1)

    desk_img[0] = img[0:148, 35:208]
    desk_img[1] = img[0:148, 223:400]
    desk_img[2] = img[0:148, 418:585]

    desk_img[3] = img[164:304, 26:200]
    desk_img[4] = img[163:308, 219:397]
    desk_img[5] = img[160:307, 415:596]

    desk_img[6] = img[320:466, 18:193]
    desk_img[7] = img[320:466, 213:393]
    desk_img[8] = img[324:466, 412:597]

    for i in range(len(desk_img)):
        cell = desk_img[i]
        cell = cv2.bitwise_not(cell)
        contours, _ = cv2.findContours(cell, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(cell, contours, -1, (255, 255, 255), thickness=-1)
        white_density = cv2.countNonZero(cell) / cell.size

        if white_density > 0.25:
            desk_status[i] = '0'
        elif white_density > 0.08:
            desk_status[i] = 'x'
        else:
            desk_status[i] = '_'

    output = ''.join(desk_status)
    print('Current desk status:', output)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        videoCaptureObject.release()
        cv2.destroyAllWindows()
