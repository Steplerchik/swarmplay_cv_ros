import cv2
import pytesseract
import numpy as np

videoCaptureObject = cv2.VideoCapture(0)
kernel = np.ones((5, 5), np.uint8)
kernel2 = np.ones((7, 7), np.uint8)
bordersize = 20
desk_img = [None] * 9
desk_status = ['_'] * 9

while True:
    ret, img = videoCaptureObject.read()
    grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    (thresh, img) = cv2.threshold(grayImage, 217, 255, cv2.THRESH_BINARY)
    img = cv2.erode(img, kernel, iterations=1)


    desk_img[0] = img[25:134, 75:250]
    desk_img[1] = img[20:134, 200:430]
    desk_img[2] = img[25:134, 410:570]

    desk_img[3] = img[158:274, 75:250]
    desk_img[4] = img[158:274, 200:430]
    desk_img[5] = img[158:274, 410:575]

    desk_img[6] = img[293:414, 75:250]
    desk_img[7] = img[293:414, 200:430]
    desk_img[8] = img[293:414, 410:570]

    for i in range(len(desk_img)):
        cell = desk_img[i]
        cell = cv2.copyMakeBorder(
            cell,
            top=bordersize,
            bottom=bordersize,
            left=bordersize,
            right=bordersize,
            borderType=cv2.BORDER_CONSTANT,
            value=[255, 255, 255])

        cell_text = pytesseract.image_to_string(cell).lower()
        if 'x' in cell_text:
            desk_status[i] = 'x'
        elif 'o' in cell_text:
            desk_status[i] = '0'

    output = ''.join(desk_status)
    print('Current desk status:', output)

    cv2.imshow('Result', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        videoCaptureObject.release()
        cv2.destroyAllWindows()
