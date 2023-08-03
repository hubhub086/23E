import time
import cv2
import os


if __name__ =='__main__':
    save_folder = './img/'
    if not os.path.exists(save_folder):
        os.mkdir(save_folder)
        print('makedir: {}'.format(save_folder))
    cap = cv2.VideoCapture(2,cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
    cap.set(cv2.CAP_PROP_EXPOSURE, 60)
    # cap.set(cv2.CAP_PROP_BRIGHTNESS, 10)
    cap.set(3, 640)
    cap.set(4, 480)
    print(cap.get(3))
    print(cap.get(4))
    print(cap.get(cv2.CAP_PROP_AUTO_EXPOSURE))
    print(cap.get(cv2.CAP_PROP_EXPOSURE))
    print(cap.get(cv2.CAP_PROP_BRIGHTNESS))
    num = 0
    count = 0
    key = False
    while 1:
        _, frame = cap.read()
        # frame = frame[117:294,290:532]
        
        cv2.imshow("result", frame)

        k = cv2.waitKey(1)
        if k == 116:
            key = True
        elif k == 115:
            key = False
        elif k == 49:
            print('change camera sys')
            cap.set(3, 320)
            cap.set(4, 240)
            print(cap.get(3))
            print(cap.get(4))
        if k == 27:
            break
        if key:
            count = count + 1
            num += 1
            cv2.imwrite('./img/frame' + str(count)+'.jpg', frame)
            print('img' + str(count) + ' ---> '+ str(num))
            time.sleep(0.1)
            key = False
    cap.release()
    cv2.destroyAllWindows()
