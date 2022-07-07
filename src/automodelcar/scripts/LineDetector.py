import cv2
import numpy as np
import rospy
def main():
    cap = cv2.VideoCapture(1)
    img2 = cv2.imread("formas.jpeg")
    #img = np.zeros((480,640,3),np.uint8)
    #img[128:256,0:256] = np.ones((128,256,3),np.uint8)*255
    #img[256,256] = (255,255,255)
    #cv.imshow("Mascara",img)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        ret, img = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        #cv2.imshow('Original', img)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        #cv2.imshow('HSV', img_hsv)
        umbral_bajo_w = (0,0,240)
        umbral_alto_w = (95,30,255)
        #print(img_hsv[20][260])
        maskw = cv2.inRange(img_hsv, umbral_bajo_w, umbral_alto_w)


        #print(maskr[60][106])
        #res = cv2.bitwise_and(img, img, mask=mask)
        maskw = maskw[240:480, 0:640]
        maskw_final = maskw.copy()
        kernel = np.ones((5,5),np.float32)/25
        maskw_final = cv2.filter2D(maskw,-1,kernel)
        for i in range((maskw_final.shape)[0]):
            for j in range((maskw_final.shape)[1]):
                if(maskw_final[i][j]<255):
                    maskw_final[i][j] = 0
        maskw_final_l = maskw_final[0:240, 0:320]
        maskw_final_r = maskw_final[0:240, 320:640]

        #cv2.imshow('Mask white left', maskw_final_l)
        #cv2.imshow('Mask white right', maskw_final_r)
        maskw_array_l = maskw_final_l.flatten()
        maskw_array_r = maskw_final_r.flatten()
        pixels_desity_l = 0
        pixels_desity_r = 0
        for pixel in maskw_array_l:
            pixels_desity_l += pixel
        pixels_desity_l = pixels_desity_l/255
        for pixel in maskw_array_r:
            pixels_desity_r += pixel
        pixels_desity_r = pixels_desity_r/255
        print('Density right ' , pixels_desity_r)
        print('Density left ' , pixels_desity_l)
        #cv2.imshow('Mask Red', maskr)
        #cv2.imshow('Mask Yellow', masky)
        if cv2.waitKey(1) == ord('q'):
            break

if __name__ == '__main__':
	main()
