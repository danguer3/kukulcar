import cv2
import numpy as np

def main():
    cap = cv2.VideoCapture(0)
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
        scale_percent = 30 # percent of original size
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        dim = (width, height)
        img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
        #cv2.imshow('Original', img)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        umbral_bajo_v = (20,100,100)
        umbral_alto_v = (45,200,180)
        umbral_bajo_r = (110,150,150)
        umbral_alto_r = (130,255,255)
        umbral_bajo_y = (80,150,150)
        umbral_alto_y = (100,255,255)
        #print(img_hsv[410][120])
        maskv = cv2.inRange(img_hsv, umbral_bajo_v, umbral_alto_v)
        maskr = cv2.inRange(img_hsv, umbral_bajo_r, umbral_alto_r)
        masky = cv2.inRange(img_hsv, umbral_bajo_y, umbral_alto_y)
        pixels_desity_g = 0
        pixels_desity_r = 0
        pixels_desity_y = 0
        green_array = maskv.flatten()
        yellow_array = masky.flatten()
        red_array = maskr.flatten()
        pixels_desity_g = sum(green_array)/255
        pixels_desity_r = sum(red_array)/255
        pixels_desity_y = sum(yellow_array)/255
        print("G {0} R {1} Y {2}".format(pixels_desity_g, pixels_desity_r,pixels_desity_y))
        if(pixels_desity_g > 5000):
            print("objeto verde")
        if(pixels_desity_r > 5000):
            print("objeto rojo")
        if(pixels_desity_y > 5000):
            print("objeto amarillo")


        #print(maskr[60][106])
        #res = cv2.bitwise_and(img, img, mask=mask)
        #cv2.imshow('Mask Green', maskv)
        #cv2.imshow('Mask Red', maskr)
        #cv2.imshow('Mask Yellow', masky)
        #if cv2.waitKey(1) == ord('q'):
           # break

if __name__ == '__main__':
	main()
