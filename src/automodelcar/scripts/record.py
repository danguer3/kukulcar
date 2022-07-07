import cv2
import numpy as np

# Create a VideoCapture object
cap = cv2.VideoCapture(0)
# Check if camera opened successfully
if (cap.isOpened() == False): 
  print("Unable to read camera feed")

# Default resolutions of the frame are obtained.The default resolutions are system dependent.
# We convert the resolutions from float to integer.
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

# Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))

while(True):
  ret, frame = cap.read()

  if ret == True: 
    
    # Write the frame into the file 'output.avi'
    out.write(frame)
    dim = (160,120)
    frameR = cv2.resize(frame,dim)
    #cv2.imshow("img",frameR)
    #if cv2.waitKey(1) == ord('q'):
    #    break
cap.release()
cv2.destroyAllWindows()
