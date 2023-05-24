import cv2
import sys


for j in range(3,4):
    for k in range(3,4):
        # Opens the Video file
        #
        cap= cv2.VideoCapture(r"C:\Users\lidko\Desktop\MA1project\ArUCo-Markers-Pose-Estimation-Generation-Python-main\vods\video"+str(j)+str(k)+".mp4")
        count = 1
        i=1
        while(cap.isOpened()):
            ret, frame = cap.read()
            if ret == False:
                break
            if i % 15 == 0: # This is the that make it only save one frame every 15
                cv2.imwrite(r"C:\Users\lidko\Desktop\MA1project\ArUCo-Markers-Pose-Estimation-Generation-Python-main\sampled_images\rvr_"+str(j)+str(k)+"_"+str(count)+".jpg",frame)
                count += 1
            i+=1


        cap.release()
        cv2.destroyAllWindows()