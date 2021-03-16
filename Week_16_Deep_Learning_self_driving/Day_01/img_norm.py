#!/usr/bin/env python3

import cv2, glob, os

video_files = glob.glob("*.mkv")
caps = [cv2.VideoCapture(name) for name in video_files]
cap_length = max([int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) for cap in caps])

#video_file = 'epoch01_front.mkv'
#cap = cv2.VideoCapture(video_file)
#cap = cv2.VideoCapture(0)

if not os.path.isdir("./images/"):
    os.mkdir("./images/")

for i in range(cap_length):
    for cap in caps:
        cap_name = str(cap).split(" ")[1][:-1]
        
        ret, img = cap.read()
        if not ret: 
            continue
        img = cv2.resize(img, dsize=(200, 112))
        img = img[46:112, :]
        cv2.imwrite("images/" + str(i) + "-" + cap_name + ".jpg", img)
        #cv2.imshow(cap_name, img)


for cap, video_file in zip(caps, video_files):
    current_name = str(video_file).split(".")[0]
    cap_name = str(cap).split(" ")[1][:-1]
    os.rename(current_name+".mkv", cap_name+".mkv")
    os.rename(current_name+".csv", cap_name+".csv")


for cap in caps:
    cap.release()
 
cv2.destroyAllWindows()
