import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Header
import matplotlib.pyplot as plt

def processImage(image, lower_white, upper_white):

    lower_grey = np.array([90, 0, 0])
    upper_grey = np.array([255, 124, 255])

    # blur image
    blur = cv2.GaussianBlur(image,(15,15),0)

    # generate mask
    mask = cv2.inRange(blur, lower_white, upper_white)
    maskg = cv2.inRange(blur, lower_grey, upper_grey)
    
    # filter out lanes
    result = andImg(image, mask)
    resultg = andImg(image, maskg)
    
    #Convert image to grayscale, apply threshold
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    grayg = cv2.cvtColor(resultg, cv2.COLOR_BGR2GRAY)
    
    _, thresh = cv2.threshold(gray, 230, 255, cv2.THRESH_BINARY)
    _, threshg = cv2.threshold(grayg, 135, 255, cv2.THRESH_BINARY)
    
    thresh_combined = cv2.add(thresh, threshg)

    final_thresh = np.zeros(thresh_combined.shape[:2],dtype=np.uint8)

    contours, _ = cv2.findContours(image =thresh_combined , mode = cv2.RETR_TREE,method = cv2.CHAIN_APPROX_SIMPLE)

    # loop through the contours
    for i,cnt in enumerate(contours):
        if  cv2.contourArea(cnt) > 50:
            cv2.drawContours(final_thresh,[cnt], 0, (255), -1)

    return final_thresh

def region_of_interest(image):
    height, width =image.shape
    rectangle=np.array([[(0, 0), (0, 290), (width, 290), (width, 0)]])
    mask=255*np.ones((height, width, 1), np.uint8)
    cv2.fillPoly(mask, rectangle, 0)

    roi=andImg(mask, image)
    return roi

def orImg(image1, image2):
    result=cv2.bitwise_or(image1, image2)

    return result

def andImg(image, mask):
    result = cv2.bitwise_and(image, image, mask = mask)

    return result

def convertCloudFromOpen3dToRos(open3d_cloud, FIELDS_XYZ, frame_id="camera_depth_frame"):
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)
    fields=FIELDS_XYZ
    cloud_data=points
    
    # # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)
