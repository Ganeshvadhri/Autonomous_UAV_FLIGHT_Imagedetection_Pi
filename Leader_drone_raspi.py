import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob
import picamera
import time
from time import sleep
import socket
import pandas as pd
BBox = ((79.59415119141339,   79.59425280299188,      
         13.707720414574958,13.707602013165788))
from dronekit import connect,VehicleMode,LocationGlobalRelative
import argparse
parser = argparse.ArgumentParser(description = 'commands')
parser.add_argument('--connect')
args = parser.parse_args()
connection_string = "/dev/ttyAMA0"
baud_rate = 115200
#print("connection to the vehicle on %s" %connection_string)
vehicle = connect(connection_string,baud = baud_rate,wait_ready =True)
UDP_IP = "192.168.43.102"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
box_count = 0
df = pd.DataFrame()
while True:
    camera = picamera.PiCamera()
    camera.start_preview()
    camera.capture("/home/pi/Desktop/image.jpg")
    camera.stop_preview()
    leader_gps = vehicle.location.global_frame
    img = cv2.imread("/home/pi/Desktop/image.jpg")
    img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    img_rgb = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    green_min = np.array([147,193, 139], np.uint8)
    green_max = np.array([155, 199, 147], np.uint8)
    threshold_green_img = cv2.inRange(img_rgb, green_min, green_max)
    threshold_green_img = cv2.cvtColor(threshold_green_img, cv2.COLOR_GRAY2RGB)
    ret,binary_3 = cv2.threshold(threshold_green_img,127,256,cv2.THRESH_BINARY)
    kernel_sharpening = np.array([[-1,-1,-1],
                              [-1, 9,-1],
                              [-1,-1,-1]])
    sharpened = cv2.filter2D(threshold_green_img, -1, kernel_sharpening)
    gray = cv2.cvtColor(threshold_green_img, cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(gray, 50, 250)
    k_1 = np.ones((39,39), np.uint8)
    img_dilation_1 = cv2.dilate(edges, k_1, iterations=2)
    k_2 = np.ones((57,57), np.uint8)
    img_erosion = cv2.erode(img_dilation_1, k_2, iterations=2)
    k_3 = np.ones((21,21), np.uint8)
    img_dilation_2 = cv2.dilate(img_erosion, k_1, iterations=2)
    contours, hierarchy = cv2.findContours(img_erosion, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    data, addr = sock.recvfrom(1024)
    if '&' in data:
        box_count+=1
        b=data.split('&')
        df = df.append({'latitude':float(b[0]),'longitude': float(b[1])}, ignore_index=True)
        fig, ax = plt.subplots(figsize = (8,7))
        ax.scatter(df.longitude, df.latitude, zorder=1, alpha= 1, c='r', s=2)
        ax.set_title('Plotting Spatial Data')
        ax.set_xlim(BBox[0],BBox[1])
        ax.set_ylim(BBox[2],BBox[3])
        #ax.text(13.70765, 79.5942, ". Data: (1, 5)")
        ax.imshow(img, zorder=0, extent = BBox, aspect= 'equal')
        #ax.annotate(str(79.5942),xy=(13.70765,79.5942))
        #plt.text(13.70765,79.5942,'This text starts at point (2,4)')
        plt.show()
    if len(contours) == 1:
        df = df.append({'latitude':float(leader_gps.lat),'longitude': float(leader_gps.lon)}, ignore_index=True)
        box_count+=1
        fig, ax = plt.subplots(figsize = (8,7))
        ax.scatter(df.longitude, df.latitude, zorder=1, alpha= 1, c='r', s=2)
        ax.set_title('Plotting Spatial Data')
        ax.set_xlim(BBox[0],BBox[1])
        ax.set_ylim(BBox[2],BBox[3])
        #ax.text(13.70765, 79.5942, ". Data: (1, 5)")
        ax.imshow(img, zorder=0, extent = BBox, aspect= 'equal')
        #ax.annotate(str(79.5942),xy=(13.70765,79.5942))
        #plt.text(13.70765,79.5942,'This text starts at point (2,4)')
        plt.show()
    if box_couunt ==5:
        break;
