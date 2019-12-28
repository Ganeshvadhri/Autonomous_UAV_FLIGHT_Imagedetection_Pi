import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob
import picamera
import time
from time import sleep
import socket
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

    if len(contours) == 1:
        MESSAGE = str(leader_gps.lat) + "&" + str(leader_gps.lon)
        sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sock.sendto(MESSAGE,(UDP_IP,UDP_PORT))
