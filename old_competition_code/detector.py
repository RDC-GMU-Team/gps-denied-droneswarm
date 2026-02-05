import os.path
from time import perf_counter
import cv2
from cv2 import CAP_V4L2
from cv2 import IMWRITE_JPEG_QUALITY
from cv2 import error as cv2_error
from cv2 import VideoCapture
import socket
import struct
import numpy as np
from dataclasses import dataclass
from typing import List
import json


@dataclass
class Tag:
    id: int
    lat: int
    lon: int
    times_tried: int
    selected: bool
    sprayed: bool

try:
    from picamera2 import Picamera2
except ImportError:
    pass


# Resolution of video capture
W_RES = 640
H_RES = 480

# ArUCo Params
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}
ARUCO_TAG_DICT = ARUCO_DICT['DICT_6X6_250']
ARUCO_FRIENDLY = [16]
ARUCO_ENEMY = [17, 18, 19, 20]
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters()


class Detector:
    DUMP_TIME = 2
    CAM_DEG_FOV = 110

    def __init__(self, device_type: str):
        # 0 = SIM, 1 = picam
        self.mode = 0 if device_type.upper() == 'REAL' else 0
        with open('tags.json') as json_file:
            self.aruco_list = json.load(json_file)
 
        # print(f'mode: {self.mode}')
        if self.mode == 0:
            self.s = 0
            self.socket_setup('127.0.0.1', 5599)
        else:
            self.picam2 = 0
            self.picam_setup()

    def socket_setup(self, ip: str, port: int):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((ip, port))

    def picam_setup(self):
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            {'format': 'BGR888', 'size': (640, 480)})
        self.picam2.configure(config)
        self.picam2.start()

    def getCamFrame(self):
        if self.mode == 0:
            header_size = struct.calcsize("=HH")
            header = self.s.recv(header_size)
            if len(header) != header_size:
                print("Header size mismatch")

            width, height = struct.unpack("=HH", header)

            # receive image
            bytes_to_read = width * height
            img = bytes()
            while len(img) < bytes_to_read:
                img += self.s.recv(min(bytes_to_read - len(img), 4096))

            # convert incoming bytes to a numpy array (a grayscale image)
            frame = np.frombuffer(img, np.uint8).reshape((height, width))

            # vis0 = cv.fromarray(frame)

            return frame
        elif self.mode == 1:
            frame_im = self.picam2.capture_array()
            frame_im = cv2.cvtColor(frame_im, cv2.COLOR_RGB2BGR)

            return frame_im

    def arucoDisplay(self, corners, ids, rejected, image):
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

                designation_str = 'Enemy' if markerID in ARUCO_ENEMY else 'Friend' if markerID in ARUCO_FRIENDLY else 'Unknown'
                cv2.putText(image, designation_str, (bottomLeft[0], bottomLeft[1] + 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)
                print("[Inference] ArUco marker ID: {}".format(markerID))
            # show the output image
        # return image

    def detect(self):

        frame = self.getCamFrame()
        if self.mode == 0:
            detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
            (corners, ids, rejected) = detector.detectMarkers(frame)
        else:
            aruco_dict_rpi = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
            aruco_params_rpi = cv2.aruco.DetectorParameters_create()
            corners, ids, rejected = cv2.aruco.detectMarkers(
                frame, aruco_dict_rpi, parameters=aruco_params_rpi)

        # self.arucoDisplay(corners, ids, rejected, frame)

        return (frame, ids, corners)


    def total_aruco(self, count: int):
        self.aruco_count = count

    # Get next tag not "sprayed" where location is not 0,0
    def next_tag(self):
        # Find tags with times_tried == 0
        for t in self.aruco_list:
            if self.aruco_list[t]['times_tried'] == 0 and not self.aruco_list[t]['sprayed']:
                # update field
                return self.aruco_list[t]
        # Find tags with not sprayed and least amount of times tried
        least = None
        for t in self.aruco_list:
            if not self.aruco_list[t]['sprayed'] and least == None:
                least = self.aruco_list[t]
            # if not sprayed and current tag has more runs
            elif not self.aruco_list[t]['sprayed'] and least['times_tried'] > self.aruco_list[t]['times_tried']:
                least = self.aruco_list[t]
        return least

    # def set_list(self, index: int, tag: Tag):
    #     self.aruco_list[index] = tag

    def set_selected(self, tag:str):
        self.aruco_list[tag]['selected']= True

    def set_sprayed(self, tag:str ):
        self.aruco_list[tag]['sprayed'] = True
