"""
    File: main.py
    Author: Yojan Gautam
    Created: January 26, 2024
    Purpose: Brief description of the purpose of this script or module.

    Additional details and usage information can be added here as needed.
"""

import argparse
import controller
import detector
from controller import Controller, get_height
from detector import Detector, Tag
from search import Search
from logger import Logger
import cv2
import time
import traceback

PGAIN = 0.001
DGAIN = 0.000
IGAIN = 0.000

prevx = 0
prevy = 0
prevDetection = 0
ix = 0
iy = 0

# True for dumping water, false otherwise
def position(ctrl, x, y):
    global prevx
    global prevy
    global prevDetection
    global ix
    global iy
    centerX = 640 / 2
    centerY = 480 / 2
    xErr = x - centerX
    yErr = centerY - y

    dx = (xErr - prevx) / (time.time() - prevDetection)
    dy = (prevy - yErr) / (time.time() - prevDetection)

    ix = ix + dx * (time.time() - prevDetection)
    iy = iy + dy * (time.time() - prevDetection)


    track_pos = ctrl.get_position()
    # print(f'new alt: {new_alt}')
    ctrl.move_pos_vel(PGAIN * yErr + (IGAIN * iy) + (DGAIN * dy),
                      PGAIN * xErr + (IGAIN * ix) + (DGAIN * dx), 0)

    prevx = xErr
    prevy = yErr
    prevDetection = time.time()
    # print(f'xErr: {xErr}, yErr: {yErr}, x: {x}, y: {y}')
    if (-15 < xErr < 15) and (-15 < yErr < 15):
        return True

def main(device_type,search_type):
    ctrl = Controller(device_type)
    detect = Detector(search_type)
    search = Search(search_type)
    logger = Logger()
    ctrl.arm()
    ctrl.takeoff(ctrl.HOLD_ALT)
    # Not sure if this accepts low values like decimals...
    # ctrl.set_max_velocity(2)
    time.sleep(10)

    home = land = ctrl.get_position()

    # currently set to degrees TODO: Change to meters eventually
    try:
        while True:
            (img, ids, corners) = detect.detect()
            # cv2.imshow("image", img)
            # if cv2.waitKey(1) == ord("q"):
                # break

            # tag detected
            if len(corners) > 0:
                targets = dict(zip(ids.flatten(), corners))
                search.cur_pos = ctrl.get_position()
                # print("current latitude: " + str(search.cur_pos.lat))
                # print("current longitude: " + str(search.cur_pos.lon))
                # print(targets.keys())
                for target in targets.keys():
                    print(f'target: {target}')
                    # if not friendly aruco
                    if not detect.aruco_list[str(target)]['friendly'] and not detect.aruco_list[str(target)]['sprayed']:
                        print('enemy')
                        acctualcorners = targets[target].reshape((4, 2))
                        (topLeft, topRight, bottomRight, bottomLeft) = acctualcorners
                        x = bottomRight[0]
                        y = bottomRight[1]
                        if position(ctrl, x, y):
                            detect.set_selected(str(target))
                            # current time in seconds since epoch
                            start = time.time()
                            # log water dump
                            logger.log(search.cur_pos, target)
                            while (time.time() - start < Detector.DUMP_TIME):
                                print("DUMP water")
                                ctrl.dump_water()
                            # Select new target after dumping for 2 seconds
                            # FOR LAWNMOWER: return to prev position before detection
                            if search.mode.upper() == 'LAWNMOWER':
                                search.goto_pos = -1
                                search.pos_reached = True
                            detect.set_sprayed(str(target))
                            search.reset_param()
                            # search.new_tag()
                    else:
                        search.search(ctrl,detect)
                        if search.mode == 'SPIRAL' and search.tag is None:
                            break # go to finally section

            else:
                # change more if you want using search.set_mode()
                search.search(ctrl, detect)
                if search.mode == 'SPIRAL' and search.tag is None:
                    break # go to finally section

    except Exception as e:
        print(e)
        print(traceback.format_exc())
        print("Keyboard interrupt detected. Stopping the program. 1")
        ctrl.go_to(land['lat'], land['lon'], Controller.HOLD_ALT)
        ctrl.dump_water_stop()
        Controller.reached = ctrl.until_pos_not_reached(land['lat'], land['lon'], search.mode)
        ctrl.land_at_place(0, 0, 0)
    except:
        print("Main except: 2")
        if not Controller.reached:
            ctrl.go_to(land['lat'], land['lon'], Controller.HOLD_ALT)
            ctrl.dump_water_stop()
            Controller.reached = ctrl.until_pos_not_reached(land['lat'], land['lon'], search.mode)
            ctrl.land_at_place(0, 0, 0)
    finally:
        print("Finally...")
        if not Controller.reached:
            ctrl.go_to(land['lat'], land['lon'], Controller.HOLD_ALT)
            ctrl.dump_water_stop()
            Controller.reached = ctrl.until_pos_not_reached(land['lat'], land['lon'], search.mode)
            ctrl.land_at_place(0, 0, 0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
                    prog='Flysensei RDC',
                    description='Controls the RDC drone'
                    )
    parser.add_argument('-s','--searchtype', required=True , help="Type of search that you want to perform ['levy', 'spiral', 'lawnmower']")
    parser.add_argument('-t', "--type",  required=True, help="Type of device that you are running it [SIM/REAL]")
    args = parser.parse_args()
    main(args.type,args.searchtype)
