import time

from pymavlink import mavutil
import pymavlink.dialects.v20.ardupilotmega as mavlink
from pymavlink.dialects.v20.ardupilotmega import (MAVLink_global_position_int_message, MAVLink_attitude_message,
                                                  MAVLink_set_position_target_global_int_message,
                                                  MAVLink_set_position_target_local_ned_message,
                                                  MAVLink_set_position_target_local_ned_message,
                                                  MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_FRAME_BODY_OFFSET_NED,
                                                  MAV_FRAME_BODY_OFFSET_NED, MAV_CMD_DO_SET_MODE, MAV_CMD_NAV_TAKEOFF,
                                                  MAV_CMD_NAV_WAYPOINT,
                                                  MAV_CMD_GUIDED_CHANGE_SPEED,
                                                  MAV_CMD_NAV_LAND)
from pymavlink.dialects.v20.common import (MAV_CMD_DO_CHANGE_SPEED)

from time import sleep
from time import perf_counter
import numpy as np
import asyncio
from typing import Tuple
from detector import W_RES, H_RES

CONTROLER_HOLD_ALT = 5
CONTROLLER_LOWEST_ALT = 3

''' angle: uniform function for angle
    distance: function for distance
    field_height: max for distance function '''
def sample_functions(angle, distance, field_height):
    get_angle = angle.rvs(size = 1)[0]
    # preventative measures of angle greater than 180 or less than -180, should not happen though...
    get_angle = min(get_angle, 180) if get_angle > 180 else max(get_angle, -180)
    return get_angle, min(distance.rvs(size = 1)[0], field_height)

def get_height(xErr, yErr):
    return CONTROLLER_LOWEST_ALT + np.sqrt(xErr**2 + yErr**2)*2/ np.sqrt((W_RES/2)**2 + (H_RES/2)**2)

class Controller:
    HOLD_ALT = CONTROLER_HOLD_ALT
    LOWEST_ALT = CONTROLLER_LOWEST_ALT
    reached = False

    def __init__(self, device_type: str):
        connStr = "/dev/ttyAMA0" if device_type.upper() == 'REAL' else "udp:localhost:14550"
        self.conn = mavutil.mavlink_connection(connStr)
        self.conn.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" %
              (self.conn.target_system, self.conn.target_component))

    # def __init__(self, connStr: str):
    #     self.conn = mavutil.mavlink_connection(connStr, baud=57600)
    #     self.conn.wait_heartbeat()
    #     print("Heartbeat from system (system %u component %u)" %
    #           (self.conn.target_system, self.conn.target_component))
    #
    def arm(self):
        # Set mode to guided
        self.conn.mav.command_long_send(
            self.conn.target_system,  # target_system
            self.conn.target_component,
            mavlink.MAV_CMD_DO_SET_MODE,  # command
            0,          # confirmation
            89,         # param1 (Mode)
            4,          # param2 (Custom mode)
            0,          # param3 (Custom Sub Mode)
            0,          # param4 (None)
            0,          # param5 (None)
            0,          # param6 (None)
            0)          # param7 (None)

    # Arm motor
        self.conn.arducopter_arm()
        sleep(1)

    def takeoff(self, altitude):
        self.conn.mav.command_long_send(
            self.conn.target_system,  # target_system
            self.conn.target_component,
            mavlink.MAV_CMD_NAV_TAKEOFF,  # command
            0,          # confirmation
            0,          # param1 (Min Pitch)
            0,          # param2 (None)
            0,          # param3 (None)
            0,          # param4 (Yaw Angle)
            0,          # param5 (Latitude)
            0,          # param6 (Longitude)
            altitude)   # param7 (Altitude)

    # async def move_vel(self, pos):
    #     move_vel_msg = mavlink.MAVLink_set_position_target_local_ned_message(
    #
    #     )

    def move_pos_vel(self, x, y, z, yaw_rate=0):
        # Generate a move message based on relative position received
        move_msg = mavlink.MAVLink_set_position_target_local_ned_message(
            int(perf_counter() * 1000),
            self.conn.target_system,
            self.conn.target_component,
            mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0,
            0,
            0,
            0,
            x,
            y,
            z,
            0,
            0,
            0,
            0,
            yaw_rate
        )
    # Send move message
        self.conn.mav.send(move_msg)

    def land(self):
        self.conn.set_mode_rtl()

    def move_pos_rel(self, x, y, z):
        # Generate a move message based on relative position received
        move_msg = mavlink.MAVLink_set_position_target_local_ned_message(
            int(perf_counter() * 1000),
            self.conn.target_system,
            self.conn.target_component,
            mavlink.MAV_FRAME_BODY_OFFSET_NED,
            3576,
            x,
            y,
            z,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0
        )
    # Send move message
        self.conn.mav.send(move_msg)

    def distance(self, point1: Tuple[int, int], point2: Tuple[int, int]):
        point1 = np.asarray(point1)
        point2 = np.asarray(point2)

        return np.linalg.norm(point1 - point2)

    def go_to(self, lat, lon, alt, yaw=0):
        """
            Move the robot to GPS coordinate
        """
        move_msg = MAVLink_set_position_target_global_int_message(
            int(perf_counter() * 1000),
            self.conn.target_system,
            self.conn.target_component,
            MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            504,
            lat,
            lon,
            alt,
            0,
            0,
            0,
            0,
            0,
            0,
            yaw,
            0
        )
        # Send move message
        self.conn.mav.send(move_msg)

    def move_pos_rel_yaw(self, x, y, z, yaw):
        # Generate a move message based on relative position received
        move_msg = mavlink.MAVLink_set_position_target_local_ned_message(
            int(perf_counter() * 1000),
            self.conn.target_system,
            self.conn.target_component,
            mavlink.MAV_FRAME_BODY_OFFSET_NED,
            3576,
            x,
            y,
            z,
            0,
            0,
            0,
            0,
            0,
            0,
            yaw,
            0
        )
        # Send move message
        self.conn.mav.send(move_msg)

    def get_position(self):
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            33,
            0,
            0,
            0,
            0,
            0,
            0)
        pos = self.conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        return {'lat':pos.lat, 'lon':pos.lon}


    def dump_water(self):
        servo_msg = mavlink.MAVLink_command_long_message(
            self.conn.target_system,
            self.conn.target_component,
            mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            9, #SERVO pin -> Aux 1 is servo Pin 9
            2000,
            0,
            0,
            0,
            0,
            0
        )
        self.conn.mav.send(servo_msg)

    def dump_water_stop(self):
        servo_msg = mavlink.MAVLink_command_long_message(
            self.conn.target_system,
            self.conn.target_component,
            mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            9, #SERVO pin -> Aux 1 is servo Pin 9
            1500,
            0,
            0,
            0,
            0,
            0
        )
        self.conn.mav.send(servo_msg)

    def set_max_velocity(self, speed):
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0, # airspeed
            speed, # speed
            -1,  # no change to throttle
            0,
            0,
            0,
            0,
            0)
    
    def land_at_place(self, lon, lat, alt):
        self.conn.mav.command_long_send(
            self.conn.target_system,  # target_system
            self.conn.target_component,
            mavlink.MAV_CMD_NAV_LAND,  # command
            0,
            0,
            0,
            0,
            0,
            lat,
            lon,
            0
            )

    def check_reached(self, mode, goto_pos, cur_pos):
        if mode.upper() == 'LEVY':
            # if x then check if lat is about the same as the destination
            print(goto_pos,cur_pos)
            return self.distance(goto_pos, cur_pos) <= 10
        elif mode.upper() == 'LAWNMOWER':
            return self.distance(goto_pos, cur_pos) <= 5
        else: 
            return self.distance(goto_pos, cur_pos) <= 10

    ''' If we are going up the field then we need to check to see if the current
        latitude is greater than the latitude of the end of the field. Otherwise, 
        we need to check the other way around. This logic also accounts for if going 
        up the field results in an increase of positive latitude or negative through
        "climbingIsPositive" parameter.
        
        Essentially:

        climbling: going up the field
        climbingIsPositive: if going up the field adds or subtracts to latitude
        '''
    def field_end_reached(self, goto_pos, cur_pos, climbing, climbingIsPositive):
        if climbingIsPositive:
            if climbing:
                return cur_pos['lat'] >= goto_pos['lat']
            return cur_pos['lat'] <= goto_pos['lat']
        else:
            # Climbing is negative
            if climbing:
                return cur_pos['lat'] <= goto_pos['lat']
            return cur_pos['lat'] >= goto_pos['lat']

    # Returns levy's position to go to
    def levy(self, uniform_model, levy_model, field_height):
        # sample for angle and length:
        angle, length = sample_functions(uniform_model, levy_model, field_height)
        print(f'samples = angle:{angle}, distance:{length}')
        length = length * 2
        # extract x and y distances
        goto_pos = self.get_position()
        # int typecase due to int requirement of go_to
        goto_pos['lat'] = int(goto_pos['lat'] + (int(length) * np.cos(int(angle))))
        goto_pos['lon'] = int(goto_pos['lon'] + (int(length) * np.sin(int(angle))))
        # goto distance location first
        print(f'going to pos: {goto_pos}')
        self.go_to(goto_pos['lat'], goto_pos['lon'], self.HOLD_ALT)
        return goto_pos

    def until_pos_not_reached(self, lat, lon, mode: str):
        reach_pos = False
        while not reach_pos:
            reached_loc = self.get_position()
            reach_pos = self.check_reached(mode, (lat, lon), (reached_loc['lat'], reached_loc['lon']))
        return True

    def move_vel_rel_alt(self, vx, vy, z):
        move_msg = MAVLink_set_position_target_local_ned_message(
            int(perf_counter() * 1000),
            self.conn.target_system,
            self.conn.target_component,
            MAV_FRAME_BODY_OFFSET_NED,
            3555,
            0,
            0,
            z,
            vx,
            vy,
            0,
            0,
            0,
            0,
            0,
            0
        )
        # Send move message
        self.conn.mav.send(move_msg)