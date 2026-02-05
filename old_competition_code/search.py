from detector import Detector
from controller import Controller
import math
from scipy.stats import levy, uniform
import numpy as np
from dataclasses import dataclass
import json

@dataclass
class custom_pos:
    lat: int
    lon: int

def increment_distance(distance, increment):
    if increment < 0:
        return (distance + increment) * -1 if (distance < 0) else (distance - increment) * -1
    else:
        return (distance + increment) * -1 if (distance >= 0) else (distance - increment) * -1

# for lawnmower
def setup_lane(cur_lane: int, lon_axis, lat_axis, field_map):
    alternate = True
    lane = []
    for y in lat_axis:
        if alternate:
            lane.append(custom_pos(y, lon_axis[cur_lane]))
        else:
            lane.append(custom_pos(y, lon_axis[cur_lane + 1]))
        alternate = not alternate
    return lane

# for lawnmower
def setup_field(lanes: int, lon_axis, lat_axis, field_map):
    for l in range(lanes):
        field_map.append(setup_lane(l, lon_axis, lat_axis, field_map))

class Search:
    # CHALLANGE MODE
    mode = ''
    cur_pos = 0
    goto_pos = 0

    # Sprial fields
    distance = Detector.CAM_DEG_FOV
    increment = (int)(.8 * Detector.CAM_DEG_FOV)  # 20% overlap
    pos_reached = True
    MAX_SPIRALS = 12 # 12 edges of a spiral (3 spirals)
    travel_x = True
    tag = 0
    total_spirals = 0

    # Lawnmower fields
    lanes = 2
    deg_inc = 10
    search_lane = 0
    search_point = 0
    search_up_array = True
    # lane 1 = [pos(x, y), pos2(x, y), ...] , lane 2 = [...] , ...
    field_map = []
    # Read start and end of field form json file
    with open('field.json') as json_file:
        data = json.load(json_file)
        bottom_left = data['bottom left']
        bottom_right = data['bottom right']
        top_left = data['top left']
        top_right = data['top right']
    # x-axis in sim
    lon_axis = list(map(int, np.linspace(bottom_left['lon'], bottom_right['lon'], lanes + 1)))
    # y-axis in sim
    lat_axis = list(map(int, np.linspace(bottom_left['lat'], top_left['lat'], deg_inc)))

    print(f'lon_axis: {lon_axis}, lat_axis: {lat_axis}')
    print(f'bottom_left: {bottom_left}, bottom_right: {bottom_right} top_left: {top_left}, top_right: {top_right}')

    # Levy Walk:
    field_height = 50
    # angle function setup: degrees from -180 to 180
    uniform_model = uniform(loc=-180, scale=360)
    # distance function setup: width of field TODO: convert from degrees to meters
    # min distance: 1, max: field_height
    x = np.linspace(1, field_height, 100)

    def __init__(self, mode: str = 'SPRIAL'):
        self.mode = mode.upper()

    def set_mode(self, mode: str):
        self.mode = mode.upper()

    def search(self, ctrl: Controller, detect: Detector):
        if self.mode == 'SPIRAL':
            self.spiral(ctrl, detect)
        elif self.mode == 'LAWNMOWER':
            self.lawnmower(ctrl, detect)
        elif self.mode == 'LEVY':
            self.levy(ctrl, detect)
        else:
            return 'Invalid Search'

    def reset_param(self):
        if self.mode == 'SPIRAL':
            # Get next tag after dumping water (for spiral)
            self.tag = 0
            # reset spiral distance after detection (For sprial mode)
            self.distance = Detector.CAM_DEG_FOV

    def spiral(self, ctrl: Controller, detect: Detector):
        # get aruco
        if self.tag == 0:
            self.goto_pos = self.tag = detect.next_tag()
            # no more tags = go home
            if self.tag is None:
                print('Returning None')
                return
            print(f'going to new tag id: {self.tag["id"]}')
            ctrl.go_to(self.tag['lat'], self.tag['lon'], Controller.HOLD_ALT)
            self.pos_reached = False
        if self.pos_reached:
            # If 3 sprials (total_spirals == 11) move to next tag
            if self.total_spirals == self.MAX_SPIRALS:
                print('3 sprials complete, moving on...')
                self.tag['times_tried'] += 1
                self.tag = 0
                self.total_spirals = 0
            else:
                self.pos_reached = not self.pos_reached  # set false to show need to travel to new pos
                if self.travel_x and (self.goto_pos != self.tag):  # increment every x rotation (goto_pos == tag if running for the first time)
                    self.distance = increment_distance(self.distance, self.increment)
                self.goto_pos = ctrl.get_position()
                self.goto_pos['lat'] = self.goto_pos['lat'] + (self.distance if self.travel_x else 0)
                self.goto_pos['lon'] = self.goto_pos['lon'] + (self.distance if not self.travel_x else 0)
                self.total_spirals += 1
                print(f'going to pos: {self.goto_pos}')
                ctrl.go_to(self.goto_pos['lat'], self.goto_pos['lon'], Controller.HOLD_ALT)
        else:
            self.cur_pos = ctrl.get_position()
            ctrl.go_to(self.goto_pos['lat'], self.goto_pos['lon'], Controller.HOLD_ALT)
            # check if position reached
            if ctrl.check_reached(self.mode, (self.goto_pos['lat'], self.goto_pos['lon']), (self.cur_pos['lat'], self.cur_pos['lon'])):
                self.goto_pos = 0
                self.pos_reached = True
                self.travel_x = not self.travel_x

    def next_pos_field(self):
        # traverse array
        if self.search_up_array:
            # no points left in lane, go to next lane
            if self.search_point + 1 >= len(self.field_map[self.search_lane]):
                print(f'next point, lane: {self.search_lane}, point: {self.search_point}')
                self.search_lane += 1
                self.search_point = 0
            # no lanes left
            if self.search_lane >= len(self.field_map):
                self.search_up_array = not self.search_up_array
                self.search_point = 0
                self.search_lane -= 1
                self.pos_reached = True
                return 'switch'
            else:
                print(f'next point, lane: {self.search_lane}, point: {self.search_point}')
                self.search_point += 1
                return self.field_map[self.search_lane][self.search_point - 1]
        # traverse array in reverse
        else:
            # lanes points left in lane, go to next lane
            if self.search_point + 1 >= len(self.field_map[self.search_lane]):
                self.search_lane -= 1
                self.search_point = 0
            # no lanes left
            if self.search_lane < 0:
                self.search_up_array = not self.search_up_array
                self.search_point = 0
                self.search_lane += 1
                self.pos_reached = True
                return 'switch'
            else:
                self.search_point += 1
                return self.field_map[self.search_lane][self.search_point - 1]

    def lawnmower(self, ctrl: Controller, detect: Detector):
        # Set up DP array of field
        if not self.field_map:
            setup_field(self.lanes, self.lon_axis, self.lat_axis, self.field_map)
            print(self.field_map)
        self.cur_pos = ctrl.get_position()
        if self.pos_reached:
            # set goto_pos only if prev point had no detection (-1 == had detection)
            if self.goto_pos != -1:
                print('new pos selected')
                # keep the same up distance and go to edge of field to continue from right side
                self.goto_pos = self.next_pos_field()
                # redo if we are the opposite direction now
                if self.goto_pos == 'switch':
                    self.goto_pos = self.next_pos_field()
                self.pos_reached = False
            else:
                print('same pos selected')
                self.goto_pos = self.field_map[self.search_lane][self.search_point - 1]
            print(f'going to pos: {self.goto_pos}')
            ctrl.go_to(self.goto_pos.lat, self.goto_pos.lon, ctrl.HOLD_ALT)
        else:
            self.cur_pos = ctrl.get_position()
            # check if position reached, if so next 2 lines, otherwise nothing
            if ctrl.check_reached(self.mode, (self.goto_pos.lat, self.goto_pos.lon), (self.cur_pos['lat'], self.cur_pos['lon'])):
                print(f'reached; cur_pos: {self.cur_pos}, goto_pos: {self.goto_pos}')
                self.pos_reached = True

    def levy(self, ctrl: Controller, detect: Detector):
        if self.pos_reached:
            self.pos_reached = not self.pos_reached
            self.goto_pos = ctrl.levy(self.uniform_model, levy, self.field_height)
        else:
            self.cur_pos = ctrl.get_position()
            # check if position reached, if so next line, otherwise nothing
            if ctrl.check_reached(self.mode, (self.goto_pos['lat'], self.goto_pos['lon']), (self.cur_pos['lat'], self.cur_pos['lon'])):
                print(f'reached; cur_pos: {self.cur_pos}, goto_pos: {self.goto_pos}')
                self.pos_reached = True
