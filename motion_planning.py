import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import re

import sys
import networkx as nx

from planning_utils import a_star1, a_star2, heuristic1, heuristic2, create_grid, prune_path, create_grid_and_edges, find_closest_point
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
from udacidrone.frame_utils import local_to_global

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        
        TARGET_ALTITUDE = 15
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        f = open("colliders.csv")
        line = f.readline()
        f.close()
        lat_lon = re.findall(r"[-+]?\d*\.\d+|\d+", line)
        lat0 = float(lat_lon[1])
        lon0 = float(lat_lon[3])
        #print([lon0, lat0, 0])
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)
        # TODO: retrieve current global position
        # TODO: convert to current local position using global_to_local()
        local_position = global_to_local(self.global_position, self.global_home)
        #print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         #self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        print("Creating a grid (and edges) ...")
        #grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("Grid created, creating voronoi graph")
        G = nx.Graph()
        for edge in edges:
        	p1 = edge[0]
        	p2 = edge[1]
        	dist = np.linalg.norm(np.array(p1) - np.array(p2))
        	G.add_edge(p1, p2, weight = dist)
        print("Created Graph")
        
        grid_start = (int(local_position[0]-north_offset), int(local_position[1]-east_offset))
 
        goal_local_pos = global_to_local([-122.39628088, 37.79698094, self.global_home[2]], self.global_home)
        #print(goal_local_pos)
        goal_x = int(np.ceil(goal_local_pos[0]) - north_offset)
        goal_y = int(np.ceil(goal_local_pos[1]) - east_offset) 		 
        grid_goal = (goal_x, goal_y)
        #grid_goal = (-north_offset + 10, -east_offset + 10)
        start_ne_g = find_closest_point(G, grid_start)
        goal_ne_g = find_closest_point(G, grid_goal)
        #print("Searching for a path ...")
        print("Starting a start search")
        path, path_cost = a_star2(G, heuristic2, start_ne_g, goal_ne_g)
        # TODO: prune path to minimize number of waypoints


        
        path = [(int(np.ceil(x[0])), int(np.ceil(x[1]))) for x in path]
        path = prune_path(path)
        grid_start = path[-1]
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        path2, _ = a_star1(grid, heuristic1, grid_start, grid_goal)
       	path2 = prune_path(path2)
       	path = path + path2
       	path = prune_path(path)
       	print(path)
        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        #print(waypoints)
        for i in range(1, len(waypoints)):
        	x_old = waypoints[i-1][0]
        	y_old = waypoints[i-1][1]

        	x_curr = waypoints[i][0]
        	y_curr = waypoints[i][1]
        	psi = np.arctan2(y_curr - y_old, x_curr - x_old)
        	waypoints[i][3] = psi
        	
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
