import argparse
import time
import msgpack
import random
from enum import Enum, auto

import numpy as np
import re

from planning_utils import heuristic, create_grid_and_edges, prune_path, create_graph_from_edges, a_star_graph, collision_check, add_altitude_gradient
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

class PointsOfInterest():
    def __init__(self):
        self.poi_dict = {
        "Mission Spear" : (float(-122.39314), float(37.79270), 5.0 ),
        "Washington Street" : (float(-122.40131), float(37.79666), 5.0 ),
        "California Street" : (float(-122.39624), float(37.79391), 5.0 ),
        "Sacramento Front" : (float(-122.39931), float(37.79471), 5.0 ),
        "Roof of building" : (float(-122.39876), float(37.79553), 136.0 )
        }

    def get(self, poi_name):
        return (self.poi_dict.get(poi_name))

    def get_random(self):
        key = random.choice(list(self.poi_dict.keys()))
        print("Destination: ",key)
        return (self.poi_dict.get(key))


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.poi = PointsOfInterest()

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
                    local = self.local_position
                    local[2] = -1 * local[2]
                    #distance to waypoint
                    if np.linalg.norm(self.target_position[0:3] - local[0:3]) < 7.0:
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
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # DONE: read lat0, lon0 from colliders into floating point values
        lat_lon = []
        with open('colliders.csv') as f:
            lat_lon = (f.readline()).split(",")
        lat0 = float(re.findall(r'-?\d+\.?\d*', lat_lon[0])[1])
        lon0 = float(re.findall(r'-?\d+\.?\d*', lat_lon[1])[1])
        print("Lat: {0}, Lon: {1}".format(lat0,lon0))

        
        # DONE: set home position to (lon0, lat0, 0)
        self.set_home_position = (lon0, lat0, 0)

        # DONE: retrieve current global position
        # DONE: convert to current local position using global_to_local()
        start_pos = global_to_local(self.global_position,self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        # NOTE: Creating grid with edges to enable graph search below
        grid, north_offset, east_offset, edges = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

        print("found {} edges. Creating graph... ".format(len(edges)))
        G = create_graph_from_edges(edges)
        print("Graph created")

        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # DONE: convert start position to current position rather than map center
        grid_start = (int(start_pos[0]) - north_offset, int(start_pos[1]) - east_offset)

        # DONE: adapt to set goal as latitude / longitude position and convert
        # Note use: self.poi.get_random() for a random POI
        goal = global_to_local(self.poi.get("Washington Street"), self.global_home)
        grid_goal = (int(goal[0]) - north_offset, int(goal[1]) - east_offset)

        # Run A* to find a path from start to goal
        # DONE: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        # NOTE: I've implemented the diagonals but then implemented moved to graph search
        #       I left the diagnonal code in.

        # Figure out the closest node to the the grid start and goal positions
        nodes = np.array(G.nodes)
        node_start = np.linalg.norm(np.array(grid_start) - np.array(nodes), axis=1).argmin()
        node_goal = np.linalg.norm(np.array(grid_goal) - np.array(nodes), axis=1).argmin()
        g_start = tuple(nodes[node_start]) + (self.global_home[2],)
        g_goal =  tuple(nodes[node_goal]) + (-1*(int(goal[2])),)
        print('Local Start and Goal: ', g_start, g_goal)


        print('Starting A*')
        path, _ = a_star_graph(G, heuristic, g_start, g_goal)

        # Add gentle gradient for flying car (especially if goal is higher)
        path = add_altitude_gradient(path, g_start, g_goal)

        # The graph search ends at the clsoest node so I add the last point if there are no obstacles on the way
        if len(path) > 0 and (path[-1] != grid_goal) and not(collision_check(path[-1], g_goal, grid)):
            print("Adding end position")
            path.append(g_goal)
                
        # DONE: prune path to minimize number of waypoints
        # DONE (if you're feeling ambitious): Try a different approach altogether!
        # NOTE: I further prune the path by removing nodes not blocked by obstacles
        #       I take height into consideration so it does have paths that fly over
        #       low obstacles
        path = prune_path(path, grid)
       
        # Convert path to waypoints
        waypoints = [[int(p[0]) + north_offset, int(p[1]) + east_offset, int(p[2]), 0] for p in path]

        #Asjust waypoint headings
        for i in range(1,len(waypoints)):
            wp1 = waypoints[i-1]
            wp2 = waypoints[i]
            wp2[3] = np.arctan2((wp2[1]-wp1[1]), (wp2[0]-wp1[0]))

        print(waypoints)
        # Set self.waypoints
        self.waypoints = waypoints
        # DONE: send waypoints to sim (this is just for visualization of waypoints)
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

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=500)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
