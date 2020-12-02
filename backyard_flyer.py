import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5
     

class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)


    def local_position_callback(self):
        print ("Local position")
        #print ( self.local_position_callback)

        return 

    def velocity_callback(self):
        
        print ("Local velocity")
        #print ( self.velocity_callback)

        return 

    def state_callback(self):
        #State machine dict
        options = { 
            States.MANUAL :   self.arming_transition,
            States.ARMING :   self.takeoff_transition,
            States.TAKEOFF :  self.waypoint_transition,
            States.WAYPOINT:  self.landing_transition,
            States.LANDING :  self.disarming_transition,
            States.DISARMING: self.manual_transition
        }
        options[self.flight_state]()
        #time.sleep(2)
        
    def calculate_box(self):
        self.all_waypoints = [(0,10,3,0),(10,10,3,0),(10,0,3,0),(0,0,3,0)]
        return self.all_waypoints 

    def arming_transition(self):

        print("arming transition")
        self.take_control()
        self.arm()
        home_location = self.target_position
        self.flight_state = States.ARMING       

    def takeoff_transition(self):

        print("takeoff transition")
        target_position = 3
        self.takeoff(target_position)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):

        print("waypoint transition")
        self.target_position = self.calculate_box()
        for pos in self.target_position:
            print ("Waypoints are ")
            print (pos)
            self.cmd_position(*pos)
            time.sleep(2.0)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):

        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):

        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):

        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):

        print("Creating log file")
        self.start_log("Logs", "TLog-manual.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60, threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
