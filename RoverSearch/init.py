# Paul Kudyba, OCT-2023

# this holds initializations for the rover search profile
# it holds constants and waypoints for scripted search patterns

from aerpawlib.util import Coordinate, VectorNED

class TrialInit:
    def __init__(self) -> None:
        # CONSTANTS
        self.STEP_SIZE = 40  # when going forward - how far, in meters (to be deprecated?)
        self.NORTHWEST = 315 # azimuth in degrees
        self.SEARCH_ALTITUDE = 35 # in meters

        self.MEASUREMENT_SPAN = 20 # in meters

        # waypoints
        self.waypoints = []
        #waypoint1
        self.waypoints.append(Coordinate(lat=35.727753, lon=-78.696723, alt=self.SEARCH_ALTITUDE))
        # #waypoint2
        # self.waypoints.append(Coordinate(lat=35.727764, lon=-78.697129, alt=self.SEARCH_ALTITUDE))
        # #waypoint3
        # self.waypoints.append(Coordinate(lat=35.728132, lon=-78.696719, alt=self.SEARCH_ALTITUDE))

    def get_next_waypoint(self) -> Coordinate or None:
        '''returns (pops) the next waypoint in the list,
        if the list is empty, returns None'''
        return self.waypoints.pop(0) if len(self.waypoints)>0 else None
        