#https://www.litchiutilities.com.wesbarris.com/docs/litchiCsv.php

import json

def litchi_safety_points(
    direction:str, 
    speed,
    curvesize=0,
    rotationdir=0,
    gimbalmode=0,
    gimbalpitchangle=0,
    actiontype1=-1,
    actionparam1=0,
    altitudemode=0,
    poi_lat=0,
    poi_lon=0,
    poi_alt=0,
    poi_altitudemode=0
):

    heading = {"south": 0, "north": 180, "east": 270, "west": 90}.get(direction)
    
    with open("coords/site_geofence.json", "r") as f:
            data = json.load(f)
    
    safety_points = data["safety_waypoints"][direction]
    
    waypoints = [
        litchi_waypoint(
            lat, lon, alt,
            heading=heading,
            curvesize=curvesize,
            rotationdir=rotationdir,
            gimbalmode=gimbalmode,
            gimbalpitchangle=gimbalpitchangle,
            actiontype1=actiontype1,
            actionparam1=actionparam1,
            altitudemode=altitudemode,
            speed=speed,
            poi_lat=poi_lat,
            poi_lon=poi_lon,
            poi_alt=poi_alt,
            poi_altitudemode=poi_altitudemode
        )
        for lat, lon, alt in safety_points
    ]
    return waypoints

def litchi_waypoint(
    lat, 
    lon, 
    alt, 
    heading, 
    curvesize, #radius of the curve
    gimbalpitchangle, # Not understood how it interacts with the POI
    speed,
    poi_lat,
    poi_lon,
    poi_alt,
    rotationdir = 0, #(0 = clockwise, 1 = counterclockwise)
    gimbalmode = 1, #(Options: 0 --> Disabled, 1 --> Focus POI, 2 --> Interpolate)
    actiontype1 = -1, #(-1 -> do nothing; 0 -> Stay For; 1 -> Take Photo; 2 -> Start Recording; 3 -> Stop Recording; 4 -> Rotate Aircraft; 5 -> Tilt Camera)
    actionparam1 = 0, #set the parameter of the action (for actiontype=-1 default is 0)
    altitudemode = 0 , #0 is relative to take-off, 1 is relative to ground
    poi_altitudemode = 0, #0 is relative to take-off, 1 is relative to ground
):
    return  [lat, 
             lon, 
             alt, 
             heading, 
             curvesize, 
             rotationdir,
             gimbalmode,
             gimbalpitchangle,
             actiontype1,
             actionparam1,
             altitudemode,
             speed,
             poi_lat,
             poi_lon,
             poi_alt,
             poi_altitudemode,
            ]
        