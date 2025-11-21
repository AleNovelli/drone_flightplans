# https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/file_formats/plan.html

from typing import List, Optional

MAVLINK_COMMANDS = {
        "waypoint": {"cmd": 16, "altmode": 1, "frame": 3},
        "roi": {"cmd": 195, "altmode": 1, "frame": 3},
        "rth": {"cmd": 20, "altmode": 3, "frame": 2},
        "speed": {"cmd": 178, "altmode": None, "frame": 2},
        "delay": {"cmd": 19, "altmode": None, "frame":2 }
    }

def mission_item(
    doJumpId: int,
    command: int,
    frame: int,
    params: List[float],
    autocontinue: bool=True,
    type: str="SimpleItem",
    ):
    return {
        "doJumpId":doJumpId,
        "command":command,
        "frame":frame,
        "params":params,
        "autoContinue":autocontinue,
        "type":type,
        "AMSLAltAboveTerrain": None,
    }   

def qgc_waypoint(
        doJumpId: int,
        lat: float,
        lon: float,
        alt: float,
    ):
    mav = MAVLINK_COMMANDS["waypoint"]
    
    waypoint = mission_item(
        doJumpId = doJumpId,
        command = mav["cmd"],
        frame = mav["frame"],
        params = [0, 0, 0, 0, lat, lon, alt],
                      )
    waypoint["Altitude"] = alt
    waypoint["AltitudeMode"]= mav["altmode"]
    
    return waypoint

def qgc_poi(
    doJumpId: int,
    lat: float,
    lon: float,
    alt: float,
):
    
    mav = MAVLINK_COMMANDS["roi"]
    
    item = mission_item(
        doJumpId=doJumpId,
        command=mav["cmd"],
        frame=mav["frame"],
        params=[0, 0, 0, 0, lat, lon, alt],
    )
    item["Altitude"] = alt
    item["AltitudeMode"] = mav["altmode"]
    return item

def qgc_speed(
    doJumpId: int, 
    speed: float
):
    mav = MAVLINK_COMMANDS["speed"]
    
    item = mission_item(
        doJumpId=doJumpId,
        command=mav["cmd"],
        frame=mav["frame"],
        params=[1, speed, -1, 0, 0, 0, 0],  # standard speed params
    )
    return item


def qgc_rth(doJumpId: int):
    """Create a Return-To-Launch mission item."""
    mav = MAVLINK_COMMANDS["rth"]
    item = mission_item(
        doJumpId=doJumpId,
        command=mav["cmd"],
        frame=mav["frame"],
        params=[0] * 7,  # RTH params are unused
    )
    return item

def qgc_delay(
    doJumpId: int,
    time_s:float,
    ):
    mav = MAVLINK_COMMANDS["delay"]
    
    item = mission_item(
        doJumpId = doJumpId,
        command = mav["cmd"],
        frame = mav["frame"], #unused
        params = [time_s, 0, 0, 0, 0, 0, 0],
    )
    
    return item 



def append_waypoint(
        waypoints: List,
        lat: float,
        lon: float,
        alt: float,
    ):
    doJumpId = len(waypoints)+1
    
    waypoint = qgc_waypoint(
        doJumpId=doJumpId,
        lat=lat,
        lon=lon,
        alt=alt,
        )
    waypoints.append(waypoint)
    return waypoints


def append_poi(
        waypoints: List,
        lat: float,
        lon: float,
        alt: float,
    ):
    doJumpId = len(waypoints)+1
    
    waypoint = qgc_poi(
        doJumpId=doJumpId,
        lat=lat,
        lon=lon,
        alt=alt,
        )
    waypoints.append(waypoint)
    return waypoints

def append_speed(
        waypoints: List,
        speed: float
    ):
    doJumpId = len(waypoints)+1
    
    waypoint = qgc_speed(
        doJumpId=doJumpId,
        speed=speed,
        )
    waypoints.append(waypoint)
    return waypoints

def append_rth(
        waypoints: List,
    ):
    doJumpId = len(waypoints)+1
    
    waypoint = qgc_rth(
        doJumpId=doJumpId,
        )
    waypoints.append(waypoint)
    return waypoints

def append_delay(
        waypoints: List,
        time_s: float
    ):
    doJumpId = len(waypoints)+1
    
    waypoint = qgc_delay(
        doJumpId=doJumpId,
        time_s=time_s,
        )
    waypoints.append(waypoint)
    return waypoints