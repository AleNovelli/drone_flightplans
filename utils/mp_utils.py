#https://ardupilot.org/planner/docs/common-mavlink-mission-command-messages-mav_cmd.html
#https://docs.rs/mavlink/latest/mavlink/common/enum.MavCmd.html
MAVLINK_COMMANDS = {
        "waypoint": {"cmd": 16, "altmode": 1, "frame": 3},
        "roi": {"cmd": 195, "altmode": 1, "frame": 3},
        "rth": {"cmd": 20, "altmode": 3, "frame": 2},
        "speed": {"cmd": 178, "altmode": None, "frame": 2},
        "delay": {"cmd": 19, "altmode": None, "frame":2 }
    }

def header():
    return "QGC WPL 110\n"
    
def make_item(
    seq: int,
    current: int,
    frame: int,
    command: int,
    param1: float = 0.,
    param2: float = 0.,
    param3: float = 0.,
    param4: float = 0.,
    x: float = 0.,
    y: float = 0.,
    z: float = 0.,
    autocontinue: int = 1
):
    return (
        f"{seq}\t"
        f"{current}\t"
        f"{frame}\t"
        f"{command}\t"
        f"{param1:.6f}\t"
        f"{param2:.6f}\t"
        f"{param3:.6f}\t"
        f"{param4:.6f}\t"
        f"{x:.8f}\t"
        f"{y:.8f}\t"
        f"{z:.6f}\t"
        f"{autocontinue}\n"
    )

def waypoint (seq, lat, lon, alt, current=0 ,autocontinue=1, frame=None):
    cmd_info = MAVLINK_COMMANDS["waypoint"]
    if not frame:
        frame = cmd_info["frame"]
    return make_item(
        seq, 
        current, 
        frame,
        cmd_info["cmd"],
        0,
        0,
        0,
        0,
        lat,
        lon,
        alt,
        autocontinue
    )

def rth(seq: int, current: int = 0, autocontinue: int = 1) -> str:
    """Create a Return-to-Launch mission item."""
    cmd_info = MAVLINK_COMMANDS["rth"]
    return make_item(
        seq=seq,
        current=current,
        frame=cmd_info["frame"],
        command=cmd_info["cmd"],
        param1=0,
        param2=0,
        param3=0,
        param4=0,
        x=0,
        y=0,
        z=0,
        autocontinue=autocontinue
    )

def speed(seq: int, speed_value: float, current: int = 0, autocontinue: int = 1) -> str:
    """Create a speed change mission item."""
    cmd_info = MAVLINK_COMMANDS["speed"]
    # Standard param mapping: param1=type (1 = airspeed), param2=value, param3=-1 (unused)
    return make_item(
        seq=seq,
        current=current,
        frame=cmd_info["frame"],
        command=cmd_info["cmd"],
        param1=0,
        param2=speed_value,
        param3=0,
        param4=0,
        x=0,
        y=0,
        z=0,
        autocontinue=autocontinue
    )

def roi(seq: int, lat: float, lon: float, alt: float, current: int = 0, autocontinue: int = 1) -> str:
    """Create a ROI (Region of Interest) mission item."""
    cmd_info = MAVLINK_COMMANDS["roi"]
    return make_item(
        seq=seq,
        current=current,
        frame=cmd_info["frame"],
        command=cmd_info["cmd"],
        param1=0,
        param2=0,
        param3=0,
        param4=0,
        x=lat,
        y=lon,
        z=alt,
        autocontinue=autocontinue
    )


def delay(seq: int, time_s: float, current: int = 0, autocontinue: int = 1) -> str:
    """Create a speed change mission item."""
    cmd_info = MAVLINK_COMMANDS["delay"]
    return make_item(
        seq=seq,
        current=current,
        frame=cmd_info["frame"],#unused
        command=cmd_info["cmd"],
        param1=time_s,
        param2=0,
        param3=0,
        param4=0,
        x=0,
        y=0,
        z=0,
        autocontinue=autocontinue
    )