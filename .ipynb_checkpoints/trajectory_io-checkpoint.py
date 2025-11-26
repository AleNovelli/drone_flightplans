from data_containers import Site, ENU, Geodetic, DroneTrajectory
import numpy as np
import utils.qgc_utils as qgc
import utils.litchi_utils as litchi
import utils.mp_utils as mp

import json
import warnings

import pandas as pd



def export_mission_qgc(trajectory, move_speed, scan_speed, n_repeat, savepath=None, safety_waypoints=None, fence=False, add_rth=False):
    
    waypoints=[]
    
    #Set the POI
    waypoints = qgc.append_poi(
            waypoints,
            lat= trajectory.poi.lat,
            lon= trajectory.poi.lon,
            alt= trajectory.poi.alt-trajectory.landing_site.alt,
        )
    
    #set the speed used to reach the first point
    waypoints = qgc.append_speed(
            waypoints,
            speed = move_speed
        )
    
    with open("coords/site_geofence.json", "r") as f:
            data = json.load(f)
            
    #add the safety points
    if safety_waypoints:
        safety_points = data["safety_waypoints"][safety_waypoints]
        for safety_point in safety_points:
            waypoints = qgc.append_waypoint(
                waypoints,
                lat=safety_point[0],
                lon=safety_point[1],
                alt=safety_point[2],
            )
    else:
        warnings.warn("No safety points specified!\nUse safety_waypoints='south'/'north'/etc", UserWarning)
        
    
    #create the arc and it's repetitions
    drone_traj = trajectory.geodetic.tolist()
    drone_traj+=drone_traj[1:-1][::-1]
    drone_traj*=n_repeat
    
    #add the first waypoint
    waypoints = qgc.append_waypoint(
            waypoints,
            lat= drone_traj[0][0],
            lon= drone_traj[0][1],
            alt= drone_traj[0][2]-trajectory.landing_site.alt,
        )
    
    #set the scan speed
    waypoints = qgc.append_speed(
            waypoints,
            speed = scan_speed
        )
    
    #add the rest of the waypoints
    for point in drone_traj[1:]:
        waypoints = qgc.append_waypoint(
                waypoints,
                lat= point[0],
                lon= point[1],
                alt= point[2]-trajectory.landing_site.alt,
            )
    
    #add return to launch
    if add_rth:
        waypoints = qgc.append_rth(waypoints)
    
    #set geofencing
    
    if fence:
        geoFence= data["fences"]
    else:
        geoFence = {"circles": [],
            "polygons": [],
            "version": 2
        }
    
    mission_plan = {
        "fileType": "Plan",
        "geoFence": geoFence,
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": 0,
            "firmwareType": 12,
            #"hoverSpeed": scan_speed,
            "items": waypoints,
            "plannedHomePosition":[trajectory.landing_site.lat, trajectory.landing_site.lon, trajectory.landing_site.alt],
            "vehicleType": 2,
            "version": 2
        },
        "rallyPoints": {
            "points": [],
            "version": 2
        },
        "version": 1
    }
    
    #dump json
    if savepath:
        with open(savepath, 'w') as f:
            json.dump(mission_plan, f, indent=4) 
    
    return mission_plan


def export_mission_litchi(trajectory, move_speed, scan_speed, n_repeat, savepath=None, safety_waypoints=None):
    
    columns = ["latitude", "longitude", "altitude(m)", "heading(deg)", "curvesize(m)", "rotationdir", "gimbalmode", "gimbalpitchangle", "actiontype1", "actionparam1", "altitudemode", "speed(m/s)", "poi_latitude", "poi_longitude", "poi_altitude(m)", "poi_altitudemode"]

    waypoints=[]
    
    
    #add the safety points
    if safety_waypoints:
        safety_points =litchi.litchi_safety_points(
            direction =safety_waypoints, 
            speed = move_speed,
            curvesize=trajectory.curveradius,
            rotationdir=0,
            gimbalmode= 1,
            gimbalpitchangle = trajectory.pitch[0],
            actiontype1=-1,
            actionparam1=0,
            altitudemode=0,
            poi_lat=trajectory.poi.lat,
            poi_lon=trajectory.poi.lon,
            poi_alt=trajectory.poi.alt-trajectory.landing_site.alt,
            poi_altitudemode=0
        )
        for safety_point in safety_points:
            waypoints.append(safety_point)
    
    #create the arc and it's repetitions
    drone_traj = np.concatenate([trajectory.geodetic, 
                                 trajectory.yaw.reshape(-1,1), 
                                 trajectory.pitch.reshape(-1,1)], axis=1)
    drone_traj = drone_traj.tolist()
    drone_traj+=drone_traj[1:-1][::-1]
    drone_traj*=n_repeat
    
    for i in range(len(drone_traj)):
        waypoints.append( litchi.litchi_waypoint(
            drone_traj[i][0],
            drone_traj[i][1],
            drone_traj[i][2]-trajectory.landing_site.alt,
            drone_traj[i][3],
            trajectory.curveradius,
            drone_traj[i][4],
            scan_speed,
            trajectory.poi.lat,
            trajectory.poi.lon,
            trajectory.poi.alt-trajectory.landing_site.alt,
        ) ) 
    
    if len(waypoints)>=100:
        raise RuntimeError("Litchi flightplans can't have more than 100 waypoints!")
        return None
    
    df = pd.DataFrame(data=waypoints, columns=columns)
    
    if savepath:
        df.to_csv(savepath, index=False)
        
    return df
    
    
def export_mission_mp(trajectory, move_speed, scan_speed, n_repeat, savepath=None, safety_waypoints=None,  add_rth=False):
    
    waypoints=""
    
    waypoints+=mp.header()
    seq=0
    
    # set home position (will be overwritten at take-off)
    waypoints+=mp.waypoint(
        seq,
        trajectory.landing_site.lat,
        trajectory.landing_site.lon,
        trajectory.landing_site.alt,
        frame=0,
        current=0,
    )
    seq+=1
        
        
    #set the poi
    waypoints+=mp.roi(
        seq=seq,
        lat=trajectory.poi.lat,
        lon=trajectory.poi.lon,
        alt=trajectory.poi.alt-trajectory.landing_site.alt
    )
    seq+=1
    
    #set the speed
    waypoints+=mp.speed(
        seq=seq,
        speed_value=move_speed,
    )
    seq+=1
    
    #add the safety points
    if safety_waypoints:
        with open("coords/site_geofence.json", "r") as f:
            data = json.load(f)
        safety_points = data["safety_waypoints"][safety_waypoints]
        for safety_point in safety_points:
            waypoints+=mp.waypoint(
                seq,
                safety_point[0],
                safety_point[1],
                safety_point[2],
            )
            seq+=1
    else:
        warnings.warn("No safety points specified!\nUse safety_waypoints='south'/'north'/etc", UserWarning)
    
    #create the arc and it's repetitions
    drone_traj = trajectory.geodetic.tolist()
    drone_traj+=drone_traj[1:-1][::-1]
    drone_traj*=n_repeat
    
    
    #go to the first waypoint of the arc
    waypoints+=mp.waypoint(
        seq,
        drone_traj[0][0],
        drone_traj[0][1],
        drone_traj[0][2]-trajectory.landing_site.alt,
    )
    seq+=1
    
    
    #set the speed
    waypoints+=mp.speed(
        seq=seq,
        speed_value=scan_speed,
    )
    seq+=1
    
    #finish the arc
    for point in drone_traj[1:]:
        waypoints+=mp.waypoint(
            seq,
            point[0],
            point[1],
            point[2]-trajectory.landing_site.alt,
        )
        seq+=1
    
    if add_rth:
        waypoints+=mp.rth(seq)
        seq+=1
    
    if savepath:
        with open(savepath, 'w') as f:
            f.write(waypoints)
        
    return waypoints

def export_test_mission_mp_switching_pois(landing_site, poi1, poi2, savepath = None, n_repeat=3):
    waypoints=""
    
    waypoints+=mp.header()
    seq=0
    
    # set home position (will be overwritten at take-off)
    waypoints+=mp.waypoint(
        seq,
        landing_site.lat,
        landing_site.lon,
        landing_site.alt,
        frame=0,
        current=0,
    )
    seq+=1

    #set the speed
    waypoints+=mp.speed(
        seq=seq,
        speed_value=1,
    )
    seq+=1

    #go 10 meters above the landing site
    waypoints+=mp.waypoint(
        seq,
        landing_site.lat,
        landing_site.lon,
        10,
    )
    seq+=1

    #wait 10 seconds
    waypoints+=mp.delay(
        seq,
        time_s=10
    )
    seq+=1

    #set the poi on the ground
    waypoints+=mp.roi(
        seq=seq,
        lat=landing_site.lat,
        lon=landing_site.lon,
        alt=0
    )
    seq+=1

    #wait 10 seconds
    waypoints+=mp.delay(
        seq,
        time_s=10
    )
    seq+=1

    for i in range(n_repeat):
        #set the poi on poi1
        waypoints+=mp.roi(
            seq=seq,
            lat=poi1.lat,
            lon=poi1.lon,
            alt=poi1.alt-landing_site.alt
        )
        seq+=1
    
        #wait 10 seconds
        waypoints+=mp.delay(
            seq,
            time_s=10
        )
        seq+=1
    
        #set the poi on poi2
        waypoints+=mp.roi(
            seq=seq,
            lat=poi1.lat,
            lon=poi1.lon,
            alt=poi1.alt-landing_site.alt
        )
        seq+=1
    
        #wait 10 seconds
        waypoints+=mp.delay(
            seq,
            time_s=10
        )
        seq+=1

    #return to home
    waypoints+=mp.rth(seq)
    seq+=1
    
    if savepath:
        with open(savepath, 'w') as f:
            f.write(waypoints)
        
    return waypoints
    

    