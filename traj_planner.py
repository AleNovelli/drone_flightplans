import pymap3d as pm
from data_containers import Site, ENU, Geodetic, DroneTrajectory
import numpy as np
import matplotlib.pyplot as plt
import warnings
from typing import List

class TrajectoryPlanner:
    
    def __init__(self, site: Site):
        self.site = site
        if self.site.origin is None:
            self.site.set_origin()
    
    def telescope_to_target_aer(self, telescope, target_enu):
        """
        Compute AER (az, el, range) from telescope to a target ENU position.
        telescope : Telescope object from `self.site.telescopes[...]`
        target_enu : ENU object or array of ENU coordinates
                     - ENU instance → single point
                     - (N,3) array → multiple points
        """
        if isinstance(target_enu, ENU):
            return pm.enu2aer(
                target_enu.e - telescope.enu.e,
                target_enu.n - telescope.enu.n,
                target_enu.u - telescope.enu.u
            )
        elif isinstance(target_enu, np.ndarray):
            return pm.enu2aer(
                target_enu[:,0] - telescope.enu.e,
                target_enu[:,1] - telescope.enu.n,
                target_enu[:,2] - telescope.enu.u
            )
        else:
            raise TypeError("target_enu must be a ENU, or ndarray")
        
        
    def compute_boresight(self, telescope, drone_traj):
        """
        Computes the az, el of the arc center as seen by the telescope.
        Falls back to trajectory mean if arccenter is missing.
        """
        if drone_traj.arccenter:
            arcc = drone_traj.arccenter
        else:
            e,n,u = drone_traj.enu.mean(axis=0)
            arcc = ENU(e=e, n=n, u=u)

        return self.telescope_to_target_aer(telescope, arcc)
    
    def old_arc_trajectory_202404(self,
                              poi_enu: ENU,
                              slant_range: float,
                              az_center: float,
                              el_center: float,
                              el_range: float,
                              num_steps_el: int,
                             ):
        el_vals = np.linspace(el_center - el_range/2, el_center + el_range/2, num_steps_el) 
            
        enu_points = np.zeros((num_steps_el, 3))

        e, n, u = pm.aer2enu(az_center, el_vals, slant_range)
        enu_points[:, 0] = e + poi_enu.e
        enu_points[:, 1] = n + poi_enu.n
        enu_points[:, 2] = u + poi_enu.u
        
        #Compute yaw/pitch to see the POI
        yaw, pitch, _ = pm.enu2aer(
            -enu_points[:,0] + poi_enu.e,
            -enu_points[:,1] + poi_enu.n,
            -enu_points[:,2] + poi_enu.u
        )

        trajectory = DroneTrajectory(enu=enu_points, 
                                     yaw=yaw, 
                                     pitch=pitch, 
                                     npoints=num_steps_el, 
                                     poi=self.site.enu_to_geodetic(poi_enu),
                                     curveradius=slant_range
                                    )
        trajectory.compute_geodetic(self.site)
        trajectory.landing_site = self.site.landing_site
        return trajectory
    
    def new_arc_trajectory_202412(self,
                                  # nominal values are used to compute the arc-center 
                                  # (same configuration as old trajectory)
                                  nominal_poi: ENU,
                                  nominal_az: float,
                                  nominal_el: float,
                                  nominal_srange: float,
                                  # actual POI with respect to which we move
                                  poi: ENU,
                                  delta_el: float,
                                  num_steps_el: int,
                                 ):
        
        #Compute arc center ENU relative to nominal POI
        arccenter_e, arccenter_n, arccenter_u = pm.aer2enu(
            az=nominal_az,
            el=nominal_el,
            srange=nominal_srange
        )
        arccenter_e += nominal_poi.e
        arccenter_n += nominal_poi.n
        arccenter_u += nominal_poi.u
        
        arccenter = ENU(e=arccenter_e, n=arccenter_n, u=arccenter_u)         
        
        # we find the center az,el position of the drone as seen by the poi
        az0, el0, srange0 = pm.enu2aer(
                arccenter_e-poi.e, 
                arccenter_n-poi.n,
                arccenter_u-poi.u
                )
        
        #we move on an arc by modulating the el
        el_vals = np.linspace(el0 - delta_el/2, el0 + delta_el/2, num_steps_el)
        
        enu_points = np.zeros((num_steps_el, 3))
        # ENU position relative to POI
        e, n, u = pm.aer2enu(az=az0, el=el_vals, srange=srange0)
        
        enu_points[:, 0] = e + poi.e
        enu_points[:, 1] = n + poi.n
        enu_points[:, 2] = u + poi.u

        # Compute yaw/pitch to see the POI
        yaw, pitch, _ = pm.enu2aer(
            -enu_points[:,0] + poi.e,
            -enu_points[:,1] + poi.n,
            -enu_points[:,2] + poi.u
        )
        
        trajectory = DroneTrajectory(
            enu=enu_points,
            yaw=yaw,
            pitch=pitch,
            npoints=num_steps_el,
            arccenter=arccenter,
            poi=self.site.enu_to_geodetic(poi),
            curveradius = srange0
        )
        trajectory.compute_geodetic(self.site)
        trajectory.landing_site = self.site.landing_site
        return trajectory
        
    def plot_trajectories(
            self,
            trajectories,
            telescopes: List[str] = None,
            title: str = "",
            boresight_table: bool=False,
            save_path=None,
        ):
        """
        Plot one or multiple drone trajectories as seen by telescopes.
        Accepts:
            - a single DroneTrajectory
            - a list of DroneTrajectory
            - a dict {name: DroneTrajectory}
        """

        #Normalize the trajectories into a dictionary
        if isinstance(trajectories, DroneTrajectory):
            # single trajectory
            traj_dict = {"trajectory_0": trajectories}
        elif isinstance(trajectories, list):
            # list → auto-name
            traj_dict = {
                f"trajectory_{i}" : t for i, t in enumerate(trajectories)
            }
        elif isinstance(trajectories, dict):
            # dict: assume keys are names
            traj_dict = trajectories
        else:
            raise TypeError("trajectories must be a DroneTrajectory, list, or dict")

        #Load all telescopes from the site if not specified
        if telescopes is None:
            telescopes = list(self.site.telescopes.keys())
        
        if boresight_table:
            if len(list(traj_dict.items()))*len(telescopes)>20:
                    warnings.warn("Plotting table with more than 20 entries.\nPlot might be malformed.", UserWarning)
            fig, axes = plt.subplots(1, 2, figsize=(18, 7), gridspec_kw={'width_ratios': [3, 1]})
            ax1, ax2 = axes
        else:
            fig, axes = plt.subplots(1, 1, figsize=(12, 7),)
            ax1 = axes

        azel_summary = []
        markers = ["o", "s", "D", "^", "v", "*", "P", "X",]

        #Loop over trajectories
        for j, (traj_name, drone_traj) in enumerate(traj_dict.items()):

            for i, telescope_name in enumerate(telescopes):
                tel = self.site.telescopes[telescope_name]

                # az/el from telescope to drone
                az, el, srange = self.telescope_to_target_aer(tel,drone_traj.enu)

                # find boresight (boresight is the arccenter as seen by the telescope)
                az0, el0, _ = self.compute_boresight(tel, drone_traj)

                if drone_traj.plot_boresight:
                    azel_summary.append([telescope_name, traj_name, f"{az0:.2f}", f"{el0:.2f}"])

                # plot
                #ax1.plot(az, el, linestyle = "-", marker=markers[j%len(markers)], color=f"C{i}", label=f"{traj_name} – {telescope_name}")
                ax1.plot(az, el, linestyle = "-", color=f"C{i}", label=f"{traj_name} – {telescope_name}")
                if drone_traj.plot_boresight:
                    ax1.scatter(az0, el0, marker="x", color="black")

        ax1.grid()
        ax1.set_title(title)
        ax1.set_xlabel("Drone azimuth [deg]")
        ax1.set_ylabel("Drone elevation [deg]")


        # Table
        if boresight_table and len(azel_summary)>0:
            columns = ["Telescope", "Trajectory", "boresight_az [°]", "boresight_el [°]"]
            azel_summary = sorted(azel_summary, key=lambda row: row[0])
            table = ax2.table(cellText=azel_summary, colLabels=columns, loc='upper center')
            table.auto_set_font_size(False)
            table.set_fontsize(11)
            table.scale(1.2, 1.5)
            ax2.axis('off')
            ax2.set_title("Boresight pointing", fontsize=12)

        plt.tight_layout()
        if save_path:
            plt.savefig(save_path)
        plt.show()