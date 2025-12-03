from dataclasses import dataclass
from typing import Optional, Dict, Union
import json
import numpy as np
import pymap3d as pm
import warnings

@dataclass
class Geodetic:
    lat: float
    lon: float
    alt: float
    def as_array(self):
        return np.array([self.e, self.n, self.u])
    
@dataclass
class ECEF:
    x: float
    y: float
    z: float

@dataclass
class ENU:
    e: float
    n: float
    u: float
    def __sub__(self, other: "ENU") -> "ENU":
        if not isinstance(other, ENU):
            return NotImplemented
        return ENU(self.e - other.e, self.n - other.n, self.u - other.u)

    def __add__(self, other: "ENU") -> "ENU":
        if not isinstance(other, ENU):
            return NotImplemented
        return ENU(self.e + other.e, self.n + other.n, self.u + other.u)

    def as_array(self):
        return np.array([self.e, self.n, self.u])

@dataclass
class Telescope:
    name: str
    geodetic: Geodetic
    focalplane_height: float = 0.0
    ecef: Optional[ECEF] = None
    enu: Optional[ENU] = None

@dataclass
class DroneTrajectory:
    enu: np.ndarray  # shape (N, 3), efficient and compact
    yaw: np.ndarray
    pitch: np.ndarray
    npoints: int
    arccenter: Optional[ENU] = None
    geodetic: Optional[np.ndarray] = None
    poi: Optional[Geodetic] = None
    landing_site: Optional[Geodetic]=None
    curveradius: Optional[float]=None
    plot_boresight: Optional[bool]=False
    
    @property
    def E(self):
        return self.enu[:, 0]

    @property
    def N(self):
        return self.enu[:, 1]

    @property
    def U(self):
        return self.enu[:, 2]
    
    @property
    def LAT(self):
        if self.geodetic is None:
            raise ValueError("Geodetic coordinates not computed yet. Call compute_geodetic(site) first.")
        return self.geodetic[:, 0]

    @property
    def LON(self):
        if self.geodetic is None:
            raise ValueError("Geodetic coordinates not computed yet. Call compute_geodetic(site) first.")
        return self.geodetic[:, 1]

    @property
    def ALT(self):
        if self.geodetic is None:
            raise ValueError("Geodetic coordinates not computed yet. Call compute_geodetic(site) first.")
        return self.geodetic[:, 2]
    
    def compute_geodetic(self, site: "Union[Site, Geodetic]"):
        
        if isinstance(site, Site):
            if site.origin is None:
                raise ValueError("Site origin must be set to convert ENU → geodetic.")
            origin=site.origin
        elif isinstance(site, Geodetic):
            origin=site
        else:
            raise TypeError("Site origin mist be eiter Site or Geodetic")
            
        lats, lons, alts = pm.enu2geodetic(
            self.E, self.N, self.U,
            origin.lat, origin.lon, origin.alt
        )

        # store vectorized array
        self.geodetic = np.column_stack([lats, lons, alts])
        
    def export_kml(self, path: str):
        """
        Export the trajectory to a KML file using simplekml.
        Requires that compute_geodetic() has been called first.
        """
        try:
            import simplekml
        except ImportError:
            raise ImportError("simplekml is not installed. Install it with: pip install simplekml")
    
        if self.geodetic is None:
            raise ValueError("Geodetic coordinates not computed. Call compute_geodetic(site) first.")
    
        kml = simplekml.Kml()
    
        # ---- Add the trajectory path ----
        coords = [(lon, lat, alt) for lat, lon, alt in self.geodetic]
    
        lin = kml.newlinestring(
            name="Drone Trajectory",
            coords=coords,
        )
        #lin.altitudemode = simplekml.AltitudeMode.absolute
        #lin.extrude = 1
        lin.style.linestyle.width = 3

        """
        # ---- Add point markers (optional) ----
        for i, (lat, lon, alt) in enumerate(self.geodetic):
            pnt = kml.newpoint(
                name=f"Point {i}",
                coords=[(lon, lat, alt)]
            )
            pnt.altitudemode = simplekml.AltitudeMode.absolute
            pnt.style.labelstyle.scale = 0.6
        """
    
        # ---- Save file ----
        kml.save(path)
    

class Site:
    def __init__(self, path: str):
        #load telescope positions from a JSON file
        with open(path) as f:
                data = json.load(f)
        
        # Convert telescope dictionaries into Telescope objects
        self.telescopes: Dict[str, Telescope] = {}
        
        telescopes= data["telescopes"]
        for name in telescopes.keys():
            
            self.telescopes[name] = Telescope(
                 name=name,
                 geodetic=Geodetic(
                    lat=telescopes[name]["lat"],
                    lon=telescopes[name]["lon"],
                    alt=telescopes[name]["alt"],
                    ),
                 focalplane_height = telescopes[name].get("focalplane_height", 0.0)
            )

        # Optional landing site
        ls = data.get("landing_site")
        self.landing_site: Optional[Geodetic] = Geodetic(**ls) if ls else None

        # Optional origin (can be set later, e.g., barycenter)
        origin = data.get("origin")
        self.origin: Optional[Geodetic] = Geodetic(**origin) if origin else None
        
    
    def set_origin(self, geodetic: Optional[Geodetic] = None):
        """Set the site origin and update ENU coordinates of all telescopes."""
        
        if geodetic is None:
            warnings.warn("No origin provided: assuming the barycenter of all telescopes as origin.", UserWarning)
            geodetic = self.compute_barycenter()
        
        self.origin = geodetic
        self._update_all_enu()
        
        
    def _update_all_enu(self):
        """Compute ENU coordinates for all telescopes relative to the site origin."""
        if self.origin is None:
            raise ValueError("Origin must be set before computing ENU coordinates.")

        for t in self.telescopes.values():
            e, n, u = pm.geodetic2enu(
                lat=t.geodetic.lat,
                lon=t.geodetic.lon,
                h=t.geodetic.alt + t.focalplane_height,
                lat0=self.origin.lat,
                lon0=self.origin.lon,
                h0=self.origin.alt,
            )
            t.enu = ENU(e=e, n=n, u=u)
            
    def compute_barycenter(self):
        """Compute the geodetic barycenter of all telescopes (including focal heights) using NumPy."""
        if not self.telescopes:
            raise ValueError("No telescopes defined in the site.")
            
        ecef_coords_list = []
        for t in self.telescopes.values():
            x, y, z = pm.geodetic2ecef(
                lat=t.geodetic.lat,
                lon=t.geodetic.lon,
                alt=t.geodetic.alt + t.focalplane_height
            )
            ecef_coords_list.append([x, y, z])
        ecef_coords = np.array(ecef_coords_list)  

        # Compute mean of X, Y, Z
        Xc, Yc, Zc = np.mean(ecef_coords, axis=0)

        # Convert back to geodetic
        lat, lon, alt = pm.ecef2geodetic(Xc, Yc, Zc)

        return Geodetic(lat=lat, lon=lon, alt=alt)
        
    def geodetic_to_enu(self, geo):
    
        if self.origin is None:
            raise ValueError("Origin must be set before converting to ENU.")
    
        # ---------------------------------------------------------
        # Case 1: single object
        # ---------------------------------------------------------
        if isinstance(geo, Geodetic):
            e, n, u = pm.geodetic2enu(
                lat=geo.lat,
                lon=geo.lon,
                h=geo.alt,
                lat0=self.origin.lat,
                lon0=self.origin.lon,
                h0=self.origin.alt
            )
            return ENU(e=e, n=n, u=u)
    
        # ---------------------------------------------------------
        # Case 2: array of shape (N,3)
        # ---------------------------------------------------------
        if isinstance(geo, np.ndarray):
            if geo.ndim != 2 or geo.shape[1] != 3:
                raise ValueError("NumPy input must have shape (N,3) as (lat, lon, alt).")
    
            lat = geo[:, 0]
            lon = geo[:, 1]
            alt = geo[:, 2]
    
            e, n, u = pm.geodetic2enu(
                lat=lat,
                lon=lon,
                h=alt,
                lat0=self.origin.lat,
                lon0=self.origin.lon,
                h0=self.origin.alt
            )
            return np.column_stack([e, n, u])
    
        raise TypeError("Input must be a Geodetic object or a NumPy array of shape (N,3).")

    def enu_to_geodetic(self, enu):
        """
        Convert a single ENU position or an array of ENU points
        back to Geodetic coordinates.
    
        Parameters
        ----------
        enu : ENU | np.ndarray
            - ENU(e, n, u)
            - OR NumPy array of shape (N,3)
    
        Returns
        -------
        Geodetic | np.ndarray
            - Geodetic(lat, lon, alt) for single input
            - np.ndarray of shape (N,3) for batch input
        """
        if self.origin is None:
            raise ValueError("Origin must be set before converting to Geodetic.")
    
        # ---------------------------------------------------------
        # Case 1: single ENU object
        # ---------------------------------------------------------
        if isinstance(enu, ENU):
            lat, lon, alt = pm.enu2geodetic(
                e=enu.e,
                n=enu.n,
                u=enu.u,
                lat0=self.origin.lat,
                lon0=self.origin.lon,
                h0=self.origin.alt
            )
            return Geodetic(lat=lat, lon=lon, alt=alt)
    
        # ---------------------------------------------------------
        # Case 2: array of shape (N,3)
        # ---------------------------------------------------------
        if isinstance(enu, np.ndarray):
            if enu.ndim == 2 and enu.shape[1] == 3:
                
                e = enu[:, 0]
                n = enu[:, 1]
                u = enu[:, 2]
        
                lat, lon, alt = pm.enu2geodetic(
                    e=e,
                    n=n,
                    u=u,
                    lat0=self.origin.lat,
                    lon0=self.origin.lon,
                    h0=self.origin.alt
                )
                return np.column_stack([lat, lon, alt])

            elif enu.ndim == 0 and enu.shape[0] == 3:
                
                e = enu[0]
                n = enu[1]
                u = enu[2]
        
                lat, lon, alt = pm.enu2geodetic(
                    e=e,
                    n=n,
                    u=u,
                    lat0=self.origin.lat,
                    lon0=self.origin.lon,
                    h0=self.origin.alt
                )
                return np.column_stack([lat, lon, alt])

            else:
                raise ValueError("NumPy input must have shape (N,3) as (e, n, u).")
    
    
        raise TypeError("Input must be an ENU object or a NumPy array of shape (N,3).")


    def observe_points(self,
                   points_geodetic,
                   telescope: Union[str, Telescope, Geodetic, ENU]) -> np.ndarray:
        """
        Compute (az, el, srange) under which a telescope observes a sequence of points.
        
        Parameters
        ----------
        points : list or np.ndarray
            Either a list/array of Geodetic or ENU objects, OR a NumPy array
            of shape (N,3) interpreted as (lat,lon,alt) or (e,n,u).
        telescope : str | Telescope | Geodetic | ENU
            Telescope selector. Can be:
            - telescope name (string),
            - a Telescope object,
            - a Geodetic position,
            - an ENU position.
    
        Returns
        -------
        np.ndarray
            Array of shape (N,3): (azimuth_deg, elevation_deg, slant_range_m)
        """
        # -----------------------------
        # Resolve telescope position
        # -----------------------------
        if isinstance(telescope, str):
            if telescope not in self.telescopes:
                raise ValueError(f"Telescope '{telescope}' not found.")
            tel_enu = self.telescopes[telescope].enu
            if tel_enu is None:
                raise ValueError("Telescope ENU positions are not computed. Call site.set_origin().")
    
        elif isinstance(telescope, Telescope):
            if telescope.enu is None:
                raise ValueError("Telescope ENU not defined. Call site.set_origin().")
            tel_enu = telescope.enu
    
        elif isinstance(telescope, ENU):
            tel_enu = telescope
    
        elif isinstance(telescope, Geodetic):
            tel_enu = self.geodetic_to_enu(telescope)
    
        else:
            raise TypeError("Invalid type for telescope specification.")
    
        tel_pos = np.array([tel_enu.e, tel_enu.n, tel_enu.u])
    
    
        points_enu = self.geodetic_to_enu(points_geodetic)
    
        dENU = points_enu - tel_pos
    
        az, el, srange = pm.enu2aer(
            e=dENU[:, 0],
            n=dENU[:, 1],
            u=dENU[:, 2],
        )
    
        return np.column_stack([az, el, srange])
    
    
