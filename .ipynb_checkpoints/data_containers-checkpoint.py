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
        
        
    def geodetic_to_enu(self, geo: Geodetic) -> ENU:
        """
        Convert a Geodetic position to ENU coordinates relative to the site origin.
        """
        if self.origin is None:
            raise ValueError("Origin must be set before converting to ENU.")

        e, n, u = pm.geodetic2enu(
            lat=geo.lat,
            lon=geo.lon,
            h=geo.alt,
            lat0=self.origin.lat,
            lon0=self.origin.lon,
            h0=self.origin.alt
        )
        return ENU(e=e, n=n, u=u)

    def enu_to_geodetic(self, enu: ENU) -> Geodetic:
        """
        Convert ENU coordinates relative to the site origin back to Geodetic.
        """
        if self.origin is None:
            raise ValueError("Origin must be set before converting to Geodetic.")

        lat, lon, alt = pm.enu2geodetic(
            e=enu.e,
            n=enu.n,
            u=enu.u,
            lat0=self.origin.lat,
            lon0=self.origin.lon,
            h0=self.origin.alt
        )
        return Geodetic(lat=lat, lon=lon, alt=alt)
    
