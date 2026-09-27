"""WGS84 -> continuous MGRS metres (Transverse Mercator, Kruger series, error well below 1 mm).

The organizers' frame is UTM zone 37N with the 100 km square 37UCB as origin and *no* modulo:
``x = easting - 300000``, ``y = northing - 6100000``. No geoid correction is applied to altitude.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

_A = 6378137.0
_F = 1.0 / 298.257223563
_K0 = 0.9996
_E0 = 500000.0
_N = _F / (2.0 - _F)
_E = math.sqrt(_F * (2.0 - _F))
_A_HAT = _A / (1.0 + _N) * (1.0 + _N**2 / 4.0 + _N**4 / 64.0 + _N**6 / 256.0)
_ALPHA = (
    _N / 2 - 2 * _N**2 / 3 + 5 * _N**3 / 16 + 41 * _N**4 / 180 - 127 * _N**5 / 288 + 7891 * _N**6 / 37800,
    13 * _N**2 / 48 - 3 * _N**3 / 5 + 557 * _N**4 / 1440 + 281 * _N**5 / 630 - 1983433 * _N**6 / 1935360,
    61 * _N**3 / 240 - 103 * _N**4 / 140 + 15061 * _N**5 / 26880 + 167603 * _N**6 / 181440,
    49561 * _N**4 / 161280 - 179 * _N**5 / 168 + 6601661 * _N**6 / 7257600,
    34729 * _N**5 / 80640 - 3418889 * _N**6 / 1995840,
    212378941 * _N**6 / 319334400,
)


@dataclass(frozen=True)
class ProjectionConfig:
    utm_zone: int = 37
    northern: bool = True
    false_easting_m: float = 300000.0
    false_northing_m: float = 6100000.0


@dataclass(frozen=True)
class MapPoint:
    x_m: float
    y_m: float
    z_m: float


def project_wgs84(
    latitude_deg: float, longitude_deg: float, altitude_m: float, config: ProjectionConfig
) -> MapPoint:
    values = (latitude_deg, longitude_deg, altitude_m)
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        raise ValueError("latitude, longitude and altitude must be finite numbers")
    if not -90.0 <= latitude_deg <= 90.0 or not -180.0 <= longitude_deg <= 180.0:
        raise ValueError("latitude/longitude out of range")
    if not 1 <= config.utm_zone <= 60:
        raise ValueError("utm_zone must be 1..60")

    lon0 = math.radians(config.utm_zone * 6 - 183)
    phi = math.radians(latitude_deg)
    lam = math.radians(longitude_deg) - lon0
    lam = (lam + math.pi) % (2 * math.pi) - math.pi

    sin_phi = math.sin(phi)
    t = math.sinh(math.atanh(sin_phi) - _E * math.atanh(_E * sin_phi))
    xi0 = math.atan2(t, math.cos(lam))
    eta0 = math.atanh(math.sin(lam) / math.hypot(1.0, t))
    xi, eta = xi0, eta0
    for j, alpha in enumerate(_ALPHA, start=1):
        xi += alpha * math.sin(2 * j * xi0) * math.cosh(2 * j * eta0)
        eta += alpha * math.cos(2 * j * xi0) * math.sinh(2 * j * eta0)

    easting = _E0 + _K0 * _A_HAT * eta
    northing = _K0 * _A_HAT * xi + (0.0 if config.northern else 10_000_000.0)
    return MapPoint(
        x_m=easting - config.false_easting_m,
        y_m=northing - config.false_northing_m,
        z_m=float(altitude_m),
    )
