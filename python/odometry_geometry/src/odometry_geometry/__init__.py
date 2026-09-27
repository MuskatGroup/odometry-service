"""Map geometry for the tram odometry service: Pathgraph, WGS84 -> continuous MGRS, antenna transforms.

Standard library only; knows nothing about ROS or the estimator core.
"""

from .antennas import (
    BaseLinkPose,
    TramGeometry,
    base_link_from_dual_antenna,
    base_link_from_single_antenna,
)
from .pathgraph import (
    STATUS_AMBIGUOUS,
    STATUS_MATCHED,
    STATUS_OUT_OF_GRAPH,
    OutOfGraphError,
    PathGraph,
    TrackMatch,
    TrackPose,
    normalize_route_id,
)
from .projection import MapPoint, ProjectionConfig, project_wgs84

__all__ = [
    "STATUS_AMBIGUOUS",
    "STATUS_MATCHED",
    "STATUS_OUT_OF_GRAPH",
    "BaseLinkPose",
    "MapPoint",
    "OutOfGraphError",
    "PathGraph",
    "ProjectionConfig",
    "TramGeometry",
    "TrackMatch",
    "TrackPose",
    "base_link_from_dual_antenna",
    "base_link_from_single_antenna",
    "normalize_route_id",
    "project_wgs84",
]
