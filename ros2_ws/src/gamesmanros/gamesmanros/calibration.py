"""Physical robot-board calibration, independent of UWAPI game geometry.

Normalized (u, v) board coordinates are mapped onto this physical rectangle.
Capture and pickup are shared workspace locations in the robot base frame
(millimeters), not game-specific SVG points.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class WorkspacePoint:
    """A physical location in the robot base frame, in millimeters."""

    x_mm: float
    y_mm: float


# Playing surface size. A 1×1 normalized board may be mapped onto a rectangle.
BOARD_WIDTH_MM = 150.0
BOARD_HEIGHT_MM = 150.0

# Affine mapping matching the existing ROS 2 robot frame:
#   x = (u' * width)  - origin_x
#   y = (v' * height) + origin_y
# where u'/v' are optionally flipped from normalized UWAPI coordinates.
ORIGIN_X_MM = BOARD_WIDTH_MM / 2.0 + 50.0  # 125 mm
ORIGIN_Y_MM = 100.0

FLIP_X = False
FLIP_Y = True  # SVG y grows down; robot y grows away from the base

PICKUP_Z_MM = 135.0
LIFT_Z_MM = 185.0

# Shared off-board storage, to the robot's left of a 150×150 mm board.
CAPTURE_XY_MM = WorkspacePoint(-160.0, 175.0)

# Sequential Type1 supply slots, stacked along the same off-board edge.
PICKUP_XY_MM: tuple[WorkspacePoint, ...] = tuple(
    WorkspacePoint(-160.0, 230.0 - index * 18.0) for index in range(12)
)


def get_capture() -> WorkspacePoint:
    """Shared physical capture / drop-off location."""
    return CAPTURE_XY_MM


def get_pickup() -> tuple[WorkspacePoint, ...]:
    """Shared physical piece-supply slots, indexed by Type1 place order."""
    return PICKUP_XY_MM
