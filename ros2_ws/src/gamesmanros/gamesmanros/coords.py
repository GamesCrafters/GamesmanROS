"""Normalized board (u, v) → robot-frame meters."""

from __future__ import annotations

from .calibration import (
    BOARD_HEIGHT_MM,
    BOARD_WIDTH_MM,
    FLIP_X,
    FLIP_Y,
    LIFT_Z_MM,
    ORIGIN_X_MM,
    ORIGIN_Y_MM,
    PICKUP_Z_MM,
    WorkspacePoint,
)
from .centers import is_usable_center


class CoordinateError(ValueError):
    """Raised when a target is not a valid physical robot destination."""


class CoordConverter:
    def __init__(
        self,
        board_width_mm: float = BOARD_WIDTH_MM,
        board_height_mm: float = BOARD_HEIGHT_MM,
        origin_x_mm: float = ORIGIN_X_MM,
        origin_y_mm: float = ORIGIN_Y_MM,
        flip_x: bool = FLIP_X,
        flip_y: bool = FLIP_Y,
        pickup_z_mm: float = PICKUP_Z_MM,
        lift_z_mm: float = LIFT_Z_MM,
        workspace_limit_mm: float = 400.0,
    ):
        if board_width_mm <= 0 or board_height_mm <= 0:
            raise CoordinateError("Board width and height must be positive")
        self.board_width_mm = board_width_mm
        self.board_height_mm = board_height_mm
        self.origin_x_mm = origin_x_mm
        self.origin_y_mm = origin_y_mm
        self.flip_x = flip_x
        self.flip_y = flip_y
        self.pickup_z = pickup_z_mm / 1000.0
        self.lift_z = lift_z_mm / 1000.0
        self.workspace_limit_mm = workspace_limit_mm

    def normalized_to_meters(
        self, point: list[float] | tuple[float, float], z_mm: float
    ) -> tuple[float, float, float]:
        """Map a normalized board coordinate to robot base-frame meters."""
        if not is_usable_center(point):
            raise CoordinateError(f"Board target is not robot-usable: {point}")
        u, v = float(point[0]), float(point[1])
        u_mapped = (1.0 - u) if self.flip_x else u
        v_mapped = (1.0 - v) if self.flip_y else v
        x_mm = u_mapped * self.board_width_mm - self.origin_x_mm
        y_mm = v_mapped * self.board_height_mm + self.origin_y_mm
        return self._meters(x_mm, y_mm, z_mm)

    def workspace_to_meters(
        self, point: WorkspacePoint, z_mm: float
    ) -> tuple[float, float, float]:
        """Map a shared physical workspace point to robot base-frame meters."""
        return self._meters(point.x_mm, point.y_mm, z_mm)

    def to_meters(
        self,
        point: WorkspacePoint | list[float] | tuple[float, float],
        z_mm: float,
    ) -> tuple[float, float, float]:
        if isinstance(point, WorkspacePoint):
            return self.workspace_to_meters(point, z_mm)
        return self.normalized_to_meters(point, z_mm)

    def _meters(self, x_mm: float, y_mm: float, z_mm: float) -> tuple[float, float, float]:
        limit = self.workspace_limit_mm
        if not all(abs(value) <= limit for value in (x_mm, y_mm)) or abs(z_mm) > limit:
            raise CoordinateError(
                f"Physical target outside workspace: ({x_mm}, {y_mm}, {z_mm}) mm"
            )
        return x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0
