"""UWAPI-backed board geometry for GamesmanROS.

Logical board coordinates are fetched by (game_id, variant_id), then fitted
uniformly into a centered unit square. Bounds include the declared board and
all non-metadata centers, including legitimate negative coordinates.

AutoGUI center indices are preserved, including special/off-board entries.
Physical capture and pickup locations live in calibration.py, not here.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import urlopen
import json
import math
import os
import threading


UWAPI_URL = os.getenv(
    "GAMESCRAFTERS_API_URL",
    "https://nyc.cs.berkeley.edu/universal/v1",
).rstrip("/")

# Normalized targets must lie within the unit square.
USABLE_UV_MARGIN = 0.0

# Reviewed against UWAPI's image_autogui_data.py. Match identity, index AND raw
# coordinate so an upstream index repurposed for a real position is not hidden.
# No magnitude-based filtering: legitimate negative coordinates still participate.
_METADATA_CENTERS = {
    ("3spot", "regular", "standard"): {13: (-98.9875, -98.9875)},
    ("chungtoi", "regular", "regular"): {9: (-99.0, -99.0)},
    ("lgame", "regular", "regular"): {i: (-99.0, 99.0) for i in range(16, 21)},
    ("ninemensmorris", "regular", "wikipedia"): {26: (-999.0, -999.0), 27: (-999.0, -999.0)},
    ("ninemensmorris", "6mmNoFly", "wikipedia"): {18: (-999.0, -999.0), 19: (-999.0, -999.0)},
    ("tictactwo", "regular", "regular"): {i: (-999.0, -999.0) for i in range(35, 38)},
    ("topitop", "regular", "beach"): {54: (-99.0, -99.0), 55: (-99.0, -99.0)},
    ("adugo", "5x5", "regular"): {0: (100.0, 100.0)},
}

_CACHE: dict[tuple[str, str, str], "BoardGeometry"] = {}
_CACHE_LOCK = threading.Lock()


class GeometryError(Exception):
    """Raised when UWAPI geometry is missing, malformed, or unreachable."""


@dataclass(frozen=True)
class BoardGeometry:
    game: str
    variant: str
    theme: str
    space: tuple[float, float]
    centers: tuple[tuple[float, float], ...]

    @property
    def excluded_indices(self) -> frozenset[int]:
        candidates = _METADATA_CENTERS.get((self.game, self.variant, self.theme), {})
        return frozenset(index for index, expected in candidates.items()
                         if index < len(self.centers) and all(
                             math.isclose(value, target, rel_tol=0.0, abs_tol=1e-8)
                             for value, target in zip(self.centers[index], expected)))

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        """Bounds of the declared board and non-metadata centers."""
        width, height = self.space
        excluded = self.excluded_indices
        points = [point for index, point in enumerate(self.centers) if index not in excluded]
        return (min([0.0] + [x for x, _ in points]),
                min([0.0] + [y for _, y in points]),
                max([width] + [x for x, _ in points]),
                max([height] + [y for _, y in points]))

    @property
    def normalization_scale(self) -> float:
        xmin, ymin, xmax, ymax = self.bounds
        return max(xmax - xmin, ymax - ymin)

    @property
    def normalized_centers(self) -> tuple[tuple[float, float] | None, ...]:
        xmin, ymin, xmax, ymax = self.bounds
        scale = self.normalization_scale
        # Shared denominator preserves aspect ratio; padding centers the bounds.
        pad_x = (1.0 - (xmax - xmin) / scale) / 2.0
        pad_y = (1.0 - (ymax - ymin) / scale) / 2.0
        excluded = self.excluded_indices
        return tuple(None if index in excluded else
                     ((x - xmin) / scale + pad_x, (y - ymin) / scale + pad_y)
                     for index, (x, y) in enumerate(self.centers))

    def center(self, index: int) -> tuple[float, float] | None:
        """Return the normalized center, or None for excluded metadata."""
        return self.normalized_centers[index]


def _cache_key(game: str, variant: str, theme: str | None) -> tuple[str, str, str]:
    return (game, variant, theme or "")


def clear_geometry_cache() -> None:
    with _CACHE_LOCK:
        _CACHE.clear()


def cache_geometry(
    geometry: BoardGeometry, as_default: bool = True
) -> BoardGeometry:
    """Store geometry by resolved theme. Default-theme loads also fill the empty key."""
    with _CACHE_LOCK:
        _CACHE[_cache_key(geometry.game, geometry.variant, geometry.theme)] = geometry
        if as_default:
            _CACHE[_cache_key(geometry.game, geometry.variant, None)] = geometry
    return geometry


def get_cached_geometry(
    game: str, variant: str, theme: str | None = None
) -> BoardGeometry | None:
    with _CACHE_LOCK:
        return _CACHE.get(_cache_key(game, variant, theme))


def is_usable_center(
    point: tuple[float, float] | list[float] | None,
    margin: float = USABLE_UV_MARGIN,
) -> bool:
    """True if a normalized (u, v) point is a plausible on-board robot target."""
    if point is None or len(point) != 2:
        return False
    u, v = float(point[0]), float(point[1])
    if not (math.isfinite(u) and math.isfinite(v)):
        return False
    return -margin <= u <= 1.0 + margin and -margin <= v <= 1.0 + margin


def geometry_from_autogui(
    data: dict[str, Any],
    game: str,
    variant: str,
    theme: str | None = None,
) -> BoardGeometry:
    """Parse `imageAutoGUIData` and cache the result.

    Uses `defaultTheme` unless `theme` is given. Different themes may have
    different geometry; the cache key includes the resolved theme name.
    """
    if not data or not isinstance(data, dict):
        raise GeometryError(f"No Image AutoGUI data for {game}/{variant}")

    themes = data.get("themes")
    if not isinstance(themes, dict) or not themes:
        raise GeometryError(f"No AutoGUI themes for {game}/{variant}")

    theme_name = theme or data.get("defaultTheme")
    if not theme_name or theme_name not in themes:
        raise GeometryError(
            f"Theme {theme_name!r} missing for {game}/{variant}; "
            f"available: {sorted(themes)}"
        )

    theme_data = themes[theme_name]
    if not isinstance(theme_data, dict):
        raise GeometryError(f"Invalid theme {theme_name!r} for {game}/{variant}")

    try:
        width, height = (float(theme_data["space"][0]), float(theme_data["space"][1]))
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        raise GeometryError(f"Invalid board space for {game}/{variant}") from exc

    if not all(math.isfinite(value) and value > 0 for value in (width, height)):
        raise GeometryError(
            f"Invalid board dimensions for {game}/{variant}: {(width, height)}"
        )

    raw_centers = theme_data.get("centers")
    if not isinstance(raw_centers, list) or not raw_centers:
        raise GeometryError(f"No board centers for {game}/{variant}")

    centers: list[tuple[float, float]] = []
    for index, point in enumerate(raw_centers):
        try:
            if len(point) != 2:
                raise TypeError("center is not a pair")
            x, y = float(point[0]), float(point[1])
        except (TypeError, ValueError) as exc:
            raise GeometryError(
                f"Invalid center {index} for {game}/{variant}: {point}"
            ) from exc
        if not all(math.isfinite(value) for value in (x, y)):
            raise GeometryError(
                f"Non-finite center {index} for {game}/{variant}: {point}"
            )
        centers.append((x, y))

    return cache_geometry(
        BoardGeometry(
            game=game,
            variant=variant,
            theme=str(theme_name),
            space=(width, height),
            centers=tuple(centers),
        ),
        as_default=str(theme_name) == str(data.get("defaultTheme")),
    )


def _fetch_variant_payload(game: str, variant: str) -> dict[str, Any]:
    url = f"{UWAPI_URL}/{quote(game, safe='')}/{quote(variant, safe='')}/"
    try:
        with urlopen(url, timeout=5) as response:
            payload = json.load(response)
    except HTTPError as exc:
        raise GeometryError(
            f"UWAPI HTTP {exc.code} for {game}/{variant} ({url})"
        ) from exc
    except URLError as exc:
        raise GeometryError(
            f"UWAPI unreachable for {game}/{variant}: {exc.reason}"
        ) from exc
    except TimeoutError as exc:
        raise GeometryError(f"UWAPI timed out for {game}/{variant}") from exc
    except json.JSONDecodeError as exc:
        raise GeometryError(f"UWAPI returned invalid JSON for {game}/{variant}") from exc

    if not isinstance(payload, dict):
        raise GeometryError(f"Unexpected UWAPI payload for {game}/{variant}")
    return payload


def get_geometry(
    game: str,
    variant: str,
    theme: str | None = None,
) -> BoardGeometry:
    """Fetch and cache geometry for a game variant (default theme unless given)."""
    cached = get_cached_geometry(game, variant, theme)
    if cached is not None:
        return cached

    payload = _fetch_variant_payload(game, variant)
    return geometry_from_autogui(
        payload.get("imageAutoGUIData"), game, variant, theme
    )


def get_centers(game: str, variant: str, theme: str | None = None) -> list[list[float] | None]:
    """Index-preserving normalized pairs; metadata slots are None (not targets)."""
    return [list(point) if point is not None else None
            for point in get_geometry(game, variant, theme).normalized_centers]


def get_dim(game: str, variant: str, theme: str | None = None) -> tuple[float, float]:
    """Original UWAPI width and height (not a scalar grid size)."""
    return get_geometry(game, variant, theme).space
