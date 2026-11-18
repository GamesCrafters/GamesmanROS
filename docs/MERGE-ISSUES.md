# Merge issues — `3fee54c`

State after merging (Manlin) and (Ryan) onto `meow`.

**Builds:** 4 packages, clean.
**Tests:** 16 of 18 pass.
**Runtime:** `arm_controller` cannot start. See issue 1.

The merge itself is complete — no conflicts remain. Everything below is a
collision between two branches that were developed in parallel and never saw
each other, not a conflict-resolution error.

---

## Root cause

Ryan's `centers.py` rewrite changed the coordinate API. Three consumers still
call the old one.

| | before | after (Merge) |
|---|---|---|
| geometry lookup | `get_dim(game)` → `int` | `get_dim(game, variant, theme=None)` → `(w, h)` |
| converter | `CoordConverter(dim)` | `CoordConverter(board_width_mm, board_height_mm, …)` — defaults from `calibration.py` |
| conversion | `conv.svg_to_meters(svg, z_mm)` | `normalized_to_meters()` / `workspace_to_meters()` / `to_meters()` |
| cell count | `conv.dim` | gone — the board is sized in mm, not cells |

Only `robot_control.py` was updated. `arm_controller.py`, `scene.py` and
`reach_sweep.py` were not.

---

## 1. `arm_controller` cannot start — BLOCKING

`_setup_scene()` runs during `__init__`, and every line of it fails:

```python
dim = get_dim(game)             # TypeError: missing 'variant'
self._conv = CoordConverter(dim)  # TypeError: '<=' not supported, tuple vs int
```

Reproduce:

```python
from gamesmanros.centers import get_dim
get_dim('dodgem')   # TypeError
```

No test caught this because constructing `ArmController` needs `MoveItPy`,
which needs a live robot description.

**Owner:** Manlin (wrote `_setup_scene`) with Ryan (owns the new API).
**Needs deciding:** where the variant comes from. `arm_controller` has a
`game_id` parameter but no `variant_id`; `ExecuteMove.action` now carries both,
so the scene may need to be built per-move rather than once at startup.

## 2. `scene.py` collision geometry — BLOCKING, same root cause

```python
x1, y1, _ = conv.svg_to_meters([conv.dim, conv.dim], 0.0)
```

Both `svg_to_meters` and `conv.dim` are gone. `board_box()` sized the slab by
converting cell `(0,0)` to cell `(dim, dim)`; under the new geometry, centers
are fitted into a unit square, so that span is also conceptually wrong — not
just a renamed method.

**Not fixed deliberately.** This box is what MoveIt treats as the board for
collision checking. Too large and valid motions get rejected; too small and the
arm drives into the board. The correct span depends on whether Ryan's unit
square is `[0,1]` or centred on the origin, which the docstring does not say.

Fails: `test_scene.py::test_board_box_covers_every_cell_and_pieces_sit_on_it`,
`test_scene.py::test_board_footprint_matches_the_calibrated_board_size`

**Owner:** Manlin + Ryan.

## 3. `reach_sweep.py` — FIXED

Ported to the new API: `get_centers(game, variant)`, bare `CoordConverter()`,
and `normalized_to_meters()` in place of `svg_to_meters()`. Takes a `variant`
parameter now. Verified against mock hardware.

## 4. `pymycobot` is undeclared

`mycobot_topics.py` needs it, imports it, and mentions it only in a
docstring. It is in neither `package.xml` nor the Dockerfile, so a clean
container cannot run the serial driver.

**Fix:** add to the Dockerfile and as an `<exec_depend>`.

## 5. `upstream/Updated-CV` is unmerged

Two commits, deliberately left out. The branch is a YOLO segmentation model
whose weights are not in the repository and whose path points at a Windows lab
machine. It cannot run as committed.

**Needs deciding:** delete it, or someone claims it. Leaving it looks like
pending work that nobody is doing.

---

## Resolved during the merge, for the record

- `centers.py` — took Ryan's version wholesale. It supersedes the hand-maintained
  tables and removes the bug class `test_board_dimensions.py` guarded, so that
  test was deleted.
- `robot_control.py` — kept both sides: Ryan's injected geometry
  (`centers`, `conv`) plus the vision pick-correction (`observe` callback,
  `BoardState` subscription) that his branch predates.
- `BASE_FRAME` is now `g_base`, matching the SRDF. Manlin had already fixed this.
- `MycobotGripperStatus.status` is lowercase. Manlin hit this on hardware and
  fixed it in `fa7fa63`.
- `test_hardware_path.py` assigned ints to `float32` fields. Fixed.
- `test_scene.py` updated to the new API — still fails on issue 2.
