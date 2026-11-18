"""Inspect UWAPI-to-robot coordinates without importing ROS or sending motion."""

import argparse
import json
import math
from pathlib import Path

from .centers import GeometryError, geometry_from_autogui, get_geometry
from .coords import CoordinateError, CoordConverter
from .calibration import get_capture, get_pickup


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('game')
    parser.add_argument('variant')
    parser.add_argument('--theme')
    parser.add_argument('--input', type=Path, help='Offline variant JSON or imageAutoGUIData JSON')
    parser.add_argument('--indices', type=int, nargs='+', help='AutoGUI indices; default: all')
    defaults = CoordConverter()
    for name in ('board_width_mm', 'board_height_mm', 'origin_x_mm', 'origin_y_mm'):
        parser.add_argument('--' + name.replace('_', '-'), type=float, default=getattr(defaults, name))
    parser.add_argument('--flip-x', action=argparse.BooleanOptionalAction, default=defaults.flip_x)
    parser.add_argument('--flip-y', action=argparse.BooleanOptionalAction, default=defaults.flip_y)
    parser.add_argument('--z-mm', type=float, default=defaults.pickup_z * 1000)
    parser.add_argument('--json', action='store_true', help='Machine-readable output')
    args = parser.parse_args(argv)
    try:
        if args.input:
            payload = json.loads(args.input.read_text())
            geometry = geometry_from_autogui(payload.get('imageAutoGUIData', payload), args.game, args.variant, args.theme)
        else:
            geometry = get_geometry(args.game, args.variant, args.theme)
        values = {name: getattr(args, name) for name in ('board_width_mm', 'board_height_mm', 'origin_x_mm', 'origin_y_mm', 'flip_x', 'flip_y')}
        if not all(math.isfinite(v) for v in (*values.values(), args.z_mm)):
            raise CoordinateError('Calibration values must be finite')
        converter = CoordConverter(**values)
        rows = []
        indices = args.indices if args.indices is not None else range(len(geometry.centers))
        for index in indices:
            if not 0 <= index < len(geometry.centers):
                raise ValueError(f'Index {index} outside 0..{len(geometry.centers)-1}')
            uv = geometry.center(index)
            row = dict(index=index, raw=geometry.centers[index], normalized=uv,
                       excluded=index in geometry.excluded_indices)
            try:
                if row['excluded']:
                    raise CoordinateError('excluded metadata center')
                row['robot_m'] = converter.normalized_to_meters(uv, args.z_mm)
                row['status'] = 'usable'
            except CoordinateError as exc:
                row.update(robot_m=None, status=str(exc))
            rows.append(row)
        storage = {'capture_m': converter.workspace_to_meters(get_capture(), args.z_mm),
                   'pickup_m': [converter.workspace_to_meters(p, args.z_mm) for p in get_pickup()]}
    except (GeometryError, CoordinateError, ValueError, OSError, TypeError) as exc:
        parser.error(str(exc))
    if args.json:
        print(json.dumps(dict(game=args.game, variant=args.variant, theme=geometry.theme,
                              space=geometry.space, bounds=geometry.bounds, scale=geometry.normalization_scale, calibration=values, z_mm=args.z_mm,
                              centers=rows, storage=storage), indent=2))
    else:
        print(f'{args.game}/{args.variant} theme={geometry.theme} space={geometry.space}')
        print(f'Normalization bounds={geometry.bounds}, shared scale={geometry.normalization_scale}')
        print('index | raw (x,y) | normalized (u,v) | robot (x,y,z) meters | status')
        for row in rows:
            print(f"{row['index']:5} | {row['raw']} | {row['normalized']} | {row['robot_m']} | {row['status']}")
        print(f"Shared capture (meters): {storage['capture_m']}")
        print(f"Shared pickup slots (meters): {storage['pickup_m']}")


if __name__ == '__main__':
    main()
