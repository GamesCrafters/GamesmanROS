#!/usr/bin/env python3
"""
Board dimension validation.

The coordinate pipeline in robot_control.py converts an SVG cell coordinate to
metres with:

    scaling = BOARD_SIZE_MM / dim
    x_mm    = svg_x * scaling - x_offset

For that to place the board correctly on the table, the coordinate table and the
declared `dim` have to agree. Concretely, for a dim x dim grid whose cell centres
sit at k + 0.5 (k = 0 .. dim-1):

    largest coordinate  ==  dim - 0.5
    board footprint     ==  dim * (BOARD_SIZE_MM / dim)  ==  BOARD_SIZE_MM

If `dim` disagrees with the table, every square is mis-scaled and the arm reaches
for the wrong place. This module checks that invariant for every game.

Run standalone for a readable report:

    python3 test/test_board_dimensions.py

Or under pytest:

    pytest test/test_board_dimensions.py -v
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from gamesmanros.centers import _centers, _dim, _pickup, _capture   # noqa: E402

BOARD_SIZE_MM = 150.0
TOLERANCE_MM  = 1.0

# Games whose coordinate data is known to be wrong. Each entry is a bug to fix,
# not an exemption — delete the entry once the data is corrected and the test
# will hold the line from then on.
KNOWN_BROKEN = {
    'dao':             'normalised 0-1 coords; scaling assumes raw cell units',
    'allqueenschess':  'normalised 0-1 coords; scaling assumes raw cell units',
    'dragonsandswans': 'normalised 0-1 coords, and no _dim entry at all',
    'jan':             'dim=3 but the table is 4x4, and cells are corner- not centre-aligned',
    'dawsonschess':    'dim=5 but coordinates run to 5.5, implying 6',
}

# Type classes that need extra tables beyond _centers.
NEEDS_PICKUP  = {'dawsonschess'}          # Type1 picks from an off-board stack
NEEDS_CAPTURE = {'1dchess'}               # Type7 moves captured pieces aside


def audit(game):
    """Return a findings dict for one game. Empty 'errors' means it is consistent."""
    centers = _centers.get(game)
    dim     = _dim.get(game)
    errors  = []

    if not centers:
        return {'game': game, 'errors': ['no entry in _centers'], 'dim': dim}

    max_x = max(p[0] for p in centers)
    max_y = max(p[1] for p in centers)
    max_c = max(max_x, max_y)
    min_c = min(min(p[0] for p in centers), min(p[1] for p in centers))

    if dim is None:
        errors.append('no entry in _dim (CoordConverter would raise TypeError)')
        return {'game': game, 'errors': errors, 'dim': None,
                'max': max_c, 'n': len(centers), 'extent_mm': None}

    normalised = max_c <= 1.0
    if normalised:
        errors.append(f'coords are normalised 0-1 (max={max_c:.3f}) but scaling '
                      f'expects raw cell units')

    scaling   = BOARD_SIZE_MM / dim
    extent_mm = (max_c + 0.5) * scaling      # +0.5 for the far half-cell margin

    # Centre-aligned tables put cells at k+0.5; corner-aligned ones at k.
    corner_aligned = (min_c == int(min_c))
    implied_dim    = (max_c + 1) if corner_aligned else (max_c + 0.5)

    expected_max = dim - 0.5
    if not normalised and abs(max_c - expected_max) > 1e-6:
        errors.append(f'largest coord is {max_c:g}, expected {expected_max:g} '
                      f'for dim={dim} (table implies dim={implied_dim:g})')

    if not normalised and abs(extent_mm - BOARD_SIZE_MM) > TOLERANCE_MM:
        errors.append(f'board footprint works out to {extent_mm:.1f} mm, '
                      f'expected {BOARD_SIZE_MM:.0f} mm')

    if not normalised and corner_aligned:
        errors.append(f'cells are corner-aligned (min={min_c:g}), expected '
                      f'centre-aligned at 0.5 — every square is offset by half a cell')

    if game in NEEDS_PICKUP and not _pickup.get(game):
        errors.append('Type1 game but _pickup has no entry (raises TypeError on '
                      'first move)')
    if game in NEEDS_CAPTURE and not _capture.get(game):
        errors.append('needs a _capture square but none is defined')

    return {'game': game, 'errors': errors, 'dim': dim, 'max': max_c,
            'n': len(centers), 'extent_mm': extent_mm}


# ---------------------------------------------------------------------------
# pytest
# ---------------------------------------------------------------------------

def test_every_game_with_centers_has_a_dim():
    missing = [g for g in _centers if g not in _dim and g not in KNOWN_BROKEN]
    assert not missing, f'games with centres but no dim: {missing}'


def test_board_dimensions_are_consistent():
    """Every game not on the quarantine list must satisfy the dimension invariant."""
    failures = {}
    for game in sorted(_centers):
        if game in KNOWN_BROKEN:
            continue
        result = audit(game)
        if result['errors']:
            failures[game] = result['errors']
    assert not failures, f'dimension check failed: {failures}'


def test_quarantine_list_is_still_accurate():
    """A game on KNOWN_BROKEN that now passes should be removed from the list."""
    fixed = [g for g in KNOWN_BROKEN if g in _centers and not audit(g)['errors']]
    assert not fixed, (
        f'these games now pass and should be removed from KNOWN_BROKEN: {fixed}')


# ---------------------------------------------------------------------------
# standalone report
# ---------------------------------------------------------------------------

def main():
    rows = [audit(g) for g in sorted(_centers)]
    ok   = [r for r in rows if not r['errors']]
    bad  = [r for r in rows if r['errors']]

    print(f'\nBoard dimension audit  —  target footprint {BOARD_SIZE_MM:.0f} mm\n')
    print(f'{"game":18}{"dim":>5}{"cells":>7}{"max":>8}{"footprint":>12}   status')
    print('-' * 72)
    for r in rows:
        dim  = r['dim'] if r['dim'] is not None else '-'
        mx   = f"{r['max']:.3f}" if r.get('max') is not None else '-'
        ext  = f"{r['extent_mm']:.1f} mm" if r.get('extent_mm') else '-'
        mark = 'OK' if not r['errors'] else f"{len(r['errors'])} problem(s)"
        print(f"{r['game']:18}{str(dim):>5}{r['n']:>7}{mx:>8}{ext:>12}   {mark}")

    if bad:
        print('\nDetails\n' + '-' * 72)
        for r in bad:
            print(f"\n  {r['game']}")
            for e in r['errors']:
                print(f"    - {e}")

    print(f'\n{len(ok)} consistent, {len(bad)} with problems.\n')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
