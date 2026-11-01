"""Versioned functional population selection; no component/table waivers."""
from collections import Counter

ALL = 'all'
NONROTOR = 'sim-m02-nonrotor-v1'


def functional_rows(rows, scope=ALL):
    if scope == ALL:
        return rows, []
    if scope != NONROTOR:
        raise ValueError('Unknown functional population scope')
    if Counter(r['category'] for r in rows) != {'fixed-wing':31, 'rotorcraft':4, 'store':53}:
        raise ValueError('The approved scope requires the complete maintained88 design roster')
    return [r for r in rows if r['category'] != 'rotorcraft'], [r for r in rows if r['category'] == 'rotorcraft']
