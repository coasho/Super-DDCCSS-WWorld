"""Immutable V1 records for native sensor/link-derived platform fusion."""
from collections import namedtuple
from enum import IntEnum


class FusionStateV1(IntEnum):
    LIVE=0
    COASTING=1
    LOST=2


class FusionSourceKindV1(IntEnum):
    LOCAL_RADAR=0
    DELIVERED_LINK=1


FusionSourceV1=namedtuple('FusionSourceV1','kind system payload diagonal_link_covariance track sample_time_s received_time_s')
FusedEntityV1=namedtuple('FusedEntityV1','id state time_s last_update_s valid_until_s identity_known standard_identity state_ecef covariance_ecef sources')
FusionStatusV1=namedtuple('FusionStatusV1','time_s rejected_inputs capacity_refusals entity_capacity source_capacity')


def _entities_v1(rows):
    return [FusedEntityV1(r[0],FusionStateV1(r[1]),*r[2:5],bool(r[5]),r[6],r[7],r[8],
        tuple(FusionSourceV1(FusionSourceKindV1(s[0]),s[1],s[2],bool(s[3]),*s[4:]) for s in r[9])) for r in rows]
