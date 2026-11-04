"""Explicit V1 weapon-link facade over an Agent's native permission-scoped handle.

Construct with ``WeaponLinksV1(agent)``. This facade is deliberately absent from
the default strict actor callable-method list and frozen engagement.reports.v1.
Returned telemetry is onboard data received over modeled native transport.
"""
from collections import namedtuple
from enum import IntEnum

class WeaponLinkDirectionV1(IntEnum):
    UNSUPPORTED=0
    UPLINK=1
    TWO_WAY=2

class WeaponLinkReasonV1(IntEnum):
    NONE=0
    UNSUPPORTED=1
    UNKNOWN_WEAPON=2
    NOT_OWNER=3
    INVALID=4
    REQUEST_CONFLICT=5
    CAPACITY=6
    CLOSED=7
    UNKNOWN_ASSESSMENT=8
    EXPIRED=9

class WeaponAssessmentStateV1(IntEnum):
    PENDING=0
    ACTIVE=1
    SOFT_LOST=2
    LOST=3
    END_OF_FLIGHT=4
    CANCELED=5
    EXPIRED=6

WeaponLinkCapabilityV1=namedtuple('WeaponLinkCapabilityV1',
    'instance activity weapon direction reason update_period_s feedback_period_s latency_s data_rate_bps range_m frequency_hz maximum_age_s assessment_timeout_s retention_s direction_source loss_probability maximum_queue_s queue_capacity report_capacity assessment_capacity jitter_s')
WeaponAssessmentReceiptV1=namedtuple('WeaponAssessmentReceiptV1','accepted assessment request_id reason')
WeaponAssessmentStatusV1=namedtuple('WeaponAssessmentStatusV1','assessment request_id state reason requested_time_s expires_time_s last_received_time_s')
WeaponFeedbackV1=namedtuple('WeaponFeedbackV1','assessment sequence weapon sample_time_s sent_time_s received_time_s valid_until_s navigation_valid tracking active supported end_of_flight state_ecef time_to_active_s time_to_intercept_s navigation_time_s navigation_covariance navigation_gnss_aided')
FusedDesignationV1=namedtuple('FusedDesignationV1','target entity valid')

class WeaponLinksV1:
    """Own platform/controller; no World, referee or raw transport access is returned."""
    __slots__=('_handle','_platform','_instance')

    def __init__(self,agent):
        from .agent import Agent
        if not isinstance(agent,Agent):
            raise TypeError('WeaponLinksV1 requires an Agent')
        self._handle=agent._world._h
        self._platform=agent.vehicle_id
        self._instance=self._handle.weapon_links_v1(0,self._platform)

    @property
    def instance(self):
        return self._instance

    def capability(self,weapon):
        r=self._handle.weapon_links_v1(1,self._platform,int(weapon))
        return WeaponLinkCapabilityV1(*r[:3],WeaponLinkDirectionV1(r[3]),WeaponLinkReasonV1(r[4]),*r[5:])

    def designate_fused(self,entity,*,instance=None):
        return self._handle.weapon_links_v1(2,self._platform,self._instance if instance is None else int(instance),int(entity))

    def fused_designations(self):
        return [FusedDesignationV1(r[0],r[1],bool(r[2])) for r in self._handle.weapon_links_v1(3,self._platform)]

    def assess(self,weapon,request_id,*,period_s=.2,duration_s=30,instance=None):
        r=self._handle.weapon_links_v1(4,self._platform,self._instance if instance is None else int(instance),int(weapon),int(request_id),float(period_s),float(duration_s))
        return WeaponAssessmentReceiptV1(bool(r[0]),r[1],r[2],WeaponLinkReasonV1(r[3]))

    def cancel(self,weapon,assessment,*,instance=None):
        return self._handle.weapon_links_v1(5,self._platform,self._instance if instance is None else int(instance),int(weapon),int(assessment))

    def status(self,weapon,assessment):
        r=self._handle.weapon_links_v1(6,self._platform,int(weapon),int(assessment))
        return WeaponAssessmentStatusV1(r[0],r[1],WeaponAssessmentStateV1(r[2]),WeaponLinkReasonV1(r[3]),*r[4:])

    def feedback(self,weapon,assessment,*,since=0):
        return [WeaponFeedbackV1(*r[:7],*(bool(x) for x in r[7:12]),*r[12:17],bool(r[17]))
            for r in self._handle.weapon_links_v1(7,self._platform,int(weapon),int(assessment),int(since))]


_LINK_CALLS_V1={'instance':'property','capability':'method','designate_fused':'method',
                'fused_designations':'method','assess':'method','cancel':'method',
                'status':'method','feedback':'method'}

def weapon_link_calls_v1():
    """The explicit optional proxy's complete callable list; default Agent lists are unchanged."""
    return dict(_LINK_CALLS_V1)

class RemoteWeaponLinksV1:
    """Only the V1 weapon-link methods of one platform/controller, in an actor process."""
    __slots__=('_channel',)

    def __init__(self,channel):
        object.__setattr__(self,'_channel',channel)

    def __getattr__(self,name):
        from ._adapters.codec import _encode
        kind=_LINK_CALLS_V1.get(name)
        if kind is None:
            raise AttributeError('%r is not a V1 weapon-link call' % name)
        if kind=='property':
            return self._channel.request({'op':'agent','m':name})
        return lambda *args,**kwargs:self._channel.request({'op':'agent','m':name,
            'a':[_encode(value) for value in args],
            'k':{key:_encode(value) for key,value in kwargs.items()}})

    def __setattr__(self,name,value):
        raise AttributeError('a remote weapon-link facade holds nothing to set')

    def __repr__(self):
        return 'RemoteWeaponLinksV1()'


