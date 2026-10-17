"""The records the World and Agent wrappers share (ARCH-07): both build them from what the library returns, so they sit
below both wrappers. fsim.world re-exports each and remains its owner (__module__ "fsim.world"), so qualified names,
reprs and pickling are unchanged."""
import collections

_OWNER = "fsim.world"

# Systems, nets and the DMS (ADR-31, AB-4)
DmsReceived = collections.namedtuple("DmsReceived", "message source topic net time_sent time_delivered data", module=_OWNER)
DmsTransmission = collections.namedtuple("DmsTransmission", "message topic time_sent published queue_dropped reached destinations",
                                         module=_OWNER)
NetStats = collections.namedtuple("NetStats", "transmissions delivered duplicates out_of_range masked lost queue_dropped failed "
                                              "overflowed", module=_OWNER)
# AB-4b: a system's picture - a peer's last position report and heartbeat, a track received over a link
PeerReport = collections.namedtuple("PeerReport", "system net received_s tail_number has_position time_s latitude_rad longitude_rad "
                                                  "altitude_msl_m velocity_ned_ms heading_rad airspeed_true_ms airspeed_calibrated_ms mach",
                                    module=_OWNER)
# A legal report's provenance and age (docs/sdk/mission-training.md, section 10): source "own_navigation", "own_sensor",
# "own_platform", "ma_core", "dms" or "link"; validity "valid", "stale", "invalid" or "unknown"; the system it came
# from and the net it came by where a link delivered it, the payload whose report it is, and its times in simulation
# seconds - each None where there is none or the source states none.
ReportMeta = collections.namedtuple("ReportMeta", "source validity system net payload sample_time_s receive_time_s age_s",
                                    module=_OWNER)
# One item of a report collection, a collection ("available", "unavailable", "not_applicable" or "not_owner"; no items
# unless available), and a single report (meta and value None unless available).
ReportItem = collections.namedtuple("ReportItem", "meta value", module=_OWNER)
ReportCollection = collections.namedtuple("ReportCollection", "availability items", module=_OWNER)
Report = collections.namedtuple("Report", "availability meta value", module=_OWNER)
# AB-5a: C2's authority and command statuses
C2Authority = collections.namedtuple("C2Authority", "designated designated_modes designated_by designated_s operators commands",
                                     module=_OWNER)
CommandStatus = collections.namedtuple("CommandStatus", "source received_s command_id topic state reason activity time_s percent",
                                       module=_OWNER)
# AB-5b: C2's tasks, overrides, release consent
C2Task = collections.namedtuple("C2Task", "source received_s state percent task_id type priority latitude_rad longitude_rad altitude_m "
                                          "radius_m start_s end_s target_source target_payload target_track text", module=_OWNER)
ActiveOverride = collections.namedtuple("ActiveOverride", "source kind value expiry_s", module=_OWNER)
# AB-5c: an MA's lost-communication state and its team
CommsState = collections.namedtuple("CommsState", "lost_comms_set threshold_s plan lost lost_since_s alerts package_id leader members",
                                    module=_OWNER)
ConsentRequest = collections.namedtuple("ConsentRequest", "source received_s answered request_id strike store quantity target_source "
                                                          "target_payload target_track latitude_rad longitude_rad altitude_m time_s",
                                        module=_OWNER)
LinkTrack = collections.namedtuple("LinkTrack", "source net received_s track payload updates time_s latitude_rad longitude_rad height_m "
                                                "velocity_ned_ms position_sigma_m velocity_sigma_ms", module=_OWNER)

StrikeStatus = collections.namedtuple("StrikeStatus", "phase jettison target stores weapons", module=_OWNER)
StrikeStatus.__doc__ = """A strike's or jettison's progress (MS-7): ``phase`` "powering_up", "awaiting_consent",
"releasing" or "done"; the stores it releases (their indices) and the weapons released so far (entity ids)."""
