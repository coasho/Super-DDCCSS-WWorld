"""TEMPORARY (an OMS stand-in, see __init__): what each fifth-generation fighter carries - notional numbers from public
estimates, rounded; none of it is authoritative, and only how they compare matters to a test.

rcs_m2       frontal radar cross-section (m^2): the stealth that shortens everyone else's detection and seeker ranges
radar_m      the range its radar detects a 1 m^2 target at, head-on (m); a target of RCS s at radar_m * s^(1/4)
gimbal_deg   how far off the nose the radar sees and holds a track (the Su-57's side arrays widen its field)
irst_m       how far its infra-red search and track sees a fighter head-on at dry power (m; 0 none): angles only
missile      its long-range missile (missile.py's table) and how many it carries internally
max_g        the load it pulls in a defensive turn (the fly-by-wire's limit, a little under)"""

FIGHTERS = {
    "f22a": dict(rcs_m2=0.0001, radar_m=220000.0, gimbal_deg=60.0, irst_m=0.0, missile="aim120d", missiles=6, max_g=7.5),
    "f35a": dict(rcs_m2=0.001, radar_m=170000.0, gimbal_deg=60.0, irst_m=45000.0, missile="aim120d", missiles=4, max_g=7.0),
    "j20a": dict(rcs_m2=0.01, radar_m=180000.0, gimbal_deg=60.0, irst_m=50000.0, missile="pl15", missiles=4, max_g=7.0),
    "su57": dict(rcs_m2=0.1, radar_m=170000.0, gimbal_deg=75.0, irst_m=55000.0, missile="r77m", missiles=4, max_g=7.5),
}

# Where the fight is known from before anyone's radar sees it: a ground controller's or an AWACS's picture,
# updated every few seconds, a couple of kilometres off.
PICTURE_PERIOD_S = 8.0
PICTURE_ERROR_M = 2000.0
