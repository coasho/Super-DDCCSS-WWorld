# Commanded dual-pulse energy management

PL-15 and R-37M select `fsim.propulsion.jsbsim_commanded_dual_pulse.v1`.
This new explicit selection retains the existing first pulse, authored coast,
second pulse, Isp, grain, mass and thrust-table values. The legacy solid and
`fsim/ignition-time-thrust-v1` selections retain their original automatic timing.

The native engine executes the first pulse and complete original 2 N coast.
It then holds at zero thrust, consuming no reserve fuel, until one valid second
ignition command. The native latch maps elapsed time after ignition onto the
unchanged second-pulse table. Repeating the command cannot restart it. Starvation
remains native and authoritative. A missing or expired command leaves unspent
propellant; a successful complete burn conserves the original authored impulse.

The finite engineering policy consumes the magnitude of the onboard resolved
navigation velocity, native motor age and fuel. Its reserve-speed policy is
ground-relative under this declared estimator; it is not calibrated aerodynamic
energy or a perfect true-airspeed read. Missing navigation refuses an ignition
decision rather than falling back to native true speed.
Within the explicit ignition window, it commands the reserve pulse when own
speed crosses its configured reserve threshold or an admitted terminal seeker
time-to-go reaches the declared lead. Referee target truth never enters this
decision. Unknown terminal timing does not become zero. The initial policy is
an explicit uncalibrated energy-reserve rule, not a claimed optimal guidance
law or a reconstruction of classified fitted logic.

Card policy values carry nonzero screening uncertainty. The hold limits are
30 s (PL-15) and45 s (R-37M), reserve speed450 m/s, and terminal lead6/8 s.
These are engineering assumptions. No scene, guidance gain, phase thrust,
motor impulse or fuze value is fitted to make an encounter pass.

Per-engine native properties live under `fsim/motor-pulse/`: version,
first/second boundaries, finite policy values, second-command, native-age-s,
virtual-age-s and second-ignition-age-s. The generated JSBSim patch uses those
only for the new named table and adds no FGRocket object members or virtual
methods. Native reset clears its latch/ages. The SDK's private manager also
resets its request/expiry history with the owning release.

Upstream FGRocket's solid motor starts at throttle1 and ignores later throttle
once burning. That established implementation is reused; the new latch is an
explicit versioned control seam, not a replacement flight/propulsion solver.
Native, installed, per-variant and conservation results are reported only after
their actual root-owned execution; this method page does not claim those gates.
