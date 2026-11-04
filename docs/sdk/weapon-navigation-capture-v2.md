# Native weapon navigation capture and control epochs

This additive integration applies to explicitly selected body-IMU strapdown
navigation. NavigationV1, old SDK layouts, actor permissions, radio generation
rules and the existing native outer-step control cadence remain unchanged.

`NavigationCaptureV2::observe` retains the immediately adjacent native own-motion
captures. At a real scheduled IMU or fitted GNSS epoch it interpolates those
captures and advances `NavigationStrapdownV2` to that event. Dense FDM captures
between events do not integrate the state or covariance. An explicit native
control read advances the held-IMU mechanization to the control epoch. Integer
sensor clocks and RNG streams belong to the existing estimator; capture and
World batch boundaries do not reset them.

`VehiclePool::setNativeObserverFactoryV1` scopes observation around exactly one
native `FlightModel::step`. A synchronous pre-existing observer for the same
native model receives the same callbacks, without duplicate invocation of an
identical observer. RAII restores the previous thread-local observer after both
normal return and exception unwinding. Observers read native motion and update
private navigation at sensor events; they do not change forces, controls, native
integration or another vehicle.

World consumes the current captured own pose after native effects establish
GNSS availability and before pulse policy, estimated-body demand resolution and
gyro feedback. Each inner FDM sample supplies physical interpolation support;
the final inner sample is read at the canonical next outer-control or World
boundary instead of introducing a differently rounded duplicate epoch. The
canonical final pool state advances navigation before SDK sensing/guidance
consumes the completed decision boundary. A capture refusal suppresses new
modeled control issuance and is reported at the main World boundary, rather
than throwing through a native worker thread.

Selected strapdown control resolves demand and Earth-rate subtraction through
estimated attitude. The retained 9.80665 m/s² control-law vertical uses WGS84
geodetic latitude computed from estimated own ECEF position, through
`navigationGravityVerticalV2`; true native latitude/longitude do not correct it.
This control-law approximation is separate from the strapdown J2/Coriolis/
centrifugal mechanization. Legacy control retains its previous vertical.

`Entry::finNavigationTimeS` freezes the navigation epoch actually read by the
native controller. The additive internal referee-only
`World::weaponNavigationControlEpochsV2` returns controller and navigation read
epochs. It adds no SDK facade, actor report, proxy method or existing public
struct field. Reset makes both epochs unavailable until another control read.

The focused source `test_weapon_navigation_integration.cpp` covers identical
rotating native-input history across batch sizes, sensor RNG/covariance equality,
dense-capture decimation, estimated WGS84 vertical, actual native observer fanout
and actual World release/reset/control epochs at frameSkip 1 and 4. Its direct
World release fixture is explicitly referee diagnostic evidence, not actor
consent acceptance. The root owns execution and current test results; this
method record claims no execution or hardware calibration.

The physical conventions and primary references remain in
[the strapdown method map](../research/sim-m03/strapdown-navigation-method.md).
The source integration window is main HEAD
`e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`; its bounded paths are World.cpp,
World.h, WorldWeaponLinks.cpp, VehiclePool.h/.cpp, NavigationStrapdownV2.h/.cpp,
the new NavigationCaptureV2.h, the focused integration test and this method note.
