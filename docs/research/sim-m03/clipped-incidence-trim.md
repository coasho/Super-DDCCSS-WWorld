# Incidence-constrained native control trim

The configured load-feedback cascade previously constrained the requested incidence while retaining the path rate implied by the rejected force request. A state that simultaneously balanced the available force and moment therefore received a spurious rate correction. `Autopilot.cpp` now solves both balances at the constrained incidence. This is a control-law correction; the retained R-77M coupled encounter still fails and is not accepted by this result.

For either transverse axis, write the native local plant as

```
F = A*x + B*delta + F_res
M = C*x + D*omega + E*delta + M_res = G
m*V*omega = -s*(F + m*g)
```

`x` is incidence, `omega` is path rate, `delta` is actual virtual control angle, and `G` is the corresponding Euler coupling torque. Pitch uses `s=+1`, yaw `s=-1`. All quantities are SI. Eliminating the actual control angle at the retained incidence gives

```
F0 = (A-B*C/E)*x + F_res - B*(M_res-G)/E
omega = -s*(F0+m*g)/(m*V-s*B*D/E)
delta = -(C*x+D*omega+M_res-G)/E
```

The update introduces no state, gain, response-time, incidence-limit or scene change. It executes only for configuration version 1 when the existing combined-incidence cone is active. Unversioned callers and unconstrained arithmetic are retained. Nonfinite or singular solutions refuse. The existing finite actuator, signed stops and authored physical-fin mixer still realize the command; the trim equations never apply an external force.

The domain is the existing local forward-flight, diagonal-inertia control approximation. The current-Mach derivatives, actual mass/CG/inertia, measured aerodynamic residuals and actual control coordinates remain required. Clipping does not assert attainable force throughout a subsequent transient. Physical-fin saturation, coupled nonlinear loads, changing trim, finite response, measurement delay and unmodeled cross-axis effectiveness remain failure modes. The exact per-model plants and sensitivities are recorded in `component-methods/per-model-six-components.json`.

The numerical oracle uses `m=100 kg`, `V=500 m/s`, retained incidence `0.1 rad`, `g=9.80665 m/s2`, and rejected specific-force request `-300 m/s2`. Pitch derivatives `(A,B,C,E,D)` are `(-10000,-2000,-5000,-10000,-2000)`; yaw uses `(-10000,2000,5000,-10000,-2000)`. Residual cases use `(F_res,M_res)=(7 N,15 Nm)`. These are independent synthetic method inputs, not equipment parameter substitutions.

| Axis | Residual loads | Reference path rate (rad/s) | Reference actual angle (rad) | Native angle comparison |
|---|---|---:|---:|---|
| Pitch | None | -0.001600496031746031 | -0.049679900793650797 | Absolute error below 1e-12 rad |
| Pitch | Present | -0.0016798611111111105 | -0.048164027777777778 | Absolute error below 1e-12 rad |
| Yaw | None | 0.001600496031746031 | 0.049679900793650797 | Absolute error below 1e-12 rad |
| Yaw | Present | 0.0017989087301587294 | 0.051140218253968257 | Absolute error below 1e-12 rad |

The oracle independently requires simultaneous force and moment residuals below `1e-9`, then calls the actual production `finCommands`. The unchanged original production source fails the new trim assertion. The corrected source passes all 17 executable cases, including the 16 retained cases. These four new configurations have zero Euler coupling; existing separate coupling tests remain in the same executable, and this table does not claim new joint clipped-plus-coupling coverage.

Exact commands, elapsed times, before/current source hashes and retained failure logs are in `build/architecture-reviews/sim-m03-production-20261103/clipped-trim/regression-receipt.json`. The normal native gate was `tools/launch/fsim.ps1 --preset ucrt64-release test -R '^missile_autopilot$' --no-tests=error --timeout 60` with separate RED and GREEN JUnit destinations.

As conceptual primary-source context, Cotting and Cox describe model-dependent dynamic inversion and control allocation for six-degree-of-freedom simulation in [AIAA-2005-0032, pages 3-6](https://ntrs.nasa.gov/api/citations/20050029394/downloads/20050029394.pdf). The algebra and four numerical cases above are an independent analytic trim comparison; they do not claim execution of that paper's complete controller.
