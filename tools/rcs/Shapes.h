#pragma once
// Canonical targets with closed-form radar cross sections (Knott, Shaeffer and
// Tuley, "Radar Cross Section", 2nd ed., table 6.1; Ruck et al., "Radar Cross
// Section Handbook"): what the engine is validated against.

#include "Engine.h"

namespace fsim::rcs::shapes {

/// A sphere of radius `a` (m), its normals radial: sigma = pi a^2 once a >> lambda.
Mesh sphere(double a, int rings = 256);
/// A square flat plate of side `a` in the y-z plane, facing +-x: sigma = 4 pi a^4 / lambda^2 broadside.
Mesh plate(double a, int cells = 1);
/// A right dihedral: two square plates of side `a`, their seam along y, opening towards +x (each face at 45 deg
/// to it): sigma = 8 pi a^4 / lambda^2 towards +x (the faces' area a^2 each).
Mesh dihedral(double a);
/// A square trihedral corner reflector: three square plates of side `a` along the axes, opening along (1,1,1)
/// rotated onto +x: sigma = 12 pi a^4 / lambda^2 along its symmetry axis.
Mesh trihedral(double a);
/// A closed circular cylinder of radius `a` and length `l` along y: sigma = 2 pi a l^2 / lambda broadside.
Mesh cylinder(double a, double l, int sides = 512);

} // namespace fsim::rcs::shapes
