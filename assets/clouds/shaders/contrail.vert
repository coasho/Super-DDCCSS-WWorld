// fsim: a contrail segment (world::Airflow): the box round everything of the trail between two samples (the wake's
// tubes and curtain below the flight path, the plume below them), its far faces drawn; the planes that end the segment
// (each the bisector of the trail's bend there, shared with the neighbour, so a ray is marched through each stretch
// once); and the light there - the sun's and the sky's from the atmosphere (as takram's clouds take theirs), and the
// aerial perspective between it and the eye (Bruneton's GetSkyRadianceToPoint, as takram's clouds apply it).

#version 450
#include "contrail.glsl"

layout(location = 0) out vec3 v_pos;                 // the fragment from the eye
layout(location = 1) flat out vec3 v_a;              // the segment's ends from the eye (on the flight path)
layout(location = 2) flat out vec3 v_b;
layout(location = 3) flat out vec3 v_y;              // the trail's frame: across (horizontal), up
layout(location = 4) flat out vec3 v_z;
layout(location = 5) flat out vec3 v_nA;             // the planes that end it, through v_a and v_b, along the trail
layout(location = 6) flat out vec3 v_nB;
layout(location = 7) flat out vec3 v_sunLight;       // the sun's irradiance and the sky's radiance there, through
layout(location = 8) flat out vec3 v_skyLight;       // the aerial perspective
layout(location = 9) flat out vec3 v_inscatter;      // the aerial perspective's light, per opacity
layout(location = 10) flat out vec4 v_plumeA;        // the samples' rows 1 to 3
layout(location = 11) flat out vec4 v_wakeA;
layout(location = 12) flat out vec4 v_moreA;
layout(location = 13) flat out vec4 v_plumeB;
layout(location = 14) flat out vec4 v_wakeB;
layout(location = 15) flat out vec4 v_moreB;
layout(location = 16) flat out vec4 v_trail;         // b0, the trail's phase, the secondary wake's extinction at
layout(location = 17) flat out vec2 v_secondary;     // each end
layout(location = 18) flat out vec3 v_yWake;         // the wake's frame: across and up as the aircraft was banked
layout(location = 19) flat out vec3 v_zWake;
layout(location = 20) flat out vec4 v_fire;          // a fire's: the plume's albedo at each end, its flame's mean
layout(location = 21) flat out vec2 v_flicker;       // temperature (K) at each end, how it fluctuates; the sun's disk's
layout(location = 22) flat out vec3 v_solar;         // luminance there
out gl_PerVertex { vec4 gl_Position; };

const int CAP = CONTRAIL_CAP, STRIDE = CONTRAIL_STRIDE;

void hide() {
  gl_Position = vec4(0.0, 0.0, -2.0, 1.0);
}

void main() {
  int src = gl_InstanceIndex / CAP, i = gl_InstanceIndex - src * CAP;
  int ia = (src * CAP + i) * STRIDE, ib = (src * CAP + (i + 1) % CAP) * STRIDE;
  vec4 a0 = samples[ia], b0 = samples[ib];
  vec4 pa = samples[ia + 1], pb = samples[ib + 1];
  // a segment joins two samples taken one after the other (a gap in the sequence is a gap in the trail)
  if (a0.w < 0.0 || b0.w != a0.w + 1.0) { hide(); return; }
  vec4 wa = samples[ia + 2], wb = samples[ib + 2], ma = samples[ia + 3], mb = samples[ib + 3], ta = samples[ia + 4], tb = samples[ib + 4];
  if (max(ta.w, tb.w) <= 0.0) { hide(); return; }
  // the neighbours, for the planes between segments
  vec4 before = samples[(src * CAP + (i + CAP - 1) % CAP) * STRIDE], after = samples[(src * CAP + (i + 2) % CAP) * STRIDE];

  vec3 A = a0.xyz, B = b0.xyz;
  vec3 axis = B - A;
  float L = length(axis);
  if (L < 1e-3) { hide(); return; }
  vec3 T = axis / L;
  vec3 M = 0.5 * (A + B);
  vec3 Z = normalize(F_eye.xyz + M);
  Z = Z - T * dot(Z, T);
  Z = length(Z) > 1e-4 ? normalize(Z) : normalize(cross(T, vec3(0.0, 0.0, 1.0)) + 1e-4);
  vec3 Y = cross(T, Z);
  // the wake's frame: the aircraft's up (at both ends) across the trail - its tubes and curtain lie as its wings did.
  // fsim: they were laid level whatever the bank, beside a banked aircraft's nozzles
  vec3 up = samples[ia + 5].xyz + samples[ib + 5].xyz;
  vec3 Zw = up - T * dot(up, T);
  Zw = length(Zw) > 1e-3 ? normalize(Zw) : Z;
  vec3 Yw = cross(T, Zw);
  vec3 nA = before.w == a0.w - 1.0 && length(B - before.xyz) > 1e-3 ? normalize(B - before.xyz) : T;
  vec3 nB = after.w == b0.w + 1.0 && length(after.xyz - A) > 1e-3 ? normalize(after.xyz - A) : T;

  // prefiltered by the pixel: a part thinner than one is widened by it, its extinction kept
  float sigmaPixel = 0.5 * F_eye.w * max(length(A), length(B));
  float pixel2 = sigmaPixel * sigmaPixel;
  // the box the segment's parts lie in (3.2 sigma round each, as the fragments bound them): across the track; from
  // above the flight path (the curtain's top, the plume's) to below it (the tubes' lowest, the plume's)
  // fsim: only of the parts that hold ice at either end (the fragments march no other): CoCiP's plume has its size
  // from the start, but no ice until the wake has dissolved into it - sized by it, the young trail's boxes were
  // hundreds of metres across, a camera near the aircraft stood inside dozens of them, and every pixel of the screen
  // marched through each (behind tactics.bvr's fighters, 18 screens' worth of fragments for a puff of trail)
  bool wakeIce = max(max(ma.z, mb.z), max(ta.z, tb.z)) > 0.0, plumeIce = max(pa.w, pb.w) > 0.0;
  float tube = max(ma.x, mb.x) + sigmaPixel;
  float crow = max(wa.z, wb.z);
  float halfDepth = max(0.5 * max(wa.y, wb.y), tube);
  float plumeY = plumeIce ? 3.2 * sqrt(max(pa.x, pb.x) + pixel2) : 0.0;
  float plumeZ = plumeIce ? 3.2 * sqrt(max(pa.y, pb.y) + pixel2) : 0.0;
  float plumeDrop = plumeIce ? max(ma.w, mb.w) : 0.0, plumeRise = plumeIce ? plumeZ - min(ma.w, mb.w) : 0.0;
  float across = max(wakeIce ? max(wa.x, wb.x) + crow + 3.2 * tube : 0.0, max(plumeY, 3.2 * 1.5 * tube));
  float top = max(max(3.2 * tube, wakeIce ? -halfDepth + 3.2 * sqrt(0.35) * halfDepth + sigmaPixel : 0.0), plumeRise);
  float bottom = max(wakeIce ? max(wa.y, wb.y) + crow + 3.2 * tube : 3.2 * tube, plumeDrop + plumeZ);
  // (the wake banked: its parts anywhere within their reach round the flight path)
  if (dot(Zw, Z) < 0.999) {
    float reach = length(vec2(across, max(top, bottom)));
    across = max(across, reach);
    top = max(top, reach);
    bottom = max(bottom, reach);
  }
  // (its ends a little past the samples: the planes between segments lean with the trail's bend)
  float lean = 0.1 * max(across, max(top, bottom)) + 1.0;

  // the box drawn as one strip of 14 vertices, its far faces only (the near ones culled): each pixel's ray through
  // the segment once
  uint bit = 1u << uint(gl_VertexIndex);
  vec3 corner = vec3((0x287au & bit) != 0u ? 1.0 : 0.0, (0x02afu & bit) != 0u ? 1.0 : 0.0, (0x31e3u & bit) != 0u ? 1.0 : 0.0);
  vec3 pos = A + T * mix(-lean, L + lean, corner.x) + Y * mix(-across, across, corner.y) + Z * mix(-bottom, top, corner.z);

  // the light at its middle
  vec3 sun = F_sun.xyz;
  vec3 at = (F_eye.xyz + M + F_correction.xyz) * METER_TO_LENGTH_UNIT;
  vec3 eye = (F_eye.xyz + F_correction.xyz) * METER_TO_LENGTH_UNIT;
  vec3 skyIrradiance;
  vec3 sunIrradiance = GetSunAndSkyScalarIrradiance(at, sun, skyIrradiance);
  vec3 transmittance;
  vec3 inscatter = GetSkyRadianceToPoint(eye, at, 0.0, sun, transmittance);
  // a fire's smoke, black: over a short path the aerial perspective is mostly the tables' interpolation error (as the
  // detonation's, world::Detonations), at most (1 - its transmittance) of the light reaching the air scattered to the eye
  vec4 fa = samples[ia + 5], fb = samples[ib + 5], ga = samples[ia + 6], gb = samples[ib + 6];
  bool fire = min(fa.w, fb.w) < 1.0;
  if (fire) inscatter = min(inscatter, (vec3(1.0) - transmittance) * (sunIrradiance + skyIrradiance) * RECIPROCAL_PI4 * 2.0);

  v_pos = pos;
  v_a = A;
  v_b = B;
  v_y = Y;
  v_z = Z;
  v_nA = nA;
  v_nB = nB;
  v_sunLight = sunIrradiance * transmittance;
  v_skyLight = skyIrradiance * RECIPROCAL_PI4 * transmittance;
  v_inscatter = inscatter;
  v_plumeA = pa;
  v_wakeA = wa;
  v_moreA = ma;
  v_plumeB = pb;
  v_wakeB = wb;
  v_moreB = mb;
  v_trail = vec4(ta.x, ta.y, 0.0, 0.0);
  v_secondary = vec2(ta.z, tb.z);
  v_yWake = Yw;
  v_zWake = Zw;
  v_fire = fire ? vec4(fa.w, fb.w, ga.x, gb.x) : vec4(1.0, 1.0, 0.0, 0.0);
  v_flicker = vec2(max(ga.y, gb.y), 0.0);
  v_solar = GetSolarLuminance() * transmittance;
  gl_Position = pc.projection * pc.modelView * vec4(pos, 1.0);
}
