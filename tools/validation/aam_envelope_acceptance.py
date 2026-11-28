#!/usr/bin/env python3
"""
24-AAM Envelope, Hit-Rate, Stability & Reticle-Performance Acceptance Suite.

Evaluates all 24 maintained air-to-air missiles in live 6-DOF native flight
through the built Python SDK against quantitative engineering gates:
  1. kin_10km   : 10 km MSL, M0.90 straight/lofted kinematic envelope
  2. headon_5km : 5 km clean head-on (6 km MSL, 250 vs 250 m/s)
  3. nom70_env  : 70% of nominal range_km (10 km MSL, 285 vs 255 m/s head-on)
  4. evade_7g   : 7.5G defensive break/beam turn (12 km BVR / 4 km WVR, no CM)
  5. evade_cm   : 7.5G defensive break/beam turn + 4x chaff/flare salvos
  6. hobs_3km   : 3 km High Off-Boresight (40 deg off-nose crossing target)

Usage:
  python tools/validation/aam_envelope_acceptance.py [--build build/ucrt64-release]
      [--missiles aim-120c,pl-15] [--workers 8] [--json out.json] [--markdown out.md] [--check]
"""
from __future__ import annotations

import argparse
import concurrent.futures
import json
import math
import os
import sys
import time
import traceback
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
EARTH_R = 6371000.0

HOBS_MISSILES = ("aim-9x", "iris-t", "asraam", "pl-10", "r-73", "mica-ir")
ARH_MISSILES = ("aim-120c", "aim-120d", "meteor", "mica-em", "pl-12", "pl-15", "r-37m", "r-77m")
RETICLE_MISSILES = ("aim-9m", "magic-2", "pl-8", "r-27et", "r-27t", "r-60m", "r-73")


def init_fsim(build_dir: Path):
    os.environ["FSIM_DLL_DIR"] = str(build_dir / "bin")
    os.environ["FSIM_AIRCRAFT_PATH"] = os.pathsep.join([
        str(build_dir / "share" / "flightsim" / "aircraft"),
        str(build_dir / "share" / "flightsim" / "aircraft" / "stores"),
    ])
    os.environ["PATH"] = (
        str(build_dir / "bin")
        + os.pathsep
        + r"D:\ENV\DevLanguages\Cpp\msys2\ucrt64\bin"
        + os.pathsep
        + os.environ.get("PATH", "")
    )
    py_dir = str(build_dir / "python")
    if py_dir not in sys.path:
        sys.path.insert(0, py_dir)
    import fsim
    return fsim


def load_carrier_map() -> dict[str, tuple[str, int]]:
    fits = {}
    for p in sorted((ROOT / "aircraft").glob("*/payloads.json")):
        fits[p.parent.name] = json.loads(p.read_text(encoding="utf-8"))

    payload_cards = {}
    for p in sorted((ROOT / "assets" / "payloads").glob("*.json")):
        d = json.loads(p.read_text(encoding="utf-8"))
        payload_cards[d["id"]] = d

    preferred_carriers = {
        "aim-120c": "f16c",
        "aim-120d": "f15c",
        "aim-7m": "f15c",
        "aim-9m": "f16c",
        "aim-9x": "f16c",
        "asraam": "typhoon",
        "iris-t": "typhoon",
        "magic-2": "mirage2000",
        "meteor": "typhoon",
        "mica-em": "rafale",
        "mica-ir": "rafale",
        "pl-10": "j20a",
        "pl-12": "j10a",
        "pl-15": "j20a",
        "pl-8": "j10a",
        "r-27er": "su27s",
        "r-27et": "su27s",
        "r-27r": "su27s",
        "r-27t": "su27s",
        "r-37m": "su57",
        "r-60m": "mig29a",
        "r-73": "su27s",
        "r-77m": "su57",
        "super-530d": "mirage2000",
    }

    result: dict[str, tuple[str, int]] = {}
    for sp in sorted((ROOT / "assets" / "stores").glob("*.json")):
        card = json.loads(sp.read_text(encoding="utf-8"))
        if card.get("class") != "aam":
            continue
        cid = card["id"]
        mass = float(card.get("parameters", {}).get("mass_kg", {}).get("value", 0.0))
        candidates = []
        for ac, fit in fits.items():
            has_radar = False
            st_card = None
            for pl in fit.get("payloads", []):
                pc = payload_cards.get(pl["card"])
                if not pc:
                    continue
                if pc.get("family") == "air_radar":
                    has_radar = True
                if pc.get("family") == "store_station":
                    st_card = pc
            if not has_radar or st_card is None:
                continue
            authored_stations = set()
            for ld in fit.get("loadouts", []):
                for st in ld.get("stations", []):
                    if st.get("card") == cid:
                        authored_stations.add(st["number"])
            for st in st_card.get("stations", []):
                if "aam" not in st.get("takes", []) or mass > st.get("max_kg", 0):
                    continue
                if st.get("capacity", 1) < 1:
                    continue
                pref_rank = 0 if ac == preferred_carriers.get(cid) else 1
                auth_rank = 0 if st["number"] in authored_stations else 1
                candidates.append((pref_rank, auth_rank, ac, st["number"]))
        _, _, ac, st_num = min(candidates)
        result[cid] = (ac, st_num)
    return result


def read_xml_propellant_kg(build_dir: Path, model_name: str) -> float:
    xml_path = build_dir / "share" / "flightsim" / "aircraft" / "stores" / model_name / f"{model_name}.xml"
    if not xml_path.exists():
        xml_path = ROOT / "aircraft" / "stores" / model_name / f"{model_name}.xml"
    if not xml_path.exists():
        return float("nan")
    try:
        root = ET.fromstring(xml_path.read_text(encoding="utf-8"))
        total_lbs = 0.0
        for tank in root.findall(".//tank"):
            c = tank.find("contents")
            if c is not None and c.text:
                unit = c.attrib.get("unit", "LBS").upper()
                val = float(c.text)
                total_lbs += val if unit == "LBS" else val / 0.45359237
        return round(total_lbs * 0.45359237, 2)
    except Exception:
        return float("nan")


def dist3(a, b) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def run_single_encounter(
    fsim,
    card: dict[str, Any],
    carrier: str,
    station_num: int,
    scenario_name: str,
    *,
    distance_m: float,
    alt_m: float,
    launcher_speed_ms: float,
    target_speed_ms: float,
    target_heading_deg: float,
    duration_s: float,
    bearing_offset_deg: float = 0.0,
    evade_g: float | None = None,
    evade_delay_s: float = 1.2,
    cm_salvos: list[float] | None = None,
    kinematic_mode: bool = False,
    min_kin_age_s: float = 18.0,
    wall_timeout_s: float = 45.0,
    frame_skip: int = 4,
) -> dict[str, Any]:
    cid = card["id"]
    seeker_type = card.get("seeker", "")
    is_ir = "infrared" in seeker_type
    is_sarh = seeker_type == "semi_active_radar"

    t0 = time.perf_counter()
    try:
        with fsim.World(
            f"acc-{cid}-{scenario_name}",
            publish=False,
            terrain=False,
            workers=1,
            pin_workers=False,
            seed=19,
            dt=1.0 / 120.0,
            frame_skip=frame_skip,
            jsbsim_root=str(ROOT / "third_party" / "jsbsim"),
        ) as w:
            own = w.create_vehicle(
                "launcher",
                "jsbsim:" + carrier,
                latitude_deg=40.0,
                longitude_deg=0.0,
                altitude_msl_m=alt_m,
                heading_deg=90.0,
                airspeed_ms=launcher_speed_ms,
            )
            launcher = w.create_agent(own, "launcher")
            launcher.submit(
                fsim.Level.VELOCITY,
                airspeed_ms=launcher_speed_ms,
                vertical_speed_ms=0.0,
                heading_rad=math.pi / 2.0,
            )

            station = next(p.index for p in launcher.payloads if p.family == "store_station")
            radar = next(p.index for p in launcher.payloads if p.family == "air_radar")
            own.load_store(station_num, cid, payload=station)
            for pl in (station, radar):
                launcher.set_payload_state(pl, "operate")
            for _ in range(math.ceil(25.0 / w.step_seconds)):
                w.step()
                if sum(p.state == fsim.PayloadState.OPERATE for p in launcher.payloads if p.index in (station, radar)) == 2:
                    break

            st = own.state
            az_rad = math.radians(90.0 + bearing_offset_deg)
            d_north = distance_m * math.cos(az_rad)
            d_east = distance_m * math.sin(az_rad)
            # For HOBS crossing engagements, pre-position the target back along relative
            # velocity by the short pre-launch search/power-up lead so that AT LAUNCH TIME
            # the target is genuinely at (distance_m, bearing_offset_deg) instead of having
            # crossed abeam/behind the wing before release.
            is_hobs = abs(bearing_offset_deg) >= 25.0
            if is_hobs:
                lead_s = 2.0
                th_rad = math.radians(target_heading_deg)
                v_rel_n = target_speed_ms * math.cos(th_rad) - launcher_speed_ms * math.cos(math.pi / 2.0)
                v_rel_e = target_speed_ms * math.sin(th_rad) - launcher_speed_ms * math.sin(math.pi / 2.0)
                d_north -= v_rel_n * lead_s
                d_east -= v_rel_e * lead_s
            dlat_deg = math.degrees(d_north / EARTH_R)
            dlon_deg = math.degrees(d_east / (EARTH_R * math.cos(st.latitude_rad)))

            if distance_m >= 180000.0:
                target_model = "jsbsim:b52h"
            elif distance_m >= 125000.0:
                target_model = "jsbsim:su27s"
            else:
                target_model = "jsbsim:f16c"
            target = w.create_vehicle(
                "target",
                target_model,
                latitude_deg=math.degrees(st.latitude_rad) + dlat_deg,
                longitude_deg=math.degrees(st.longitude_rad) + dlon_deg,
                altitude_msl_m=alt_m,
                heading_deg=target_heading_deg,
                airspeed_ms=target_speed_ms,
            )
            opp = w.create_agent(target, "target")
            opp.submit(
                fsim.Level.VELOCITY,
                airspeed_ms=target_speed_ms,
                vertical_speed_ms=0.0,
                heading_rad=math.radians(target_heading_deg),
            )
            dispenser = next((p.index for p in opp.payloads if p.family == "countermeasures"), None)
            if cm_salvos and dispenser is not None:
                opp.set_payload_state(dispenser, "operate")

            rscale = max(60000.0, min(distance_m * 1.5, 420000.0))
            az_hw = max(math.radians(15.0), math.radians(abs(bearing_offset_deg) + 12.0))
            az_hw = min(az_hw, math.radians(55.0))
            el_hw = math.radians(6.0)
            launcher.amti_search(
                radar,
                azimuth_half_width_rad=az_hw,
                elevation_half_width_rad=el_hw,
                range_scale_m=rscale,
            )
            rec_model = card.get("component_models", {}).get("components", {}).get("seeker", {}).get("bindings", {}).get("receiver_model", "")
            is_reticle = (rec_model == "thermal_conscan_reticle.v1")
            track = None
            if is_reticle and not is_hobs and distance_m < 30000.0:
                search_time = 1.2
            elif is_hobs:
                search_time = 1.8
            elif is_sarh and distance_m >= 30000.0:
                search_time = 12.0
            elif distance_m >= 30000.0:
                search_time = 10.0
            else:
                search_time = 5.0
            for _ in range(math.ceil(search_time / w.step_seconds)):
                w.step()
                tracks = launcher.amti_tracks(radar)
                if tracks:
                    track = max(tracks, key=lambda tr: tr.updates)
                    if track.updates >= (1 if (is_hobs or is_reticle) else 2):
                        break
                if time.perf_counter() - t0 > wall_timeout_s * 0.5:
                    break

            if track is not None:
                launcher.amti_track(track.id, radar, illuminate=is_sarh, range_scale_m=rscale)
                desig = launcher.designate(radar, track.id)
                target_arg = desig if desig else target.id
            else:
                target_arg = target.id

            try:
                own.strike(cid, target=target_arg, payload=station, consent=True)
            except Exception as exc:
                return dict(
                    scenario=scenario_name,
                    released=False,
                    had_radar_track=(track is not None),
                    reason=f"STRIKE_REJECTED:{type(exc).__name__}",
                    error=str(exc),
                    wall_s=round(time.perf_counter() - t0, 2),
                )

            weapon_id = None
            for _ in range(math.ceil(8.0 / w.step_seconds)):
                w.step()
                if w.weapons:
                    weapon_id = int(w.weapons[-1].id)
                    break
                if time.perf_counter() - t0 > wall_timeout_s:
                    return dict(
                        scenario=scenario_name,
                        released=False,
                        had_radar_track=(track is not None),
                        reason="WALL_TIMEOUT_PRE_RELEASE",
                        wall_s=round(time.perf_counter() - t0, 2),
                    )
            if weapon_id is None:
                return dict(
                    scenario=scenario_name,
                    released=False,
                    had_radar_track=(track is not None),
                    reason="NOT_RELEASED",
                    wall_s=round(time.perf_counter() - t0, 2),
                )

            rec0 = next(r for r in w.weapons if r.id == weapon_id)
            launch_t = float(rec0.launch_time_s)
            launch_pos = tuple(w.vehicle(weapon_id).state.position_ecef)
            launch_range_m = dist3(launch_pos, target.state.position_ecef)

            cm_fired: set[int] = set()
            max_mach = 0.0
            max_g = 0.0
            max_alpha_deg = 0.0
            max_beta_deg = 0.0
            max_ground_dist_m = 0.0
            supersonic_reach_m = 0.0
            effective_reach_m = 0.0
            effective_time_s = 0.0
            min_live_dist_m = float("inf")
            ever_tracked = False
            ever_active = False
            ever_supported = False
            lost_after_track = False
            lost_inside_750m = False
            reverse_flow = False
            diverged = False
            burn_time_s = 0.0
            peak_thrust_n = 0.0
            terminal_mach = 0.0
            terminal_alt_m = 0.0
            max_alt_m = alt_m
            target_peak_g = 0.0
            wall_timed_out = False
            evade_roll_done = False

            while w.time < launch_t + duration_s:
                if time.perf_counter() - t0 > wall_timeout_s:
                    wall_timed_out = True
                    break

                age = w.time - launch_t
                if evade_g is not None and age >= evade_delay_s:
                    roll_deg = math.degrees(target.state.roll_rad)
                    if not evade_roll_done and roll_deg < 80.0:
                        rr = 2.6
                    else:
                        evade_roll_done = True
                        rr = (math.radians(82.0) - target.state.roll_rad) * 2.0
                    target.command_acceleration(load_factor_g=evade_g, roll_rate_rad_s=rr, throttle=1.0)

                if cm_salvos and dispenser is not None:
                    kind = "flare" if is_ir else "chaff"
                    for idx_cm, t_cm in enumerate(cm_salvos):
                        if idx_cm not in cm_fired and age >= t_cm:
                            try:
                                opp.dispense(kind)
                            except Exception:
                                pass
                            cm_fired.add(idx_cm)

                rec = next(r for r in w.weapons if r.id == weapon_id)
                ever_tracked = ever_tracked or bool(getattr(rec, "tracking", False))
                ever_active = ever_active or bool(getattr(rec, "active", False)) or (int(rec.phase) >= 2)
                ever_supported = ever_supported or bool(getattr(rec, "supported", False))

                ts = target.state
                target_peak_g = max(target_peak_g, abs(float(ts.load_factor)))

                if rec.end != "in_flight":
                    for _ in range(2):
                        w.step()
                    break

                ms_veh = w.vehicle(weapon_id)
                s = ms_veh.state
                if s.diverged or not math.isfinite(s.mach):
                    diverged = True
                    break
                if math.cos(s.alpha_rad) * math.cos(s.beta_rad) <= 0.0:
                    reverse_flow = True

                mach = float(s.mach)
                spd = float(s.airspeed_true_ms)
                g_load = abs(float(s.load_factor))
                max_mach = max(max_mach, mach)
                max_g = max(max_g, g_load)
                max_alpha_deg = max(max_alpha_deg, abs(math.degrees(s.alpha_rad)))
                max_beta_deg = max(max_beta_deg, abs(math.degrees(s.beta_rad)))
                max_alt_m = max(max_alt_m, float(s.altitude_msl_m))
                terminal_mach = mach
                terminal_alt_m = float(s.altitude_msl_m)

                thrust = sum(float(x) for x in list(s.thrust_n)[: max(1, int(s.engine_count))])
                if thrust > 50.0:
                    burn_time_s += w.step_seconds
                    peak_thrust_n = max(peak_thrust_n, thrust)

                flown_m = dist3(launch_pos, s.position_ecef)
                max_ground_dist_m = max(max_ground_dist_m, flown_m)
                if mach >= 1.0:
                    supersonic_reach_m = max(supersonic_reach_m, flown_m)
                if spd >= 240.0:
                    effective_reach_m = max(effective_reach_m, flown_m)
                    effective_time_s = max(effective_time_s, age)

                d_tgt = dist3(s.position_ecef, ts.position_ecef)
                min_live_dist_m = min(min_live_dist_m, d_tgt)

                if ever_tracked and not rec.tracking and rec.end == "in_flight":
                    lost_after_track = True
                    if 100.0 <= d_tgt <= 750.0:
                        lost_inside_750m = True

                if kinematic_mode and age > min_kin_age_s and thrust < 10.0 and spd < 210.0:
                    break

                w.step()

            final = next(r for r in w.weapons if r.id == weapon_id)
            end_str = str(final.end)
            miss_m = float(final.miss_m) if math.isfinite(final.miss_m) else min_live_dist_m
            tof_s = (float(final.end_time_s) - launch_t) if math.isfinite(final.end_time_s) else (w.time - launch_t)

            damaged = False
            destroyed = False
            for ev in w.engagement_events():
                if getattr(ev, "target", 0) == target.id and getattr(ev, "kind", "") == "damage":
                    damaged = True
                    if getattr(ev, "destroyed", False):
                        destroyed = True

            reason = "HIT"
            if end_str != "detonated":
                if wall_timed_out:
                    reason = f"WALL_TO_d{int(min_live_dist_m)}m"
                elif diverged:
                    reason = "DIVERGED"
                elif reverse_flow:
                    reason = "AERO_DEPARTURE"
                elif end_str == "in_flight":
                    reason = "LOW_ENERGY_TIMEOUT" if terminal_mach < 0.85 else f"PASSED_CPA_{int(min_live_dist_m)}m"
                elif end_str == "impact":
                    reason = "GROUND_LOW_ENERGY" if terminal_alt_m < 1000.0 else "IMPACT_MISS"
                elif end_str == "expired":
                    reason = "LIFE_EXPIRED"
                elif not ever_tracked:
                    reason = "NO_SEEKER_LOCK"
                elif lost_after_track:
                    reason = "LOST_TRACK"
                else:
                    reason = f"MISS_{round(miss_m,1)}m"

            headon_reach_km = (effective_reach_m + target_speed_ms * effective_time_s) / 1000.0
            return dict(
                scenario=scenario_name,
                released=True,
                had_radar_track=(track is not None),
                launch_range_km=round(launch_range_m / 1000.0, 2),
                end=end_str,
                hit=(end_str == "detonated"),
                damaged=damaged,
                destroyed=destroyed,
                miss_m=round(miss_m, 2) if math.isfinite(miss_m) else None,
                min_live_m=round(min_live_dist_m, 2) if math.isfinite(min_live_dist_m) else None,
                tof_s=round(tof_s, 2),
                peak_mach=round(max_mach, 2),
                terminal_mach=round(terminal_mach, 2),
                peak_g=round(max_g, 1),
                target_peak_g=round(target_peak_g, 1),
                peak_alpha_deg=round(max_alpha_deg, 1),
                peak_beta_deg=round(max_beta_deg, 1),
                max_alt_m=round(max_alt_m, 0),
                burn_s=round(burn_time_s, 2),
                peak_thrust_kn=round(peak_thrust_n / 1000.0, 1),
                flown_km=round(max_ground_dist_m / 1000.0, 2),
                eff_reach_km=round(effective_reach_m / 1000.0, 2),
                headon_reach_km=round(headon_reach_km, 2),
                supersonic_km=round(supersonic_reach_m / 1000.0, 2),
                ever_tracked=ever_tracked,
                ever_active=ever_active,
                ever_supported=ever_supported,
                lost_after_track=lost_after_track,
                lost_inside_750m=lost_inside_750m,
                reverse_flow=reverse_flow,
                diverged=diverged,
                wall_timed_out=wall_timed_out,
                reason=reason,
                wall_s=round(time.perf_counter() - t0, 2),
            )
    except Exception as exc:
        return dict(
            scenario=scenario_name,
            released=False,
            reason=f"EXC:{type(exc).__name__}",
            error=f"{exc}\n{traceback.format_exc(limit=3)}",
            wall_s=round(time.perf_counter() - t0, 2),
        )


def benchmark_one_missile(args: tuple[str, str, int, float, str]) -> dict[str, Any]:
    cid, carrier, station_num, wall_to, build_str = args
    build_dir = Path(build_str)
    fsim = init_fsim(build_dir)
    card = json.loads((ROOT / "assets" / "stores" / f"{cid}.json").read_text(encoding="utf-8"))
    ps = card.get("parameters", {})

    def pv(k, default=None):
        return ps.get(k, {}).get("value", default)

    nom_range_km = float(pv("range_km", 20.0))
    nom_mach = float(pv("speed_mach", 3.0))
    nom_mass_kg = float(pv("mass_kg", 100.0))
    seeker = card.get("seeker", "")
    model_name = card.get("model", "").replace("jsbsim:", "")
    prop_kg = read_xml_propellant_kg(build_dir, model_name)

    comp_map = card.get("component_models", {}).get("components", {})

    def role_val(comp_name: str, role: str, default=None):
        c = comp_map.get(comp_name, {})
        ref = c.get("parameter_refs", {}).get(role)
        if ref and ref in ps:
            return ps[ref].get("value", default)
        return default

    acquire_m = role_val("seeker", "acquire_m")
    gimbal_deg = role_val("seeker", "gimbal_deg")
    launch_fov_deg = role_val("seeker", "launch_fov_deg")
    max_g_cfg = role_val("guidance", "max_g")
    life_s_cfg = float(role_val("guidance", "life_s", 110.0))
    loft_deg_cfg = role_val("guidance", "loft_deg")
    fuze_r_m = role_val("fuze", "radius_m") or pv("fuze_radius_m")
    receiver_model = comp_map.get("seeker", {}).get("bindings", {}).get("receiver_model", "")
    inertial_mid = bool(card.get("guidance_capabilities", {}).get("inertial_midcourse", False))

    is_ir = "infrared" in seeker
    is_wvr = is_ir and nom_range_km <= 35.0

    # 1. Kinematic envelope at 10 km MSL, M0.90 launch
    kin_dist_km = max(nom_range_km * 1.15, 25.0)
    if seeker == "semi_active_radar":
        # Keep SARH kinematic test inside carrier illumination radar detection envelope.
        kin_dist_km = min(kin_dist_km, 92.0)
    kin_dur = min(life_s_cfg + 5.0, 360.0)
    min_kin_age = 115.0 if cid in ("pl-15", "r-37m", "meteor", "r-77m") else 20.0

    # 3. 70% nominal range head-on at 10 km MSL (capped to seeker lock envelope for non-midcourse LOBL IR)
    nom70_km = round(nom_range_km * 0.70, 1)
    if is_ir and not inertial_mid and acquire_m:
        # LOBL IR missiles cannot be fired beyond their own optical lock-on envelope
        nom70_km = round(min(nom70_km, float(acquire_m) * 0.85 / 1000.0), 1)
    nom70_dur = min(life_s_cfg + 5.0, 360.0)

    # 4 & 5. Realistic 7.5G defensive break turn (12 km BVR / 4 km WVR)
    evade_km = 4.0 if is_wvr else 12.0
    evade_alt = 6000.0 if is_wvr else 8000.0
    evade_dur = 25.0 if is_wvr else 40.0
    evade_delay = 0.6 if is_wvr else 2.5

    runs = {}
    runs["kin_10km"] = run_single_encounter(
        fsim, card, carrier, station_num, "kin_10km",
        distance_m=kin_dist_km * 1000.0, alt_m=10000.0,
        launcher_speed_ms=270.0, target_speed_ms=250.0, target_heading_deg=270.0,
        duration_s=kin_dur, kinematic_mode=True, min_kin_age_s=min_kin_age,
        wall_timeout_s=wall_to,
    )
    runs["headon_5km"] = run_single_encounter(
        fsim, card, carrier, station_num, "headon_5km",
        distance_m=5000.0, alt_m=6000.0,
        launcher_speed_ms=250.0, target_speed_ms=250.0, target_heading_deg=270.0,
        duration_s=22.0, wall_timeout_s=wall_to,
    )
    runs["nom70_env"] = run_single_encounter(
        fsim, card, carrier, station_num, "nom70_env",
        distance_m=nom70_km * 1000.0, alt_m=10000.0,
        launcher_speed_ms=285.0, target_speed_ms=255.0, target_heading_deg=270.0,
        duration_s=nom70_dur, wall_timeout_s=wall_to,
    )
    runs["evade_7g"] = run_single_encounter(
        fsim, card, carrier, station_num, "evade_7g",
        distance_m=evade_km * 1000.0, alt_m=evade_alt,
        launcher_speed_ms=270.0, target_speed_ms=265.0, target_heading_deg=270.0,
        duration_s=evade_dur, evade_g=7.5, evade_delay_s=evade_delay, wall_timeout_s=wall_to,
    )
    runs["evade_cm"] = run_single_encounter(
        fsim, card, carrier, station_num, "evade_cm",
        distance_m=evade_km * 1000.0, alt_m=evade_alt,
        launcher_speed_ms=270.0, target_speed_ms=265.0, target_heading_deg=270.0,
        duration_s=evade_dur, evade_g=7.5, evade_delay_s=evade_delay,
        cm_salvos=[evade_delay, evade_delay + 1.0, evade_delay + 2.0, evade_delay + 3.0],
        wall_timeout_s=wall_to,
    )
    runs["hobs_3km"] = run_single_encounter(
        fsim, card, carrier, station_num, "hobs_3km",
        distance_m=3000.0, alt_m=6000.0,
        launcher_speed_ms=250.0, target_speed_ms=250.0, target_heading_deg=180.0,
        duration_s=20.0, bearing_offset_deg=40.0, wall_timeout_s=wall_to,
    )

    return dict(
        id=cid,
        designation=card.get("designation", cid),
        carrier=carrier,
        station=station_num,
        seeker=seeker,
        receiver_model=receiver_model,
        nom_mass_kg=nom_mass_kg,
        xml_prop_kg=prop_kg,
        nom_range_km=nom_range_km,
        nom_mach=nom_mach,
        acquire_m=acquire_m,
        gimbal_deg=gimbal_deg,
        launch_fov_deg=launch_fov_deg,
        max_g_cfg=max_g_cfg,
        life_s_cfg=life_s_cfg,
        loft_deg_cfg=loft_deg_cfg,
        fuze_r_m=round(float(fuze_r_m), 2) if fuze_r_m else None,
        runs=runs,
    )


def fmt_row(res: dict[str, Any]) -> str:
    r = res["runs"]
    kin = r["kin_10km"]
    h5 = r["headon_5km"]
    n70 = r["nom70_env"]
    ev7 = r["evade_7g"]
    evc = r["evade_cm"]
    hob = r["hobs_3km"]
    combat = (h5, n70, ev7, evc, hob)
    hits = sum(1 for x in combat if x.get("hit"))
    peak_g_all = max((x.get("peak_g", 0.0) for x in r.values() if x.get("released")), default=0.0)

    def tag(x):
        if x.get("hit"):
            return f"HIT({x.get('miss_m',0):.1f}m)"
        return str(x.get("reason", "ERR"))[:15]

    return (
        f"{res['id']:10s} | nom M={res['nom_mach']:.1f} R={res['nom_range_km']:5.1f}km | "
        f"act M={kin.get('peak_mach',0):4.2f} r={kin.get('eff_reach_km',0):5.1f}/{kin.get('headon_reach_km',0):5.1f}km pkG={peak_g_all:4.1f} | "
        f"5km:{tag(h5):11s} 70%({n70.get('launch_range_km',0):5.1f}):{tag(n70):14s} "
        f"7G:{tag(ev7):14s} CM:{tag(evc):14s} HOBS:{tag(hob):14s} [{hits}/5] "
        f"t={sum(x.get('wall_s',0) for x in r.values()):4.1f}s"
    )


def evaluate_gates(results: list[dict[str, Any]]) -> tuple[bool, list[str]]:
    by_id = {r["id"]: r for r in results}
    n = len(results)
    h5_hits = sum(1 for r in results if r["runs"]["headon_5km"].get("hit"))
    n70_hits = sum(1 for r in results if r["runs"]["nom70_env"].get("hit"))
    ev7_hits = sum(1 for r in results if r["runs"]["evade_7g"].get("hit"))
    evc_hits = sum(1 for r in results if r["runs"]["evade_cm"].get("hit"))
    departures = [
        f"{r['id']}:{sc}"
        for r in results
        for sc, run in r["runs"].items()
        if run.get("reverse_flow") or run.get("diverged")
    ]
    arh_blinds = [
        r["id"]
        for r in results
        if r["id"] in ARH_MISSILES and r["runs"]["evade_7g"].get("lost_inside_750m") and not r["runs"]["evade_7g"].get("hit")
    ]
    hobs_checked = [cid for cid in HOBS_MISSILES if cid in by_id]
    hobs_hits = sum(1 for cid in hobs_checked if by_id[cid]["runs"]["hobs_3km"].get("hit"))
    reticle_checked = [cid for cid in RETICLE_MISSILES if cid in by_id]
    reticle_slow = [
        f"{cid}:{sc}={run.get('wall_s')}s(tof={run.get('tof_s')}s)"
        for cid in reticle_checked
        for sc, run in by_id[cid]["runs"].items()
        if run.get("wall_timed_out") or run.get("wall_s", 0.0) > max(40.0, 3.5 * float(run.get("tof_s") or 0.0))
    ]
    mach_outliers = []
    for r in results:
        pm = r["runs"]["kin_10km"].get("peak_mach", 0.0)
        nm = r["nom_mach"]
        if nm > 0 and (pm < 0.80 * nm or pm > 1.25 * nm):
            mach_outliers.append(f"{r['id']}(act={pm:.2f},nom={nm:.2f})")

    lines = [
        f"headon_5km : {h5_hits}/{n} HIT (gate: {n}/{n})",
        f"nom70_env  : {n70_hits}/{n} HIT (gate: >= {math.ceil(0.83*n)}/{n})",
        f"evade_7g   : {ev7_hits}/{n} HIT (gate: >= {math.ceil(0.83*n)}/{n})",
        f"evade_cm   : {evc_hits}/{n} HIT (non-degenerate realism check)",
        f"hobs_3km   : {hobs_hits}/{len(hobs_checked)} HOBS HIT (gate: {len(hobs_checked)}/{len(hobs_checked)})",
        f"departures : {len(departures)} {departures}",
        f"arh_blind  : {len(arh_blinds)} {arh_blinds}",
        f"reticle_stl: {len(reticle_slow)} {reticle_slow}",
        f"mach_band  : {len(mach_outliers)} outliers {mach_outliers}",
    ]
    ok = (
        h5_hits == n
        and n70_hits >= math.ceil(0.83 * n)
        and ev7_hits >= math.ceil(0.83 * n)
        and hobs_hits == len(hobs_checked)
        and len(departures) == 0
        and len(arh_blinds) == 0
        and len(reticle_slow) == 0
        and len(mach_outliers) == 0
    )
    return ok, lines


def main() -> int:
    parser = argparse.ArgumentParser(description="24-AAM Envelope & Hit-Rate Acceptance Suite")
    parser.add_argument("--build", default=str(ROOT / "build" / "ucrt64-release"))
    parser.add_argument("--missiles", default="", help="Comma-separated subset of store IDs")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--json", dest="json_out", default="")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    build_dir = Path(args.build).resolve()
    init_fsim(build_dir)
    t_start = time.perf_counter()
    carriers = load_carrier_map()
    selected = {x.strip() for x in args.missiles.split(",") if x.strip()} if args.missiles else set(carriers)

    fast_tasks = []
    reticle_tasks = []
    for cid, (ac, st) in sorted(carriers.items()):
        if cid not in selected:
            continue
        card = json.loads((ROOT / "assets" / "stores" / f"{cid}.json").read_text(encoding="utf-8"))
        rec_mod = card.get("component_models", {}).get("components", {}).get("seeker", {}).get("bindings", {}).get("receiver_model", "")
        if rec_mod == "thermal_conscan_reticle.v1":
            reticle_tasks.append((cid, ac, st, 90.0, str(build_dir)))
        else:
            fast_tasks.append((cid, ac, st, 180.0, str(build_dir)))

    results_by_id: dict[str, dict[str, Any]] = {}
    if fast_tasks:
        print(f"Running {len(fast_tasks)} fast AAMs (ARH/SARH/IIR) on {min(args.workers, len(fast_tasks))} workers...", flush=True)
        with concurrent.futures.ProcessPoolExecutor(max_workers=min(args.workers, len(fast_tasks))) as pool:
            futs = {pool.submit(benchmark_one_missile, t): t[0] for t in fast_tasks}
            for fut in concurrent.futures.as_completed(futs):
                res = fut.result()
                results_by_id[res["id"]] = res
                print(fmt_row(res), flush=True)

    if reticle_tasks:
        print(f"\nRunning {len(reticle_tasks)} conscan-reticle IR AAMs sequentially...", flush=True)
        for t in reticle_tasks:
            res = benchmark_one_missile(t)
            results_by_id[res["id"]] = res
            print(fmt_row(res), flush=True)

    ordered = [results_by_id[cid] for cid in sorted(results_by_id)]
    if args.json_out:
        Path(args.json_out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json_out).write_text(json.dumps(ordered, indent=2) + "\n", encoding="utf-8")

    ok, lines = evaluate_gates(ordered)
    print(f"\n=== Acceptance Summary ({len(ordered)} AAMs in {time.perf_counter()-t_start:.1f}s) ===", flush=True)
    for ln in lines:
        print("  " + ln, flush=True)
    print(f"OVERALL: {'PASS' if ok else 'FAIL'}", flush=True)
    if args.check and not ok:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
