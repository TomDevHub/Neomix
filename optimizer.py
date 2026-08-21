"""Builds a driving strategy that maximises the judge's score for a level.

The optimizer is analytic rather than search-based: brake points are derived
from kinematics, the fastest legal tyre is chosen for the weather, and fuel is
modelled in closed form so pit stops can be sized exactly.
"""

import math
from simulator import WEATHER_FRICTION_KEY, max_corner_speed, fuel_consumed

# Safety margins tuned against the judge's numerics.
#   BRAKE_SAFETY_M = 0: the simulator is deterministic, so braking to exactly
#     the corner limit lands within its 1e-9 crash-check tolerance. Any positive
#     margin would only cost time.
#   FUEL_SAFETY_L = 0.05: comfortably above the worst-case error from rounding
#     the refuel amount to two decimal places, so we never limp on the last lap.
BRAKE_SAFETY_M = 0.0
FUEL_SAFETY_L = 0.05


def optimize(level_data):
    """Dispatch to the optimizer for the level named in the data."""
    level_name = level_data["race"].get("name", "")
    if "Level 1" in level_name:
        return _optimize_level1(level_data)
    if "Level 2" in level_name:
        return _optimize_level2(level_data)
    raise ValueError(f"No optimizer implemented for: {level_name}")


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def _best_tyre_for_weather(level_data, weather_cond):
    """Return the id of the available tyre set with the most grip in this weather."""
    tyre_props_map = level_data["tyres"]["properties"]
    best_id = None
    best_friction = -1.0
    for tyre_set in level_data.get("available_sets", []):
        props = tyre_props_map[tyre_set["compound"]]
        friction = props["life_span"] * props[WEATHER_FRICTION_KEY[weather_cond]]
        if friction > best_friction:
            best_friction = friction
            best_id = tyre_set["ids"][0]
    return best_id


def _binding_corner_speed(straight_index, segments, corner_max_speeds):
    """Speed a straight must brake down to before its next run of corners.

    Returns the lowest limit among the corners immediately following the
    straight. If the straight is the last segment, the run wraps around to the
    corners at the start of the lap.
    """
    n = len(segments)
    min_speed = float("inf")
    for j in range(straight_index + 1, n):
        if segments[j]["type"] == "corner":
            min_speed = min(min_speed, corner_max_speeds[segments[j]["id"]])
        else:
            break
    if min_speed == float("inf"):
        for j in range(n):
            if segments[j]["type"] == "corner":
                min_speed = min(min_speed, corner_max_speeds[segments[j]["id"]])
            else:
                break
    return min_speed


def _braking_distance(v_from, v_to, eff_brake):
    """Minimum distance needed to decelerate from v_from to v_to."""
    if v_from <= v_to:
        return 0.0
    return (v_from ** 2 - v_to ** 2) / (2 * eff_brake)


def _build_segment_template(level_data, weather_cond, eff_brake, props,
                            max_speed, crawl_speed):
    """Per-segment plan: full speed on straights, latest safe brake points.

    The plan is identical every lap (weather and tyre are fixed), so it is
    computed once and reused across all laps.
    """
    segments = level_data["track"]["segments"]
    friction = props["life_span"] * props[WEATHER_FRICTION_KEY[weather_cond]]

    corner_max_speeds = {
        seg["id"]: max_corner_speed(friction, seg["radius_m"], crawl_speed)
        for seg in segments if seg["type"] == "corner"
    }

    template = []
    for i, seg in enumerate(segments):
        if seg["type"] == "straight":
            required_exit = _binding_corner_speed(i, segments, corner_max_speeds)
            brake_dist = _braking_distance(max_speed, required_exit, eff_brake)
            brake_dist += BRAKE_SAFETY_M
            brake_dist = min(brake_dist, seg["length_m"] * 0.99)
            template.append({
                "id": seg["id"],
                "type": "straight",
                "target_m/s": max_speed,
                "brake_start_m_before_next": brake_dist,
            })
        else:
            template.append({"id": seg["id"], "type": "corner"})
    return template


def _simulate_lap_fuel(level_data, seg_template, entry_speed):
    """Return (exit_speed, fuel_used) for one lap driven to the template.

    This is a lightweight copy of the straight/corner kinematics in the
    simulator, used to size fuel loads without running the full scored race.
    """
    car = level_data["car"]
    k_base = car["fuel_consumption_l/m"]
    accel = car["accel_m/se2"]
    brake_rate = car["brake_m/se2"]

    seg_map = {s["id"]: s for s in seg_template}
    speed = entry_speed
    total_fuel = 0.0

    for seg in level_data["track"]["segments"]:
        action = seg_map[seg["id"]]
        length = seg["length_m"]

        if seg["type"] == "straight":
            target = action.get("target_m/s", car["max_speed_m/s"])
            brake_start = action.get("brake_start_m_before_next", 0.0)
            brake_point = max(0.0, length - brake_start)
            entry = speed

            if entry < target:
                accel_dist = (target ** 2 - entry ** 2) / (2 * accel)
                if accel_dist >= brake_point:
                    v_at_brake = math.sqrt(entry ** 2 + 2 * accel * brake_point)
                    total_fuel += fuel_consumed(entry, v_at_brake, brake_point, k_base)
                else:
                    total_fuel += fuel_consumed(entry, target, accel_dist, k_base)
                    cruise_dist = brake_point - accel_dist
                    if cruise_dist > 0:
                        total_fuel += fuel_consumed(target, target, cruise_dist, k_base)
                    v_at_brake = target
            else:
                v_at_brake = entry
                if brake_point > 0:
                    total_fuel += fuel_consumed(entry, entry, brake_point, k_base)

            exit_speed = math.sqrt(max(0.0, v_at_brake ** 2 - 2 * brake_rate * brake_start))
            if brake_start > 0:
                total_fuel += fuel_consumed(v_at_brake, exit_speed, brake_start, k_base)
            speed = exit_speed
        else:
            total_fuel += fuel_consumed(speed, speed, length, k_base)

    return speed, total_fuel


def _steady_state_lap_fuel(level_data, seg_template):
    """Converge lap entry speed to its steady state and return (speed, fuel).

    After the first lap the car crosses the line at a repeating speed. A few
    iterations are enough for it to settle.
    """
    speed = 0.0
    for _ in range(6):
        speed, _ = _simulate_lap_fuel(level_data, seg_template, speed)
    _, fuel = _simulate_lap_fuel(level_data, seg_template, speed)
    return speed, fuel


# ---------------------------------------------------------------------------
# Level 1: fastest time, fuel unlimited
# ---------------------------------------------------------------------------

def _optimize_level1(level_data):
    car = level_data["car"]
    race = level_data["race"]
    weather_conditions = level_data["weather"]["conditions"]

    max_speed = car["max_speed_m/s"]
    crawl_speed = car["crawl_constant_m/s"]
    brake = car["brake_m/se2"]
    num_laps = race["laps"]

    starting_weather = next(c for c in weather_conditions
                            if c["id"] == race["starting_weather_condition_id"])
    w_cond = starting_weather["condition"]
    eff_brake = brake * starting_weather["deceleration_multiplier"]

    tyre_id = _best_tyre_for_weather(level_data, w_cond)
    props = _tyre_props(level_data, tyre_id)

    seg_template = _build_segment_template(level_data, w_cond, eff_brake,
                                           props, max_speed, crawl_speed)

    laps = [
        {"lap": lap, "segments": seg_template, "pit": {"enter": False}}
        for lap in range(1, num_laps + 1)
    ]
    return {"initial_tyre_id": tyre_id, "laps": laps}


# ---------------------------------------------------------------------------
# Level 2: fastest time plus a fuel-usage bonus, so pit stops matter
# ---------------------------------------------------------------------------

def _optimize_level2(level_data):
    car = level_data["car"]
    race = level_data["race"]
    weather_conditions = level_data["weather"]["conditions"]

    max_speed = car["max_speed_m/s"]
    crawl_speed = car["crawl_constant_m/s"]
    brake = car["brake_m/se2"]
    tank_cap = car["fuel_tank_capacity_l"]
    initial_fuel = car["initial_fuel_l"]
    num_laps = race["laps"]
    pit_exit_speed = race["pit_exit_speed_m/s"]

    starting_weather = next(c for c in weather_conditions
                            if c["id"] == race["starting_weather_condition_id"])
    w_cond = starting_weather["condition"]
    eff_brake = brake * starting_weather["deceleration_multiplier"]

    tyre_id = _best_tyre_for_weather(level_data, w_cond)
    props = _tyre_props(level_data, tyre_id)

    seg_template = _build_segment_template(level_data, w_cond, eff_brake,
                                           props, max_speed, crawl_speed)

    # Fuel needed for a normal lap in steady state, and for the lap that starts
    # slowly from the pit exit.
    _, steady_lap_fuel = _steady_state_lap_fuel(level_data, seg_template)
    _, post_pit_lap_fuel = _simulate_lap_fuel(level_data, seg_template, pit_exit_speed)

    laps = []
    fuel = initial_fuel
    speed = 0.0

    for lap in range(1, num_laps + 1):
        exit_speed, lap_fuel = _simulate_lap_fuel(level_data, seg_template, speed)
        fuel_after = fuel - lap_fuel
        laps_remaining = num_laps - lap
        pit = {"enter": False}

        if laps_remaining > 0:
            fuel_to_finish = laps_remaining * steady_lap_fuel

            if fuel_after >= fuel_to_finish:
                # Enough fuel to reach the flag without stopping.
                fuel = fuel_after
                speed = exit_speed
            elif fuel_after < steady_lap_fuel + FUEL_SAFETY_L:
                # Would risk running dry next lap: pit now and refuel exactly
                # enough to finish (capped by tank size).
                fuel_for_rest = (post_pit_lap_fuel
                                 + max(0, laps_remaining - 1) * steady_lap_fuel)
                refuel = fuel_for_rest - max(0.0, fuel_after) + FUEL_SAFETY_L
                refuel = max(0.0, min(refuel, tank_cap - max(0.0, fuel_after)))
                refuel = round(refuel, 2)

                pit = {"enter": True, "fuel_refuel_amount_l": refuel}
                fuel = max(0.0, fuel_after) + refuel
                speed = pit_exit_speed
            else:
                fuel = fuel_after
                speed = exit_speed
        else:
            fuel = fuel_after

        laps.append({"lap": lap, "segments": seg_template, "pit": pit})

    return {"initial_tyre_id": tyre_id, "laps": laps}


def _tyre_props(level_data, tyre_id):
    """Look up the property block for the compound of a given tyre set id."""
    compound = next(s["compound"] for s in level_data["available_sets"]
                    if tyre_id in s["ids"])
    return level_data["tyres"]["properties"][compound]
