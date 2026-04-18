import math
from simulator import GRAVITY, WEATHER_FRICTION_KEY, max_corner_speed, fuel_consumed


def optimize(level_data):
    level_name = level_data["race"].get("name", "")
    if "Level 1" in level_name:
        return _optimize_level1(level_data)
    if "Level 2" in level_name:
        return _optimize_level2(level_data)
    raise ValueError(f"No optimizer implemented for: {level_name}")


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def _ceil_mm(value):
    return math.ceil(value * 1000) / 1000


def _best_tyre_for_weather(level_data, weather_cond):
    tyre_props_map = level_data["tyres"]["properties"]
    available_sets = level_data.get("available_sets", [])
    best_id = None
    best_compound = None
    best_friction = -1.0
    for tset in available_sets:
        compound = tset["compound"]
        props = tyre_props_map[compound]
        friction = props["life_span"] * props[WEATHER_FRICTION_KEY[weather_cond]]
        if friction > best_friction:
            best_friction = friction
            best_compound = compound
            best_id = tset["ids"][0]
    return best_id, best_compound, best_friction


def _required_entry_speed(seg_index, segments, corner_max_speeds):
    """Min corner max speed of all consecutive corners immediately after this straight."""
    n = len(segments)
    min_speed = float("inf")
    for j in range(seg_index + 1, n):
        if segments[j]["type"] == "corner":
            min_speed = min(min_speed, corner_max_speeds[segments[j]["id"]])
        else:
            break
    # Wrap-around: last segment(s) of lap feed into first of next
    if min_speed == float("inf"):
        for j in range(0, n):
            if segments[j]["type"] == "corner":
                min_speed = min(min_speed, corner_max_speeds[segments[j]["id"]])
            else:
                break
    return min_speed


def _compute_brake_start(v_from, v_to, eff_brake):
    if v_from <= v_to:
        return 0.0
    return (v_from ** 2 - v_to ** 2) / (2 * eff_brake)


def _build_segment_template(level_data, weather_cond, eff_brake, props, max_spd, crawl_spd):
    """Build the per-segment strategy: max speed on straights, corner no-ops."""
    segments = level_data["track"]["segments"]
    friction = props["life_span"] * props[WEATHER_FRICTION_KEY[weather_cond]]

    corner_max_speeds = {}
    for seg in segments:
        if seg["type"] == "corner":
            corner_max_speeds[seg["id"]] = max_corner_speed(friction, seg["radius_m"], crawl_spd)

    template = []
    for i, seg in enumerate(segments):
        if seg["type"] == "straight":
            required_exit = _required_entry_speed(i, segments, corner_max_speeds)
            brake_dist = _compute_brake_start(max_spd, required_exit, eff_brake)
            brake_dist = _ceil_mm(brake_dist)
            brake_dist = min(brake_dist, seg["length_m"] * 0.99)
            template.append({
                "id": seg["id"],
                "type": "straight",
                "target_m/s": max_spd,
                "brake_start_m_before_next": brake_dist,
            })
        else:
            template.append({"id": seg["id"], "type": "corner"})

    return template


def _compute_lap_fuel(level_data, seg_template, entry_speed):
    """Analytically compute (exit_speed, fuel_used) for one lap."""
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
            bs = action.get("brake_start_m_before_next", 0.0)
            bp = max(0.0, length - bs)
            entry = speed

            if entry < target:
                ad = (target ** 2 - entry ** 2) / (2 * accel)
                if ad >= bp:
                    v_brake = math.sqrt(entry ** 2 + 2 * accel * bp)
                    total_fuel += fuel_consumed(entry, v_brake, bp, k_base)
                else:
                    total_fuel += fuel_consumed(entry, target, ad, k_base)
                    cd = bp - ad
                    if cd > 0:
                        total_fuel += fuel_consumed(target, target, cd, k_base)
                    v_brake = target
            else:
                v_brake = entry
                if bp > 0:
                    total_fuel += fuel_consumed(entry, entry, bp, k_base)

            exit_spd = math.sqrt(max(0.0, v_brake ** 2 - 2 * brake_rate * bs))
            if bs > 0:
                total_fuel += fuel_consumed(v_brake, exit_spd, bs, k_base)
            speed = exit_spd
        else:
            total_fuel += fuel_consumed(speed, speed, length, k_base)

    return speed, total_fuel


def _steady_state_fuel(level_data, seg_template):
    """Converge to steady-state lap entry speed and return (entry_speed, fuel_per_lap)."""
    speed = 0.0
    for _ in range(6):
        speed, _ = _compute_lap_fuel(level_data, seg_template, speed)
    _, fuel = _compute_lap_fuel(level_data, seg_template, speed)
    return speed, fuel


# ---------------------------------------------------------------------------
# Level 1
# ---------------------------------------------------------------------------

def _optimize_level1(level_data):
    car = level_data["car"]
    race = level_data["race"]
    weather_conditions = level_data["weather"]["conditions"]

    max_spd = car["max_speed_m/s"]
    crawl_spd = car["crawl_constant_m/s"]
    brake = car["brake_m/se2"]
    num_laps = race["laps"]

    starting_cond_id = race["starting_weather_condition_id"]
    starting_weather = next(c for c in weather_conditions if c["id"] == starting_cond_id)
    w_cond = starting_weather["condition"]
    eff_brake = brake * starting_weather["deceleration_multiplier"]

    best_tyre_id, _, _ = _best_tyre_for_weather(level_data, w_cond)
    props = level_data["tyres"]["properties"][
        next(s["compound"] for s in level_data["available_sets"] if best_tyre_id in s["ids"])
    ]

    seg_template = _build_segment_template(level_data, w_cond, eff_brake, props, max_spd, crawl_spd)

    laps = [
        {"lap": lap, "segments": seg_template, "pit": {"enter": False}}
        for lap in range(1, num_laps + 1)
    ]

    return {"initial_tyre_id": best_tyre_id, "laps": laps}


# ---------------------------------------------------------------------------
# Level 2
# ---------------------------------------------------------------------------

def _optimize_level2(level_data):
    car = level_data["car"]
    race = level_data["race"]
    weather_conditions = level_data["weather"]["conditions"]

    max_spd = car["max_speed_m/s"]
    crawl_spd = car["crawl_constant_m/s"]
    brake = car["brake_m/se2"]
    tank_cap = car["fuel_tank_capacity_l"]
    initial_fuel = car["initial_fuel_l"]
    num_laps = race["laps"]
    pit_exit_spd = race["pit_exit_speed_m/s"]

    starting_cond_id = race["starting_weather_condition_id"]
    starting_weather = next(c for c in weather_conditions if c["id"] == starting_cond_id)
    w_cond = starting_weather["condition"]
    eff_brake = brake * starting_weather["deceleration_multiplier"]

    best_tyre_id, _, _ = _best_tyre_for_weather(level_data, w_cond)
    props = level_data["tyres"]["properties"][
        next(s["compound"] for s in level_data["available_sets"] if best_tyre_id in s["ids"])
    ]

    seg_template = _build_segment_template(level_data, w_cond, eff_brake, props, max_spd, crawl_spd)

    # Pre-compute steady-state and post-pit fuel per lap
    ss_entry, ss_fuel = _steady_state_fuel(level_data, seg_template)
    _, pit_lap_fuel = _compute_lap_fuel(level_data, seg_template, pit_exit_spd)

    # Greedily schedule pit stops: pit at end of lap when fuel won't cover the next lap
    SAFETY = 1.0  # litres safety buffer
    laps = []
    fuel = initial_fuel
    speed = 0.0

    for lap in range(1, num_laps + 1):
        exit_speed, lap_fuel = _compute_lap_fuel(level_data, seg_template, speed)
        fuel_after = fuel - lap_fuel

        remaining_after = num_laps - lap
        pit = {"enter": False}

        if remaining_after > 0:
            fuel_for_all = remaining_after * ss_fuel
            if fuel_after >= fuel_for_all:
                fuel = fuel_after
                speed = exit_speed
            elif fuel_after < ss_fuel + SAFETY:
                # Must pit — compute exact refuel needed
                fuel_for_rest = pit_lap_fuel + max(0, remaining_after - 1) * ss_fuel
                refuel = fuel_for_rest - max(0.0, fuel_after) + SAFETY
                refuel = max(0.0, min(refuel, tank_cap - max(0.0, fuel_after)))
                refuel = round(refuel, 2)

                pit = {"enter": True, "fuel_refuel_amount_l": refuel}
                fuel = max(0.0, fuel_after) + refuel
                speed = pit_exit_spd
            else:
                fuel = fuel_after
                speed = exit_speed
        else:
            fuel = fuel_after

        laps.append({"lap": lap, "segments": seg_template, "pit": pit})

    return {"initial_tyre_id": best_tyre_id, "laps": laps}
