import math
from simulator import GRAVITY, WEATHER_FRICTION_KEY, max_corner_speed


def _ceil_cm(value):
    return math.ceil(value * 100) / 100

def optimize(level_data):
    level_name = level_data["race"].get("name", "")
    if "Level 1" in level_name:
        return _optimize_level1(level_data)
    raise ValueError(f"No optimizer implemented for: {level_name}")


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
    """
    Returns the minimum corner max speed of all consecutive corners
    immediately following the straight at seg_index.
    """
    n = len(segments)
    min_speed = float("inf")
    for j in range(seg_index + 1, n):
        if segments[j]["type"] == "corner":
            min_speed = min(min_speed, corner_max_speeds[segments[j]["id"]])
        else:
            break
    # Wrap around for the case where the straight is the last segment
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


def _optimize_level1(level_data):
    car = level_data["car"]
    race = level_data["race"]
    segments = level_data["track"]["segments"]
    tyre_props_map = level_data["tyres"]["properties"]
    weather_conditions = level_data["weather"]["conditions"]

    max_spd = car["max_speed_m/s"]
    accel = car["accel_m/se2"]
    brake = car["brake_m/se2"]
    crawl_spd = car["crawl_constant_m/s"]
    num_laps = race["laps"]

    starting_cond_id = race["starting_weather_condition_id"]
    starting_weather = next(c for c in weather_conditions if c["id"] == starting_cond_id)
    w_cond = starting_weather["condition"]
    eff_brake = brake * starting_weather["deceleration_multiplier"]

    best_tyre_id, best_compound, _ = _best_tyre_for_weather(level_data, w_cond)
    props = tyre_props_map[best_compound]

    # No degradation in Level 1: friction is constant
    friction = props["life_span"] * props[WEATHER_FRICTION_KEY[w_cond]]

    corner_max_speeds = {}
    for seg in segments:
        if seg["type"] == "corner":
            corner_max_speeds[seg["id"]] = max_corner_speed(friction, seg["radius_m"], crawl_spd)

    seg_strategies_template = []
    for i, seg in enumerate(segments):
        if seg["type"] == "straight":
            required_exit = _required_entry_speed(i, segments, corner_max_speeds)
            brake_dist = _compute_brake_start(max_spd, required_exit, eff_brake)
            # Round UP to nearest cm so floating-point exit speed never exceeds corner max
            brake_dist = _ceil_cm(brake_dist)
            brake_dist = min(brake_dist, seg["length_m"] * 0.99)

            seg_strategies_template.append({
                "id": seg["id"],
                "type": "straight",
                "target_m/s": max_spd,
                "brake_start_m_before_next": brake_dist,
            })
        else:
            seg_strategies_template.append({
                "id": seg["id"],
                "type": "corner",
            })

    laps = []
    for lap in range(1, num_laps + 1):
        laps.append({
            "lap": lap,
            "segments": seg_strategies_template,
            "pit": {"enter": False},
        })

    return {
        "initial_tyre_id": best_tyre_id,
        "laps": laps,
    }
