import math

GRAVITY = 9.8
K_STRAIGHT = 0.0000166
K_BRAKING = 0.0398
K_CORNER = 0.000265
K_FUEL_DRAG = 1.5e-9

WEATHER_FRICTION_KEY = {
    "dry": "dry_friction_multiplier",
    "cold": "cold_friction_multiplier",
    "light_rain": "light_rain_friction_multiplier",
    "heavy_rain": "heavy_rain_friction_multiplier",
}

WEATHER_DEGRADATION_KEY = {
    "dry": "dry_degradation",
    "cold": "cold_degradation",
    "light_rain": "light_rain_degradation",
    "heavy_rain": "heavy_rain_degradation",
}


def get_weather(race_time, conditions):
    t = 0.0
    while True:
        for cond in conditions:
            t += cond["duration_s"]
            if race_time < t:
                return cond


def tyre_friction(compound, props, degradation, weather_cond):
    # life_span is the initial friction value (not the hardcoded compound values from PDF)
    base = props["life_span"]
    mult = props[WEATHER_FRICTION_KEY[weather_cond]]
    return max(0.0, (base - degradation) * mult)


def max_corner_speed(friction, radius, crawl_const):
    return math.sqrt(max(0.0, friction * GRAVITY * radius)) + crawl_const


def fuel_consumed(vi, vf, distance, k_base):
    avg = (vi + vf) / 2.0
    return (k_base + K_FUEL_DRAG * avg * avg) * distance


def simulate_race(level_data, strategy, apply_degradation=True):
    car = level_data["car"]
    race = level_data["race"]
    segments = level_data["track"]["segments"]
    weather_conditions = level_data["weather"]["conditions"]
    tyre_props_map = level_data["tyres"]["properties"]

    k_base = car["fuel_consumption_l/m"]
    max_spd = car["max_speed_m/s"]
    accel = car["accel_m/se2"]
    brake = car["brake_m/se2"]
    limp_spd = car["limp_constant_m/s"]
    crawl_spd = car["crawl_constant_m/s"]

    pit_base = race["base_pit_stop_time_s"]
    pit_tyre_time = race["pit_tyre_swap_time_s"]
    pit_refuel_rate = race["pit_refuel_rate_l/s"]
    crash_penalty = race["corner_crash_penalty_s"]
    pit_exit_spd = race["pit_exit_speed_m/s"]
    time_ref = race["time_reference_s"]
    fuel_soft_cap = race["fuel_soft_cap_limit_l"]

    tyre_id_map = {}
    for tset in level_data.get("available_sets", []):
        for tid in tset["ids"]:
            tyre_id_map[tid] = tset["compound"]

    compound = tyre_id_map[strategy["initial_tyre_id"]]
    props = tyre_props_map[compound]

    speed = 0.0
    fuel = car["initial_fuel_l"]
    degradation = 0.0
    race_time = 0.0
    limp = False
    crawl = False
    fuel_used_total = 0.0
    crashes = 0
    blowouts = 0
    total_tyre_degradation = 0.0

    for lap_data in strategy["laps"]:
        seg_actions = {s["id"]: s for s in lap_data["segments"]}

        for seg in segments:
            sid = seg["id"]
            seg_type = seg["type"]
            length = seg["length_m"]
            action = seg_actions.get(sid, {})

            weather = get_weather(race_time, weather_conditions)
            w_cond = weather["condition"]
            eff_accel = accel * weather["acceleration_multiplier"]
            eff_brake = brake * weather["deceleration_multiplier"]

            if limp:
                race_time += length / limp_spd
                f = fuel_consumed(limp_spd, limp_spd, length, k_base)
                fuel -= f
                fuel_used_total += f
                continue

            if seg_type == "straight":
                crawl = False
                target = min(action.get("target_m/s", max_spd), max_spd)
                brake_start = action.get("brake_start_m_before_next", 0.0)
                brake_point = max(0.0, length - brake_start)

                entry = speed

                if entry < target:
                    accel_dist = (target ** 2 - entry ** 2) / (2 * eff_accel)
                    if accel_dist >= brake_point:
                        v_at_brake = math.sqrt(entry ** 2 + 2 * eff_accel * brake_point)
                        race_time += (v_at_brake - entry) / eff_accel
                        f = fuel_consumed(entry, v_at_brake, brake_point, k_base)
                        fuel -= f
                        fuel_used_total += f
                    else:
                        v_at_brake = target
                        race_time += (target - entry) / eff_accel
                        f = fuel_consumed(entry, target, accel_dist, k_base)
                        fuel -= f
                        fuel_used_total += f
                        cruise_dist = brake_point - accel_dist
                        if cruise_dist > 0:
                            race_time += cruise_dist / target
                            f = fuel_consumed(target, target, cruise_dist, k_base)
                            fuel -= f
                            fuel_used_total += f
                else:
                    v_at_brake = entry
                    if brake_point > 0:
                        race_time += brake_point / entry
                        f = fuel_consumed(entry, entry, brake_point, k_base)
                        fuel -= f
                        fuel_used_total += f

                exit_spd = math.sqrt(max(0.0, v_at_brake ** 2 - 2 * eff_brake * brake_start))
                if v_at_brake > exit_spd:
                    race_time += (v_at_brake - exit_spd) / eff_brake
                if brake_start > 0:
                    f = fuel_consumed(v_at_brake, exit_spd, brake_start, k_base)
                    fuel -= f
                    fuel_used_total += f

                if apply_degradation:
                    deg_rate = props[WEATHER_DEGRADATION_KEY[w_cond]]
                    straight_deg = deg_rate * length * K_STRAIGHT
                    braking_deg = ((v_at_brake / 100) ** 2 - (exit_spd / 100) ** 2) * K_BRAKING * deg_rate
                    degradation += straight_deg + braking_deg
                    total_tyre_degradation += straight_deg + braking_deg

                speed = exit_spd

                if fuel <= 0:
                    fuel = 0.0
                    limp = True
                if apply_degradation and degradation >= props["life_span"] and not limp:
                    limp = True
                    blowouts += 1

            elif seg_type == "corner":
                radius = seg["radius_m"]
                corner_spd = crawl_spd if crawl else speed

                friction = tyre_friction(compound, props, degradation, w_cond)
                max_cs = max_corner_speed(friction, radius, crawl_spd)

                if corner_spd > max_cs + 1e-9:
                    race_time += crash_penalty
                    if apply_degradation:
                        degradation += 0.1
                    crawl = True
                    corner_spd = crawl_spd
                    crashes += 1

                race_time += length / corner_spd
                f = fuel_consumed(corner_spd, corner_spd, length, k_base)
                fuel -= f
                fuel_used_total += f

                if apply_degradation:
                    deg_rate = props[WEATHER_DEGRADATION_KEY[w_cond]]
                    corner_deg = K_CORNER * (corner_spd ** 2 / radius) * deg_rate
                    degradation += corner_deg
                    total_tyre_degradation += corner_deg

                speed = corner_spd

                if fuel <= 0:
                    fuel = 0.0
                    limp = True
                if apply_degradation and degradation >= props["life_span"] and not limp:
                    limp = True
                    blowouts += 1

        pit = lap_data.get("pit", {})
        if pit.get("enter", False):
            pit_time = pit_base
            new_tyre_id = pit.get("tyre_change_set_id")
            if new_tyre_id:
                pit_time += pit_tyre_time
                compound = tyre_id_map[new_tyre_id]
                props = tyre_props_map[compound]
                degradation = 0.0

            refuel = pit.get("fuel_refuel_amount_l") or 0
            if refuel > 0:
                pit_time += refuel / pit_refuel_rate
                fuel = min(fuel + refuel, car["fuel_tank_capacity_l"])

            race_time += pit_time
            speed = pit_exit_spd
            limp = False
            crawl = False

    base_score = 500000 * (time_ref / race_time) ** 3
    fuel_bonus = -500000 * (1 - fuel_used_total / fuel_soft_cap) ** 2 + 500000
    tyre_bonus = 100000 * total_tyre_degradation - 50000 * blowouts

    return {
        "total_time": race_time,
        "base_score": base_score,
        "fuel_bonus": fuel_bonus,
        "tyre_bonus": tyre_bonus,
        "final_score": base_score + fuel_bonus,
        "crashes": crashes,
        "blowouts": blowouts,
        "fuel_used": fuel_used_total,
        "fuel_remaining": fuel,
    }
