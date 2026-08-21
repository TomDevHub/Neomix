"""Physics simulation of a race given a track, car, and driving strategy.

The simulator mirrors the Entelect Grand Prix judge: it steps the car through
every segment of every lap using simple kinematics, tracks fuel and tyre wear,
and computes the three score components (base, fuel bonus, tyre bonus).
"""

import math

GRAVITY = 9.8

# Empirical constants that map physical quantities onto the judge's tyre-wear
# model. They were reverse-engineered by matching simulator output to the
# judge's per-segment logs, not taken from a physics textbook.
K_STRAIGHT = 0.0000166   # wear per metre of straight-line running
K_BRAKING = 0.0398       # wear per unit of braking energy shed
K_CORNER = 0.000265      # wear per unit of lateral load in a corner
K_FUEL_DRAG = 1.5e-9     # velocity-squared term in the fuel-burn model

# Weather condition names map to per-compound property keys in the level data.
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
    """Return the weather condition active at the given elapsed race time.

    Conditions are laid out end to end in order. The level data always spans
    the full race, so the first matching condition is returned.
    """
    elapsed = 0.0
    for condition in conditions:
        elapsed += condition["duration_s"]
        if race_time < elapsed:
            return condition
    return conditions[-1]


def tyre_friction(props, degradation, weather_cond):
    """Current grip of a tyre, reduced by wear and scaled by the weather.

    life_span doubles as the tyre's starting friction coefficient; grip falls
    linearly as degradation accumulates and is scaled by the weather multiplier.
    """
    base = props["life_span"]
    multiplier = props[WEATHER_FRICTION_KEY[weather_cond]]
    return max(0.0, (base - degradation) * multiplier)


def max_corner_speed(friction, radius, crawl_const):
    """Fastest speed that keeps lateral acceleration within available grip."""
    return math.sqrt(max(0.0, friction * GRAVITY * radius)) + crawl_const


def fuel_consumed(v_initial, v_final, distance, k_base):
    """Fuel burned over a distance, using the average speed for the drag term."""
    avg_speed = (v_initial + v_final) / 2.0
    return (k_base + K_FUEL_DRAG * avg_speed * avg_speed) * distance


def simulate_race(level_data, strategy, apply_degradation=True):
    """Run the full race and return timing, scores, and end-state diagnostics.

    Args:
        level_data: parsed level JSON (car, race, track, weather, tyres).
        strategy: the per-lap, per-segment plan produced by the optimizer.
        apply_degradation: whether tyre wear and blowouts are simulated.

    Returns:
        A dict with total_time, the three score components, and counters for
        crashes, blowouts, and fuel usage.
    """
    car = level_data["car"]
    race = level_data["race"]
    segments = level_data["track"]["segments"]
    weather_conditions = level_data["weather"]["conditions"]
    tyre_props_map = level_data["tyres"]["properties"]

    k_base = car["fuel_consumption_l/m"]
    max_speed = car["max_speed_m/s"]
    accel = car["accel_m/se2"]
    brake = car["brake_m/se2"]
    limp_speed = car["limp_constant_m/s"]
    crawl_speed = car["crawl_constant_m/s"]

    pit_base_time = race["base_pit_stop_time_s"]
    pit_tyre_time = race["pit_tyre_swap_time_s"]
    pit_refuel_rate = race["pit_refuel_rate_l/s"]
    crash_penalty = race["corner_crash_penalty_s"]
    pit_exit_speed = race["pit_exit_speed_m/s"]
    time_reference = race["time_reference_s"]
    fuel_soft_cap = race["fuel_soft_cap_limit_l"]

    # Map each tyre set id to its compound name.
    tyre_id_to_compound = {}
    for tyre_set in level_data.get("available_sets", []):
        for tyre_id in tyre_set["ids"]:
            tyre_id_to_compound[tyre_id] = tyre_set["compound"]

    compound = tyre_id_to_compound[strategy["initial_tyre_id"]]
    props = tyre_props_map[compound]

    speed = 0.0
    fuel = car["initial_fuel_l"]
    degradation = 0.0
    race_time = 0.0
    limping = False       # out of fuel: locked to limp speed for the rest
    crawling = False      # crashed this lap: corners taken at crawl speed
    fuel_used_total = 0.0
    crashes = 0
    blowouts = 0
    total_tyre_degradation = 0.0

    for lap_data in strategy["laps"]:
        seg_actions = {s["id"]: s for s in lap_data["segments"]}

        for seg in segments:
            seg_id = seg["id"]
            seg_type = seg["type"]
            length = seg["length_m"]
            action = seg_actions.get(seg_id, {})

            weather = get_weather(race_time, weather_conditions)
            w_cond = weather["condition"]
            eff_accel = accel * weather["acceleration_multiplier"]
            eff_brake = brake * weather["deceleration_multiplier"]

            # Once out of fuel the car crawls home at limp speed.
            if limping:
                race_time += length / limp_speed
                burned = fuel_consumed(limp_speed, limp_speed, length, k_base)
                fuel -= burned
                fuel_used_total += burned
                continue

            if seg_type == "straight":
                crawling = False
                target = min(action.get("target_m/s", max_speed), max_speed)
                brake_start = action.get("brake_start_m_before_next", 0.0)
                brake_point = max(0.0, length - brake_start)
                entry = speed

                # Phase 1: accelerate (and possibly cruise) up to the brake point.
                if entry < target:
                    accel_dist = (target ** 2 - entry ** 2) / (2 * eff_accel)
                    if accel_dist >= brake_point:
                        # Still accelerating when we reach the brake point.
                        v_at_brake = math.sqrt(entry ** 2 + 2 * eff_accel * brake_point)
                        race_time += (v_at_brake - entry) / eff_accel
                        burned = fuel_consumed(entry, v_at_brake, brake_point, k_base)
                        fuel -= burned
                        fuel_used_total += burned
                    else:
                        # Reach target, then cruise the remaining distance.
                        v_at_brake = target
                        race_time += (target - entry) / eff_accel
                        burned = fuel_consumed(entry, target, accel_dist, k_base)
                        fuel -= burned
                        fuel_used_total += burned
                        cruise_dist = brake_point - accel_dist
                        if cruise_dist > 0:
                            race_time += cruise_dist / target
                            burned = fuel_consumed(target, target, cruise_dist, k_base)
                            fuel -= burned
                            fuel_used_total += burned
                else:
                    # Already at or above target: cruise to the brake point.
                    v_at_brake = entry
                    if brake_point > 0:
                        race_time += brake_point / entry
                        burned = fuel_consumed(entry, entry, brake_point, k_base)
                        fuel -= burned
                        fuel_used_total += burned

                # Phase 2: brake over the final brake_start metres.
                exit_speed = math.sqrt(max(0.0, v_at_brake ** 2 - 2 * eff_brake * brake_start))
                if v_at_brake > exit_speed:
                    race_time += (v_at_brake - exit_speed) / eff_brake
                if brake_start > 0:
                    burned = fuel_consumed(v_at_brake, exit_speed, brake_start, k_base)
                    fuel -= burned
                    fuel_used_total += burned

                if apply_degradation:
                    deg_rate = props[WEATHER_DEGRADATION_KEY[w_cond]]
                    straight_deg = deg_rate * length * K_STRAIGHT
                    braking_deg = (((v_at_brake / 100) ** 2 - (exit_speed / 100) ** 2)
                                   * K_BRAKING * deg_rate)
                    degradation += straight_deg + braking_deg
                    total_tyre_degradation += straight_deg + braking_deg

                speed = exit_speed

            elif seg_type == "corner":
                radius = seg["radius_m"]
                corner_speed = crawl_speed if crawling else speed

                friction = tyre_friction(props, degradation, w_cond)
                limit = max_corner_speed(friction, radius, crawl_speed)

                # Too fast for the corner: crash penalty, then crawl through.
                if corner_speed > limit + 1e-9:
                    race_time += crash_penalty
                    if apply_degradation:
                        degradation += 0.1
                    crawling = True
                    corner_speed = crawl_speed
                    crashes += 1

                race_time += length / corner_speed
                burned = fuel_consumed(corner_speed, corner_speed, length, k_base)
                fuel -= burned
                fuel_used_total += burned

                if apply_degradation:
                    deg_rate = props[WEATHER_DEGRADATION_KEY[w_cond]]
                    corner_deg = K_CORNER * (corner_speed ** 2 / radius) * deg_rate
                    degradation += corner_deg
                    total_tyre_degradation += corner_deg

                speed = corner_speed

            # Running out of fuel or wearing a tyre to nothing both end the lap
            # early: the car drops to limp speed for the rest of the race.
            if fuel <= 0:
                fuel = 0.0
                limping = True
            if apply_degradation and degradation >= props["life_span"] and not limping:
                limping = True
                blowouts += 1

        # End-of-lap pit stop.
        pit = lap_data.get("pit", {})
        if pit.get("enter", False):
            pit_time = pit_base_time

            new_tyre_id = pit.get("tyre_change_set_id")
            if new_tyre_id:
                pit_time += pit_tyre_time
                compound = tyre_id_to_compound[new_tyre_id]
                props = tyre_props_map[compound]
                degradation = 0.0

            refuel = pit.get("fuel_refuel_amount_l") or 0
            if refuel > 0:
                pit_time += refuel / pit_refuel_rate
                fuel = min(fuel + refuel, car["fuel_tank_capacity_l"])

            race_time += pit_time
            speed = pit_exit_speed
            limping = False
            crawling = False

    base_score = 500000 * (time_reference / race_time) ** 3
    fuel_bonus = -500000 * (1 - fuel_used_total / fuel_soft_cap) ** 2 + 500000
    tyre_bonus = 100000 * total_tyre_degradation - 50000 * blowouts

    return {
        "total_time": race_time,
        "base_score": base_score,
        "fuel_bonus": fuel_bonus,
        "tyre_bonus": tyre_bonus,
        "crashes": crashes,
        "blowouts": blowouts,
        "fuel_used": fuel_used_total,
        "fuel_remaining": fuel,
    }
