"""Entry point: read a level, optimize a strategy, simulate it, print the score.

Usage:
    python main.py [level_file] [output_file]

Defaults to levels/level_1.json and solution.txt.
"""

import json
import sys

from optimizer import optimize
from simulator import simulate_race


def judge_score(level_name, result):
    """Score the judge awards for this level, per the problem statement.

        Level 1:      base score only
        Levels 2, 3:  base score + fuel bonus
        Level 4:      base score + fuel bonus + tyre bonus
    """
    if "Level 1" in level_name:
        return result["base_score"]
    if "Level 4" in level_name:
        return result["base_score"] + result["fuel_bonus"] + result["tyre_bonus"]
    return result["base_score"] + result["fuel_bonus"]


def main():
    level_file = sys.argv[1] if len(sys.argv) > 1 else "levels/level_1.json"
    output_file = sys.argv[2] if len(sys.argv) > 2 else "solution.txt"

    with open(level_file) as f:
        level_data = json.load(f)

    level_name = level_data["race"].get("name", "")

    # Degradation is disabled for Levels 1 and 2 per the problem statement.
    # The spec is ambiguous for Level 2; if the judge does apply degradation
    # there, flip this to True and rebuild the strategy.
    apply_deg = "Level 1" not in level_name and "Level 2" not in level_name

    strategy = optimize(level_data)
    result = simulate_race(level_data, strategy, apply_degradation=apply_deg)

    score = judge_score(level_name, result)

    tyre_id = strategy["initial_tyre_id"]
    compound = next(
        s["compound"] for s in level_data["available_sets"] if tyre_id in s["ids"]
    )
    pit_count = sum(1 for lap in strategy["laps"] if lap["pit"].get("enter"))

    print(f"Race:          {level_name}")
    print(f"Tyre:          {compound} (id={tyre_id})")
    print(f"Total time:    {result['total_time']:.3f} s")
    print(f"Base score:    {result['base_score']:,.0f}")
    print(f"Fuel bonus:    {result['fuel_bonus']:,.0f}")
    print(f"Tyre bonus:    {result['tyre_bonus']:,.0f}")
    print(f"Judge score:   {score:,.0f}")
    print(f"Crashes:       {result['crashes']}")
    print(f"Blowouts:      {result['blowouts']}")
    print(f"Fuel used:     {result['fuel_used']:.3f} L  "
          f"(remaining: {result['fuel_remaining']:.3f} L)")
    print(f"Pit stops:     {pit_count}")

    with open(output_file, "w") as f:
        json.dump(strategy, f, indent=2)

    print(f"\nSolution written to {output_file}")


if __name__ == "__main__":
    main()
