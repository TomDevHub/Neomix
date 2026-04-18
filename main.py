import json
import sys
from optimizer import optimize
from simulator import simulate_race


def main():
    level_file = sys.argv[1] if len(sys.argv) > 1 else "1.txt"
    output_file = sys.argv[2] if len(sys.argv) > 2 else "solution.txt"

    with open(level_file) as f:
        level_data = json.load(f)

    level_name = level_data["race"].get("name", "")
    apply_deg = "Level 1" not in level_name and "Level 2" not in level_name

    strategy = optimize(level_data)

    result = simulate_race(level_data, strategy, apply_degradation=apply_deg)

    tyre_id = strategy["initial_tyre_id"]
    compound = next(
        s["compound"] for s in level_data["available_sets"] if tyre_id in s["ids"]
    )

    print(f"Race:          {level_name}")
    print(f"Tyre:          {compound} (id={tyre_id})")
    print(f"Total time:    {result['total_time']:.3f} s")
    print(f"Base score:    {result['base_score']:,.0f}")
    print(f"Fuel bonus:    {result['fuel_bonus']:,.0f}")
    print(f"Final score:   {result['final_score']:,.0f}")
    print(f"Crashes:       {result['crashes']}")
    print(f"Blowouts:      {result['blowouts']}")
    print(f"Fuel used:     {result['fuel_used']:.3f} L  (remaining: {result['fuel_remaining']:.3f} L)")

    with open(output_file, "w") as f:
        json.dump(strategy, f, indent=2)

    print(f"\nSolution written to {output_file}")


if __name__ == "__main__":
    main()
