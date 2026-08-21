# Entelect Grand Prix: Race Strategy Optimiser

A physics-based race strategy optimiser for the Entelect Grand Prix challenge.
It reads a level description (track, car, tyres, weather), computes an optimal
driving and pit strategy analytically, and simulates the race to score it the
same way the competition judge does.

Placed **5th out of roughly 150 teams**.

## Team

Built with Etienne Tredoux and Felix de Bruin for the Entelect Grand Prix 2026.

## The challenge

Each level defines a Formula 1 style race as data: a track made of straights
and corners, a car with fixed acceleration, braking, top speed and fuel tank,
a set of tyre compounds with different grip and wear rates, and a weather
timeline that scales grip and acceleration. The goal is to produce a strategy
(target speed and brake point per segment, tyre choice, and pit stops) that
maximises the judge's score.

The score has three parts, which apply from different levels onward:

* Base score, based on total race time against a reference time.
* Fuel bonus, higher the less fuel you burn (introduced with pit stops).
* Tyre bonus, for managing tyre wear without a blowout.

## Approach

The solver is analytic rather than search-based. Nothing is brute-forced.

**Corner speed limits.** The fastest a car can take a corner of radius `r`
follows from grip: lateral acceleration cannot exceed `friction * g`, so the
limit is `sqrt(friction * g * r)`. Grip depends on the tyre compound, its
current wear, and the weather multiplier.

**Braking points.** For each straight the solver looks ahead to the next run of
corners, finds the lowest speed limit among them, and computes the exact
distance needed to brake from top speed down to that limit using
`d = (v_from^2 - v_to^2) / (2 * a)`. The car then runs flat out and brakes as
late as possible. If a straight is the last segment on the lap, the look-ahead
wraps around to the corners at the start.

**Tyre choice.** For the race weather the solver picks the available compound
with the highest effective grip (`life_span * friction_multiplier`).

**Fuel and pit strategy.** Fuel burn is modelled in closed form (a base rate
plus a velocity-squared drag term), so a lap's fuel cost can be computed without
simulating it. The lap entry speed converges to a steady state after a couple of
iterations. Using that, the Level 2 solver pits only when it would otherwise run
dry, and refuels the exact amount needed to reach the flag (or the tank limit),
which minimises fuel-bonus loss.

**Scoring simulator.** `simulator.py` steps the car through every segment of
every lap, tracking time, fuel and tyre wear, applying crash and blowout
penalties, and computing the three score components exactly as the judge does.
The optimizer and the simulator are kept separate so strategies can be scored
independently of how they were produced.

## Project layout

```
main.py            CLI: read a level, optimize, simulate, print the score
optimizer.py       builds the driving and pit strategy for a level
simulator.py       physics simulation and scoring
levels/            sample level inputs (JSON)
sample_runs/       per-segment judge-style logs from earlier submissions
```

## Running it

Requires Python 3 (standard library only, no dependencies).

```
python main.py levels/level_1.json solution.txt
```

The first argument is the level file and the second is where the strategy JSON
is written. Both are optional and default to `levels/level_1.json` and
`solution.txt`. The program prints the total time, each score component, the
judge score, and fuel and pit-stop diagnostics.

Level 1 (fastest time) and Level 2 (time plus fuel bonus) have dedicated
optimizers. Levels 3 and 4 are not implemented and raise a clear error.

## Notes on the tuning constants

A few constants in the code were reverse-engineered by matching the simulator's
output to the judge's per-segment logs (the tyre-wear coefficients) or tuned to
the judge's numerics (the brake and fuel safety margins in `optimizer.py`).
These are documented inline. They are specific to this competition's engine
rather than general physics.
