import os
import random
import statistics

import wandb

from orchestrator import StationManager


PROJECT_NAME = "autonomous-drone-routing"
SEED = 8695665


TEST_CASES = [
    ("DRONE_01", "AIRPORT"),
    ("DRONE_02", "AIRPORT"),
    ("DRONE_03", "SUPERMARKET"),
    ("DRONE_04", "SUPERMARKET"),
]


def run_evaluation():
    random.seed(42)

    manager = StationManager(
        db_path="evaluation.db",
        reset=True,
        seed=SEED,
    )

    results = []

    for drone_id, destination in TEST_CASES:
        trip = manager.plan_trip(
            drone_id=drone_id,
            drain_per_cell=3.0,
            destination=destination,
        )

        route_length = (
            len(trip.plan.cells) - 1
            if trip.plan is not None and trip.plan.cells
            else 0
        )

        pilot_cautious_waits = sum(
            1
            for decision in (trip.pilot_decisions or [])
            if getattr(decision, "action", None) == "CAUTIOUS_WAIT"
        )

        results.append(
            {
                "drone_id": drone_id,
                "destination": destination,
                "verified": int(trip.verified),
                "route_length": route_length,
                "arrival_minutes": (
                    trip.arrival_minutes
                    if trip.arrival_minutes is not None
                    else -1
                ),
                "start_battery": trip.start_battery,
                "final_battery": (
                    trip.actual_final_battery
                    if trip.actual_final_battery is not None
                    else -1
                ),
                "charging_used": int(trip.charging_pad is not None),
                "charging_pad": (
                    trip.charging_pad.id
                    if trip.charging_pad is not None
                    else "None"
                ),
                "pilot_cautious_waits": pilot_cautious_waits,
            }
        )

    verified_count = sum(r["verified"] for r in results)

    valid_arrivals = [
        r["arrival_minutes"]
        for r in results
        if r["arrival_minutes"] >= 0
    ]

    valid_routes = [
        r["route_length"]
        for r in results
        if r["route_length"] > 0
    ]

    valid_final_battery = [
        r["final_battery"]
        for r in results
        if r["final_battery"] >= 0
    ]

    summary = {
        "seed": SEED,
        "test_cases": len(results),
        "verified_cases": verified_count,
        "verification_rate": verified_count / len(results),
        "mean_route_length": (
            statistics.mean(valid_routes)
            if valid_routes
            else 0
        ),
        "mean_arrival_minutes": (
            statistics.mean(valid_arrivals)
            if valid_arrivals
            else 0
        ),
        "mean_final_battery": (
            statistics.mean(valid_final_battery)
            if valid_final_battery
            else 0
        ),
        "charging_cases": sum(
            r["charging_used"] for r in results
        ),
        "total_cautious_waits": sum(
            r["pilot_cautious_waits"] for r in results
        ),
    }

    return results, summary


def main():
    results, summary = run_evaluation()

    run = wandb.init(
        project=PROJECT_NAME,
        name="baseline-evaluation-seed-8695665",
        config={
            "seed": SEED,
            "num_test_cases": len(TEST_CASES),
            "planner": "A*",
            "safety_verifier": "Z3/SMT",
            "adaptive_pilot": "Q-learning advisory policy",
            "planning_drain_pct": 3.0,
        },
    )

    for key, value in summary.items():
        wandb.summary[key] = value

    table = wandb.Table(
        columns=list(results[0].keys())
    )

    for result in results:
        table.add_data(
            *[result[column] for column in table.columns]
        )

    wandb.log({
        "evaluation_results": table,
    })

    wandb.finish()

    print("\nEvaluation complete.")
    print("=" * 60)

    for result in results:
        print(
            f"{result['drone_id']} -> "
            f"{result['destination']}: "
            f"verified={bool(result['verified'])}, "
            f"route={result['route_length']} steps, "
            f"arrival={result['arrival_minutes']:.1f} min, "
            f"final battery={result['final_battery']:.1f}%"
        )

    print("=" * 60)
    print(f"Verification rate: {summary['verification_rate']:.1%}")
    print(f"Mean route length: {summary['mean_route_length']:.2f}")
    print(
        f"Mean arrival time: "
        f"{summary['mean_arrival_minutes']:.2f} minutes"
    )
    print(
        f"Mean final battery: "
        f"{summary['mean_final_battery']:.2f}%"
    )


if __name__ == "__main__":
    main()