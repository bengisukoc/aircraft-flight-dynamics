#!/usr/bin/env python3
"""Run and compare three aircraft flight scenarios."""

from dataclasses import replace
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import matplotlib.pyplot as plt
import pandas as pd

from flight_dynamics import AircraftParameters, simulate_flight, summarise_flight


def main() -> None:
    base = AircraftParameters()
    cases = {
        "Baseline": (base, 0.0),
        "15% higher mass": (replace(base, mass=1.15 * base.mass), 0.0),
        "15 m/s headwind": (base, -15.0),
    }
    output = ROOT / "outputs"
    output.mkdir(exist_ok=True)
    results = {}
    metrics = {}
    for name, (parameters, wind) in cases.items():
        results[name] = simulate_flight(parameters=parameters, duration=300.0, wind_mps=wind)
        metrics[name] = summarise_flight(results[name])
        slug = name.lower().replace(" ", "_").replace("%", "percent").replace("/", "_per_")
        results[name].to_csv(output / f"{slug}.csv",
                             index=False, float_format="%.5f")
    metrics_table = pd.DataFrame(metrics).T
    metrics_table.index.name = "Scenario"
    metrics_table.to_csv(output / "scenario_metrics.csv", float_format="%.3f")

    colors = {"Baseline": "#2563eb", "15% higher mass": "#dc2626", "15 m/s headwind": "#059669"}
    # The headwind case overlaps the baseline in air-relative plots, so draw it dashed.
    styles = {"Baseline": "-", "15% higher mass": "-", "15 m/s headwind": "--"}
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for name, data in results.items():
        axes[0, 0].plot(data["Range (m)"] / 1000, data["Altitude (m)"], label=name, color=colors[name], linestyle=styles[name])
        axes[0, 1].plot(data["Time (s)"], data["Altitude (m)"], color=colors[name], linestyle=styles[name])
        axes[1, 0].plot(data["Time (s)"], data["Airspeed (m/s)"], color=colors[name], linestyle=styles[name])
        axes[1, 1].plot(data["Time (s)"], data["Thrust (N)"], color=colors[name], linestyle=styles[name])
    axes[0, 0].set(xlabel="Ground distance (km)", ylabel="Altitude (m)", title="Flight trajectory")
    axes[0, 1].set(xlabel="Time (s)", ylabel="Altitude (m)", title="Altitude response")
    axes[1, 0].set(xlabel="Time (s)", ylabel="Airspeed (m/s)", title="Airspeed control")
    axes[1, 1].set(xlabel="Time (s)", ylabel="Thrust (N)", title="Thrust demand")
    axes[0, 0].legend(frameon=False)
    for ax in axes.flat:
        ax.grid(alpha=0.25)
    fig.suptitle("Aircraft climb scenario comparison", weight="bold", fontsize=15)
    fig.tight_layout()
    fig.savefig(output / "flight_scenarios.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    summary = [
        "# Flight-simulation results", "", "```text", metrics_table.round(3).to_string(), "```", "",
        "## Interpretation", "",
        "The headwind changes ground distance but not the air-relative aerodynamic response in this model.",
        "Increasing mass changes the lift and thrust required during the climb. All comparisons use the",
        "same simplified control law, making the scenario differences reproducible rather than optimally tuned.",
    ]
    (output / "analysis_summary.md").write_text("\n".join(summary) + "\n", encoding="utf-8")
    print(metrics_table.round(3).to_string())
    print(f"\nResults saved in {output}")


if __name__ == "__main__":
    main()

