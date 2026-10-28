# Flight-simulation results

```text
                 Time to 98% target altitude (s)  Final altitude (m)  Ground distance (km)  Maximum flight-path angle (deg)  Maximum lift coefficient  Maximum thrust (N)
Scenario                                                                                                                                                                 
Baseline                                  179.25             999.136                20.933                              8.0                     0.399            4048.996
15% higher mass                           178.50             999.180                20.933                              8.0                     0.441            4520.790
15 m/s headwind                           179.25             999.136                16.433                              8.0                     0.399            4048.996
```

## Interpretation

The headwind changes ground distance but not the air-relative aerodynamic response in this model.
Increasing mass changes the lift and thrust required during the climb. All comparisons use the
same simplified control law, making the scenario differences reproducible rather than optimally tuned.
