# Five-speed lookahead real Kev test

Same crossing map as kev-turn45-2000: 2000-step upper limit, 1-second control period, 45 deg/s turn-rate assumption, initial course east. Current arrival radius is 100 m. Available speed classes: 10, 20, 30, 40, 50 knots. First waypoint is the internal coordinate origin; display coordinates are shifted to match the old map.

The CUDA Kev service made 62 real decisions. There are 64 samples including stopping. The boat reached 1/10 waypoints, used 50 knots on 19 commands and 10 knots on 43 commands, then issued two zero-speed commands. No sampled/swept waypoint speed violations were recorded. Speed stages were cruise, alignment, approach and recovery. Full-speed cruise worked; at P1 the boat slowed to 10 knots as planned. After switching to P2 the model continued to choose unsuitable headings. Following low-speed recovery and 20 consecutive observations without improvement, NAVIGATION_NO_PROGRESS was raised and zero-speed parking was confirmed. The test ended after 63 simulated seconds, 13.44 seconds wall time; it did not run all 2000 steps.

107 offline regression tests passed. This does not establish successful navigation: the direction-selection failure remains visible in this real run. Idealized motion pauses simulated time during inference; these are not live-vessel dynamics.

Files: summary.json, audit.json, crossing_km/task.json, crossing_km/trajectory.jsonl, crossing_km/trajectory.png, crossing_km-route-zoom.png, speed-profile.png, comparison.png, run.py and policy snapshots. The raw task fields and external protocol were preserved.
