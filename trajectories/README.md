# Shared motion recordings

Record station-to-station routes on the actual workstation and teach the precise station points separately. The pickup takes in this directory belong to this workstation; other segments still need recording.

| Name | Start | End |
| --- | --- | --- |
| `pick_cup` | HOME/approach reachable by the planned entry `MovL` | Near `CUP_RECEIVE`; recipe then moves precisely to that taught point |
| `cup_to_coffee` | Departure start reachable by `MovL` from `CUP_RECEIVE`, after cup dispensing | Near `COFFEE_POS`; recipe then executes a point `MovL` |
| `cup_to_ice` | Departure start reachable by `MovL` from `CUP_RECEIVE`, after cup dispensing | Near `ICE_POS`; recipe then executes a point `MovL` |
| `ice_to_coffee` | Departure start reachable by `MovL` from `ICE_POS`, after ice/water dispensing | Near `COFFEE_POS`; recipe then executes a point `MovL` |
| `coffee_to_window` | Coffee dispensing position | Cup placed and released, fixture withdrawn clear of the window |

Record withdrawal, corridors, approaches, and release movements as appropriate. Start and finish each take at rest. Prefer starting the outgoing take at its taught station point; any separate point-to-start connection must be clear for a planned straight `MovL`. Equipment waits occur after the taught station point has actually been reached. Point edits do not rewrite the recordings, and they reset workflow verification flags.

Example:

```powershell
python record_trajectory.py pick_cup --duration 30 --rate 50
```

Use the same command with the other names. A new take such as `pick_cup_002` can be selected by updating its shared catalog entry:

```json
"pick_cup": {"file": "trajectories/pick_cup_002.json"}
```

Recording keeps `NAME_raw.json` intact and writes a validated time-processed copy to `NAME.json`. Time processing changes only timestamps, preserving every joint/pose pair and endpoint. If it cannot meet dynamics and gap limits, only the raw backup is kept. Existing recorded and tested smoothed files remain unchanged; choose their registered names or explicitly update a recipe's logical mapping to select one.

The `coffee_to_window` take can be shared only if both coffee approaches end in the same arm state and the recording is validated for both payload/cup conditions. Otherwise register a separate take and reference it from the affected recipe.

Use each recipe's `--check-trajectories` for offline inspection and `--check-steps` for confirmed physical commissioning. After physical validation, manually set `replay_verified` in the take and the recipe's `safe_routes_physically_verified` flag. A recording that stopped with an error is rejected even during commissioning.

Physically verify the HOME connections, each station-point approach, each departure-to-trajectory-start MovL, and any small start correction. Untaught points or missing segments stop the workflow before connection or enable.
