# Shared motion recordings

Record these segments on the actual workstation; no executable example trajectories are supplied.

| Name | Start | End |
| --- | --- | --- |
| `pick_cup` | An open approach position reachable from HOME by the verified entry `MovL` | Cup receiving position, ready to wait for dispensing |
| `cup_to_coffee` | End of `pick_cup`, after receiving the cup | Coffee dispensing position |
| `cup_to_ice` | End of `pick_cup`, after receiving the cup | Ice/water receiving position |
| `ice_to_coffee` | End of `cup_to_ice`, after ice/water dispensing | Coffee dispensing position |
| `coffee_to_window` | Coffee dispensing position | Cup placed and released, fixture withdrawn clear of the window |

Record the actual withdrawal, corridor, approach, and release movements as appropriate. Stop at the end of each segment to wait for the associated device outside playback. Start and finish each take at rest; the next take should start at the previous one's endpoint, including matching joint configuration.

Example:

```powershell
python record_trajectory.py pick_cup --duration 30 --rate 50
```

Use the same command with the other names. A new take such as `pick_cup_002` can be selected by updating its shared catalog entry:

```json
"pick_cup": {"file": "trajectories/pick_cup_002.json"}
```

The `coffee_to_window` take can be shared only if both coffee approaches end in the same arm state and the recording is validated for both payload/cup conditions. Otherwise register a separate take and reference it from the affected recipe.

Use each recipe's `--check-trajectories` for offline inspection and `--check-steps` for confirmed physical commissioning. After physical validation, manually set `replay_verified` in the take and the recipe's `safe_routes_physically_verified` flag. A recording that stopped with an error is rejected even during commissioning.

The HOME entry/return connections and small start corrections must also be physically verified. Old point-table values do not establish that a recorded segment or its connections are safe.
