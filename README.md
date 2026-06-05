# tool-changer

Viam module providing a generic tool-changer service for swapping end-effectors on a robot arm.

## Model viam:tool-changer:default

A generic service that drives an arm through a tool-changer dock/undock sequence. Only purely mechanical changers are supported — docking and undocking happen as a consequence of how the arm moves through a rack of tool holders. Pneumatic and electrically-actuated changers are not supported today; we're interested in adding support for them.

### Configuration

The following attribute template can be used to configure this model:

```json
{
  "arm": <string>,
  "parking-pose": {
    "point": { "X": <float>, "Y": <float>, "Z": <float> },
    "orientation": { "x": <float>, "y": <float>, "z": <float>, "th": <float> }
  },
  "tools": [
    {
      "name": <string>,
      "slot-pose": {
        "point": { "X": <float>, "Y": <float>, "Z": <float> },
        "orientation": { "x": <float>, "y": <float>, "z": <float>, "th": <float> }
      },
      "slide-offset-mm": { "X": <float>, "Y": <float>, "Z": <float> },
      "lift-offset-mm": { "X": <float>, "Y": <float>, "Z": <float> },
      "slide-allowed-collisions": [
        { "Frame1": <string>, "Frame2": <string> }
      ]
    }
  ],
  "transit-constraints": <motionplan.Constraints>,
  "lift-constraints": <motionplan.Constraints>,
  "slide-constraints": <motionplan.Constraints>,
  "slide-speed": {
    "max_vel_degs_per_sec": <float>,
    "max_acc_degs_per_sec2": <float>
  },
  "save-plan-requests": <bool>
}
```

#### Attributes

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `arm` | string | Required | Name of the `arm` component this service drives through the rack. The framesystem service is added as an implicit dependency for planning. |
| `parking-pose` | object | Required | Pose the arm moves to before entering the rack and returns to after a successful change. `orientation` is required. |
| `tools` | array | Required | One entry per tool slot. Must contain at least one entry. See `tools[]` entries table. |
| `transit-constraints` | object | Optional | RDK `motionplan.Constraints` applied to moves between `parking-pose` and each tool's rack-side entry/exit pose (`lift-pose` for take entry / `slide-pose` for take exit, and the reverse for release). Defaults to `nil` (free plan). |
| `lift-constraints` | object | Optional | RDK `motionplan.Constraints` applied to the vertical move between `lift-pose` and `slot-pose` (take entry and release exit). Defaults to `nil` (free plan). |
| `slide-constraints` | object | Optional | RDK `motionplan.Constraints` applied to the final slide into `slot-pose` and the slide-out on release. Defaults to `nil` (free plan). |
| `slide-speed` | object | Optional | Joint-velocity/acceleration cap applied only to the slide-in (engagement) and slide-out (disengagement) steps. Transit and lift steps use the arm's default speed regardless. When unset, slide steps also use the arm's default. See `slide-speed` fields table. |
| `save-plan-requests` | bool | Optional | When true, every `switch_tool` and `release` call writes per-step `armplanning.PlanRequest` JSON files plus a `metadata.json` (command, from/to, total planning time, error) to a per-call subdirectory under the Viam capture dir (e.g. `<capture-dir>/tool-changer-<ts>-<command>/`). World-state payloads are deliberately stripped from each saved request; the rest of the request (frame system, goals, start state, constraints, planner options) is preserved verbatim and can be replayed against the planner offline. Defaults to false. |

##### `slide-speed` fields

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `max_vel_degs_per_sec` | float | Optional | Max joint velocity in degrees per second. Must be non-negative. Internally converted to radians for `arm.MoveOptions`. |
| `max_acc_degs_per_sec2` | float | Optional | Max joint acceleration in degrees per second². Must be non-negative. Internally converted to radians for `arm.MoveOptions`. |

##### `tools[]` entries

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `name` | string | Required | Caller-facing tool name (e.g. `"tongs"`, `"spoon"`). Must be unique within `tools`. |
| `slot-pose` | object | Required | Pose at which the robot-side changer is mechanically engaged in this tool's rack holder. `orientation` is required. |
| `slide-offset-mm` | object | Required | Slide direction relative to `slot-pose`. From `slot-pose + slide-offset-mm`, sliding to `slot-pose` is what mechanically engages the tool. Must be non-zero. |
| `lift-offset-mm` | object | Required | Perpendicular clearance from the rack relative to `slot-pose + slide-offset-mm`. From that point, moving by this further brings the arm fully clear of the rack (typically a vertical lift). Must be non-zero. |
| `slide-allowed-collisions` | array | Optional | List of `{Frame1, Frame2}` pairs the motion planner is allowed to ignore during this tool's `SlideIn` and `SlideOut` steps. Used to permit the inevitable gripper-tool contact during mechanical engagement. Pairs are not honored on other step types (transit, lift) — those keep full collision checking. Frame identifiers use the standard `<frame_name>:<geometry_label>` form. |
| `gripper` | string | Optional | Name of a gripper component associated with this tool. Reserved for use by the paired state-store model (forthcoming). When set, the gripper component must be a configured resource on the machine. |

#### Rack-side poses

- `slide-pose` = `slot-pose + slide-offset-mm`
- `lift-pose` = `slot-pose + lift-offset-mm`
- `slot-pose` = mechanically engaged

Take traversal: `parking-pose → lift-pose → slot-pose → slide-pose → parking-pose`.

Release traversal: `parking-pose → slide-pose → slot-pose → lift-pose → parking-pose`.

Take descends onto the tool (engagement on the descent), then slides out with the tool attached. Release slides into the holder (deposit on the slide-in), then lifts up to leave the tool behind.

### DoCommand

`DoCommand` accepts exactly one of the following top-level keys. Any other key returns `unknown command, expected 'switch_tool', 'release', or 'set_world_state'`.

#### `switch_tool`

Idempotently brings the named tool onto the arm. If the named tool is already attached, returns immediately with `changed: false`. Otherwise releases any currently-attached tool, then docks the target. Every step in the sequence is planned upfront against a rolling start state; nothing moves if any step fails to plan.

Request:

```json
{ "switch_tool": "tongs" }
```

Response:

```json
{
  "success": true,
  "changed": true,
  "from": null,
  "to": "tongs"
}
```

Errors: `unknown tool "<name>"` if the target isn't in `tools`; `switch_tool: value must be a string` if the value is the wrong type.

#### `release`

Returns the currently-attached tool to its rack slot. No-op when nothing is attached (returns `released: null` without moving).

Request:

```json
{ "release": true }
```

Response:

```json
{ "success": true, "released": "tongs" }
```

#### `set_world_state`

Stores a `referenceframe.WorldState` for use by all subsequent motion commands. Pass `null` to clear. The stored state is reused across calls; callers update it whenever obstacles change.

Request (set):

```json
{ "set_world_state": { "obstacles": [ ... ], "transforms": [ ... ] } }
```

Request (clear):

```json
{ "set_world_state": null }
```

Response:

```json
{ "success": true, "set": true }
```
