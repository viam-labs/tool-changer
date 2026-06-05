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
      "lift-offset-mm": { "X": <float>, "Y": <float>, "Z": <float> }
    }
  ],
  "transit-constraints": <motionplan.Constraints>,
  "lift-constraints": <motionplan.Constraints>,
  "slide-constraints": <motionplan.Constraints>
}
```

#### Attributes

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `arm` | string | Required | Name of the `arm` component this service drives through the rack. The framesystem service is added as an implicit dependency for planning. |
| `parking-pose` | object | Required | Pose the arm moves to before entering the rack and returns to after a successful change. `orientation` is required. |
| `tools` | array | Required | One entry per tool slot. Must contain at least one entry. See `tools[]` entries table. |
| `transit-constraints` | object | Optional | RDK `motionplan.Constraints` applied to moves between `parking-pose` and each tool's rack-side entry/exit pose (`slide-pose` for take entry / `lift-pose` for take exit, and the reverse for release). Defaults to `nil` (free plan). |
| `lift-constraints` | object | Optional | RDK `motionplan.Constraints` applied to the vertical move between `slot-pose` and `lift-pose` (take exit and release entry). Defaults to `nil` (free plan). |
| `slide-constraints` | object | Optional | RDK `motionplan.Constraints` applied to the final slide into `slot-pose` and the slide-out on release. Defaults to `nil` (free plan). |

##### `tools[]` entries

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `name` | string | Required | Caller-facing tool name (e.g. `"tongs"`, `"spoon"`). Must be unique within `tools`. |
| `slot-pose` | object | Required | Pose at which the robot-side changer is mechanically engaged in this tool's rack holder. `orientation` is required. |
| `slide-offset-mm` | object | Required | Slide direction relative to `slot-pose`. From `slot-pose + slide-offset-mm`, sliding to `slot-pose` is what mechanically engages the tool. Must be non-zero. |
| `lift-offset-mm` | object | Required | Perpendicular clearance from the rack relative to `slot-pose + slide-offset-mm`. From that point, moving by this further brings the arm fully clear of the rack (typically a vertical lift). Must be non-zero. |

#### Rack-side poses

- `slide-pose` = `slot-pose + slide-offset-mm`
- `lift-pose` = `slot-pose + lift-offset-mm`
- `slot-pose` = mechanically engaged

Take traversal: `parking-pose → slide-pose → slot-pose → lift-pose → parking-pose`.

Release traversal: `parking-pose → lift-pose → slot-pose → slide-pose → parking-pose`.

Take enters via the slide and exits via the lift so the engaged tool stays engaged; release enters via the lift and exits via the slide so the deposited tool isn't pulled back out.

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
