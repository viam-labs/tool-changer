# tool-changer

Viam module providing a generic tool-changer service for swapping end-effectors on a robot arm.

## Model viam-labs:tool-changer:default

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
| `transit-constraints` | object | Optional | RDK `motionplan.Constraints` applied to moves between `parking-pose` and each tool's `clear-pose`. Defaults to `nil` (free plan). |
| `lift-constraints` | object | Optional | RDK `motionplan.Constraints` applied to the descent between `clear-pose` and `slide-pose`, and the lift on the reverse. Defaults to `nil` (free plan). |
| `slide-constraints` | object | Optional | RDK `motionplan.Constraints` applied to the final slide into `slot-pose` and the slide-out on release. Defaults to `nil` (free plan). |

##### `tools[]` entries

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `name` | string | Required | Caller-facing tool name (e.g. `"tongs"`, `"spoon"`). Must be unique within `tools`. |
| `slot-pose` | object | Required | Pose at which the robot-side changer is mechanically engaged in this tool's rack holder. `orientation` is required. |
| `slide-offset-mm` | object | Required | Slide direction relative to `slot-pose`. From `slot-pose + slide-offset-mm`, sliding to `slot-pose` is what mechanically engages the tool. Must be non-zero. |
| `lift-offset-mm` | object | Required | Perpendicular clearance from the rack relative to `slot-pose + slide-offset-mm`. From that point, moving by this further brings the arm fully clear of the rack (typically a vertical lift). Must be non-zero. |

#### Rack-side poses

- `clear-pose` = `slot-pose + slide-offset-mm + lift-offset-mm`
- `slide-pose` = `slot-pose + slide-offset-mm`
- `slot-pose` = mechanically engaged
