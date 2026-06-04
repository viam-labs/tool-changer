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
      "approach-offset-mm": { "X": <float>, "Y": <float>, "Z": <float> }
    }
  ],
  "approach-constraints": <motionplan.Constraints>,
  "dock-constraints": <motionplan.Constraints>,
  "extra": <map[string]any>,
  "save-plans": <bool>
}
```

#### Attributes

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `arm` | string | Required | Name of the `arm` component this service drives through the rack. The framesystem service is added as an implicit dependency for planning. |
| `parking-pose` | object | Required | Pose the arm moves to before entering the rack and returns to after a successful change. `orientation` is required. |
| `tools` | array | Required | One entry per tool slot. Must contain at least one entry with a non-empty unique `name`, a `slot-pose` with `orientation`, and a non-zero `approach-offset-mm`. |
| `approach-constraints` | object | Optional | RDK `motionplan.Constraints` applied to moves between `parking-pose` and each tool's `slot-pose + approach-offset-mm`. Defaults to `nil` (free plan). |
| `dock-constraints` | object | Optional | RDK `motionplan.Constraints` applied to the final linear plunge into `slot-pose` and the retract back out. Defaults to `nil` (free plan). |
| `extra` | object | Optional | Free-form key/value map passed through to `armplanning.PlanRequest.Extra` on every plan call. |
| `save-plans` | bool | Optional | When `true`, every motion command writes a JSON plan record to `/root/.viam/capture/`. Defaults to `false`. |

##### `tools[]` entries

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `name` | string | Required | Caller-facing tool name (e.g. `"tongs"`, `"spoon"`). Must be unique within `tools`. |
| `slot-pose` | object | Required | Pose at which the robot-side changer is mechanically engaged in this tool's rack holder. `orientation` is required. |
| `approach-offset-mm` | object | Required | Per-axis offset from `slot-pose` defining where the linear approach begins. Must be non-zero on at least one axis. |
