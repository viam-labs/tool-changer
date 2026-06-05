package toolchanger

import (
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	genericservice "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/spatialmath"
)

func init() {
	resource.RegisterService(genericservice.API, DefaultModel,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newToolChanger,
		},
	)
}

type Pose struct {
	Point       r3.Vector                             `json:"point,omitzero"`
	Orientation *spatialmath.OrientationVectorDegrees `json:"orientation"`
}

type ToolConfig struct {
	Name          string    `json:"name"`
	SlotPose      Pose      `json:"slot-pose"`
	SlideOffsetMM r3.Vector `json:"slide-offset-mm,omitzero"`
	LiftOffsetMM  r3.Vector `json:"lift-offset-mm,omitzero"`
}

type Config struct {
	Arm                string                  `json:"arm"`
	ParkingPose        Pose                    `json:"parking-pose"`
	Tools              []ToolConfig            `json:"tools"`
	TransitConstraints *motionplan.Constraints `json:"transit-constraints,omitempty"`
	LiftConstraints    *motionplan.Constraints `json:"lift-constraints,omitempty"`
	SlideConstraints   *motionplan.Constraints `json:"slide-constraints,omitempty"`
}

func (c *Config) Validate(path string) ([]string, []string, error) {
	if c.Arm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}
	if c.ParkingPose.Orientation == nil {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "parking-pose.orientation")
	}
	if err := c.ParkingPose.Orientation.IsValid(); err != nil {
		return nil, nil, fmt.Errorf("%s: parking-pose.orientation: %w", path, err)
	}
	if len(c.Tools) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "tools")
	}

	seen := map[string]bool{}
	for i, tool := range c.Tools {
		toolPath := fmt.Sprintf("%s.tools[%d]", path, i)
		if seen[tool.Name] {
			return nil, nil, fmt.Errorf("%s: duplicate tool name %q", toolPath, tool.Name)
		}
		seen[tool.Name] = true
		if err := tool.Validate(toolPath); err != nil {
			return nil, nil, err
		}
	}

	return []string{c.Arm, framesystem.PublicServiceName.String()}, nil, nil
}

func (t ToolConfig) Validate(path string) error {
	if t.Name == "" {
		return resource.NewConfigValidationFieldRequiredError(path, "name")
	}
	if t.SlotPose.Orientation == nil {
		return resource.NewConfigValidationFieldRequiredError(path, "slot-pose.orientation")
	}
	if err := t.SlotPose.Orientation.IsValid(); err != nil {
		return fmt.Errorf("%s: slot-pose.orientation: %w", path, err)
	}
	if t.SlideOffsetMM == (r3.Vector{}) {
		return fmt.Errorf("%s: slide-offset-mm is required and must be non-zero", path)
	}
	if t.LiftOffsetMM == (r3.Vector{}) {
		return fmt.Errorf("%s: lift-offset-mm is required and must be non-zero", path)
	}
	return nil
}
