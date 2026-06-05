package toolchanger

import (
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
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
	Name                   string                                                    `json:"name"`
	SlotPose               Pose                                                      `json:"slot-pose"`
	SlideOffsetMM          r3.Vector                                                 `json:"slide-offset-mm,omitzero"`
	LiftOffsetMM           r3.Vector                                                 `json:"lift-offset-mm,omitzero"`
	SlideAllowedCollisions []motionplan.CollisionSpecificationAllowedFrameCollisions `json:"slide-allowed-collisions,omitempty"`
}

type Config struct {
	Arm                string                  `json:"arm"`
	ParkingPose        Pose                    `json:"parking-pose"`
	Tools              []ToolConfig            `json:"tools"`
	TransitConstraints *motionplan.Constraints `json:"transit-constraints,omitempty"`
	LiftConstraints    *motionplan.Constraints `json:"lift-constraints,omitempty"`
	SlideConstraints   *motionplan.Constraints `json:"slide-constraints,omitempty"`
	SlideSpeed         *SpeedConfig            `json:"slide-speed,omitempty"`
	SavePlanRequests   bool                    `json:"save-plan-requests,omitempty"`
}

type SpeedConfig struct {
	MaxVelDegsPerSec  float64 `json:"max_vel_degs_per_sec,omitempty"`
	MaxAccDegsPerSec2 float64 `json:"max_acc_degs_per_sec2,omitempty"`
}

func (s *SpeedConfig) Validate(path string) error {
	if s.MaxVelDegsPerSec < 0 {
		return fmt.Errorf("%s.max_vel_degs_per_sec must be non-negative", path)
	}
	if s.MaxAccDegsPerSec2 < 0 {
		return fmt.Errorf("%s.max_acc_degs_per_sec2 must be non-negative", path)
	}
	return nil
}

// MoveOptions returns *arm.MoveOptions populated from the config, or nil if
// neither field is positive. Callers should pass nil to
// MoveThroughJointPositions to use the arm's default speed.
func (s *SpeedConfig) MoveOptions() *arm.MoveOptions {
	if s == nil || (s.MaxVelDegsPerSec <= 0 && s.MaxAccDegsPerSec2 <= 0) {
		return nil
	}
	opts := &arm.MoveOptions{}
	if s.MaxVelDegsPerSec > 0 {
		opts.MaxVelRads = s.MaxVelDegsPerSec * math.Pi / 180.0
	}
	if s.MaxAccDegsPerSec2 > 0 {
		opts.MaxAccRads = s.MaxAccDegsPerSec2 * math.Pi / 180.0
	}
	return opts
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

	if c.SlideSpeed != nil {
		if err := c.SlideSpeed.Validate(path + ".slide-speed"); err != nil {
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
