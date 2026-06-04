package toolchanger

import (
	"context"
	"fmt"
	"sync"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
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
	Name             string    `json:"name"`
	SlotPose         Pose      `json:"slot-pose"`
	ApproachOffsetMM r3.Vector `json:"approach-offset-mm,omitzero"`
}

type Config struct {
	Arm                 string                  `json:"arm"`
	ParkingPose         Pose                    `json:"parking-pose"`
	Tools               []ToolConfig            `json:"tools"`
	ApproachConstraints *motionplan.Constraints `json:"approach-constraints,omitempty"`
	DockConstraints     *motionplan.Constraints `json:"dock-constraints,omitempty"`
	Extra               map[string]any          `json:"extra,omitempty"`
	SavePlans           bool                    `json:"save-plans,omitempty"`
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
		if tool.Name == "" {
			return nil, nil, resource.NewConfigValidationFieldRequiredError(toolPath, "name")
		}
		if seen[tool.Name] {
			return nil, nil, fmt.Errorf("%s: duplicate tool name %q", toolPath, tool.Name)
		}
		seen[tool.Name] = true
		if tool.SlotPose.Orientation == nil {
			return nil, nil, resource.NewConfigValidationFieldRequiredError(toolPath, "slot-pose.orientation")
		}
		if err := tool.SlotPose.Orientation.IsValid(); err != nil {
			return nil, nil, fmt.Errorf("%s: slot-pose.orientation: %w", toolPath, err)
		}
		if tool.ApproachOffsetMM == (r3.Vector{}) {
			return nil, nil, fmt.Errorf("%s: approach-offset-mm must be non-zero", toolPath)
		}
	}

	return []string{c.Arm, framesystem.PublicServiceName.String()}, nil, nil
}

type toolChanger struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *Config

	arm       arm.Arm
	fsService framesystem.Service

	mu          sync.Mutex
	currentTool *string
	worldState  *referenceframe.WorldState
}

func newToolChanger(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	cfg, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	a, err := arm.FromProvider(deps, cfg.Arm)
	if err != nil {
		return nil, fmt.Errorf("failed to get arm %q: %w", cfg.Arm, err)
	}

	fs, err := framesystem.FromProvider(deps)
	if err != nil {
		return nil, fmt.Errorf("failed to get framesystem service: %w", err)
	}

	return &toolChanger{
		name:      rawConf.ResourceName(),
		logger:    logger,
		cfg:       cfg,
		arm:       a,
		fsService: fs,
	}, nil
}

func (s *toolChanger) Name() resource.Name {
	return s.name
}

func (s *toolChanger) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (s *toolChanger) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["get_current_tool"]; ok {
		return s.doGetCurrentTool(), nil
	}
	if v, ok := cmd["set_current_tool"]; ok {
		return s.doSetCurrentTool(v)
	}
	if v, ok := cmd["set_world_state"]; ok {
		return s.doSetWorldState(v)
	}
	if _, ok := cmd["park"]; ok {
		return s.doPark(ctx)
	}
	return nil, fmt.Errorf("unknown command, expected 'get_current_tool', 'set_current_tool', 'set_world_state', or 'park'")
}

func (s *toolChanger) doGetCurrentTool() map[string]interface{} {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.currentTool == nil {
		return map[string]interface{}{"tool": nil}
	}
	return map[string]interface{}{"tool": *s.currentTool}
}

func (s *toolChanger) doSetCurrentTool(v interface{}) (map[string]interface{}, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	if v == nil {
		s.currentTool = nil
		return map[string]interface{}{"success": true, "tool": nil}, nil
	}
	name, ok := v.(string)
	if !ok {
		return nil, fmt.Errorf("set_current_tool: value must be a string or null, got %T", v)
	}
	if !s.knownTool(name) {
		return nil, fmt.Errorf("unknown tool %q", name)
	}
	s.currentTool = &name
	return map[string]interface{}{"success": true, "tool": name}, nil
}

func (s *toolChanger) knownTool(name string) bool {
	for _, t := range s.cfg.Tools {
		if t.Name == name {
			return true
		}
	}
	return false
}

func (s *toolChanger) doSetWorldState(v interface{}) (map[string]interface{}, error) {
	if v == nil {
		s.mu.Lock()
		s.worldState = nil
		s.mu.Unlock()
		return map[string]interface{}{"success": true, "set": false}, nil
	}
	ws, err := parseWorldState(v)
	if err != nil {
		return nil, fmt.Errorf("set_world_state: %w", err)
	}
	s.mu.Lock()
	s.worldState = ws
	s.mu.Unlock()
	return map[string]interface{}{"success": true, "set": true}, nil
}

func (s *toolChanger) doPark(ctx context.Context) (map[string]interface{}, error) {
	s.mu.Lock()
	ws := s.worldState
	s.mu.Unlock()

	traj, err := s.plan(ctx, s.cfg.ParkingPose, ws)
	if err != nil {
		return nil, fmt.Errorf("park: %w", err)
	}
	if err := s.execute(ctx, traj); err != nil {
		return nil, fmt.Errorf("park: %w", err)
	}
	return nil, nil
}

func (s *toolChanger) Close(ctx context.Context) error {
	return nil
}
