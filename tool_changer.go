package toolchanger

import (
	"context"
	"fmt"
	"sync"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
)

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

	motionMu sync.Mutex
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
	if v, ok := cmd["set_world_state"]; ok {
		return s.doSetWorldState(v)
	}
	if v, ok := cmd["switch_tool"]; ok {
		return s.doSwitchTool(ctx, v)
	}
	if _, ok := cmd["release"]; ok {
		return s.doRelease(ctx)
	}
	return nil, fmt.Errorf("unknown command, expected 'switch_tool', 'release', or 'set_world_state'")
}

func (s *toolChanger) knownTool(name string) bool {
	for _, t := range s.cfg.Tools {
		if t.Name == name {
			return true
		}
	}
	return false
}

func (s *toolChanger) findTool(name string) ToolConfig {
	for _, t := range s.cfg.Tools {
		if t.Name == name {
			return t
		}
	}
	return ToolConfig{}
}

// rackVisitSteps returns the six-step round-trip for visiting one tool's rack
// slot: parking -> clear -> slide -> slot -> slide -> clear -> parking. The
// mechanical changer determines whether the visit picks up or releases based
// on the arm's pre-visit state; the motion is identical either way.
func (s *toolChanger) rackVisitSteps(tool ToolConfig) []PlanStep {
	slidePose := Pose{
		Point:       tool.SlotPose.Point.Add(tool.SlideOffsetMM),
		Orientation: tool.SlotPose.Orientation,
	}
	clearPose := Pose{
		Point:       slidePose.Point.Add(tool.LiftOffsetMM),
		Orientation: tool.SlotPose.Orientation,
	}
	return []PlanStep{
		{Type: transitInStepType, ToolName: tool.Name, Goal: clearPose, Constraints: s.cfg.TransitConstraints},
		{Type: liftDownStepType, ToolName: tool.Name, Goal: slidePose, Constraints: s.cfg.LiftConstraints},
		{Type: slideInStepType, ToolName: tool.Name, Goal: tool.SlotPose, Constraints: s.cfg.SlideConstraints},
		{Type: slideOutStepType, ToolName: tool.Name, Goal: slidePose, Constraints: s.cfg.SlideConstraints},
		{Type: liftUpStepType, ToolName: tool.Name, Goal: clearPose, Constraints: s.cfg.LiftConstraints},
		{Type: transitOutStepType, ToolName: tool.Name, Goal: s.cfg.ParkingPose, Constraints: s.cfg.TransitConstraints},
	}
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

func (s *toolChanger) doRelease(ctx context.Context) (map[string]interface{}, error) {
	s.motionMu.Lock()
	defer s.motionMu.Unlock()

	s.mu.Lock()
	cur := s.currentTool
	ws := s.worldState
	s.mu.Unlock()

	if cur == nil {
		return map[string]interface{}{"success": true, "released": nil}, nil
	}

	plan, err := s.plan(ctx, s.rackVisitSteps(s.findTool(*cur)), ws)
	if err != nil {
		return nil, fmt.Errorf("release: %w", err)
	}
	if err := s.execute(ctx, plan); err != nil {
		return nil, fmt.Errorf("release: %w", err)
	}

	s.mu.Lock()
	s.currentTool = nil
	s.mu.Unlock()

	return map[string]interface{}{"success": true, "released": *cur}, nil
}

func (s *toolChanger) doSwitchTool(ctx context.Context, v interface{}) (map[string]interface{}, error) {
	name, ok := v.(string)
	if !ok {
		return nil, fmt.Errorf("switch_tool: value must be a string, got %T", v)
	}
	if !s.knownTool(name) {
		return nil, fmt.Errorf("unknown tool %q", name)
	}

	s.motionMu.Lock()
	defer s.motionMu.Unlock()

	s.mu.Lock()
	cur := s.currentTool
	ws := s.worldState
	s.mu.Unlock()

	var from interface{}
	if cur != nil {
		from = *cur
	}

	if cur != nil && *cur == name {
		return map[string]interface{}{
			"success": true,
			"changed": false,
			"from":    from,
			"to":      name,
		}, nil
	}

	var steps []PlanStep
	if cur != nil {
		steps = append(steps, s.rackVisitSteps(s.findTool(*cur))...)
	}
	steps = append(steps, s.rackVisitSteps(s.findTool(name))...)

	plan, err := s.plan(ctx, steps, ws)
	if err != nil {
		return nil, fmt.Errorf("switch_tool: %w", err)
	}
	if err := s.execute(ctx, plan); err != nil {
		return nil, fmt.Errorf("switch_tool: %w", err)
	}

	s.mu.Lock()
	s.currentTool = &name
	s.mu.Unlock()

	return map[string]interface{}{
		"success": true,
		"changed": true,
		"from":    from,
		"to":      name,
	}, nil
}

func (s *toolChanger) Close(ctx context.Context) error {
	return nil
}
