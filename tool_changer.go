package toolchanger

import (
	"context"
	"fmt"
	"sync"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/services/worldstatestore"
)

type toolChanger struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *Config

	arm       arm.Arm
	fsService framesystem.Service
	grippers  map[string]gripper.Gripper
	wss       worldstatestore.Service

	mu          sync.Mutex
	currentTool *string
	worldState  *commonpb.WorldState

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

	grippers := make(map[string]gripper.Gripper)
	for _, tool := range cfg.Tools {
		if tool.Gripper == "" {
			continue
		}
		g, err := gripper.FromProvider(deps, tool.Gripper)
		if err != nil {
			return nil, fmt.Errorf("tool %q: failed to get gripper %q: %w", tool.Name, tool.Gripper, err)
		}
		grippers[tool.Name] = g
	}

	var wss worldstatestore.Service
	if cfg.WorldStateStore != "" {
		wss, err = worldstatestore.FromDependencies(deps, cfg.WorldStateStore)
		if err != nil {
			return nil, fmt.Errorf("failed to get world state store %q: %w", cfg.WorldStateStore, err)
		}
	}

	return &toolChanger{
		name:      rawConf.ResourceName(),
		logger:    logger,
		cfg:       cfg,
		arm:       a,
		fsService: fs,
		grippers:  grippers,
		wss:       wss,
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
	if _, ok := cmd["get_status"]; ok {
		return s.doGetStatus()
	}
	return nil, fmt.Errorf("unknown command, expected 'switch_tool', 'release', 'set_world_state', or 'get_status'")
}

// doGetStatus reports the current attachment state without moving the arm.
func (s *toolChanger) doGetStatus() (map[string]interface{}, error) {
	s.mu.Lock()
	cur := s.currentTool
	wsSet := s.worldState != nil
	s.mu.Unlock()

	var current interface{}
	if cur != nil {
		current = *cur
	}
	return map[string]interface{}{
		"success":         true,
		"attached":        cur != nil,
		"current_tool":    current,
		"world_state_set": wsSet,
	}, nil
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

// rackPoses returns the two derived poses around a tool's slot: slide-pose
// (slot + slide) and lift-pose (slot + lift). Orientation is taken from
// slot-pose throughout.
func (s *toolChanger) rackPoses(tool ToolConfig) (slidePose, liftPose Pose) {
	slidePose = Pose{
		Point:       tool.SlotPose.Point.Add(tool.SlideOffsetMM),
		Orientation: tool.SlotPose.Orientation,
	}
	liftPose = Pose{
		Point:       tool.SlotPose.Point.Add(tool.LiftOffsetMM),
		Orientation: tool.SlotPose.Orientation,
	}
	return
}

// mergeSlideConstraints returns a *motionplan.Constraints that combines the
// service-level slide constraints with the per-tool allowed collision pairs.
// The result is a fresh value; the base constraints and the allowed list
// are not mutated. Returns nil when both base and allowed are empty.
func mergeSlideConstraints(base *motionplan.Constraints, allowed []motionplan.CollisionSpecificationAllowedFrameCollisions) *motionplan.Constraints {
	if len(allowed) == 0 {
		return base
	}
	result := &motionplan.Constraints{}
	if base != nil {
		*result = *base
	}
	result.CollisionSpecification = append(
		append([]motionplan.CollisionSpecification{}, result.CollisionSpecification...),
		motionplan.CollisionSpecification{Allows: append([]motionplan.CollisionSpecificationAllowedFrameCollisions{}, allowed...)},
	)
	return result
}

// takeSteps returns the 4-step traversal for picking a tool up out of the
// rack: parking -> lift -> slot -> slide -> parking. Engagement happens on
// the descent from lift-pose to slot; the arm exits via slide-pose with
// the tool attached.
func (s *toolChanger) takeSteps(tool ToolConfig) []PlanStep {
	slidePose, liftPose := s.rackPoses(tool)
	slideConstraints := mergeSlideConstraints(s.cfg.SlideConstraints, tool.SlideAllowedCollisions)
	return []PlanStep{
		{Type: transitInStepType, ToolName: tool.Name, Goal: liftPose, Constraints: s.cfg.TransitConstraints},
		{Type: liftDownStepType, ToolName: tool.Name, Goal: tool.SlotPose, Constraints: s.cfg.LiftConstraints},
		{Type: slideOutStepType, ToolName: tool.Name, AttachedTool: tool.Name, Goal: slidePose, Constraints: slideConstraints},
		{Type: transitOutStepType, ToolName: tool.Name, AttachedTool: tool.Name, Goal: s.cfg.ParkingPose, Constraints: s.cfg.TransitConstraints},
	}
}

// releaseSteps returns the 4-step traversal for putting a tool back in the
// rack: parking -> slide -> slot -> lift -> parking. Deposit happens on
// the slide-in from slide-pose to slot; the arm exits via lift-pose,
// leaving the tool in the holder.
func (s *toolChanger) releaseSteps(tool ToolConfig) []PlanStep {
	slidePose, liftPose := s.rackPoses(tool)
	slideConstraints := mergeSlideConstraints(s.cfg.SlideConstraints, tool.SlideAllowedCollisions)
	return []PlanStep{
		{Type: transitInStepType, ToolName: tool.Name, AttachedTool: tool.Name, Goal: slidePose, Constraints: s.cfg.TransitConstraints},
		{Type: slideInStepType, ToolName: tool.Name, AttachedTool: tool.Name, Goal: tool.SlotPose, Constraints: slideConstraints},
		{Type: liftUpStepType, ToolName: tool.Name, Goal: liftPose, Constraints: s.cfg.LiftConstraints},
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

	plan, planErr := s.plan(ctx, s.releaseSteps(s.findTool(*cur)), ws)
	var motionErr, storeErr error
	if planErr == nil {
		motionErr, storeErr = s.execute(ctx, plan)
	}
	finalMotionErr := planErr
	if finalMotionErr == nil {
		finalMotionErr = motionErr
	}
	if s.cfg.SavePlanRequests {
		s.savePlanRequests(plan, "release", *cur, "", finalMotionErr)
	}
	if finalMotionErr != nil {
		return nil, fmt.Errorf("release: %w", finalMotionErr)
	}

	s.mu.Lock()
	released := *cur
	s.currentTool = nil
	s.mu.Unlock()

	if storeErr != nil {
		return nil, fmt.Errorf("release: released %q but world_state_store update failed: %w", released, storeErr)
	}

	return map[string]interface{}{"success": true, "released": released}, nil
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
		steps = append(steps, s.releaseSteps(s.findTool(*cur))...)
	}
	steps = append(steps, s.takeSteps(s.findTool(name))...)

	plan, planErr := s.plan(ctx, steps, ws)
	var motionErr, storeErr error
	if planErr == nil {
		motionErr, storeErr = s.execute(ctx, plan)
	}
	finalMotionErr := planErr
	if finalMotionErr == nil {
		finalMotionErr = motionErr
	}
	if s.cfg.SavePlanRequests {
		fromName := ""
		if cur != nil {
			fromName = *cur
		}
		s.savePlanRequests(plan, "switch_tool", fromName, name, finalMotionErr)
	}
	if finalMotionErr != nil {
		return nil, fmt.Errorf("switch_tool: %w", finalMotionErr)
	}

	s.mu.Lock()
	s.currentTool = &name
	s.mu.Unlock()

	if g, ok := s.grippers[name]; ok {
		if _, err := g.DoCommand(ctx, map[string]interface{}{"activate": true}); err != nil {
			return nil, fmt.Errorf("switch_tool: swapped to %q but gripper activate failed (retry with DoCommand{\"activate\":true} on the gripper): %w", name, err)
		}
	}

	if storeErr != nil {
		return nil, fmt.Errorf("switch_tool: swapped to %q but world_state_store update failed: %w", name, storeErr)
	}

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
