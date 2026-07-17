package toolchanger

import (
	"context"
	"encoding/json"
	"fmt"
	"time"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
	"google.golang.org/protobuf/encoding/protojson"
)

const (
	transitInStepType  = "TransitIn"
	liftDownStepType   = "LiftDown"
	slideInStepType    = "SlideIn"
	slideOutStepType   = "SlideOut"
	liftUpStepType     = "LiftUp"
	transitOutStepType = "TransitOut"
)

const (
	plannedStatus         = "Planned"
	executedStatus        = "Executed"
	failedToExecuteStatus = "FailedToExecute"
)

type PlanStep struct {
	Type         string                   `json:"type"`
	ToolName     string                   `json:"tool_name,omitempty"`
	AttachedTool string                   `json:"attached_tool,omitempty"`
	Goal         Pose                     `json:"goal"`
	Constraints  *motionplan.Constraints  `json:"constraints,omitempty"`
	Request      *armplanning.PlanRequest `json:"-"`
	Trajectory   motionplan.Trajectory    `json:"trajectory,omitempty"`
	Status       string                   `json:"status"`
	PlanningTime time.Duration            `json:"planning_time"`
}

type Plan struct {
	Steps             []PlanStep    `json:"steps"`
	TotalPlanningTime time.Duration `json:"total_planning_time"`
}

func stepLabel(st PlanStep) string {
	if st.ToolName == "" {
		return st.Type
	}
	return st.Type + "-" + st.ToolName
}

func (s *toolChanger) plan(
	ctx context.Context,
	steps []PlanStep,
	baseWS *commonpb.WorldState,
) (*Plan, error) {
	fs, err := framesystem.NewFromService(ctx, s.fsService, nil)
	if err != nil {
		return nil, fmt.Errorf("build frame system: %w", err)
	}

	currentInputs, err := s.arm.CurrentInputs(ctx)
	if err != nil {
		return nil, fmt.Errorf("get arm inputs: %w", err)
	}

	aggregatorTransforms, err := s.fetchAggregatorTransforms(ctx)
	if err != nil {
		return nil, fmt.Errorf("fetch world state store transforms: %w", err)
	}

	startInputs := referenceframe.NewZeroInputs(fs)
	startInputs[s.cfg.Arm] = currentInputs
	startState := armplanning.NewPlanState(nil, startInputs)

	plan := &Plan{Steps: make([]PlanStep, 0, len(steps))}
	for _, st := range steps {
		goalPose := spatialmath.NewPose(st.Goal.Point, st.Goal.Orientation)
		goalState := armplanning.NewPlanState(
			referenceframe.FrameSystemPoses{
				s.cfg.Arm: referenceframe.NewPoseInFrame(referenceframe.World, goalPose),
			},
			nil,
		)
		stepWS, err := s.buildStepWorldState(baseWS, aggregatorTransforms, st.AttachedTool)
		if err != nil {
			return nil, fmt.Errorf("build world state for step %q: %w", stepLabel(st), err)
		}
		req := &armplanning.PlanRequest{
			FrameSystem: fs,
			WorldState:  stepWS,
			StartState:  startState,
			Goals:       []*armplanning.PlanState{goalState},
			Constraints: st.Constraints,
		}
		planStart := time.Now()
		p, _, err := armplanning.PlanMotion(ctx, s.logger, req)
		planDur := time.Since(planStart)
		if err != nil {
			return nil, fmt.Errorf("plan step %q: %w", stepLabel(st), err)
		}
		traj := p.Trajectory()
		st.Request = req
		st.Trajectory = traj
		st.Status = plannedStatus
		st.PlanningTime = planDur
		plan.Steps = append(plan.Steps, st)
		plan.TotalPlanningTime += planDur
		if len(traj) > 0 {
			startState = armplanning.NewPlanState(nil, traj[len(traj)-1])
		}
	}
	return plan, nil
}

// execute runs the trajectory for each PlanStep and, between steps, publishes
// or removes the attached-tool transform to the world state store based on
// PlanStep.AttachedTool transitions. Returns (motionErr, storeErr) separately:
// motion errors abort the sequence and should skip state updates upstream;
// store errors are logged but do not abort motion, so the arm reaches a safe
// end pose even when the store is temporarily unreachable.
func (s *toolChanger) execute(ctx context.Context, plan *Plan) (motionErr, storeErr error) {
	slideOpts := s.cfg.SlideSpeed.MoveOptions()
	for i := range plan.Steps {
		step := &plan.Steps[i]
		armInputs := make([][]referenceframe.Input, len(step.Trajectory))
		for j, fsInputs := range step.Trajectory {
			armInputs[j] = fsInputs[s.cfg.Arm]
		}
		var opts *arm.MoveOptions
		if step.Type == slideInStepType || step.Type == slideOutStepType {
			opts = slideOpts
		}
		if err := s.arm.MoveThroughJointPositions(ctx, armInputs, opts, nil); err != nil {
			step.Status = failedToExecuteStatus
			return fmt.Errorf("execute step %q: %w", stepLabel(*step), err), storeErr
		}
		step.Status = executedStatus

		if i+1 < len(plan.Steps) {
			if err := s.reflectAttachTransition(ctx, step.AttachedTool, plan.Steps[i+1].AttachedTool); err != nil {
				s.logger.CWarnw(ctx, "world state store update failed; motion continues",
					"error", err, "after_step", stepLabel(*step))
				if storeErr == nil {
					storeErr = err
				}
			}
		}
	}
	return nil, storeErr
}

func (s *toolChanger) reflectAttachTransition(ctx context.Context, prev, next string) error {
	if prev == next {
		return nil
	}
	if prev != "" {
		if err := s.removeAttachedTool(ctx); err != nil {
			return fmt.Errorf("remove attached tool %q: %w", prev, err)
		}
	}
	if next != "" {
		if err := s.publishAttachedTool(ctx, s.findTool(next)); err != nil {
			return fmt.Errorf("publish attached tool %q: %w", next, err)
		}
	}
	return nil
}

func parseWorldState(raw interface{}) (*commonpb.WorldState, error) {
	bytes, err := json.Marshal(raw)
	if err != nil {
		return nil, fmt.Errorf("marshaling world-state: %w", err)
	}
	var proto commonpb.WorldState
	if err := protojson.Unmarshal(bytes, &proto); err != nil {
		return nil, fmt.Errorf("unmarshaling world-state: %w", err)
	}
	return &proto, nil
}
