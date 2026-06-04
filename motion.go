package toolchanger

import (
	"context"
	"encoding/json"
	"fmt"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
	"google.golang.org/protobuf/encoding/protojson"
)

func (s *toolChanger) plan(
	ctx context.Context,
	goal Pose,
	worldState *referenceframe.WorldState,
) (motionplan.Trajectory, error) {
	fs, err := framesystem.NewFromService(ctx, s.fsService, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to build frame system: %w", err)
	}

	currentInputs, err := s.arm.CurrentInputs(ctx)
	if err != nil {
		return nil, fmt.Errorf("failed to get arm inputs: %w", err)
	}

	startInputs := referenceframe.NewZeroInputs(fs)
	startInputs[s.cfg.Arm] = currentInputs

	goalPose := spatialmath.NewPose(goal.Point, goal.Orientation)
	goalState := armplanning.NewPlanState(
		referenceframe.FrameSystemPoses{
			s.cfg.Arm: referenceframe.NewPoseInFrame(referenceframe.World, goalPose),
		},
		nil,
	)

	req := &armplanning.PlanRequest{
		FrameSystem: fs,
		WorldState:  worldState,
		StartState:  armplanning.NewPlanState(nil, startInputs),
		Goals:       []*armplanning.PlanState{goalState},
	}

	p, _, err := armplanning.PlanMotion(ctx, s.logger, req)
	if err != nil {
		return nil, fmt.Errorf("failed to plan: %w", err)
	}
	return p.Trajectory(), nil
}

func (s *toolChanger) execute(ctx context.Context, traj motionplan.Trajectory) error {
	armInputs := make([][]referenceframe.Input, len(traj))
	for i, fsInputs := range traj {
		armInputs[i] = fsInputs[s.cfg.Arm]
	}
	return s.arm.MoveThroughJointPositions(ctx, armInputs, nil, nil)
}

func worldStateFromCommand(cmd map[string]interface{}) (*referenceframe.WorldState, error) {
	raw, ok := cmd["world-state"]
	if !ok || raw == nil {
		return nil, nil
	}
	bytes, err := json.Marshal(raw)
	if err != nil {
		return nil, fmt.Errorf("marshaling world-state: %w", err)
	}
	var proto commonpb.WorldState
	if err := protojson.Unmarshal(bytes, &proto); err != nil {
		return nil, fmt.Errorf("unmarshaling world-state: %w", err)
	}
	return referenceframe.WorldStateFromProtobuf(&proto)
}
