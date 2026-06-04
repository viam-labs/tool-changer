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

type step struct {
	name        string
	goal        Pose
	constraints *motionplan.Constraints
}

func (s *toolChanger) planAll(
	ctx context.Context,
	steps []step,
	worldState *referenceframe.WorldState,
) ([]motionplan.Trajectory, error) {
	fs, err := framesystem.NewFromService(ctx, s.fsService, nil)
	if err != nil {
		return nil, fmt.Errorf("build frame system: %w", err)
	}

	currentInputs, err := s.arm.CurrentInputs(ctx)
	if err != nil {
		return nil, fmt.Errorf("get arm inputs: %w", err)
	}

	startInputs := referenceframe.NewZeroInputs(fs)
	startInputs[s.cfg.Arm] = currentInputs
	startState := armplanning.NewPlanState(nil, startInputs)

	trajectories := make([]motionplan.Trajectory, 0, len(steps))
	for _, st := range steps {
		goalPose := spatialmath.NewPose(st.goal.Point, st.goal.Orientation)
		goalState := armplanning.NewPlanState(
			referenceframe.FrameSystemPoses{
				s.cfg.Arm: referenceframe.NewPoseInFrame(referenceframe.World, goalPose),
			},
			nil,
		)
		req := &armplanning.PlanRequest{
			FrameSystem: fs,
			WorldState:  worldState,
			StartState:  startState,
			Goals:       []*armplanning.PlanState{goalState},
			Constraints: st.constraints,
		}
		p, _, err := armplanning.PlanMotion(ctx, s.logger, req)
		if err != nil {
			return nil, fmt.Errorf("plan step %q: %w", st.name, err)
		}
		traj := p.Trajectory()
		trajectories = append(trajectories, traj)
		if len(traj) > 0 {
			startState = armplanning.NewPlanState(nil, traj[len(traj)-1])
		}
	}
	return trajectories, nil
}

func (s *toolChanger) executeAll(ctx context.Context, trajectories []motionplan.Trajectory) error {
	for _, traj := range trajectories {
		armInputs := make([][]referenceframe.Input, len(traj))
		for i, fsInputs := range traj {
			armInputs[i] = fsInputs[s.cfg.Arm]
		}
		if err := s.arm.MoveThroughJointPositions(ctx, armInputs, nil, nil); err != nil {
			return err
		}
	}
	return nil
}

// TODO: if motion commands ever need to override the stored world-state for a
// single call (e.g. a transient obstacle the caller doesn't want to keep), add
// an optional per-call world-state field that merges with or replaces stored.
// The current set-once contract stays backwards compatible.
func parseWorldState(raw interface{}) (*referenceframe.WorldState, error) {
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
