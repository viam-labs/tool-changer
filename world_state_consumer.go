package toolchanger

import (
	"context"
	"errors"
	"fmt"
	"strings"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/worldstatestore"
	"go.viam.com/rdk/spatialmath"
)

func (s *toolChanger) fetchAggregatorTransforms(ctx context.Context) ([]*commonpb.Transform, error) {
	if s.wss == nil {
		return nil, nil
	}
	uuids, err := s.wss.ListUUIDs(ctx, nil)
	if err != nil {
		// The world_state_store server treats an empty store as an error
		// (ErrNilResponse). When that error crosses gRPC it becomes a
		// *status.Error and loses its sentinel identity, so errors.Is no
		// longer matches. Fall back to a message-text check so an empty
		// store surfaces as "no transforms" rather than a failed switch.
		if errors.Is(err, worldstatestore.ErrNilResponse) ||
			strings.Contains(err.Error(), worldstatestore.ErrNilResponse.Error()) {
			return nil, nil
		}
		return nil, err
	}
	own := s.attachedUUID()
	out := make([]*commonpb.Transform, 0, len(uuids))
	for _, u := range uuids {
		if string(u) == own {
			continue
		}
		t, err := s.wss.GetTransform(ctx, u, nil)
		if err != nil {
			if errors.Is(err, worldstatestore.ErrNilResponse) {
				continue
			}
			return nil, fmt.Errorf("get transform %q: %w", string(u), err)
		}
		out = append(out, t)
	}
	return out, nil
}

func (s *toolChanger) buildStepWorldState(
	base *commonpb.WorldState,
	aggregator []*commonpb.Transform,
	attachedTool string,
) (*referenceframe.WorldState, error) {
	merged := &commonpb.WorldState{}
	if base != nil {
		merged.Obstacles = append(merged.Obstacles, base.Obstacles...)
		merged.Transforms = append(merged.Transforms, base.Transforms...)
	}
	merged.Transforms = append(merged.Transforms, aggregator...)

	if attachedTool != "" {
		tool := s.findTool(attachedTool)
		if tool.Geometry != nil {
			gif, err := buildAttachedObstacle(tool)
			if err != nil {
				return nil, err
			}
			merged.Obstacles = append(merged.Obstacles, gif)
		}
	}

	if len(merged.Obstacles) == 0 && len(merged.Transforms) == 0 {
		return nil, nil
	}
	return referenceframe.WorldStateFromProtobuf(merged)
}

func buildAttachedObstacle(tool ToolConfig) (*commonpb.GeometriesInFrame, error) {
	geoProto, err := tool.Geometry.ToProtobuf()
	if err != nil {
		return nil, fmt.Errorf("tool %q: geometry: %w", tool.Name, err)
	}
	attach := spatialmath.NewPoseFromProtobuf(attachPose(tool))
	var existing spatialmath.Pose = spatialmath.NewZeroPose()
	if geoProto.GetCenter() != nil {
		existing = spatialmath.NewPoseFromProtobuf(geoProto.GetCenter())
	}
	geoProto.Center = spatialmath.PoseToProtobuf(spatialmath.Compose(attach, existing))
	return &commonpb.GeometriesInFrame{
		ReferenceFrame: tool.AttachFrame,
		Geometries:     []*commonpb.Geometry{geoProto},
	}, nil
}
