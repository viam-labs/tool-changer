package toolchanger

import (
	"context"
	"errors"
	"fmt"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/worldstatestore"
)

func (s *toolChanger) fetchAggregatorTransforms(ctx context.Context) ([]*commonpb.Transform, error) {
	if s.wss == nil {
		return nil, nil
	}
	uuids, err := s.wss.ListUUIDs(ctx, nil)
	if err != nil {
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
			t, err := buildAttachedTransform(s.attachedUUID(), tool)
			if err != nil {
				return nil, err
			}
			merged.Transforms = append(merged.Transforms, t)
		}
	}

	if len(merged.Obstacles) == 0 && len(merged.Transforms) == 0 {
		return nil, nil
	}
	return referenceframe.WorldStateFromProtobuf(merged)
}
