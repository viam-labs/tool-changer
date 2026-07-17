package toolchanger

import (
	"context"
	"encoding/json"
	"fmt"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/spatialmath"
	"google.golang.org/protobuf/encoding/protojson"
)

func (s *toolChanger) attachedUUID() string {
	return "tool-changer/" + s.name.Name + "/attached"
}

func (s *toolChanger) publishAttachedTool(ctx context.Context, tool ToolConfig) error {
	if s.wss == nil || tool.Geometry == nil {
		return nil
	}
	payload, err := buildSetTransformPayload(s.attachedUUID(), tool)
	if err != nil {
		return err
	}
	_, err = s.wss.DoCommand(ctx, map[string]interface{}{"set_transform": payload})
	return err
}

func (s *toolChanger) removeAttachedTool(ctx context.Context) error {
	if s.wss == nil {
		return nil
	}
	_, err := s.wss.DoCommand(ctx, map[string]interface{}{
		"remove_transform": map[string]interface{}{"uuid": s.attachedUUID()},
	})
	return err
}

func buildAttachedTransform(uuid string, tool ToolConfig) (*commonpb.Transform, error) {
	geoProto, err := tool.Geometry.ToProtobuf()
	if err != nil {
		return nil, fmt.Errorf("tool %q: geometry: %w", tool.Name, err)
	}
	return &commonpb.Transform{
		ReferenceFrame:      tool.Name,
		PoseInObserverFrame: &commonpb.PoseInFrame{ReferenceFrame: tool.AttachFrame, Pose: attachPose(tool)},
		PhysicalObject:      geoProto,
		Uuid:                []byte(uuid),
	}, nil
}

func buildSetTransformPayload(uuid string, tool ToolConfig) (map[string]interface{}, error) {
	t, err := buildAttachedTransform(uuid, tool)
	if err != nil {
		return nil, err
	}
	raw, err := protojson.MarshalOptions{UseProtoNames: true}.Marshal(t)
	if err != nil {
		return nil, err
	}
	var m map[string]interface{}
	if err := json.Unmarshal(raw, &m); err != nil {
		return nil, err
	}
	// Uuid appears in the proto marshalling as base64 bytes; overwrite with the
	// plain string the aggregator expects at the top level.
	m["uuid"] = uuid
	return m, nil
}

func attachPose(tool ToolConfig) *commonpb.Pose {
	if tool.AttachPose == nil {
		return spatialmath.PoseToProtobuf(spatialmath.NewZeroPose())
	}
	sp := spatialmath.NewPose(tool.AttachPose.Point, tool.AttachPose.Orientation)
	return spatialmath.PoseToProtobuf(sp)
}
