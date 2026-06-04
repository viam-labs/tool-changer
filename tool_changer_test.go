package toolchanger

import (
	"context"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func validOrientation() *spatialmath.OrientationVectorDegrees {
	return &spatialmath.OrientationVectorDegrees{OX: 0, OY: 0, OZ: -1, Theta: 0}
}

func validConfig() *Config {
	return &Config{
		Arm: "left-arm",
		ParkingPose: Pose{
			Point:       r3.Vector{X: 250, Y: 0, Z: 600},
			Orientation: validOrientation(),
		},
		Tools: []ToolConfig{
			{
				Name: "tongs",
				SlotPose: Pose{
					Point:       r3.Vector{X: 450, Y: -300, Z: 120},
					Orientation: validOrientation(),
				},
				ApproachOffsetMM: r3.Vector{X: 0, Y: 0, Z: 80},
			},
		},
	}
}

func TestValidate(t *testing.T) {
	tests := []struct {
		name     string
		mutate   func(*Config)
		wantErr  string
		wantDeps []string
	}{
		{
			name:    "missing arm",
			mutate:  func(c *Config) { c.Arm = "" },
			wantErr: `Field: "arm"`,
		},
		{
			name:    "missing parking-pose orientation",
			mutate:  func(c *Config) { c.ParkingPose.Orientation = nil },
			wantErr: "parking-pose.orientation",
		},
		{
			name: "invalid parking-pose orientation",
			mutate: func(c *Config) {
				c.ParkingPose.Orientation = &spatialmath.OrientationVectorDegrees{}
			},
			wantErr: "parking-pose.orientation",
		},
		{
			name:    "empty tools",
			mutate:  func(c *Config) { c.Tools = nil },
			wantErr: `Field: "tools"`,
		},
		{
			name:    "tool with empty name",
			mutate:  func(c *Config) { c.Tools[0].Name = "" },
			wantErr: `Field: "name"`,
		},
		{
			name: "duplicate tool names",
			mutate: func(c *Config) {
				c.Tools = append(c.Tools, c.Tools[0])
			},
			wantErr: "duplicate tool name",
		},
		{
			name:    "tool missing slot-pose orientation",
			mutate:  func(c *Config) { c.Tools[0].SlotPose.Orientation = nil },
			wantErr: `Field: "slot-pose.orientation"`,
		},
		{
			name: "tool invalid slot-pose orientation",
			mutate: func(c *Config) {
				c.Tools[0].SlotPose.Orientation = &spatialmath.OrientationVectorDegrees{}
			},
			wantErr: "slot-pose.orientation",
		},
		{
			name:    "tool with zero approach-offset",
			mutate:  func(c *Config) { c.Tools[0].ApproachOffsetMM = r3.Vector{} },
			wantErr: "approach-offset-mm",
		},
		{
			name:     "happy path",
			mutate:   func(c *Config) {},
			wantDeps: []string{"left-arm", framesystem.PublicServiceName.String()},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			cfg := validConfig()
			tt.mutate(cfg)
			deps, _, err := cfg.Validate("services.toolchanger")

			if tt.wantErr != "" {
				test.That(t, err, test.ShouldNotBeNil)
				test.That(t, err.Error(), test.ShouldContainSubstring, tt.wantErr)
				return
			}
			test.That(t, err, test.ShouldBeNil)
			test.That(t, deps, test.ShouldResemble, tt.wantDeps)
		})
	}
}

func newTestService() *toolChanger {
	return &toolChanger{cfg: validConfig()}
}

func TestDoCommand_UnknownKey(t *testing.T) {
	s := newTestService()
	_, err := s.DoCommand(context.Background(), map[string]interface{}{"nope": true})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "unknown command")
}

func TestDoCommand_SetWorldState_Valid(t *testing.T) {
	s := newTestService()
	// Empty WorldState object decodes to a valid (empty) state.
	res, err := s.DoCommand(context.Background(), map[string]interface{}{
		"set_world_state": map[string]interface{}{},
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res["success"], test.ShouldEqual, true)
	test.That(t, res["set"], test.ShouldEqual, true)
	test.That(t, s.worldState, test.ShouldNotBeNil)
}

func TestDoCommand_SetWorldState_Clear(t *testing.T) {
	s := newTestService()
	_, err := s.DoCommand(context.Background(), map[string]interface{}{
		"set_world_state": map[string]interface{}{},
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, s.worldState, test.ShouldNotBeNil)

	res, err := s.DoCommand(context.Background(), map[string]interface{}{"set_world_state": nil})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res["set"], test.ShouldEqual, false)
	test.That(t, s.worldState, test.ShouldBeNil)
}

func TestDoCommand_SetWorldState_Malformed(t *testing.T) {
	s := newTestService()
	_, err := s.DoCommand(context.Background(), map[string]interface{}{
		"set_world_state": "not an object",
	})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "set_world_state")
}

func TestDoCommand_GetCurrentTool_Empty(t *testing.T) {
	s := newTestService()
	res, err := s.DoCommand(context.Background(), map[string]interface{}{"get_current_tool": true})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res["tool"], test.ShouldBeNil)
}

func TestDoCommand_SetCurrentTool_Valid(t *testing.T) {
	s := newTestService()
	res, err := s.DoCommand(context.Background(), map[string]interface{}{"set_current_tool": "tongs"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res["success"], test.ShouldEqual, true)
	test.That(t, res["tool"], test.ShouldEqual, "tongs")

	res, err = s.DoCommand(context.Background(), map[string]interface{}{"get_current_tool": true})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res["tool"], test.ShouldEqual, "tongs")
}

func TestDoCommand_SetCurrentTool_Unknown(t *testing.T) {
	s := newTestService()
	_, err := s.DoCommand(context.Background(), map[string]interface{}{"set_current_tool": "spoon"})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "unknown tool")
}

func TestDoCommand_SetCurrentTool_Nil(t *testing.T) {
	s := newTestService()
	_, err := s.DoCommand(context.Background(), map[string]interface{}{"set_current_tool": "tongs"})
	test.That(t, err, test.ShouldBeNil)

	res, err := s.DoCommand(context.Background(), map[string]interface{}{"set_current_tool": nil})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res["tool"], test.ShouldBeNil)

	res, err = s.DoCommand(context.Background(), map[string]interface{}{"get_current_tool": true})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res["tool"], test.ShouldBeNil)
}

func TestDoCommand_SetCurrentTool_WrongType(t *testing.T) {
	s := newTestService()
	_, err := s.DoCommand(context.Background(), map[string]interface{}{"set_current_tool": 42})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "must be a string or null")
}
