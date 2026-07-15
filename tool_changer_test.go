package toolchanger

import (
	"context"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/motionplan"
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
				SlideOffsetMM: r3.Vector{X: 100, Y: 0, Z: 0},
				LiftOffsetMM:  r3.Vector{X: 0, Y: 0, Z: 80},
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
			name:    "tool with zero slide-offset",
			mutate:  func(c *Config) { c.Tools[0].SlideOffsetMM = r3.Vector{} },
			wantErr: "slide-offset-mm is required",
		},
		{
			name:    "tool with zero lift-offset",
			mutate:  func(c *Config) { c.Tools[0].LiftOffsetMM = r3.Vector{} },
			wantErr: "lift-offset-mm is required",
		},
		{
			name:    "negative slide-speed velocity",
			mutate:  func(c *Config) { c.SlideSpeed = &SpeedConfig{MaxVelDegsPerSec: -1} },
			wantErr: "max_vel_degs_per_sec",
		},
		{
			name:    "negative slide-speed acceleration",
			mutate:  func(c *Config) { c.SlideSpeed = &SpeedConfig{MaxAccDegsPerSec2: -1} },
			wantErr: "max_acc_degs_per_sec2",
		},
		{
			name: "valid slide-speed config",
			mutate: func(c *Config) {
				c.SlideSpeed = &SpeedConfig{MaxVelDegsPerSec: 30, MaxAccDegsPerSec2: 60}
			},
			wantDeps: []string{"left-arm", framesystem.PublicServiceName.String()},
		},
		{
			name: "tool with gripper",
			mutate: func(c *Config) {
				c.Tools[0].Gripper = "tongs-gripper"
			},
			wantDeps: []string{"left-arm", framesystem.PublicServiceName.String(), "tongs-gripper"},
		},
		{
			name: "two tools with different grippers",
			mutate: func(c *Config) {
				c.Tools[0].Gripper = "tongs-gripper"
				c.Tools = append(c.Tools, ToolConfig{
					Name: "spoon",
					SlotPose: Pose{
						Point:       r3.Vector{X: 500, Y: -300, Z: 120},
						Orientation: validOrientation(),
					},
					SlideOffsetMM: r3.Vector{X: 100, Y: 0, Z: 0},
					LiftOffsetMM:  r3.Vector{X: 0, Y: 0, Z: 80},
					Gripper:       "spoon-gripper",
				})
			},
			wantDeps: []string{"left-arm", framesystem.PublicServiceName.String(), "tongs-gripper", "spoon-gripper"},
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

func TestDoCommand_SwitchTool_WrongType(t *testing.T) {
	s := newTestService()
	_, err := s.DoCommand(context.Background(), map[string]interface{}{"switch_tool": 42})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "must be a string")
}

func TestDoCommand_SwitchTool_Unknown(t *testing.T) {
	s := newTestService()
	_, err := s.DoCommand(context.Background(), map[string]interface{}{"switch_tool": "spoon"})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "unknown tool")
}

func TestDoCommand_SwitchTool_AlreadyAttached(t *testing.T) {
	s := newTestService()
	tongs := "tongs"
	s.currentTool = &tongs

	res, err := s.DoCommand(context.Background(), map[string]interface{}{"switch_tool": "tongs"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res["changed"], test.ShouldEqual, false)
	test.That(t, res["from"], test.ShouldEqual, "tongs")
	test.That(t, res["to"], test.ShouldEqual, "tongs")
	test.That(t, s.currentTool, test.ShouldNotBeNil)
	test.That(t, *s.currentTool, test.ShouldEqual, "tongs")
}

func TestDoCommand_Release_Empty(t *testing.T) {
	s := newTestService()
	res, err := s.DoCommand(context.Background(), map[string]interface{}{"release": true})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res["released"], test.ShouldBeNil)
	test.That(t, s.currentTool, test.ShouldBeNil)
}

func TestDoCommand_GetStatus_Empty(t *testing.T) {
	s := newTestService()
	res, err := s.DoCommand(context.Background(), map[string]interface{}{"get_status": true})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res["success"], test.ShouldEqual, true)
	test.That(t, res["attached"], test.ShouldEqual, false)
	test.That(t, res["current_tool"], test.ShouldBeNil)
	test.That(t, res["world_state_set"], test.ShouldEqual, false)
}

func TestDoCommand_GetStatus_Attached(t *testing.T) {
	s := newTestService()
	tongs := "tongs"
	s.currentTool = &tongs

	res, err := s.DoCommand(context.Background(), map[string]interface{}{"get_status": true})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res["attached"], test.ShouldEqual, true)
	test.That(t, res["current_tool"], test.ShouldEqual, "tongs")
}

func TestMergeSlideConstraints_Nil(t *testing.T) {
	test.That(t, mergeSlideConstraints(nil, nil), test.ShouldBeNil)
}

func TestMergeSlideConstraints_OnlyAllowed(t *testing.T) {
	allowed := []motionplan.CollisionSpecificationAllowedFrameCollisions{
		{Frame1: "gripper:claws", Frame2: "tongs:body"},
	}
	got := mergeSlideConstraints(nil, allowed)
	test.That(t, got, test.ShouldNotBeNil)
	test.That(t, len(got.CollisionSpecification), test.ShouldEqual, 1)
	test.That(t, got.CollisionSpecification[0].Allows, test.ShouldResemble, allowed)
}

func TestMergeSlideConstraints_BaseAndAllowed(t *testing.T) {
	base := &motionplan.Constraints{
		LinearConstraint: []motionplan.LinearConstraint{{LineToleranceMm: 1.0}},
		CollisionSpecification: []motionplan.CollisionSpecification{
			{Allows: []motionplan.CollisionSpecificationAllowedFrameCollisions{
				{Frame1: "a", Frame2: "b"},
			}},
		},
	}
	allowed := []motionplan.CollisionSpecificationAllowedFrameCollisions{
		{Frame1: "gripper:claws", Frame2: "tongs:body"},
	}
	got := mergeSlideConstraints(base, allowed)
	test.That(t, got, test.ShouldNotBeNil)
	test.That(t, len(got.LinearConstraint), test.ShouldEqual, 1)
	test.That(t, len(got.CollisionSpecification), test.ShouldEqual, 2)
	// Base's existing CollisionSpecification stays at index 0; new pairs appended.
	test.That(t, got.CollisionSpecification[0].Allows[0].Frame1, test.ShouldEqual, "a")
	test.That(t, got.CollisionSpecification[1].Allows[0].Frame2, test.ShouldEqual, "tongs:body")

	// Base must not have been mutated.
	test.That(t, len(base.CollisionSpecification), test.ShouldEqual, 1)
}
