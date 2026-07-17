package toolchanger

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"time"

	"github.com/erh/vmodutils/file_utils"
	"go.viam.com/rdk/motionplan/armplanning"
)

func newPlanRequestForSaving(req *armplanning.PlanRequest) *armplanning.PlanRequest {
	if req == nil {
		return nil
	}
	return &armplanning.PlanRequest{
		FrameSystem:    req.FrameSystem,
		WorldState:     req.WorldState,
		Goals:          req.Goals,
		StartState:     req.StartState,
		Constraints:    req.Constraints,
		PlannerOptions: req.PlannerOptions,
	}
}

type saveMetadata struct {
	Command           string        `json:"command"`
	From              string        `json:"from,omitempty"`
	To                string        `json:"to,omitempty"`
	SavedAt           time.Time     `json:"saved_at"`
	TotalPlanningTime time.Duration `json:"total_planning_time"`
	Error             string        `json:"error,omitempty"`
	StepCount         int           `json:"step_count"`
}

// savePlanRequests writes the metadata and per-step PlanRequest JSON files
// for a single motion command. plan may be nil when planning failed before
// any step was completed; in that case only the metadata file is written.
// Errors are logged but not propagated — failing to save should never break
// the caller's command.
func (s *toolChanger) savePlanRequests(plan *Plan, cmd, from, to string, opErr error) {
	now := time.Now().UTC()
	subdir := fmt.Sprintf("tool-changer-%d-%s", now.Unix(), cmd)
	dir := file_utils.GetPathInCaptureDir(subdir)
	if err := os.MkdirAll(dir, 0o755); err != nil {
		s.logger.Warnf("save-plan-requests: mkdir %s: %v", dir, err)
		return
	}

	meta := saveMetadata{
		Command: cmd,
		From:    from,
		To:      to,
		SavedAt: now,
	}
	if opErr != nil {
		meta.Error = opErr.Error()
	}
	if plan != nil {
		meta.TotalPlanningTime = plan.TotalPlanningTime
		meta.StepCount = len(plan.Steps)
	}

	metaBytes, err := json.MarshalIndent(meta, "", "  ")
	if err != nil {
		s.logger.Warnf("save-plan-requests: marshal metadata: %v", err)
		return
	}
	if err := os.WriteFile(filepath.Join(dir, "metadata.json"), metaBytes, 0o644); err != nil {
		s.logger.Warnf("save-plan-requests: write metadata: %v", err)
		return
	}

	if plan == nil {
		return
	}
	for i, step := range plan.Steps {
		if step.Request == nil {
			continue
		}
		filename := fmt.Sprintf("%02d_%s.json", i, stepLabel(step))
		stripped := newPlanRequestForSaving(step.Request)
		if err := stripped.WriteToFile(filepath.Join(dir, filename)); err != nil {
			s.logger.Warnf("save-plan-requests: write %s: %v", filename, err)
		}
	}
}
