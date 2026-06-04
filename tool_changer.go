package toolchanger

import (
	"context"
	"fmt"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"
)

func init() {
	resource.RegisterService(genericservice.API, DefaultModel,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newToolChanger,
		},
	)
}

type Config struct{}

func (c *Config) Validate(path string) ([]string, []string, error) {
	return nil, nil, nil
}

type toolChanger struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
}

func newToolChanger(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	return &toolChanger{
		name:   rawConf.ResourceName(),
		logger: logger,
	}, nil
}

func (s *toolChanger) Name() resource.Name {
	return s.name
}

func (s *toolChanger) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (s *toolChanger) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, fmt.Errorf("not implemented")
}

func (s *toolChanger) Close(ctx context.Context) error {
	return nil
}
