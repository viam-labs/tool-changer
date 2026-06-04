package main

import (
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"

	toolchanger "github.com/viam-labs/tool-changer"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: genericservice.API, Model: toolchanger.DefaultModel},
	)
}
