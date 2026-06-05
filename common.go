package toolchanger

import "go.viam.com/rdk/resource"

var (
	NamespaceFamily = resource.ModelNamespace("viam").WithFamily("tool-changer")
	DefaultModel    = NamespaceFamily.WithModel("default")
)
