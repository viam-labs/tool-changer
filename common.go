package toolchanger

import "go.viam.com/rdk/resource"

var (
	NamespaceFamily = resource.ModelNamespace("viam-labs").WithFamily("tool-changer")
	DefaultModel    = NamespaceFamily.WithModel("default")
)
