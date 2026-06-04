GO_BUILD_ENV :=
GO_BUILD_FLAGS :=
MODULE_BINARY := bin/tool-changer

ifeq ($(VIAM_TARGET_OS), windows)
	GO_BUILD_ENV += GOOS=windows GOARCH=amd64
	GO_BUILD_FLAGS := -tags no_cgo
	MODULE_BINARY = bin/tool-changer.exe
endif

$(MODULE_BINARY): Makefile go.mod *.go cmd/module/*.go
	GOOS=$(VIAM_BUILD_OS) GOARCH=$(VIAM_BUILD_ARCH) $(GO_BUILD_ENV) go build $(GO_BUILD_FLAGS) -o $(MODULE_BINARY) ./cmd/module

module.tar.gz: $(MODULE_BINARY) meta.json
	tar czf module.tar.gz $(MODULE_BINARY) meta.json

lint:
	gofmt -s -w .

test:
	go test ./...

update:
	go get go.viam.com/rdk@latest
	go mod tidy

setup:
	go mod tidy

.PHONY: lint test update setup
