build:
	CGO_ENABLED=0 go build -o bin/ts2phc-go
	CGO_ENABLED=0 go build -o bin/ubxcfg ./cmd/ubxcfg

build-bbb:
	GOOS=linux GOARCH=arm GOARM=7 CGO_ENABLED=0 go build -o bin/ts2phc-go-bbb
	GOOS=linux GOARCH=arm GOARM=7 CGO_ENABLED=0 go build -o bin/ubxcfg-bbb ./cmd/ubxcfg

# Deployable binaries live at the repo root and are what the fleet installs, so
# they must be rebuilt whenever main.go changes or boxes silently run old code.
build-amd64:
	GOOS=linux GOARCH=amd64 CGO_ENABLED=0 go build -o ts2phc-go-amd64

build-arm64:
	GOOS=linux GOARCH=arm64 CGO_ENABLED=0 go build -o ts2phc-go-arm64

build-armv7:
	GOOS=linux GOARCH=arm GOARM=7 CGO_ENABLED=0 go build -o ts2phc-go-armv7

build-all: build-amd64 build-arm64 build-armv7
