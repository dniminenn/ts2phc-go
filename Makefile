build:
	CGO_ENABLED=0 go build -o bin/ts2phc-go
	CGO_ENABLED=0 go build -o bin/ubxcfg ./cmd/ubxcfg

build-bbb:
	GOOS=linux GOARCH=arm GOARM=7 CGO_ENABLED=0 go build -o bin/ts2phc-go-bbb
	GOOS=linux GOARCH=arm GOARM=7 CGO_ENABLED=0 go build -o bin/ubxcfg-bbb ./cmd/ubxcfg
