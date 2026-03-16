package main

import (
	"encoding/binary"
	"encoding/hex"
	"errors"
	"fmt"
	"log"
	"os"
	"strings"
	"time"

	"ts2phc-go/ubx"

	"go.bug.st/serial"
)

type receiverMode string

const (
	modeAuto receiverMode = "auto"
	modeM8N  receiverMode = "m8n"
	modeM9N  receiverMode = "m9n"
)

var (
	m8nProbeBauds = []int{115200, 9600, 38400, 57600, 19200, 4800}
	// CFG-PRT: set UART1 8N1 115200 with UBX+NMEA in/out.
	m8nCfgPrt115200 = mustHex("b5620600140001000000c008000000c201000700030000000000b07e")
	// CFG-GNSS profile with Galileo enabled and QZSS disabled.
	m8nCfgGnssGalileoOnQzssOff = mustHex("b562063e3c000020200700081000010001010101030001000101020408000100010103081000000001010400080000000103050003000000010506080e00010001015547")
)

type m8nNMEAToggle struct {
	label string
	msgID uint8
	rate  uint8
}

var m8nNMEAToggles = []m8nNMEAToggle{
	{label: "CFG-MSG GGA on", msgID: 0x00, rate: 1},
	{label: "CFG-MSG RMC on", msgID: 0x04, rate: 1},
	{label: "CFG-MSG ZDA on", msgID: 0x08, rate: 1},
	{label: "CFG-MSG GLL off", msgID: 0x01, rate: 0},
	{label: "CFG-MSG GSA off", msgID: 0x02, rate: 0},
	{label: "CFG-MSG GSV off", msgID: 0x03, rate: 0},
	{label: "CFG-MSG VTG off", msgID: 0x05, rate: 0},
}

func main() {
	log.SetFlags(log.Ltime | log.Lmicroseconds)

	dev := "/dev/ttyACM0"
	baud := 115200
	mode := modeAuto
	dynModel := uint8(2) // stationary
	antCableDelayNs := int16(38)

	// Simple arg parsing
	args := os.Args[1:]
	for i := 0; i < len(args); i++ {
		switch args[i] {
		case "--dev", "-d":
			i++
			dev = args[i]
		case "--baud", "-b":
			i++
			fmt.Sscanf(args[i], "%d", &baud)
		case "--dynmodel":
			i++
			fmt.Sscanf(args[i], "%d", &dynModel)
		case "--mode":
			i++
			mode = receiverMode(strings.ToLower(args[i]))
		case "--ant-cable-delay-ns":
			i++
			fmt.Sscanf(args[i], "%d", &antCableDelayNs)
		case "--help", "-h":
			fmt.Fprintf(os.Stderr, `ubxcfg - configure u-blox GNSS receiver (M8N/M9N)

Usage: ubxcfg [flags]

Flags:
  --dev DEV                  serial device (default /dev/ttyACM0)
  --baud RATE                baud rate (default 115200, also first probe for M8N)
  --mode MODE                receiver mode: auto|m8n|m9n (default auto)
  --dynmodel MODEL           dynamic model: 0=portable, 2=stationary (default 2)
  --ant-cable-delay-ns NS    antenna cable delay in ns (M8N+M9N, default 38)
`)
			os.Exit(0)
		default:
			fmt.Fprintf(os.Stderr, "unknown flag: %s\n", args[i])
			os.Exit(1)
		}
	}
	if mode != modeAuto && mode != modeM8N && mode != modeM9N {
		log.Fatalf("invalid --mode %q (expected auto|m8n|m9n)", mode)
	}
	if mode == modeM8N {
		if err := configureM8NDevice(dev, baud, dynModel, antCableDelayNs); err != nil {
			log.Fatal(err)
		}
		return
	}

	port, err := serial.Open(dev, &serial.Mode{BaudRate: baud})
	if err != nil {
		log.Fatalf("serial open %s: %v", dev, err)
	}
	defer port.Close()
	port.SetReadTimeout(300 * time.Millisecond)
	log.Printf("opened %s @ %d baud", dev, baud)

	effectiveMode := mode
	if effectiveMode == modeAuto {
		detected, mon, err := detectReceiverMode(port)
		if err != nil {
			log.Fatalf("auto detect failed: %v (set --mode m8n or --mode m9n)", err)
		}
		effectiveMode = detected
		log.Printf("detected receiver mode=%s sw=%q hw=%q", effectiveMode, mon.SwVersion, mon.HwVersion)
	}

	switch effectiveMode {
	case modeM9N:
		if err := configureM9N(port, dynModel, antCableDelayNs); err != nil {
			log.Fatal(err)
		}
	case modeM8N:
		_ = port.Close()
		if err := configureM8NDevice(dev, baud, dynModel, antCableDelayNs); err != nil {
			log.Fatal(err)
		}
	default:
		log.Fatalf("unsupported mode %q", effectiveMode)
	}
}

func configureM9N(port serial.Port, dynModel uint8, antCableDelayNs int16) error {
	frame := ubx.EncodeValset(ubx.LayerRAM|ubx.LayerBBR|ubx.LayerFlash,
		ubx.CfgU1(ubx.CfgNavspgDynModel, dynModel),
		ubx.CfgI2(ubx.CfgTpAntCableDelay, antCableDelayNs),
		ubx.CfgU1(ubx.CfgMsgoutUbxNavPvtUSB, 1),
		ubx.CfgU1(ubx.CfgMsgoutUbxNavDopUSB, 1),
		ubx.CfgU1(ubx.CfgMsgoutUbxNavTimeUSB, 1),
		ubx.CfgU1(ubx.CfgMsgoutUbxNavClkUSB, 1),
		ubx.CfgU1(ubx.CfgMsgoutUbxNavSatUSB, 5),
		ubx.CfgU1(ubx.CfgMsgoutUbxTimTpUSB, 1),
		ubx.CfgU1(ubx.CfgMsgoutUbxNavSigUSB, 5),
		ubx.CfgL(ubx.CfgUSBOutprotUBX, true),
		ubx.CfgL(ubx.CfgUSBOutprotNMEA, true),
		ubx.CfgL(ubx.CfgUSBInprotUBX, true),
		ubx.CfgL(ubx.CfgUSBInprotNMEA, true),
	)

	if err := sendAndAwaitAck(port, frame, ubx.ClassCFG, ubx.IDCfgValset, "VALSET"); err != nil {
		return err
	}
	log.Printf("sent M9N VALSET (RAM+BBR+FLASH): dynModel=%d antCableDelay=%dns", dynModel, antCableDelayNs)
	return nil
}

func configureM8N(port serial.Port, dynModel uint8, antCableDelayNs int16) error {
	if err := sendAndAwaitAck(port, m8nCfgGnssGalileoOnQzssOff, ubx.ClassCFG, ubx.IDCfgGnss, "CFG-GNSS"); err != nil {
		return err
	}
	if err := applyM8NNMEAToggles(port); err != nil {
		return err
	}
	if err := applyM8NAntCableDelay(port, antCableDelayNs); err != nil {
		return err
	}

	nav5 := ubx.EncodeCfgNav5DynModel(dynModel)
	if err := sendAndAwaitAck(port, nav5, ubx.ClassCFG, ubx.IDCfgNav5, "CFG-NAV5"); err != nil {
		return err
	}

	save := ubx.EncodeCfgCfgSave(ubx.CfgCfgMaskAll, ubx.CfgCfgDevBBR|ubx.CfgCfgDevFlash)
	if err := sendAndAwaitAck(port, save, ubx.ClassCFG, ubx.IDCfgCfg, "CFG-CFG(save)"); err != nil {
		return err
	}
	log.Printf("sent M8N CFG-GNSS + CFG-MSG + CFG-TP5 + CFG-NAV5 + CFG-CFG(save): dynModel=%d antCableDelay=%dns", dynModel, antCableDelayNs)
	return nil
}

func configureM8NDevice(dev string, baud int, dynModel uint8, antCableDelayNs int16) error {
	probeBauds := prependUnique(baud, m8nProbeBauds)
	currentBaud, err := probeBaud(dev, probeBauds)
	if err != nil {
		return fmt.Errorf("probe m8n baud: %w", err)
	}
	log.Printf("M8N responding at %d baud", currentBaud)

	if currentBaud != 115200 {
		port, err := serial.Open(dev, &serial.Mode{BaudRate: currentBaud})
		if err != nil {
			return fmt.Errorf("open %s @ %d: %w", dev, currentBaud, err)
		}
		port.SetReadTimeout(300 * time.Millisecond)
		log.Printf("upgrading M8N baud %d -> 115200", currentBaud)
		if _, err := port.Write(m8nCfgPrt115200); err != nil {
			_ = port.Close()
			return fmt.Errorf("write CFG-PRT: %w", err)
		}
		_ = port.Close()
		time.Sleep(300 * time.Millisecond)
	}

	port, err := serial.Open(dev, &serial.Mode{BaudRate: 115200})
	if err != nil {
		return fmt.Errorf("open %s @ 115200: %w", dev, err)
	}
	defer port.Close()
	port.SetReadTimeout(300 * time.Millisecond)
	log.Printf("opened %s @ 115200 baud", dev)

	return configureM8N(port, dynModel, antCableDelayNs)
}

func detectReceiverMode(port serial.Port) (receiverMode, *ubx.MonVer, error) {
	if _, err := port.Write(ubx.EncodePoll(ubx.ClassMON, ubx.IDMonVer)); err != nil {
		return "", nil, fmt.Errorf("write MON-VER poll: %w", err)
	}

	frame, err := readFrameByClassID(port, ubx.MsgMonVer, 2*time.Second)
	if err != nil {
		return "", nil, err
	}
	mon, err := ubx.ParseMonVer(frame.Payload)
	if err != nil {
		return "", nil, fmt.Errorf("parse MON-VER: %w", err)
	}

	s := strings.ToUpper(mon.SwVersion + " " + mon.HwVersion + " " + strings.Join(mon.Extensions, " "))
	switch {
	case strings.Contains(s, "M9"):
		return modeM9N, mon, nil
	case strings.Contains(s, "M8"):
		return modeM8N, mon, nil
	default:
		return "", nil, fmt.Errorf("unable to infer receiver from MON-VER: sw=%q hw=%q ext=%q", mon.SwVersion, mon.HwVersion, strings.Join(mon.Extensions, ","))
	}
}

func sendAndAwaitAck(port serial.Port, msg []byte, cls uint8, id uint8, label string) error {
	if _, err := port.Write(msg); err != nil {
		return fmt.Errorf("write %s: %w", label, err)
	}
	if err := awaitAck(port, cls, id, 2*time.Second); err != nil {
		return fmt.Errorf("%s: %w", label, err)
	}
	return nil
}

func awaitAck(port serial.Port, cls uint8, id uint8, timeout time.Duration) error {
	deadline := time.Now().Add(timeout)
	buf := make([]byte, 0, 1024)
	tmp := make([]byte, 256)

	for time.Now().Before(deadline) {
		n, err := port.Read(tmp)
		if err != nil {
			return err
		}
		if n == 0 {
			continue
		}
		buf = append(buf, tmp[:n]...)
		frames, rem := parseFrames(buf)
		buf = rem
		for _, f := range frames {
			if f.Class != ubx.ClassACK {
				continue
			}
			ack, err := ubx.ParseAck(f.Payload)
			if err != nil {
				continue
			}
			if ack.ClsID != cls || ack.MsgID != id {
				continue
			}
			if f.ID == ubx.IDAckAck {
				log.Printf("ACK %02x/%02x", cls, id)
				return nil
			}
			if f.ID == ubx.IDAckNak {
				return fmt.Errorf("NAK %02x/%02x", cls, id)
			}
		}
	}
	return errors.New("ACK timeout")
}

func readFrameByClassID(port serial.Port, classID uint16, timeout time.Duration) (ubx.Frame, error) {
	deadline := time.Now().Add(timeout)
	buf := make([]byte, 0, 1024)
	tmp := make([]byte, 256)

	for time.Now().Before(deadline) {
		n, err := port.Read(tmp)
		if err != nil {
			return ubx.Frame{}, err
		}
		if n == 0 {
			continue
		}
		buf = append(buf, tmp[:n]...)
		frames, rem := parseFrames(buf)
		buf = rem
		for _, f := range frames {
			if f.ClassID() == classID {
				return f, nil
			}
		}
	}
	return ubx.Frame{}, errors.New("frame timeout")
}

func parseFrames(data []byte) ([]ubx.Frame, []byte) {
	frames := make([]ubx.Frame, 0)
	i := 0
	for i+ubx.HeaderLen+ubx.ChecksumLen <= len(data) {
		if data[i] != ubx.SyncA || data[i+1] != ubx.SyncB {
			i++
			continue
		}
		length := int(binary.LittleEndian.Uint16(data[i+4 : i+6]))
		total := ubx.HeaderLen + length + ubx.ChecksumLen
		if i+total > len(data) {
			break
		}
		f, err := ubx.Decode(data[i : i+total])
		if err != nil {
			i++
			continue
		}
		frames = append(frames, f)
		i += total
	}
	if i >= len(data) {
		return frames, nil
	}
	return frames, data[i:]
}

func probeBaud(dev string, bauds []int) (int, error) {
	for _, baud := range bauds {
		port, err := serial.Open(dev, &serial.Mode{BaudRate: baud})
		if err != nil {
			continue
		}
		port.SetReadTimeout(300 * time.Millisecond)
		_, _ = port.Write(ubx.EncodePoll(ubx.ClassMON, ubx.IDMonVer))
		_, err = readFrameByClassID(port, ubx.MsgMonVer, 1200*time.Millisecond)
		_ = port.Close()
		if err == nil {
			return baud, nil
		}
	}
	return 0, errors.New("no MON-VER response on probed bauds")
}

func prependUnique(first int, rest []int) []int {
	out := make([]int, 0, len(rest)+1)
	out = append(out, first)
	for _, v := range rest {
		if v == first {
			continue
		}
		out = append(out, v)
	}
	return out
}

func mustHex(s string) []byte {
	b, err := hex.DecodeString(s)
	if err != nil {
		panic(err)
	}
	return b
}

func applyM8NNMEAToggles(port serial.Port) error {
	for _, t := range m8nNMEAToggles {
		msg := encodeM8NCfgMsgRate(0xf0, t.msgID, t.rate)
		if err := sendAndAwaitAck(port, msg, ubx.ClassCFG, ubx.IDCfgMsg, t.label); err != nil {
			return err
		}
	}
	return nil
}

func encodeM8NCfgMsgRate(msgClass, msgID, rate uint8) []byte {
	// CFG-MSG payload: msgClass,msgID, rateI2C,rateUART1,rateUART2,rateUSB,rateSPI,reserved
	payload := []byte{msgClass, msgID, rate, rate, rate, rate, rate, 0x00}
	return ubx.Encode(ubx.ClassCFG, ubx.IDCfgMsg, payload)
}

func applyM8NAntCableDelay(port serial.Port, antCableDelayNs int16) error {
	if _, err := port.Write(ubx.Encode(ubx.ClassCFG, ubx.IDCfgTp5, []byte{0x00})); err != nil {
		return fmt.Errorf("write CFG-TP5 poll: %w", err)
	}
	frame, err := readFrameByClassID(port, uint16(ubx.ClassCFG)<<8|uint16(ubx.IDCfgTp5), 2*time.Second)
	if err != nil {
		return fmt.Errorf("read CFG-TP5 poll response: %w", err)
	}
	if len(frame.Payload) < 32 {
		return fmt.Errorf("CFG-TP5 payload too short: %d", len(frame.Payload))
	}
	payload := make([]byte, len(frame.Payload))
	copy(payload, frame.Payload)
	binary.LittleEndian.PutUint16(payload[4:6], uint16(antCableDelayNs))
	cfg := ubx.Encode(ubx.ClassCFG, ubx.IDCfgTp5, payload)
	if err := sendAndAwaitAck(port, cfg, ubx.ClassCFG, ubx.IDCfgTp5, "CFG-TP5"); err != nil {
		return err
	}
	return nil
}
