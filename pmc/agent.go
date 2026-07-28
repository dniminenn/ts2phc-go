package pmc

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"net"
	"os"
	"time"
)

// PTP Management IDs
const (
	MID_DEFAULT_DATA_SET         = 0x2000
	MID_PORT_DATA_SET            = 0x2004
	MID_TIME_PROPERTIES_DATA_SET = 0x2003
	MID_GRANDMASTER_SETTINGS_NP  = 0xC001
	MID_PORT_PROPERTIES_NP       = 0xC004
	MID_PORT_HWCLOCK_NP          = 0xC007
	MID_SUBSCRIBE_EVENTS_NP      = 0xC003
)

// timePropertiesDS flags (IEEE 1588-2019 8.2.4)
const (
	TF_LEAP_61       = 1 << 0
	TF_LEAP_59       = 1 << 1
	TF_UTC_OFF_VALID = 1 << 2
	TF_PTP_TIMESCALE = 1 << 3
	TF_TIME_TRACEABLE = 1 << 4
	TF_FREQ_TRACEABLE = 1 << 5
)

// timeSource values (IEEE 1588-2019 7.6.2.8)
const (
	TS_GNSS                = 0x20
	TS_INTERNAL_OSCILLATOR = 0xA0
)

// PTP Message Types
const (
	MANAGEMENT = 0xD
)

// Management Actions
const (
	GET       = 0x0
	SET       = 0x1
	RESPONSE  = 0x2
	COMMAND   = 0x3
	ACK       = 0x4
)

// Port States
const (
	PS_INITIALIZING = 1
	PS_FAULTY       = 2
	PS_DISABLED     = 3
	PS_LISTENING    = 4
	PS_PRE_MASTER   = 5
	PS_MASTER       = 6
	PS_PASSIVE      = 7
	PS_UNCALIBRATED = 8
	PS_SLAVE        = 9
)

type ClockIdentity [8]byte
type PortIdentity struct {
	ClockIdentity ClockIdentity
	PortNumber    uint16
}

type MsgHeader struct {
	SdoIdAndMsgType uint8
	VersionPTP      uint8
	MessageLength   uint16
	DomainNumber    uint8
	MinorSdoId      uint8
	Flags           [2]uint8
	CorrectionField int64
	MsgTypeSpec     [4]uint8
	SourcePortIdentity PortIdentity
	SequenceId      uint16
	ControlField    uint8
	LogMessageInterval int8
}

type ManagementMsg struct {
	Header         MsgHeader
	TargetPortIdentity PortIdentity
	StartingBoundaryHops uint8
	BoundaryHops   uint8
	ActionField    uint8
	Reserved       uint8
}

type ManagementTLV struct {
	Type   uint16
	Length uint16
	Id     uint16
	Data   []byte
}

type DefaultDS struct {
	TwoStepFlag   uint8
	ClockIdentity ClockIdentity
	NumberPorts   uint16
	ClockQuality  [4]byte
	Priority1     uint8
	Priority2     uint8
	ClockDomain   uint8
	SlaveOnly     uint8
}

type Agent struct {
	conn       *net.UnixConn
	udsPath    string
	syncOffset int
	leap       int
	dds        DefaultDS
	identity   PortIdentity
	seqId      uint16
}

func NewAgent(udsPath string) (*Agent, error) {
	localAddr := &net.UnixAddr{Name: fmt.Sprintf("/var/run/ts2phc-go.%d", os.Getpid()), Net: "unixgram"}
	remoteAddr := &net.UnixAddr{Name: udsPath, Net: "unixgram"}

	os.Remove(localAddr.Name)

	conn, err := net.DialUnix("unixgram", localAddr, remoteAddr)
	if err != nil {
		return nil, fmt.Errorf("failed to connect to ptp4l: %w", err)
	}

	// linuxptp's pmc sends a zero clockIdentity with its PID as the port
	// number; mirror that so ptp4l treats us identically.
	var ident PortIdentity
	ident.PortNumber = uint16(os.Getpid())

	return &Agent{
		conn:     conn,
		udsPath:  udsPath,
		identity: ident,
	}, nil
}

func (a *Agent) Close() {
	if a.conn != nil {
		a.conn.Close()
	}
	os.Remove(fmt.Sprintf("/var/run/ts2phc-go.%d", os.Getpid()))
}

// Low-level send request
func (a *Agent) sendRequest(mgtId uint16, action uint8, data []byte) error {
	var msg ManagementMsg
	msg.Header.SdoIdAndMsgType = MANAGEMENT
	msg.Header.VersionPTP = 0x12 // PTP v2.1, as linuxptp's pmc sends
	msg.Header.SourcePortIdentity = a.identity
	a.seqId++
	msg.Header.SequenceId = a.seqId
	msg.Header.LogMessageInterval = 0x7f
	msg.TargetPortIdentity.ClockIdentity = [8]byte{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}
	msg.TargetPortIdentity.PortNumber = 0xffff
	msg.ActionField = action

	tlv := ManagementTLV{
		Type:   1, // TLV_MANAGEMENT
		Length: uint16(2 + len(data)),
		Id:     mgtId,
	}

	// Calculate lengths: 48-byte header+management fields, then the TLV
	// (type 2 + length 2 + id 2 = 6 bytes) plus its data.
	baseLen := 48
	msg.Header.MessageLength = uint16(baseLen + 6 + len(data))

	buf := new(bytes.Buffer)
	binary.Write(buf, binary.BigEndian, msg.Header)
	binary.Write(buf, binary.BigEndian, msg.TargetPortIdentity)
	binary.Write(buf, binary.BigEndian, msg.StartingBoundaryHops)
	binary.Write(buf, binary.BigEndian, msg.BoundaryHops)
	binary.Write(buf, binary.BigEndian, msg.ActionField)
	binary.Write(buf, binary.BigEndian, msg.Reserved)
	binary.Write(buf, binary.BigEndian, tlv.Type)
	binary.Write(buf, binary.BigEndian, tlv.Length)
	binary.Write(buf, binary.BigEndian, tlv.Id)
	if len(data) > 0 {
		buf.Write(data)
	}

	_, err := a.conn.Write(buf.Bytes())
	return err
}

// Low-level receive response
func (a *Agent) recvResponse(expectedId uint16) ([]byte, error) {
	a.conn.SetReadDeadline(time.Now().Add(200 * time.Millisecond))
	buf := make([]byte, 1024)
	n, _, err := a.conn.ReadFromUnix(buf)
	if err != nil {
		return nil, err
	}

	reader := bytes.NewReader(buf[:n])
	var msg ManagementMsg
	if err := binary.Read(reader, binary.BigEndian, &msg.Header); err != nil {
		return nil, err
	}
	if msg.Header.SdoIdAndMsgType&0xf != MANAGEMENT {
		return nil, fmt.Errorf("not a management message")
	}

	binary.Read(reader, binary.BigEndian, &msg.TargetPortIdentity)
	binary.Read(reader, binary.BigEndian, &msg.StartingBoundaryHops)
	binary.Read(reader, binary.BigEndian, &msg.BoundaryHops)
	binary.Read(reader, binary.BigEndian, &msg.ActionField)
	binary.Read(reader, binary.BigEndian, &msg.Reserved)

	var tlv ManagementTLV
	binary.Read(reader, binary.BigEndian, &tlv.Type)
	binary.Read(reader, binary.BigEndian, &tlv.Length)
	binary.Read(reader, binary.BigEndian, &tlv.Id)

	if tlv.Id != expectedId {
		return nil, fmt.Errorf("unexpected management ID: %x", tlv.Id)
	}
	if msg.ActionField != RESPONSE {
		return nil, fmt.Errorf("unexpected action: %x", msg.ActionField)
	}

	data := make([]byte, tlv.Length-2)
	reader.Read(data)
	return data, nil
}

func (a *Agent) QueryDDS() error {
	if err := a.sendRequest(MID_DEFAULT_DATA_SET, GET, nil); err != nil {
		return err
	}
	data, err := a.recvResponse(MID_DEFAULT_DATA_SET)
	if err != nil {
		return err
	}

	reader := bytes.NewReader(data)
	binary.Read(reader, binary.BigEndian, &a.dds)
	return nil
}

func (a *Agent) GetNumberPorts() int {
	return int(a.dds.NumberPorts)
}

// GrandmasterSettings mirrors linuxptp's grandmaster_settings_np (8 bytes on
// the wire): clockQuality, currentUtcOffset, time flags, timeSource.
type GrandmasterSettings struct {
	ClockClass              uint8
	ClockAccuracy           uint8
	OffsetScaledLogVariance uint16
	UtcOffset               int16
	TimeFlags               uint8
	TimeSource              uint8
}

func (a *Agent) GetGrandmasterSettings() (*GrandmasterSettings, error) {
	// GET of this dataset must carry a zero payload sized to the dataset
	// (8 bytes), matching linuxptp's pmc; ptp4l drops shorter TLVs.
	if err := a.sendRequest(MID_GRANDMASTER_SETTINGS_NP, GET, make([]byte, 8)); err != nil {
		return nil, err
	}
	data, err := a.recvResponse(MID_GRANDMASTER_SETTINGS_NP)
	if err != nil {
		return nil, err
	}
	gs := &GrandmasterSettings{}
	if err := binary.Read(bytes.NewReader(data), binary.BigEndian, gs); err != nil {
		return nil, err
	}
	return gs, nil
}

// SetGrandmasterSettings pushes new grandmaster settings to ptp4l at runtime;
// ptp4l starts announcing them immediately. ptp4l acks a SET with a RESPONSE
// carrying the dataset.
func (a *Agent) SetGrandmasterSettings(gs *GrandmasterSettings) error {
	buf := new(bytes.Buffer)
	if err := binary.Write(buf, binary.BigEndian, gs); err != nil {
		return err
	}
	if err := a.sendRequest(MID_GRANDMASTER_SETTINGS_NP, SET, buf.Bytes()); err != nil {
		return err
	}
	_, err := a.recvResponse(MID_GRANDMASTER_SETTINGS_NP)
	return err
}

func (a *Agent) Subscribe() error {
	// Subscribe to NOTIFY_PORT_STATE
	sen := make([]byte, 4) 
	// Duration: UPDATES_PER_SUBSCRIPTION * node->update_interval (we'll just use a large number for prototype)
	sen[0] = 100 // Example
	// bitmask for NOTIFY_PORT_STATE (bit 1)
	sen[2] = 1 << 0 

	return a.sendRequest(MID_SUBSCRIBE_EVENTS_NP, SET, sen)
}

func (a *Agent) PollEvents() (uint16, uint16, error) {
	// Simple non-blocking poll
	a.conn.SetReadDeadline(time.Now().Add(10 * time.Millisecond))
	buf := make([]byte, 1024)
	n, _, err := a.conn.ReadFromUnix(buf)
	if err != nil {
		return 0, 0, err
	}

	reader := bytes.NewReader(buf[:n])
	var msg ManagementMsg
	binary.Read(reader, binary.BigEndian, &msg.Header)
	binary.Read(reader, binary.BigEndian, &msg.TargetPortIdentity)
	binary.Read(reader, binary.BigEndian, &msg.StartingBoundaryHops)
	binary.Read(reader, binary.BigEndian, &msg.BoundaryHops)
	binary.Read(reader, binary.BigEndian, &msg.ActionField)
	binary.Read(reader, binary.BigEndian, &msg.Reserved)

	var tlv ManagementTLV
	binary.Read(reader, binary.BigEndian, &tlv.Type)
	binary.Read(reader, binary.BigEndian, &tlv.Length)
	binary.Read(reader, binary.BigEndian, &tlv.Id)

	if tlv.Id == MID_PORT_DATA_SET {
		// Port state is byte at offset 33 of the management tlv data
		// Let's decode PortDS: 
		// portIdentity (10), portState (1), logMinDelayReqInterval (1)...
		data := make([]byte, tlv.Length-2)
		reader.Read(data)
		
		if len(data) >= 11 {
			portNumber := binary.BigEndian.Uint16(data[8:10])
			state := data[10]
			return portNumber, uint16(state), nil
		}
	}
	return 0, 0, fmt.Errorf("no port data set event")
}
