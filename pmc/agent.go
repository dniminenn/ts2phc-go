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
	MID_PORT_PROPERTIES_NP       = 0xC004
	MID_PORT_HWCLOCK_NP          = 0xC007
	MID_SUBSCRIBE_EVENTS_NP      = 0xC003
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
}

func NewAgent(udsPath string) (*Agent, error) {
	localAddr := &net.UnixAddr{Name: fmt.Sprintf("/var/run/ts2phc-go.%d", os.Getpid()), Net: "unixgram"}
	remoteAddr := &net.UnixAddr{Name: udsPath, Net: "unixgram"}

	os.Remove(localAddr.Name)

	conn, err := net.DialUnix("unixgram", localAddr, remoteAddr)
	if err != nil {
		return nil, fmt.Errorf("failed to connect to ptp4l: %w", err)
	}

	return &Agent{
		conn:    conn,
		udsPath: udsPath,
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
	msg.Header.VersionPTP = 2
	msg.TargetPortIdentity.ClockIdentity = [8]byte{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}
	msg.TargetPortIdentity.PortNumber = 0xffff
	msg.StartingBoundaryHops = 0
	msg.BoundaryHops = 0
	msg.ActionField = action

	tlv := ManagementTLV{
		Type:   1, // TLV_MANAGEMENT
		Length: uint16(2 + len(data)),
		Id:     mgtId,
	}

	// Calculate lengths
	baseLen := 48 // Size of MsgHeader + Management fields
	msg.Header.MessageLength = uint16(baseLen + 4 + len(data))

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
	a.conn.SetReadDeadline(time.Now().Add(1 * time.Second))
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
