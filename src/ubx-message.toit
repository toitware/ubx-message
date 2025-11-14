// Copyright (C) 2025 Toit contributors.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

/**
Support for the UBX messages from the UBX data protocol.

The UBX data protocol is used by the ublox GNSS receivers in the Max-M*
  series. Some messages are deprecated between versions.

The description for each receiver describes the supported UBX message.
- Max-M8: https://www.u-blox.com/en/docs/UBX-13003221
- Max-M9: https://www.u-blox.com/en/docs/UBX-19035940

To do list:
- MGA-* (AssistNow) messages: Assisted GNSS injection (time, eph/almanac) for
  fast TTFF.  A path for MGA-INI-TIME_UTC at minimum.
- ESF-* for combination with DR/ADR/IMU fusion
*/

import io
//import semver
import io show LITTLE-ENDIAN
import reader as old-reader

/**
A UBX message from the UBX data protocol.
*/
class Message:
  /** Maximum size of an encoded message.

  The protocol allows length up to 65535 (2-byte field), though most real
  messages are far smaller. 2 KB is sensible safety cap, but some messages
  (e.g., MGA assistance blocks, large MON dumps) can exceed that on newer
  firmware.  May need to set this differently later, possibly 8K/16K.
  */
  static MAX-MESSAGE-SIZE_ ::= 2048


  /** The class of this message. */
  cls /int
  /** The ID of this message. */
  id /int
  /** The Payload of this message. */
  payload /ByteArray

  /** The Navigation result (NAV) class byte. */
  static NAV ::= 0x01
  /** The Receiver Manager (RXM) class byte. */
  static RXM ::= 0x02
  /** The Information (INF) class byte. */
  static INF ::= 0x04
  /** The ack/nak (ACK) class byte. */
  static ACK ::= 0x05
  /** The Configuration Input (CFG) class byte. */
  static CFG ::= 0x06
  /** The Firmware Update (UPD) class byte. */
  static UPD ::= 0x09
  /** The Monitoring (MON) class byte. */
  static MON ::= 0x0A
  /** The AssistNow Aiding (AID) class byte. */
  static AID ::= 0x0B
  /** The Time (TIM) class byte. */
  static TIM ::= 0x0D
  /** The External Sensor Fusion class byte. */
  static ESF ::= 0x10
  /** The Multiple GNSS Assistance (MGA) class byte.*/
  static MGA ::= 0x13
  /** The Logging (LOG) class byte. */
  static LOG ::= 0x21
  /** The Security Feature (SEC) class byte. */
  static SEC ::= 0x27
  /** The High-Rate Navigation Result (HNR) class byte. */
  static HNR ::= 0x28

  /**
  Map from class byte to its string representation.
  */
  static PACK-CLASSES ::= {
    NAV: "NAV",
    RXM: "RXM",
    INF: "INF",
    ACK: "ACK",
    CFG: "CFG",
    UPD: "UPD",
    MON: "MON",
    AID: "AID",
    TIM: "TIM",
    ESF: "ESF",
    MGA: "MGA",
    LOG: "LOG",
    SEC: "SEC",
    HNR: "HNR"}

  /**
  Map from Message byte/type to its string representation.

  Not all messages are handled in this driver, however all message ID's found in
    6M and M8 manuals have been added to help if an information message presents
    something that needs to be looked at.  Implemented as nested Maps.
  */
  static PACK-MESSAGE-TYPES := {
    // ACK (0x05)
    ACK: {
      0x00: "ACK-NAK",
      0x01: "ACK-ACK",
    },

    // CFG (0x06)
    CFG: {
      0x00: "PRT",
      0x01: "MSG",
      0x02: "INF",
      0x04: "RST",
      0x06: "DAT",
      0x07: "TP",
      0x08: "RATE",
      0x09: "CFG",
      0x0E: "FXN",       // 6-series
      0x11: "RXM",
      0x12: "EKF",       // 6-series LEA-6R
      0x13: "ANT",
      0x16: "SBAS",
      0x17: "NMEA",
      0x1B: "USB",
      0x1D: "TMODE",     // 6-series
      0x1E: "ODO",
      0x23: "NAVX5",
      0x24: "NAV5",
      0x29: "ESFGWT",    // 6-series (LEA-6R)
      0x31: "TP5",
      0x34: "RINV",
      0x39: "ITFM",
      0x3B: "PM2",
      0x3D: "TMODE2",
      0x3E: "GNSS",      // M8+
      0x47: "LOGFILTER", // M8+
      0x53: "TXSLOT",    // M8+
      0x56: "ESFALG",    // M8+
      0x57: "PWR",       // M8+
      0x5C: "HNR",       // M8+
      0x60: "ESRC",      // M8+
      0x61: "DOSC",      // M8+
      0x62: "SMGR",      // M8+
      0x64: "SPT",       // M8+
      0x69: "GEOFENCE",  // M8+
      0x70: "DGNSS",     // M8+
      0x71: "TMODE3",    // M8+
      0x82: "ESFWT",     // M8+
      0x86: "PMS",       // M8+
      0x88: "SENIF",     // M8+
      0x8D: "SLAS",      // M8+
      0x93: "BATCH",     // M8+
    },

    // MON (0x0A)
    MON: {
      0x02: "IO",
      0x04: "VER",
      0x06: "MSGPP",
      0x07: "RXBUF",
      0x08: "TXBUF",
      0x09: "HW",
      0x0B: "HW2",
      0x21: "RXR",
      0x27: "PATCH",     // M8+
      0x28: "GNSS",      // M8+
      0x2E: "SMGR",      // M8+
      0x2F: "SPT",       // M8+
      0x32: "BATCH",     // M8+
    },

    // NAV (0x01)
    NAV: {
      0x01: "POSECEF",
      0x02: "POSLLH",
      0x03: "STATUS",
      0x04: "DOP",
      0x05: "ATT",        // M8+
      0x06: "SOL",
      0x07: "PVT",        // M8+
      0x09: "ODO",        // M8+
      0x10: "RESETODO",   // M8+
      0x11: "VELECEF",
      0x12: "VELNED",
      0x13: "HPPOSECEF",  // M8+
      0x14: "HPPOSLLH",   // M8+
      0x20: "TIMEGPS",
      0x21: "TIMEUTC",
      0x23: "TIMEGLO",    // M8+
      0x24: "TIMEBDS",    // M8+
      0x25: "TIMEGAL",    // M8+
      0x26: "TIMELS",     // M8+
      0x28: "NMI",        // M8+
      0x30: "SVINFO",
      0x31: "DGPS",
      0x32: "SBAS",
      0x35: "SAT",        // M8+
      0x39: "GEOFENCE",   // M8+
      0x3B: "SVIN",       // M8+
      0x3C: "RELPOSNED",  // M8+
      0x3D: "EELL",       // M8+
      0x42: "SLAS",       // M8+
      0x60: "AOPSTATUS",
      0x61: "EOE",        // M8+
    },

    // RXM (0x02)
    RXM: {
      0x10: "RAW",     // 6-series
      0x11: "SFRB",    // 6-series
      0x13: "SFRBX",   // M8+
      0x14: "MEASX",   // M8+
      0x15: "RAWX",    // M8+
      0x20: "SVSI",
      0x30: "ALM",
      0x31: "EPH",
      0x32: "RTCM",    // M8+
      0x41: "PMREQ",
      0x59: "RLM",     // M8+
      0x61: "IMES",    // M8+
    },

    // TIM (0x0D)
    TIM: {
      0x01: "TP",
      0x03: "TM2",
      0x04: "SVIN",
      0x06: "VRFY",    // 6-series
      0x11: "DOSC",    // M8+
      0x12: "TOS",     // M8+
      0x13: "SMEAS",   // M8+
      0x15: "VCOCAL",  // M8+
      0x16: "FCHG",    // M8+
      0x17: "HOC",     // M8+
    },

    // SEC (0x27) — M8+
    SEC: {
      0x03: "SEC-UNIQID",
    },

    // AID (0x0B) — legacy assistance (6 & M8)
    AID: {
      0x01: "INI",
      0x02: "HUI",
      0x30: "ALM",
      0x31: "EPH",
      0x33: "AOP",
      0x50: "ALP",   // 6-series
      0x10: "DATA",  // 6-series
      0x32: "ALPSRV" // 6-series
    },

    // ESF (0x10) — external sensor fusion
    ESF: {
      0x02: "MEAS",  // 6-series LEA-6R / M8 ESF-MEAS (different payloads)
      0x03: "RAW",   // M8+
      0x10: "STATUS",// 6-series LEA-6R / M8 ESF-STATUS
      0x14: "ALG",   // M8+
      0x15: "INS",   // M8+
    },

    // HNR (0x28) — M8+
    HNR: {
      0x00: "PVT",
      0x01: "ATT",
      0x02: "INS",
    },

    // LOG (0x21) — M8+
    LOG: {
      0x03: "ERASE",
      0x07: "CREATE",
      0x0B: "INFO",
      0x0D: "RETRIEVE",
      0x0E: "RETRIEVEPOS",
      0x0F: "RETRIEVESTRING",
      0x10: "FINDTIME",
      0x11: "BATCH",
    },

    // MGA (0x13) — M8+ multi-GNSS assistance (index only; many subtypes)
    MGA: {
      0x00: "GPS",
      0x02: "GAL",
      0x03: "BDS",
      0x05: "QZSS",
      0x06: "GLO",
      0x20: "ANO",
      0x40: "INI",
      0x60: "ACK",
      0x80: "DBD",
    },
  }

  static INVALID-UBX-MESSAGE_ ::= "INVALID UBX MESSAGE"
  static RESERVED_ ::= 0

  /**
  Represents the minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.

  Todo: convert to semver for later ease of use...?
  */
  static MIN-PROTVER/string := "15.0"

  /**
  Represents the maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.

  Todo: convert to semver for later ease of use.
  */
  static MAX-PROTVER/string := ""


  /** Constructs a UBX message with the given $cls, $id, and $payload. */
  constructor.private_ .cls .id .payload:

  /**
  Constructs a UBX message with the given $cls, $id, and $payload.

  If message is implemented in this package, then it returns the appropriate
    sub-class.
  */
  constructor cls id payload:
    if cls == Message.ACK:
      if id == AckAck.ID:
        return AckAck.private_ payload
      else if id == AckNak.ID:
        return AckNak.private_ payload
      else:
        Message.private_ cls id payload
        //throw "Unexpected UBX-ACK-xxx packet payload type."

    if cls == Message.NAV:
      if id == NavPvt.ID:
        return NavPvt.private_ payload
      else if id == NavStatus.ID:
        return NavStatus.private_ payload
      else if id == NavSat.ID:
        return NavSat.private_ payload
      else if id == NavPosLlh.ID:
        return NavPosLlh.private_ payload
      else if id == NavSvInfo.ID:
        return NavSvInfo.private_ payload
      else if id == NavSol.ID:
        return NavSol.private_ payload
      else if id == NavTimeUtc.ID:
        return NavTimeUtc.private_ payload

    if cls == Message.MON:
      if id == MonVer.ID:
        return MonVer.private_ payload

    if cls == Message.CFG:
      if id == CfgPrt.ID:
        return CfgPrt.private_ payload
      if id == CfgTp5.ID:
        return CfgTp5.private_ payload
      if id == CfgNav5.ID:
        return CfgNav5.private_ payload
      if id == CfgGnss.ID:
        return CfgGnss.private_ payload

    return Message.private_ cls id payload

  /**
  Constructs a UBX message from the given $bytes.

  The $bytes must be a valid UBX message (contain the sync bytes and a valid
    checksum).  A message created, parsed using .from-bytes and converted back
    to a byte array with .to-byte-array should end up identical.
  */
  constructor.from-bytes bytes/ByteArray:
    if not is-valid-frame_ bytes: throw INVALID-UBX-MESSAGE_
    cls = bytes[2]
    id = bytes[3]
    length := LITTLE-ENDIAN.uint16 bytes 4
    if bytes.size != length + 8: throw INVALID-UBX-MESSAGE_
    //payload = bytes[4 .. bytes.size - 2] - this did not pass identical test
    payload = bytes[6 .. 6 + length]

  /**
  Constructs a UBX message from the given $reader.

  The $reader must be able to provide a valid UBX frame.

  If message is implemented in this package, then it returns the appropriate
    sub-class.

  The $reader should be an $io.Reader, but an $old-reader.Reader is also accepted
    for backwards compatibility. The use of $old-reader.Reader is deprecated and
    will be removed in a future release.
  */
  constructor.from-reader reader/old-reader.Reader:
    io-reader/io.Reader := reader is io.Reader ? reader as io.Reader : io.Reader.adapt reader

    if (io-reader.peek-byte 0) != 0xb5 or (io-reader.peek-byte 1) != 0x62: throw INVALID-UBX-MESSAGE_

    // Verify the length and get full the packet.
    // AI suggested we need to mask both byte 4 *and* 5 in case of signed values
    //length ::= (io-reader.peek-byte 4) | (((io-reader.peek-byte 5) & 0xff) << 8)
    length ::= ((io-reader.peek-byte 4) & 0xff) | (((io-reader.peek-byte 5) & 0xff) << 8)
    if not 0 <= length <= MAX-MESSAGE-SIZE_: throw INVALID-UBX-MESSAGE_
    frame ::= io-reader.peek-bytes length + 8

    // Verify the checksum.
    if not is-valid-frame_ frame: throw INVALID-UBX-MESSAGE_

    msg-class ::= frame[2]
    msg-id    ::= frame[3]
    payload   ::= frame[6..length + 6]
    io-reader.skip length + 8
    return Message msg-class msg-id payload

  static is-valid-frame_ frame/ByteArray -> bool:
    // Check the sync bytes.
    //if frame[0] != 0xb5 or frame[1] != 0x62: throw INVALID-UBX-MESSAGE_
    // Don't throw here- let this function do as designed and allow
    // the specific constructors etc determine what to do.  In every call of
    // is-valid-frame_, it is thrown on false.
    if frame[0] != 0xb5 or frame[1] != 0x62: return false

    // Check the payload length.
    length ::= LITTLE-ENDIAN.uint16 frame 4
    if not 0 <= length <= MAX-MESSAGE-SIZE_: return false
    if frame.size != length + 8: return false

    ck-a ::= frame[frame.size - 2]
    ck-b ::= frame[frame.size - 1]

    // AI suggested this close was bad:
    //compute-checksum_ frame: | a b |
    //  return ck-a == a and ck-b == b
    //return false

    // And suggested this replacement.  (Want to discuss)
    ok := false
    compute-checksum_ frame: | a b |
      ok = (ck-a == a) and (ck-b == b)
    return ok

  /**
  Computes the checksum of the given $bytes.

  Calls the $callback with the computed checksum values ck_a and ck_b as
    arguments.
  */
  static compute-checksum_ bytes/ByteArray [callback]:
    ck-a := 0
    ck-b := 0
    bytes = bytes[2..bytes.size - 2]
    bytes.size.repeat: | i |
      ck-a = (ck-a + bytes[i]) & 0xff
      ck-b = (ck-b + ck-a) & 0xff
    callback.call ck-a ck-b

  /**
  Transforms this message to a byte array that can be send to a ublox
    GNSS receiver.

  The byte array contains the starting magic bytes 0xB5 and 0x62 as well as
    the trailing checksum.
  */
  to-byte-array -> ByteArray:
    bytes := ByteArray 8 + payload.size
    bytes[0] = 0xB5
    bytes[1] = 0x62
    bytes[2] = cls
    bytes[3] = id
    LITTLE-ENDIAN.put-uint16 bytes 4 payload.size
    bytes.replace 6 payload
    compute-checksum_ bytes: | ck-a ck-b |
      bytes[bytes.size - 2] = ck-a
      bytes[bytes.size - 1] = ck-b
    return bytes

  class-string_ -> string:
    return PACK-CLASSES.get cls --if-absent=:
      return "0x$(%02x cls)"

  id-string_ -> string:
    return "0x$(%02x id)"

  /** See $super. */
  stringify -> string:
    return "UBX-$class-string_-$id-string_"

  /** Hash Code for use as an identifier in a Map. */
  hash-code:
    return #[cls, id]

/**
The UBX-ACK-ACK message.

Contains the class ID and message ID of the acknowledged message.
*/
class AckAck extends Message:
  /** The UBX-ACK-ACK message ID. */
  static ID ::= 0x01

  /** Lowest protocol version with this message type. */
  static MIN-PROTVER ::= "12.0"

  /** Constructs a dummy acknowledge message. */
  constructor.private_ cls id:
    super.private_ Message.ACK ID #[cls, id]

  /** Constructs a dummy acknowledge message. */
  constructor.private_ payload:
    super.private_ Message.ACK ID payload

  /** The class ID of the acknowledged message. */
  class-id -> int:
    return LITTLE-ENDIAN.uint8 payload 0

  /** The class ID (converted to text) of the acknowledged message. */
  class-id-text -> string:
    return Message.PACK-CLASSES[class-id]

  /** The message ID of the acknowledged message. */
  message-id -> int:
    return LITTLE-ENDIAN.uint8 payload 1

  /** The message ID (converted to text, if known) of the acknowledged message. */
  message-id-text -> string:
    output := ""
    if Message.PACK-MESSAGE-TYPES.contains class-id:
      if Message.PACK-MESSAGE-TYPES[class-id].contains message-id:
        output = Message.PACK-MESSAGE-TYPES[class-id][message-id]
    return output

  id-string_ -> string:
    return "ACK"

  stringify -> string:
    return  "$(super.stringify): [$(class-id):$(class-id-text),$(message-id):$(message-id-text)]"

/**
The UBX-ACK-NAK message.

Contains the class ID and message ID of the NAK (not acknowledged) message.
*/

class AckNak extends Message:
  /** The UBX-ACK-NAK message ID. */
  static ID ::= 0x00

  /** Lowest protocol version with this message type. */
  static MIN-PROTVER ::= "12.0"

  /** Constructs a dummy NAK message. */
  constructor.private_ cls id:
    super.private_ Message.ACK ID #[cls, id]

  constructor.private_ bytearray/ByteArray:
    super.private_ Message.ACK ID bytearray

  /** The class ID of the NAK message. */
  class-id -> int:
    return LITTLE-ENDIAN.uint8 payload 0

  /** The class ID (converted to text) of the negative-acknowledge message. */
  class-id-text -> string:
    return Message.PACK-CLASSES[class-id]

  /** The message ID of the NAK message. */
  message-id -> int:
    return LITTLE-ENDIAN.uint8 payload 1

  /** The message ID (converted to text, if known) of the acknowledged message. */
  message-id-text -> string:
    output := ""
    if Message.PACK-MESSAGE-TYPES.contains class-id:
      if Message.PACK-MESSAGE-TYPES[class-id].contains message-id:
        output = Message.PACK-MESSAGE-TYPES[class-id][message-id]
    return output

  id-string_ -> string:
    return "NAK"

  stringify -> string:
    return  "$(super.stringify): [$(class-id):$(class-id-text),$(message-id):$(message-id-text)]"


/**
The UBX-CFG-MSG message.

Configures the rate at which messages are sent by the receiver.
*/
class CfgMsg extends Message:
  /** The UBX-CFG-MSG message ID. */
  static ID ::= 0x01

  /**
  Constructs a configuration message.

  When sent to the receiver, the message with the given $msg-class and
    $msg-id will be sent at the given $rate.
  */
  constructor.message-rate --msg-class --msg-id --rate:
    super.private_ Message.CFG ID #[msg-class, msg-id, rate]

  /** Poll the configuration. */
  constructor.poll --msg-class --msg-id:
    super.private_ Message.CFG ID #[msg-class, msg-id]

  /** Set per-port rates. */
  constructor.per-port --msg-class --msg-id --rates/ByteArray:
    assert: rates.size == 6
    super.private_ Message.CFG ID (#[msg-class, msg-id] + rates)

  id-string_ -> string:
    return "MSG"


/**
The UBX-CFG-PRT message.

Configures a port (most commonly UART1) for baud rate, framing, and protocol masks.
Also supports polling a port's current configuration.

Payload layout (legacy M8/M9):
  offset size  field
  0      1     portID
  1      1     reserved0
  2      2     txReady (ignored here)
  4      4     mode       (bitfield: data bits, parity, stop bits)
  8      4     baudRate   (uint32, e.g., 115200)
  12     2     inProtoMask  (bit0=UBX, bit1=NMEA, bit2=RTCM2, bit5=RTCM3...)
  14     2     outProtoMask (same bit layout)
  16     2     flags
  18     2     reserved1
Total payload length: 20 bytes
*/
class CfgPrt extends Message:
  /** The UBX-CFG-PRT message ID. */
  static ID ::= 0x00

  static MIN-PROTVER/string := "15.0"
  static MAX-PROTVER/string := "23.0"   // Manual says not after this version.

  // Common constants (see u-blox docs)
  static PORT-UART1 ::= 0x01
  static PORT-UART2 ::= 0x02

  // Todo: expose these on the constructor
  // mode bitfield shortcut: 8 data bits, no parity, 1 stop (8N1)
  // (charLen=3 -> bits 6..7 = 0b11; parity=0 -> bits 9..11 = 0; nStop=1 -> bit 12 = 0)
  // u-blox ref value: 0x000008D0
  static MODE-DATA-BITS-MASK_ := 0b00000000_01100000
  static MODE-PARITY-MASK_    := 0b00000111_00000000
  static MODE-STOP-BITS-MASK_ := 0b00011000_00000000

  // Common Mode Presets
  static MODE-8N1 ::= 0x000008D0
  static MODE-7E1 ::= 0x00000080
  static MODE-8O2 :=  0x000000C0

  // Protocol mask bits (legacy)
  static PROTO-UBX   ::= 0b00000001
  static PROTO-NMEA  ::= 0b00000010
  static PROTO-RTCM2 ::= 0b00000100
  static PROTO-RTCM3 ::= 0b00100000

  /**
  Build a configuration to set a UART port.

  Defaults: UART1, 115200 baud, 8N1, UBX-only (in/out).  Explicit proto masks
    can be specified via --in-proto/--out-proto.
  */
  constructor.uart
      --port-id/int=CfgPrt.PORT-UART1
      --baud/int=9600
      --mode/int=CfgPrt.MODE-8N1
      --in-proto/int=CfgPrt.PROTO-UBX
      --out-proto/int=CfgPrt.PROTO-UBX
      --flags/int=0:
    super.private_ Message.CFG ID (ByteArray 20)

    // portID, reserved0, txReady(2)
    LITTLE-ENDIAN.put-uint8  payload 0 port-id
    LITTLE-ENDIAN.put-uint8  payload 1 0
    LITTLE-ENDIAN.put-uint16 payload 2 0     // txReady off

    // mode (framing)
    LITTLE-ENDIAN.put-uint32 payload 4 mode

    // baudRate
    LITTLE-ENDIAN.put-uint32 payload 8 baud

    // in/out proto masks
    LITTLE-ENDIAN.put-uint16 payload 12 in-proto
    LITTLE-ENDIAN.put-uint16 payload 14 out-proto

    // flags, reserved1
    LITTLE-ENDIAN.put-uint16 payload 16 flags
    LITTLE-ENDIAN.put-uint16 payload 18 0

  /**
  Poll the configuration for a given port.
  The poll payload is a single byte: portID.
  */
  constructor.poll --port-id/int=CfgPrt.PORT-UART1:
    super.private_ Message.CFG ID (ByteArray 1)
    LITTLE-ENDIAN.put-uint8 payload 0 port-id

  /** Construct from an incoming payload. */
  constructor.private_ payload/ByteArray:
    super.private_ Message.CFG ID payload

  port-id -> int:
    return LITTLE-ENDIAN.uint8 payload 0

  mode -> int:
    return LITTLE-ENDIAN.uint32 payload 4

  baud-rate -> int:
    return LITTLE-ENDIAN.uint32 payload 8

  in-proto-mask -> int:
    return LITTLE-ENDIAN.uint16 payload 12

  out-proto-mask -> int:
    return LITTLE-ENDIAN.uint16 payload 14

  flags -> int:
    return LITTLE-ENDIAN.uint16 payload 16

  id-string_ -> string:
    return "PRT"


/**
The UBX-CFG-RST message.

Resets the receiver.
*/
class CfgRst extends Message:
  /** The UBX-CFG-RST message ID. */
  static ID ::= 0x04

  /**
  Constructs a reset message.

  The default parameters are a controlled software reset with a cold start.
  See the description for other parameter options.
  */
  constructor --clear-sections=0xFFFF --reset-mode=2:
    super.private_ Message.CFG ID (ByteArray 4)
    LITTLE-ENDIAN.put-uint16 payload 0 clear-sections
    LITTLE-ENDIAN.put-uint8  payload 2 reset-mode
    LITTLE-ENDIAN.put-uint8  payload 3 Message.RESERVED_

  id-string_ -> string:
    return "RST"

/**
The UBX-NAV-STATUS message.

The receiver navigation status.
*/
// https://www.u-blox.com/en/docs/UBX-13003221#%5B%7B%22num%22%3A1204%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C559.1%2Cnull%5D
class NavStatus extends Message:
  /** The UBX-NAV-STATUS message ID. */
  static ID ::= 0x03

  /** Unknown GNSS fix. */
  static NO-FIX ::= 0
  /** Dead reckoning only. */
  static DEAD-RECKONING-ONLY ::= 1
  /** 2D fix. */
  static FIX-2D ::= 2
  /** 3D fix. */
  static FIX-3D ::= 3
  /** GPS and dead reckoning. */
  static GPS-DEAD-FIX ::= 4
  /** Time only fix. */
  static TIME-ONLY ::= 5
  /** Constructs a poll UBX-NAV-STATUS message. */

  static PACK-FIX-TYPES ::= {
    NO-FIX: "NO-FIX",
    DEAD-RECKONING-ONLY : "DEAD-RECKONING-ONLY",
    FIX-2D : "FIX-2D",
    FIX-3D : "FIX-3D",
    GPS-DEAD-FIX : "GPS-DEAD-FIX",
    TIME-ONLY : "TIME-ONLY"}

  constructor.poll:
    super.private_ Message.NAV ID #[]

  constructor.private_ payload:
    super.private_ Message.NAV ID payload

  id-string_ -> string:
    return "STATUS"

  /** The GPS interval time of week of the navigation epoch. */
  itow -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint32 payload 0

  /**
  Returns the type of fix.

  One of $NO-FIX, $DEAD-RECKONING-ONLY, $FIX-2D, $FIX-3D, $GPS-DEAD-FIX, $TIME-ONLY.
  */
  gps-fix -> int:
    assert: not payload.is-empty
    //return LITTLE-ENDIAN.uint8 payload 4
    return LITTLE-ENDIAN.uint8 payload 4

  // Thinking to remove this and have the user/driver do it via the PACK... static.
  gps-fix-text -> string:
    assert: not payload.is-empty
    return PACK-FIX-TYPES[gps-fix]

  /**
  Navigation status flags.
  See receiver specification for details.
  */
  flags -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 5

  /**
  Fix status information
  See receiver specification for details.
  */
  fix-stat -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 6

  /**
  Additional status information.
  See receiver specification for details.
  */
  flags2 -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 7

  /** Time to first fix in milliseconds. */
  time-to-first-fix -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint32 payload 8

  /** Milliseconds since startup or reset. */
  msss -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint32 payload 12

/**
The UBX-NAV-SAT message.

Satellite information.
*/
// https://www.u-blox.com/en/docs/UBX-13003221#%5B%7B%22num%22%3A1039%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C841.89%2Cnull%5D
class NavSat extends Message:
  /** The UBX-NAV-SAT message ID. */
  static ID ::= 0x35

  constructor.private_ payload/ByteArray:
    super.private_ Message.NAV ID payload

  id-string_ -> string:
    return "SAT"

  /** The GPS interval time of week of the navigation epoch. */
  itow -> int:
    assert: not payload.is-empty
    // Correction itow is int32 (as per all other instances in this code)
    //return LITTLE-ENDIAN.uint8 payload 0
    return LITTLE-ENDIAN.uint32 payload 0

  /** Message version. */
  version -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 4

  /** Number of satellites. */
  num-svs -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 5

  /**
  How many satellites in the message.

  Introduced for compatibility with NavSvInfo
  */
  satellite-count -> int:
    return num-svs

  /**
  The satellite data in the package for the given $index.

  The $index must satisfy 0 <= $index < $num-svs.
  */
  satellite-data index -> SatelliteData:
    assert: not payload.is-empty
    if not 0 <= index < num-svs: throw "INVALID ARGUMENT"
    return SatelliteData index payload --src-id=ID

/**
Satellite data for a single satellite.

The satellite data is included in the UBX-NAV-SAT message (See $NavSat).  Class
  contains properties for SV's common to both UBX-NAV-SVINFO and UBX-NAV-SAT,
  and parsers for use with both message types.  Messages are the same length in
  both cases, but different information and layout.
*/
class SatelliteData:
  /** Contains the source of this Satellite entry.

  Content and format of fields like $flags will depend on the message source.
  */
  source/int

  /** The index of this data in the original message. */
  index/int

  /** GNSS identifier. */
  gnss-id/int := 0

  /** GNSS identifier. (Legacy: same field as GNSS identifier.)*/
  channel/int := 0

  /** Satellite identifier. */
  sv-id/int

  /** Carrier to noise ratio. */
  cno/int

  /** Elevation. */
  elev/int

  /** Azimuth. */
  azim/int

  /** Pseudorange residual. */
  pr-res/float

  /** Space Vehicle health indicator
    For compatibility:
    0: unknown
    1: healthy
    2: unhealthy
  */
  health/int
  /**
  Flags. Includes $quality, $orbit-source, $alm-avail, and $ano-avail.  See
    receiver specification for details.
  */
  flags/int

  /** Signal quality indicator.

  Signal quality values:
   - 0: no signal
   - 1: searching signal
   - 2: signal acquired
   - 3: signal detected but unusable
   - 4: code locked and time synchronized
   - 5, 6, 7: code and carrier locked and time synchronized

  Note: Since IMES signals are not time synchronized, a channel tracking an IMES
    signal can never reach a quality indicator value of higher than 3.
  */
  quality/int

  /**
  Orbit source.

  Field Definitions (M8)
    0: no orbit information is available for this SV
    1: ephemeris is used
    2: almanac is used
    3: AssistNow Offline orbit is used
    4: AssistNow Autonomous orbit is used
    5, 6, 7: other orbit information is used
  */
  orbit-source/int

  /**
  Whether orbit information available.

  In 6M, this is true/false. In M8, this is true/false, plus information in the
    other fields about which variants are available.
  */
  orbit-info-avail/bool

  /** Almanac available for this satellite. */
  alm-avail/int := 0

  /** Ephemeris available for this satellite. */
  eph-avail/int := 0

  /** AssistNow Offline data is available for this SV */
  ano-avail/int := 0

  /** AssistNow Autonomous data is available for this SV. */
  aop-avail/int := 0

  /** Differential Correction Data is available for this satellite. */
  diff-corr/bool

  /** Differential Correction Data is available for this satellite. */
  sv-used/bool

  /** Indicates that a carrier smoothed pseudorange used.*/
  smoothed/bool

  /**
  Constructs the satellite data for the given message $payload and data.
    Parses UBX-NAV-SAT and UBX-NAV-SVINFO sourced Space Vehicles.
  */
  constructor .index payload/ByteArray --src-id/int:

    source = src-id

    // Defaults defined here to help keep them visible.
    orbit-info-avail = false
    health = 0
    diff-corr = false
    sv-used = false
    orbit-source = 0
    offset := 0

    if src-id == NavSat.ID:
      offset = index * 12
      gnss-id = LITTLE-ENDIAN.uint8 payload offset + 8

      sv-id = LITTLE-ENDIAN.uint8 payload offset + 9
      cno = LITTLE-ENDIAN.uint8 payload offset + 10
      elev = LITTLE-ENDIAN.int8 payload offset + 11
      azim = LITTLE-ENDIAN.int16 payload offset + 12
      pr-res = (LITTLE-ENDIAN.uint8 payload offset + 14).to-float / 10 // scale 0.1
      flags = LITTLE-ENDIAN.uint32 payload offset + 16

      quality-mask        := 0b00000000_00000111
      sv-used-mask        := 0b00000000_00001000
      health-mask         := 0b00000000_00110000
      diff-corr-mask      := 0b00000000_01000000
      smoothed-mask       := 0b00000000_10000000
      orbit-source-mask   := 0b00000111_00000000
      eph-avail-mask      := 0b00001000_00000000
      alm-avail-mask      := 0b00010000_00000000
      ano-avail-mask      := 0b00100000_00000000
      aop-avail-mask      := 0b01000000_00000000


      quality      = (flags & quality-mask) >> quality-mask.count-trailing-zeros
      health       = (flags & health-mask) >> health-mask.count-trailing-zeros
      orbit-source = (flags & orbit-source-mask) >> orbit-source-mask.count-trailing-zeros
      alm-avail    = (flags & alm-avail-mask) >> alm-avail-mask.count-trailing-zeros
      eph-avail    = (flags & eph-avail-mask) >> eph-avail-mask.count-trailing-zeros
      ano-avail    = (flags & ano-avail-mask) >> ano-avail-mask.count-trailing-zeros
      aop-avail    = (flags & alm-avail-mask) >> alm-avail-mask.count-trailing-zeros
      diff-corr    = ((flags & diff-corr-mask) >> diff-corr-mask.count-trailing-zeros) != 0
      sv-used      = ((flags & sv-used-mask) >> sv-used-mask.count-trailing-zeros) != 0
      smoothed     = ((flags & smoothed-mask) >> smoothed-mask.count-trailing-zeros) != 0

      if (eph-avail != 0) or (alm-avail != 0) or (ano-avail != 0) or (aop-avail != 0):
        orbit-info-avail = true

    else if src-id == NavSvInfo.ID:
      offset = index * 12
      channel = LITTLE-ENDIAN.uint8 payload offset + 8

      sv-id = LITTLE-ENDIAN.uint8 payload offset + 9
      cno = LITTLE-ENDIAN.uint8 payload offset + 12
      elev = LITTLE-ENDIAN.int8 payload offset + 13
      azim = LITTLE-ENDIAN.int8 payload offset + 14
      pr-res = (LITTLE-ENDIAN.uint32 payload offset + 16).to-float / 100 // Scaled in cm.
      flags = LITTLE-ENDIAN.uint32 payload offset + 10

      quality-mask   := 0b00000111
      quality = (LITTLE-ENDIAN.uint32 payload offset + 11) & quality-mask

      sv-used-mask     := 0b00000001
      diff-corr-mask   := 0b00000010
      orbit-avail-mask := 0b00000100
      orbit-eph-mask   := 0b00001000
      unhealthy-mask   := 0b00010000
      orbit-alm-mask   := 0b00100000
      orbit-aop-mask   := 0b01000000
      smoothed-mask    := 0b10000000

      // Directly usable
      diff-corr        = ((flags & diff-corr-mask) >> diff-corr-mask.count-trailing-zeros) != 0
      sv-used          = ((flags & sv-used-mask) >> sv-used-mask.count-trailing-zeros) != 0
      orbit-info-avail = ((flags & orbit-avail-mask) >> orbit-avail-mask.count-trailing-zeros) != 0
      smoothed         = ((flags & smoothed-mask) >> smoothed-mask.count-trailing-zeros) != 0

      // Translated to return outputs matching the later definition
      unhealthy-raw  := (flags & unhealthy-mask) >> unhealthy-mask.count-trailing-zeros
      if unhealthy-mask == 1:
        health = 2
      else if unhealthy-mask == 0:
        health = 1

      // Translated to return outputs as close to the M8 definition as possible
      if ((flags & orbit-eph-mask) >> orbit-eph-mask.count-trailing-zeros) == 1:
        orbit-source = 1
      else if ((flags & orbit-alm-mask) >> orbit-alm-mask.count-trailing-zeros) == 1:
        orbit-source = 2
      else if ((flags & orbit-aop-mask) >> orbit-aop-mask.count-trailing-zeros) == 1:
        orbit-source = 4

    else:
      throw "Unknown Space Vehicle Definition Source"


  /** See $super. */
  stringify -> string:
    codes := ""
    if alm-avail == 1: codes += "A"
    if ano-avail == 1: codes += "N"

    // TODO(kasper): Make this output a whole lot prettier and easier to parse.
    //          ian: Added class/id type string from $super to assist with
    //               tests.  Would be cool to standardise them somehow...?
    return "$(super.stringify): $index|$gnss-id|$sv-id|$cno|$quality|$orbit-source|$codes"

/**
The UBX-MON-VER message.

Handles receiver/software/hardware version information.
*/
// Poll request: class MON (0x0A), id 0x04, payload length 0
// Reply: payload = swVersion[30], hwVersion[10], then 0..N extension
//        strings each 30 bytes. All are NULL terminated ASCII.
// Note:  M8 and M9 (and later versions of 6M have extra data returned
//        in extensions.
class MonVer extends Message:
  /** The UBX-MON-VER message ID. */
  static ID ::= 0x04

  /** Construct a poll-request UBX-MON-VER. */
  constructor.poll:
    super.private_ Message.MON ID #[]

  /** Construct from an incoming payload. */
  constructor.private_ payload/ByteArray:
    super.private_ Message.MON ID payload

  /** See $super. */
  id-string_ -> string:
    return "VER"

  /** Software version string (NULL-terminated inside 30 bytes). */
  sw-version -> string:
    return convert-string_ 0 30

  /** Hardware version string (NULL-terminated inside 10 bytes). */
  hw-version -> string:
    return convert-string_ 30 10

  /** Returns true if an extension row exists containing the supplied string. */
  has-extension str/string -> bool:
    //return (extensions-raw.contains extension-name)
    return (extensions-raw.any: (it.index-of str) > -1)

  /** If a string exists in the extensions with the supplied text, return the
  entire line . */
  extension str/string -> string?:
    //return (extensions-raw.contains extension-name)
    extensions-raw.any:
      if (it.index-of str) > -1:
        return it
    return null

  /**
  Returns a map of extension string AVPs.

  This function returns a map of strings with the keyed by the first part, with
    the value being the remainder past the first instance of "=".

  DISABLED: Originally it looked like the extensions would be AVPs delimited by
    '='.  This is not true in all device generations. In addition, the sets of
    tags shown in the extensions aren't grouped, and can be split across more
    than one extension 'row'.  So for now, this is disabled in favor of the
    driver doing what is required for parsing of these messages.

  extensions -> Map:
    raw-extensions := extensions-raw
    output-extensions := {:}
    print "TESTING $(extensions-raw)"
    raw-extensions.do:
      eq-pos := it.index-of "="
      if eq-pos > -1:
        output-extensions[it[..eq-pos]] = it[..(eq-pos + 1)]
      else:
        output-extensions[it] = ""
    return output-extensions
  */

  /**
  Returns a list of extension strings, if present.

  If provided by the firmware version on the device, it provides a list of n 30
    byte entries.  Each entry is a NUL-terminated ASCII string with an AVP
    delimited by '='.
  */
  extensions-raw -> List:
    raw-extensions := []
    offset := 40
    eq-pos := ?
    while offset + 30 <= payload.size:
      raw-extensions.add (convert-string_ offset 30)
      offset += 30
    return raw-extensions


  /** Helper: read a NULL-terminated string from a fixed-size field. */
  convert-string_ start length -> string:
    // Find first NUL within [start .. start+length).
    end := start
    limit := start + length
    while (end < limit) and (LITTLE-ENDIAN.uint8 payload end) != 0:
      end += 1

    // Slice bytes [start .. end) and convert to a Toit string.
    return (payload[start..end]).to-string.trim


/**
The UBX-NAV-POSLLH message.

Geodetic position solution.  Works on u-blox 6 and M8.
*/
class NavPosLlh extends Message:
  static ID ::= 0x02

  constructor.private_ payload/ByteArray:
    super.private_ Message.NAV ID payload

  id-string_ -> string:
    return "POSLLH"

  /** GPS time of week of the navigation epoch. */
  itow -> int:
    return LITTLE-ENDIAN.uint32 payload 0

  /** longitude. (1e-7 degrees.) */
  longitude-raw -> int:
    return LITTLE-ENDIAN.int32  payload 4

  /** Latitude. (1e-7 degrees.) */
  latitude-raw    -> int:
    return LITTLE-ENDIAN.int32  payload 8

  /** Height above ellipsoid. */
  height-mm  -> int:
    return LITTLE-ENDIAN.int32  payload 12

  /** Height above mean sea level. */
  height-msl-mm   -> int:
    return LITTLE-ENDIAN.int32  payload 16

  /** Horizontal measurement accuracy estimate. */
  horizontal-accuracy-mm   -> int:
    return LITTLE-ENDIAN.uint32 payload 20

  /** Horizontal measurement accuracy estimate. */
  vertical-accuracy-mm   -> int:
    return LITTLE-ENDIAN.uint32 payload 24

  // Convenience in float degrees
  longitude-deg -> float:
    return longitude-raw / 1e7

  latitude-deg -> float:
    return latitude-raw / 1e7

  stringify -> string:
    return  "UBX-$class-string_-$id-string_: [Latitude:$(latitude-deg),Longtidude:$(longitude-deg)]"


/**
The UBX-NAV-SVINFO message.

"Space vehicle info" message (legacy). Present on u-blox 6; kept on M8 for
  backward compatibility.
*/
class NavSvInfo extends Message:
  static ID ::= 0x30

  constructor.private_ payload/ByteArray:
    super.private_ Message.NAV ID payload

  id-string_ -> string:
    return "SVINFO"

  /** The GPS interval time of week of the navigation epoch. (ms) */
  itow   -> int:
    return LITTLE-ENDIAN.uint32 payload 0

  /** Number of channels. */
  num-ch     -> int:
    return LITTLE-ENDIAN.uint8 payload 4

  /** Global flags bitmask.

  Mask 0b00000111 contains a number representing chip hardware generation:
   - 0: Antaris, Antaris 4
   - 1: u-blox 5
   - 2: u-blox 6
   - 3: u-blox 7
   - 4: u-blox 8 / u-blox M8
  */
  global-flags -> int:
    return LITTLE-ENDIAN.uint8 payload 5

  /**
  How many satellites in the message.

  Introduced for compatibility with NavSat
  */
  satellite-count -> int:
    return num-ch

  /**
  The satellite data in the package for the given $index.

  The $index must satisfy 0 <= $index < $num-ch.
  */
  satellite-data index -> SatelliteData:
    assert: not payload.is-empty
    if not 0 <= index < num-ch: throw "INVALID ARGUMENT"
    return SatelliteData index payload --src-id=ID


/**
The UBX-NAV-PVT message.

Navigation, position, velocity, and time solution.
*/
class NavPvt extends Message:
  /** The UBX-NAV-PVT message ID. */
  static ID ::= 0x07

  /** Unknown GNSS fix. */
  static FIX-TYPE-UNKNOWN ::= 0
  /** Dead reckoning only. */
  static FIX-TYPE-DEAD ::= 1
  /** 2D fix. */
  static FIX-TYPE-2D ::= 2
  /** 3D fix. */
  static FIX-TYPE-3D ::= 3
  /** GNSS and dead reckoning. */
  static FIX-TYPE-GNSS-DEAD ::= 4
  /** Time only fix. */
  static FIX-TYPE-TIME-ONLY ::= 5

  /** Constructs a poll UBX-NAV-PVT message. */
  constructor.poll:
    super.private_ Message.NAV ID #[]

  constructor.private_ payload/ByteArray:
    super.private_ Message.NAV ID payload

  id-string_ -> string:
    return "PVT"

  /** Whether this is a GNSS fix. */
  is-gnss-fix -> bool:
    return (flags & 0b00000001) != 0

  /** The time in UTC. */
  utc-time -> Time:
    return Time.utc year month day h m s --ns=ns

  /** The GPS interval time of week of the navigation epoch. */
  itow -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint32 payload 0

  /** The year (UTC). */
  year -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint16 payload 4

  /**
  The month (UTC).
  In the range [1..12].
  */
  month -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 6

  /**
  The day (UTC).
  In the range [1..31].
  */
  day -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 7

  /**
  The hours (UTC).
  In the range [0..23].
  */
  h -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 8

  /**
  The minutes (UTC).
  In the range [0..59].
  */
  m -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 9

  /**
  The seconds (UTC).
  In the range [0..60].
  */
  s -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 10

  /**
  Validity flag.
  See receiver specification for details.
  */
  valid -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 11

  /** Time accuracy estimate in nanoseconds */
  time-acc -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint32 payload 12

  /**
  Fraction of second in nano seconds.
  The fraction may be negative.
  */
  ns -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.int32 payload 16

  /**
  The type of fix.
  One of $FIX-TYPE-UNKNOWN, $FIX-TYPE-DEAD, $FIX-TYPE-2D, $FIX-TYPE-3D, $FIX-TYPE-GNSS-DEAD, $FIX-TYPE-TIME-ONLY.
  */
  fix-type -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 20

  /**
  Fix status flags.
  See receiver specification for details.
  */
  flags -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 21

  /**
  Additional fix status flags.
  See receiver specification for details.
  */
  flags2 -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 22

  /** Number of satellites used for fix. */
  num-sv -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 23

  /** Longitude. */
  lon -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.int32 payload 24

  /** Latitude. */
  lat -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.int32 payload 28

  /** Height above ellipsoid in millimeter. */
  height -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.int32 payload 32

  /** Height above mean sea level in millimeter. */
  height-msl -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.int32 payload 36

  /** Horizontal accuracy in millimeter. */
  horizontal-acc -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint32 payload 40

  /** Vertical accuracy in millimeter. */
  vertical-acc -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint32 payload 44

  /** NED north velocity in millimeters per second. */
  north-vel -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.int32 payload 48

  /** NED east velocity in millimeters per second. */
  east-vel -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.int32 payload 52

  /** NED down velocity in millimeters per second. */
  down-vel -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.int32 payload 56

  /** Ground speed (2D) in millimeters per second. */
  ground-speed -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.int32 payload 60

  /** Heading of motion (2D). */
  heading-of-motion -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.int32 payload 64

  /** Speed accuracy in millimeters per second. */
  speed-acc -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint32 payload 68

  /** Hading accuracy. */
  heading-acc -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint32 payload 72

  /**
  Position DOP.

  Position 'Dilution of Position' scale.  Scale 0.01.
  */
  position-dop -> float:
    assert: not payload.is-empty
    return (LITTLE-ENDIAN.uint16 payload 76).to-float / 100

  /**
  Additional flags.
  See receiver specification for details.
  */
  flags3 -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint32 payload 78

  /**
  The heading of the vehicle.
  See receiver specification for details.
  */
  heading-vehicle -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.int32 payload 84

  /**
  Magnetic declination.
  See receiver specification for details.
  */
  magnetic-declination -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.int16 payload 88

  /**
  Accuracy of magnetic declination.
  See receiver specification for details.
  */
  magnetic-acc -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint16 payload 90


/**
The UBX-NAV-SOL message.

Legacy Navigation solution, in ECEF. Included for backwards compatibility.
  Works on M8 and later however NAV-PVT messages are preferred).
*/
class NavSol extends Message:
  static ID ::= 0x06

  /** Unknown GNSS fix. */
  static FIX-TYPE-UNKNOWN ::= 0
  /** Dead reckoning only. */
  static FIX-TYPE-DEAD ::= 1
  /** 2D fix. */
  static FIX-TYPE-2D ::= 2
  /** 3D fix. */
  static FIX-TYPE-3D ::= 3
  /** GNSS and dead reckoning. */
  static FIX-TYPE-GNSS-DEAD ::= 4
  /** Time only fix. */
  static FIX-TYPE-TIME-ONLY ::= 5

  /** Constructs a poll UBX-NAV-SOL message. */
  constructor.poll:
    super.private_ Message.NAV ID #[]

  constructor.private_ payload/ByteArray:
    super.private_ Message.NAV ID payload

  id-string_ -> string:
    return "SOL"

  /** Whether this is a GNSS fix. */
  is-gnss-fix -> bool:
    return (flags & 0b00000001) != 0

  /** The time in UTC.

  Time is not included in this legacy Message type.  It can be obtained using
    other messages however.  Need to think what to do with this one.  Take out
    for both, or leave in?  Perhaps remove custom properties (made for human
    consumption) on the message types, and have the driver worry about
    presentation of message data...

  utc-time -> Time:
    return Time.utc year month day h m s --ns=ns
  */

  /** The GPS interval time of week of the navigation epoch. */
  itow -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint32 payload 0

  /**
  The fractional GPS interval time of week (in ns) of the navigation epoch.

  Range in ns: -500000..+500000
  */
  ftow -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.int32 payload 4

  /**
  The GPS Week number.

  This is the week number since Jan 6 1980.
  */
  week -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.int16  payload 8

  /**
  Returns if GPS Week number is Valid. (WKNSET)
  */
  valid-week -> bool:
    week-valid-mask := 0b00000100
    return ((flags & week-valid-mask) >> week-valid-mask.count-trailing-zeros) != 0

  /**
  Returns if GPS Time of Week number is Valid. (TOWSET)
  */
  valid-time-of-week -> bool:
    time-of-week-valid-mask := 0b00001000
    return ((flags & time-of-week-valid-mask) >> time-of-week-valid-mask.count-trailing-zeros) != 0

  //The precise GPS time of week in seconds is:
  //(iTOW * 1e-3) + (fTOW * 1e-9)

  /**
  Returns true if DGPS is used.  (diffSoln)
  */
  dgps-used -> bool:
    dgps-used-mask := 0b00000010
    return ((flags & dgps-used-mask) >> dgps-used-mask.count-trailing-zeros) != 0

  /**
  The type of fix.
  One of $FIX-TYPE-UNKNOWN, $FIX-TYPE-DEAD, $FIX-TYPE-2D, $FIX-TYPE-3D, $FIX-TYPE-GNSS-DEAD, $FIX-TYPE-TIME-ONLY.
  */
  fix-type -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 10

  /**
  Fix status flags.
  See receiver specification for details.
  */
  flags -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 11

  /**
  Number of satellites used for fix.
  */
  num-sv -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8 payload 47

  /**
  Position DOP.

  Position 'Dilution of Position' scale.  Scale 0.01.
  */
  position-dop -> float:
    assert: not payload.is-empty
    return (LITTLE-ENDIAN.uint16 payload 44).to-float / 100

  // Raw Fields - try to present so as to be similar to NAV-PVT
  ecef-x-cm  -> int:   return LITTLE-ENDIAN.int32  payload 12      // I4 cm
  ecef-y-cm  -> int:   return LITTLE-ENDIAN.int32  payload 16      // I4 cm
  ecef-z-cm  -> int:   return LITTLE-ENDIAN.int32  payload 20      // I4 cm
  p-acc-cm   -> int:   return LITTLE-ENDIAN.uint32 payload 24      // U4 cm

  ecef-vx-cms -> int:  return LITTLE-ENDIAN.int32  payload 28      // I4 cm/s
  ecef-vy-cms -> int:  return LITTLE-ENDIAN.int32  payload 32      // I4 cm/s
  ecef-vz-cms -> int:  return LITTLE-ENDIAN.int32  payload 36      // I4 cm/s
  s-acc-cms   -> int:  return LITTLE-ENDIAN.uint32 payload 40      // U4 cm/s

  reserved1  -> int:   return LITTLE-ENDIAN.uint8  payload 46      // U1
  reserved2  -> int:   return LITTLE-ENDIAN.uint32 payload 48      // U4 (M8 doc shows U1[4]; same 4 bytes)

/**
The UBX-NAV-TIMEUTC message.

UTC time solution, and functions on 6M and later devices.
*/
class NavTimeUtc extends Message:
  static ID ::= 0x21

  /** Constructs a poll UBX-NAV-TIMEUTC message. */
  constructor.poll:
    super.private_ Message.NAV ID #[]

  constructor.private_ payload/ByteArray:
    super.private_ Message.NAV ID payload

  id-string_ -> string: return "TIMEUTC"

  /** The GPS interval time of week of the navigation epoch. */
  itow -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint32 payload 0

  /** The time in UTC. */
  utc-time -> Time:
    assert: not payload.is-empty
    return Time.utc year month day h m s --ns=ns

  /** UTC Time accuracy estimate, in nanoseconds */
  time-acc -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint32 payload 4

  ns -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.int32  payload 8

  year -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint16 payload 12

  month -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8  payload 14

  day -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8  payload 15

  h -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8  payload 16

  m -> int:
    assert: not payload.is-empty
    return LITTLE-ENDIAN.uint8  payload 17

  /**
  Return UTC Seconds.

  Normally 00..59, but leap second can produce 60.
  */
  s -> int:
    return LITTLE-ENDIAN.uint8  payload 18

  /**
  Validity of time flags.

  M8+: upper bits carry UTC standard
  */
  validflags -> int:
    return LITTLE-ENDIAN.uint8 payload 19

  /**
  Returns if GPS Week number is Valid. (ValidWKN)
  */
  valid-week -> bool:
    week-valid-mask := 0b00000010
    return ((validflags & week-valid-mask) >> week-valid-mask.count-trailing-zeros) != 0

  /**
  Returns if GPS Time of Week number is Valid. (ValidTOW)
  */
  valid-time-of-week -> bool:
    time-of-week-valid-mask := 0b00000001
    return ((validflags & time-of-week-valid-mask) >> time-of-week-valid-mask.count-trailing-zeros) != 0

  /**
  Returns if UTC time is valid. (ValidUTC - If the leap seconds are known.)
  */
  valid-utc -> bool:
    valid-utc-mask := 0b00000100
    return ((validflags & valid-utc-mask) >> valid-utc-mask.count-trailing-zeros) != 0

  /**
  Returns UTC standard code. (Returns 0 on Legacy)

  Common values:
    0: Information not available
    1: Communications Research Labratory (CRL), Tokyo, Japan
    2: National Institute of Standards and Technology (NIST)
    3: U.S. Naval Observatory (USNO)
    4: International Bureau of Weights and Measures (BIPM)
    5: European laboratories
    6: Former Soviet Union (SU)
    7: National Time Service Center (NTSC), China
    8: National Physics Laboratory India (NPLI)
    15: Unknown
  */
  utc-standard -> int:
    return (validflags >> 4) & 0x0F

/**
The UBX-NAV-TP5 message.

Used to configure the TIMEPULSE/PPS pin for time synchronisation.
*/
/*
Payload (32 bytes):
  0  : tpIdx (U1)       // 0=TIMEPULSE, 1=TIMEPULSE2 (if available)
  1  : version (U1)     // 1
  2..3 : reserved
  4..5 : antCableDelay (I2, ns)
  6..7 : rfGroupDelay  (I2, ns)
  8..11 : freqPeriod (U4)         // Hz if !isLength; period in us if isLength
  12..15: freqPeriodLock (U4)
  16..19: pulseLenRatio (U4)      // duty × 1e-9 if isLength=0; length in ns if isLength=1
  20..23: pulseLenRatioLock (U4)
  24..27: userConfigDelay (I4, ns)
  28..31: flags (U4)
Key flag bits (common):
  bit0: active (1=on)
  bit1: lockGnssFreq
  bit2: lockedOtherSet
  bit5: isFreq (else period)
  bit6: isLength (else ratio)
  bit10: alignToTow
  bit11: polarity (1=active high)
  bit14: timeGridUtc (vs. GNSS time)  [chip-specific]
*/
class CfgTp5 extends Message:
  static ID ::= 0x31

  // Index
  static TP-IDX-0 ::= 0
  static TP-IDX-1 ::= 1

  // Flags helpers
  static FLAG-ACTIVE       ::= 0b00000001
  static FLAG-IS-FREQ      ::= 0b00100000
  static FLAG-IS-LENGTH    ::= 0b01000000
  static FLAG-ALIGN-TOW    ::= 0b00000100_00000000
  static FLAG-POLARITY-HI  ::= 0b00001000_00000000
  static FLAG-UTC-GRID     ::= 0b01000000_00000000

  /** Poll the TP5 configuration for tpIdx (0 or 1). */
  constructor.poll --tp-idx/int=TP-IDX-0:
    new-payload := ByteArray 2
    LITTLE-ENDIAN.put-uint8 new-payload 0 tp-idx
    LITTLE-ENDIAN.put-uint8 new-payload 1 1  // version
    super.private_ Message.CFG ID new-payload

  /** Construct an instance with bytes from a retrieved message. */
  constructor.private_ payload/ByteArray:
    super.private_ Message.CFG ID payload

  /**
  Set TP5, starting with common defaults:
    - active, align to TOW, UTC grid off by default
    - frequency mode at 1 Hz, 50% duty, active-high
  */
  constructor.set
      --tp-idx/int=TP-IDX-0
      --ant-cable-ns/int=0
      --rf-group-ns/int=0
      --freq-hz/int=1
      --duty-permille/int=500
      --use-utc/bool=false
      --active/bool=true
      --polarity-high/bool=true:
    new-payload := ByteArray 32
    LITTLE-ENDIAN.put-uint8  new-payload 0 tp-idx
    LITTLE-ENDIAN.put-uint8  new-payload 1 1  // version
    LITTLE-ENDIAN.put-uint16 new-payload 2 0
    LITTLE-ENDIAN.put-int16  new-payload 4 ant-cable-ns
    LITTLE-ENDIAN.put-int16  new-payload 6 rf-group-ns

    // Frequency mode: set freqPeriod=freq, isFreq=1; pulseLenRatio = duty * 1e-9
    LITTLE-ENDIAN.put-uint32 new-payload 8  freq-hz
    LITTLE-ENDIAN.put-uint32 new-payload 12 freq-hz

    dutyRatioNano := duty-permille * 1_000_000  // permille → nanos of 1e9
    LITTLE-ENDIAN.put-uint32 new-payload 16 dutyRatioNano
    LITTLE-ENDIAN.put-uint32 new-payload 20 dutyRatioNano
    LITTLE-ENDIAN.put-int32  new-payload 24 0

    flags := 0
    if active:        flags |= FLAG-ACTIVE
    flags |= FLAG-IS-FREQ                  // frequency mode
    // we used ratio (not length), so FLAG-IS-LENGTH stays 0
    flags |= FLAG-ALIGN-TOW
    if polarity-high: flags |= FLAG-POLARITY-HI
    if use-utc:       flags |= FLAG-UTC-GRID
    LITTLE-ENDIAN.put-uint32 new-payload 28 flags

    super.private_ Message.CFG ID new-payload

  tp-idx -> int:
    return LITTLE-ENDIAN.uint8 payload 0

  flags -> int:
    return LITTLE-ENDIAN.uint32 payload 28

  freq -> int:
    return LITTLE-ENDIAN.uint32 payload 8

  duty-nano -> int:
    return LITTLE-ENDIAN.uint32 payload 16

  id-string_ -> string:
    return "TP5"


/**
The UBX-CFG-NAV5 message.

Classic navigation engine settings (legacy but still widely used).
*/
/*
Payload (36 bytes):
  0..1  mask (U2)             // which fields to apply
  2     dynModel (U1)         // 0=portable, 2=stationary, 3=pede, 4=auto, 6=sea, 7=air1g, 8=air2g, 9=air4g
  3     fixMode (U1)          // 1=2D only, 2=3D only, 3=auto 2D/3D
  4..7  fixedAlt (I4, cm)     // for 2D mode if used
  8..11 fixedAltVar (U4, cm^2)
  12    minElev (I1, deg)
  13    drLimit (U1)          // dead reckoning limit (s)
  14..15 pDop (U2, 0.1)
  16..17 tDop (U2, 0.1)
  18..19 pAcc (U2, m)
  20..21 tAcc (U2, m)
  22    staticHoldThresh (U1, cm/s)
  23    dgnssTimeout (U1, s)
  24    cnoThreshNumSVs (U1)
  25    cnoThresh (U1, dBHz)
  26..27 reserved1
  28..29 staticHoldMaxDist (U2, m)
  30    utcStandard (U1)
  31..35 reserved2
*/
class CfgNav5 extends Message:
  static ID ::= 0x24

  // Mask bits (subset)
  static DYN-MASK_      ::= 0b00000000_00000001
  static FIXMODE-MASK_  ::= 0b00000000_00000010
  static OUTLYING-MASK_ ::= 0b00000000_00000100
  static ALT-MASK_      ::= 0b00000000_00001000
  static DGPS-MASK_     ::= 0b00000000_00010000
  static TDOP-MASK_     ::= 0b00000000_00100000
  static PDOP-MASK_     ::= 0b00000000_01000000
  static PACC-MASK_     ::= 0b00000000_10000000
  static TACC-MASK_     ::= 0b00000001_00000000
  static STATIC-MASK_   ::= 0b00000010_00000000
  static UTC-MASK_      ::= 0b00000100_00000000

  // Dynamic models (subset)
  static DYN-PORTABLE   ::= 0
  static DYN-STATIONARY ::= 2
  static DYN-PEDESTRIAN ::= 3
  static DYN-AUTOMOTIVE ::= 4
  static DYN-SEA        ::= 6
  static DYN-AIR1G      ::= 7
  static DYN-AIR2G      ::= 8
  static DYN-AIR4G      ::= 9

  // Fix mode
  static FIX-2D   ::= 1
  static FIX-3D   ::= 2
  static FIX-AUTO ::= 3

  static PACK-MODELS ::= {
    DYN-PORTABLE: "PORTABLE",
    DYN-STATIONARY: "STATIONARY",
    DYN-PEDESTRIAN: "PEDESTRIAN",
    DYN-AUTOMOTIVE: "AUTOMOTIVE",
    DYN-SEA: "SEA",
    DYN-AIR1G: "AIR1G",
    DYN-AIR2G: "AIR2G",
    DYN-AIR4G: "AIR4G"
  }


  /** Poll current NAV5. */
  constructor.poll:
    super.private_ Message.CFG ID (ByteArray 0)

  /** Construct an instance with bytes from a retrieved message. */
  constructor.private_ payload/ByteArray:
    super.private_ Message.CFG ID payload

  /** Minimal setter: set dyn model + auto 2D/3D, leave others default. */
  constructor.set-basic
      --dyn/int=DYN-AUTOMOTIVE
      --fix/int=FIX-AUTO:
    new-payload := ByteArray 36
    LITTLE-ENDIAN.put-uint16 new-payload 0 (DYN-MASK_ | FIXMODE-MASK_)
    LITTLE-ENDIAN.put-uint8  new-payload 2 dyn
    LITTLE-ENDIAN.put-uint8  new-payload 3 fix
    // sensible defaults / zeros elsewhere
    super.private_ Message.CFG ID new-payload

  /** Full setter for advanced control (pass null to skip a field & mask). */
  constructor.set-advanced
      --dyn/int?=null
      --fix/int?=null
      --fixed-alt-cm/int?=null
      --fixed-alt-var-cm2/int?=null
      --min-elev-deg/int?=null
      --dr-limit-s/int?=null
      --p-dop-x10/int?=null
      --t-dop-x10/int?=null
      --p-acc-m/int?=null
      --t-acc-m/int?=null
      --static-hold-thresh-cmps/int?=null
      --dgnss-timeout-s/int?=null
      --cno-thresh-num-sv/int?=null
      --cno-thresh-dbHz/int?=null
      --static-hold-max-dist-m/int?=null
      --utc-standard/int?=null:
    new-payload := ByteArray 36
    mask := 0

    if dyn != null:
      mask |= DYN-MASK_
      LITTLE-ENDIAN.put-uint8 new-payload 2 dyn
    if fix != null:
      mask |= FIXMODE-MASK_
      LITTLE-ENDIAN.put-uint8 new-payload 3 fix
    if fixed-alt-cm != null:
      mask |= ALT-MASK_
      LITTLE-ENDIAN.put-int32 new-payload 4 fixed-alt-cm
    if fixed-alt-var-cm2 != null:
      mask |= ALT-MASK_
      LITTLE-ENDIAN.put-uint32 new-payload 8 fixed-alt-var-cm2
    if min-elev-deg != null:
      mask |= OUTLYING-MASK_
      LITTLE-ENDIAN.put-int8 new-payload 12 min-elev-deg
    if dr-limit-s != null:
      mask |= OUTLYING-MASK_
      LITTLE-ENDIAN.put-uint8 new-payload 13 dr-limit-s
    if p-dop-x10 != null:
      mask |= PDOP-MASK_
      LITTLE-ENDIAN.put-uint16 new-payload 14 p-dop-x10
    if t-dop-x10 != null:
      mask |= TDOP-MASK_
      LITTLE-ENDIAN.put-uint16 new-payload 16 t-dop-x10
    if p-acc-m != null:
      mask |= PACC-MASK_
      LITTLE-ENDIAN.put-uint16 new-payload 18 p-acc-m
    if t-acc-m != null:
      mask |= TACC-MASK_
      LITTLE-ENDIAN.put-uint16 new-payload 20 t-acc-m
    if static-hold-thresh-cmps != null:
      mask |= STATIC-MASK_
      LITTLE-ENDIAN.put-uint8 new-payload 22 static-hold-thresh-cmps
    if dgnss-timeout-s != null:
      mask |= DGPS-MASK_
      LITTLE-ENDIAN.put-uint8 new-payload 23 dgnss-timeout-s
    if cno-thresh-num-sv != null:
      mask |= OUTLYING-MASK_
      LITTLE-ENDIAN.put-uint8 new-payload 24 cno-thresh-num-sv
    if cno-thresh-dbHz != null:
      mask |= OUTLYING-MASK_
      LITTLE-ENDIAN.put-uint8 new-payload 25 cno-thresh-dbHz
    if static-hold-max-dist-m != null:
      mask |= STATIC-MASK_
      LITTLE-ENDIAN.put-uint16 new-payload 28 static-hold-max-dist-m
    if utc-standard != null:
      mask |= UTC-MASK_
      LITTLE-ENDIAN.put-uint8 new-payload 30 utc-standard

    LITTLE-ENDIAN.put-uint16 new-payload 0 mask
    super.private_ Message.CFG ID new-payload

  dyn-model -> int:
    return LITTLE-ENDIAN.uint8 payload 2

  dyn-model-text -> string:
    return PACK-MODELS[dyn-model]

  fix-mode -> int:
    return LITTLE-ENDIAN.uint8 payload 3

  mask -> int:
    return LITTLE-ENDIAN.uint16 payload 0

  id-string_ -> string:
    return "NAV5"

/**
The UBX-CFG-GNSS message.

Configuring constellations/signals.  Note: Signal bitmasks inside flags are
  chip-family specific (M8 vs M9/M10).  Keeping signals-mask=0 lets firmware
  choose defaults, or bits can be set as needed for advanced use.

Each block is a map with 5 keys, each with:
  block["gnssId"]:   gnssId (1 byte)   - 0=GPS, 1=SBAS, 2=Galileo, 3=BeiDou, 5=QZSS, 6=GLONASS, etc.
  block["resTrkCh"]: resTrkCh (1 byte) - reserved tracking channels
  block["maxTrkCh"]: maxTrkCh (1 byte) - max tracking channels to use
  block["flags"]:    4 byte value      - bit0 enable; higher bits = signal bitmask (chip-depend

Multiple blocks can be created, use the convenience builder for these.
*/
class CfgGnss extends Message:
  static ID ::= 0x3E

  // Common gnssId values
  static GNSS-GPS      ::= 0
  static GNSS-SBAS     ::= 1
  static GNSS-GALILEO  ::= 2
  static GNSS-BEIDOU   ::= 3
  static GNSS-QZSS     ::= 5
  static GNSS-GLONASS  ::= 6

  // Block field numbers
  static BLOCK-GNSSID_    ::= 0
  static BLOCK-RESTRKCH_  ::= 1
  static BLOCK-MAXTRKCH_  ::= 2
  static BLOCK-RESERVED1_ ::= 3
  static BLOCK-FLAGS_     ::= 4

  // Flags helpers
  static FLAG-ENABLE ::= 1

  /** Construct a poll message to get current GNSS configuration. */
  constructor.poll:
    // Empty payload poll (some firmwares accept either empty or msgVer=0)
    super.private_ Message.CFG ID (ByteArray 0)

  /** Construct an instance with bytes from a retrieved message. */
  constructor.private_ payload/ByteArray:
    super.private_ Message.CFG ID payload

  /** Build from a list of 8-byte blocks. numTrkChHw/Use are advisory. */
  constructor.set
      --msg-ver/int=0
      --num-trk-ch-hw/int=0
      --num-trk-ch-use/int=0
      --blocks/List=[]:
    num := blocks.size
    new-payload := ByteArray (4 + 8 * num)
    LITTLE-ENDIAN.put-uint8 new-payload 0 msg-ver
    LITTLE-ENDIAN.put-uint8 new-payload 1 num-trk-ch-hw
    LITTLE-ENDIAN.put-uint8 new-payload 2 num-trk-ch-use
    LITTLE-ENDIAN.put-uint8 new-payload 3 num
    i := 0
    while i < num:
      block := blocks[i]  // Expect map with fields: "gnssId", "resTrkCh", "maxTrkCh", "flags"
      assert: block.size = 5
      base := 4 + 8 * i
      LITTLE-ENDIAN.put-uint8 new-payload (base + BLOCK-GNSSID_) block["gnssId"]
      LITTLE-ENDIAN.put-uint8 new-payload (base + BLOCK-RESTRKCH_) block["resTrkCh"]
      LITTLE-ENDIAN.put-uint8 new-payload (base + BLOCK-MAXTRKCH_) block["maxTrkCh"]
      LITTLE-ENDIAN.put-uint8 new-payload (base + BLOCK-RESERVED1_) 0
      LITTLE-ENDIAN.put-uint32 new-payload (base + BLOCK-FLAGS_) block["flags"]
      i += 1
    super.private_ Message.CFG ID new-payload

  // Convenience builder for one block, many can be supplied
  static create-block
      gnss-id/int
      --enable/bool=true
      --signals-mask/int=0
      --res-trk/int=0
      --max-trk/int=0
      -> Map:
    flags := (enable ? FLAG-ENABLE : 0) | signals-mask
    block/Map := {"gnssId": gnss-id, "resTrkCh": res-trk, "maxTrkCh": max-trk, "flags": flags}
    return block

  msg-ver -> int:
    return LITTLE-ENDIAN.uint8 payload 0

  num-config-blocks -> int:
    return LITTLE-ENDIAN.uint8 payload 3

  block-gnss-id i/int -> int:
    return LITTLE-ENDIAN.uint8 payload (4 + 8*i)

  block-flags i/int -> int:
    return LITTLE-ENDIAN.uint32 payload (4 + 8*i + 4)

  id-string_ -> string: return "GNSS"
