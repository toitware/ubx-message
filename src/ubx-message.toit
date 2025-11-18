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
*/

/*
To do list:
- MGA-* (AssistNow) messages: Assisted GNSS injection (time, eph/almanac) for
  fast TTFF.  A path for MGA-INI-TIME_UTC at minimum.
- ESF-* for combination with DR/ADR/IMU fusion.
- CFG-TP5: Complete so setters and getters match, and avoid PROTVER15-16
  differences.
*/

import io
import io show LITTLE-ENDIAN
import reader as old-reader

/**
A UBX message from the UBX data protocol.
*/
class Message:
  /** Maximum size of an encoded message. */
  /*
  The protocol allows length up to 65535 (2-byte field), though most real
    messages are far smaller. 2 KB is sensible safety cap, but some messages
    (for example, MGA assistance blocks, large MON dumps) can exceed that on
    newer firmware.  May need to set this differently later, possibly 8K/16K.
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
    6M and M8 manuals have been added.  This is to help where an information
    message presents itself that may not yet be implemented.  Implemented as
    nested Maps.
  */
  static PACK-MESSAGE-TYPES := {
    // NAV (0x01).
    NAV: {
      0x01: "POSECEF",
      0x02: "POSLLH",
      0x03: "STATUS",
      0x04: "DOP",
      0x05: "ATT",        // M8+.
      0x06: "SOL",
      0x07: "PVT",        // M8+.
      0x09: "ODO",        // M8+.
      0x10: "RESETODO",   // M8+.
      0x11: "VELECEF",
      0x12: "VELNED",
      0x13: "HPPOSECEF",  // M8+.
      0x14: "HPPOSLLH",   // M8+.
      0x20: "TIMEGPS",
      0x21: "TIMEUTC",
      0x23: "TIMEGLO",    // M8+.
      0x24: "TIMEBDS",    // M8+.
      0x25: "TIMEGAL",    // M8+.
      0x26: "TIMELS",     // M8+.
      0x28: "NMI",        // M8+.
      0x30: "SVINFO",
      0x31: "DGPS",
      0x32: "SBAS",
      0x35: "SAT",        // M8+.
      0x39: "GEOFENCE",   // M8+.
      0x3B: "SVIN",       // M8+.
      0x3C: "RELPOSNED",  // M8+.
      0x3D: "EELL",       // M8+.
      0x42: "SLAS",       // M8+.
      0x60: "AOPSTATUS",
      0x61: "EOE",        // M8+.
    },

    // RXM (0x02).
    RXM: {
      0x10: "RAW",     // 6-series.
      0x11: "SFRB",    // 6-series.
      0x13: "SFRBX",   // M8+.
      0x14: "MEASX",   // M8+.
      0x15: "RAWX",    // M8+.
      0x20: "SVSI",
      0x30: "ALM",
      0x31: "EPH",
      0x32: "RTCM",    // M8+.
      0x41: "PMREQ",
      0x59: "RLM",     // M8+.
      0x61: "IMES",    // M8+.
    },

    // ACK (0x05).
    ACK: {
      0x00: "ACK-NAK",
      0x01: "ACK-ACK",
    },

    // CFG (0x06).
    CFG: {
      0x00: "PRT",
      0x01: "MSG",
      0x02: "INF",
      0x04: "RST",
      0x06: "DAT",
      0x07: "TP",
      0x08: "RATE",
      0x09: "CFG",
      0x0E: "FXN",       // 6-series.
      0x11: "RXM",
      0x12: "EKF",       // 6-series LEA-6R.
      0x13: "ANT",
      0x16: "SBAS",
      0x17: "NMEA",
      0x1B: "USB",
      0x1D: "TMODE",     // 6-series.
      0x1E: "ODO",
      0x23: "NAVX5",
      0x24: "NAV5",
      0x29: "ESFGWT",    // 6-series (LEA-6R).
      0x31: "TP5",
      0x34: "RINV",
      0x39: "ITFM",
      0x3B: "PM2",
      0x3D: "TMODE2",
      0x3E: "GNSS",      // M8+.
      0x47: "LOGFILTER", // M8+.
      0x53: "TXSLOT",    // M8+.
      0x56: "ESFALG",    // M8+.
      0x57: "PWR",       // M8+.
      0x5C: "HNR",       // M8+.
      0x60: "ESRC",      // M8+.
      0x61: "DOSC",      // M8+.
      0x62: "SMGR",      // M8+.
      0x64: "SPT",       // M8+.
      0x69: "GEOFENCE",  // M8+.
      0x70: "DGNSS",     // M8+.
      0x71: "TMODE3",    // M8+.
      0x82: "ESFWT",     // M8+.
      0x86: "PMS",       // M8+.
      0x88: "SENIF",     // M8+.
      0x8D: "SLAS",      // M8+.
      0x93: "BATCH",     // M8+.
    },

    // MON (0x0A).
    MON: {
      0x02: "IO",
      0x04: "VER",
      0x06: "MSGPP",
      0x07: "RXBUF",
      0x08: "TXBUF",
      0x09: "HW",
      0x0B: "HW2",
      0x21: "RXR",
      0x27: "PATCH",     // M8+.
      0x28: "GNSS",      // M8+.
      0x2E: "SMGR",      // M8+.
      0x2F: "SPT",       // M8+.
      0x32: "BATCH",     // M8+.
    },

    // AID (0x0B) — legacy assistance (6 & M8).
    AID: {
      0x01: "INI",
      0x02: "HUI",
      0x30: "ALM",
      0x31: "EPH",
      0x33: "AOP",
      0x50: "ALP",   // 6-series.
      0x10: "DATA",  // 6-series.
      0x32: "ALPSRV" // 6-series.
    },

    // TIM (0x0D).
    TIM: {
      0x01: "TP",
      0x03: "TM2",
      0x04: "SVIN",
      0x06: "VRFY",    // 6-series.
      0x11: "DOSC",    // M8+.
      0x12: "TOS",     // M8+.
      0x13: "SMEAS",   // M8+.
      0x15: "VCOCAL",  // M8+.
      0x16: "FCHG",    // M8+.
      0x17: "HOC",     // M8+.
    },

    // ESF (0x10) — external sensor fusion.
    ESF: {
      0x02: "MEAS",  // 6-series LEA-6R / M8 ESF-MEAS (different payloads).
      0x03: "RAW",   // M8+.
      0x10: "STATUS",// 6-series LEA-6R / M8 ESF-STATUS.
      0x14: "ALG",   // M8+.
      0x15: "INS",   // M8+.
    },

    // MGA (0x13) — M8+ multi-GNSS assistance (index only; many subtypes).
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

    // LOG (0x21) — M8+.
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

    // SEC (0x27) — M8+.
    SEC: {
      0x03: "SEC-UNIQID",
    },

    // HNR (0x28) — M8+.
    HNR: {
      0x00: "PVT",
      0x01: "ATT",
      0x02: "INS",
    },

  }

  static INVALID-UBX-MESSAGE_ ::= "INVALID UBX MESSAGE"
  static RESERVED_ ::= 0

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  min-protver/string := "15.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  max-protver/string := ""

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
    checksum).
  */
  constructor.from-bytes bytes/ByteArray:
    if not is-valid-frame_ bytes: throw INVALID-UBX-MESSAGE_
    cls = bytes[2]
    id = bytes[3]
    length := LITTLE-ENDIAN.uint16 bytes 4
    if bytes.size != length + 8: throw INVALID-UBX-MESSAGE_
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
    length ::= (io-reader.peek-byte 4) | (((io-reader.peek-byte 5) & 0xff) << 8)
    if not 0 <= length <= MAX-MESSAGE-SIZE_: throw INVALID-UBX-MESSAGE_
    frame ::= io-reader.peek-bytes length + 8

    // Verify the checksum.
    if not is-valid-frame_ frame: throw INVALID-UBX-MESSAGE_

    msg-class ::= frame[2]
    msg-id    ::= frame[3]
    payload   ::= frame[6..length + 6]
    io-reader.skip length + 8
    return Message msg-class msg-id payload

  // Checks frame is valid - lets callers determine what to do if it fails.
  static is-valid-frame_ frame/ByteArray -> bool:
    // Check the sync bytes.
    if frame[0] != 0xb5 or frame[1] != 0x62: return false

    // Check the payload length.
    length ::= LITTLE-ENDIAN.uint16 frame 4
    if not 0 <= length <= MAX-MESSAGE-SIZE_: return false
    if frame.size != length + 8: return false

    ck-a ::= frame[frame.size - 2]
    ck-b ::= frame[frame.size - 1]

    compute-checksum_ frame: | a b |
      return ck-a == a and ck-b == b
    unreachable

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
    //return #[cls, id]
    // hash-code needs to be an integer.  By these numbers we could assume
    // cls << 8 would be ok, but I want to be sure:
    return (cls << 16) | id

  /** Helper to return uint8 from payload index. */
  int8_ index --payload=payload -> int: return LITTLE-ENDIAN.int8 payload index

  /** Helper to return uint8 from payload index. */
  uint8_ index --payload=payload -> int: return payload[index]

  /** Helper to return int16 from payload index. */
  int16_ index --payload=payload -> int: return LITTLE-ENDIAN.int16 payload index

  /** Helper to return uint16 from payload index. */
  uint16_ index --payload=payload -> int: return LITTLE-ENDIAN.uint16 payload index

  /** Helper to return int32 from payload index. */
  int32_ index --payload=payload -> int: return LITTLE-ENDIAN.int32 payload index

  /** Helper to return uint32 from payload index. */
  uint32_ index --payload=payload -> int: return LITTLE-ENDIAN.uint32 payload index


  /** Helper to insert int8 into payload index. */
  put-int8_ index value --payload=payload -> none:
    LITTLE-ENDIAN.put-int8 payload index value

  /** Helper to insert uint8 into payload index. */
  put-uint8_ index value --payload=payload -> none:
    payload[index] = value

  /** Helper to insert int16 into payload index. */
  put-int16_ index value --payload=payload -> none:
    LITTLE-ENDIAN.put-int16 payload index value

  /** Helper to insert uint16 into payload index. */
  put-uint16_ index value --payload=payload -> none:
    LITTLE-ENDIAN.put-uint16 payload index value

  /** Helper to insert int32 into payload index. */
  put-int32_ index value --payload=payload -> none:
    LITTLE-ENDIAN.put-int32 payload index value

  /** Helper to insert uint32 into payload index. */
  put-uint32_ index value --payload=payload -> none:
    LITTLE-ENDIAN.put-uint32 payload index value

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

  id-string_ -> string:
    return "ACK"

  /** The class ID of the acknowledged message. */
  class-id -> int:
    return uint8_ 0

  /** The class ID (converted to text) of the acknowledged message. */
  class-id-text -> string:
    return Message.PACK-CLASSES[class-id]

  /** The message ID of the acknowledged message. */
  message-id -> int:
    return uint8_ 1

  /** The message ID (converted to text, if known) of the acknowledged message. */
  message-id-text -> string:
    output := ""
    if Message.PACK-MESSAGE-TYPES.contains class-id:
      if Message.PACK-MESSAGE-TYPES[class-id].contains message-id:
        output = Message.PACK-MESSAGE-TYPES[class-id][message-id]
    return output

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

  id-string_ -> string:
    return "NAK"

  /** The class ID of the NAK message. */
  class-id -> int:
    return uint8_ 0

  /** The class ID (converted to text) of the negative-acknowledge message. */
  class-id-text -> string:
    return Message.PACK-CLASSES[class-id]

  /** The message ID of the NAK message. */
  message-id -> int:
    return uint8_ 1

  /** The message ID (converted to text, if known) of the acknowledged message. */
  message-id-text -> string:
    output := ""
    if Message.PACK-MESSAGE-TYPES.contains class-id:
      if Message.PACK-MESSAGE-TYPES[class-id].contains message-id:
        output = Message.PACK-MESSAGE-TYPES[class-id][message-id]
    return output

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
*/
/*
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

  min-protver/string := "15.0"
  max-protver/string := "23.0"   // Manual says not after this version.

  // Common constants (see u-blox docs).
  static PORT-UART1 ::= 0x01
  static PORT-UART2 ::= 0x02

  // Todo: expose these on the constructor.
  // mode bitfield shortcut: 8 data bits, no parity, 1 stop (8N1).
  // (charLen=3 -> bits 6..7 = 0b11; parity=0 -> bits 9..11 = 0; nStop=1 -> bit 12 = 0).
  // u-blox ref value: 0x000008D0.
  static MODE-DATA-BITS-MASK_ := 0b00000000_01100000
  static MODE-PARITY-MASK_    := 0b00000111_00000000
  static MODE-STOP-BITS-MASK_ := 0b00011000_00000000

  // Common Mode Presets.
  static MODE-8N1 ::= 0x000008D0
  static MODE-7E1 ::= 0x00000080
  static MODE-8O2 :=  0x000000C0

  // Protocol mask bits (legacy).
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

    // PortID, Reserved0, TxReady(2).
    put-uint8_ 0 port-id
    put-uint8_ 1 0
    put-uint16_ 2 0     // txReady off.

    // Mode (framing).
    put-uint32_ 4 mode

    // BaudRate.
    put-uint32_ 8 baud

    // In/out proto masks.
    put-uint16_ 12 in-proto
    put-uint16_ 14 out-proto

    // Flags, reserved1.
    put-uint16_ 16 flags
    put-uint16_ 18 0

  /**
  Poll the configuration for a given port.
  The poll payload is a single byte: portID.
  */
  constructor.poll --port-id/int=CfgPrt.PORT-UART1:
    super.private_ Message.CFG ID #[port-id]

  /** Construct from an incoming payload. */
  constructor.private_ payload/ByteArray:
    super.private_ Message.CFG ID payload

  id-string_ -> string:
    return "PRT"

  /**
  Ublox internal port ID.

  Depending on the device, there can be more than 1 UART, as well as DDC (I2C
    compatible) USB and SPI types.  See the ublox Integration Manual for all
    valid UART port IDs.
  */
  port-id -> int:
    return uint8_ 0

  mode -> int:
    return uint32_ 4

  baud-rate -> int:
    return uint32_ 8

  in-proto-mask -> int:
    return uint16_ 12

  out-proto-mask -> int:
    return uint16_ 14

  flags -> int:
    return uint16_ 16

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
    put-uint16_ 0 clear-sections
    put-uint8_ 2 reset-mode
    put-uint8_ 3 Message.RESERVED_

  id-string_ -> string:
    return "RST"

/**
The UBX-NAV-STATUS message.

The receiver navigation status.
*/
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
    TIME-ONLY : "TIME-ONLY",
  }

  constructor.poll:
    super.private_ Message.NAV ID #[]

  constructor.private_ payload:
    super.private_ Message.NAV ID payload

  id-string_ -> string:
    return "STATUS"

  /** The GPS interval time of week of the navigation epoch. */
  itow -> int:
    assert: not payload.is-empty
    return uint32_ 0

  /**
  Returns the current fix type.

  One of $NO-FIX, $DEAD-RECKONING-ONLY, $FIX-2D, $FIX-3D, $GPS-DEAD-FIX, $TIME-ONLY.
  */
  gps-fix -> int:
    assert: not payload.is-empty
    //return uint8_ 4
    return uint8_ 4

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
    return uint8_ 5

  /**
  Fix status information.
  See receiver specification for details.
  */
  fix-stat -> int:
    assert: not payload.is-empty
    return uint8_ 6

  /**
  Additional status information.
  See receiver specification for details.
  */
  flags2 -> int:
    assert: not payload.is-empty
    return uint8_ 7

  /**
  Time to first fix in milliseconds.

  Take care when deciding what to do with this value. It is not a live indicator
  of current signal quality, and the value is not kept up to date or changed
  when the fix is lost.  It is not a countdown timer for the next expected fix.
  It is simply a historical value about the most recent acquisition event.
  */
  time-to-first-fix -> int:
    assert: not payload.is-empty
    return uint32_ 8

  /** Milliseconds since startup or reset. (msss) */
  ms-since-startup -> int:
    assert: not payload.is-empty
    return uint32_ 12

/**
The UBX-NAV-SAT message.

Satellite information.
*/
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
    return uint32_ 0

  /** Message version. */
  version -> int:
    assert: not payload.is-empty
    return uint8_ 4

  /** Number of satellites in the message. */
  num-svs -> int:
    assert: not payload.is-empty
    return uint8_ 5

  /** Number of satellites in the message. */
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

Satellite data can be provided via UBX-NAV-SAT and/or UBX-NAV-SVINFO messages
  (See $NavSat and $NavSvInfo).  This class is a container for satellite
  properties.  (Satellites are referred to in documentation as 'Space Vehicles'
  or 'SV', and the terms are used interchangably.)  It stores/parses properties
  common to both UBX-NAV-SVINFO and UBX-NAV-SAT message types.  Messages are
  approximately the same length in both cases, but have different information
  and layout depending on which message was the source.
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

  /**
  Space Vehicle health indicator.

  For compatibility: 0=unknown; 1=healthy; 2=unhealthy.  UBX-NAV-SVINFO would
  normally have 0=healthy and 1=unhealthy, but these values have been shifted to
  match results from UBX-NAV-SAT.
  */
  health/int

  /**
  Flags.

  Includes $quality, $orbit-source, $alm-avail, and $ano-avail.  See
    receiver specification for details.
  */
  flags/int

  /**
  Signal quality indicator.

  ```
  Signal quality values:
  - 0: no signal
  - 1: searching signal
  - 2: signal acquired
  - 3: signal detected but unusable
  - 4: code locked and time synchronized
  - 5, 6, 7: code and carrier locked and time synchronized

  Note: Since IMES signals are not time synchronized, a channel tracking an IMES
    signal can never reach a quality indicator value of higher than 3.
  ```
  */
  quality/int

  /**
  Orbit source.

  ```
  Field Definitions (M8)
  - 0: no orbit information is available for this SV
  - 1: ephemeris is used
  - 2: almanac is used
  - 3: AssistNow Offline orbit is used
  - 4: AssistNow Autonomous orbit is used
  - 5, 6, 7: other orbit information is used
  ```
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
      gnss-id = LITTLE-ENDIAN.uint8 payload (offset + 8)

      sv-id = LITTLE-ENDIAN.uint8 payload (offset + 9)
      cno = LITTLE-ENDIAN.uint8 payload (offset + 10)
      elev = LITTLE-ENDIAN.int8 payload (offset + 11)
      azim = LITTLE-ENDIAN.int16 payload (offset + 12)
      pr-res = (LITTLE-ENDIAN.uint8 payload offset + 14) / 10.0 // scale 0.1
      flags = LITTLE-ENDIAN.uint32 payload (offset + 16)

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

      orbit-info-avail = (eph-avail != 0) or (alm-avail != 0) or (ano-avail != 0) or (aop-avail != 0)

    else if src-id == NavSvInfo.ID:
      offset = index * 12
      channel = LITTLE-ENDIAN.uint8 payload offset + 8

      sv-id = LITTLE-ENDIAN.uint8 payload offset + 9
      cno = LITTLE-ENDIAN.uint8 payload offset + 12
      elev = LITTLE-ENDIAN.int8 payload offset + 13
      azim = LITTLE-ENDIAN.int16 payload offset + 14
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

      // In the case of NavSvInfo messages, there are only two possibl statuses.
      // In the case of NavSat, there are 3 possibilities. This binary output is
      // moved (+1) to convert its outputs to match NavSat definitions.
      unhealthy-raw  := (flags & unhealthy-mask) >> unhealthy-mask.count-trailing-zeros
      health = unhealthy-raw + 1

      // Translated to return outputs as close to the M8 definition as possible
      if ((flags & orbit-eph-mask) >> orbit-eph-mask.count-trailing-zeros) == 1:
        orbit-source = 1
      else if ((flags & orbit-alm-mask) >> orbit-alm-mask.count-trailing-zeros) == 1:
        orbit-source = 2
      else if ((flags & orbit-aop-mask) >> orbit-aop-mask.count-trailing-zeros) == 1:
        orbit-source = 4

    else:
      throw "Unknown Space Vehicle Definition Source"

  /** Helper to return uint8 from payload index. */
  int8_ payload index -> int: return LITTLE-ENDIAN.int8 payload index

  /** Helper to return uint8 from payload index. */
  uint8_ payload index -> int: return payload[index]

  /** Helper to return int16 from payload index. */
  int16_ payload index -> int: return LITTLE-ENDIAN.int16 payload index

  /** Helper to return uint16 from payload index. */
  uint16_ payload index -> int: return LITTLE-ENDIAN.uint16 payload index

  /** Helper to return int32 from payload index. */
  int32_ payload index -> int: return LITTLE-ENDIAN.int32 payload index

  /** Helper to return uint32 from payload index. */
  uint32_ payload index -> int: return LITTLE-ENDIAN.uint32 payload index

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

  /** Software version string. */
  // Null terminated with fixed field size of 30 bytes.
  sw-version -> string:
    return convert-string_ 0 30

  /** Hardware version string. */
  // Null terminated with fixed field size of 10 bytes.
  hw-version -> string:
    return convert-string_ 30 10

  /** Returns true if an extension row exists containing the supplied string. */
  has-extension str/string -> bool:
    return extensions-raw.any: it.contains str

  /**
  The entire line of the extension with the given $str.

  Null if this message doesn't have the extension (see $has-extension).
  */
  extension str/string -> string?:
    extensions-raw.any:
      if (it.index-of str) > -1:
        return it
    return null

  /*
  A map of extension string AVPs.

  This function returns a map of strings with the keyed by the first part, with
    the value being the remainder past the first instance of "=".

  DISABLED: Originally it looked like the extensions would be AVPs delimited by
    '='.  This is not true in all device generations. In addition, the sets of
    tags shown in the extensions aren't grouped, and can be split across more
    than one extension 'row'.  So for now, this is disabled in favor of the
    driver parsing these messages.

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
  A list of extension strings, if present.

  If provided by the firmware version on the device, it provides a list of n 30
    byte entries.  Each entry is a NUL-terminated ASCII string.
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
    while (end < limit) and (uint8_ end) != 0:
      end++

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
    return uint32_ 0

  /** longitude. (1e-7 degrees.) */
  longitude-raw -> int:
    return int32_ 4

  /** Latitude. (1e-7 degrees.) */
  latitude-raw    -> int:
    return int32_ 8

  /** Height above ellipsoid. */
  height-mm  -> int:
    return int32_ 12

  /** Height above mean sea level. */
  height-msl-mm   -> int:
    return int32_ 16

  /** Horizontal measurement accuracy estimate. */
  horizontal-accuracy-mm   -> int:
    return uint32_ 20

  /** Vertical measurement accuracy estimate. */
  vertical-accuracy-mm   -> int:
    return uint32_ 24

  // Convenience in float degrees
  longitude-deg -> float:
    return longitude-raw / 1e7

  // Convenience in float degrees
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
    return uint32_ 0

  /** Number of channels. */
  num-ch     -> int:
    return uint8_ 4

  /** Global flags bitmask.

  ```
  Mask 0b00000111 contains a number representing chip hardware generation:
   - 0: Antaris, Antaris 4
   - 1: u-blox 5
   - 2: u-blox 6
   - 3: u-blox 7
   - 4: u-blox 8 / u-blox M8
  ```
  */
  global-flags -> int:
    return uint8_ 5

  /**
  How many satellites in the message.

  Introduced for compatibility with NavSat.
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
    return uint32_ 0

  /** The year (UTC). */
  year -> int:
    assert: not payload.is-empty
    return uint16_ 4

  /**
  The month (UTC).
  In the range [1..12].
  */
  month -> int:
    assert: not payload.is-empty
    return uint8_ 6

  /**
  The day (UTC).
  In the range [1..31].
  */
  day -> int:
    assert: not payload.is-empty
    return uint8_ 7

  /**
  The hours (UTC).
  In the range [0..23].
  */
  h -> int:
    assert: not payload.is-empty
    return uint8_ 8

  /**
  The minutes (UTC).
  In the range [0..59].
  */
  m -> int:
    assert: not payload.is-empty
    return uint8_ 9

  /**
  The seconds (UTC).
  In the range [0..60].
  */
  s -> int:
    assert: not payload.is-empty
    return uint8_ 10

  /**
  Validity flag.
  See receiver specification for details.
  */
  valid -> int:
    assert: not payload.is-empty
    return uint8_ 11

  /** Time accuracy estimate in nanoseconds */
  time-acc -> int:
    assert: not payload.is-empty
    return uint32_ 12

  /**
  Fraction of second in nano seconds.
  The fraction may be negative.
  */
  ns -> int:
    assert: not payload.is-empty
    return int32_ 16

  /**
  The type of fix.
  One of $FIX-TYPE-UNKNOWN, $FIX-TYPE-DEAD, $FIX-TYPE-2D, $FIX-TYPE-3D, $FIX-TYPE-GNSS-DEAD, $FIX-TYPE-TIME-ONLY.
  */
  fix-type -> int:
    assert: not payload.is-empty
    return uint8_ 20

  /**
  Fix status flags.
  See receiver specification for details.
  */
  flags -> int:
    assert: not payload.is-empty
    return uint8_ 21

  /**
  Additional fix status flags.
  See receiver specification for details.
  */
  flags2 -> int:
    assert: not payload.is-empty
    return uint8_ 22

  /** Number of satellites used for fix. */
  num-sv -> int:
    assert: not payload.is-empty
    return uint8_ 23

  /** Longitude. */
  lon -> int:
    assert: not payload.is-empty
    return int32_ 24

  /** Latitude. */
  lat -> int:
    assert: not payload.is-empty
    return int32_ 28

  /** Height above ellipsoid in millimeter. */
  height -> int:
    assert: not payload.is-empty
    return int32_ 32

  /** Height above mean sea level in millimeter. */
  height-msl -> int:
    assert: not payload.is-empty
    return int32_ 36

  /** Horizontal accuracy in millimeter. */
  horizontal-acc -> int:
    assert: not payload.is-empty
    return uint32_ 40

  /** Vertical accuracy in millimeter. */
  vertical-acc -> int:
    assert: not payload.is-empty
    return uint32_ 44

  /** NED north velocity in millimeters per second. */
  north-vel -> int:
    assert: not payload.is-empty
    return int32_ 48

  /** NED east velocity in millimeters per second. */
  east-vel -> int:
    assert: not payload.is-empty
    return int32_ 52

  /** NED down velocity in millimeters per second. */
  down-vel -> int:
    assert: not payload.is-empty
    return int32_ 56

  /** Ground speed (2D) in millimeters per second. */
  ground-speed -> int:
    assert: not payload.is-empty
    return int32_ 60

  /** Heading of motion (2D). */
  heading-of-motion -> int:
    assert: not payload.is-empty
    return int32_ 64

  /** Speed accuracy in millimeters per second. */
  speed-acc -> int:
    assert: not payload.is-empty
    return uint32_ 68

  /** Heading accuracy. */
  heading-acc -> int:
    assert: not payload.is-empty
    return uint32_ 72

  /**
  Position DOP.

  Position 'Dilution of Position' scale.  Scale 0.01.
  */
  position-dop -> float:
    assert: not payload.is-empty
    return (uint16_ 76).to-float / 100

  /**
  Additional flags.
  See receiver specification for details.
  */
  flags3 -> int:
    assert: not payload.is-empty
    return uint32_ 78

  /**
  The heading of the vehicle.
  See receiver specification for details.
  */
  heading-vehicle -> int:
    assert: not payload.is-empty
    return int32_ 84

  /**
  Magnetic declination.
  See receiver specification for details.
  */
  magnetic-declination -> int:
    assert: not payload.is-empty
    return int16_ 88

  /**
  Accuracy of magnetic declination.
  See receiver specification for details.
  */
  magnetic-acc -> int:
    assert: not payload.is-empty
    return uint16_ 90


/**
The UBX-NAV-SOL message.

Legacy Navigation solution, in ECEF (Earth-Centered, Earth-Fixed cartesian
  coordinates).  This message is included for backwards compatibility.  Whilst
  it is available on M8 and later, UBX-NAV-PVT messages are preferred).
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

  /* The time in UTC.

  Time is not included in this legacy Message type, although it can be obtained
    using other message types.  Need to think what to do with this one.  Take out
    for both, or leave in?  Perhaps remove custom properties (made for human
    consumption) on the message types, and have the driver worry about
    presentation of message data...

  utc-time -> Time:
    return Time.utc year month day h m s --ns=ns
  */

  /** The GPS interval time of week of the navigation epoch. */
  itow -> int:
    assert: not payload.is-empty
    return uint32_ 0

  /**
  The fractional GPS interval time of week (in ns) of the navigation epoch.

  Range in ns: -500000..+500000
  */
  ftow -> int:
    assert: not payload.is-empty
    return int32_ 4

  /**
  The GPS Week number.

  This is the week number since Jan 6 1980.
  */
  week -> int:
    assert: not payload.is-empty
    return int16_ 8

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
  Whether DGPS is used.  (diffSoln)
  */
  dgps-used -> bool:
    dgps-used-mask := 0b00000010
    return ((flags & dgps-used-mask) >> dgps-used-mask.count-trailing-zeros) != 0

  /**
  The type of fix.

  One of $NavSol.FIX-TYPE-UNKNOWN, $NavSol.FIX-TYPE-DEAD, $NavSol.FIX-TYPE-2D,
    $NavSol.FIX-TYPE-3D, $NavSol.FIX-TYPE-GNSS-DEAD, $NavSol.FIX-TYPE-TIME-ONLY.
  */
  fix-type -> int:
    assert: not payload.is-empty
    return uint8_ 10

  /**
  Fix status flags.

  See receiver specification for details.
  */
  flags -> int:
    assert: not payload.is-empty
    return uint8_ 11

  /**
  Number of satellites used for fix.
  */
  num-sv -> int:
    assert: not payload.is-empty
    return uint8_ 47

  /**
  Position DOP.

  Position 'Dilution of Position' scale.  Scale 0.01.
  */
  position-dop -> float:
    assert: not payload.is-empty
    return (uint16_ 44).to-float / 100

  /** ECEF X coordinate. [cm] */
  ecef-x-cm  -> int: return int32_ 12      // I4 cm.

  /** ECEF Y coordinate. [cm] */
  ecef-y-cm  -> int: return int32_ 16      // I4 cm.

  /** ECEF Z coordinate. [cm] */
  ecef-z-cm  -> int: return int32_ 20      // I4 cm.

  /** 3D Position Accuracy Estimate. [cm] */
  p-acc-cm   -> int: return uint32_ 24      // U4 cm.

  /** ECEF X velocity. [cm/s] */
  ecef-vx-cms -> int: return int32_ 28      // I4 cm/s.

  /** ECEF Y velocity. [cm/s] */
  ecef-vy-cms -> int: return int32_ 32      // I4 cm/s.

  /** ECEF Z velocity. [cm/s] */
  ecef-vz-cms -> int: return int32_ 36      // I4 cm/s.

  /** Speed Accuracy Estimate. [cm/s] */
  s-acc-cms   -> int: return uint32_ 40      // U4 cm/s.

  /** Reserved 1. */
  reserved1  -> int: return uint8_ 46      // U1.

  /** Reserved 2. */
  reserved2  -> int: return uint32_ 48      // U4 (M8 doc shows U1[4]; same 4 bytes).

/**
The UBX-NAV-TIMEUTC message.

UTC time solution.  Functions on 6M and later devices.
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
    return uint32_ 0

  /** The time in UTC. */
  utc-time -> Time:
    assert: not payload.is-empty
    return Time.utc year month day h m s --ns=ns

  /** UTC Time accuracy estimate, in nanoseconds. */
  time-acc -> int:
    assert: not payload.is-empty
    return uint32_ 4

  ns -> int:
    assert: not payload.is-empty
    return int32_ 8

  year -> int:
    assert: not payload.is-empty
    return uint16_ 12

  month -> int:
    assert: not payload.is-empty
    return uint8_ 14

  day -> int:
    assert: not payload.is-empty
    return uint8_ 15

  h -> int:
    assert: not payload.is-empty
    return uint8_ 16

  m -> int:
    assert: not payload.is-empty
    return uint8_ 17

  /**
  Return UTC Seconds.

  Normally 00..59, but leap seconds can produce between 59 to 61 seconds.  The
    uBlox manual states "u-blox receivers are designed to handle leap seconds in
    their UTC output and consequently users processing UTC times from either
    NMEA and UBX messages should be prepared to handle minutes that are either
    59 or 61 seconds long." (Section 9.7 "Leap Seconds" - ublox 8 Datasheet, pp
    27.)
  */
  s -> int:
    return uint8_ 18

  /**
  Validity of time flags.

  M8+: upper bits carry UTC standard
  */
  valid-flags-raw -> int:
    return uint8_ 19

  /**
  Returns if GPS Week number is Valid. (ValidWKN)
  */
  valid-week -> bool:
    week-valid-mask := 0b00000010
    return ((valid-flags-raw & week-valid-mask) >> week-valid-mask.count-trailing-zeros) != 0

  /**
  Returns if GPS Time of Week number is Valid. (ValidTOW)
  */
  valid-time-of-week -> bool:
    time-of-week-valid-mask := 0b00000001
    return ((valid-flags-raw & time-of-week-valid-mask) >> time-of-week-valid-mask.count-trailing-zeros) != 0

  /**
  Returns if UTC time is valid. (ValidUTC - If the leap seconds are known.)
  */
  valid-utc -> bool:
    valid-utc-mask := 0b00000100
    return ((valid-flags-raw & valid-utc-mask) >> valid-utc-mask.count-trailing-zeros) != 0

  /**
  Returns UTC standard code.

  Returns 0 on Legacy. Common values:
  ```
  - 0: Information not available.
  - 1: Communications Research Labratory (CRL), Tokyo, Japan.
  - 2: National Institute of Standards and Technology (NIST).
  - 3: U.S. Naval Observatory (USNO).
  - 4: International Bureau of Weights and Measures (BIPM).
  - 5: European laboratories.
  - 6: Former Soviet Union (SU).
  - 7: National Time Service Center (NTSC), China.
  - 8: National Physics Laboratory India (NPLI).
  - 15: Unknown.
  ```
  */
  utc-standard -> int:
    return (valid-flags-raw >> 4) & 0x0F

/**
The UBX-CFG-TP5 message.

Used to configure the pulse signal on the TIMEPULSE/PPS pin, used for time
  synchronisation.  The message controls parameters like the pulse's period
  and duty cycle.

Parameters and flags are different starting from Protocol version 16.
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
    super.private_ Message.CFG ID new-payload
    put-uint8_ 0 tp-idx
    put-uint8_ 1 1  // Version.

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
    super.private_ Message.CFG ID new-payload

    put-uint8_ 0 tp-idx
    put-uint8_ 1 1  // Version.
    put-uint16_ 2 0
    put-int16_ 4 ant-cable-ns
    put-int16_ 6 rf-group-ns

    // Frequency mode: set freqPeriod=freq, isFreq=1; pulseLenRatio = duty * 1e-9.
    put-uint32_ 8 freq-hz
    put-uint32_ 12 freq-hz

    duty-ratio-nano := duty-permille * 1_000_000  // permille -> nanos of 1e9.
    put-uint32_ 16 duty-ratio-nano
    put-uint32_ 20 duty-ratio-nano
    put-int32_ 24 0

    flags := 0
    if active: flags |= FLAG-ACTIVE
    flags |= FLAG-IS-FREQ // frequency mode
    // we used ratio (not length), so FLAG-IS-LENGTH stays 0
    flags |= FLAG-ALIGN-TOW
    if polarity-high: flags |= FLAG-POLARITY-HI
    if use-utc: flags |= FLAG-UTC-GRID
    put-uint32_ 28 flags



  id-string_ -> string:
    return "TP5"

  /**
  Time pulse selection.

  CfgTp5.TP-IDX-0=TIMEPULSE, CfgTp5.TP-IDX-1=TIMEPULSE2
  */
  tp-idx -> int:
    return uint8_ 0

  /**
  Configuration flags.

  Parameters and flags are different starting from Protocol version 16.
  */
  flags -> int:
    return uint32_ 28

  freq -> int:
    return uint32_ 8

  duty-nano -> int:
    return uint32_ 16




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

  // Mask bits (subset).
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

  // Dynamic models (subset).
  static DYN-PORTABLE   ::= 0
  static DYN-STATIONARY ::= 2
  static DYN-PEDESTRIAN ::= 3
  static DYN-AUTOMOTIVE ::= 4
  static DYN-SEA        ::= 6
  static DYN-AIR1G      ::= 7
  static DYN-AIR2G      ::= 8
  static DYN-AIR4G      ::= 9

  // Fix mode.
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
    super.private_ Message.CFG ID #[]

  /** Construct an instance with bytes from a retrieved message. */
  constructor.private_ payload/ByteArray:
    super.private_ Message.CFG ID payload

  /** Minimal setter: set dyn model + auto 2D/3D, leave others default. */
  constructor.set-basic
      --dyn/int=DYN-AUTOMOTIVE
      --fix/int=FIX-AUTO:

    // Sensible defaults / zeros elsewhere.
    new-payload := ByteArray 36
    super.private_ Message.CFG ID new-payload
    put-uint16_ 0 (DYN-MASK_ | FIXMODE-MASK_)
    put-uint8_ 2 dyn
    put-uint8_ 3 fix


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
    super.private_ Message.CFG ID new-payload

    mask := 0
    if dyn:
      mask |= DYN-MASK_
      put-uint8_ 2 dyn
    if fix:
      mask |= FIXMODE-MASK_
      put-uint8_ 3 fix
    if fixed-alt-cm:
      mask |= ALT-MASK_
      put-int32_ 4 fixed-alt-cm
    if fixed-alt-var-cm2:
      mask |= ALT-MASK_
      put-uint32_ 8 fixed-alt-var-cm2
    if min-elev-deg:
      mask |= OUTLYING-MASK_
      put-int8_ 12 min-elev-deg
    if dr-limit-s:
      mask |= OUTLYING-MASK_
      put-uint8_ 13 dr-limit-s
    if p-dop-x10:
      mask |= PDOP-MASK_
      put-uint16_ 14 p-dop-x10
    if t-dop-x10:
      mask |= TDOP-MASK_
      put-uint16_ 16 t-dop-x10
    if p-acc-m:
      mask |= PACC-MASK_
      put-uint16_ 18 p-acc-m
    if t-acc-m:
      mask |= TACC-MASK_
      put-uint16_ 20 t-acc-m
    if static-hold-thresh-cmps:
      mask |= STATIC-MASK_
      put-uint8_ 22 static-hold-thresh-cmps
    if dgnss-timeout-s:
      mask |= DGPS-MASK_
      put-uint8_ 23 dgnss-timeout-s
    if cno-thresh-num-sv:
      mask |= OUTLYING-MASK_
      put-uint8_ 24 cno-thresh-num-sv
    if cno-thresh-dbHz:
      mask |= OUTLYING-MASK_
      put-uint8_ 25 cno-thresh-dbHz
    if static-hold-max-dist-m:
      mask |= STATIC-MASK_
      put-uint16_ 28 static-hold-max-dist-m
    if utc-standard:
      mask |= UTC-MASK_
      put-uint8_ 30 utc-standard

    put-uint16_ 0 mask


  id-string_ -> string:
    return "NAV5"

  dyn-model -> int:
    return uint8_ 2

  dyn-model-text -> string:
    return PACK-MODELS[dyn-model]

  fix-mode -> int:
    return uint8_ 3

  mask -> int:
    return uint16_ 0

/**
The UBX-CFG-GNSS message.

Configuring constellations/signals.  Note: Signal bitmasks inside flags are
  chip-family specific (M8 vs M9/M10).  Keeping signals-mask=0 lets firmware
  choose defaults, or bits can be set as needed for advanced use.

Each block is a map with four keys, each with:
```
  block["gnssId"]:   gnssId (1 byte)   - 0=GPS, 1=SBAS, 2=Galileo, 3=BeiDou, 5=QZSS, 6=GLONASS, etc.
  block["resTrkCh"]: resTrkCh (1 byte) - reserved tracking channels.
  block["maxTrkCh"]: maxTrkCh (1 byte) - max tracking channels to use.
  block["flags"]:    4 byte value      - bit0 enable; higher bits = signal bitmask**.
```
**'Signal Bitmask' bits/masks/meanings are dependent on the chipset in use.  See
  the manual for your chipset for the UBX-CFG-GNSS signal bitmask definitions.

Multiple blocks can be created for each `gnssId` type.  Use the convenience
  builder $create-config-block for these.

Common `gnssId` values are given by the constants `CfgGnss.GNSS-GPS`,
  `$CfgGnss.GNSS-SBAS`, `$CfgGnss.GNSS-GALILEO`, `$CfgGnss.GNSS-BEIDOU`,
  `$CfgGnss.GNSS-QZSS`, and `$CfgGnss.GNSS-GLONASS`
*/
class CfgGnss extends Message:
  static ID ::= 0x3E

  // Common gnssId values.
  static GNSS-GPS      ::= 0
  static GNSS-SBAS     ::= 1
  static GNSS-GALILEO  ::= 2
  static GNSS-BEIDOU   ::= 3
  static GNSS-QZSS     ::= 5
  static GNSS-GLONASS  ::= 6

  // Block field numbers.
  static BLOCK-GNSSID_    ::= 0
  static BLOCK-RESTRKCH_  ::= 1
  static BLOCK-MAXTRKCH_  ::= 2
  static BLOCK-RESERVED1_ ::= 3
  static BLOCK-FLAGS_     ::= 4

  // Flags helpers.
  static FLAG-ENABLE ::= 1

  /** Construct a poll message to get current GNSS configuration. */
  constructor.poll:
    // Empty payload poll (some firmwares accept either empty or msgVer=0).
    super.private_ Message.CFG ID #[]

  /** Construct an instance with bytes from a retrieved message. */
  constructor.private_ payload/ByteArray:
    super.private_ Message.CFG ID payload

  /** Build from a list of 8-byte blocks. numTrkChHw/Use are advisory. */
  constructor.set
      --msg-ver/int=0
      --num-trk-ch-hw/int=0
      --num-trk-ch-use/int=0
      --blocks/List=[]:
    new-payload := ByteArray (4 + 8 * blocks.size)
    super.private_ Message.CFG ID new-payload

    put-uint8_ 0 msg-ver
    put-uint8_ 1 num-trk-ch-hw
    put-uint8_ 2 num-trk-ch-use
    put-uint8_ 3 blocks.size

    blocks.size.repeat: | i/int |
      block := blocks[i]  // Expect map with fields: "gnssId", "resTrkCh", "maxTrkCh", "flags"
      assert: block.size = 5
      base := 4 + 8 * i
      put-uint8_ (base + BLOCK-GNSSID_) block["gnssId"]
      put-uint8_ (base + BLOCK-RESTRKCH_) block["resTrkCh"]
      put-uint8_ (base + BLOCK-MAXTRKCH_) block["maxTrkCh"]
      put-uint8_ (base + BLOCK-RESERVED1_) 0
      put-uint32_ (base + BLOCK-FLAGS_) block["flags"]

  id-string_ -> string:
    return "GNSS"

  /**
  Convenience builder for a configuration block.

  One block is one `gnssId`, with a set of 3 properties applying to it.  (One
    `gnssId` is one of 0=GPS, 1=SBAS, 2=Galileo, 3=BeiDou, 5=QZSS, 6=GLONASS,
    etc.).  More than one block can be provided in a single message.

  `enable` is bit 1 of the flags field.  The content of the `flags` field is
    different depending on the hardware in question.
  */
  static create-config-block -> Map
      gnss-id/int
      --enable/bool?=null
      --res-trk/int=0
      --max-trk/int=0
      --flags/int=0:
    if enable:
      flags = (enable ? FLAG-ENABLE : 0) | flags
    block/Map := {"gnssId": gnss-id, "resTrkCh": res-trk, "maxTrkCh": max-trk, "flags": flags}
    return block

  /** Message version for this set of config blocks.  */
  msg-ver -> int:
    return uint8_ 0

  /** Number of config blocks in this message.  */
  num-config-blocks -> int:
    return uint8_ 3

  /** The `gnssId` for the i'th config block. */
  config-block-gnss-id i/int -> int:
    assert: 0 < i <= num-config-blocks
    return uint8_ (4 + 8*i)

  /** The flags for the i'th config block. */
  config-block-flags i/int -> int:
    return uint32_ (4 + 8*i + 4)

  /**
  The entire config block (map) for the i'th config block.

  A config block can be retrieved using this function for modification, and
    sending back.
  */
  config-block i/int -> Map:
    assert: 0 < i <= num-config-blocks
    base := (4 + 8*i)
    block := {:}
    block["gnssId"] = uint8_ (base + BLOCK-GNSSID_)
    block["resTrkCh"] = uint8_ (base + BLOCK-RESTRKCH_)
    block["maxTrkCh"] = uint8_ (base + BLOCK-MAXTRKCH_)
    block["flags"] = uint32_ (base + BLOCK-FLAGS_)
    return block
