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

    // INF (0x04) - INFO messages.
    INF: {
      0x00: "ERROR",   // M6+
      0x01: "WARNING", // M6+
      0x02: "NOTICE",  // M6+
      0x03: "TEST",    // M6+
      0x04: "DEBUG",   // M6+
    },

    // ACK (0x05).
    ACK: {
      0x00: "NAK",
      0x01: "ACK",
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

      0x8A: "VALSET",    // M8/9+.
      0x8B: "VALGET",    // M8/9+.
      0x8C: "VALDEL",    // M8/9+.
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

  // Fix type constants used through several messages.

  static INVALID-UBX-MESSAGE_ ::= "INVALID UBX MESSAGE"
  static RESERVED_ ::= 0

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := "15.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

  /** The class of this message. */
  cls/int
  /** The ID of this message. */
  id/int
  /** The Payload of this message. */
  payload/ByteArray

  /** Constructs a UBX message with the given $cls, $id, and $payload. */
  constructor.private_ .cls/int .id/int .payload/ByteArray:

  /**
  Constructs a UBX message with the given $cls, $id, and $payload.

  If message is implemented in this package, then it returns the appropriate
    sub-class.
  */
  constructor cls/int id/int payload:
    if cls == Message.ACK:
      if id == AckAck.ID:
        return AckAck.private_ payload
      else if id == AckNak.ID:
        return AckNak.private_ payload

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
      if id == CfgInf.ID:
        return CfgInf.private_ payload
      if id == CfgMsg.ID:
        return CfgMsg.private_ payload

    if cls == Message.INF:
      if id == InfError.ID:
        return InfError.private_ payload
      if id == InfWarning.ID:
        return InfWarning.private_ payload
      if id == InfNotice.ID:
        return InfNotice.private_ payload
      if id == InfTest.ID:
        return InfTest.private_ payload
      if id == InfDebug.ID:
        return InfDebug.private_ payload

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

  cls-string_ clsid/int=cls -> string:
    return PACK-CLASSES.get clsid --if-absent=:
      return "0x$(%02x clsid)"

  id-string_ clsid/int=cls msgid/int=id -> string:
    if Message.PACK-MESSAGE-TYPES.contains clsid and
        Message.PACK-MESSAGE-TYPES[clsid].contains msgid:
      return Message.PACK-MESSAGE-TYPES[clsid][msgid]
    return "0x$(%02x msgid)"

  /** The message type name in full UBX-* format. */
  full-name -> string:
    return "UBX-$cls-string_-$id-string_"

  /** See $super. */
  stringify -> string:
    return full-name

  /** Hash code for use as an identifier in a Map. */
  hash-code:
    return payload.hash-code ^ ((cls << 16) | id)

  /** Helper to return an int8 from payload index. */
  int8_ index/int --payload/ByteArray=payload -> int: return LITTLE-ENDIAN.int8 payload index

  /** Helper to return an uint8 from payload index. */
  uint8_ index/int --payload/ByteArray=payload -> int: return payload[index]

  /** Helper to return an int16 from payload index. */
  int16_ index/int --payload/ByteArray=payload -> int: return LITTLE-ENDIAN.int16 payload index

  /** Helper to return an uint16 from payload index. */
  uint16_ index/int --payload/ByteArray=payload -> int: return LITTLE-ENDIAN.uint16 payload index

  /** Helper to return an int32 from payload index. */
  int32_ index/int --payload/ByteArray=payload -> int: return LITTLE-ENDIAN.int32 payload index

  /** Helper to return an uint32 from payload index. */
  uint32_ index/int --payload/ByteArray=payload -> int: return LITTLE-ENDIAN.uint32 payload index

  /** Helper to insert an int8 into payload index. */
  put-int8_ index/int value/int --payload/ByteArray=payload -> none:
    LITTLE-ENDIAN.put-int8 payload index value

  /** Helper to insert an uint8 into payload index. */
  put-uint8_ index/int value/int --payload/ByteArray=payload -> none:
    payload[index] = value

  /** Helper to insert an int16 into payload index. */
  put-int16_ index/int value/int --payload=payload -> none:
    LITTLE-ENDIAN.put-int16 payload index value

  /** Helper to insert an uint16 into payload index. */
  put-uint16_ index/int value/int --payload=payload -> none:
    LITTLE-ENDIAN.put-uint16 payload index value

  /** Helper to insert an int32 into payload index. */
  put-int32_ index/int value/int --payload=payload -> none:
    LITTLE-ENDIAN.put-int32 payload index value

  /** Helper to insert an uint32 into payload index. */
  put-uint32_ index/int value/int --payload=payload -> none:
    LITTLE-ENDIAN.put-uint32 payload index value

  /** Helper to read a '\0'-terminated string from the payload. */
  convert-string_ start/int length/int -> string:
    // Find first '\0' within [start .. start+length].
    assert: start >= 0
    assert: length >= 0
    assert: start + length <= payload.size
    assert: payload.size > 0

    end := start + length
    pos := payload.index-of '\0' --from=start --to=end
    if pos > -1: end = pos
    return payload[start..end].to-string

/**
The UBX-ACK-ACK message.

Contains the class ID and message ID of the acknowledged message.
*/
class AckAck extends Message:
  /** The UBX-ACK-ACK message ID. */
  static ID ::= 0x01

  /** Lowest protocol version with this message type. */
  static MIN-PROTVER ::= "12.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

  /** Constructs a dummy acknowledge message. */
  constructor.private_ cls/int id:
    super.private_ Message.ACK ID #[cls, id]

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.ACK ID bytes

  /** The class ID of the original message being ACK'ed. */
  class-id -> int:
    return uint8_ 0

  /** The class ID (converted to text) of the ACK'ed message. */
  class-id-text -> string:
    return cls-string_ class-id

  /** The message ID of the original message being ACK'ed. */
  message-id -> int:
    return uint8_ 1

  /** The message ID (converted to text, if known) of the ACK'ed message. */
  message-id-text -> string:
    return id-string_ class-id message-id

  /** See $super. */
  stringify -> string:
    return  "$super: {0x$(%02x class-id)($(cls-string_ class-id)):0x$(%02x message-id)($(id-string_ class-id message-id))}"

/**
The UBX-ACK-NAK message.

Contains the class ID and message ID of the NAK (not acknowledged) message.
*/
class AckNak extends Message:
  /** The UBX-ACK-NAK message ID. */
  static ID ::= 0x00

  /** Lowest protocol version with this message type. */
  static MIN-PROTVER ::= "12.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

  /** Constructs a dummy NAK message. */
  constructor.private_ cls/int id/int:
    super.private_ Message.ACK ID #[cls, id]

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.ACK ID bytes

  /** The class ID of the original message being NAK'ed. */
  class-id -> int:
    return uint8_ 0

  /** The class ID (converted to text) of the NAK'ed message. */
  class-id-text -> string:
    return cls-string_ class-id

  /** The message ID of the original message being NAK'ed. */
  message-id -> int:
    return uint8_ 1

  /** The message ID (converted to text, if known) of the NAK'ed message. */
  message-id-text -> string:
    return id-string_ class-id message-id

  /** See $super. */
  stringify -> string:
    return  "$super: {0x$(%02x class-id)($(cls-string_ class-id)):0x$(%02x message-id)(:$(id-string_ class-id message-id))}"

/**
The UBX-CFG-MSG message.

Configures the rate at which messages are sent by the receiver.
*/
class CfgMsg extends Message:
  /** The UBX-CFG-MSG message ID. */
  static ID ::= 0x01

  /** Static port type constants. */
  static PORT-ALL   ::= -1  // Not a valid port identifier, used for config.
  static PORT-DDC   ::= 0
  static PORT-UART1 ::= 1
  static PORT-UART2 ::= 2
  static PORT-USB   ::= 3
  static PORT-SPI   ::= 4
  static PORT-RES5  ::= 5

  static PACK-PORT-TYPES := {
    PORT-ALL: "ALL",
    PORT-DDC: "DDC",
    PORT-UART1: "UART1",
    PORT-UART2: "UART2",
    PORT-USB: "USB",
    PORT-SPI: "SPI",
    PORT-RES5: "RES5",
  }

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := "15.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

  /**
  Constructs a blank configuration-rate message.

  When sent to the device it will overwrite the rates for all ports for the
    specified message type.

  In order to edit rates (as opposed to replacing them) poll for this message
    first.  Then change the required rates on the reply message, and send it
    back.
  */
  constructor.message-rate --msg-class --msg-id --rate/int --port/int=PORT-ALL:
    assert: 0 <= msg-class <= 255
    assert: 0 <= msg-id <= 255
    assert: 0 <= rate <= 255
    super.private_ Message.CFG ID (ByteArray 6 --initial=0x00)
    put-uint8_ 0 msg-class
    put-uint8_ 1 msg-id
    set-rate port --rate=rate

  /** Constructs a message to retrieve the current rate for $msg-class and $msg-id. */
  constructor.poll --msg-class/int --msg-id/int:
    assert: 0 <= msg-class <= 255
    assert: 0 <= msg-id <= 255
    super.private_ Message.CFG ID #[msg-class, msg-id]

  /** Set all port rates at once using byte array ($rates) with 6 entries. */
  constructor.per-port --msg-class/int --msg-id/int --rates/ByteArray:
    assert: 0 <= msg-class <= 255
    assert: 0 <= msg-id <= 255
    assert: rates.size == 6
    super.private_ Message.CFG ID (#[msg-class, msg-id] + rates)

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.CFG ID bytes

  class-id -> int:
    return uint8_ 0

  message-id -> int:
    return uint8_ 1

  /** Whether this is a poll message (2-byte payload). */
  is-poll -> bool:
    return payload.size == 2

  /** The name of the given $port. */
  port-string_ port/int -> string:
    assert: port == PORT-ALL or 0 <= port <= 5
    return PACK-PORT-TYPES[port]

  /**
  Gets the rate for the message type for the given $port.

  The $port parameter must be one of $PORT-DDC(0), $PORT-UART1(1),
    $PORT-UART2(2), $PORT-USB(3), $PORT-SPI(4), $PORT-RES5(5).
  */
  get-rate port/int -> int:
    assert: payload.size == 8
    assert: 0 <= port <= 5
    return uint8_ (port + 2)

  /**
  Sets the rate for the message type for the specified $port.

  Sets the rate for all ports if $port is omitted.
  */
  set-rate port/int=PORT-ALL --rate/int:
    assert: payload.size == 8
    assert: port == PORT-ALL or 0 <= port <= 5
    assert: 0 <= rate <= 0xFF

    if port == PORT-ALL:
      // Deliberately misses port PORT-RES5, as stipulated in the manual.
      5.repeat:
        put-uint8_ (2 + it) rate
    else:
      put-uint8_ (2 + port) rate

  // Convenience per-port setters/getters.
  get-rate-ddc -> int: return get-rate PORT-DDC
  get-rate-uart1 -> int: return get-rate PORT-UART1
  get-rate-uart2 -> int: return get-rate PORT-UART2
  get-rate-usb -> int: return get-rate PORT-USB
  get-rate-spi -> int: return get-rate PORT-SPI
  get-rate-res5 -> int: return get-rate PORT-RES5

  set-rate-ddc rate/int -> none: set-rate PORT-DDC --rate=rate
  set-rate-uart1 rate/int -> none: set-rate PORT-UART1 --rate=rate
  set-rate-uart2 rate/int -> none: set-rate PORT-UART2 --rate=rate
  set-rate-usb rate/int -> none: set-rate PORT-USB --rate=rate
  set-rate-spi rate/int -> none: set-rate PORT-SPI --rate=rate
  set-rate-res5 rate/int -> none: set-rate PORT-RES5 --rate=rate

  /** See $super. */
  stringify -> string:
    out-str := "$super"
    info-str := "{0x$(%02x class-id)($(cls-string_ class-id)):0x$(%02x message-id)($(id-string_ class-id message-id))}"
    if is-poll:
      return "$out-str: (poll) $info-str"
    out-str += ": $info-str"
    out-set := []
    6.repeat:
      out-set.add "$(port-string_ (it))=0x$(%02x payload[2 + it])"
    return "$out-str $(out-set.join "|")"

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

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := "15.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := "23.0"  // Manual says not after this version.

  // Common constants (see u-blox docs).
  static PORT-ALL   ::= -1  // Not a valid port identifier, used for config.
  static PORT-DDC   ::= 0
  static PORT-UART1 ::= 1
  static PORT-UART2 ::= 2
  static PORT-USB   ::= 3
  static PORT-SPI   ::= 4
  static PORT-RES5  ::= 5

  static PACK-PORT-TYPES := {
    PORT-ALL: "ALL",
    PORT-DDC: "DDC",
    PORT-UART1: "UART1",
    PORT-UART2: "UART2",
    PORT-USB: "USB",
    PORT-SPI: "SPI",
    PORT-RES5: "RES5",
  }

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
      --port-id/int=PORT-UART1
      --baud/int=9600
      --mode/int=MODE-8N1
      --in-proto/int=PROTO-UBX
      --out-proto/int=PROTO-UBX
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
  Constructs a message to retrieve the configuration for a $port-id.

  The poll payload is a single byte: $port-id, which must be one of
    $PORT-UART1, or $PORT-UART2.
  */
  constructor.poll --port-id/int=PORT-UART1:
    super.private_ Message.CFG ID #[port-id]

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.CFG ID bytes

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

  /** Reset mode 00: Hardware reset (watchdog) immediately */
  static RESET-MODE-HW-WD-IMMEDIATE ::= 0x00
  /** Reset mode 01: Controlled software reset */
  static RESET-MODE-SW-CONTROLLED ::= 0x01
  /** Reset mode 02: Controlled software reset (GNSS only) */
  static RESET-MODE-SW-CONTROLLED-GNSS-ONLY ::= 0x02
  /** Reset mode 04: Hardware reset (watchdog) after shutdown */
  static RESET-MODE-HW-RESET-WD-AFTER-SHUTDOWN ::= 0x04
  /** Reset mode 08: Controlled GNSS stop */
  static RESET-MODE-CONTROLLED-GNSS-STOP ::= 0x08
  /** Reset mode 09: Controlled GNSS start */
  static RESET-MODE-CONTROLLED-GNSS-START ::= 0x09

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := "15.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

  /**
  Constructs a reset message.

  The default parameters are a controlled software reset with a cold start.
  See the description for other parameter options.
  */
  constructor --clear-sections/int=0xFFFF --reset-mode=2:
    super.private_ Message.CFG ID (ByteArray 4)
    put-uint16_ 0 clear-sections
    put-uint8_ 2 reset-mode
    put-uint8_ 3 Message.RESERVED_

/**
The UBX-NAV-STATUS message.

The receiver navigation status.
*/
class NavStatus extends Message:
  /** The UBX-NAV-STATUS message ID. */
  static ID ::= 0x03

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := "15.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

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

  /** Fix Types for Lookup*/
  static PACK-FIX-TYPE_ ::= {
    NO-FIX: "NO-FIX",
    DEAD-RECKONING-ONLY: "DEAD-RECKONING-ONLY",
    FIX-2D: "FIX-2D",
    FIX-3D: "FIX-3D",
    GPS-DEAD-FIX: "GPS-DEAD-FIX",
    TIME-ONLY: "TIME-ONLY",
  }

  /** Constructs a message to retrieve a UBX-NAV-STATUS message. */
  constructor.poll:
    super.private_ Message.NAV ID #[]

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.NAV ID bytes

  /** The GPS interval time of week of the navigation epoch. */
  itow -> int:
    return uint32_ 0

  /**
  The current fix type.

  One of $NO-FIX, $DEAD-RECKONING-ONLY, $FIX-2D, $FIX-3D, $GPS-DEAD-FIX, $TIME-ONLY.
  */
  fix-type -> int:
    return uint8_ 4

  // Deprecated - use $fix-type.
  gps-fix -> int: return fix-type

  /**
  The current fix type in string form.

  One of "NO-FIX", "DEAD-RECKONING-ONLY", "FIX-2D", "FIX-3D", "GPS-DEAD-FIX",
    "TIME-ONLY", or "UNKNOWN" in case of error or unexpected output.
  */
  fix-type-text fixtype=fix-type -> string:
    return PACK-FIX-TYPE_.get fixtype --if-absent=:
      return "UNKNOWN"

  /**
  Navigation status flags.

  See receiver specification for details.
  */
  flags -> int:
    return uint8_ 5

  /**
  Fix status information.

  See receiver specification for details. bit[0] = 1 if differential corrections
    are available.  Bit[7..6] carries map matching status:

  ```
  00: none
  01: valid but not used, i.e. map matching data was received, but was too old
  10: valid and used, map matching data was applied
  11: valid and used, map matching data was applied. In case of sensor
      unavailability map matching data enables dead reckoning. This requires
      map matched latitude/longitude or heading data
  ```
  */
  fix-status -> int:
    return uint8_ 6

  /**
  Additional status information.

  See receiver specification for details.  Contains power-saving mode and
    spoofing detection information.  Requires PROTVER >= 18.00.
  */
  flags2 -> int:
    return uint8_ 7

  /**
  Time to first fix in milliseconds.

  Take care when deciding what to do with this value. It is not a live indicator
  of current signal quality, and the value is not kept up to date or changed
  when the fix is lost.  It is not a countdown timer for the next expected fix.
  It is simply a historical value about the most recent acquisition event.
  */
  time-to-first-fix -> int:
    return uint32_ 8

  /** Milliseconds since startup or reset. (msss) */
  ms-since-startup -> int:
    return uint32_ 12

  stringify -> string:
    return  "$super: fix-type:$fix-type-text|ttff:$(Duration --ms=time-to-first-fix)"

/**
The UBX-NAV-SAT message.

Satellite information.
*/
class NavSat extends Message:
  /** The UBX-NAV-SAT message ID. */
  static ID ::= 0x35

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := "15.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.NAV ID bytes

  /** The GPS interval time of week of the navigation epoch. */
  itow -> int:
    return uint32_ 0

  /** Message version. */
  version -> int:
    return uint8_ 4

  /** Number of satellites in the message. */
  num-svs -> int:
    return uint8_ 5

  /** Number of satellites in the message. */
  satellite-count -> int:
    return num-svs

  /**
  The satellite data in the package for the given $index.

  The $index must satisfy 0 <= $index < $num-svs.
  */
  satellite-data index -> SatelliteData:
    if not 0 <= index < num-svs: throw "INVALID ARGUMENT"
    return SatelliteData index payload --src-id=ID

  /** See $super. */
  stringify -> string:
    return "$super: satellite-count:$satellite-count"

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
  /**
  The source of this Satellite entry.

  Content and format of fields like $flags depend on the message source.
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

  /** Indicates that a carrier smoothed pseudorange used. */
  smoothed/bool

  /**
  Constructs the satellite data for the given message $payload and data.
    Parses entire UBX-NAV-SAT and UBX-NAV-SVINFO sourced Space Vehicles.
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

      offset = index * 12
      gnss-id = LITTLE-ENDIAN.uint8 payload (offset + 8)
      sv-id = LITTLE-ENDIAN.uint8 payload (offset + 9)
      cno = LITTLE-ENDIAN.uint8 payload (offset + 10)
      elev = LITTLE-ENDIAN.int8 payload (offset + 11)
      azim = LITTLE-ENDIAN.int16 payload (offset + 12)
      pr-res = (LITTLE-ENDIAN.int16 payload (offset + 14)) / 10.0 // Scale 0.1.
      flags = LITTLE-ENDIAN.uint32 payload (offset + 16)

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
      // For quality register.
      quality-mask     := 0b00001111

      // For flags bitfield.
      sv-used-mask     := 0b00000001
      diff-corr-mask   := 0b00000010
      orbit-avail-mask := 0b00000100
      orbit-eph-mask   := 0b00001000
      unhealthy-mask   := 0b00010000
      orbit-alm-mask   := 0b00100000
      orbit-aop-mask   := 0b01000000
      smoothed-mask    := 0b10000000

      offset = index * 12
      channel = LITTLE-ENDIAN.uint8 payload (offset + 8)
      sv-id = LITTLE-ENDIAN.uint8 payload (offset + 9)
      flags = LITTLE-ENDIAN.uint8 payload (offset + 10)
      quality = (LITTLE-ENDIAN.uint8 payload (offset + 11)) & quality-mask
      cno = LITTLE-ENDIAN.uint8 payload (offset + 12)
      elev = LITTLE-ENDIAN.int8 payload (offset + 13)
      azim = LITTLE-ENDIAN.int16 payload (offset + 14)
      pr-res = (LITTLE-ENDIAN.int32 payload (offset + 16)).to-float / 100 // Scaled in cm.

      // Directly usable
      diff-corr        = ((flags & diff-corr-mask) >> diff-corr-mask.count-trailing-zeros) != 0
      sv-used          = ((flags & sv-used-mask) >> sv-used-mask.count-trailing-zeros) != 0
      orbit-info-avail = ((flags & orbit-avail-mask) >> orbit-avail-mask.count-trailing-zeros) != 0
      smoothed         = ((flags & smoothed-mask) >> smoothed-mask.count-trailing-zeros) != 0

      // In the case of NavSvInfo messages, there are only two possible statuses.
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

  /** See $super. */
  stringify -> string:
    codes := ""      //gnss-id = LITTLE-ENDIAN.uint8 payload (offset + 8)

    if alm-avail == 1: codes += "A"
    if ano-avail == 1: codes += "N"

    // TODO(kasper): Make this output a whole lot prettier and easier to parse.
    //          ian: Added class/id type string from $super to assist with
    //               tests.  Would be cool to standardise them somehow...?
    return "$super: $index|$gnss-id|$sv-id|$cno|$quality|$orbit-source|$codes"

/**
The UBX-MON-VER message.

Handles receiver/software/hardware version information.
*/
class MonVer extends Message:
  /** The UBX-MON-VER message ID. */
  static ID ::= 0x04

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := "15.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

  /** Constructs a message to retrieve a UBX-MON-VER message. */
  constructor.poll:
    super.private_ Message.MON ID #[]

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.MON ID bytes

  /** Software version string. */
  sw-version -> string:
    return convert-string_ 0 30

  /** Hardware version string. */
  hw-version -> string:
    return convert-string_ 30 10

  /** Whether an extension row exists containing string $str. */
  has-extension str/string -> bool:
    return extensions-raw.any: it.contains str

  /** Whether this is a poll message (1-byte payload). */
  is-poll -> bool:
    return payload.size == 0

  /**
  The entire line of the extension with the given $str.

  Null if this message doesn't have the extension (see $has-extension).
  */
  extension str/string -> string?:
    extensions-raw.do:
      if it.contains str: return it
    return null

  /**
  A list of extension strings.

  If provided by the firmware version on the device, this function obtains its
    list of 30 byte entries, converted to strings.
  */
  extensions-raw -> List:
    raw-extensions := []
    offset := 40
    field-size := 30
    num-extensions := (payload.size - offset) / field-size
    num-extensions.repeat: | i/int |
      raw-extensions.add (convert-string_ (offset + (i * field-size)) field-size)
    return raw-extensions

  /** See $super. */
  stringify -> string:
    if is-poll: return "$super: (poll)"
    return "$super: $sw-version|$hw-version"

/**
The UBX-NAV-POSLLH message.

Geodetic position solution.  Works on u-blox 6 and M8.
*/
class NavPosLlh extends Message:
  static ID ::= 0x02

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := "15.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

  static DEGREES-SCALING-FACTOR_ ::= 1e7

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.NAV ID bytes

  /** GPS time of week of the navigation epoch. */
  itow -> int:
    return uint32_ 0

  /** Raw Longitude value returned by the device (Degrees: / 1e7). */
  longitude-raw -> int:
    return int32_ 4

  /** Raw Latitude value returned by the device (Degrees: / 1e7). */
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

  /** Longitude value converted to degrees (as float). */
  longitude-deg -> float:
    return longitude-raw / DEGREES-SCALING-FACTOR_

  /** Latitude value converted to degrees (as float). */
  latitude-deg -> float:
    return latitude-raw / DEGREES-SCALING-FACTOR_

  stringify -> string:
    return  "$super: latitude:$(latitude-deg)|longitude:$(longitude-deg)"


/**
The UBX-NAV-SVINFO message.

"Space Vehicle INFOrmation" message.  Is legacy, present on u-blox 6 and kept
  on M8 for backward compatibility.
*/
class NavSvInfo extends Message:
  static ID ::= 0x30

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := "15.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.NAV ID bytes

  /** The GPS interval time of week of the navigation epoch. (ms). */
  itow -> int:
    return uint32_ 0

  /** Number of channels. */
  num-ch -> int:
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

  Function returns $num-ch. Is included to help this 'legacy' class become
    functionally similar with UBX-NAV-SAT.
  */
  satellite-count -> int:
    return num-ch

  /**
  The satellite data in the package for the given $index.

  The $index must satisfy 0 <= $index < $num-ch.
  */
  satellite-data index/int -> SatelliteData:
    if not 0 <= index < num-ch: throw "INVALID ARGUMENT"
    return SatelliteData index payload --src-id=ID

  stringify -> string:
    return  "$super: satellite-count:$satellite-count|num-ch:$num-ch"


/**
The UBX-NAV-PVT message.

Navigation, position, velocity, and time solution.
*/
class NavPvt extends Message:
  /** The UBX-NAV-PVT message ID. */
  static ID ::= 0x07

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := "15.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

  static DEGREES-SCALING-FACTOR_ ::= 1e7

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

  /** Constructs a message to retrieve a UBX-NAV-PVT message. */
  constructor.poll:
    super.private_ Message.NAV ID #[]

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.NAV ID bytes

  /** Whether this is a GNSS fix. */
  is-gnss-fix -> bool:
    return (flags & 0b00000001) != 0

  /** The time in UTC. */
  utc-time -> Time:
    return Time.utc year month day h m s --ns=ns

  /** The GPS interval time of week of the navigation epoch. */
  itow -> int:
    return uint32_ 0

  /** The year (UTC). */
  year -> int:
    return uint16_ 4

  /**
  The month (UTC).
  In the range [1..12].
  */
  month -> int:
    return uint8_ 6

  /**
  The day (UTC).
  In the range [1..31].
  */
  day -> int:
    return uint8_ 7

  /**
  The hours (UTC).
  In the range [0..23].
  */
  h -> int:
    return uint8_ 8

  /**
  The minutes (UTC).
  In the range [0..59].
  */
  m -> int:
    return uint8_ 9

  /**
  The seconds (UTC).
  In the range [0..60].
  */
  s -> int:
    return uint8_ 10

  /**
  Validity flag.
  See receiver specification for details.
  */
  valid -> int:
    return uint8_ 11

  /** Time accuracy estimate in nanoseconds */
  time-acc -> int:
    return uint32_ 12

  /**
  Fraction of second in nano seconds.
  The fraction may be negative.
  */
  ns -> int:
    return int32_ 16

  /**
  The type of fix.
  One of $NO-FIX, $DEAD-RECKONING-ONLY, $FIX-2D, $FIX-3D, $GPS-DEAD-FIX, $TIME-ONLY.
  */
  fix-type -> int:
    return uint8_ 20

  /**
  Fix status flags.
  See receiver specification for details.
  */
  flags -> int:
    return uint8_ 21

  /**
  Additional fix status flags.
  See receiver specification for details.
  */
  flags2 -> int:
    return uint8_ 22

  /** Number of satellites used for fix. */
  num-sv -> int:
    return uint8_ 23

  /** Longitude. */
  lon -> int:
    return int32_ 24

  /** Convenience method for $lon. */
  // To match message type 'UBX-NAV-POSLLH'.
  longitude-raw -> int:
    return lon

  /** Convenience method for $lon converted to degrees (as float). */
  // To match message type 'UBX-NAV-POSLLH'.
  longitude-deg -> float:
    return lon / DEGREES-SCALING-FACTOR_

  /** Latitude. */
  lat -> int:
    return int32_ 28

  /** Convenience method for $lat. */
  // To match message type 'UBX-NAV-POSLLH'.
  latitude-raw -> int:
    return lat

  /** Convenience method for $lat converted to degrees (as float). */
  // To match message type 'UBX-NAV-POSLLH'.
  latitude-deg -> float:
    return lat / DEGREES-SCALING-FACTOR_

  /** Height above ellipsoid in millimeter. */
  height -> int:
    return int32_ 32

  /** Height above mean sea level in millimeter. */
  height-msl -> int:
    return int32_ 36

  /** Horizontal accuracy in millimeter. */
  horizontal-acc -> int:
    return uint32_ 40

  /** Vertical accuracy in millimeter. */
  vertical-acc -> int:
    return uint32_ 44

  /** NED north velocity in millimeters per second. */
  north-vel -> int:
    return int32_ 48

  /** NED east velocity in millimeters per second. */
  east-vel -> int:
    return int32_ 52

  /** NED down velocity in millimeters per second. */
  down-vel -> int:
    return int32_ 56

  /** Ground speed (2D) in millimeters per second. */
  ground-speed -> int:
    return int32_ 60

  /** Heading of motion (2D). */
  heading-of-motion -> int:
    return int32_ 64

  /** Speed accuracy in millimeters per second. */
  speed-acc -> int:
    return uint32_ 68

  /** Heading accuracy. */
  heading-acc -> int:
    return uint32_ 72

  /**
  Position DOP.

  Position 'Dilution of Position' scale.  Scale 0.01.
  */
  position-dop -> float:
    return (uint16_ 76) / 100.0

  /**
  Additional flags.

  See receiver specification for details.
  */
  flags3 -> int:
    return uint32_ 78

  /**
  The heading of the vehicle.

  See receiver specification for details.
  */
  heading-vehicle -> int:
    return int32_ 84

  /**
  Magnetic declination.
  See receiver specification for details.
  */
  magnetic-declination -> int:
    return int16_ 88

  /**
  Accuracy of magnetic declination.
  See receiver specification for details.
  */
  magnetic-acc -> int:
    return uint16_ 90

  stringify -> string:
    return  "$super: latitude:$(latitude-deg)|longitude:$(longitude-deg)"

/**
The UBX-NAV-SOL message.

Legacy Navigation solution, in ECEF (Earth-Centered, Earth-Fixed cartesian
  coordinates).  This message is included for backwards compatibility.  Whilst
  it is available on M8 and later, UBX-NAV-PVT messages are preferred).
*/
class NavSol extends Message:
  static ID ::= 0x06

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := "15.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

  static FLAGS-GPS-FIX-OK_              ::= 0b00000001 // e.g. is within DOP & ACC Masks.
  static FLAGS-DGPS-USED-MASK_          ::= 0b00000010
  static FLAGS-WEEK-VALID-MASK_         ::= 0b00000100
  static FLAGS-TIME-OF-WEEK-VALID-MASK_ ::= 0b00001000

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

  /** Fix Types for Lookup*/
  static PACK-FIX-TYPE_ ::= {
    NO-FIX: "NO-FIX",
    DEAD-RECKONING-ONLY: "DEAD-RECKONING-ONLY",
    FIX-2D: "FIX-2D",
    FIX-3D: "FIX-3D",
    GPS-DEAD-FIX: "GPS-DEAD-FIX",
    TIME-ONLY: "TIME-ONLY",
  }

  /** Constructs a message to retrieve a UBX-NAV-SOL message. */
  constructor.poll:
    super.private_ Message.NAV ID #[]

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.NAV ID bytes

  /** Whether this is a GNSS fix. */
  is-gnss-fix -> bool:
    return (flags & FLAGS-GPS-FIX-OK_) != 0

  /** The GPS interval time of week of the navigation epoch. */
  itow -> int:
    return uint32_ 0

  /**
  The fractional GPS interval time of week (in ns) of the navigation epoch.

  Range in ns: -500000..+500000.
  */
  ftow -> int:
    return int32_ 4

  /**
  The GPS Week number.

  This is the week number since Jan 6 1980.  This value is not always valid.  A
    potentially non-zero value can be obtained if it has taken a long time to
    get a fix.  Test for $has-valid-week before using this value.
  */
  week -> int:
    return int16_ 8

  /**
  Whether GPS Week number is valid. (UBX field: WKNSET.)

  See $week.  Time values should not be used until this returns true.
  */
  has-valid-week -> bool:
    return ((flags & FLAGS-WEEK-VALID-MASK_) >> FLAGS-WEEK-VALID-MASK_.count-trailing-zeros) != 0

  /**
  Whether GPS Time of Week number is valid. (UBX field: TOWSET.)
  */
  valid-time-of-week -> bool:
    return ((flags & FLAGS-TIME-OF-WEEK-VALID-MASK_) >> FLAGS-TIME-OF-WEEK-VALID-MASK_.count-trailing-zeros) != 0

  //The precise GPS time of week in seconds is:
  //(iTOW * 1e-3) + (fTOW * 1e-9)

  /**
  Whether DGPS is used.  (UBX field: diffSoln)
  */
  dgps-used -> bool:
    return ((flags & FLAGS-DGPS-USED-MASK_) >> FLAGS-DGPS-USED-MASK_.count-trailing-zeros) != 0

  /**
  The type of fix.

  One of $NO-FIX, $DEAD-RECKONING-ONLY, $FIX-2D, $FIX-3D, $GPS-DEAD-FIX, $TIME-ONLY.
  */
  fix-type -> int:
    return uint8_ 10

  /**
  The current fix type in string form.

  One of "NO-FIX", "DEAD-RECKONING-ONLY", "FIX-2D", "FIX-3D", "GPS-DEAD-FIX",
    "TIME-ONLY", or "UNKNOWN" in case of error or unexpected output.
  */
  fix-type-text fixtype/int=fix-type -> string:
    return PACK-FIX-TYPE_.get fixtype --if-absent=:
      return "UNKNOWN"

  /**
  Fix status flags.

  See receiver specification for details.
  */
  flags -> int:
    return uint8_ 11

  /**
  Number of satellites used for fix.
  */
  num-sv -> int:
    return uint8_ 47

  /**
  Position DOP.

  Position 'Dilution of Position' scale.  Scale 0.01.
  */
  position-dop -> float:
    return (uint16_ 44).to-float / 100

  /** ECEF X coordinate. */
  ecef-x-cm -> int: return int32_ 12      // I4 cm.

  /** ECEF Y coordinate. */
  ecef-y-cm -> int: return int32_ 16      // I4 cm.

  /** ECEF Z coordinate. */
  ecef-z-cm -> int: return int32_ 20      // I4 cm.

  /** 3D Position Accuracy Estimate. */
  p-acc-cm  -> int: return uint32_ 24     // U4 cm.

  /** ECEF X velocity in cm/s. */
  ecef-vx-cms -> int: return int32_ 28      // I4 cm/s.

  /** ECEF Y velocity in cm/s. */
  ecef-vy-cms -> int: return int32_ 32      // I4 cm/s.

  /** ECEF Z velocity in cm/s. */
  ecef-vz-cms -> int: return int32_ 36      // I4 cm/s.

  /** Speed Accuracy Estimate in cm/s. */
  s-acc-cms  -> int: return uint32_ 40      // U4 cm/s.

  /** Reserved 1. */
  reserved1 -> int: return uint8_ 46      // U1.

  /** Reserved 2. */
  reserved2 -> int: return uint32_ 48      // U4 (M8 doc shows U1[4]; same 4 bytes).

  stringify -> string:
    return  "$super: fix-type:$fix-type-text|num-sv:$num-sv"

/**
The UBX-NAV-TIMEUTC message.

UTC time solution.  Functions on 6M and later devices.
*/
class NavTimeUtc extends Message:
  static ID ::= 0x21

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := "15.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

  static TIME-OF-WEEK-VALID-MASK_ ::= 0b00000001
  static WEEK-VALID-MASK_         ::= 0b00000010
  static UTC-VALID-MASK_          ::= 0b00000100

  /** Constructs a message to retrieve a UBX-NAV-TIMEUTC message. */
  constructor.poll:
    super.private_ Message.NAV ID #[]

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.NAV ID bytes

  /** The GPS interval time of week of the navigation epoch. */
  itow -> int:
    return uint32_ 0

  /** The time in UTC. */
  utc-time -> Time:
    return Time.utc year month day h m s --ns=ns

  /** UTC Time accuracy estimate, in nanoseconds. */
  time-accuracy-est -> int:
    return uint32_ 4

  /** UTC time, nanoseconds only. */
  ns -> int:
    return int32_ 8

  /** UTC time, year only. */
  year -> int:
    return uint16_ 12

  /** UTC time, month only. */
  month -> int:
    return uint8_ 14

  /** UTC time, calendar day only. */
  day -> int:
    return uint8_ 15

  /** UTC time, hours only. */
  h -> int:
    return uint8_ 16

  /** UTC time, minutes only. */
  m -> int:
    return uint8_ 17

  /**
  UTC Seconds.

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

  M8+: upper bits carry UTC standard.
  */
  valid-flags-raw -> int:
    return uint8_ 19

  /**
  Returns if GPS Week number is Valid. (UBX field: ValidWKN)
  */
  valid-week -> bool:
    return ((valid-flags-raw & WEEK-VALID-MASK_) >> WEEK-VALID-MASK_.count-trailing-zeros) != 0

  /**
  Returns if GPS Time of Week number is Valid. (UBX field: ValidTOW)
  */
  valid-time-of-week -> bool:
    return ((valid-flags-raw & TIME-OF-WEEK-VALID-MASK_) >> TIME-OF-WEEK-VALID-MASK_.count-trailing-zeros) != 0

  /**
  Returns if UTC time is valid. (UBX field: ValidUTC - If the leap seconds are known.)
  */
  valid-utc -> bool:
    return ((valid-flags-raw & UTC-VALID-MASK_) >> UTC-VALID-MASK_.count-trailing-zeros) != 0

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

  stringify -> string:
    return  "$super: valid-utc:$valid-utc|utc-time:$utc-time"

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

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := "15.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

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

  /** Constructs a message to retrieve the TP5 configuration for a given $tp-idx. */
  constructor.poll --tp-idx/int=TP-IDX-0:
    new-payload := ByteArray 2
    super.private_ Message.CFG ID new-payload
    put-uint8_ 0 tp-idx
    put-uint8_ 1 1  // Version.

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.CFG ID bytes

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

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := "15.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

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

  /** Constructs a message to retrieve the current NAV5 configuration. */
  constructor.poll:
    super.private_ Message.CFG ID #[]

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.CFG ID bytes

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

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := "15.0"

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

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

  /** Constructs a poll message to retrieve the current GNSS configuration. */
  constructor.poll:
    // Empty payload poll (some firmwares accept either empty or msgVer=0).
    super.private_ Message.CFG ID #[]

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.CFG ID bytes

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
      assert: block.size == 5
      base := 4 + 8 * i
      put-uint8_ (base + BLOCK-GNSSID_) block["gnssId"]
      put-uint8_ (base + BLOCK-RESTRKCH_) block["resTrkCh"]
      put-uint8_ (base + BLOCK-MAXTRKCH_) block["maxTrkCh"]
      put-uint8_ (base + BLOCK-RESERVED1_) 0
      put-uint32_ (base + BLOCK-FLAGS_) block["flags"]

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

/**
The UBX-CFG-INF message.

This message type polls, gets and sets the configuration for UBX-CFG-* message
  types.  It controls whether UBX-INF-* (and/or NMEA informational) messages
  are emitted, and on which interface/port.

  These types are asynchronous human-readable text messages, emitted
  by the receiver when an error condition occurs.  The UBX-INF-* message types
  cannot be directly polled for.
*/
class CfgInf extends Message:
  /** The UBX-CFG-INF message ID. */
  static ID ::= 0x02

  // protocolID values.
  static PROTO-UBX  ::= 0
  static PROTO-NMEA ::= 1

  // infMsgMask bits (per port).
  static LEVEL-NONE    ::= 0b00000000
  static LEVEL-ALL     ::= 0b00011111
  static LEVEL-ERROR   ::= 0b00000001
  static LEVEL-WARNING ::= 0b00000010
  static LEVEL-NOTICE  ::= 0b00000100
  static LEVEL-TEST    ::= 0b00001000
  static LEVEL-DEBUG   ::= 0b00010000

  // Port indices infMsgMask[6].
  static PORT-ALL   ::= -1  // Not a valid port identifier, used for config.
  static PORT-DDC   ::= 0
  static PORT-UART1 ::= 1
  static PORT-UART2 ::= 2
  static PORT-USB   ::= 3
  static PORT-SPI   ::= 4
  static PORT-RES5  ::= 5

  static PACK-PORT-TYPES := {
    PORT-ALL: "ALL",
    PORT-DDC: "DDC",
    PORT-UART1: "UART1",
    PORT-UART2: "UART2",
    PORT-USB: "USB",
    PORT-SPI: "SPI",
    PORT-RES5: "RES5",
  }

  static ENABLED ::= true
  static DISABLED ::= false
  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := ""

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := "23.01"

  /**
  Retrieves the current INF configuration for a given $protocol-id.

  $protocol-id = one of $PROTO-UBX (0, default) to query UBX-INF-* enable masks,
    or $PROTO-NMEA (1) to query NMEA informational enable masks.
  */
  constructor.poll --protocol-id/int=PROTO-UBX:
    assert: protocol-id == PROTO-UBX or protocol-id == PROTO-NMEA
    super.private_ Message.CFG ID #[protocol-id]

  /**
  Constructs a blank instance of UBX-CFG-INF.

  # Warning
  Configurations present in this message overwrite the current configuration,
    not edit it.  To edit the current configuration, poll for this message, and
    use the methods in the response message.  Then send that message back as a
    configuration message.
  */
  constructor --protocol-id/int=PROTO-UBX:
    assert: protocol-id == PROTO-UBX or protocol-id == PROTO-NMEA
    super.private_ Message.CFG ID (ByteArray 10)
    put-uint8_ 0 protocol-id

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.CFG ID bytes

  /** Whether this is a poll message (1-byte payload). */
  is-poll -> bool:
    return payload.size == 1

  /**
  The protocol ($PROTO-UBX, or $PROTO-NMEA).
  */
  protocol-id -> int:
    return uint8_ 0

  /**
  Enable or disable) a specific logging $level, for a specific $port.

  Sets the logging $level for all ports if $port is omitted. Logging $level
    should be one of  $LEVEL-ERROR, $LEVEL-WARNING, $LEVEL-NOTICE, $LEVEL-TEST,
    or $LEVEL-DEBUG.
  */
  set-port-level port/int=PORT-ALL --level/int enable/bool=true -> none:
    assert: protocol-id == PROTO-UBX or protocol-id == PROTO-NMEA
    assert: port == PORT-ALL or 0 <= port <= 5
    assert: 0 <= level <= 0x1F

    if port == PORT-ALL:
      // Deliberately misses port PORT-RES5 as stated in the manual.
      5.repeat:
        if enable: put-uint8_ (4 + it) ((uint8_ (4 + it)) | level)
        else: put-uint8_ (4 + it) ((uint8_ (4 + it)) & (~level & 0xFF))
    else:
      if enable: put-uint8_ (4 + port) ((uint8_ (4 + port)) | level)
      else: put-uint8_ (4 + port) ((uint8_ (4 + port)) & (~level & 0xFF))

  /**
  Sets the raw logging level mask byte the given $port.

  $raw-value is a mask of all the bits of the requested levels combined.
    Individual bits are given by $LEVEL-ERROR, $LEVEL-WARNING, $LEVEL-NOTICE
    $LEVEL-TEST, and $LEVEL-DEBUG.

  $port must be one of  $PORT-ALL(-1), $PORT-DDC(0), $PORT-UART1(1),
    $PORT-UART2(2), $PORT-USB(3), $PORT-SPI(4), $PORT-RES5(5).
  */
  set-port-level-raw port/int=PORT-ALL --raw-value/int -> none:
    assert: protocol-id == PROTO-UBX or protocol-id == PROTO-NMEA
    assert: port == PORT-ALL or 0 <= port <= 5
    assert: 0 <= raw-value <= 0x1F

    if port == PORT-ALL:
      // Deliberately misses port PORT-RES5, as stipulated in the manual.
      5.repeat:
        put-uint8_ (4 + it) raw-value
    else:
      put-uint8_ (4 + port) raw-value

  /**
  Gets the raw logging level mask byte the given $port.

  The $port parameter must be one of $PORT-DDC(0), $PORT-UART1(1),
    $PORT-UART2(2), $PORT-USB(3), $PORT-SPI(4), $PORT-RES5(5).
  */
  get-port-level-raw port/int -> int:
    assert: payload.size >= 10
    assert: 0 <= port <= 5
    return uint8_ (4 + port)

  // Convenience per-port getters/setters.
  get-ddc-level-raw -> int: return get-port-level-raw PORT-DDC
  get-uart1-level-raw -> int: return get-port-level-raw PORT-UART1
  get-uart2-level-raw -> int: return get-port-level-raw PORT-UART2
  get-usb-level-raw -> int: return get-port-level-raw PORT-USB
  get-spi-level-raw -> int: return get-port-level-raw PORT-SPI
  get-res5-level-raw -> int: return get-port-level-raw PORT-RES5

  set-ddc-level-raw level/int -> none: set-port-level-raw PORT-DDC --raw-value=level
  set-uart1-level-raw level/int -> none: set-port-level-raw PORT-UART1 --raw-value=level
  set-uart2-level-raw level/int -> none: set-port-level-raw PORT-UART2 --raw-value=level
  set-usb-level-raw level/int -> none: set-port-level-raw PORT-USB --raw-value=level
  set-spi-level-raw level/int -> none: set-port-level-raw PORT-SPI --raw-value=level
  set-res5-level-raw level/int -> none: set-port-level-raw PORT-RES5 --raw-value=level

  // Helpers for checking whether a given INF type is enabled on a port.
  error-enabled port/int -> bool: return (get-port-level-raw port) & LEVEL-ERROR != 0
  warning-enabled port/int -> bool: return (get-port-level-raw port) & LEVEL-WARNING != 0
  notice-enabled port/int -> bool: return (get-port-level-raw port) & LEVEL-NOTICE != 0
  test-enabled port/int -> bool: return (get-port-level-raw port) & LEVEL-TEST != 0
  debug-enabled port/int -> bool: return (get-port-level-raw port) & LEVEL-DEBUG != 0

  // Helpers for enabling a given INF type on a port.
  enable-error port/int=PORT-ALL -> none: set-port-level port --level=LEVEL-ERROR ENABLED
  enable-warning port/int=PORT-ALL -> none: set-port-level port --level=LEVEL-WARNING ENABLED
  enable-notice port/int=PORT-ALL -> none: set-port-level port --level=LEVEL-NOTICE ENABLED
  enable-test port/int=PORT-ALL -> none: set-port-level port --level=LEVEL-TEST ENABLED
  enable-debug port/int=PORT-ALL -> none: set-port-level port --level=LEVEL-DEBUG ENABLED
  /**
  Enables all message types for the given $port.

  Sets the type for all ports if $port is omitted.
  */
  enable-all --port/int=PORT-ALL:
    set-port-level port --level=LEVEL-ALL

  // Helpers for disabling a given INF type on a port.
  disable-error port/int=PORT-ALL -> none: set-port-level port --level=LEVEL-ERROR DISABLED
  disable-warning port/int=PORT-ALL -> none: set-port-level port --level=LEVEL-WARNING DISABLED
  disable-notice port/int=PORT-ALL -> none: set-port-level port --level=LEVEL-NOTICE DISABLED
  disable-test port/int=PORT-ALL -> none: set-port-level port --level=LEVEL-TEST DISABLED
  disable-debug port/int=PORT-ALL -> none: set-port-level port --level=LEVEL-DEBUG DISABLED
  /**
  Disables all message types for the given $port.

  Sets the type for all ports if $port is omitted.
  */
  disable-all --port/int=PORT-ALL:
    set-port-level port --level=LEVEL-NONE

  /** The name of this messages' protocol. */
  proto-string_ -> string:
    if protocol-id == PROTO-UBX: return "UBX"
    if protocol-id == PROTO-NMEA: return "NMEA"
    return "UNKNOWN($protocol-id)"

  /** The name of the given $port. */
  port-string_ port/int -> string:
    assert: port == PORT-ALL or 0 <= port <= 5
    return PACK-PORT-TYPES[port]

  /** See $super. */
  stringify -> string:
    out-str := "$super: {$(proto-string_)}"
    if is-poll:
      return "$out-str (poll)"
    6.repeat:
      out-str += "|$(port-string_ it)=0x$(%02x payload[4 + it])"
    return out-str


/**
The UBX-INF-ERROR message.

Asynchronous human-readable text message emitted by the receiver (if
  configured) when an error condition occurs in the ublox firmware.

This message is not pollable.  It is enabled/disabled via configuration using
  $CfgInf (UBX-CFG-INF).
*/
class InfError extends Message:
  /** The UBX-INF-ERROR message ID. */
  static ID ::= 0x00

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := ""

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.INF ID bytes

  /** The text payload. */
  text -> string:
    return convert-string_ 0 payload.size

  /** See $super. */
  stringify -> string:
    if payload.size > 0: return "$super: $text"
    return super

/**
The UBX-INF-WARNING message.

Asynchronous human-readable text message emitted by the receiver (if configured)
  for ublox-sourced warnings.

This message is not pollable.  It is enabled/disabled via configuration using
  $CfgInf (UBX-CFG-INF).
*/
class InfWarning extends Message:
  /** The UBX-INF-WARNING message ID. */
  static ID ::= 0x01

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := ""

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.INF ID bytes

  /** The text payload. */
  text -> string:
    return convert-string_ 0 payload.size

  /** See $super. */
  stringify -> string:
    if payload.size > 0: return "$super: $text"
    return super

/**
The UBX-INF-NOTICE message.

Asynchronous human-readable text message emitted by the receiver for ublox-
  sourced notice output (if configured).

This message is not pollable.  It is enabled/disabled via configuration using
  $CfgInf (UBX-CFG-INF).
*/
class InfNotice extends Message:
  /** The UBX-INF-NOTICE message ID. */
  static ID ::= 0x02

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := ""

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.INF ID bytes

  /** The text payload. */
  text -> string:
    return convert-string_ 0 payload.size

  /** See $super. */
  stringify -> string:
    if payload.size > 0: return "$super: $text"
    return super

/**
The UBX-INF-TEST message.

Asynchronous human-readable text message emitted by the receiver (if
  configured) for ublox-sourced test output.

This message is not pollable.  It is enabled/disabled via configuration using
  $CfgInf (UBX-CFG-INF).
*/
class InfTest extends Message:
  /** The UBX-INF-TEST message ID. */
  static ID ::= 0x03

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := ""

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.INF ID bytes

  /** The text payload. */
  text -> string:
    return convert-string_ 0 payload.size

  /** See $super. */
  stringify -> string:
    if payload.size > 0: return "$super: $text"
    return super

/**
The UBX-INF-DEBUG message.

Asynchronous human-readable text message emitted by the receiver (if
  configured) for ublox-sourced debug output.

This message is not pollable.  It is enabled/disabled via configuration using
  $CfgInf (UBX-CFG-INF).
*/
class InfDebug extends Message:
  /** The UBX-INF-DEBUG message ID. */
  static ID ::= 0x04

  /**
  The minimum protocol version for the message type.

  Devices must support at least this protocol version to use the message.
  */
  static MIN-PROTVER/string := ""

  /**
  The maximum protocol version for the message type.

  Devices supporting protocol version newer than this may not be able to
    work with the message type.
  */
  static MAX-PROTVER/string := ""

  /** Constructs an instance with the payload $bytes from a retrieved message. */
  constructor.private_ bytes/ByteArray:
    super.private_ Message.INF ID bytes

  /** The text payload. */
  text -> string:
    return convert-string_ 0 payload.size

  /** See $super. */
  stringify -> string:
    if payload.size > 0: return "$super: $text"
    return super
