// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import reader show *
/**
Support for the UBX messages from the UBX data protocol.

The UBX data protocol is used by the ublox GNSS receivers in the Max-M*
  series. Some messages are deprecated between versions.

The receiver description for each receiver describes the supported UBX
  message.
- Max-M8: https://www.u-blox.com/en/docs/UBX-13003221
- Max-M9: https://www.u-blox.com/en/docs/UBX-19035940
*/

import binary show LITTLE_ENDIAN UINT32_MAX

/**
A UBX message from the UBX data protocol.
*/
class Message:
  /** The class of this message. */
  clazz /int
  /** The ID of this message. */
  id /int
  /** The Payload of this message. */
  payload /ByteArray ::= #[]

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

  /** Map from class bytes to their string representations. */
  static PACK_CLASSES ::= {NAV: "NAV", RXM: "RXM", INF: "INF", ACK: "ACK", CFG: "CFG", UPD: "UPD", MON: "MON", AID: "AID", TIM: "TIM", ESF: "ESF", MGA: "MGA", LOG: "LOG", SEC: "SEC", HNR: "HNR"}

  static INVALID_UBX_MESSAGE_ ::= "INVALID UBX MESSAGE"

  /** Constructs a UBX message with the given $clazz, $id, and $payload. */
  constructor .clazz .id .payload:

  /**
  Constructs a UBX message from the given $bytes.

  The $bytes must be a valid UBX message (contain the sync bytes and a
    valid checksum).
  */
  constructor.from_bytes bytes/ByteArray:
    if not is_valid_frame_ bytes: throw INVALID_UBX_MESSAGE_
    clazz = bytes[2]
    id = bytes[3]
    payload = bytes[4..bytes.size-2]

  /**
  Constructs a UBX message from the given $reader.

  The $reader must be able to provide a valid UBX frame.
  */
  constructor.from_reader reader/BufferedReader:
    if (reader.byte 0) != 0xb5 or (reader.byte 1) != 0x62: throw INVALID_UBX_MESSAGE_

    // Verify the length and get full the packet.
    length ::= (reader.byte 4) | (((reader.byte 5) & 0xff) << 8)
    if not (0 <= length and length <= 512): throw INVALID_UBX_MESSAGE_
    frame ::= reader.bytes length + 8

    // Verify the checksum.
    if not is_valid_frame_ frame: throw INVALID_UBX_MESSAGE_

    msg_class ::= frame[2]
    msg_id    ::= frame[3]
    payload   ::= frame[6..length + 6]
    reader.skip length + 8
    return Message msg_class msg_id payload

  static is_valid_frame_ frame/ByteArray -> bool:
    // Check the sync bytes.
    if frame[0] != 0xb5 or frame[1] != 0x62: throw INVALID_UBX_MESSAGE_

    // Check the payload length.
    length ::= LITTLE_ENDIAN.uint16 frame 4
    if not (0 <= length and length <= 512): return false

    ck_a ::= frame[frame.size - 2]
    ck_b ::= frame[frame.size - 1]
    compute_checksum_ frame: | a b |
      return ck_a == a and ck_b == b
    return false

  /**
  Computes the checksum of the given $bytes.

  Calls the $callback with the computed checksum values ck_a and ck_b as
    arguments.
  */
  static compute_checksum_ bytes/ByteArray [callback]:
    ck_a := 0
    ck_b := 0
    bytes = bytes[2..bytes.size - 2]
    bytes.size.repeat: | i |
      ck_a = (ck_a + bytes[i]) & 0xff
      ck_b = (ck_b + ck_a) & 0xff
    callback.call ck_a ck_b

  /**
  Transforms this message to a byte array that can be send to a ublox
    GNSS receiver.

  The byte array contains the starting magic bytes 0xB5 and 0x62 as well as
    the trailing checksum.
  */
  to_byte_array -> ByteArray:
    bytes := ByteArray 8 + payload.size
    bytes[0] = 0xB5
    bytes[1] = 0x62
    bytes[2] = clazz
    bytes[3] = id
    LITTLE_ENDIAN.put_uint16 bytes 4 payload.size
    bytes.replace 6 payload
    compute_checksum_ bytes: | ck_a ck_b |
      bytes[bytes.size - 2] = ck_a
      bytes[bytes.size - 1] = ck_b
    return bytes

  class_string_ -> string:
    PACK_CLASSES.get clazz
      --if_present=:
        return it
      --if_absent=:
        return "0x$(%02x clazz)"
    unreachable

  id_string_ -> string:
    return "0x$(%02x id)"

  /** See $super. */
  stringify -> string:
    return "UBX-$class_string_-$id_string_"

  /** Whether this is an instance of $NavPosllh. */
  is_ubx_nav_posllh -> bool:
    return NavPosllh.is_instance this

  /** This message as a $NavPosllh. */
  ubx_nav_posllh -> NavPosllh:
    return NavPosllh this

  /** Whether this is an instance of $NavPvt. */
  is_ubx_nav_pvt -> bool:
    return NavPvt.is_instance this

  /** This message as a $NavPvt. */
  ubx_nav_pvt -> NavPvt:
    return NavPvt this

  /** Whether this is an instance of $NavStatus. */
  is_ubx_nav_status -> bool:
    return NavStatus.is_instance this

  /** This message as a $NavStatus. */
  ubx_nav_status -> NavStatus:
    return NavStatus this

  /** Whether this is an instance of $NavSat. */
  is_ubx_nav_sat -> bool:
    return NavSat.is_instance this

  /** This message as a $NavSat. */
  ubx_nav_sat -> NavSat:
    return NavSat this

  /** Whether this is an instance of $NavTimeutc. */
  is_ubx_nav_timeutc -> bool:
    return NavTimeutc.is_instance this

  /** This message as a $NavTimeutc. */
  ubx_nav_timeutc -> NavTimeutc:
    return NavTimeutc this

/** The UBX-ACK-ACK message. */
class AckAck extends Message:
  static ID ::= 0x01

  constructor cls id:
    super Message.ACK ID #[cls, id]

/** The UBX-ACK-NAK message. */
class AckNak extends Message:
  static ID ::= 0x02

  constructor cls id:
    super Message.ACK ID #[cls, id]

/** The UBX-CFG-MSG message. */
class CfgMsg extends Message:
  static ID ::= 0x01

  constructor --msg_class --msg_id --rate:
    super Message.CFG ID #[msg_class, msg_id, rate]

  id_string__ -> string:
    return "MSG"

/**
The UBX-CFG-GNSS message.
*/
// https://www.u-blox.com/en/docs/UBX-13003221#%5B%7B%22num%22%3A667%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C379.84%2Cnull%5D
class CfgGnss extends Message:
  static ID ::= 0x3e

  /** Constructs the get version of the message. */
  constructor.get:
    super Message.CFG ID #[]

  /**
  Constructs a message with the given parameters.
  The "reserved" parameters are the amount of channels the receiver will
    use for each system.
  The "max" parameters are the maximum amount of channels the receiver will
    use for each system.
  */
  constructor
      --gps_reserved_channels=8 --gps_max_channels=16
      --sbas_reserved_channels=1 --sbas_max_channels=3
      --qzss_reserved_channels=0 --qzss_max_channels=3
      --glonas_reserved_channels=8 --glonas_max_channels=14:

    // U-blox recommend using QZSS whenever GPS is active at the same time.
    pl := #[
      0, 0, 32, 7,
      0, gps_reserved_channels, gps_max_channels, 0, 1, 0, 1, 1,
      1, sbas_reserved_channels,  sbas_max_channels, 0, 1, 0, 1, 1,
      2, 0,  0, 0, 0, 0, 1, 1,
      3, 0,  0, 0, 0, 0, 1, 1,
      4, 0,  0, 0, 0, 0, 1, 1,
      5, qzss_reserved_channels, qzss_max_channels, 0, 1, 0, 1, 1,
      6, glonas_reserved_channels, glonas_max_channels, 0, 1, 0, 1, 1
    ]
    super Message.CFG ID pl

  id_string_ -> string:
    return "GNSS"

/** The UBX-CFG-NAVX5 message. */
class CfgNavx5 extends Message:
  static ID ::= 0x23

  constructor.get:
    super Message.CFG ID #[]

  constructor --ack_aiding/bool:
    pl :=  #[
      2, 0,             // 0: version
      255, 255,         // 2: mask1
      63, 2, 0, 0,      // 4: mask2
      3, 2,             // 8: reserved1
      3,                // 10: minSVs
      32,               // 11: maxSVs
      6,                // 12: minCNO
      0,                // 13: reserved2
      0,                // 14: initial fix must be 3D
      1, 0,             // 15: reserved3
      0,                // 17: ackAiding
      75, 7,            // 18: wknRollover
      0, 1, 0, 0, 1, 1, // 20: reserved4
      0,                // 26: usePPP
      0,                // 27: aopCfg
      0, 100,           // 28: reserved5
      100, 0,           // 30: aopOrBMaxErr
      0, 1, 17, 0,      // 32: reserved6
      0, 0, 0,          // 36: reserved7
      0                 // 39: useAdr
    ]
    if ack_aiding:
      // We are assigning the ackAiding byte after creation, so that the
      // payload initially points to the read-only byte-array.
      // If we made the 17th byte conditional on a parameter, then the
      // compiler wouldn't be able to create the literal efficiently.
      pl[17] = 1
    super Message.CFG ID pl

  id_string_ -> string:
    return "NAVX5"

/** The UBX-CFG-NAV5 message. */
class CfgNav5 extends Message:
  static ID ::= 0x24

  constructor.get:
    super Message.CFG ID #[]

  id_string_ -> string:
    return "NAV5"

/** The UBX-CFG-RATE message. */
class CfgRate extends Message:
  static ID ::= 0x08

  constructor.get:
    super Message.CFG ID #[]

  id_string_ -> string:
    return "RATE"

/** The UBX-CFG-SBAS message. */
class CfgSbas extends Message:
  static ID ::= 0x16
  constructor.get:
    super Message.CFG ID #[]

  constructor --scanmode=0b1_00000000_01001000:
    super Message.CFG ID (ByteArray 8)
    LITTLE_ENDIAN.put_uint8 payload 0 0b00000001
    LITTLE_ENDIAN.put_uint8 payload 1 0b00000011
    LITTLE_ENDIAN.put_uint8 payload 2 3           // Obsolete value.
    LITTLE_ENDIAN.put_uint8 payload 3 0
    LITTLE_ENDIAN.put_uint32 payload 4 scanmode

  id_string_ -> string:
    return "SBAS"

/** The UBX-MGA-INI-TIME-UTC message. */
class MgaIniTimeUtc extends Message:
  static ID ::= 0x40
  constructor --time/Time=Time.now --second_accuracy=0 --nanosecond_accuracy=200_000_000:
    super Message.MGA ID (ByteArray 24)
    type := 0x10
    version := 0x00
    time_info := time.utc
    LITTLE_ENDIAN.put_uint8  payload 0  type
    LITTLE_ENDIAN.put_uint8  payload 1  version
    LITTLE_ENDIAN.put_uint8  payload 2  0x00            // Time reference.
    LITTLE_ENDIAN.put_uint8  payload 3  18              // Number of leap seconds since 1980 (18 as of Dec 31, 2016 - might be updated again in 2020)
    LITTLE_ENDIAN.put_uint16 payload 4  time_info.year
    LITTLE_ENDIAN.put_uint8  payload 6  time_info.month
    LITTLE_ENDIAN.put_uint8  payload 7  time_info.day
    LITTLE_ENDIAN.put_uint8  payload 8  time_info.h
    LITTLE_ENDIAN.put_uint8  payload 9  time_info.m
    LITTLE_ENDIAN.put_uint8  payload 10 time_info.s
    LITTLE_ENDIAN.put_uint8  payload 11 0               // Reserved.
    LITTLE_ENDIAN.put_uint32 payload 12 time_info.ns
    LITTLE_ENDIAN.put_uint16 payload 16 second_accuracy
    LITTLE_ENDIAN.put_uint16 payload 18 0               // Reserved.
    LITTLE_ENDIAN.put_uint32 payload 20 nanosecond_accuracy

  id_string_ -> string:
    return "INI-TIME_UTC"

/** The UBX-MGA-INI-POS-LLH message. */
class MgaIniPosLlh extends Message:
  static ID ::= 0x40
  constructor --latitude/int --longitude/int --altitude/int --accuracy_cm/int:
    super Message.MGA ID (ByteArray 20: 0)
    type := 0x01
    version := 0x00
    LITTLE_ENDIAN.put_uint8 payload 0 type
    LITTLE_ENDIAN.put_int32 payload 4 latitude
    LITTLE_ENDIAN.put_int32 payload 8 longitude
    LITTLE_ENDIAN.put_int32 payload 12 altitude
    LITTLE_ENDIAN.put_uint32 payload 16 accuracy_cm

  id_string_ -> string:
    return "INI-POS_LLH"


/** The UBX-CFG-RST message. */
class CfgRst extends Message:
  static ID ::= 0x04
  // Default clear_sections is a cold start, 0xFFFF is a controlled software reset.
  constructor --clear_sections=0xFFFF --reset_mode=2:
    super Message.CFG ID (ByteArray 4)
    LITTLE_ENDIAN.put_uint16 payload 0 clear_sections
    LITTLE_ENDIAN.put_uint8 payload 2 reset_mode
    LITTLE_ENDIAN.put_uint8 payload 3 0

  id_string_ -> string:
    return "RST"

/** The UBX-RXM-PMREQ message. */
class RxmPmreq extends Message:
  static ID ::= 0x41
  // Put the GPS into backup mode.
  constructor --time=0:
    super Message.RXM ID (ByteArray 16)
    LITTLE_ENDIAN.put_uint32 payload 4 time
    LITTLE_ENDIAN.put_int32  payload 8 0b10  // Configuration flag
    LITTLE_ENDIAN.put_int32  payload 12 0  // Configuration flag

  id_string_ -> string:
    return "PMREQ"

/** The UBX-CFG-PMS message. */
class CfgPms extends Message:
  static ID ::= 0x86
  constructor.get:
    super Message.CFG ID #[]

  // Set settings.
  constructor:
    super Message.CFG ID (ByteArray 8: 0)

  id_string_ -> string:
    return "PMS"

/** The UBX-CFG-RXM message. */
class CfgRxm extends Message:
  static ID ::= 0x11
  // Set power mode.
  constructor --mode=1:
    super Message.CFG ID (ByteArray 2)
    payload[1] = mode

  constructor.get:
    super Message.CFG ID #[]

  id_string_ -> string:
    return "RXM"

/** The UBX-MON-HW message. */
class MonHw extends Message:
  static ID ::= 0x09
  constructor:
    super Message.MON ID #[]

  id_string_ -> string:
    return "HW"

/** The UBX-CFG-PM2 message. */
class CfgPm2 extends Message:
  static ID ::= 0x3b
  constructor.get:
    super Message.CFG ID #[]

  // Set advanced power settings.
  constructor:
    super Message.CFG ID (ByteArray 44)
    payload[0] = 0x01
    payload[2] = 1
    LITTLE_ENDIAN.put_uint32 payload 4 0
    LITTLE_ENDIAN.put_uint32 payload 8 0
    LITTLE_ENDIAN.put_uint32 payload 12 10000
    LITTLE_ENDIAN.put_uint32 payload 16 10000
    LITTLE_ENDIAN.put_uint16 payload 20 0
    LITTLE_ENDIAN.put_uint16 payload 22 0

  id_string_ -> string:
    return "PM2"

/** The UBX-NAV-POSLLH message. */
// https://www.u-blox.com/en/docs/UBX-13003221#%5B%7B%22num%22%3A1021%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C748.35%2Cnull%5D
class NavPosllh extends Message:
  static ID ::= 0x02

  constructor packet/Message:
    super packet.clazz packet.id packet.payload

  constructor.poll:
    super Message.NAV ID #[]

  id_string_ -> string:
    return "POSLLH"

  static is_instance packet/Message -> bool:
    return packet.clazz == Message.NAV and packet.id == ID

  is_valid -> bool:
    return is_instance this

  lon -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 4
  lat -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 8
  height -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 12
  height_msl -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 16
  horizontal_acc -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint32 payload 20
  vertical_acc -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint32 payload 24


/** The UBX-NAV-PBT message. */
// https://www.u-blox.com/en/docs/UBX-13003221#%5B%7B%22num%22%3A1021%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C351.5%2Cnull%5D
class NavPvt extends Message:
  static ID ::= 0x07

  constructor packet/Message:
    super packet.clazz packet.id packet.payload

  constructor.poll:
    super Message.NAV ID #[]

  id_string_ -> string:
    return "PVT"

  static is_instance packet/Message -> bool:
    return packet.clazz == Message.NAV and packet.id == ID

  is_valid -> bool:
    return is_instance this

  is_gnss_fix -> bool:
    return (flags & 0b00000001) != 0

  static FIX_TYPE_UNKNOWN ::= 0
  static FIX_TYPE_DEAD ::= 1
  static FIX_TYPE_2D ::= 2
  static FIX_TYPE_3D ::= 3
  static FIX_TYPE_GNNS_DEAD ::= 4
  static FIX_TYPE_TIME_ONLY ::= 5

  utc_time -> Time:
    return Time.utc year month day hours minutes seconds --ns=nanoseconds

  year -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint16 payload 4
  month -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 6
  day -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 7
  hours -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 8
  minutes -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 9
  seconds -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 10
  valid -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 11
  time_acc -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint32 payload 12
  nanoseconds -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 16
  fix_type -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 20
  flags -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 21
  flags2 -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 22
  num_sv -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 23
  lon -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 24
  lat -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 28
  height -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 32
  height_msl -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 36
  horizontal_acc -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint32 payload 40
  vertical_acc -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint32 payload 44

/** The UBX-NAV-STATUS message. */
// https://www.u-blox.com/en/docs/UBX-13003221#%5B%7B%22num%22%3A1057%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C841.89%2Cnull%5D
class NavStatus extends Message:
  static ID ::= 0x03

  constructor packet/Message:
    super packet.clazz packet.id packet.payload

  constructor.poll:
    super Message.NAV ID #[]

  id_string_ -> string:
    return "STATUS"

  static is_instance packet/Message -> bool:
    return packet.clazz == Message.NAV and packet.id == ID

  is_valid -> bool:
    return is_instance this

  time_to_first_fix -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint32 payload 8

/** The UBX-NAV-SAT message. */
// https://www.u-blox.com/en/docs/UBX-13003221#%5B%7B%22num%22%3A1039%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C841.89%2Cnull%5D
class NavSat extends Message:
  static ID ::= 0x35

  constructor packet/Message:
    super packet.clazz packet.id packet.payload

  constructor.poll:
    super Message.NAV ID #[]

  id_string_ -> string:
    return "SAT"

  static is_instance packet/Message -> bool:
    return packet.clazz == Message.NAV and packet.id == ID

  is_valid -> bool:
    return is_instance this

  satellite_count -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 5

  satellite_data index -> SatelliteData?:
    assert: not payload.is_empty
    if not index < satellite_count: return null
    return SatelliteData index payload

/** The satellite data produced by the $NavSat messages. */
class SatelliteData:
  index/int

  gnss_id/int
  sv_id/int
  cno/int

  quality/int
  orbit_source/int
  alm_avail/bool
  ano_avail/bool

  constructor .index payload/ByteArray:
    offset ::= index * 12
    gnss_id = LITTLE_ENDIAN.uint8 payload offset + 8
    sv_id = LITTLE_ENDIAN.uint8 payload offset + 9
    cno = LITTLE_ENDIAN.uint8 payload offset + 10

    flags ::= LITTLE_ENDIAN.uint32 payload offset + 16
    quality = flags & 0x003
    orbit_source = (flags & 0x700)  >> 8
    alm_avail = (flags & 0x800)  >> 11 == 1
    ano_avail = (flags & 0x1000) >> 12 == 1

  stringify -> string:
    codes := ""
    if alm_avail: codes += "A"
    if ano_avail: codes += "N"
    return "$index|$gnss_id|$sv_id|$cno|$quality|$orbit_source|$codes"

/** The UBX-NAV-TIMEUTC message. */
// https://www.u-blox.com/en/docs/UBX-13003221#%5B%7B%22num%22%3A1105%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C683.15%2Cnull%5D
class NavTimeutc extends Message:
  static ID ::= 0x21

  constructor packet/Message:
    super packet.clazz packet.id packet.payload

  constructor.poll:
    super Message.NAV ID #[]

  id_string_ -> string:
    return "TIMEUTC"

  static is_instance packet/Message -> bool:
    return packet.clazz == Message.NAV and packet.id == ID

  is_valid -> bool:
    return is_instance this

  accuracy -> Duration?:
    assert: not payload.is_empty
    value := LITTLE_ENDIAN.uint32 payload 4
    if value == UINT32_MAX: return null
    return Duration --ns=value
