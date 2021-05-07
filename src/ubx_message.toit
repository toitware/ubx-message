// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

/**
UBX messages from the UBX protocol for communicating with the GNSS receivers
  in the ublox Max-M* series.

A description of the UBX protocol can be found here: https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
*/

import binary show LITTLE_ENDIAN UINT32_MAX

class Message:
  clazz /int
  id /int
  payload /ByteArray ::= #[]

  static UBX_NAV ::= 0x01
  static UBX_RXM ::= 0x02
  static UBX_INF ::= 0x04
  static UBX_ACK ::= 0x05
  static UBX_CFG ::= 0x06
  static UBX_MON ::= 0x0A
  static UBX_MGA ::= 0x13

  static PACK_CLASSES ::= {UBX_NAV: "NAV", UBX_INF: "INF", UBX_ACK: "ACK", UBX_CFG: "CFG", UBX_MON: "MON", UBX_MGA: "MGA"}

  static PACK_IDS ::= {
    UBX_NAV: {0x02: "POSLLH", 0x03: "STATUS", 0x07: "PVT", 0x21: "TIMEUTC",  0x35: "SAT"},
    UBX_INF: {0x03: "TEST"},
    UBX_ACK: {0x00: "NAK", 0x01: "ACK"},
    UBX_CFG: {0x01: "MSG", 0x04: "RST", 0x08: "RATE", 0x11: "RXM", 0x23: "NAVX5", 0x24: "NAV5", 0x3b: "PM2", 0x3e: "GNSS", 0x86: "PMS"},
    UBX_MON: {0x09: "HW"},
    UBX_MGA: {0x40: "INI", 0x60: "ACK"},
  }

  constructor.MGA_INI_POS_LLH --latitude/int --longitude/int --altitude/int --accuracy_cm/int:
    clazz = UBX_MGA
    id = 0x40
    payload = ByteArray 20: 0
    type := 0x01
    version := 0x00
    LITTLE_ENDIAN.put_uint8 payload 0 type
    LITTLE_ENDIAN.put_int32 payload 4 latitude
    LITTLE_ENDIAN.put_int32 payload 8 longitude
    LITTLE_ENDIAN.put_int32 payload 12 altitude
    LITTLE_ENDIAN.put_uint32 payload 16 accuracy_cm

  constructor.NAV_TIMEUTC_poll:
    clazz = UBX_NAV
    id = 0x21
    payload = ByteArray 0

  constructor.NAV_POS_LLH_poll:
    clazz = UBX_NAV
    id = 0x02
    payload = ByteArray 0

  constructor.NAV_STATUS_poll:
    clazz = UBX_NAV
    id = 0x03
    payload = ByteArray 0

  constructor.NAV_PVT_poll:
    clazz = UBX_NAV
    id = 0x07
    payload = ByteArray 0

  // Default clear_sections is a cold start, 0xFFFF is a controlled software reset.
  constructor.CFG_RST --clear_sections=0xFFFF --reset_mode=2:
    clazz = UBX_CFG
    id = 0x04
    payload = ByteArray 4
    LITTLE_ENDIAN.put_uint16 payload 0 clear_sections
    LITTLE_ENDIAN.put_uint8 payload 2 reset_mode
    LITTLE_ENDIAN.put_uint8 payload 3 0

  // Put the GPS into backup mode.
  constructor.RXM_PMREQ --time=0:
    clazz = UBX_RXM
    id = 0x41
    payload = ByteArray 16
    LITTLE_ENDIAN.put_uint32 payload 4 time
    LITTLE_ENDIAN.put_int32  payload 8 0b10  // Configuration flag
    LITTLE_ENDIAN.put_int32  payload 12 0  // Configuration flag

  // Set settings.
  constructor.CFG_PMS --get=false:
    clazz = 0x06
    id = 0x86
    if get:
      payload = ByteArray 0
    else:
      payload = ByteArray 8: 0

  // Set advanced power settings.
  constructor.CFG_PM2 --get=false:
    clazz = 0x06
    id = 0x3B
    if get:
      payload = ByteArray 0
    else:
      payload = ByteArray 44
      payload[0] = 0x01
      payload[2] = 1
      LITTLE_ENDIAN.put_uint32 payload 4 0
      LITTLE_ENDIAN.put_uint32 payload 8 0
      LITTLE_ENDIAN.put_uint32 payload 12 10000
      LITTLE_ENDIAN.put_uint32 payload 16 10000
      LITTLE_ENDIAN.put_uint16 payload 20 0
      LITTLE_ENDIAN.put_uint16 payload 22 0

  // Set power mode.
  constructor.CFG_RXM --mode=1:
    clazz = 0x06
    id = 0x11
    payload = ByteArray 2
    payload[1] = mode

  constructor.MON_HW:
    clazz = UBX_MON
    id = 0x09
    payload = ByteArray 0

  constructor .clazz .id .payload:

  to_byte_array -> ByteArray:
    bytes := ByteArray 8 + payload.size
    bytes[0] = 0xB5
    bytes[1] = 0x62
    bytes[2] = clazz
    bytes[3] = id
    LITTLE_ENDIAN.put_uint16 bytes 4 payload.size
    bytes.replace 6 payload
    compute_checksum bytes: | ck_a ck_b |
      bytes[bytes.size - 2] = ck_a
      bytes[bytes.size - 1] = ck_b
    return bytes

  class_string -> string:
    PACK_CLASSES.get clazz
      --if_present=:
        return it
      --if_absent=:
        return "0x$(%02x clazz)"
    unreachable

  id_string -> string:
    absent_block ::=: return "0x$(%02x id)"
    ids := PACK_IDS.get clazz --if_absent=absent_block
    ids.get id
      --if_present=:
        return it
      --if_absent=absent_block
    unreachable

  message_type -> string:
    return "UBX-$class_string-$id_string"

  is_ubx_nav_pos_llh -> bool:
    return UBXNavPosLLH.is_instance this

  ubx_nav_pos_llh -> UBXNavPosLLH:
    return UBXNavPosLLH this

  is_ubx_nav_pvt -> bool:
    return UbxNavPvt.is_instance this

  ubx_nav_pvt -> UbxNavPvt:
    return UbxNavPvt this

  is_ubx_nav_status -> bool:
    return UbxNavStatus.is_instance this

  ubx_nav_status -> UbxNavStatus:
    return UbxNavStatus this

  is_ubx_nav_sat -> bool:
    return UbxNavSat.is_instance this

  ubx_nav_sat -> UbxNavSat:
    return UbxNavSat this

  is_ubx_nav_timeutc -> bool:
    return UbxNavTimeUtc.is_instance this

  ubx_nav_timeutc -> UbxNavTimeUtc:
    return UbxNavTimeUtc this

compute_checksum msg/ByteArray --from=2 --except=2 [callback]:
  ck_a := 0
  ck_b := 0
  for i := from; i < msg.size - except; i++:
    ck_a = (ck_a + msg[i]) & 0xff
    ck_b = (ck_b + ck_a) & 0xff
  callback.call ck_a ck_b

class CfgMsg extends Message:
  static ID ::= 0x01

  constructor --msg_class --msg_id --rate:
    super Message.UBX_CFG ID #[msg_class, msg_id, rate]

/*
Spec:
https://www.u-blox.com/en/docs/UBX-13003221#%5B%7B%22num%22%3A667%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C379.84%2Cnull%5D
*/
class CfgGnss extends Message:
  static ID ::= 0x3e

  constructor --get=false
      --gps_reserved_channels=8 --gps_max_channels=16
      --sbas_reserved_channels=1 --sbas_max_channels=3
      --qzss_reserved_channels=0 --qzss_max_channels=3
      --glonas_reserved_channels=8 --glonas_max_channels=14:

    pl := #[]
    // U-blox recommend using QZSS whenever GPS is active at the same time.
    if not get:
      pl = #[
        0, 0, 32, 7,
        0, gps_reserved_channels, gps_max_channels, 0, 1, 0, 1, 1,
        1, sbas_reserved_channels,  sbas_max_channels, 0, 1, 0, 1, 1,
        2, 0,  0, 0, 0, 0, 1, 1,
        3, 0,  0, 0, 0, 0, 1, 1,
        4, 0,  0, 0, 0, 0, 1, 1,
        5, qzss_reserved_channels, qzss_max_channels, 0, 1, 0, 1, 1,
        6, glonas_reserved_channels, glonas_max_channels, 0, 1, 0, 1, 1
      ]
    super Message.UBX_CFG ID pl

class CfgNavx5 extends Message:
  static ID ::= 0x23

  constructor --ack_aiding/bool --get=false:
    pl := #[]
    if not get:
      // Payload copied from chip output.
      pl = #[
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
    super Message.UBX_CFG ID pl

class CfgNav5 extends Message:
  static ID ::= 0x24

  constructor.CFG_NAV5 --get=false:
    super Message.UBX_CFG ID #[]

class CfgRxm extends Message:
  static ID ::= 0x11

  constructor --get=false:
    super Message.UBX_CFG ID #[]

class CfgRate extends Message:
  static ID ::= 0x08
  constructor --get=false:
    super Message.UBX_CFG ID #[]


class CfgSbas extends Message:
  static ID ::= 0x16
  constructor --get=false --scanmode=0b1_00000000_01001000:
    pl := #[]
    if not get:
      pl = ByteArray 8
      LITTLE_ENDIAN.put_uint8 pl 0 0b00000001
      LITTLE_ENDIAN.put_uint8 pl 1 0b00000011
      LITTLE_ENDIAN.put_uint8 pl 2 3           // Obsolete value.
      LITTLE_ENDIAN.put_uint8 pl 3 0
      LITTLE_ENDIAN.put_uint32 pl 4 scanmode
    super Message.UBX_CFG ID pl

class MgaIniTimeUtc extends Message:
  static ID ::= 0x40
  constructor --time/Time=Time.now --second_accuracy=0 --nanosecond_accuracy=200_000_000:
    super Message.UBX_MGA ID (ByteArray 24)
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

/*
Spec:
https://www.u-blox.com/en/docs/UBX-13003221#%5B%7B%22num%22%3A1021%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C748.35%2Cnull%5D
*/
class UBXNavPosLLH extends Message:
  static ID ::= 0x02

  payload_/ByteArray ::= ?

  constructor packet/Message:
    payload_ = packet.payload
    super packet.clazz packet.id packet.payload

  static is_instance packet/Message -> bool:
    return packet.clazz == Message.UBX_NAV and packet.id == ID

  is_valid -> bool:
    return is_instance this

  lon -> int:
    return LITTLE_ENDIAN.int32 payload_ 4
  lat -> int:
    return LITTLE_ENDIAN.int32 payload_ 8
  height -> int:
    return LITTLE_ENDIAN.int32 payload_ 12
  height_msl -> int:
    return LITTLE_ENDIAN.int32 payload_ 16
  horizontal_acc -> int:
    return LITTLE_ENDIAN.uint32 payload_ 20
  vertical_acc -> int:
    return LITTLE_ENDIAN.uint32 payload_ 24


/*
Spec:
https://www.u-blox.com/en/docs/UBX-13003221#%5B%7B%22num%22%3A1021%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C351.5%2Cnull%5D
*/
class UbxNavPvt extends Message:
  static ID ::= 0x07

  payload_/ByteArray ::= ?

  constructor packet/Message:
    payload_ = packet.payload
    super packet.clazz packet.id packet.payload

  static is_instance packet/Message -> bool:
    return packet.clazz == Message.UBX_NAV and packet.id == ID

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

  year -> int:
    return LITTLE_ENDIAN.uint16 payload_ 4
  month -> int:
    return LITTLE_ENDIAN.uint8 payload_ 6
  day -> int:
    return LITTLE_ENDIAN.uint8 payload_ 7
  hours -> int:
    return LITTLE_ENDIAN.uint8 payload_ 8
  minutes -> int:
    return LITTLE_ENDIAN.uint8 payload_ 9
  seconds -> int:
    return LITTLE_ENDIAN.uint8 payload_ 10
  valid -> int:
    return LITTLE_ENDIAN.uint8 payload_ 11
  time_acc -> int:
    return LITTLE_ENDIAN.uint32 payload_ 12
  nanoseconds -> int:
    return LITTLE_ENDIAN.int32 payload_ 16
  fix_type -> int:
    return LITTLE_ENDIAN.uint8 payload_ 20
  flags -> int:
    return LITTLE_ENDIAN.uint8 payload_ 21
  flags2 -> int:
    return LITTLE_ENDIAN.uint8 payload_ 22
  num_sv -> int:
    return LITTLE_ENDIAN.uint8 payload_ 23
  lon -> int:
    return LITTLE_ENDIAN.int32 payload_ 24
  lat -> int:
    return LITTLE_ENDIAN.int32 payload_ 28
  height -> int:
    return LITTLE_ENDIAN.int32 payload_ 32
  height_msl -> int:
    return LITTLE_ENDIAN.int32 payload_ 36
  horizontal_acc -> int:
    return LITTLE_ENDIAN.uint32 payload_ 40
  vertical_acc -> int:
    return LITTLE_ENDIAN.uint32 payload_ 44

/*
Spec:
https://www.u-blox.com/en/docs/UBX-13003221#%5B%7B%22num%22%3A1057%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C841.89%2Cnull%5D
*/
class UbxNavStatus extends Message:
  static ID ::= 0x03

  payload_/ByteArray ::= ?

  constructor packet/Message:
    payload_ = packet.payload
    super packet.clazz packet.id packet.payload

  static is_instance packet/Message -> bool:
    return packet.clazz == Message.UBX_NAV and packet.id == ID

  is_valid -> bool:
    return is_instance this

  time_to_first_fix -> int:
    return LITTLE_ENDIAN.uint32 payload_ 8


/*
Spec:
https://www.u-blox.com/en/docs/UBX-13003221#%5B%7B%22num%22%3A1039%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C841.89%2Cnull%5D
*/
class UbxNavSat extends Message:
  static ID ::= 0x35

  payload_/ByteArray ::= ?

  constructor packet/Message:
    payload_ = packet.payload
    super packet.clazz packet.id packet.payload

  static is_instance packet/Message -> bool:
    return packet.clazz == Message.UBX_NAV and packet.id == ID

  is_valid -> bool:
    return is_instance this

  satellite_count -> int:
    return LITTLE_ENDIAN.uint8 payload_ 5

  satellite_data index -> SatelliteData?:
    if not index < satellite_count: return null
    return SatelliteData index payload_

class SatelliteData:
  index ::= 0

  gnss_id ::= 0
  sv_id ::= 0
  cno ::= 0

  quality ::= 0
  orbit_source ::= 0
  alm_avail ::= 0
  ano_avail ::= 0

  constructor .index payload/ByteArray:
    offset ::= index * 12
    gnss_id = LITTLE_ENDIAN.uint8 payload offset + 8
    sv_id = LITTLE_ENDIAN.uint8 payload offset + 9
    cno = LITTLE_ENDIAN.uint8 payload offset + 10

    flags ::= LITTLE_ENDIAN.uint32 payload offset + 16
    quality = flags & 0x003
    orbit_source = (flags & 0x700)  >> 8
    alm_avail = (flags & 0x800)  >> 11
    ano_avail = (flags & 0x1000) >> 12

  stringify -> string:
    codes := ""
    if alm_avail: codes += "A"
    if ano_avail: codes += "N"
    // TODO(kasper): Make this output a whole lot prettier and easier to parse.
    return "$index|$gnss_id|$sv_id|$cno|$quality|$orbit_source|$codes"

/*
Spec:
https://www.u-blox.com/en/docs/UBX-13003221#%5B%7B%22num%22%3A1105%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C683.15%2Cnull%5D
*/
class UbxNavTimeUtc extends Message:
  static ID ::= 0x21

  payload_/ByteArray ::= ?

  constructor packet/Message:
    payload_ = packet.payload
    super packet.clazz packet.id packet.payload

  static is_instance packet/Message -> bool:
    return packet.clazz == Message.UBX_NAV and packet.id == ID

  is_valid -> bool:
    return is_instance this

  accuracy -> Duration?:
    value := LITTLE_ENDIAN.uint32 payload_ 4
    if value == UINT32_MAX: return null
    return Duration --ns=value
