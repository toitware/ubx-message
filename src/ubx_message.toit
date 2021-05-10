// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import reader show *
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

  static NAV ::= 0x01
  static RXM ::= 0x02
  static INF ::= 0x04
  static ACK ::= 0x05
  static CFG ::= 0x06
  static MON ::= 0x0A
  static MGA ::= 0x13

  static PACK_CLASSES ::= {NAV: "NAV", RXM: "RXM", INF: "INF", ACK: "ACK", CFG: "CFG", MON: "MON", MGA: "MGA"}

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
    return "0x$(%02x id)"

  message_type -> string:
    return "UBX-$class_string-$id_string"

  is_ubx_nav_pos_llh -> bool:
    return UBXNavPosllh.is_instance this

  ubx_nav_pos_llh -> UBXNavPosllh:
    return UBXNavPosllh this

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

compute_checksum msg/ByteArray [callback]:
  ck_a := 0
  ck_b := 0
  msg = msg[2..msg.size - 2]
  msg.size.repeat: | i |
    ck_a = (ck_a + msg[i]) & 0xff
    ck_b = (ck_b + ck_a) & 0xff
  callback.call ck_a ck_b

is_valid_frame frame/ByteArray -> bool:
  ck_a ::= frame[frame.size - 2]
  ck_b ::= frame[frame.size - 1]
  compute_checksum frame: | a b |
    return ck_a == a and ck_b == b
  return false

message r/Reader -> Message?:
  reader := BufferedReader r
  if (reader.byte 0) != 0xb5 or (reader.byte 1) != 0x62: return null

  // Verify length and get full the packet.
  length ::= (reader.byte 4) | (((reader.byte 5) & 0xff) << 8)
  if length < 0 or length > 512: return null
  frame ::= reader.bytes length + 8

  // Verify the checksum.
  if not is_valid_frame frame: return null

  msg_class ::= frame[2]
  msg_id    ::= frame[3]
  payload   ::= frame[6..length + 6]
  reader.skip length + 8
  return Message msg_class msg_id payload


class CfgMsg extends Message:
  static ID ::= 0x01

  constructor --msg_class --msg_id --rate:
    super Message.CFG ID #[msg_class, msg_id, rate]

  id_string -> string:
    return "MSG"

/*
Spec:
https://www.u-blox.com/en/docs/UBX-13003221#%5B%7B%22num%22%3A667%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C379.84%2Cnull%5D
*/
class CfgGnss extends Message:
  static ID ::= 0x3e

  constructor.get:
    super Message.CFG ID #[]

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

  id_string -> string:
    return "GNSS"

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

  id_string -> string:
    return "NAVX5"

class CfgNav5 extends Message:
  static ID ::= 0x24

  constructor.get:
    super Message.CFG ID #[]

  id_string -> string:
    return "NAV5"

class CfgRate extends Message:
  static ID ::= 0x08

  constructor.get:
    super Message.CFG ID #[]

  id_string -> string:
    return "RATE"

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

  id_string -> string:
    return "SBAS"

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

  id_string -> string:
    return "INI-TIME_UTC"

class NavTimeutcPoll extends Message:
  static ID ::= 0x21
  constructor:
    super Message.NAV ID #[]

  id_string -> string:
    return "TIMEUTC"

class MgaIniPosLLH extends Message:
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

  id_string -> string:
    return "INI-POS_LLH"

class NavPosLlh extends Message:
  static ID ::= 0x02
  constructor.poll:
    super Message.NAV ID #[]

  id_string -> string:
    return "POSLLH"

class NavStatus extends Message:
  static ID ::= 0x03
  constructor.poll:
    super Message.NAV ID #[]

  id_string -> string:
    return "STATUS"

class NavPvt extends Message:
  static ID ::= 0x07
  constructor.poll:
    super Message.NAV ID #[]

  id_string -> string:
    return "PVT"

class CfgRst extends Message:
  static ID ::= 0x04
  // Default clear_sections is a cold start, 0xFFFF is a controlled software reset.
  constructor --clear_sections=0xFFFF --reset_mode=2:
    super Message.CFG ID (ByteArray 4)
    LITTLE_ENDIAN.put_uint16 payload 0 clear_sections
    LITTLE_ENDIAN.put_uint8 payload 2 reset_mode
    LITTLE_ENDIAN.put_uint8 payload 3 0

  id_string -> string:
    return "RST"

class RxmPmreq extends Message:
  static ID ::= 0x41
  // Put the GPS into backup mode.
  constructor --time=0:
    super Message.RXM ID (ByteArray 16)
    LITTLE_ENDIAN.put_uint32 payload 4 time
    LITTLE_ENDIAN.put_int32  payload 8 0b10  // Configuration flag
    LITTLE_ENDIAN.put_int32  payload 12 0  // Configuration flag

  id_string -> string:
    return "PMREQ"

class CfgPms extends Message:
  static ID ::= 0x86
  constructor.get:
    super Message.CFG ID #[]

  // Set settings.
  constructor:
    super Message.CFG ID (ByteArray 8: 0)

  id_string -> string:
    return "PMS"

class CfgRxm extends Message:
  static ID ::= 0x11
  // Set power mode.
  constructor --mode=1:
    super Message.CFG ID (ByteArray 2)
    payload[1] = mode

  constructor.get:
    super Message.CFG ID #[]

  id_string -> string:
    return "RXM"

class MonHw extends Message:
  static ID ::= 0x09
  constructor:
    super Message.MON ID #[]

  id_string -> string:
    return "HW"

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

  id_string -> string:
    return "PM2"

/*
Spec:
https://www.u-blox.com/en/docs/UBX-13003221#%5B%7B%22num%22%3A1021%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C748.35%2Cnull%5D
*/
class UBXNavPosllh extends Message:
  static ID ::= 0x02

  payload_/ByteArray ::= ?

  constructor packet/Message:
    payload_ = packet.payload
    super packet.clazz packet.id packet.payload

  id_string -> string:
    return "POSLLH"

  static is_instance packet/Message -> bool:
    return packet.clazz == Message.NAV and packet.id == ID

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

  id_string -> string:
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

  id_string -> string:
    return "STATUS"

  static is_instance packet/Message -> bool:
    return packet.clazz == Message.NAV and packet.id == ID

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

  id_string -> string:
    return "SAT"

  static is_instance packet/Message -> bool:
    return packet.clazz == Message.NAV and packet.id == ID

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

  constructor packet/Message:
    super packet.clazz packet.id packet.payload

  constructor.poll:
    super Message.NAV ID #[]

  id_string -> string:
    return "TIMEUTC"

  static is_instance packet/Message -> bool:
    return packet.clazz == Message.NAV and packet.id == ID

  is_valid -> bool:
    return is_instance this

  accuracy -> Duration?:
    value := LITTLE_ENDIAN.uint32 payload 4
    if value == UINT32_MAX: return null
    return Duration --ns=value
