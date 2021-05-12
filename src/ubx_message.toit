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
