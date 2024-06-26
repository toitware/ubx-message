// Copyright (C) 2021 Toitware ApS. All rights reserved.
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

import io
import io show LITTLE_ENDIAN
import reader as old-reader

/**
A UBX message from the UBX data protocol.
*/
class Message:
  /** Maximum size of an encoded message. */
  static MAX_MESSAGE_SIZE_ ::= 2048

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

  /** Map from class bytes to their string representations. */
  static PACK_CLASSES ::= {NAV: "NAV", RXM: "RXM", INF: "INF", ACK: "ACK", \
    CFG: "CFG", UPD: "UPD", MON: "MON", AID: "AID", TIM: "TIM", ESF: "ESF", \
    MGA: "MGA", LOG: "LOG", SEC: "SEC", HNR: "HNR"}

  static INVALID_UBX_MESSAGE_ ::= "INVALID UBX MESSAGE"
  static RESERVED_ ::= 0

  /** Constructs a UBX message with the given $cls, $id, and $payload. */
  constructor.private_ .cls .id .payload:

  /**
  Constructs a UBX message with the given $cls, $id, and $payload.

  If message is implemented in this package, then it returns the appropriate
    sub-class.
  */
  constructor cls id payload:
    if cls == Message.NAV:
      if id == NavPvt.ID:
        return NavPvt.private_ payload
      else if id == NavStatus.ID:
        return NavStatus.private_ payload
      else if id == NavSat.ID:
        return NavSat.private_ payload

    return Message.private_ cls id payload

  /**
  Constructs a UBX message from the given $bytes.

  The $bytes must be a valid UBX message (contain the sync bytes and a
    valid checksum).
  */
  constructor.from_bytes bytes/ByteArray:
    if not is_valid_frame_ bytes: throw INVALID_UBX_MESSAGE_
    cls = bytes[2]
    id = bytes[3]
    payload = bytes[4 .. bytes.size - 2]

  /**
  Constructs a UBX message from the given $reader.

  The $reader must be able to provide a valid UBX frame.

  If message is implemented in this package, then it returns the appropriate
    sub-class.

  The $reader should be an $io.Reader, but an $old-reader.Reader is also accepted
    for backwards compatibility. The use of $old-reader.Reader is deprecated and
    will be removed in a future release.
  */
  constructor.from_reader reader/old-reader.Reader:
    io-reader/io.Reader := reader is io.Reader ? reader as io.Reader : io.Reader.adapt reader

    if (io-reader.peek-byte 0) != 0xb5 or (io-reader.peek-byte 1) != 0x62: throw INVALID_UBX_MESSAGE_

    // Verify the length and get full the packet.
    length ::= (io-reader.peek-byte 4) | (((io-reader.peek-byte 5) & 0xff) << 8)
    if not 0 <= length <= MAX_MESSAGE_SIZE_: throw INVALID_UBX_MESSAGE_
    frame ::= io-reader.peek-bytes length + 8

    // Verify the checksum.
    if not is_valid_frame_ frame: throw INVALID_UBX_MESSAGE_

    msg_class ::= frame[2]
    msg_id    ::= frame[3]
    payload   ::= frame[6..length + 6]
    io-reader.skip length + 8
    return Message msg_class msg_id payload

  static is_valid_frame_ frame/ByteArray -> bool:
    // Check the sync bytes.
    if frame[0] != 0xb5 or frame[1] != 0x62: throw INVALID_UBX_MESSAGE_

    // Check the payload length.
    length ::= LITTLE_ENDIAN.uint16 frame 4
    if not 0 <= length <= MAX_MESSAGE_SIZE_: return false

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
    bytes[2] = cls
    bytes[3] = id
    LITTLE_ENDIAN.put_uint16 bytes 4 payload.size
    bytes.replace 6 payload
    compute_checksum_ bytes: | ck_a ck_b |
      bytes[bytes.size - 2] = ck_a
      bytes[bytes.size - 1] = ck_b
    return bytes

  class_string_ -> string:
    return PACK_CLASSES.get cls --if_absent=:
      return "0x$(%02x cls)"

  id_string_ -> string:
    return "0x$(%02x id)"

  /** See $super. */
  stringify -> string:
    return "UBX-$class_string_-$id_string_"

/**
The UBX-ACK-ACK message.

Contains the class ID and message ID of the acknowledged message.
*/
class AckAck extends Message:
  /** The UBX-ACK-ACK message ID. */
  static ID ::= 0x01

  /** Constructs a dummy acknowledge message. */
  constructor.private_ cls id:
    super.private_ Message.ACK ID #[cls, id]

  /** The class ID of the acknowledged message. */
  class_id -> int:
    return LITTLE_ENDIAN.uint8 payload 0

  /** The message ID  of the acknowledged message. */
  message_id -> int:
    return LITTLE_ENDIAN.uint8 payload 1

  id_string_ -> string:
    return "ACK"


/**
The UBX-ACK-NAK message.

Contains the class ID and message ID of the NAK (not acknowledged) message.
*/
class AckNak extends Message:
  /** The UBX-ACK-NAK message ID. */
  static ID ::= 0x02

  /** Constructs a dummy NAK message. */
  constructor.private_ cls id:
    super.private_ Message.ACK ID #[cls, id]

  /** The class ID of the NAK message. */
  class_id -> int:
    return LITTLE_ENDIAN.uint8 payload 0

  /** The message ID of the NAK message. */
  message_id -> int:
    return LITTLE_ENDIAN.uint8 payload 1

  id_string_ -> string:
    return "NAK"

/**
The UBX-CFG-MSG message.

Configures the rate at which messages are sent by the receiver.
*/
class CfgMsg extends Message:
  /** The UBX-CFG-MSG message ID. */
  static ID ::= 0x01

  /**
  Constructs a configuration message.

  When sent to the receiver, the message with the given $msg_class and
    $msg_id will be sent at the given $rate.
  */
  constructor.message_rate --msg_class --msg_id --rate:
    super.private_ Message.CFG ID #[msg_class, msg_id, rate]

  id_string_ -> string:
    return "MSG"

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
  constructor --clear_sections=0xFFFF --reset_mode=2:
    super.private_ Message.CFG ID (ByteArray 4)
    LITTLE_ENDIAN.put_uint16 payload 0 clear_sections
    LITTLE_ENDIAN.put_uint8  payload 2 reset_mode
    LITTLE_ENDIAN.put_uint8  payload 3 Message.RESERVED_

  id_string_ -> string:
    return "RST"

/**
The UBX-NAV-PVT message.

Navigation, position, velocity, and time solution.
*/
class NavPvt extends Message:
  /** The UBX-NAV-PVT message ID. */
  static ID ::= 0x07

  /** Unknown GNSS fix. */
  static FIX_TYPE_UNKNOWN ::= 0
  /** Dead reckoning only. */
  static FIX_TYPE_DEAD ::= 1
  /** 2D fix. */
  static FIX_TYPE_2D ::= 2
  /** 3D fix. */
  static FIX_TYPE_3D ::= 3
  /** GNSS and dead reckoning. */
  static FIX_TYPE_GNSS_DEAD ::= 4
  /** Time only fix. */
  static FIX_TYPE_TIME_ONLY ::= 5

  /** Constructs a poll UBX-NAV-PVT message. */
  constructor.poll:
    super.private_ Message.NAV ID #[]

  constructor.private_ payload/ByteArray:
    super.private_ Message.NAV ID payload

  id_string_ -> string:
    return "PVT"

  /** Whether this is a GNSS fix. */
  is_gnss_fix -> bool:
    return (flags & 0b00000001) != 0

  /** The time in UTC. */
  utc_time -> Time:
    return Time.utc year month day h m s --ns=ns

  /** The GPS interval time of week of the navigation epoch. */
  itow -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint32 payload 0

  /** The year (UTC). */
  year -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint16 payload 4

  /**
  The month (UTC).
  In the range [1..12].
  */
  month -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 6

  /**
  The day (UTC).
  In the range [1..31].
  */
  day -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 7

  /**
  The hours (UTC).
  In the range [0..23].
  */
  h -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 8

  /**
  The minutes (UTC).
  In the range [0..59].
  */
  m -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 9

  /**
  The seconds (UTC).
  In the range [0..60].
  */
  s -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 10

  /**
  Validity flag.
  See receiver specification for details.
  */
  valid -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 11

  /** Time accuracy estimate in nanoseconds */
  time_acc -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint32 payload 12

  /**
  Fraction of second in nano seconds.
  The fraction may be negative.
  */
  ns -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 16

  /**
  The type of fix.
  One of $FIX_TYPE_UNKNOWN, $FIX_TYPE_DEAD, $FIX_TYPE_2D, $FIX_TYPE_3D, $FIX_TYPE_GNSS_DEAD, $FIX_TYPE_TIME_ONLY.
  */
  fix_type -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 20

  /**
  Fix status flags.
  See receiver specification for details.
  */
  flags -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 21

  /**
  Additional fix status flags.
  See receiver specification for details.
  */
  flags2 -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 22

  /** Number of satellites used for fix. */
  num_sv -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 23

  /** Longitude. */
  lon -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 24

  /** Latitude. */
  lat -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 28

  /** Height above ellipsoid in millimeter. */
  height -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 32

  /** Height above mean sea level in millimeter. */
  height_msl -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 36

  /** Horizontal accuracy in millimeter. */
  horizontal_acc -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint32 payload 40

  /** Vertical accuracty in millimeter. */
  vertical_acc -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint32 payload 44

  /** NED north velocity in millimeters per second. */
  north_vel -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 48

  /** NED east velocity in millimeters per second. */
  east_vel -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 52

  /** NED down velocity in millimeters per second. */
  down_vel -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 56

  /** Ground speed (2D) in millimeters per second. */
  ground_speed -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 60

  /** Heading of motion (2D). */
  heading_of_motion -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 64

  /** Speed accuracy in millimeters per second. */
  speed_acc -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint32 payload 68

  /** Hading accuracy. */
  heading_acc -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint32 payload 72

  /**
  Position DOP.

  Describes how many satellites are directly above the receiver (and not on
    the horizon).
  */
  position_dop -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint16 payload 76

  /**
  Additional flags.
  See receiver specification for details.
  */
  flags3 -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint32 payload 78

  /**
  The heading of the vehicle.
  See receiver specification for details.
  */
  heading_vehicle -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int32 payload 84

  /**
  Magnetic declination.
  See receiver specification for details.
  */
  magnetic_declination -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.int16 payload 88

  /**
  Accuracy of magnetic declination.
  See receiver specification for details.
  */
  magnetic_acc -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint16 payload 90

/**
The UBX-NAV-STATUS message.

The receiver navigation status.
*/
// https://www.u-blox.com/en/docs/UBX-13003221#%5B%7B%22num%22%3A1204%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C559.1%2Cnull%5D
class NavStatus extends Message:
  /** The UBX-NAV-STATUS message ID. */
  static ID ::= 0x03

  /** Unknown GNSS fix. */
  static NO_FIX ::= 0
  /** Dead reckoning only. */
  static DEAD_RECKONING_ONLY ::= 1
  /** 2D fix. */
  static FIX_2D ::= 2
  /** 3D fix. */
  static FIX_3D ::= 3
  /** GPS and dead reckoning. */
  static GPS_DEAD_FIX ::= 4
  /** Time only fix. */
  static TIME_ONLY ::= 5
  /** Constructs a poll UBX-NAV-STATUS message. */
  constructor.poll:
    super.private_ Message.NAV ID #[]

  constructor.private_ payload:
    super.private_ Message.NAV ID payload

  id_string_ -> string:
    return "STATUS"

  /** The GPS interval time of week of the navigation epoch. */
  itow -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint32 payload 0

  /**
  The type of fix.
  One of $NO_FIX, $DEAD_RECKONING_ONLY, $FIX_2D, $FIX_3D, $GPS_DEAD_FIX, $TIME_ONLY.
  */
  gps_fix -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 4

  /**
  Navigation status flags.
  See receiver specification for details.
  */
  flags -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 5

  /**
  Fix status information
  See receiver specification for details.
  */
  fix_stat -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 6

  /**
  Additional status information.
  See receiver specification for details.
  */
  flags2 -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 7

  /** Time to first fix in milliseconds. */
  time_to_first_fix -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint32 payload 8

  /** Milliseconds since startup or reset. */
  msss -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint32 payload 12

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

  id_string_ -> string:
    return "SAT"

  /** The GPS interval time of week of the navigation epoch. */
  itow -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 0

  /** Message version. */
  version -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 4

  /** Number of satellites. */
  num_svs -> int:
    assert: not payload.is_empty
    return LITTLE_ENDIAN.uint8 payload 5

  /**
  The satellite data in the package for the given $index.

  The $index must satisfy 0 <= $index < $num_svs.
  */
  satellite_data index -> SatelliteData:
    assert: not payload.is_empty
    if not 0 <= index < num_svs: throw "INVALID ARGUMENT"
    return SatelliteData index payload

/**
Satellite data for a single satellite.

The satellite data is included in the UBX-NAV-SAT message (See $NavSat).
*/
class SatelliteData:
  /** The index os this data in the original message. */
  index/int
  /** GNSS identifier. */
  gnss_id/int
  /** Satellite identifier. */
  sv_id/int
  /** Carrier to noise ratio. */
  cno/int
  /** Elevation. */
  elev/int
  /** Azimuth. */
  azim/int
  /** Pseudorange residual. */
  pr_res/int

  /**
  Flags. Includes $quality, $orbit_source, $alm_avail, and $ano_avail.
  See receiver specification for details.
  */
  flags/int
  /** Quality indicator. */
  quality/int
  /** Orbit source. */
  orbit_source/int
  /** Almanac available for this satellite. */
  alm_avail/int
  /** AssistanceNow Offline data available for this satellite. */
  ano_avail/int

  /**
  Constructs the satellite data for the given message $payload and with
    the given data.
  */
  constructor .index payload/ByteArray:
    offset ::= index * 12

    gnss_id = LITTLE_ENDIAN.uint8 payload offset + 8
    sv_id = LITTLE_ENDIAN.uint8 payload offset + 9
    cno = LITTLE_ENDIAN.uint8 payload offset + 10
    elev = LITTLE_ENDIAN.uint8 payload offset + 11
    azim = LITTLE_ENDIAN.uint8 payload offset + 12
    pr_res = LITTLE_ENDIAN.uint8 payload offset + 14
    flags = LITTLE_ENDIAN.uint32 payload offset + 16

    quality = flags & 0x003
    orbit_source = (flags & 0x700)  >> 8
    alm_avail = (flags & 0x800)  >> 11
    ano_avail = (flags & 0x1000) >> 12

  /** See $super. */
  stringify -> string:
    codes := ""
    if alm_avail == 1: codes += "A"
    if ano_avail == 1: codes += "N"
    // TODO(kasper): Make this output a whole lot prettier and easier to parse.
    return "$index|$gnss_id|$sv_id|$cno|$quality|$orbit_source|$codes"
