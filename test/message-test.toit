// Copyright (C) 2025 Toit contributors.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import ubx.ubx-message as ubx
import expect show *

main:
  test-byte-array-conversion
  test-message
  test-nav-pvt
  test-nav-status

test-byte-array-conversion:
  // Create any example message, and convert to byte array.
  message-ba-before := (ubx.NavStatus.poll).to-byte-array

  // Use the package to parse the byte array message version back to a message
  // again (using the driver code).
  ubx-message := ubx.Message.from-bytes message-ba-before
  message-ba-after := ubx-message.to-byte-array

  // See that the message came back as expected.
  expect-equals message-ba-before message-ba-after

test-message:
  message := ubx.AckAck.private_ 0x06 0x13

  expect-equals 0x05 message.cls
  expect-equals 0x01 message.id

  bytes := message.to-byte-array
  expect-bytes-equal #[0xb5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x13, 0x21, 0x4A] bytes

  expect-no-throw:
    ubx.Message.from-bytes bytes

  bytes[2] = 0
  expect-throw "INVALID UBX MESSAGE":
    ubx.Message.from-bytes bytes

  // The ACK/NAK stringify was adjusted (in this version of ubx-message package)
  // to include more information.  Therefore, this test adjusted to reflect the
  // the current stringify output.
  expect-equals "UBX-ACK-ACK: [6:CFG,19:ANT]" message.stringify

  // Message is still the expected message type.
  expect message is ubx.AckAck

test-nav-pvt:
  pvt-message := ubx.Message ubx.Message.NAV ubx.NavPvt.ID #[]
  expect pvt-message is ubx.NavPvt

test-nav-status:
  // Get values out of class statics.
  msg-nav-type-id/int := ubx.Message.NAV
  msg-nav-status-id/int := ubx.NavStatus.ID
  msg-payload/ByteArray := #[]

  // Construct using generic constructor, using specified types, with empty payload
  status-message := (ubx.Message msg-nav-type-id msg-nav-status-id msg-payload) as ubx.NavStatus
