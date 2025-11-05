// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import ubx.ubx-message as ubx
import expect show *

main:
  test-message
  test-nav-pvt
  test-nav-status

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

  expect-equals "UBX-ACK-ACK" message.stringify

test-nav-pvt:
  pvt-message := ubx.Message ubx.Message.NAV ubx.NavPvt.ID #[]
  expect pvt-message is ubx.NavPvt

test-nav-status:
  status-message := ubx.Message ubx.Message.NAV ubx.NavStatus.ID #[]
  expect status-message is ubx.NavStatus
