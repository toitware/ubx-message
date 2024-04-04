// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import ubx.ubx_message as ubx
import expect show *

main:
  test_message
  test_nav_pvt
  test_nav_status

test_message:
  message := ubx.AckAck.private_ 0x06 0x13

  expect_equals 0x05 message.cls
  expect_equals 0x01 message.id

  bytes := message.to_byte_array
  expect_bytes_equal #[0xb5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x13, 0x21, 0x4A] bytes

  expect_no_throw:
    ubx.Message.from_bytes bytes

  bytes[2] = 0
  expect_throw "INVALID UBX MESSAGE":
    ubx.Message.from_bytes bytes

  expect_equals "UBX-ACK-ACK" message.stringify

test_nav_pvt:
  pvt_message := ubx.Message ubx.Message.NAV ubx.NavPvt.ID #[]
  expect pvt_message is ubx.NavPvt

test_nav_status:
  status_message := ubx.Message ubx.Message.NAV ubx.NavStatus.ID #[]
  expect status_message is ubx.NavStatus
