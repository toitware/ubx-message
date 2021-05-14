// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import ubx.ubx_message
import expect show *

main:
  test_message

test_message:
  message := ubx.AckAck.private_ 0x06 0x013

  bytes := message.to_byte_array

  expect_bytes_equal #[0xb5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x13, 0x21, 0x4A] bytes

  expect_no_throw:
    ubx.Message.from_bytes bytes

  bytes[2] = 0
  expect_throw "INVALID UBX MESSAGE":
    ubx.Message.from_bytes bytes

  expect_equals "UBX-ACK-ACK" message.stringify
