// Copyright (C) 2019 Toitware ApS. All rights reserved.

import ..src.ubx_message as ubx
import expect show *

main:
  message := ubx.AckAck 0x06 0x013

  bytes := message.to_byte_array

  expect_bytes_equal #[0xb5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x13, 0x21, 0x4A] bytes

  expect_no_throw:
    ubx.Message.from_bytes bytes

  bytes[2] = 0
  expect_throw "INVALID UBX MESSAGE":
    ubx.Message.from_bytes bytes
