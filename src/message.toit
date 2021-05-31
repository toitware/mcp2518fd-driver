// Copyright (C) 2021 Toitware ApS. All rights reserved.

/**
CAN-bus message.
*/
class Message:
  id/int
  data/ByteArray

  constructor .id .data:
