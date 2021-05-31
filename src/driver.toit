// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import gpio
import binary
import monitor
import serial.protocols.spi as spi

import .message
import .channel

CLOCK_DIVISOR_1  ::= 0b00
CLOCK_DIVISOR_2  ::= 0b01
CLOCK_DIVISOR_4  ::= 0b10
CLOCK_DIVISOR_10 ::= 0b11

PAYLOAD_SIZE_8  ::= 0
PAYLOAD_SIZE_64  ::= 7

/**
Driver for MCP2518FD CAN bus controllers, using SPI.
*/
class Driver:
  static CON_REG_       ::= 0x000
  static NBTCFG_REG_    ::= 0x004
  static TDC_REG_       ::= 0x00C
  static INT_REG_       ::= 0x01C
  static RAM_START_REG_ ::= 0x400  // Inclusive.
  static RAM_END_REG_   ::= 0xC00  // Exclusive.
  static OSC_REG_       ::= 0xE00
  static IOCON_REG_     ::= 0xE04
  static DEVID_REG_     ::= 0xE14

  static MODE_CONFIGURATION_  ::= 0b100
  static MODE_LISTEN_ONLY_    ::= 0b011
  static MODE_NORMAL_         ::= 0b000
  static MODE_NORMAL_CAN_     ::= 0b110

  static FIFO_RX_CON0_  ::= 0b0000_1001  // Enable TFNRFNIE and RXOVIE.
  static FIFO_TX_CON0_  ::= 0b1001_0000  // Set TX marker and enable TXATIE.
  static FIFO_NOT_FULL_ ::= 0b1

  static RECEIVE_FIFO_INDEX_ ::= 1
  static TRANSMIT_FIFO_INDEX_ ::= 2

  static RETRY_DISABLED_  ::= 0b00
  static RETRY_UNLIMITED_ ::= 0b10

  static FRAME_FLAG_EXT_ ::= 0b1_0000

  static SPI_READ_COMMAND_  ::= 0b0011 << 12
  static SPI_WRITE_COMMAND_ ::= 0b0010 << 12

  device/spi.Device

  buffer_/ByteArray ::= ByteArray 74

  send_queue_/Channel_
  receive_queue_/Channel_

  transmit_fifo_full_ := false
  max_payload_/int := PAYLOAD_SIZE_8

  /**
  Initializes a Driver and sets up internal datastructures.

  Call $configure to fully activate the CAN bus.
  */
  constructor .device --send_queue_size=8 --receive_queue_size=64:
    send_queue_ = Channel_ send_queue_size
    receive_queue_ = Channel_ receive_queue_size

  /**
  Configures the driver for the CAN bus.

  It's currently hardcoded to 500kHz.
  */
  configure
      --clock_divide=CLOCK_DIVISOR_10
      --max_payload=PAYLOAD_SIZE_8:
    reset_

    max_payload_ = max_payload

    // Test access to RAM.
    test_memory_

    osc0 := clock_divide << 5
    write_u8_ OSC_REG_ 0 osc0

    // TODO(anders): Reconfigure SPI frequency when possible.

    clear_memory_

    // Configure CLK0 PIN.
    iocon3 := 0x03 // Respect PM1-PM0 default values
    write_u8_ IOCON_REG_ 3 iocon3

    // Configure ISO CRC Enable bit. Enable PXEDIS and ISO CRC.
    con0 := 0b0110_0000
    write_u8_ CON_REG_ 0 con0

    // Configure TDC.
    tdc := 1 << 25 // Enable Edge Filtering during Bus Integration state bit.
    tdc |= 1 << 17 // Auto TDC.
    TCDO ::= 0 & 0x7F
    tdc |= TCDO << 8
    write_u32_ TDC_REG_ tdc

    // Enable RTXAT to limit retransmissions, using TXAT.
    con2 := 0x01
    write_u8_ CON_REG_ 2 con2

    // Configure RX FIFO.
    configure_fifo_ RECEIVE_FIFO_INDEX_ --size=27

    // Configure TX FIFO.
    configure_fifo_ TRANSMIT_FIFO_INDEX_ --tx

    // Add accept-all filter.
    add_filter 0 0 0

    // Enable RX and TX interrupts.
    int2 := 0b11
    write_u8_ INT_REG_ 2 int2
    // Enable TXATIE.
    int3 := 0b100
    write_u8_ INT_REG_ 3 int3

    // Configure data rate (should be 500k).
    // TQCount := 4 (PS1=2 + PS2=1 + 1)
    // 10.4kHz: 192 == PS2=38 + PS1=153 + 1
    BRP := 20
    PS1 := 2
    PS2 := 1
    nbtcfg := BRP - 1
    nbtcfg <<= 8
    nbtcfg |= PS1 - 1
    nbtcfg <<= 8
    nbtcfg |= PS2 - 1
    nbtcfg <<= 8
    nbtcfg |= PS2 - 1
    write_u32_ NBTCFG_REG_ nbtcfg

    enter_mode_ MODE_NORMAL_CAN_

  /**
  Configures and enables a filter.

  Index is 0-based.
  */
  add_filter index/int mask/int obj/int:
    write_u32_ (filter_mask_reg_ index) mask
    write_u32_ (filter_obj_reg_ index) obj
    flt_con0 := 1 << 7 // Filter is enabled
    // Add messages to receive FIFO.
    flt_con0 |= RECEIVE_FIFO_INDEX_
    write_u8_ (filter_con_reg_ index) 0 flt_con0

  /**
  Run the interrupt rutine. Without this, the driver will not be able to
    send or receive messages.

  This methods blocks for the lifecycle of the driver.
  */
  run interrupt/gpio.Pin:
    while true:
      process_interrupts_
      while not transmit_fifo_full_:
        msg := send_queue_.try_receive
        if not msg: break

      interrupt.wait_for 0

  /**
  Send the message on the CAN bus.

  The call returns as soon as the message has been enqueued, not when the
  message is flushed to the CAN bus..
  */
  send msg/Message:
    if transmit_fifo_full_:
      send_queue_.send msg
    else:
      send_ msg

  /**
  Receives the next messages on the CAN bus.

  Blocks until a message is available.
  */
  receive -> Message:
    return receive_queue_.receive

  send_ msg/Message:
    ram_offset := read_u32_ (fifoua_reg_ TRANSMIT_FIFO_INDEX_)
    reg := RAM_START_REG_ + ram_offset

    id := msg.id
    extended_id := id >= 2048

    if extended_id: id = (id >> 18 & 0x7FF) | ((id & 0x3FFFF) << 11)

    data := msg.data
    //--- Write DLC field, FDF, BRS, RTR, IDE bits
    flags := data.size
    if extended_id: flags |= FRAME_FLAG_EXT_
    //flags |= 1 << 5

    // Write word-aligned.
    words ::= (data.size + 3) / 4

    binary.BIG_ENDIAN.put_uint16 buffer_ 0 reg | SPI_WRITE_COMMAND_
    binary.LITTLE_ENDIAN.put_uint32 buffer_ 2 id
    binary.LITTLE_ENDIAN.put_uint32 buffer_ 6 flags
    buffer_.replace 10 data

    // 0-pad to aligned end.
    aligned_end := 10 + 4 * words
    buffer_.fill --from=10 + data.size --to=aligned_end 0

    device.transfer buffer_ --to=aligned_end

    // Set UINC and TXREQ.
    tra_con1 := 0b11
    write_u8_ (fifocon_reg_ TRANSMIT_FIFO_INDEX_) 1 tra_con1

    // Check if full.
    status := read_u8_ (fifosta_reg_ TRANSMIT_FIFO_INDEX_) 0
    if status & 1 == 0:
      transmit_fifo_full_ = true
      // Add not-full interrupt.
      tra_con0 := FIFO_TX_CON0_ | FIFO_NOT_FULL_
      write_u8_ (fifocon_reg_ TRANSMIT_FIFO_INDEX_) 0 tra_con0

  receive_:
    ram_offset := read_u32_ (fifoua_reg_ RECEIVE_FIFO_INDEX_)
    reg := RAM_START_REG_ + ram_offset

    binary.BIG_ENDIAN.put_uint16 buffer_ 0 reg | SPI_READ_COMMAND_

    max_size := payload_size_length_ max_payload_
    device.transfer buffer_ --read --to=10 + max_size

    // Decode data in buffer.
    id := binary.LITTLE_ENDIAN.uint32 buffer_ 2
    flags := binary.LITTLE_ENDIAN.uint32 buffer_ 6
    if flags & FRAME_FLAG_EXT_ != 0:
      id = (id >> 11) | ((id & 0x7FF) << 18)

    length := flags & 0xF
    data := buffer_.copy 10 10 + length

    // Buffer is free, increment FIFO.
    rec_con1 := 0b1
    write_u8_ (fifocon_reg_ RECEIVE_FIFO_INDEX_) 1 rec_con1

    // TODO(anders): Drop messages if full?
    receive_queue_.send
      Message id data

  payload_size_length_ payload_size/int:
    if payload_size == PAYLOAD_SIZE_8: return 8
    if payload_size == PAYLOAD_SIZE_64: return 64
    throw "UNKNOWN PAYLOAD SIZE"

  process_interrupts_:
    flags := 0
    3.repeat:
      flags = read_u16_ INT_REG_ 0
      if flags == 0: return

      // RXIF.
      if flags & (1 << 1) != 0:
        receive_

      // TXATIF.
      if flags & (1 << 10) != 0:
        // Clear Pending Transmit Attempt interrupt bit.
        write_u8_ (fifosta_reg_ TRANSMIT_FIFO_INDEX_) 0 ~(1 << 4)

      // TXIF.
      if flags & (1 << 0) != 0:
        transmit_fifo_full_ = false
        // Transmit complete, disable not-full interrupt.
        tra_con0 := FIFO_TX_CON0_
        write_u8_ (fifocon_reg_ TRANSMIT_FIFO_INDEX_) 0 tra_con0

      // MODIF
      if flags & (1 << 3) != 0:
        // Clear the MODIF bit.
        write_u8_ INT_REG_ 0 ~(1 << 3)

      // RXOVIF
      if flags & (1 << 11) != 0:

    print "UNHANDLED: $flags"

  // Configures a FIFO buffer..
  // Index is 1-based.
  configure_fifo_ index/int --size/int=1 --tx=false --priority/int=0:
    if tx:
      // Set priority and retry policy.
      con2 := (RETRY_DISABLED_ << 5) | priority
      write_u8_ (fifocon_reg_ index) 2 con2

    // Set payload and size.
    con3 := (max_payload_ << 5) | (size - 1)
    write_u8_ (fifocon_reg_ index) 3 con3

    con0 := tx ? FIFO_TX_CON0_ : FIFO_RX_CON0_
    write_u8_ (fifocon_reg_ index) 0 con0

  enter_mode_ mode/int:
    con3 := mode
    with_timeout --ms=50:
      while true:
        write_u8_ CON_REG_ 3 con3
        if get_mode_ == mode: break
        sleep --ms=1

  get_mode_ -> int:
    val := read_u8_ CON_REG_ 2
    return val >> 5

  test_memory_:
    for i := 1; i <= binary.UINT32_MAX; i <<= 1:
      write_u32_ RAM_START_REG_ i
      if i != (read_u32_ RAM_START_REG_): throw "SPI ERROR"

  // Writes 0 into all ram words.
  clear_memory_:
    for reg := RAM_START_REG_; reg < RAM_END_REG_; reg += 4:
      write_u32_ reg 0

  read_u8_ reg/int byte_offset/int -> int:
    reg += byte_offset
    binary.BIG_ENDIAN.put_uint16 buffer_ 0 reg | SPI_READ_COMMAND_
    device.transfer buffer_ --read --to=3
    return buffer_[2]

  read_u16_ reg/int byte_offset/int -> int:
    binary.BIG_ENDIAN.put_uint16 buffer_ 0 reg | SPI_READ_COMMAND_
    device.transfer buffer_ --read --to=4
    return binary.LITTLE_ENDIAN.uint16 buffer_ 2

  read_u32_ reg/int -> int:
    binary.BIG_ENDIAN.put_uint16 buffer_ 0 reg | SPI_READ_COMMAND_
    device.transfer buffer_ --read --to=6
    return binary.LITTLE_ENDIAN.uint32 buffer_ 2

  write_u8_ reg/int byte_offset/int value/int:
    reg += byte_offset
    binary.BIG_ENDIAN.put_uint16 buffer_ 0 reg | SPI_WRITE_COMMAND_
    buffer_[2] = value
    device.transfer buffer_ --to=3

  write_u32_ reg/int value/int:
    binary.BIG_ENDIAN.put_uint16 buffer_ 0 reg | SPI_WRITE_COMMAND_
    binary.LITTLE_ENDIAN.put_uint32 buffer_ 2 value
    device.transfer buffer_ --to=6

  reset_:
    zeros := ByteArray 2
    device.transfer zeros

  static fifocon_reg_ index:
    return 0x05C + 12 * (index - 1)

  static fifosta_reg_ index:
    return 0x060 + 12 * (index - 1)

  static fifoua_reg_ index:
    return 0x064 + 12 * (index - 1)

  static filter_con_reg_ index:
    return 0x1D0 + 8 * index

  static filter_obj_reg_ index:
    return 0x1F0 + 8 * index

  static filter_mask_reg_ index:
    return 0x1F4 + 8 * index
