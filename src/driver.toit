// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import gpio
import binary
import monitor
import serial.protocols.spi as spi

import .message
import .channel

// CLKODIV.
CLOCK_DIVISOR_1  ::= 0b00
CLOCK_DIVISOR_2  ::= 0b01
CLOCK_DIVISOR_4  ::= 0b10
CLOCK_DIVISOR_10 ::= 0b11

// SCLKDIV.
S_CLOCK_DIVISOR_1 := 0b0
S_CLOCK_DIVISOR_2 := 0b1

// PLLEN.
PLL_DISABLE ::= 0b0
PLL_ENABLE_10X ::= 0b1

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

  static MODE_NORMAL_               ::= 0b000  // Mix CAN FD and Classic CAN 2.0 frames.
  static MODE_SLEEP_                ::= 0b001
  static MODE_INTERNAL_LOOPBACK_    ::= 0b010
  static MODE_LISTEN_ONLY_          ::= 0b011
  static MODE_CONFIGURATION_        ::= 0b100
  static MODE_EXTERNAL_LOOPBACK_    ::= 0b101
  static MODE_NORMAL_CAN_           ::= 0b110  // Normal CAN 2.0 mode; error on CAN FD frames.
  static MODE_RESTRICTED_OPERATION_ ::= 0b111

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

  static KNOWN_FLAGS_ ::= 0b1011_1100_0000_1011

  device_/spi.Device
  device_mutex_/monitor.Mutex ::= monitor.Mutex

  /// Accumulated number of messages that were dropped, due to queue overflow.
  num_dropped_messages/int := 0
  /// Accumulated number of invalid messages received.
  num_invalid_messages/int := 0
  /// Accumulated number of CAN bus errors.
  num_can_bus_errors/int := 0
  /// Accumulated number of system errors of the CAN controller.
  num_system_errors/int := 0

  buffer_/ByteArray ::= ByteArray 74

  send_queue_/Channel_
  receive_queue_/Channel_

  transmit_fifo_full_ := false
  bit_rate_/int := 0
  oscillator_/int := 0
  max_payload_/int := PAYLOAD_SIZE_8
  clko_/int? := 0

  /**
  Initializes a Driver and sets up internal data structures.

  Call $configure to fully activate the CAN bus.
  */
  constructor .device_ --send_queue_size=8 --receive_queue_size=64:
    send_queue_ = Channel_ send_queue_size
    receive_queue_ = Channel_ receive_queue_size

  /**
  Configures the driver for the CAN bus.

  $bit_rate can be "round numbers" that divide neatly into a 20MHz clock,
    like 200kHz, 250kHz, 500kHz.
  $oscillator specifies the speed of the crystal feeding into the chip, usually
    4, 20, or 40MHz.
  $clko specifies the frequency on the CLKO output.  We support 10 and 20MHz.
    For a 4 or 40MHz crystal we also support 4 or 40MHz clko.
    For a 2 or 20MHz crystal we also support 2 or 5MHz clko.
    Default is an unspecified CLKO speed that depends on the oscillator input.
  */
  configure
      --bit_rate=500_000
      --oscillator=40_000_000
      --max_payload=PAYLOAD_SIZE_8
      --clko=null:
    device_mutex_.do:
      configure_ device_
        --bit_rate=bit_rate
        --oscillator=oscillator
        --max_payload=max_payload
        --clko=clko

  recover_ device/spi.Device:
    num_system_errors++
    configure_ device
      --bit_rate=bit_rate_
      --oscillator=oscillator_
      --max_payload=max_payload_
      --clko=clko_

  configure_
      device/spi.Device
      --bit_rate/int
      --oscillator/int
      --max_payload/int
      --clko/int?:
    reset_ device

    bit_rate_ = bit_rate
    oscillator_ = oscillator
    max_payload_ = max_payload
    clko_ = clko

    // Test access to RAM.
    test_memory_ device

    // TODO(anders): Reconfigure SPI frequency when possible.

    clear_memory_ device

    // Configure CLK0 PIN.
    iocon3 := 0x03 // Respect PM1-PM0 default values
    write_u8_ device IOCON_REG_ 3 iocon3

    // Configure ISO CRC Enable bit. Enable PXEDIS and ISO CRC.
    con0 := 0b0110_0000
    write_u8_ device CON_REG_ 0 con0

    // Configure TDC.
    tdc := 1 << 25 // Enable Edge Filtering during Bus Integration state bit.
    tdc |= 1 << 17 // Auto TDC.
    TCDO ::= 0 & 0x7F
    tdc |= TCDO << 8
    write_u32_ device TDC_REG_ tdc

    // Enable RTXAT to limit retransmissions, using TXAT.
    con2 := 0x01
    write_u8_ device CON_REG_ 2 con2

    // Configure RX FIFO.
    configure_fifo_ device RECEIVE_FIFO_INDEX_ --size=27

    // Configure TX FIFO.
    configure_fifo_ device TRANSMIT_FIFO_INDEX_ --tx

    // Add accept-all filter.
    add_filter_ device 0 0 0

    // Enable RX and TX interrupts.
    int2 := 0b11
    write_u8_ device INT_REG_ 2 int2
    // Enable TXATIE and SERRIE.
    int3 := 0b10100
    write_u8_ device INT_REG_ 3 int3
    
    // Signal propagation from left to right:
    //                                         /--Optional divide by 2----SYSCLK----divide-by-BRP----time quantum
    //   Oscillator input----Optional PLL 10x--
    //                                         \--Divide by 1,2,4, or 10----CLKO

    // This SYS_CLOCK speed is possible from all popular oscillator inputs, and
    // is high enough to give us some good resolution when determining timing.
    SYS_CLOCK := 20_000_000

    // Find register values to get the SYS_CLOCK we want from the oscillator
    // input.
    s_clock_divide/int := ?  // SCLKDIV register.
    pll_control/int := ?     // PLLEN register.
    if oscillator == SYS_CLOCK * 2:
      pll_control = PLL_DISABLE
      s_clock_divide = S_CLOCK_DIVISOR_2
    else if oscillator == SYS_CLOCK:
      pll_control = PLL_DISABLE
      s_clock_divide = S_CLOCK_DIVISOR_1
    else if oscillator * 20 == SYS_CLOCK:
      pll_control = PLL_ENABLE_10X
      s_clock_divide = S_CLOCK_DIVISOR_2
    else if oscillator * 10 == SYS_CLOCK:
      pll_control = PLL_ENABLE_10X
      s_clock_divide = S_CLOCK_DIVISOR_1
    else:
      throw "Unsupported oscillator input"

    // Now that we know how we got our SYS_CLOCK we also have the input to the
    // CLKO divider.
    clko_input/int := pll_control == PLL_ENABLE_10X
      ? oscillator * 10
      : oscillator

    // Set the CLKO divider to give the requested CLKO output.
    clock_divide/int := ?  // CLKODIV register.
    if clko == null:
      clock_divide = CLOCK_DIVISOR_10
    else if clko == clko_input:
      clock_divide = CLOCK_DIVISOR_1
    else if clko * 2 == clko_input:
      clock_divide = CLOCK_DIVISOR_2
    else if clko * 4 == clko_input:
      clock_divide = CLOCK_DIVISOR_4
    else if clko * 10 == clko_input:
      clock_divide = CLOCK_DIVISOR_10
    else:
      // In a future driver improvement we might be able to pick a different
      // SYS_CLOCK speed to unlock different CLKO speeds if necessary.
      throw "Unsupported clko clko_input=$clko_input, clko=$clko"

    // Choose as low as possible BRP divider to give us maximum resolution of
    // the time quantum that we use to specify timings.

    // Could perhaps go to 320 with 80% sample point - but can the hardware
    // then still auto-adjust to cover over small clock differences?
    MAX_TQ_PER_CYCLE := 256

    brp/int := 1
    // Normally does zero iterations of this loop.
    while SYS_CLOCK / (brp * bit_rate_) > MAX_TQ_PER_CYCLE: brp *= 2
    SCALED_CLOCK := SYS_CLOCK / brp

    // SYNC takes 1 tick of the clock (time quantum, TQ).
    // TSEG1 ticks then pass before the value is sampled from the data lines.
    // TSEG2 ticks then pass before the next SYNC.
    // The clock period is thus (1 + TSEG1 + TSEG2)
    // Data rate is SYSCLK / (1 + TSEG1 + TSEG2).
    // See "MCP25xxFD Family Reference Manual" page 16.

    // For example for a SYSCLK of 20MHz, select TSEG1=31, TSEG2=8,
    // Data rate is 20M / (1 + 31 + 8) = 500kHz

    // TSEG1 and TSEG2 should be picked so the sample point is at 80% of the
    // clock period, or measured from the live bus.  For now we go with 80%.

    // The following algorithm should support all popular bit rates, including
    // 62.5kHz, 125kHz, 250kHz, 500kHz, and 1MHz.

    // Popular bit rates and crystals will result in a precise number of time
    // quanta per cycle.
    if SCALED_CLOCK % bit_rate_ != 0:
      // If the division is not accurate we cannot exactly hit that
      // bit rate.  If someone wants an inexact bit rate like 333kHz
      // we could soften this requirement.  We may also be able to
      // do this with non-power-of-two BRP values.
      throw "Unsupported data rate: $bit_rate ($SCALED_CLOCK)"
    tq_per_cycle := SCALED_CLOCK / bit_rate_
    tseg1_plus_tseg2 := tq_per_cycle - 1  // SYNC always takes one time quantum.
    tseg1/int := (0.8 * tseg1_plus_tseg2).to_int  // Sample point 80% through the clock cycle.
    tseg2/int := tseg1_plus_tseg2 - tseg1

    if (not 1 <= brp <= 256) or (not 1 <= tseg1 <= 256) or (not 1 <= tseg2 <= 128):
      throw "Unsupported data rate: $bit_rate"

    // SJW is the jump distance - how fast it maximally adjusts to
    // automatically track the actual speed on the wire.  The manual recommends
    // setting this as high as possible, to quickly resynchronize, and examples
    // tend to set it equal to tseg2.
    sjw/int := tseg2

    // Configure nominal bit rate (the bit rate used for the arbitration phase.
    // Currently we don't support switching to a higher bit rate for the data
    // phase - this is a CAN FD features.
    // The checks above should ensure that the following values are in the 8-bit
    // range or 7-bit range as required.
    nbtcfg := brp - 1
    nbtcfg <<= 8
    nbtcfg |= tseg1 - 1
    nbtcfg <<= 8
    nbtcfg |= tseg2 - 1
    nbtcfg <<= 8
    nbtcfg |= sjw - 1
    write_u32_ device NBTCFG_REG_ nbtcfg

    osc0 := (clock_divide << 5)
          + (s_clock_divide << 4)
          + (pll_control)
    write_u8_ device OSC_REG_ 0 osc0

    enter_mode_ device MODE_NORMAL_CAN_

  /**
  Configures and enables a filter.

  Index is 0-based.
  */
  add_filter index/int mask/int obj/int:
    device_mutex_.do:
      add_filter_ device_ index mask obj

  add_filter_ device/spi.Device index/int mask/int obj/int:
    write_u32_ device (filter_mask_reg_ index) mask
    write_u32_ device (filter_obj_reg_ index) obj
    flt_con0 := 1 << 7 // Filter is enabled
    // Add messages to receive FIFO.
    flt_con0 |= RECEIVE_FIFO_INDEX_
    write_u8_ device (filter_con_reg_ index) 0 flt_con0

  /**
  Run the interrupt routine. Without this, the driver will not be able to
    send or receive messages.

  This method blocks for the lifecycle of the driver, and should be called
    from a dedicated task.

  If the interrupt pin is triggered repeatedly without any events from the
    controller, $run will throw. This probably indicates that the interrupt pin
    is wrongly configured.
  */
  run interrupt/gpio.Pin:
    interrupt.config --input

    no_events_count := 0

    while true:
      if process_interrupts_:
        no_events_count = 0
      else:
        no_events_count++
        if no_events_count > 5:
          throw "possibly wrong interrupt pin number"

      while not transmit_fifo_full_:
        msg := send_queue_.try_receive
        if not msg: break

      while true:
        e := catch --unwind=(: it != DEADLINE_EXCEEDED_ERROR):
          with_timeout --ms=250:
            interrupt.wait_for 0
            break
        if e:
          device_mutex_.do:
            // Check if the device was rebooted by reading the mode.
            // It powers up in configuration mode.
            if (get_mode_ device_) == MODE_CONFIGURATION_:
              recover_ device_



  /**
  Send the message on the CAN bus.

  The call returns as soon as the message has been enqueued, not when the
  message is flushed to the CAN bus..
  */
  send msg/Message:
    if transmit_fifo_full_:
      send_queue_.send msg
    else:
      device_mutex_.do:
        send_ device_ msg

  /**
  Receives the next messages on the CAN bus.

  Blocks until a message is available.
  */
  receive -> Message:
    return receive_queue_.receive

  send_ device/spi.Device msg/Message:
    ram_offset := read_u32_ device (fifoua_reg_ TRANSMIT_FIFO_INDEX_)
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
    write_u8_ device (fifocon_reg_ TRANSMIT_FIFO_INDEX_) 1 tra_con1

    // Check if full.
    status := read_u8_ device (fifosta_reg_ TRANSMIT_FIFO_INDEX_) 0
    if status & 1 == 0:
      transmit_fifo_full_ = true
      // Add not-full interrupt.
      tra_con0 := FIFO_TX_CON0_ | FIFO_NOT_FULL_
      write_u8_ device (fifocon_reg_ TRANSMIT_FIFO_INDEX_) 0 tra_con0

  receive_ device/spi.Device:
    ram_offset := read_u32_ device (fifoua_reg_ RECEIVE_FIFO_INDEX_)
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
    write_u8_ device_ (fifocon_reg_ RECEIVE_FIFO_INDEX_) 1 rec_con1

    msg := Message id data
    if not receive_queue_.try_send msg:
      num_dropped_messages++

  payload_size_length_ payload_size/int:
    if payload_size == PAYLOAD_SIZE_8: return 8
    if payload_size == PAYLOAD_SIZE_64: return 64
    throw "UNKNOWN PAYLOAD SIZE"

  process_interrupts_ -> bool:
    has_events := false
    while true:
      device_mutex_.do:
        32.repeat:
          flags := read_u16_ device_ INT_REG_ 0
          if flags == 0: return has_events
          has_events = true

          if flags & ~KNOWN_FLAGS_ != 0: throw "UNEXPECTED FLAGS: 0x$(flags.stringify 16)"

          if flags & (1 << 12) != 0:
            write_u8_ device_ INT_REG_ 1 ~(1 << 4)
            recover_ device_
            return true

          // RXIF.
          if flags & (1 << 1) != 0:
            receive_ device_

          // TXATIF.
          if flags & (1 << 10) != 0:
            // Clear Pending Transmit Attempt interrupt bit.
            write_u8_ device_ (fifosta_reg_ TRANSMIT_FIFO_INDEX_) 0 ~(1 << 4)

          // TXIF.
          if flags & (1 << 0) != 0:
            transmit_fifo_full_ = false
            // Transmit complete, disable not-full interrupt.
            tra_con0 := FIFO_TX_CON0_
            write_u8_ device_ (fifocon_reg_ TRANSMIT_FIFO_INDEX_) 0 tra_con0

          // MODIF
          if flags & (1 << 3) != 0:
            // Clear the MODIF bit.
            write_u8_ device_ INT_REG_ 0 ~(1 << 3)

          // RXOVIF
          if flags & (1 << 11) != 0:
            // Clear RXOVIF flag on receive buffer.
            write_u8_ device_ (fifosta_reg_ RECEIVE_FIFO_INDEX_) 0 ~(1 << 3)
            num_dropped_messages++

          // IVMIF Invalid message occurred.
          if flags & (1 << 15) != 0:
            num_invalid_messages++
            // Clear the IVMIF bit.
            write_u8_ device_ INT_REG_ 1 ~(1 << 7)

          // CERRIF CAN Bus error.
          if flags & (1 << 13) != 0:
            num_can_bus_errors++
            // Clear the IVMIF bit.
            write_u8_ device_ INT_REG_ 1 ~(1 << 5)


      // Sleep for a short while to not starve.
      sleep --ms=2

  // Configures a FIFO buffer..
  // Index is 1-based.
  configure_fifo_ device/spi.Device index/int --size/int=1 --tx=false --priority/int=0:
    if tx:
      // Set priority and retry policy.
      con2 := (RETRY_DISABLED_ << 5) | priority
      write_u8_ device (fifocon_reg_ index) 2 con2

    // Set payload and size.
    con3 := (max_payload_ << 5) | (size - 1)
    write_u8_ device (fifocon_reg_ index) 3 con3

    con0 := tx ? FIFO_TX_CON0_ : FIFO_RX_CON0_
    write_u8_ device (fifocon_reg_ index) 0 con0

  enter_mode_ device/spi.Device mode/int:
    con3 := mode
    with_timeout --ms=50:
      while true:
        write_u8_ device CON_REG_ 3 con3
        if (get_mode_ device) == mode: break
        sleep --ms=1

  get_mode_ device/spi.Device -> int:
    val := read_u8_ device CON_REG_ 2
    return val >> 5

  test_memory_ device/spi.Device:
    for i := 1; i <= binary.UINT32_MAX; i <<= 1:
      write_u32_ device RAM_START_REG_ i
      if i != (read_u32_ device RAM_START_REG_): throw "SPI ERROR"

  // Writes 0 into all ram words.
  clear_memory_ device/spi.Device:
    for reg := RAM_START_REG_; reg < RAM_END_REG_; reg += 4:
      write_u32_ device reg 0

  read_u8_ device/spi.Device reg/int byte_offset/int -> int:
    reg += byte_offset
    binary.BIG_ENDIAN.put_uint16 buffer_ 0 reg | SPI_READ_COMMAND_
    device.transfer buffer_ --read --to=3
    return buffer_[2]

  read_u16_ device/spi.Device reg/int byte_offset/int -> int:
    binary.BIG_ENDIAN.put_uint16 buffer_ 0 reg | SPI_READ_COMMAND_
    device.transfer buffer_ --read --to=4
    return binary.LITTLE_ENDIAN.uint16 buffer_ 2

  read_u32_ device/spi.Device reg/int -> int:
    binary.BIG_ENDIAN.put_uint16 buffer_ 0 reg | SPI_READ_COMMAND_
    device.transfer buffer_ --read --to=6
    return binary.LITTLE_ENDIAN.uint32 buffer_ 2

  write_u8_ device/spi.Device reg/int byte_offset/int value/int:
    reg += byte_offset
    binary.BIG_ENDIAN.put_uint16 buffer_ 0 reg | SPI_WRITE_COMMAND_
    buffer_[2] = value
    device.transfer buffer_ --to=3

  write_u32_ device/spi.Device reg/int value/int:
    binary.BIG_ENDIAN.put_uint16 buffer_ 0 reg | SPI_WRITE_COMMAND_
    binary.LITTLE_ENDIAN.put_uint32 buffer_ 2 value
    device.transfer buffer_ --to=6

  reset_ device/spi.Device:
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
