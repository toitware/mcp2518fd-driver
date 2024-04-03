// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import gpio
import io
import monitor
import spi

import .message
import .channel

// CLKODIV.
CLOCK-DIVISOR-1  ::= 0b00
CLOCK-DIVISOR-2  ::= 0b01
CLOCK-DIVISOR-4  ::= 0b10
CLOCK-DIVISOR-10 ::= 0b11

// SCLKDIV.
S-CLOCK-DIVISOR-1 := 0b0
S-CLOCK-DIVISOR-2 := 0b1

// PLLEN.
PLL-DISABLE ::= 0b0
PLL-ENABLE-10X ::= 0b1

PAYLOAD-SIZE-8  ::= 0
PAYLOAD-SIZE-64  ::= 7

/**
Driver for MCP2518FD CAN bus controllers, using SPI.
*/
class Driver:
  static CON-REG_       ::= 0x000
  static NBTCFG-REG_    ::= 0x004
  static TDC-REG_       ::= 0x00C
  static INT-REG_       ::= 0x01C
  static RAM-START-REG_ ::= 0x400  // Inclusive.
  static RAM-END-REG_   ::= 0xC00  // Exclusive.
  static OSC-REG_       ::= 0xE00
  static IOCON-REG_     ::= 0xE04
  static DEVID-REG_     ::= 0xE14

  static MODE-NORMAL_               ::= 0b000  // Mix CAN FD and Classic CAN 2.0 frames.
  static MODE-SLEEP_                ::= 0b001
  static MODE-INTERNAL-LOOPBACK_    ::= 0b010
  static MODE-LISTEN-ONLY_          ::= 0b011
  static MODE-CONFIGURATION_        ::= 0b100
  static MODE-EXTERNAL-LOOPBACK_    ::= 0b101
  static MODE-NORMAL-CAN_           ::= 0b110  // Normal CAN 2.0 mode; error on CAN FD frames.
  static MODE-RESTRICTED-OPERATION_ ::= 0b111

  static FIFO-RX-CON0_  ::= 0b0000_1001  // Enable TFNRFNIE and RXOVIE.
  static FIFO-TX-CON0_  ::= 0b1001_0000  // Set TX marker and enable TXATIE.
  static FIFO-NOT-FULL_ ::= 0b1

  static RECEIVE-FIFO-INDEX_ ::= 1
  static TRANSMIT-FIFO-INDEX_ ::= 2

  static RETRY-DISABLED_  ::= 0b00
  static RETRY-UNLIMITED_ ::= 0b10

  static FRAME-FLAG-EXT_ ::= 0b1_0000

  static SPI-READ-COMMAND_  ::= 0b0011 << 12
  static SPI-WRITE-COMMAND_ ::= 0b0010 << 12

  static KNOWN-FLAGS_ ::= 0b1011_1100_0000_1011

  device_/spi.Device
  device-mutex_/monitor.Mutex ::= monitor.Mutex

  /// Accumulated number of messages that were dropped, due to queue overflow.
  num-dropped-messages/int := 0
  /// Accumulated number of invalid messages received.
  num-invalid-messages/int := 0
  /// Accumulated number of CAN bus errors.
  num-can-bus-errors/int := 0
  /// Accumulated number of system errors of the CAN controller.
  num-system-errors/int := 0

  buffer_/ByteArray ::= ByteArray 74

  send-queue_/Channel_
  receive-queue_/Channel_

  transmit-fifo-full_ := false
  bit-rate_/int := 0
  oscillator_/int := 0
  max-payload_/int := PAYLOAD-SIZE-8
  clko_/int? := 0

  /**
  Initializes a Driver and sets up internal data structures.

  Call $configure to fully activate the CAN bus.
  */
  constructor .device_ --send-queue-size=8 --receive-queue-size=64:
    send-queue_ = Channel_ send-queue-size
    receive-queue_ = Channel_ receive-queue-size

  /**
  Configures the driver for the CAN bus.

  $bit-rate can be "round numbers" that divide neatly into a 20MHz clock,
    like 200kHz, 250kHz, 500kHz.
  $oscillator specifies the speed of the crystal feeding into the chip, usually
    4, 20, or 40MHz.
  $clko specifies the frequency on the CLKO output.  We support 10 and 20MHz.
    For a 4 or 40MHz crystal we also support 4 or 40MHz clko.
    For a 2 or 20MHz crystal we also support 2 or 5MHz clko.
    Default is an unspecified CLKO speed that depends on the oscillator input.
  */
  configure
      --bit-rate=500_000
      --oscillator=40_000_000
      --max-payload=PAYLOAD-SIZE-8
      --clko=null:
    device-mutex_.do:
      configure_ device_
        --bit-rate=bit-rate
        --oscillator=oscillator
        --max-payload=max-payload
        --clko=clko

  recover_ device/spi.Device:
    num-system-errors++
    configure_ device
      --bit-rate=bit-rate_
      --oscillator=oscillator_
      --max-payload=max-payload_
      --clko=clko_

  configure_
      device/spi.Device
      --bit-rate/int
      --oscillator/int
      --max-payload/int
      --clko/int?:
    reset_ device

    bit-rate_ = bit-rate
    oscillator_ = oscillator
    max-payload_ = max-payload
    clko_ = clko

    // Test access to RAM.
    test-memory_ device

    // TODO(anders): Reconfigure SPI frequency when possible.

    clear-memory_ device

    // Configure CLK0 PIN.
    iocon3 := 0x03 // Respect PM1-PM0 default values
    write-u8_ device IOCON-REG_ 3 iocon3

    // Configure ISO CRC Enable bit. Enable PXEDIS and ISO CRC.
    con0 := 0b0110_0000
    write-u8_ device CON-REG_ 0 con0

    // Configure TDC.
    tdc := 1 << 25 // Enable Edge Filtering during Bus Integration state bit.
    tdc |= 1 << 17 // Auto TDC.
    TCDO ::= 0 & 0x7F
    tdc |= TCDO << 8
    write-u32_ device TDC-REG_ tdc

    // Enable RTXAT to limit retransmissions, using TXAT.
    con2 := 0x01
    write-u8_ device CON-REG_ 2 con2

    // Configure RX FIFO.
    configure-fifo_ device RECEIVE-FIFO-INDEX_ --size=27

    // Configure TX FIFO.
    configure-fifo_ device TRANSMIT-FIFO-INDEX_ --tx

    // Add accept-all filter.
    add-filter_ device 0 0 0

    // Enable RX and TX interrupts.
    int2 := 0b11
    write-u8_ device INT-REG_ 2 int2
    // Enable TXATIE and SERRIE.
    int3 := 0b10100
    write-u8_ device INT-REG_ 3 int3

    // Signal propagation from left to right:
    //                                         /--Optional divide by 2----SYSCLK----divide-by-BRP----time quantum
    //   Oscillator input----Optional PLL 10x--
    //                                         \--Divide by 1,2,4, or 10----CLKO

    // This SYS_CLOCK speed is possible from all popular oscillator inputs, and
    // is high enough to give us some good resolution when determining timing.
    SYS-CLOCK := 20_000_000

    // Find register values to get the SYS_CLOCK we want from the oscillator
    // input.
    s-clock-divide/int := ?  // SCLKDIV register.
    pll-control/int := ?     // PLLEN register.
    if oscillator == SYS-CLOCK * 2:
      pll-control = PLL-DISABLE
      s-clock-divide = S-CLOCK-DIVISOR-2
    else if oscillator == SYS-CLOCK:
      pll-control = PLL-DISABLE
      s-clock-divide = S-CLOCK-DIVISOR-1
    else if oscillator * 20 == SYS-CLOCK:
      pll-control = PLL-ENABLE-10X
      s-clock-divide = S-CLOCK-DIVISOR-2
    else if oscillator * 10 == SYS-CLOCK:
      pll-control = PLL-ENABLE-10X
      s-clock-divide = S-CLOCK-DIVISOR-1
    else:
      throw "Unsupported oscillator input"

    // Now that we know how we got our SYS_CLOCK we also have the input to the
    // CLKO divider.
    clko-input/int := pll-control == PLL-ENABLE-10X
      ? oscillator * 10
      : oscillator

    // Set the CLKO divider to give the requested CLKO output.
    clock-divide/int := ?  // CLKODIV register.
    if clko == null:
      clock-divide = CLOCK-DIVISOR-10
    else if clko == clko-input:
      clock-divide = CLOCK-DIVISOR-1
    else if clko * 2 == clko-input:
      clock-divide = CLOCK-DIVISOR-2
    else if clko * 4 == clko-input:
      clock-divide = CLOCK-DIVISOR-4
    else if clko * 10 == clko-input:
      clock-divide = CLOCK-DIVISOR-10
    else:
      // In a future driver improvement we might be able to pick a different
      // SYS_CLOCK speed to unlock different CLKO speeds if necessary.
      throw "Unsupported clko clko_input=$clko-input, clko=$clko"

    // Choose as low as possible BRP divider to give us maximum resolution of
    // the time quantum that we use to specify timings.

    // Could perhaps go to 320 with 80% sample point - but can the hardware
    // then still auto-adjust to cover over small clock differences?
    MAX-TQ-PER-CYCLE := 256

    brp/int := 1
    // Normally does zero iterations of this loop.
    while SYS-CLOCK / (brp * bit-rate_) > MAX-TQ-PER-CYCLE: brp *= 2
    SCALED-CLOCK := SYS-CLOCK / brp

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
    if SCALED-CLOCK % bit-rate_ != 0:
      // If the division is not accurate we cannot exactly hit that
      // bit rate.  If someone wants an inexact bit rate like 333kHz
      // we could soften this requirement.  We may also be able to
      // do this with non-power-of-two BRP values.
      throw "Unsupported data rate: $bit-rate ($SCALED-CLOCK)"
    tq-per-cycle := SCALED-CLOCK / bit-rate_
    tseg1-plus-tseg2 := tq-per-cycle - 1  // SYNC always takes one time quantum.
    tseg1/int := (0.8 * tseg1-plus-tseg2).to-int  // Sample point 80% through the clock cycle.
    tseg2/int := tseg1-plus-tseg2 - tseg1

    if (not 1 <= brp <= 256) or (not 1 <= tseg1 <= 256) or (not 1 <= tseg2 <= 128):
      throw "Unsupported data rate: $bit-rate"

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
    write-u32_ device NBTCFG-REG_ nbtcfg

    osc0 := (clock-divide << 5)
          + (s-clock-divide << 4)
          + (pll-control)
    write-u8_ device OSC-REG_ 0 osc0

    enter-mode_ device MODE-NORMAL-CAN_

  /**
  Configures and enables a filter.

  Index is 0-based.
  */
  add-filter index/int mask/int obj/int:
    device-mutex_.do:
      add-filter_ device_ index mask obj

  add-filter_ device/spi.Device index/int mask/int obj/int:
    write-u32_ device (filter-mask-reg_ index) mask
    write-u32_ device (filter-obj-reg_ index) obj
    flt-con0 := 1 << 7 // Filter is enabled
    // Add messages to receive FIFO.
    flt-con0 |= RECEIVE-FIFO-INDEX_
    write-u8_ device (filter-con-reg_ index) 0 flt-con0

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
    interrupt.configure --input

    no-events-count := 0

    while true:
      if process-interrupts_:
        no-events-count = 0
      else:
        no-events-count++
        if no-events-count > 5:
          throw "possibly wrong interrupt pin number"

      while not transmit-fifo-full_:
        msg := send-queue_.try-receive
        if not msg: break

      while true:
        e := catch --unwind=(: it != DEADLINE-EXCEEDED-ERROR):
          with-timeout --ms=250:
            interrupt.wait-for 0
            break
        if e:
          device-mutex_.do:
            // Check if the device was rebooted by reading the mode.
            // It powers up in configuration mode.
            if (get-mode_ device_) == MODE-CONFIGURATION_:
              recover_ device_



  /**
  Send the message on the CAN bus.

  The call returns as soon as the message has been enqueued, not when the
  message is flushed to the CAN bus..
  */
  send msg/Message:
    if transmit-fifo-full_:
      send-queue_.send msg
    else:
      device-mutex_.do:
        send_ device_ msg

  /**
  Receives the next messages on the CAN bus.

  Blocks until a message is available.
  */
  receive -> Message:
    return receive-queue_.receive

  send_ device/spi.Device msg/Message:
    ram-offset := read-u32_ device (fifoua-reg_ TRANSMIT-FIFO-INDEX_)
    reg := RAM-START-REG_ + ram-offset

    id := msg.id
    extended-id := id >= 2048

    if extended-id: id = (id >> 18 & 0x7FF) | ((id & 0x3FFFF) << 11)

    data := msg.data
    //--- Write DLC field, FDF, BRS, RTR, IDE bits
    flags := data.size
    if extended-id: flags |= FRAME-FLAG-EXT_
    //flags |= 1 << 5

    // Write word-aligned.
    words ::= (data.size + 3) / 4

    io.BIG-ENDIAN.put-uint16 buffer_ 0 reg | SPI-WRITE-COMMAND_
    io.LITTLE-ENDIAN.put-uint32 buffer_ 2 id
    io.LITTLE-ENDIAN.put-uint32 buffer_ 6 flags
    buffer_.replace 10 data

    // 0-pad to aligned end.
    aligned-end := 10 + 4 * words
    buffer_.fill --from=10 + data.size --to=aligned-end 0

    device.transfer buffer_ --to=aligned-end

    // Set UINC and TXREQ.
    tra-con1 := 0b11
    write-u8_ device (fifocon-reg_ TRANSMIT-FIFO-INDEX_) 1 tra-con1

    // Check if full.
    status := read-u8_ device (fifosta-reg_ TRANSMIT-FIFO-INDEX_) 0
    if status & 1 == 0:
      transmit-fifo-full_ = true
      // Add not-full interrupt.
      tra-con0 := FIFO-TX-CON0_ | FIFO-NOT-FULL_
      write-u8_ device (fifocon-reg_ TRANSMIT-FIFO-INDEX_) 0 tra-con0

  receive_ device/spi.Device:
    ram-offset := read-u32_ device (fifoua-reg_ RECEIVE-FIFO-INDEX_)
    reg := RAM-START-REG_ + ram-offset

    io.BIG-ENDIAN.put-uint16 buffer_ 0 reg | SPI-READ-COMMAND_

    max-size := payload-size-length_ max-payload_
    device.transfer buffer_ --read --to=10 + max-size

    // Decode data in buffer.
    id := io.LITTLE-ENDIAN.uint32 buffer_ 2
    flags := io.LITTLE-ENDIAN.uint32 buffer_ 6
    if flags & FRAME-FLAG-EXT_ != 0:
      id = (id >> 11) | ((id & 0x7FF) << 18)

    length := flags & 0xF
    data := buffer_.copy 10 10 + length

    // Buffer is free, increment FIFO.
    rec-con1 := 0b1
    write-u8_ device_ (fifocon-reg_ RECEIVE-FIFO-INDEX_) 1 rec-con1

    msg := Message id data
    if not receive-queue_.try-send msg:
      num-dropped-messages++

  payload-size-length_ payload-size/int:
    if payload-size == PAYLOAD-SIZE-8: return 8
    if payload-size == PAYLOAD-SIZE-64: return 64
    throw "UNKNOWN PAYLOAD SIZE"

  process-interrupts_ -> bool:
    has-events := false
    while true:
      device-mutex_.do:
        32.repeat:
          flags := read-u16_ device_ INT-REG_ 0
          if flags == 0: return has-events
          has-events = true

          if flags & ~KNOWN-FLAGS_ != 0: throw "UNEXPECTED FLAGS: 0x$(flags.stringify 16)"

          if flags & (1 << 12) != 0:
            write-u8_ device_ INT-REG_ 1 ~(1 << 4)
            recover_ device_
            return true

          // RXIF.
          if flags & (1 << 1) != 0:
            receive_ device_

          // TXATIF.
          if flags & (1 << 10) != 0:
            // Clear Pending Transmit Attempt interrupt bit.
            write-u8_ device_ (fifosta-reg_ TRANSMIT-FIFO-INDEX_) 0 ~(1 << 4)

          // TXIF.
          if flags & (1 << 0) != 0:
            transmit-fifo-full_ = false
            // Transmit complete, disable not-full interrupt.
            tra-con0 := FIFO-TX-CON0_
            write-u8_ device_ (fifocon-reg_ TRANSMIT-FIFO-INDEX_) 0 tra-con0

          // MODIF
          if flags & (1 << 3) != 0:
            // Clear the MODIF bit.
            write-u8_ device_ INT-REG_ 0 ~(1 << 3)

          // RXOVIF
          if flags & (1 << 11) != 0:
            // Clear RXOVIF flag on receive buffer.
            write-u8_ device_ (fifosta-reg_ RECEIVE-FIFO-INDEX_) 0 ~(1 << 3)
            num-dropped-messages++

          // IVMIF Invalid message occurred.
          if flags & (1 << 15) != 0:
            num-invalid-messages++
            // Clear the IVMIF bit.
            write-u8_ device_ INT-REG_ 1 ~(1 << 7)

          // CERRIF CAN Bus error.
          if flags & (1 << 13) != 0:
            num-can-bus-errors++
            // Clear the IVMIF bit.
            write-u8_ device_ INT-REG_ 1 ~(1 << 5)


      // Sleep for a short while to not starve.
      sleep --ms=2

  // Configures a FIFO buffer..
  // Index is 1-based.
  configure-fifo_ device/spi.Device index/int --size/int=1 --tx=false --priority/int=0:
    if tx:
      // Set priority and retry policy.
      con2 := (RETRY-DISABLED_ << 5) | priority
      write-u8_ device (fifocon-reg_ index) 2 con2

    // Set payload and size.
    con3 := (max-payload_ << 5) | (size - 1)
    write-u8_ device (fifocon-reg_ index) 3 con3

    con0 := tx ? FIFO-TX-CON0_ : FIFO-RX-CON0_
    write-u8_ device (fifocon-reg_ index) 0 con0

  enter-mode_ device/spi.Device mode/int:
    con3 := mode
    with-timeout --ms=50:
      while true:
        write-u8_ device CON-REG_ 3 con3
        if (get-mode_ device) == mode: break
        sleep --ms=1

  get-mode_ device/spi.Device -> int:
    val := read-u8_ device CON-REG_ 2
    return val >> 5

  test-memory_ device/spi.Device:
    for i := 1; i <= int.MAX-U32; i <<= 1:
      write-u32_ device RAM-START-REG_ i
      if i != (read-u32_ device RAM-START-REG_): throw "SPI ERROR"

  // Writes 0 into all ram words.
  clear-memory_ device/spi.Device:
    for reg := RAM-START-REG_; reg < RAM-END-REG_; reg += 4:
      write-u32_ device reg 0

  read-u8_ device/spi.Device reg/int byte-offset/int -> int:
    reg += byte-offset
    io.BIG-ENDIAN.put-uint16 buffer_ 0 reg | SPI-READ-COMMAND_
    device.transfer buffer_ --read --to=3
    return buffer_[2]

  read-u16_ device/spi.Device reg/int byte-offset/int -> int:
    io.BIG-ENDIAN.put-uint16 buffer_ 0 reg | SPI-READ-COMMAND_
    device.transfer buffer_ --read --to=4
    return io.LITTLE-ENDIAN.uint16 buffer_ 2

  read-u32_ device/spi.Device reg/int -> int:
    io.BIG-ENDIAN.put-uint16 buffer_ 0 reg | SPI-READ-COMMAND_
    device.transfer buffer_ --read --to=6
    return io.LITTLE-ENDIAN.uint32 buffer_ 2

  write-u8_ device/spi.Device reg/int byte-offset/int value/int:
    reg += byte-offset
    io.BIG-ENDIAN.put-uint16 buffer_ 0 reg | SPI-WRITE-COMMAND_
    buffer_[2] = value
    device.transfer buffer_ --to=3

  write-u32_ device/spi.Device reg/int value/int:
    io.BIG-ENDIAN.put-uint16 buffer_ 0 reg | SPI-WRITE-COMMAND_
    io.LITTLE-ENDIAN.put-uint32 buffer_ 2 value
    device.transfer buffer_ --to=6

  reset_ device/spi.Device:
    zeros := ByteArray 2
    device.transfer zeros

  static fifocon-reg_ index:
    return 0x05C + 12 * (index - 1)

  static fifosta-reg_ index:
    return 0x060 + 12 * (index - 1)

  static fifoua-reg_ index:
    return 0x064 + 12 * (index - 1)

  static filter-con-reg_ index:
    return 0x1D0 + 8 * index

  static filter-obj-reg_ index:
    return 0x1F0 + 8 * index

  static filter-mask-reg_ index:
    return 0x1F4 + 8 * index
