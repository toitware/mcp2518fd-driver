// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import gpio
import binary
import serial.protocols.spi as spi

import mcp2518fd

MOSI := gpio.Pin 12
MISO := gpio.Pin 13
CLOCK := gpio.Pin 14
CS := gpio.Pin 15

INT := gpio.Pin 5 --pull_up

main:
  bus := spi.Bus --mosi=MOSI --miso=MISO --clock=CLOCK

  device := bus.device --cs=CS --frequency=20_000_000

  driver := mcp2518fd.Driver device
  driver.configure

  task::
    INT.config --input
    driver.run INT

  while true:
    msg := driver.receive
    print "received: $msg.id$msg.data"
