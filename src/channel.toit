// Copyright (C) 2021 Toitware ApS. All rights reserved.

/**
A one-way communication channel between tasks.
Multiple messages (objects) can be sent, and the capacity indicates how many
  unreceived message it can buffer.

# Inheritance
This class must not be extended.
*/
monitor Channel_:
  /** Constructs a channel with a buffer of the given $capacity. */
  constructor capacity:
    buffer_ = List capacity + 1

  /**
  Sends a message with the $value on the channel.
  This operation may block if the buffer capacity has been reached. In that
    case, this task waits until another task calls $receive.
  If there are tasks blocked waiting for a value (with $receive), then one of
    them is woken up and receives the $value.
  */
  send value:
    n := 0
    await:
      n = (p_ + 1) % buffer_.size
      n != c_
    buffer_[p_] = value
    p_ = n

  try_send value -> bool:
    n := (p_ + 1) % buffer_.size
    if n == c_: return false
    buffer_[p_] = value
    p_ = n
    return true

  /**
  Receives a message from the channel.
  If no message is ready, blocks until another tasks sends a message through
    $send.
  If multiple tasks are blocked waiting for a new value, then a $send call only
    unblocks one waiting task.
  The order in which waiting tasks are unblocked is unspecified.
  */
  receive:
    await: c_ != p_
    value := buffer_[c_]
    c_ = (c_ + 1) % buffer_.size
    return value

  try_receive:
    if c_ == p_: return null
    value := buffer_[c_]
    c_ = (c_ + 1) % buffer_.size
    return value

  buffer_ := ?
  c_ := 0
  p_ := 0
