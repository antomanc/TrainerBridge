#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include "FtmsConstants.h"

struct QueuedCommand {
  uint8_t opcode;
  uint8_t len;
  uint8_t data[20];
  int16_t targetValue;
};

class CommandQueue {
public:
  static constexpr size_t CAPACITY = 8;
  static constexpr size_t MAX_PAYLOAD_LEN = 20;

  CommandQueue();

  // Pushes a command to the queue with coalescing for continuous updates (0x11, 0x05, 0x04).
  // Thread-safe across ESP32 cores. Returns true on success, false if queue is full.
  bool push(const uint8_t *data, size_t len, uint8_t opcode, int16_t targetValue = 0);

  // Pops the next command in FIFO order.
  // Thread-safe across ESP32 cores. Returns true if a command was extracted.
  bool pop(QueuedCommand &outCmd);

  // Clears all pending commands from the queue.
  void clear();

  // Returns current number of queued commands.
  size_t count();

  bool isEmpty();
  bool isFull();

private:
  QueuedCommand _buffer[CAPACITY];
  size_t _head;
  size_t _tail;
  size_t _count;
  portMUX_TYPE _mux;

  // Helper to check if an opcode should coalesce in-place (must be called within critical section)
  static bool isCoalescentOpcode(uint8_t opcode);
};
