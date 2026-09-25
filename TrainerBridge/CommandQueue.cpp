#include "CommandQueue.h"

CommandQueue::CommandQueue()
  : _head(0),
    _tail(0),
    _count(0),
    _mux(portMUX_INITIALIZER_UNLOCKED)
{
}

bool CommandQueue::isCoalescentOpcode(uint8_t opcode)
{
  return (opcode == FTMS_CP_OP_SET_INDOOR_BIKE_SIM ||
          opcode == FTMS_CP_OP_SET_TARGET_POWER ||
          opcode == FTMS_CP_OP_SET_TARGET_RESISTANCE);
}

bool CommandQueue::push(const uint8_t *data, size_t len, uint8_t opcode, int16_t targetValue)
{
  if (data == nullptr || len == 0 || len > MAX_PAYLOAD_LEN) {
    return false;
  }

  portENTER_CRITICAL(&_mux);

  // If opcode is coalescent, search for an existing pending command of the same opcode
  if (isCoalescentOpcode(opcode) && _count > 0) {
    for (size_t i = 0; i < _count; i++) {
      size_t idx = (_head + i) % CAPACITY;
      if (_buffer[idx].opcode == opcode) {
        // Coalesce in-place with latest parameter values
        memcpy(_buffer[idx].data, data, len);
        _buffer[idx].len = (uint8_t)len;
        _buffer[idx].targetValue = targetValue;
        portEXIT_CRITICAL(&_mux);
        return true;
      }
    }
  }

  // If not coalesced, ensure there is space in the queue
  if (_count >= CAPACITY) {
    portEXIT_CRITICAL(&_mux);
    return false;
  }

  _buffer[_tail].opcode = opcode;
  _buffer[_tail].len = (uint8_t)len;
  _buffer[_tail].targetValue = targetValue;
  memcpy(_buffer[_tail].data, data, len);

  _tail = (_tail + 1) % CAPACITY;
  _count++;

  portEXIT_CRITICAL(&_mux);
  return true;
}

bool CommandQueue::pop(QueuedCommand &outCmd)
{
  portENTER_CRITICAL(&_mux);

  if (_count == 0) {
    portEXIT_CRITICAL(&_mux);
    return false;
  }

  outCmd = _buffer[_head];
  _head = (_head + 1) % CAPACITY;
  _count--;

  portEXIT_CRITICAL(&_mux);
  return true;
}

void CommandQueue::clear()
{
  portENTER_CRITICAL(&_mux);
  _head = 0;
  _tail = 0;
  _count = 0;
  portEXIT_CRITICAL(&_mux);
}

size_t CommandQueue::count()
{
  portENTER_CRITICAL(&_mux);
  size_t c = _count;
  portEXIT_CRITICAL(&_mux);
  return c;
}

bool CommandQueue::isEmpty()
{
  return count() == 0;
}

bool CommandQueue::isFull()
{
  return count() >= CAPACITY;
}
