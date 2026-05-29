#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Abstract interface for command-message sources (TCP, USB serial, ...).
// CommandProcessor uses this so it can route a response back to whichever
// transport originated the command being processed.
//
// Response framing: [length, status, echo_cmd, ...payload]
//   length  = count of bytes after the length byte (status + echo + payload)
//   status  = 0 on success, non-zero on error
//   payload = ASCII message for human-facing commands, or raw bytes for
//             machine-readable ones (e.g. get-controller-info).
class MessageSource {
 public:
  virtual ~MessageSource() = default;

  // Primary: queue a response carrying a raw-byte payload.
  virtual void sendResponse(uint8_t cmd_echo, uint8_t status,
                            const uint8_t *payload, size_t payload_len) = 0;

  // Convenience: queue a response carrying a NUL-terminated ASCII message.
  void sendResponse(uint8_t cmd_echo, uint8_t status, const char *message) {
    sendResponse(cmd_echo, status,
                 reinterpret_cast<const uint8_t *>(message),
                 message ? strlen(message) : 0);
  }
};
