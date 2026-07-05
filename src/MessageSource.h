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

  // Read up to max_len bytes from the transport into buf (non-blocking).
  // Returns 0 if no bytes are available yet. Used by CommandProcessor to
  // stream the body of bulk-write commands (0x85) without buffering the
  // whole file.
  virtual size_t readBulkBytes(uint8_t* buf, size_t max_len) { return 0; }

  // Flush any queued response frame and then write up to `len` raw bytes
  // directly to the transport without framing. Used by 0x84 get-pattern-file
  // to stream file data after the header response. Returns the number of
  // bytes actually written — short of `len` means the transport stalled
  // (e.g. host stopped draining); callers must treat a short return as an
  // abort signal rather than retrying the remainder inline, since retrying
  // is exactly the unbounded spin this return value exists to avoid.
  // Default is a no-op so non-streaming MessageSource subclasses don't need
  // to override it.
  virtual size_t sendRaw(const uint8_t* buf, size_t len) { return 0; }

  // Release the currently-pending parsed command (hasCommand() -> false),
  // letting the transport parse its next one. Exposed on the base interface
  // so CommandProcessor can defer this call on whichever concrete source
  // (net_ or serial_) started an async bulk transfer — see ul_source_/
  // dl_source_ — without needing to know which one it is.
  virtual void commandConsumed() {}
};
