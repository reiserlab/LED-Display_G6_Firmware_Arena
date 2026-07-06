"""HIL test for the zero-length upload fix, PR #27 review point 7.

Source: reiserlab/LED-Display_G6_Firmware_Arena#27, review comment
pullrequestreview-4632788440. Finding and the fix implemented here:
debug/pr27-response.md, point 7 ("Zero-length 0x85 never completes").

Before this fix, handleBulkWriteCommand() accepted total_len == 0 and handed
off to serviceUpload() same as any other upload. serviceUpload()'s read
(readBulkBytes(chunk, 0)) always returns 0, and the early
`if (got == 0) return;` guard exits before the ul_remaining_ == 0 completion
check a few lines further down (which is only reached after a got > 0
write), so the transfer just idled until the 30 s upload timeout gave up on
it and reported "Upload timeout", where a 0-byte pattern file isn't a valid
input in the first place. handleBulkWriteCommand() now rejects total_len ==
0 outright, before any state checks or SD access, with a status != 0
response the caller gets back immediately.

Run:
    pixi run test-serial -- -k pr27_zero
    pixi run test-tcp    -- -k pr27_zero
"""

from .commands import ALL_OFF_CMD, SET_PATTERN_FILE_CMD


def test_zero_length_upload_rejected_immediately(transport):
    """A zero-byte SET_PATTERN_FILE (0x85) upload must be rejected right
    away, not accepted and left to time out 30 s later."""
    transport.command(ALL_OFF_CMD)

    # A short timeout is the point of the test: old firmware never responds
    # at all here until its own 30 s upload-idle deadline fires, so this
    # would raise (timeout) long before that if the fix regressed.
    st, echo, payload, _diag = transport.upload_file(0, b"", timeout=3.0)

    assert st != 0, "empty (zero-length) upload was accepted"
    assert echo == SET_PATTERN_FILE_CMD
    assert b"mpty" in bytes(payload), (
        f"expected an 'empty upload' style message, got: {bytes(payload)!r}"
    )
