"""TCP-specific tests.

Skipped automatically when --transport=serial is selected.
"""

import socket

import pytest

from .commands import GET_ETHERNET_IP_ADDRESS_CMD

pytestmark = pytest.mark.tcp_only


def test_tcp_nodelay_set(transport):
    val = transport._sock.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY)
    assert val != 0


def test_get_ethernet_ip_address(transport):
    st, echo, payload, _ = transport.command(GET_ETHERNET_IP_ADDRESS_CMD)
    assert st == 0
    assert echo == GET_ETHERNET_IP_ADDRESS_CMD
    assert len(payload) > 0
    ip_str = payload.decode("ascii", errors="replace").strip()
    assert "." in ip_str, f"payload does not look like an IP: {ip_str!r}"
