"""Find Windows Webots without assuming that the DNS server is the host."""

import ipaddress
from pathlib import Path
import shutil
import socket
import struct
import subprocess

from webots_ros2_driver.utils import is_wsl


def windows_host(requested='auto'):
    """Use localhost for mirrored WSL and the default gateway for NAT WSL."""
    if requested != 'auto':
        if not requested or any(character.isspace() for character in requested):
            raise ValueError('webots_host must be an IPv4 address or hostname')
        return requested
    if not is_wsl():
        return '127.0.0.1'
    if shutil.which('wslinfo'):
        try:
            result = subprocess.run(
                ['wslinfo', '--networking-mode'], capture_output=True,
                text=True, timeout=3, check=False)
            if result.returncode == 0 and result.stdout.strip().lower() == 'mirrored':
                return '127.0.0.1'
        except (OSError, subprocess.TimeoutExpired):
            pass
    try:
        for line in Path('/proc/net/route').read_text().splitlines()[1:]:
            fields = line.split()
            if len(fields) >= 4 and fields[1] == '00000000' and int(fields[3], 16) & 2:
                return socket.inet_ntoa(struct.pack('<I', int(fields[2], 16)))
    except (OSError, ValueError):
        pass
    try:
        result = subprocess.run(
            ['ip', '-4', 'route', 'show', 'default'], capture_output=True,
            text=True, timeout=3, check=False)
        for line in result.stdout.splitlines():
            fields = line.split()
            if result.returncode == 0 and 'via' in fields:
                return str(ipaddress.IPv4Address(fields[fields.index('via') + 1]))
    except (OSError, ValueError, IndexError, subprocess.TimeoutExpired):
        pass
    raise RuntimeError(
        'Cannot detect the Windows host. Pass webots_host:=<Windows gateway IP> '
        '(ip route show default in NAT, or 127.0.0.1 in mirrored mode).')
