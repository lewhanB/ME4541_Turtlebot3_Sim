"""Check Windows endpoint selection across WSL networking modes."""

from types import SimpleNamespace

import pytest

from robot_bringup import wsl_network


def test_explicit_host_avoids_network_detection(monkeypatch):
    monkeypatch.setattr(wsl_network, 'is_wsl', lambda: True)
    monkeypatch.setattr(wsl_network.subprocess, 'run', lambda *a, **k: pytest.fail('unexpected probe'))
    assert wsl_network.windows_host('172.28.208.1') == '172.28.208.1'


def test_mirrored_uses_localhost_instead_of_dns(monkeypatch):
    monkeypatch.setattr(wsl_network, 'is_wsl', lambda: True)
    monkeypatch.setattr(wsl_network.shutil, 'which', lambda name: '/usr/bin/wslinfo')
    monkeypatch.setattr(wsl_network.subprocess, 'run', lambda *a, **k: SimpleNamespace(
        returncode=0, stdout='mirrored\n'))
    assert wsl_network.windows_host() == '127.0.0.1'


def test_nat_uses_little_endian_route_gateway(monkeypatch):
    monkeypatch.setattr(wsl_network, 'is_wsl', lambda: True)
    monkeypatch.setattr(wsl_network.shutil, 'which', lambda name: None)
    monkeypatch.setattr(wsl_network.Path, 'read_text', lambda self: (
        'Iface Destination Gateway Flags\neth0 00000000 01D01CAC 0003\n'))
    assert wsl_network.windows_host() == '172.28.208.1'


def test_missing_route_requires_explicit_host(monkeypatch):
    monkeypatch.setattr(wsl_network, 'is_wsl', lambda: True)
    monkeypatch.setattr(wsl_network.shutil, 'which', lambda name: None)
    monkeypatch.setattr(wsl_network.Path, 'read_text', lambda self: '')
    monkeypatch.setattr(wsl_network.subprocess, 'run', lambda *a, **k: SimpleNamespace(
        returncode=1, stdout=''))
    with pytest.raises(RuntimeError, match='webots_host'):
        wsl_network.windows_host()
