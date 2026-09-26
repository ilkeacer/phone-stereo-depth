"""Bounded v1 USB snapshot decoder; deliberately independent of Tk/calibration UI."""
import json
import socket
import struct
from host.contracts import validate_header


def receive_exact(sock, length):
    if type(length) is not int or not 0 <= length <= 8_000_000:
        raise ValueError('Invalid packet length')
    data = bytearray()
    while len(data) < length:
        part = sock.recv(length - len(data))
        if not part:
            raise ConnectionError('USB connection closed')
        data.extend(part)
    return bytes(data)


def read_pair(port=8765):
    with socket.create_connection(('127.0.0.1', port), timeout=2) as sock:
        length = struct.unpack('>I', receive_exact(sock, 4))[0]
        if not 1 <= length <= 65_536:
            raise ValueError('Invalid header length')
        header = json.loads(receive_exact(sock, length))
        validate_header(header)
        if not header['ok']:
            return None
        blobs = [receive_exact(sock, header[c]['length']) for c in ('20', '21')]
    return header, blobs
