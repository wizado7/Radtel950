# SPDX-License-Identifier: GPL-3.0-or-later
"""Custom UI settings backup/upload. No direct flash addresses; upload needs radio confirmation."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import time
import zlib

MAGIC = b'RT950CFG'
REQUEST = MAGIC + b'\n'
HEADER = 48
MAX_PAYLOAD = 8192
CANONICAL_SIZE = 7992
PORTABLE_SIZE = 8008
KEY_ACTION_COUNT = 12


def parse_header(header):
    if len(header) != HEADER or header[:8] != MAGIC:
        raise ValueError('not a Custom UI settings backup')
    protocol, size, schema, payload_crc = struct.unpack_from('<4I', header, 8)
    if protocol != 1 or (schema, size) not in ((1, CANONICAL_SIZE), (2, PORTABLE_SIZE)):
        raise ValueError('unsupported backup protocol, schema or payload length')
    if struct.unpack_from('<I', header, 40)[0] != 0:
        raise ValueError('unsupported backup flags')
    if zlib.crc32(header[:44]) != struct.unpack_from('<I', header, 44)[0]:
        raise ValueError('backup header CRC mismatch')
    version_bytes = header[24:40]
    if b'\0' not in version_bytes:
        raise ValueError('unterminated firmware version')
    version, padding = version_bytes.split(b'\0', 1)
    if any(padding) or not re.fullmatch(rb'[0-9]+\.[0-9]+(?:\.[0-9]+)?', version):
        raise ValueError('invalid firmware version')
    return size, payload_crc, version.decode('ascii')


def inspect_backup(data):
    size, expected_crc, version = parse_header(data[:HEADER])
    if len(data) != HEADER + size:
        raise ValueError('truncated backup or trailing data')
    payload = data[HEADER:]
    if zlib.crc32(payload) != expected_crc:
        raise ValueError('backup payload CRC mismatch')
    words = struct.unpack('<' + str(size//4) + 'I', payload)
    schema = struct.unpack_from('<I', data, 16)[0]
    if schema == 2 and any(action >= KEY_ACTION_COUNT for action in words[CANONICAL_SIZE//4:]):
        raise ValueError('invalid user key action')
    if words[0] not in (0, 1):
        raise ValueError('invalid language field')
    # Canonical v1: 29 base words, 3 VFOs x18, 15 zones x9, 64 memories x26.
    channel_base = 29 + 3 * 18 + 15 * 9
    occupied = [words[channel_base + i * 26] for i in range(64)]
    if any(value not in (0, 1) for value in occupied):
        raise ValueError('invalid channel flags')
    return {
        'firmware': version, 'protocol': 1, 'snapshot_schema': schema,
        'payload_bytes': size, 'file_bytes': len(data),
        'occupied_channels': sum(occupied),
        'language': ('English', 'Russian')[words[0]],
        'sha256': hashlib.sha256(data).hexdigest(),
        'checked': 'header CRC, payload CRC, canonical size and basic structure',
    }


def read_backup(port, wait_seconds=35, transfer_seconds=20, clock=time.monotonic):
    """Bounded resynchronization; request only, no CPS/program/erase commands."""
    waiting_until = clock() + wait_seconds
    next_request = clock()
    pending = bytearray()
    while clock() < waiting_until:
        if clock() >= next_request:
            if port.write(REQUEST) != len(REQUEST):
                raise ValueError('short serial request write')
            next_request = clock() + 0.5
        chunk = port.read(256)
        if chunk:
            pending.extend(chunk)
        location = pending.find(MAGIC)
        if location < 0:
            del pending[:-7]
            continue
        if location:
            del pending[:location]
        if len(pending) >= HEADER:
            # Ignore noise that happens to start like a header, then resync.
            try:
                size, _, _ = parse_header(pending[:HEADER])
            except ValueError:
                del pending[0]
                continue
            break
    else:
        raise TimeoutError('no backup header; open System > Export settings and press OK')
    deadline = clock() + transfer_seconds
    total = HEADER + size
    while len(pending) < total and clock() < deadline:
        chunk = port.read(min(256, total - len(pending)))
        if chunk:
            pending.extend(chunk)
    if len(pending) != total:
        raise TimeoutError('incomplete backup; keep export open and do not press PTT')
    result = bytes(pending)
    inspect_backup(result)
    return result


def upload_backup(port, data, wait_seconds=35, clock=time.monotonic):
    """Upload only a validated container, paced by device acknowledgements.

    CHECKED means staged in RAM, not applied or saved. The operator must
    confirm on the radio. There is no address/erase/reset/program command.
    """
    inspect_backup(data)  # Before sending any bytes, even the handshake.
    pending = bytearray()
    transfer_until = None

    def send(block):
        if port.write(block) != len(block):
            raise ValueError('short serial upload write')

    def reply(expected, seconds):
        deadline = clock() + seconds
        if transfer_until is not None:
            deadline = min(deadline, transfer_until)
        while clock() < deadline:
            block = port.read(32)
            if block:
                pending.extend(block)
            while b'\n' in pending:
                line, _, tail = pending.partition(b'\n')
                pending[:] = tail
                if line == b'ERROR':
                    raise ValueError('radio rejected backup; live settings unchanged')
                if line + b'\n' == expected:
                    return
            if len(pending) > 64:
                raise ValueError('invalid upload acknowledgement')
        raise TimeoutError('upload acknowledgement timed out; no automatic retry or apply')

    send(b'RT950PUT\n')
    reply(b'READY\n', wait_seconds)
    transfer_until = clock() + 55
    send(data[:HEADER])
    reply(b'NEXT\n', 3)
    payload = data[HEADER:]
    for at in range(0, len(payload), 32):
        block = payload[at:at+32]
        send(block)
        reply(b'CHECKED\n' if at+len(block) == len(payload) else b'NEXT\n', 3)
    return inspect_backup(data)


def write_backup(path, data):
    inspect_backup(data)
    # Verify before creating a file; never overwrite an existing backup.
    with Path(path).open('xb') as target:
        target.write(data)


def load_backup(path):
    # Read at most one container plus a sentinel for trailing data.
    with Path(path).open('rb') as source:
        data = source.read(HEADER + PORTABLE_SIZE + 1)
    inspect_backup(data)
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--list-ports', action='store_true')
    modes.add_argument('--inspect', type=Path, metavar='FILE')
    modes.add_argument('--port', metavar='COMx')
    parser.add_argument('--output', type=Path, metavar='FILE')
    parser.add_argument('--upload', type=Path, metavar='FILE', help='stage backup; requires confirmation on radio')
    args = parser.parse_args()
    if (args.output or args.upload) and not args.port:
        parser.error('--output/--upload require --port COMx')
    if args.inspect:
        print(json.dumps(inspect_backup(load_backup(args.inspect)), indent=2))
        return
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        parser.error('serial access requires pyserial: python -m pip install pyserial')
    if args.list_ports:
        for port in list_ports.comports():
            print(f'{port.device}: {port.description}')
        return
    if bool(args.output) == bool(args.upload):
        parser.error('--port requires exactly one of --output FILE or --upload FILE')
    upload_data = load_backup(args.upload) if args.upload else None
    if upload_data is not None:
        inspect_backup(upload_data)
    if args.output and args.output.exists():
        parser.error('output already exists; choose another filename')
    # Configure line controls before opening. No reset, bootloader or flash commands.
    connection = serial.Serial(port=None, baudrate=115200, timeout=0.1, write_timeout=0.5)
    try:
        connection.dtr = False
        connection.rts = False
        connection.port = args.port
        connection.open()
        connection.reset_input_buffer()
        if upload_data is not None:
            upload_backup(connection, upload_data)
        else:
            data = read_backup(connection)
    finally:
        connection.close()
    if upload_data is not None:
        print('Checked in radio RAM. Select Apply import on the radio to apply; Cancel leaves settings unchanged.')
        return
    write_backup(args.output, data)
    print(json.dumps(inspect_backup(data), indent=2))
    print(f'Saved: {args.output}')


if __name__ == '__main__':
    main()
