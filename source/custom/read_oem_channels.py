# SPDX-License-Identifier: GPL-3.0-or-later
"""Read the OEM 0.29 channel region over CPS into a new local file."""
import argparse
from pathlib import Path

from oem_channels import CHANNEL_BYTES

BLOCK = 128
ACK = b'\x06'
# OEM key selector 0x08022EB0, key table 0x08030993.
KEY_FRAME = b'SEND\x10' + bytes(20)
KEY = b'BHT '


def transform(block):
    """OEM 0x08022E5C uses the same transform in both directions."""
    result = bytearray(block)
    for i, value in enumerate(block):
        key = KEY[i % len(KEY)]
        if key != 32 and value not in (0, 255, key, key ^ 255):
            result[i] = value ^ key
    return bytes(result)


def read_exact(port, size):
    result = bytearray()
    while len(result) < size:
        chunk = port.read(size - len(result))
        if not chunk:
            raise ValueError('CPS reply timed out')
        result.extend(chunk)
    return bytes(result)


def send(port, data):
    if port.write(data) != len(data):
        raise ValueError('CPS request was only partially sent')


def read_channels(port):
    port.reset_input_buffer()
    send(port, b'PROGRAMBT9000U')
    if read_exact(port, 1) != ACK:
        raise ValueError('CPS handshake rejected')
    try:
        send(port, b'F')
        read_exact(port, 16)
        send(port, b'M')
        model = read_exact(port, 12).strip(b'\0 ')
        if model != b'RT-950':
            raise ValueError(f'unexpected CPS model reply {model!r}')
        send(port, KEY_FRAME)
        if read_exact(port, 1) != ACK:
            raise ValueError('CPS key rejected')
        result = bytearray()
        for address in range(0, CHANNEL_BYTES, BLOCK):
            request = b'R' + address.to_bytes(2, 'big') + bytes([BLOCK])
            send(port, request)
            reply = read_exact(port, BLOCK + 4)
            if reply[:4] != request:
                raise ValueError(f'unexpected CPS reply header at 0x{address:04X}')
            result.extend(transform(reply[4:]))
    except (OSError, ValueError):
        try:
            send(port, b'E')
        except (OSError, ValueError):
            pass
        raise
    else:
        send(port, b'E')
        return model.decode('ascii'), bytes(result[:CHANNEL_BYTES])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    try:
        import serial
        from serial.tools import list_ports
        if args.output.exists():
            raise ValueError('output already exists')
        if args.port.casefold() not in {port.device.casefold() for port in list_ports.comports()}:
            raise ValueError('port is not present in the current serial port list')
        with serial.Serial(args.port, 115200, timeout=1, write_timeout=1) as port:
            model, channels = read_channels(port)
        with args.output.open('xb') as output:
            output.write(channels)
    except (ImportError, OSError, ValueError) as error:
        parser.error(str(error))
    print(f'{model}: saved {len(channels)} channel bytes to {args.output}')


if __name__ == '__main__':
    main()
