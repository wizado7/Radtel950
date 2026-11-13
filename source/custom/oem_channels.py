# SPDX-License-Identifier: GPL-3.0-or-later
"""Export occupied OEM 0.29 channel records from a local file to CSV."""
import argparse
import csv
from pathlib import Path

CHANNELS = 990
PER_ZONE = 99
RECORD = 32
CHANNEL_BYTES = CHANNELS * RECORD
DUMP_BYTES = 4 * 1024 * 1024


def frequency(data):
    value = 0
    for byte in reversed(data):
        high, low = byte >> 4, byte & 15
        if high > 9 or low > 9:
            raise ValueError('invalid BCD frequency')
        value = value * 100 + high * 10 + low
    return value * 10


def records(data):
    if len(data) not in (CHANNEL_BYTES, DUMP_BYTES):
        raise ValueError('expected a 31680-byte OEM channel region or a 4 MiB external dump')
    for index in range(CHANNELS):
        record = data[index * RECORD:(index + 1) * RECORD]
        # Same presence check as OEM 0.29 at 0x0800AE94.
        if record[0] == 255 or record[3] == 0:
            continue
        try:
            rx = frequency(record[:4])
            tx = 0 if record[4:8] == b'\xff' * 4 else frequency(record[4:8])
        except ValueError as error:
            raise ValueError(f'channel {index + 1}: {error}') from error
        yield (index + 1, index // PER_ZONE + 1, index % PER_ZONE + 1, rx, tx,
               record[8:10].hex(), record[10:12].hex(),
               f'{record[14]:02x}', f'{record[15]:02x}', record[20:].hex())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    try:
        with args.source.open('rb') as source:
            rows = list(records(source.read(DUMP_BYTES + 1)))
        with args.output.open('x', encoding='utf-8', newline='') as output:
            writer = csv.writer(output)
            writer.writerow(('channel', 'zone', 'zone_channel', 'rx_hz', 'stored_tx_hz',
                             'rx_tone_hex', 'tx_tone_hex', 'byte14_hex', 'flags_hex', 'name_bytes_hex'))
            writer.writerows(rows)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(f'{len(rows)} channels exported')


if __name__ == '__main__':
    main()
