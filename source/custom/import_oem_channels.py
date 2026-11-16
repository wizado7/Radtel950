# SPDX-License-Identifier: GPL-3.0-or-later
"""Merge selected OEM FM channels into a copy of a Custom UI backup."""
import argparse
from pathlib import Path
import re
import struct
import zlib

from oem_channels import CHANNELS, CHANNEL_BYTES, DUMP_BYTES, PER_ZONE, RECORD, frequency
from settings_backup import HEADER, inspect_backup, load_backup, write_backup

# Canonical word offsets in settings_snapshot.cpp, shared by schemas 1 through 4.
VFO_BASE, VFO_WORDS = 29, 18
CHANNEL_BASE, CHANNEL_WORDS, MEMORY_COUNT = 218, 26, 64
SCAN_EXCLUDED = CHANNEL_BASE + MEMORY_COUNT * CHANNEL_WORDS + 3
CTCSS = (670, 693, 719, 744, 770, 797, 825, 854, 885, 915,
         948, 974, 1000, 1035, 1072, 1109, 1148, 1188, 1230, 1273,
         1318, 1365, 1413, 1462, 1514, 1567, 1598, 1622, 1655, 1679,
         1713, 1738, 1773, 1799, 1835, 1862, 1899, 1928, 1966, 1995,
         2035, 2065, 2107, 2181, 2257, 2291, 2336, 2418, 2503, 2541)


def tone(data):
    value = int.from_bytes(data, 'little')
    if value in (0, 65535):
        return (0, 0)
    if 1 <= value <= 210:
        return (2, value)
    if value in CTCSS:
        return (1, CTCSS.index(value) + 1)
    raise ValueError('unsupported tone')


def supported_frequency(hz):
    return 18_000_000 <= hz < 30_000_000 or 136_000_000 <= hz <= 174_000_000 or 400_000_000 <= hz <= 470_000_000


def channel_words(record, source):
    if record[0] == 255 or record[3] == 0:
        raise ValueError('empty OEM channel')
    rx = frequency(record[:4])
    tx = 0 if record[4:8] == b'\xff' * 4 else frequency(record[4:8])
    if not supported_frequency(rx) or (tx and not supported_frequency(tx)):
        raise ValueError('frequency outside Custom UI channel bands')
    flags = record[15]
    if flags & 0xB1 or record[14] & 0xF0 or record[12] & 15 or record[13] & 15:
        raise ValueError('AM, signalling, PTT ID, scrambler, encryption or FHSS requires manual setup')
    power = record[14] & 15
    if power > 2:
        raise ValueError('unsupported power code')
    raw_name = re.split(rb'[\x00\xff]', record[20:], maxsplit=1)[0]
    name = re.sub(rb'[^A-Z0-9-]+', b'-', raw_name.upper()).strip(b'-')[:8]
    name = (name or f'OEM{source:03}'.encode('ascii')).ljust(9, b'\0')
    # TX inhibit is intentional, even when the source allows TX.
    return (1, rx, tx, (source - 1) // PER_ZONE, *name,
            0, 0, 0, 1, *tone(record[8:10]), *tone(record[10:12]),
            60, 3, 2 - power, int(bool(flags & 0x40)), int(bool(flags & 0x08)))


def merge(dump, backup, mapping):
    inspect_backup(backup)
    if len(dump) not in (CHANNEL_BYTES, DUMP_BYTES):
        raise ValueError('expected a 31680-byte OEM channel region or a 4 MiB external dump')
    header = bytearray(backup[:HEADER])
    payload = backup[HEADER:]
    words = list(struct.unpack(f'<{len(payload)//4}I', payload))
    destinations = set()
    for source, destination in mapping:
        if not 1 <= source <= CHANNELS or not 1 <= destination <= MEMORY_COUNT:
            raise ValueError('expected OEM channel 1–990 and destination 1–64')
        if destination in destinations:
            raise ValueError(f'duplicate destination {destination}')
        destinations.add(destination)
        offset = (source - 1) * RECORD
        try:
            record = dump[offset:offset + RECORD]
            channel = channel_words(record, source)
        except ValueError as error:
            raise ValueError(f'OEM channel {source}: {error}') from error
        index = destination - 1
        start = CHANNEL_BASE + index * CHANNEL_WORDS
        words[start:start + CHANNEL_WORDS] = channel
        mask = 1 << (index % 32)
        if record[15] & 4:
            words[SCAN_EXCLUDED + index // 32] &= ~mask
        else:
            words[SCAN_EXCLUDED + index // 32] |= mask
        for slot in range(3):
            vfo = VFO_BASE + slot * VFO_WORDS
            if words[vfo + 3] == index:
                words[vfo + 4] = 0  # Keep working VFO rather than recalling a replacement.
    payload = struct.pack(f'<{len(words)}I', *words)
    struct.pack_into('<I', header, 20, zlib.crc32(payload))
    struct.pack_into('<I', header, 44, zlib.crc32(header[:44]))
    return bytes(header) + payload


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dump', type=Path)
    parser.add_argument('backup', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--map', dest='mapping', action='append', required=True, metavar='OEM:DEST')
    args = parser.parse_args()
    try:
        mapping = [tuple(map(int, item.split(':'))) for item in args.mapping]
        with args.dump.open('rb') as source:
            dump = source.read(DUMP_BYTES + 1)
        output = merge(dump, load_backup(args.backup), mapping)
        write_backup(args.output, output)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    for source, destination in mapping:
        print(f'OEM {source} -> CH {destination}, zone {(source - 1)//PER_ZONE + 1}, TX inhibited')


if __name__ == '__main__':
    main()
