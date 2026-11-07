"""Read-only, bounded evidence extraction from the verified OEM 0.29 image.

String matches and direct BL references are leads, not a complete call graph
or proof of a particular hardware sensor. This tool never connects to a radio.
"""
import argparse
import hashlib
import re
import struct
from pathlib import Path

BASE = 0x08003000
EXPECTED = '025bc583802a96911836e274cdfdf3ce05d9919bc77451b7f7f1573fb5571907'


def audit(stock: bytes, calibration: bytes | None = None) -> str:
    digest = hashlib.sha256(stock).hexdigest()
    if digest != EXPECTED:
        raise ValueError('unexpected raw 0.29 SHA-256; do not reuse these addresses')
    lines = [f'SHA256 {digest}', f'BASE {BASE:#010x}; bytes={len(stock)}',
             'ASCII strings are leads; missing strings do not prove missing code.']
    terms = re.compile(r'temper|thermal|volt|batter|calibr|factory|diagnos|debug|test mode|super mode|144-146|rssi|power', re.I)
    for match in re.finditer(rb'[\x20-\x7e]{4,}', stock):
        value = match.group().decode('ascii')
        if terms.search(value) and len(value) < 120:
            lines.append(f'STRING {BASE + match.start():#010x} {value}')
    for word in ('温度', '过热', '电流', '驻波'):
        for encoding in ('gbk', 'utf-8', 'utf-16-le'):
            needle = word.encode(encoding)
            hits = [f'{BASE + match.start():#010x}' for match in re.finditer(re.escape(needle), stock)]
            lines.append(f'LABEL {word} {encoding}: ' + (', '.join(hits) if hits else 'no match'))
    targets = {0x0800D960: 'All Band Test Mode display handler',
               0x08016988: 'ADC2 channel1 / battery',
               0x080169C4: 'ADC2 channel0 / VOX'}
    for offset in range(0, len(stock) - 3, 2):
        first, second = struct.unpack_from('<HH', stock, offset)
        if first & 0xF800 != 0xF000 or second & 0xD000 != 0xD000:
            continue
        sign = first >> 10 & 1
        branch = ((sign << 24) | ((1 ^ (second >> 13 & 1) ^ sign) << 23) |
                  ((1 ^ (second >> 11 & 1) ^ sign) << 22) |
                  ((first & 1023) << 12) | ((second & 2047) << 1))
        if sign:
            branch -= 1 << 25
        target = BASE + offset + 4 + branch
        if target in targets:
            lines.append(f'BL {BASE + offset:#010x} -> {target:#010x} {targets[target]}')
    lines += ['Verified disassembly leads:',
              '0x08009954..0x08009992: scanner code 8, flag +0x4a, marker 0xa5, display call.',
              '0x0800d96a..0x0800d976: language byte 0x2000a8fc+8 and ADR to 0x0800d9c4.',
              '0x08009bf0: battery table from external SPI 0xf200; 0x08009c22 fixes high=187.',
              '0x080099b6..0x08009a1c: flag +0x49 values 0xa5, 0x56, 0; save call 0x08013120.',
              '0x0800d9e8..0x0800da8a: mode display; 0x0800da98=144-146/430-440, 0x0800daa8=Super Mode.',
              '0x08013120..0x080131ba: external settings journal at 0xe000 / records from 0xe020.',
              '0x08024534..0x08024574: SPI Page Program command 0x02 and 24-bit address.',
              'These observations do not establish a temperature/current/VSWR diagnostic screen.']
    if calibration is not None:
        if len(calibration) != 4 * 1024 * 1024:
            raise ValueError('expected separate 4 MiB external-memory backup')
        table = calibration[0xF200:0xF207]
        lines.append('BACKUP SPI 0xf200: ' + table.hex(' '))
        lines.append(f'Battery lower raw from backup={table[0]}; OEM upper raw=187; units are ADC raw.')
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stock', type=Path, required=True)
    parser.add_argument('--calibration-dump', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if output in {args.stock.resolve(), args.calibration_dump.resolve() if args.calibration_dump else None}:
        parser.error('report must not overwrite either input image')
    report = audit(args.stock.read_bytes(), args.calibration_dump.read_bytes() if args.calibration_dump else None)
    output.write_text(report, encoding='utf-8')
    # Windows consoles may use a Western code page; keep the saved report UTF-8.
    print(report.encode('ascii', 'backslashreplace').decode('ascii'), end='')


if __name__ == '__main__':
    main()
