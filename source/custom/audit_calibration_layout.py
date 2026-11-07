"""Read-only OEM 0.29 calibration layout and PA selector audit. No radio I/O."""
import argparse
import csv
import hashlib
import json
import struct
from pathlib import Path

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE, UC_HOOK_MEM_READ
from unicorn.arm_const import UC_CPU_ARM_CORTEX_M4, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC

STOCK_SHA = '025bc583802a96911836e274cdfdf3ce05d9919bc77451b7f7f1573fb5571907'
LEVELS = ('High', 'Middle', 'Low')
GROUPS = (
    ('UHF', 104, (0xf000, 0xf040, 0xf080)),
    ('VHF', 116, (0xf010, 0xf050, 0xf090)),
    ('200_MHz_branch', 128, (0xf020, 0xf060, 0xf0a0)),
    ('350_MHz_branch', 140, (0xf030, 0xf070, 0xf0b0)),
    ('Low_band', 92, (0xf190, 0xf1a0, 0xf1b0)),
)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stock', type=Path, required=True)
    p.add_argument('--dump', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    stock, dump = args.stock.read_bytes(), args.dump.read_bytes()
    if sha(stock) != STOCK_SHA or len(dump) != 0x400000:
        p.error('expected verified raw 0.29 and separate 4 MiB memory dump')
    output = args.output.resolve()
    if any(output == path.resolve() or output in path.resolve().parents for path in (args.stock, args.dump)):
        p.error('output must not contain or replace either input')
    u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
    u.mem_map(0x08000000, 0x100000)
    u.mem_map(0x20000000, 0x18000)
    u.mem_write(0x08003000, stock)
    u.reg_write(UC_ARM_REG_SP, 0x20015f78)
    u.emu_start(0x08003185, 0x08003280, count=3000000)
    assert u.reg_read(UC_ARM_REG_PC) == 0x08003280
    reads = []

    def spi_read(machine, address, size, data):
        offset, destination, length = (machine.reg_read(reg) for reg in (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2))
        assert 0 <= offset < offset + length <= len(dump)
        assert 0x20000000 <= destination < destination + length <= 0x20018000
        reads.append({'offset': offset, 'ram': destination, 'length': length,
                      'hex': dump[offset:offset + length].hex(' ')})
        machine.mem_write(destination, dump[offset:offset + length])
        machine.reg_write(UC_ARM_REG_PC, machine.reg_read(UC_ARM_REG_LR))

    u.hook_add(UC_HOOK_CODE, spi_read, begin=0x08024424, end=0x08024424)
    u.reg_write(UC_ARM_REG_SP, 0x20017000)
    u.emu_start(0x080121fd, 0x0801232c, count=20000)
    assert u.reg_read(UC_ARM_REG_PC) == 0x0801232c
    assert dump[0xf0e0] not in (0, 255)
    table_rows, pointers = [], {}
    for group, pointer_offset, offsets in GROUPS:
        targets = struct.unpack('<3I', u.mem_read(0x200000d4 + pointer_offset, 12))
        pointers[group] = targets
        for selector, (offset, target) in enumerate(zip(offsets, targets)):
            assert any(item['offset'] == offset and item['ram'] == target and item['length'] == 16 for item in reads)
            assert u.mem_read(target, 16) == dump[offset:offset + 16]
            for index in range(16):
                table_rows.append({'group': group, 'level': LEVELS[selector], 'selector': selector,
                                   'index': index, 'offset': f'0x{offset + index:06X}',
                                   'code_decimal': dump[offset + index],
                                   'erased_or_unused': dump[offset + index] == 255})

    # Capture the byte address actually read by the OEM selector, not merely
    # the returned value: repeated DAC codes cannot identify the table cell.
    accesses = []
    table_ranges = [(item['ram'], item['ram'] + 16) for item in reads if item['offset'] in
                    {offset for _, _, offsets in GROUPS for offset in offsets}]

    def table_read(machine, access, address, size, value, data):
        if size == 1 and any(start <= address < end for start, end in table_ranges):
            accesses.append(address)

    u.hook_add(UC_HOOK_MEM_READ, table_read)
    checks = 0

    def select(hz, selector, group, index):
        nonlocal checks
        u.mem_write(0x20000c7c + 12, struct.pack('<I', hz // 10))
        u.mem_write(0x20000c7c + 23, bytes([selector]))
        u.reg_write(UC_ARM_REG_SP, 0x20017000)
        u.reg_write(UC_ARM_REG_LR, 0x080ff001)
        accesses.clear()
        u.emu_start(0x0801ea61, 0x080ff000, count=1000)
        assert u.reg_read(UC_ARM_REG_PC) == 0x080ff000
        expected = pointers[group][selector] + index
        assert accesses == [expected], (hz, selector, group, index, accesses, expected)
        value = u.mem_read(expected, 1)[0]
        assert u.reg_read(UC_ARM_REG_R0) == value
        checks += 1
        return value

    # Current custom TX ranges plus cell boundaries, without permitting RF.
    for selector in range(3):
        for hz in range(400000000, 470000001, 100000):
            select(hz, selector, 'UHF', (hz // 1000000 - 400) // 10)
        for hz in range(136000000, 174000001, 100000):
            select(hz, selector, 'VHF', (hz // 1000000 - 130) // 3)
        for hz in range(26000000, 28000000, 25000):
            select(hz, selector, 'Low_band', (hz // 1000000 - 15) // 5)
        for hz, group, index in ((220000000, '200_MHz_branch', 4),
                                 (260000000, '200_MHz_branch', 12),
                                 (350000000, '350_MHz_branch', 0),
                                 (390000000, '350_MHz_branch', 8)):
            select(hz, selector, group, index)
        for mhz in range(410, 471, 10):
            select(mhz * 1000000 - 10, selector, 'UHF', (mhz - 401) // 10)
            select(mhz * 1000000, selector, 'UHF', (mhz - 400) // 10)
        for mhz in range(139, 173, 3):
            select(mhz * 1000000 - 10, selector, 'VHF', (mhz - 131) // 3)
            select(mhz * 1000000, selector, 'VHF', (mhz - 130) // 3)

    cells = []
    for group, low_hz, high_hz, origin_mhz, step_mhz in (
            ('UHF', 400000000, 470000000, 400, 10),
            ('VHF', 136000000, 174000000, 130, 3),
            ('Low_band', 26000000, 27999990, 15, 5)):
        offsets = next(offsets for name, _, offsets in GROUPS if name == group)
        first = (low_hz // 1000000 - origin_mhz) // step_mhz
        last = (high_hz // 1000000 - origin_mhz) // step_mhz
        for index in range(first, last + 1):
            start = max(low_hz, (origin_mhz + index * step_mhz) * 1000000)
            end = min(high_hz, (origin_mhz + (index + 1) * step_mhz) * 1000000 - 10)
            for selector in (2, 1, 0):
                address = offsets[selector] + index
                cells.append({'group': group, 'first_hz_inclusive': start, 'last_hz_inclusive': end,
                              'index': index, 'level': LEVELS[selector], 'offset': f'0x{address:06X}',
                              'code_decimal': dump[address]})
    points = []
    requests = [('P1', hz) for hz in (433075000, 434750000, 446006250, 446081250, 446093750)]
    requests += [('P2', mhz * 1000000) for mhz in (137, 140, 143, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173)]
    requests += [('P2', hz) for hz in (400025000, 410025000, 420025000, 430025000, 440025000, 450025000, 460025000, 470000000)]
    requests += [('P3', hz) for hz in (26000000, 27185000, 27995000)]
    for priority, hz in requests:
        group, index = ('UHF', (hz // 1000000 - 400) // 10) if hz >= 400000000 else (
            ('VHF', (hz // 1000000 - 130) // 3) if hz >= 136000000 else
            ('Low_band', (hz // 1000000 - 15) // 5))
        offsets = next(offsets for name, _, offsets in GROUPS if name == group)
        for selector in (2, 1, 0):
            address = offsets[selector] + index
            assert select(hz, selector, group, index) == dump[address]
            points.append({'priority': priority, 'frequency_hz': hz, 'mode': 'FM', 'bandwidth': 'Narrow',
                           'level': LEVELS[selector], 'index': index, 'offset': f'0x{address:06X}',
                           'owner_backup_code': dump[address], 'measured_watts': '', 'battery_rest_v': '',
                           'battery_tx_v': '', 'firmware': '', 'meter_load_attenuator': '',
                           'radio_source': '', 'source_dump_sha256': '', 'notes': ''})

    output.mkdir(parents=True, exist_ok=True)
    report = {'stock_sha256': sha(stock), 'dump_sha256': sha(dump), 'dump_bytes': len(dump),
              'calibration_sector_sha256': sha(dump[0xf000:0x10000]),
              'loader_reads': reads, 'power_tables': table_rows, 'selector_checks': checks,
              'battery_raw': list(dump[0xf200:0xf207]),
              'mic_raw': dump[0xf210], 'ctcss_raw': dump[0xf211], 'dcs_raw': dump[0xf212],
              'frequency_marker': dump[0xf21b], 'frequency_correction': struct.unpack_from('<h', dump, 0xf21c)[0]}
    (output / 'layout.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    for name, rows in (('power-codes.csv', table_rows), ('frequency-cells.csv', cells),
                       ('measurements-template.csv', points)):
        with (output / name).open('w', encoding='utf-8-sig', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    lines = ['# Фактические чтения загрузчика калибровки 0.29', '',
             f'Stock SHA-256: `{sha(stock)}`', f'Dump SHA-256: `{sha(dump)}`', '',
             '| SPI-адрес | Размер | RAM | Байты |', '|---|---:|---|---|']
    lines += [f"| 0x{item['offset']:06X} | {item['length']} | 0x{item['ram']:08X} | `{item['hex']}` |" for item in reads]
    lines += ['', '## Таблицы мощности: фактические коды, не ватты', '',
              '| Группа | Уровень | Начало | 16 десятичных кодов |', '|---|---|---|---|']
    for group, _, offsets in GROUPS:
        for level, offset in zip(LEVELS, offsets):
            lines.append(f'| {group} | {level} | 0x{offset:06X} | ' + ', '.join(map(str, dump[offset:offset + 16])) + ' |')
    lines += ['', f'PASS: {checks} выполнений OEM-селектора; проверены адрес выбранного байта и значение.',
              'Проверка в ARM не измеряет мощность и не подтверждает аппаратную работу дополнительных диапазонов.']
    (output / 'LAYOUT-RU.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    assert sha(args.stock.read_bytes()) == sha(stock) and sha(args.dump.read_bytes()) == sha(dump)
    print(f'PASS read-only: {len(reads)} loader reads, {len(table_rows)} PA bytes, {checks} selector checks')
    print(output)


if __name__ == '__main__':
    main()
