# SPDX-License-Identifier: GPL-3.0-or-later
"""Execute the OEM channel iterator and check the local CSV decoder."""
import argparse
import hashlib
from pathlib import Path
import struct

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_CPU_ARM_CORTEX_M4, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_SP

from oem_channels import CHANNELS, CHANNEL_BYTES, PER_ZONE, RECORD, records


def indices(stock, zone, cursor, present):
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
    cpu.mem_map(0x08000000, 0x100000)
    cpu.mem_map(0x20000000, 0x18000)
    cpu.mem_write(0x08003000, stock)
    cpu.mem_write(0x2000A91C + 6, bytes([zone is None]))
    if zone is not None:
        cpu.mem_write(0x2000A91C + 13, bytes([zone]))
    requested = []

    def occupied(machine, address, size, data):
        index = machine.reg_read(UC_ARM_REG_R0)
        requested.append(index)
        assert machine.reg_read(UC_ARM_REG_R1) == 0
        machine.reg_write(UC_ARM_REG_R0, int(index in present))
        machine.reg_write(UC_ARM_REG_PC, machine.reg_read(UC_ARM_REG_LR))

    cpu.hook_add(UC_HOOK_CODE, occupied, begin=0x0800BCEC, end=0x0800BCEC)
    cpu.reg_write(UC_ARM_REG_SP, 0x20017BB0)
    cpu.reg_write(UC_ARM_REG_LR, 0x080FF001)
    cpu.reg_write(UC_ARM_REG_R0, cursor)
    cpu.reg_write(UC_ARM_REG_R1, 0)
    cpu.emu_start(0x08022BDD, 0x080FF000, count=50000)
    assert cpu.reg_read(UC_ARM_REG_PC)==0x080FF000
    return cpu.reg_read(UC_ARM_REG_R0), requested


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stock', type=Path, required=True)
    parser.add_argument('--dump', type=Path)
    args = parser.parse_args()
    stock = args.stock.read_bytes()
    assert hashlib.sha256(stock).hexdigest() == '025bc583802a96911836e274cdfdf3ce05d9919bc77451b7f7f1573fb5571907'
    assert indices(stock, None, CHANNELS - 2, {CHANNELS - 1}) == (CHANNELS - 1, [CHANNELS - 1])
    assert indices(stock, None, CHANNELS - 1, set()) == (65535, list(range(CHANNELS)))
    for zone in (0, 1, 9):
        assert indices(stock, zone, PER_ZONE - 1, set()) == (
            65535, list(range(zone * PER_ZONE, (zone + 1) * PER_ZONE)))
    image = bytearray(b'\xff' * CHANNEL_BYTES)
    image[-RECORD:-RECORD + 4] = struct.pack('<I', 0x44608125)
    assert list(records(image))[0][:5] == (990, 10, 99, 446081250, 0)
    image[-RECORD] = 0xFA
    try:
        list(records(image))
    except ValueError as error:
        assert 'channel 990' in str(error)
    else:
        raise AssertionError('malformed BCD was accepted')
    if args.dump:
        rows = list(records(args.dump.read_bytes()))
        assert rows and all(row[3] > 0 for row in rows)
        print(f'OEM external dump: decoded {len(rows)} occupied channel records')
    print('OEM 0.29 channel iterator: 990 flat records, 99 per zone, wrap and empty scan PASS')
    print('OEM CSV decoder: final channel, absent TX, malformed BCD PASS')


if __name__ == '__main__':
    main()
