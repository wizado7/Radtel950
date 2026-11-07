"""Pad the raw application to cover both reserved, erased storage slots."""
from pathlib import Path
import subprocess
import sys

elf, raw = map(Path, sys.argv[1:3])
symbols = {}
for line in subprocess.check_output([sys.argv[3], '-n', str(elf)], text=True).splitlines():
    columns = line.split()
    if len(columns) == 3 and columns[2].startswith('_storage_'):
        symbols[columns[2]] = int(columns[0], 16)
base = 0x08003000
start = symbols['_storage_start'] - base
end = symbols['_storage_end'] - base
slot_size = symbols['_storage_slot_size']
data = raw.read_bytes()
if '--code-after-storage' in sys.argv[4:]:
    if not (0 < start < end < len(data) <= 244*1024 and end-start == 2*slot_size):
        raise SystemExit('invalid bench image layout')
    # objcopy fills gaps between loadable sections with zeroes.
    if any(data[start:end]):
        raise SystemExit('data present in storage reservation')
    padded=data[:start]+b'\xff'*(end-start)+data[end:]
    raw.write_bytes(padded+b'\xff'*(-len(padded)%1024))
    sys.exit(0)
if not (0 < len(data) <= start < end <= 244*1024 and end-start == 2*slot_size and end%1024 == 0):
    raise SystemExit('invalid storage reservation or application overlap')
raw.write_bytes(data + b'\xff'*(end-len(data)))
