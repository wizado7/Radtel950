"""Check packaging and symbol dependencies; not physical validation."""
from pathlib import Path
import sys,struct,hashlib,argparse,subprocess,tempfile
root=Path(__file__).resolve().parent
sys.path.insert(0,str(root.parent/'tools'))
from encrypt_btf import extract_firmware,expand_key,extract_key
p=argparse.ArgumentParser()
p.add_argument('--prefix',default='arm-none-eabi-')
mode=p.add_mutually_exclusive_group()
mode.add_argument('--bench',action='store_true')
mode.add_argument('--rx-only',action='store_true')
p.add_argument('--elf',type=Path)
p.add_argument('--raw',type=Path)
p.add_argument('--btf',type=Path)
args=p.parse_args()
explicit=(args.elf,args.raw,args.btf)
if any(explicit) and not all(explicit):
    p.error('--elf, --raw and --btf must be supplied together')
build=root/('build-bench' if args.bench else 'build')
name='RT950_Bench_TX.BTF' if args.bench else ('RT950_Custom_RX_Only.BTF' if args.rx_only else 'RT950_Custom_UI_v0.44.BTF')
raw=(args.raw or build/'rt950-custom-ui.bin').read_bytes()
btf=(args.btf or build/name).read_bytes()
sp,pc=struct.unpack_from('<II',raw)
assert 0x20000000<sp<=0x20018000 and sp%8==0
assert pc&1 and 0x08003000<=pc&~1<0x08003000+len(raw)
assert len(raw)<=244*1024 and len(btf)==len(raw)+1024 and len(btf)%1024==0
for off in (0x3e0,0x7e0):assert raw[off:off+12]==b'RT-950      '
decoded,_=extract_firmware(bytearray(btf),expand_key(extract_key(btf)))
assert decoded==raw
elf=str(args.elf or build/'rt950-custom-ui.elf')
assert not subprocess.check_output([args.prefix+'nm','-u',elf],text=True).strip()
symbols=subprocess.check_output([args.prefix+'nm',elf],text=True)
layout={name:int(value,16) for value,_,name in
        (line.split() for line in symbols.splitlines()) if name.startswith('_storage_')}
start=layout['_storage_start']-0x08003000
end=layout['_storage_end']-0x08003000
assert start==0xd000 and end==0x11000 and end-start==2*layout['_storage_slot_size']
assert pc&~1>=0x08003000+end and end<len(raw)
assert ('BoardUhfTx' in symbols)==(not args.rx_only)
sections=subprocess.check_output([args.prefix+'objdump','-h',elf],text=True)
for row in sections.splitlines():
    fields=row.split()
    if len(fields)<7 or not fields[0].isdigit():continue
    length,lma=int(fields[2],16),int(fields[4],16)
    if 0x08000000<=lma<0x08100000 and length:
        assert lma>=0x08003000 and lma+length<=0x08040000
        assert lma+length<=0x08010000 or lma>=0x08014000
assert raw[start:end]==b'\xff'*(end-start)
with tempfile.TemporaryDirectory(prefix='rt950-elf-check-') as folder:
    regenerated=Path(folder)/'app.bin'
    subprocess.run([args.prefix+'objcopy','-O','binary',elf,regenerated],check=True)
    subprocess.run([sys.executable,root/'pad_image.py',elf,regenerated,args.prefix+'nm',
                    '--code-after-storage'],check=True)
    assert raw==regenerated.read_bytes(),'ELF and raw image differ'

assert any(0x20000000<=int(row.split()[0],16)<0x20018000
           for row in symbols.splitlines() if 'flash_operation' in row)
print('PASS two erased reserved slots and RAM-only flash operation routine')
for forbidden in ('_GLOBAL__sub_I','__cxa','malloc','_Znwj','bk4829','flash_write','flash_erase'):
    assert forbidden not in symbols,forbidden
print(f'PASS ELF/raw match, BTF round trip, vectors, size, signatures, no unresolved symbols or forbidden dependencies')
print(f'raw={len(raw)}, BTF={len(btf)}, SP={sp:#x}, PC={pc:#x}')
print('SHA256',hashlib.sha256(btf).hexdigest())

