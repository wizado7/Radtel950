"""Reproducible native Windows build; no make or shell emulation required."""
from pathlib import Path
import argparse, subprocess, sys
p=argparse.ArgumentParser(); p.add_argument('--toolchain',required=True,type=Path); p.add_argument('--bench-tx-hz',type=int); p.add_argument('--bench-matrix',action='store_true'); p.add_argument('--rx-only',action='store_true'); p.add_argument('--elf-only',action='store_true'); a=p.parse_args()
r=Path(__file__).resolve().parent
if a.bench_matrix and a.bench_tx_hz!=446081250:
 p.error('--bench-matrix requires --bench-tx-hz 446081250')
bench=a.bench_tx_hz is not None
if bench and a.rx_only:
 p.error('--rx-only cannot be combined with --bench-tx-hz')
tx_enabled=not a.rx_only
tx_hz=a.bench_tx_hz if bench else 446081250
if bench and (not 400000000<=a.bench_tx_hz<=470000000 or a.bench_tx_hz%10):
 p.error('bench frequency must be UHF in 10 Hz units')
out=r/('build-bench' if bench else 'build'); out.mkdir(exist_ok=True)
def run(args): subprocess.run([str(x) for x in args],check=True,cwd=r)
def tool(name): return a.toolchain/('arm-none-eabi-'+name+'.exe')
flags=['-mcpu=cortex-m4','-mthumb','-mfloat-abi=hard','-mfpu=fpv4-sp-d16','-Os','-g','-ffreestanding','-fno-builtin','-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror','-DRT950_CUSTOM','-I../include']
if tx_enabled: flags+=[f'-DRT950_BENCH_TX_HZ={tx_hz}']
if a.bench_matrix or (tx_enabled and not bench): flags+=['-DRT950_BENCH_MATRIX']
if tx_enabled and not bench: flags+=['-DRT950_MAIN_TX']
c=['../src/startup.c','../src/system.c','../src/drivers/gpio.c','../src/drivers/lcd.c','../src/app/encoder.c','../diagnostic/nmea_status.c','../mvp/core.c','board.c','battery.c','runtime.c']
cpp=['application.cpp','controls.cpp','main.cpp','ui.cpp','gps_view.cpp','gps_navigation.cpp','locale.cpp','radio_settings.cpp','settings_snapshot.cpp','settings_flash.cpp','settings_export_uart.cpp','fm_board.cpp','voice_rx.cpp','voice_factory_board.cpp','aprs_dac.cpp','display_power.cpp']; objs=[]
for src in c+cpp:
 obj=out/(Path(src).stem+'.o'); objs.append(obj)
 opts=['-std=c++17','-fno-exceptions','-fno-rtti','-fno-threadsafe-statics','-Wshadow'] if src in cpp else ['-std=c11']
 if src in cpp and src!='settings_flash.cpp': opts.append('-Oz')
 run([tool('g++' if src in cpp else 'gcc'),*flags,*opts,'-c',src,'-o',obj])
elf=out/'rt950-custom-ui.elf'; raw=out/'rt950-custom-ui.bin'
run([tool('g++'),*flags,'-nostdlib','-T',('linker_bench.ld'),'-Wl,--gc-sections,-Map='+str(out/'rt950-custom-ui.map'),*objs,'-o',elf])
if a.elf_only:
 run([tool('size'),elf]); sys.exit(0)
run([tool('objcopy'),'-O','binary',elf,raw]); run([sys.executable,'pad_image.py',elf,raw,tool('nm'),'--code-after-storage'])
run([sys.executable,'../tools/encrypt_btf.py',raw,out/('RT950_Bench_TX.BTF' if bench else ('RT950_Custom_RX_Only.BTF' if a.rx_only else 'RT950_Custom_UI_v0.44.BTF')),'--key','0'*32]); run([tool('size'),elf])
