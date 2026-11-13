"""Build versioned firmware artifacts into one flat directory. No device I/O."""
import argparse
import hashlib
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile

DOCUMENTS = ('README.md', 'LICENSE')
ARTIFACT_SUFFIXES = ('.BTF', '.bin', '.elf', '.map', '.IMAGE-CHECK.txt',
                     '.SHA256SUMS.txt', '.SOURCE.zip', '.zip')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--toolchain', required=True, type=Path)
    p.add_argument('--output', type=Path, default=Path('firmware'))
    p.add_argument('--refresh', action='store_true', help='replace artifacts of this version only')
    p.add_argument('--image-only', action='store_true')
    p.add_argument('--logic-log', type=Path)
    p.add_argument('--board-log', type=Path)
    p.add_argument('--lcd-log', type=Path)
    p.add_argument('--clean-rebuild', action='store_true')
    a = p.parse_args()
    custom = Path(__file__).resolve().parent
    project = custom.parent.parent
    output = a.output.resolve()
    if re.match(r'acceptance', output.name, re.I):
        p.error('use the shared firmware directory, without a per-version acceptance folder')
    match = re.search(r'^#define RT950_CUSTOM_VERSION "([0-9]+\.[0-9]+(?:\.[0-9]+)?)"\s*$',
                      (custom / 'firmware_version.hpp').read_text(encoding='utf-8'), re.MULTILINE)
    if not match or len(match[1]) > 15:
        p.error('firmware version must be X.Y or X.Y.Z and fit in 15 characters')
    version = match[1]
    name = 'RT950_Custom_UI_v' + version
    if not a.refresh and any((output / (name + suffix)).exists() for suffix in ARTIFACT_SUFFIXES):
        p.error('this version exists; --refresh replaces only its own files')
    if not a.image_only and not (a.logic_log and a.board_log and a.lcd_log):
        p.error('a complete kit requires --logic-log, --board-log and --lcd-log')
    prefix = str(a.toolchain.resolve() / 'arm-none-eabi-')

    def run(*cmd):
        r = subprocess.run([str(x) for x in cmd], check=True, capture_output=True, text=True)
        return r.stdout + r.stderr

    build_log = run(sys.executable, custom / 'build_windows.py', '--toolchain', a.toolchain, '--elf-only')
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='rt950-firmware-') as directory:
        kit = Path(directory) / 'kit'
        kit.mkdir()
        elf = kit / (name + '.elf')
        raw = kit / (name + '.bin')
        btf = kit / (name + '.BTF')
        shutil.copyfile(custom / 'build/rt950-custom-ui.elf', elf)
        shutil.copyfile(custom / 'build/rt950-custom-ui.map', kit / (name + '.map'))
        run(prefix + 'objcopy', '-O', 'binary', elf, raw)
        run(sys.executable, custom / 'pad_image.py', elf, raw, prefix + 'nm', '--code-after-storage')
        run(sys.executable, custom.parent / 'tools/encrypt_btf.py', raw, btf, '--key', '0' * 32)
        check = run(sys.executable, custom / 'check_image.py', '--prefix', prefix,
                    '--elf', elf, '--raw', raw, '--btf', btf)
        (kit / (name + '.IMAGE-CHECK.txt')).write_text(check, encoding='utf-8')
        (kit / 'BUILD.txt').write_text(build_log, encoding='utf-8')
        digest = hashlib.sha256(btf.read_bytes()).hexdigest()
        if not a.image_only:
            # Tests must have run against the image being packaged, not a stale build.
            tested = output / btf.name
            if not tested.is_file() or tested.read_bytes() != btf.read_bytes():
                p.error('build --image-only, test that image, then package the unchanged image with --refresh')
            for source, destination, required in (
                (a.logic_log, 'LOGIC-ARM.txt', 'ARM monitor settings:'),
                (a.board_log, 'BOARD-ARM.txt', 'ARM power units UI:'),
                (a.lcd_log, 'LCD-ARM.txt', 'ARM LCD OEM 0.29:')):
                text = source.read_text(encoding='utf-8-sig')
                if required not in text or 'Traceback' in text:
                    p.error(f'incomplete or failed test log: {source}')
                if destination=='LOGIC-ARM.txt' and not all(marker in text for marker in ('ARM monitor audio:', 'ARM BLE GPS:', 'ARM auto lock:', 'ARM broadcast banks:', 'ARM about page:', 'ARM tuner steps:', 'ARM GPS DMS:', 'ARM GPS point:', 'ARM GPS geometry:', 'ARM compact storage:', 'ARM reset defaults:', 'ARM settings export:', 'HOST settings backup:', 'ARM settings import:', 'HOST settings upload:', 'ARM user keys:', 'HOST portable migration:')):
                    p.error('logic/board-audio/protocol suite has not completed')
                if destination=='BOARD-ARM.txt' and not all(marker in text for marker in ('ARM auto lock UI:', 'ARM broadcast banks UI:', 'ARM about UI:', 'ARM tuner steps UI:', 'ARM GPS DMS UI:', 'ARM GPS DMS position UI:', 'ARM GPS navigation UI:', 'ARM storage format UI:', 'ARM compact migration:', 'ARM backup UART:', 'ARM backup UI:', 'ARM reset UI:', 'ARM import UART:', 'ARM import UI:', 'ARM user keys UI:', 'ARM user keys autosave:')):
                    p.error('auto-lock bilingual LCD checks have not completed')
                if destination=='LCD-ARM.txt' and not all(marker in text for marker in ('ARM LCD bus:', 'ARM LCD power:', 'ARM LCD window:', 'ARM LCD: UTF-8')):
                    p.error('LCD bus/power/oracle suite has not completed')
                (kit / destination).write_text(text, encoding='utf-8')
            for document in DOCUMENTS:
                shutil.copyfile(project / document, kit / document)
            source_zip = kit / (name + '.SOURCE.zip')
            with zipfile.ZipFile(source_zip, 'w', zipfile.ZIP_DEFLATED) as archive:
                excluded = {'build', 'build-bench', '__pycache__', '.git'}
                for path in sorted((project / 'source').rglob('*')):
                    relative = path.relative_to(project)
                    if path.is_file() and not excluded.intersection(relative.parts):
                        archive.write(path, relative.as_posix())
                for document in DOCUMENTS:
                    archive.write(project / document, document)
            if a.clean_rebuild:
                clean = Path(directory) / 'clean'
                with zipfile.ZipFile(source_zip) as archive:
                    archive.extractall(clean)
                clean_log = run(sys.executable, clean / 'source/custom/package_firmware.py',
                                '--toolchain', a.toolchain, '--output', clean / 'out', '--image-only')
                for suffix in ('.BTF', '.bin'):
                    if (clean / 'out' / (name + suffix)).read_bytes() != (kit / (name + suffix)).read_bytes():
                        raise RuntimeError('clean source rebuild differs: ' + suffix)
                (kit / 'CLEAN-REBUILD.txt').write_text(clean_log + '\nPASS clean SOURCE rebuild: BTF and RAW byte-identical\n', encoding='utf-8')
            manifest = ''.join(f'{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}\n'
                               for path in sorted(kit.iterdir()) if path.is_file())
            (kit / 'MANIFEST-SHA256.txt').write_text(manifest, encoding='utf-8')
            with zipfile.ZipFile(kit / (name + '.zip'), 'w', zipfile.ZIP_DEFLATED) as archive:
                for path in sorted(kit.iterdir()):
                    if path.is_file() and path.suffix != '.zip':
                        archive.write(path, path.name)
                archive.write(source_zip, source_zip.name)
        # Only prefixed artifacts are published; reports/docs live in the versioned ZIP.
        published = [path for path in kit.iterdir() if path.name.startswith(name + '.')]
        for path in published:
            shutil.copyfile(path, output / path.name)
        sums = ''.join(f'{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}\n'
                       for path in sorted(published))
        (output / (name + '.SHA256SUMS.txt')).write_text(sums, encoding='ascii')
    print(output / (name + '.BTF'))
    print('BTF SHA256 ' + digest)
    print('PASS flat firmware package')


if __name__ == '__main__':
    main()
