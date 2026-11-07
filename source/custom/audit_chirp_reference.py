"""Offline inspection of the pinned NathanBarguss CHIRP reference checkout.

Loads only the three reviewed, hardware-independent codec modules. Does not
import package __init__, transport, CHIRP, pythonnet, or open any serial port.
The upstream source remains unmodified; generated cases live in RAM.
"""
import argparse
import ast
import importlib.util
import json
import subprocess
import sys
import types
from pathlib import Path

PIN = '254d2cd52e152a83cdc2f008e26afd7ed03b2798'


def inspect(repo: Path) -> dict:
    commit = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
    if commit != PIN:
        raise ValueError('reference revision changed; review the codec modules before executing them')
    reviewed = ['rt950pro/' + name + '.py' for name in ('logging', 'channel', 'sections', 'transport')]
    subprocess.run(['git', '-C', str(repo), 'diff', '--exit-code', 'HEAD', '--', *reviewed], check=True)
    prefix = '_rt950_chirp_audit'
    package = types.ModuleType(prefix)
    package.__path__ = [str(repo / 'rt950pro')]
    sys.modules[prefix] = package
    sys.dont_write_bytecode = True
    for name in ('logging', 'channel', 'sections'):
        qualified = prefix + '.' + name
        spec = importlib.util.spec_from_file_location(qualified, repo / 'rt950pro' / (name + '.py'))
        module = importlib.util.module_from_spec(spec)
        sys.modules[qualified] = module
        spec.loader.exec_module(module)
    channel = sys.modules[prefix + '.channel']
    sections = sys.modules[prefix + '.sections']
    raw = bytearray(96)
    for i, power in enumerate((0, 1, 2)):
        start = 32 * i
        raw[start:start + 8] = bytes(map(int, '44608125'))
        raw[start + 16] = power
        raw[start + 17] = 0x40  # Narrow, FM.
    entries = sections.parse_vfo_section(bytes(raw))
    decoded = [entry.tx_power.name for entry in entries]
    untouched = sections.encode_vfo_section(entries, bytes(raw))
    if untouched != raw:
        raise AssertionError('unexpected unchanged round-trip result')
    for entry in entries:
        entry.bandwidth = channel.Bandwidth.WIDE
    edited = sections.encode_vfo_section(entries, bytes(raw))
    after = [edited[32 * i + 16] & 15 for i in range(3)]
    # Parse segment declarations as syntax, without importing serial transport.
    tree = ast.parse((repo / 'rt950pro/transport.py').read_text(encoding='utf-8-sig'))
    segments = []
    for node in ast.walk(tree):
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and node.target.id == 'DEFAULT_SEGMENTS':
            for call in node.value.elts:
                segments.append([ast.literal_eval(arg) for arg in call.args])
    return {
        'repository': 'https://github.com/NathanBarguss/Chirp_Radtel-RT-950-Pro',
        'commit': commit,
        'hardware_access': False,
        'enum': {member.name: member.value for member in channel.PowerLevel},
        'vfo_input_power_codes': [0, 1, 2],
        'vfo_decoded_names': decoded,
        'unchanged_vfo_roundtrip_preserved': True,
        'vfo_power_codes_after_only_bandwidth_edit': after,
        'vfo_power_bug_reproduced': decoded != ['HIGH', 'MEDIUM', 'LOW'] or after != [0, 1, 2],
        'clone_segments_read_write_start_length': segments,
        'clone_bytes': sum(segment[3] for segment in segments),
        'missing_private_fixtures': [name for name in ('dumps/clean.bin', 'Reference') if not (repo / name).exists()],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.resolve().is_relative_to(args.reference.resolve()):
        parser.error('write the report outside the unmodified reference checkout')
    report = inspect(args.reference.resolve())
    encoded = json.dumps(report, indent=2, ensure_ascii=True) + '\n'
    args.output.write_text(encoded, encoding='utf-8')
    print(encoded, end='')


if __name__ == '__main__':
    main()
