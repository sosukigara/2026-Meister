#!/usr/bin/env python3
"""リポジトリのテキストを機械検査する。

検査できるもの:
  - 日本語（かな/漢字/全角）の直後に英単語 3 文字以上が続く綴り
  - ハングル
  - 置換文字 U+FFFD
  - tools/corruption_blocklist.txt に挙げた CJK 混在トークン
  - ヘッダの波括弧の平衡 / 名前空間の開閉 / 同一 namespace 内の識別子の重複
  - 生成ヘッダと config/meister_robot.yaml の不整合（--config）

検査**できない**もの:
  - 1 漢字だけの混入（例: 運動学 の 学）。漢字は日本語と簡体字で共有して
    いるため、この検査では原理的に検出できない。差分の目視が必須。
  - blocklist に無い CJK 混入語。blocklist は実際の混入事例から
    増やすもの。英語単語や日本語の通常語を入れると誤検出し、
    residual=0 が「clean」の意味を失う。

使い方:
    python3 tools/check_text.py
    python3 tools/check_text.py --config     # 生成物と YAML の整合も見る
"""
from __future__ import annotations

import argparse
import collections
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent

# glob で拾う。固定リストにするとファイル追加時に検査対象から外れる
# （kinematics.cpp が 4 件の混入を素通りした）。
CODE_ROOTS = ['firmware'] + sorted(
    str(p.parent.relative_to(ROOT)) for p in (ROOT / 'src').glob('*')
    if (p / 'CMakeLists.txt').is_file())


def _src(pattern: str) -> list[str]:
    return sorted(str(p.relative_to(ROOT)) for root in CODE_ROOTS
                  for p in (ROOT / root).rglob(pattern)
                  if not {'.pio', 'build'} & set(p.parts))


HEADERS = _src('include/**/*.h') + _src('include/**/*.hpp')
SOURCES = _src('src/**/*.cpp')
TESTS = _src('test/**/*.cpp') + _src('test/**/*.hpp')
DOCS = ['AGENTS.md', 'README.md', 'docs/README.md', 'docs/code-map.md',
        'firmware/README.md', 'config/meister_robot.yaml']
# docs/ 配下の仕様書は文字化けが最も繰り返されてきた場所なので glob で全部見る
DOCS += sorted(p.relative_to(ROOT) for d in
               ('docs/design', 'docs/features', 'docs/functions',
                'docs/superpowers/specs')
               for p in (ROOT / d).glob('*.md'))
TOOLS = sorted(p.relative_to(ROOT) for p in (ROOT / 'tools').glob('*.py'))

MIXED = re.compile(r'[ぁ-ゖァ-ヺ一-鿿][A-Za-z]{3,}')


def scan_text(paths) -> int:
    block = ROOT / 'tools' / 'corruption_blocklist.txt'
    tokens: list[str] = []
    if block.is_file():
        tokens = [tok for line in block.read_text().splitlines()
                  if line.strip() and not line.startswith('#')
                  for tok in line.split()]
    bad = 0
    for path in paths:
        if not path.is_file():
            continue
        for i, line in enumerate(path.read_text(errors='replace').splitlines(), 1):
            rel = path.relative_to(ROOT)
            for m in MIXED.finditer(line):
                print(f'  NG   {rel}:{i} 連結 {m.group()!r}')
                bad += 1
            for ch in line:
                if 0xAC00 <= ord(ch) <= 0xD7AF:
                    print(f'  NG   {rel}:{i} U+{ord(ch):04X} ハングル')
                    bad += 1
                elif ch == '\ufffd':
                    print(f'  NG   {rel}:{i} U+FFFD 置換文字')
                    bad += 1
            for tok in tokens:
                # ASCII だけのトークンは単語境界で照合する。部分一致で探すと
                # -blocklist- の英単語が正当な識別子の部分文字列になって
                # 全ファイルが赤字になる（実際にそうなった）。
                if re.fullmatch(r'[A-Za-z0-9_]+', tok):
                    hit = re.search(rf'\b{re.escape(tok)}\b', line)
                else:
                    hit = tok if tok in line else None
                if hit:
                    print(f'  NG   {rel}:{i} blocklist {tok!r}')
                    bad += 1
    print(f"  {'OK' if bad == 0 else 'NG'}   文字化け residual={bad}")
    return bad


def scan_structure() -> int:
    bad = 0
    for rel in HEADERS:
        p = ROOT / rel
        if not p.is_file():
            continue
        text = p.read_text()
        if text.count('{') != text.count('}'):
            print(f'  NG   {rel} 波括弧が不一致')
            bad += 1
        opened = len(re.findall(r'namespace [a-z0-9_]* \{', text))
        closed = len(re.findall(r'\}  // namespace', text))
        if opened != closed:
            print(f'  NG   {rel} 名前空間 開{opened} 閉{closed}')
            bad += 1
    for rel in HEADERS:
        p = ROOT / rel
        if not p.is_file():
            continue
        defs: collections.Counter = collections.Counter()
        for line in p.read_text().splitlines():
            m = re.match(r'\s*(?:constexpr|inline|static\s+constexpr)[^;]*?'
                         r'\b(k[A-Z]\w*)\b\s*=', line)
            if m:
                defs[m.group(1)] += 1
        dup = {k: v for k, v in defs.items() if v > 1}
        if dup:
            print(f'  NG   {rel} 識別子の重複 {dup}')
            bad += 1
    print(f"  {'OK' if bad == 0 else 'NG'}   構造")
    return bad


def check_config() -> int:
    import subprocess
    r = subprocess.run([sys.executable, str(ROOT / 'tools' / 'gen_config.py'), '--check'],
                       capture_output=True, text=True, cwd=ROOT)
    sys.stdout.write(r.stdout)
    if r.returncode != 0:
        sys.stderr.write(r.stderr)
    return r.returncode


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', action='store_true',
                    help='生成物と YAML の整合も見る')
    opts = ap.parse_args()

    print('== 1) 構造（波括弧 / 名前空間 / 識別子の重複）==')
    rc = scan_structure()
    print('== 2) 文字化け ==')
    rc += scan_text(
        [ROOT / h for h in HEADERS]
        + [ROOT / s for s in SOURCES]
        + [ROOT / t for t in TESTS]
        + [ROOT / d for d in DOCS + TOOLS]
    )
    if opts.config:
        print('== 3) 生成物と YAML の整合 ==')
        rc += check_config()

    print(f"== 総合: {'PASS' if rc == 0 else 'FAIL'} ==")
    return 1 if rc else 0


if __name__ == '__main__':
    sys.exit(main())
