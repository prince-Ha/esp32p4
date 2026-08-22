#!/usr/bin/env python3
"""Regenerate the bold display faces: korean_24_bold and digits_64.

The 14/16 px body faces carry the whole Hangul syllable block because they
render user-visible data of unknown content. These display faces only ever
render string literals that live in this repository (and, for digits_64,
nothing but numerals), so they are built from exactly the characters those
literals use. A full-range 24 px face costs about 1.6 MB of flash; this one
costs a few tens of kilobytes.

Both are cut from Malgun Gothic Bold. Noto Sans KR ships here in Regular only,
and the variable NotoSansKR-VF.ttf gives lv_font_conv no way to select a weight
instance, so a real bold has to come from a separate file.

Run it after adding or changing any Korean string shown at heading size, then
rebuild:

    python tools/make_heading_font.py && pio run
"""

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SOURCE = ROOT / "src" / "main.cpp"
OUTPUT = ROOT / "src" / "korean_24_bold.c"
DIGITS_OUTPUT = ROOT / "src" / "digits_64.c"
TTF = Path("C:/Windows/Fonts/malgunbd.ttf")

# Hangul syllables, plus compatibility Jamo for standalone consonants/vowels.
HANGUL = re.compile(r"[가-힣㄰-㆏]")


def collect_hangul(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    return "".join(sorted(set(HANGUL.findall(text))))


def run_conv(args: list) -> int:
    exe = "lv_font_conv.cmd" if sys.platform == "win32" else "lv_font_conv"
    # bpp 4 matches LVGL's own Montserrat faces. At bpp 2 the Hangul rendered
    # visibly rougher than the Latin next to it.
    cmd = [exe, "--bpp", "4", "--no-compress", "--font", str(TTF),
           "--format", "lvgl", "--lv-include", "lvgl.h"] + args
    return subprocess.run(cmd, shell=(sys.platform == "win32")).returncode


def main() -> int:
    if not TTF.exists():
        print(f"error: {TTF} not found", file=sys.stderr)
        return 1

    hangul = collect_hangul(SOURCE)
    if not hangul:
        print("error: no Hangul found in main.cpp", file=sys.stderr)
        return 1

    print(f"{len(hangul)} distinct Hangul syllables found in {SOURCE.name}")

    rc = run_conv([
        "--size", "24",
        # Latin, punctuation, general punctuation, arrows, geometric shapes.
        "--range", "0x20-0x7E",
        "--range", "0x00A0-0x00FF",
        "--range", "0x2000-0x206F",
        "--range", "0x2190-0x21FF",
        "--range", "0x25A0-0x25FF",
        "--symbols", hangul,
        "--lv-font-name", "korean_24_bold",
        "-o", str(OUTPUT),
    ])
    if rc != 0:
        print("error: lv_font_conv failed for korean_24_bold", file=sys.stderr)
        return rc

    rc = run_conv([
        "--size", "64",
        "--symbols", "0123456789.:-+ ",
        "--lv-font-name", "digits_64",
        "-o", str(DIGITS_OUTPUT),
    ])
    if rc != 0:
        print("error: lv_font_conv failed for digits_64", file=sys.stderr)
        return rc

    for path in (OUTPUT, DIGITS_OUTPUT):
        print(f"wrote {path.relative_to(ROOT)} ({path.stat().st_size / 1024:.0f} kB of C source)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
