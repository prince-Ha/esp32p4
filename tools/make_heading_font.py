#!/usr/bin/env python3
"""Regenerate src/korean_24.c, the 24 px heading face.

The 12/14/16 px faces carry the whole Hangul syllable block because they render
user-visible data of unknown content. The 24 px face only ever renders string
literals that live in this repository, so it is built from exactly the Hangul
those literals use. A full-range 24 px face costs about 1.6 MB of flash; this
one costs a few tens of kilobytes.

Run it after adding or changing any Korean string that is displayed at heading
size, then rebuild:

    python tools/make_heading_font.py && pio run
"""

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SOURCE = ROOT / "src" / "main.cpp"
OUTPUT = ROOT / "src" / "korean_24.c"
TTF = ROOT / "NotoSansKR-Regular.ttf"

# Hangul syllables, plus compatibility Jamo for standalone consonants/vowels.
HANGUL = re.compile(r"[가-힣㄰-㆏]")


def collect_hangul(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    return "".join(sorted(set(HANGUL.findall(text))))


def main() -> int:
    if not TTF.exists():
        print(f"error: {TTF.name} not found next to platformio.ini", file=sys.stderr)
        return 1

    hangul = collect_hangul(SOURCE)
    if not hangul:
        print("error: no Hangul found in main.cpp", file=sys.stderr)
        return 1

    print(f"{len(hangul)} distinct Hangul syllables found in {SOURCE.name}")

    cmd = [
        "lv_font_conv.cmd" if sys.platform == "win32" else "lv_font_conv",
        "--bpp", "2",
        "--size", "24",
        "--no-compress",
        "--font", str(TTF),
        # Latin, punctuation, general punctuation, arrows, geometric shapes.
        "--range", "0x20-0x7E",
        "--range", "0x00A0-0x00FF",
        "--range", "0x2000-0x206F",
        "--range", "0x2190-0x21FF",
        "--range", "0x25A0-0x25FF",
        "--symbols", hangul,
        "--format", "lvgl",
        "--lv-include", "lvgl.h",
        "--lv-font-name", "korean_24",
        "-o", str(OUTPUT),
    ]

    result = subprocess.run(cmd, shell=(sys.platform == "win32"))
    if result.returncode != 0:
        print("error: lv_font_conv failed", file=sys.stderr)
        return result.returncode

    size_kb = OUTPUT.stat().st_size / 1024
    print(f"wrote {OUTPUT.relative_to(ROOT)} ({size_kb:.0f} kB of C source)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
