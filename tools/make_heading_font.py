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

import hashlib
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SOURCE = ROOT / "src" / "main.cpp"
OUTPUT = ROOT / "src" / "korean_24_bold.c"
DIGITS_OUTPUT = ROOT / "src" / "digits_64.c"
STAMP = ROOT / "src" / ".heading_font_stamp"
TTF = Path("C:/Windows/Fonts/malgunbd.ttf")

# Hangul syllables, plus compatibility Jamo for standalone consonants/vowels.
HANGUL = re.compile(r"[가-힣㄰-㆏]")

# Ranges the fonts always carry: Latin, Latin-1, general punctuation, arrows,
# geometric shapes.
RANGES = [
    (0x0020, 0x007E),
    (0x00A0, 0x00FF),
    (0x2000, 0x206F),
    (0x2190, 0x21FF),
    (0x25A0, 0x25FF),
]


def collect_hangul(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    return "".join(sorted(set(HANGUL.findall(text))))


def collect_extras(path: Path) -> str:
    """Any other non-ASCII character the firmware displays.

    ℃ (U+2103) sits in none of the ranges above, and cutting the fonts by
    range alone silently dropped it — the degree sign rendered as an empty box
    on the measurement screen. Collecting leftovers the same way the Hangul is
    collected means a newly used symbol cannot go missing either.
    """
    text = path.read_text(encoding="utf-8")
    extras = set()

    for ch in text:
        code = ord(ch)
        if code < 0x00A0 or HANGUL.match(ch):
            continue
        if any(lo <= code <= hi for lo, hi in RANGES):
            continue
        extras.add(ch)

    return "".join(sorted(extras))


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

    extras = collect_extras(SOURCE)
    if extras:
        print(f"extra symbols: {extras}")

    # Regenerating takes about a minute, so skip it when the set of characters
    # has not moved. The stamp is what makes the pre-build hook cheap enough to
    # run on every build — and running on every build is what stops a newly
    # added heading from rendering as tofu.
    stamp = hashlib.sha256((hangul + extras).encode("utf-8")).hexdigest()
    force = "--force" in sys.argv

    if not force and STAMP.exists() and STAMP.read_text().strip() == stamp \
            and OUTPUT.exists() and DIGITS_OUTPUT.exists():
        print(f"heading font up to date ({len(hangul)} syllables)")
        return 0

    print(f"{len(hangul)} distinct Hangul syllables found in {SOURCE.name}")

    rc = run_conv([
        "--size", "24",
        # Latin, punctuation, general punctuation, arrows, geometric shapes.
        "--range", "0x20-0x7E",
        "--range", "0x00A0-0x00FF",
        "--range", "0x2000-0x206F",
        "--range", "0x2190-0x21FF",
        "--range", "0x25A0-0x25FF",
        "--symbols", hangul + extras,
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

    STAMP.write_text(stamp)

    for path in (OUTPUT, DIGITS_OUTPUT):
        print(f"wrote {path.relative_to(ROOT)} ({path.stat().st_size / 1024:.0f} kB of C source)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
