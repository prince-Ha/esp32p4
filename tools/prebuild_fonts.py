"""PlatformIO pre-build hook: keep the heading font in step with the source.

korean_24_bold contains only the Hangul that main.cpp's string literals use, so
adding a heading without regenerating it makes the new characters render as
empty boxes. Running the generator on every build removes that failure mode;
it exits immediately when the character set has not changed.
"""

import subprocess
import sys
from pathlib import Path

Import("env")  # noqa: F821  - injected by PlatformIO

ROOT = Path(env.subst("$PROJECT_DIR"))  # noqa: F821
SCRIPT = ROOT / "tools" / "make_heading_font.py"

if SCRIPT.exists():
    result = subprocess.run(
        [sys.executable, str(SCRIPT)],
        cwd=str(ROOT),
    )
    if result.returncode != 0:
        # A stale font is a cosmetic fault, not a reason to block the build.
        print("warning: heading font generation failed; fonts may be stale")
