@echo off
chcp 65001 >nul
cd /d "%~dp0"

echo [1/4] Check project files...
if not exist "src\main.cpp" (
  echo ERROR: src\main.cpp not found. Put this file in the PlatformIO project root.
  pause
  exit /b 1
)
if not exist "NotoSansKR-Regular.ttf" (
  echo ERROR: NotoSansKR-Regular.ttf not found in project root.
  echo Put NotoSansKR-Regular.ttf next to platformio.ini.
  pause
  exit /b 1
)

echo [2/4] Check lv_font_conv...
where lv_font_conv.cmd >nul 2>nul
if errorlevel 1 (
  echo ERROR: lv_font_conv.cmd not found.
  echo Run: npm.cmd install -g lv_font_conv
  pause
  exit /b 1
)

echo [3/4] Generate full Korean LVGL font...
echo This may take a while.
lv_font_conv.cmd --bpp 2 --size 16 --no-compress --stride 1 --align 1 --font "NotoSansKR-Regular.ttf" --range 0x20-0x7E --range 0xAC00-0xD7A3 --range 0x3130-0x318F --range 0x00A0-0x00FF --range 0x2000-0x206F --range 0x2190-0x21FF --range 0x25A0-0x25FF --format lvgl -o "src\korean_16.c"
if errorlevel 1 (
  echo ERROR: font generation failed.
  pause
  exit /b 1
)

echo [4/4] Done.
dir "src\korean_16.c"
pause
