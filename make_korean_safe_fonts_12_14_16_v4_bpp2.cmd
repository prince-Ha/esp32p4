@echo off
chcp 65001 > nul
setlocal

echo ========================================
echo LVGL Korean safe font generator v4
echo Fix: KOREAN_16 too large error
echo ========================================

cd /d "%~dp0"

if exist "NotoSansKR-Regular.ttf" (
  set "FONT=NotoSansKR-Regular.ttf"
) else if exist "C:\Windows\Fonts\malgun.ttf" (
  set "FONT=C:\Windows\Fonts\malgun.ttf"
) else (
  echo ERROR: Korean TTF font not found.
  echo Put NotoSansKR-Regular.ttf in this folder or use C:\Windows\Fonts\malgun.ttf.
  pause
  exit /b 1
)

if not exist "src" mkdir src

echo Font: %FONT%
echo.
echo This version uses:
echo   korean_12: bpp 2
echo   korean_14: bpp 2
echo   korean_16: bpp 2
echo to avoid LV_FONT_FMT_TXT_LARGE error.
echo.

echo Generating korean_12.c ...
call lv_font_conv.cmd ^
  --bpp 2 ^
  --size 12 ^
  --no-compress ^
  --font "%FONT%" ^
  --range 0x20-0x7E ^
  --range 0x00A0-0x00FF ^
  --range 0x2000-0x206F ^
  --range 0x2190-0x21FF ^
  --range 0x25A0-0x25FF ^
  --range 0x3130-0x318F ^
  --range 0xAC00-0xD7A3 ^
  --format lvgl ^
  --lv-include "lvgl.h" ^
  --lv-font-name korean_12 ^
  -o "src\korean_12.c"
if errorlevel 1 goto fail

echo Generating korean_14.c ...
call lv_font_conv.cmd ^
  --bpp 2 ^
  --size 14 ^
  --no-compress ^
  --font "%FONT%" ^
  --range 0x20-0x7E ^
  --range 0x00A0-0x00FF ^
  --range 0x2000-0x206F ^
  --range 0x2190-0x21FF ^
  --range 0x25A0-0x25FF ^
  --range 0x3130-0x318F ^
  --range 0xAC00-0xD7A3 ^
  --format lvgl ^
  --lv-include "lvgl.h" ^
  --lv-font-name korean_14 ^
  -o "src\korean_14.c"
if errorlevel 1 goto fail

echo Generating korean_16.c ...
call lv_font_conv.cmd ^
  --bpp 2 ^
  --size 16 ^
  --no-compress ^
  --font "%FONT%" ^
  --range 0x20-0x7E ^
  --range 0x00A0-0x00FF ^
  --range 0x2000-0x206F ^
  --range 0x2190-0x21FF ^
  --range 0x25A0-0x25FF ^
  --range 0x3130-0x318F ^
  --range 0xAC00-0xD7A3 ^
  --format lvgl ^
  --lv-include "lvgl.h" ^
  --lv-font-name korean_16 ^
  -o "src\korean_16.c"
if errorlevel 1 goto fail

echo.
echo DONE.
echo Generated:
echo   src\korean_12.c
echo   src\korean_14.c
echo   src\korean_16.c
echo.
echo Build again in PlatformIO.
pause
exit /b 0

:fail
echo.
echo ERROR: font generation failed.
pause
exit /b 1
