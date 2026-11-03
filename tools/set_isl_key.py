"""Put the 지능형 과학실 인증키 onto the board over the serial console.

The key is 43-44 characters and the settings screen masks it, so a character
dropped by the touch keyboard is invisible - the board just gets told
"인증키가 일치하지 않습니다" and the screen says "start 응답 오류". Pasting it
is the difference between a key that works and one that is one character short.

    python tools/set_isl_key.py            # COM7, 지금 고른 서버
    python tools/set_isl_key.py COM5

Paste at the prompt rather than typing. The key goes to NVS on the board, which
survives reflashing, so this is done once and not again.
"""

import subprocess
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else 'COM7'


def read_reply(link, seconds=3.0):
    end = time.time() + seconds
    out = b''
    while time.time() < end:
        out += link.read(4096)
    return out.decode('utf-8', 'replace').splitlines()


def send(link, line, quiet=False):
    link.write((line + '\n').encode())
    time.sleep(0.4)

    for text in read_reply(link):
        if '[CMD]' in text and not quiet:
            print('  ' + text.strip())


def clipboard_text():
    """Whatever is on the Windows clipboard, or '' if it cannot be read."""
    try:
        out = subprocess.run(
            ['powershell.exe', '-NoProfile', '-Command', 'Get-Clipboard'],
            capture_output=True, text=True, timeout=10)
        return out.stdout.strip()
    except Exception:
        return ''


# getpass hides the paste, which is the wrong trade here: a key you cannot see
# is exactly how the wrong one got onto the board. Read it plainly and show the
# ends back, so a truncated paste is visible before it is sent.
#
# The key is almost always already on the clipboard - it was copied from the
# platform to be typed in. Offering it saves the one step where characters get
# lost, and it still has to be looked at and accepted.
pasted = clipboard_text()
suggestion = pasted if len(pasted) in (43, 44) and ' ' not in pasted else ''

if suggestion:
    print('클립보드에 %d자가 있습니다: %s...%s'
          % (len(suggestion), suggestion[:6], suggestion[-6:]))
    prompt = '이걸 쓰려면 Enter, 아니면 붙여넣고 Enter: '
else:
    prompt = '인증키를 붙여넣고 Enter: '

try:
    key = input(prompt).strip() or suggestion
except (EOFError, KeyboardInterrupt):
    print('취소되었습니다.')
    raise SystemExit(1)

if not key:
    print('입력이 없습니다.')
    raise SystemExit(1)

print('%d자: %s...%s' % (len(key), key[:6], key[-6:]))

if len(key) not in (43, 44):
    print('경고: 43자 또는 44자가 아닙니다. 붙여넣기가 잘렸는지 확인하세요.')

try:
    link = serial.Serial(PORT, 115200, timeout=0.5)
except serial.SerialException as error:
    print('%s를 열 수 없습니다: %s' % (PORT, error))
    print('시리얼 모니터가 열려 있으면 닫고 다시 실행하세요.')
    raise SystemExit(1)

# Opening the port resets the board on this adapter, so wait for it to come up.
print('보드가 부팅될 때까지 기다립니다...')
time.sleep(7)
link.reset_input_buffer()

# A bare newline clears whatever reset noise landed in the board's line buffer.
send(link, '', quiet=True)
send(link, 'key ' + key)
send(link, 'status')

link.close()

print()
print('위 status 줄의 글자 수가 붙여넣은 길이와 같으면 저장된 것입니다.')
print('다른 서버 칸에도 넣으려면: 보드에서 서버를 바꾼 뒤 다시 실행하세요.')
