"""Put the 지능형 과학실 인증키 onto the board over the serial console.

The key is 44 characters and the settings screen masks it, so a character
dropped by the touch keyboard is invisible - the board just gets told
"인증키가 일치하지 않습니다" and the screen says "start 응답 오류". Pasting it
is the difference between a key that works and one that is one character short.

    python tools/set_isl_key.py [COM7]

The key is read from the prompt, not from the command line, so it does not end
up in the shell history.
"""

import getpass
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else 'COM7'


def send(link, line):
    link.write((line + '\n').encode())
    time.sleep(0.4)

    end = time.time() + 3
    out = b''
    while time.time() < end:
        out += link.read(4096)

    for text in out.decode('utf-8', 'replace').splitlines():
        if '[CMD]' in text:
            print(text.strip())


key = getpass.getpass('인증키를 붙여넣고 Enter: ').strip()

if not key:
    print('입력이 없습니다.')
    raise SystemExit(1)

print('%d자를 보냅니다.' % len(key))

link = serial.Serial(PORT, 115200, timeout=0.5)

# Opening the port resets the board on this adapter, so wait for it to come up.
print('보드가 부팅될 때까지 기다립니다...')
time.sleep(7)
link.reset_input_buffer()

# A bare newline clears whatever reset noise landed in the board's line buffer.
send(link, '')
send(link, 'key ' + key)
send(link, 'status')

link.close()
