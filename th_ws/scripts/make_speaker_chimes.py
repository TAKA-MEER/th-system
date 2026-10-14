#!/usr/bin/env python3
"""make_speaker_chimes.py — 機体スピーカーの報知音の合成（Spec-safety.md §11）。

標準ライブラリだけ（numpy 無し）で wav を作る。各 22.05 kHz・モノラル・16 bit。
  drive.wav   … 穏やかなメロディ（搬送ロボット風。880→660 Hz のチャイム）
  blocked.wav … drive と聞き分けられる注意音（520 Hz 系の 2 鳴り）
  warning.wav … 短い断続の警告音（1200 Hz の 3 連）
試し鳴らしは drive.wav を 1 回鳴らす（rpi_speaker_play.py）。

使い方:
  python3 make_speaker_chimes.py [--out th_ws/audio/speaker]
"""
import argparse
import math
import os
import struct
import wave

RATE = 22050


def _tone(freqs, dur, vol=0.5, attack=0.01, decay_end=0.0):
    """freqs（Hz の列）を dur 秒重ねたサンプル列。始端アタック・終端へ減衰。"""
    n = int(RATE * dur)
    out = []
    for i in range(n):
        t = i / RATE
        env = min(1.0, t / attack) * (1.0 - (1.0 - decay_end) * (i / n))
        s = sum(math.sin(2 * math.pi * f * t) for f in freqs) / len(freqs)
        out.append(int(32767 * vol * env * s))
    return out


def _silence(dur):
    return [0] * int(RATE * dur)


def drive():
    # 880 Hz 0.35 秒 → 休み → 660 Hz 0.5 秒の穏やかなチャイム
    return (_tone([880.0], 0.35) + _silence(0.12) + _tone([660.0], 0.55, decay_end=0.2))


def blocked():
    # 523＋784 Hz の和音を 3 鳴り（drive の単音チャイムと聞き分けられる）
    one = _tone([523.25, 784.0], 0.25) + _silence(0.15)
    return one + one + _tone([523.25, 784.0], 0.4, decay_end=0.2)


def warning():
    # 1175 Hz の短い 4 連（断続の警告）
    part = _tone([1175.0], 0.12) + _silence(0.1)
    return part + part + part + part + _tone([1175.0], 0.15, decay_end=0.2)


def write_wav(path, samples):
    with wave.open(path, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(struct.pack(f'<{len(samples)}h', *samples))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'audio', 'speaker'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    for name, fn in (('drive.wav', drive), ('blocked.wav', blocked),
                     ('warning.wav', warning)):
        path = os.path.join(args.out, name)
        write_wav(path, fn())
        print(f'{path}: {os.path.getsize(path)} bytes')


if __name__ == '__main__':
    main()
