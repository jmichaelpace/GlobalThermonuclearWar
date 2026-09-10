#!/usr/bin/env python3
"""Generate placeholder WAV sound effects for Global ThermonuclearWar.

These are procedural stand-ins (noise bursts, swept tones) so the app has
working audio out of the box. Replace them with licensed/recorded files of
the same names in assets/sounds/ — the loader accepts .wav, .ogg, or .mp3.

Run from the repo root:  python3 assets/sounds/generate_placeholders.py
"""

import math
import random
import struct
import wave

RATE = 44100


def write_wav(path, samples):
    """Write mono float samples [-1, 1] as a 16-bit WAV."""
    packed = b"".join(
        struct.pack("<h", max(-32768, min(32767, int(s * 32767)))) for s in samples
    )
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(packed)
    print(f"wrote {path} ({len(samples)/RATE:.2f}s)")


def seconds(n):
    return int(n * RATE)


def env(t, t0=0.0, attack=0.01, decay=0.3):
    """Attack/decay envelope: ramp 0->1 over `attack`, then exp decay."""
    if t < t0:
        return 0.0
    local = t - t0
    a = min(local / attack, 1.0) if attack > 0 else 1.0
    return a * math.exp(-local / decay)


class Rng:
    """Deterministic xorshift64* — same generator as the Rust version had."""

    def __init__(self, seed):
        self.state = seed & 0xFFFFFFFFFFFFFFFF or 1

    def next(self):
        x = self.state
        x ^= (x << 13) & 0xFFFFFFFFFFFFFFFF
        x ^= x >> 7
        x ^= (x << 17) & 0xFFFFFFFFFFFFFFFF
        self.state = x
        return ((x >> 11) / (1 << 53)) * 2.0 - 1.0


def missile_launch():
    """Low rumble: brown noise + rising 40->120 Hz fundamental."""
    rng = Rng(0x5EED0000C0FFEE01)
    brown = 0.0
    out = []
    for i in range(seconds(2.0)):
        t = i / RATE
        white = rng.next()
        brown = max(-1.0, min(1.0, brown + white * 0.02))
        f0 = 40.0 + 80.0 * min(t / 1.5, 1.0)
        tone = math.sin(2 * math.pi * f0 * t) * 0.4
        out.append((brown * 1.6 + tone) * env(t, attack=0.08, decay=0.9))
    return out


def interceptor_launch():
    """Sharp whoosh: low-pass noise with cutoff sweeping 200->2000 Hz."""
    rng = Rng(0xB0057EE71A5E5EED)
    lowpass = 0.0
    out = []
    dur = 0.6
    for i in range(seconds(dur)):
        t = i / RATE
        sweep = min(t / dur, 1.0)
        cutoff = 200.0 + 1800.0 * sweep
        alpha = math.exp(-2 * math.pi * cutoff / RATE)
        lowpass = lowpass * alpha + rng.next() * (1 - alpha)
        out.append(lowpass * 2.0 * env(t, attack=0.01, decay=0.25))
    return out


def intercept_hit():
    """Sharp crack (30 ms broadband) into a 60 Hz boom with long decay."""
    rng = Rng(0xDEC0DE5700000001)
    out = []
    for i in range(seconds(1.2)):
        t = i / RATE
        crack = rng.next() * env(t, attack=0.001, decay=0.01) * 1.5
        boom = math.sin(2 * math.pi * 60.0 * t) * env(t, attack=0.005, decay=0.45)
        out.append(crack + boom * 0.9)
    return out


def intercept_miss():
    """Soft muffled thud: 150 Hz low-passed noise."""
    rng = Rng(0x0FF1CE5500000001)
    lowpass = 0.0
    out = []
    for i in range(seconds(0.3)):
        t = i / RATE
        alpha = math.exp(-2 * math.pi * 150.0 / RATE)
        lowpass = lowpass * alpha + rng.next() * (1 - alpha)
        out.append(lowpass * 1.5 * env(t, attack=0.005, decay=0.12))
    return out


def self_destruct():
    """Short sharp pop."""
    rng = Rng(0x57E557E557E557E5)
    out = []
    for i in range(seconds(0.25)):
        t = i / RATE
        out.append(rng.next() * env(t, attack=0.001, decay=0.05))
    return out


def missile_impact():
    """Big boom: broadband crack + brown-noise rumble + 45 Hz fundamental."""
    rng = Rng(0x0A11CE0000000001)
    brown = 0.0
    out = []
    for i in range(seconds(2.5)):
        t = i / RATE
        crack = rng.next() * env(t, attack=0.001, decay=0.03) * 1.2
        white = rng.next()
        brown = max(-1.0, min(1.0, brown + white * 0.02))
        tone = math.sin(2 * math.pi * 45.0 * t) * 0.6
        body = (brown * 1.8 + tone) * env(t, attack=0.02, decay=1.2)
        out.append(crack + body)
    return out


def decoy_deployed():
    """Subtle pneumatic hiss: 800 Hz low-passed noise."""
    rng = Rng(0x0FF5EE000D0C0A5E)
    lowpass = 0.0
    out = []
    for i in range(seconds(0.2)):
        t = i / RATE
        alpha = math.exp(-2 * math.pi * 800.0 / RATE)
        lowpass = lowpass * alpha + rng.next() * (1 - alpha)
        out.append(lowpass * 2.0 * env(t, attack=0.01, decay=0.07))
    return out


def threat_warning():
    """Warning klaxon: two rising-falling 400->600 Hz whoops, urgent but not
    piercing. Placeholder for an air-raid/alert-tone recording."""
    out = []
    dur = 1.2
    for i in range(seconds(dur)):
        t = i / RATE
        # Two whoop cycles: triangle ramp 400->600->400 Hz per 0.6 s
        cycle = (t % 0.6) / 0.6
        f = 400.0 + 200.0 * (1 - abs(2 * cycle - 1))
        wave = math.sin(2 * math.pi * f * t)
        # Slight square-ish edge for urgency
        wave = math.copysign(1.0, wave) * 0.7 + wave * 0.3
        out.append(wave * 0.55 * env(t, attack=0.005, decay=10.0))
    return out


def main():
    sounds = {
        "missile_launch": missile_launch,
        "interceptor_launch": interceptor_launch,
        "intercept_hit": intercept_hit,
        "intercept_miss": intercept_miss,
        "self_destruct": self_destruct,
        "missile_impact": missile_impact,
        "decoy_deployed": decoy_deployed,
        "threat_warning": threat_warning,
    }
    for name, gen in sounds.items():
        write_wav(f"assets/sounds/{name}.wav", gen())


if __name__ == "__main__":
    main()