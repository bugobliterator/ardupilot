#!/usr/bin/env python3
"""Check a serial-logger file written in bench mode: a little-endian uint32 counter
starting anywhere and incrementing by one. Also generates such a stream.

  slog_verify.py LOG00001.TXT            -> exit 0 if gap-free
  slog_verify.py --generate N out.bin    -> N bytes of counter stream
"""
import struct, sys

def generate(nbytes, path, start=0):
    with open(path, "wb") as f:
        n = nbytes // 4
        f.write(b"".join(struct.pack("<I", (start + i) & 0xFFFFFFFF) for i in range(n)))

def verify(path):
    data = open(path, "rb").read()
    if len(data) < 8:
        print(f"{path}: too short ({len(data)} bytes)"); return 1
    words = struct.unpack(f"<{len(data)//4}I", data[:len(data)//4*4])
    missing, first_gap = 0, None
    for i in range(1, len(words)):
        step = (words[i] - words[i-1]) & 0xFFFFFFFF
        if step != 1:
            missing += (step - 1) * 4
            if first_gap is None:
                first_gap = i * 4
    print(f"{path}: {len(data)} bytes, first gap at {first_gap}, missing {missing} bytes")
    return 0 if first_gap is None else 1

if __name__ == "__main__":
    if sys.argv[1] == "--generate":
        generate(int(sys.argv[2]), sys.argv[3]); sys.exit(0)
    sys.exit(verify(sys.argv[1]))
