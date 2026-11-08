"""Decode a DSLogic .dsl capture (Logic 1.x zip) of the Panasonic adapter UART.

Captures in protocol/logic_analyzer/ are 100 kHz with probes VCC/RX/TX.
Digital samples are packed LSB-first, one bit per sample, giving ~10 samples
per symbol at 9600 baud. Frames are 9600 8E1, LSB-first data bits.

Usage: python3 tools/dsl_decode.py protocol/logic_analyzer/controller/nanoe_off_on.dsl
"""
import sys
import zipfile

SAMPLE_HZ = 100_000
BAUD = 9600
SPB = SAMPLE_HZ / BAUD


def bits_of(data):
    out = bytearray()
    for byte in data:
        # Captures pack samples LSB-first within each byte (verified empirically;
        # MSB-first turns valid frames into noise).
        for i in range(8):
            out.append((byte >> i) & 1)
    return out


def decode_uart(bits):
    """Return [(sample_index, byte)] for every valid 8E1 frame."""
    res = []
    i = 0
    n = len(bits)
    while i < n - 2:
        if bits[i] == 1 and bits[i + 1] == 0:  # falling edge => start bit
            start = i + 1
            if int(start + SPB * 10.5) >= n:
                break
            val = 0
            ok = True
            for b in range(8):
                s = int(start + SPB * (1.5 + b))
                if bits[s] not in (0, 1):
                    ok = False
                val |= bits[s] << b
            par = bits[int(start + SPB * 9.5)]
            stop = bits[int(start + SPB * 10.5)]
            ones = bin(val).count("1") + par
            if ok and stop == 1 and ones % 2 == 0:  # even parity
                res.append((start, val))
                i = int(start + SPB * 10.5)
                continue
        i += 1
    return res


def packets(decoded, gap_bits=25):
    """Group bytes into packets, splitting on idle gaps."""
    out, cur, last_end = [], [], None
    for start, val in decoded:
        if last_end is not None and start - last_end > gap_bits * SPB:
            if cur:
                out.append(cur)
            cur = []
        cur.append(val)
        last_end = start + SPB * 10
    if cur:
        out.append(cur)
    return out


def main(path):
    z = zipfile.ZipFile(path)
    hdr = z.read("header").decode()
    print(hdr)
    for probe, name in ((1, "RX (AC->adapter)"), (2, "TX (adapter->AC)")):
        bits = bits_of(z.read(f"L-{probe}/0"))
        pkts = packets(decode_uart(bits))
        print(f"\n===== {name}: {len(pkts)} packets =====")
        for p in pkts:
            print("  " + ".".join(f"{b:02X}" for b in p))


if __name__ == "__main__":
    main(sys.argv[1])
