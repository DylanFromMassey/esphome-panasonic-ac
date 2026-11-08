# Panasonic CS-Z42XKRW over the DNSK-P11 (CN-WLAN) protocol

Protocol and capability notes for a **Panasonic CS-Z42XKRW** (Z-Series Premium /
Aero XKR, 4.2 kW reverse cycle, R32, 5 fan speeds + auto, nanoeX, iAUTO-X) driven
by an ESP32 running `esphome-panasonic-ac` in place of the original WLAN adapter.

Everything here was **measured on hardware**: each write was confirmed twice — the
AC's ack frame echoing the set, then the following report frame — and, where
physical motion matters, by watching the unit. Bolt-ons that only exist in a
datasheet are marked as such.

* Cooling range −10…+46 °C outdoor, heating −15…+24 °C outdoor.
* Serial: 9600 baud, **8 data bits, EVEN parity, 1 stop bit** (8E1).
* Poll interval: **30 s**. Size any capture to cover at least two cycles.

---

## 1. Framing

```
5A <counter> <type hi> <type lo> 00 <length> 01 01 30 01 <pairs>
   └─ pairs: [flag][key][0x01][value] ×N, starting at byte 11
   └─ last byte: checksum = two's complement of the sum of all preceding bytes
```

| Type | Direction | Meaning |
|---|---|---|
| `10 09` | adapter → AC | query (all values `len = 00`) |
| `10 89` | AC → adapter | query response, **125 bytes, full state** |
| `10 08` | adapter → AC | set |
| `10 88` | AC → adapter | set ack (echoes the pairs) |
| `10 0A` | AC → adapter | unsolicited report, **only the changed pairs** |
| `10 8A` | adapter → AC | ack for the above |
| `01 01` | either | ping |

Notes:

* Byte 1 is a per-direction packet counter. Byte 5 is the payload length, byte 10
  the pair count.
* In the 125-byte query response the pairs are fixed-stride: key at `12 + 4i`,
  value at `14 + 4i`. That is how the offsets below were derived.
* The tail after `0x86` (`2E 2A 00 00 0B 01 01 48 30 30 30 …`) is not state —
  `48 30 30 30` is the ASCII string `H000`.
* `10 0A` reports carry only changes, so a naive "latest report" can look like a
  state where most keys are missing. Prefer the `10 89` snapshot when verifying.

---

## 2. Key map

| Key | Meaning | Values | Notes |
|---|---|---|---|
| `0x80` | Power | `0x30` on, `0x31` off | |
| `0xB0` | Mode | `0x41` auto, `0x42` cool, `0x43` heat, `0x44` dry, `0x45` fan-only | iAUTO-X shares `0x41`; see `0x34` |
| `0x31` | Target temperature | °C × 2 (`22.5 °C` = `0x2D`) | 16.0–30.0, step 0.5 |
| `0xA0` | Fan speed | `0x32`…`0x36` = 1…5, `0x41` auto | |
| `0xB2` | Preset | `0x41` normal, `0x42` powerful, `0x43` quiet | |
| `0xA1` | Swing | `0x41` both, `0x42` off, `0x43` vertical, `0x44` horizontal | **only `0x41` actually sweeps this unit** |
| `0xA4` | Flap position (up/down) | `0x41` up, `0x44` up-centre, `0x43` centre, `0x45` down-centre, `0x42` down | motorised — verified by watching the vane |
| `0xA5` | Vane position (left/right) | `0x42` left, `0x5C` left-centre, `0x43` centre, `0x56` right-centre, `0x41` right | **accepted and echoed but nothing moves** on this unit |
| `0x33` | nanoeX | `0x45` on, `0x42` off | reports `0x43` for on |
| `0x34` | iAUTO-X | `0x43` on, `0x42` off | flag `0x02` |
| `0x20` | Operation flag | `0x43` running, `0x42` idle/off | tracks `0x33`, **not** a control |
| `0x35` | Companion | `0x42` | sent alongside presets, louver moves and swing-off |
| `0x21`, `0x32` | Unknown | `0x41`, `0x42` | constant across every test |
| `0xBB` | Inside temperature | integer °C | 1° resolution, not ×0.5 |
| `0xBE` | Outside temperature | integer °C | 1° resolution, not ×0.5 |

`0x31` is the only ×0.5-encoded value; `0xBB`/`0xBE` are plain integers.

---

## 3. What works, per function

| Function | Verdict |
|---|---|
| Power on/off | works |
| Modes cool/heat/dry/fan/auto | works |
| Target temperature 16.0–30.0 | works (see trap 4) |
| Fan 1–5 + auto | works |
| Presets Normal/Powerful/Quiet | works (refused while in auto — see behaviour notes) |
| **nanoeX on/off** | **works** — single pair `flag 0x02`, `0x33` = `0x45`/`0x42` |
| **iAUTO-X on/off** | **works** — `flag 0x02`, `0x34` = `0x43`/`0x42` |
| **Flap position (up/down)** | **works** — `0xA4`, five positions |
| **Flap sweep** | **works only via `0xA1 = 0x41` ("both")**; `0x42` parks it |
| Left/right vanes | not motorised — hand-set on this unit |
| Economy | **not a protocol state** — reports identically to Normal |
| ECONAVI | not present on this model (XKR generation; needs activity + sunlight sensors) |

### Behaviour notes (device-side, not bugs)

* **`0xA1 = 0x43` ("vertical") is inert.** It is acked and echoed, but two anchored
  tests showed the flap stay put. Only `0x41` sweeps.
* **Entering auto resets the preset to Normal**, and forces fan to auto.
* **Powerful/Quiet requests are refused in auto/iAUTO-X** — acked, `0xB2` stays
  `0x41` until the mode changes.
* **iAUTO-X drops the target ~2 °C** while it accelerates cooling (per the manual).
* **Sweeping re-centres the flap** to `0x43`; a fixed position is not preserved
  when a sweep starts.
* **DRY idles the indoor fan**, so the unit's own temperature sensor swings (e.g.
  22 → 26 → 23 °C) as it sits in stagnant air by the coil. Not a fault.
* **Heat is refused outside −15…+24 °C outdoor.** At exactly 24 °C it was accepted,
  so check the outdoor reading before judging a heat test.

---

## 4. Traps that cost time

1. **The flag byte belongs to its own pair.** Layout is `[flag][key][0x01][value]`
   from byte 11. An encoder that writes the flag *after* the value shifts it into
   the next pair's slot: correct-looking packets for multi-pair writes, and a flag
   that silently vanishes on the last/only pair. That produces packets the AC
   **acks and ignores** — indistinguishable from a read-only field by black-box
   testing. Diff your bytes against a genuine capture before declaring anything
   unwritable.
2. **Ack ≠ applied.** `0xA1 = 0x43` and every `0xA5` value are acked and echoed and
   change nothing physical. Verify motion for anything that moves.
3. **Some keys need a companion pair in the same packet.** A lone `0xA1` write is
   ignored; it needs a louver-position pair, and `A1 = 0x42` (off) additionally
   needs `0x35 = 0x42` in the same packet. That is exactly what the genuine
   adapter sends: `00 A1 01 42 | 00 A5 01 43 | 02 35 01 42`.
4. **Temperature must not be the last pair.** The AC ignores (and does not even
   ack) a set whose final pair is `0x31`; include the current fan speed as a
   trailer.
5. **Serialise set packets.** Sending a new set while the previous ack is
   outstanding desynchronises the counters (`Correcting shifted tx counter`) and
   can drop commands — 8 commands in 1.2 s lost the last one. Queue while
   `waiting_for_response_`, flush from `loop()`.
6. **Fan labels must not collide with ESPHome's standard fan modes.** `Low`,
   `Medium`, `High`, `Auto`, `Off` are mapped to standard climate fan modes and the
   request is dropped before it reaches the component; the only trace is
   `[W][climate:109]:   Fan Mode MEDIUM not supported`. Use `Speed 1`…`Speed 5`.
7. **A bare `off` in YAML is a boolean.** `swing_modes: [off, both]` fails
   validation; OFF is implicit, list only the active modes.

---

## 5. Reference configuration

```yaml
climate:
  - platform: panasonic_ac
    type: wlan
    name: Panasonic AC
    # Only "both" sweeps this unit's flap; OFF is implicit.
    swing_modes: [both]
    # Up/down flap (0xA4). The left/right vanes are hand-set: do not expose them.
    vertical_swing_select:
      name: Panasonic AC Flap (up-down)
    outside_temperature:
      name: Panasonic AC Outside Temperature
    nanoex_switch:
      name: Panasonic AC NanoeX Switch
    iautox_switch:
      name: Panasonic AC iAUTO-X
```

Left/right vanes, Economy and ECONAVI are deliberately absent: the hardware cannot
act on them, and an entity that only ever displays one value is worse than none.

---

## 6. Evidence and tooling

* `protocol/logic_analyzer/controller/*.dsl` — DSLogic captures of the **genuine
  adapter**, 100 kHz, probes `VCC/RX/TX`. These are the ground truth for packet
  layout; `tools/dsl_decode.py` decodes them.
* `tools/dsl_decode.py` — UART 8E1 decoder for those captures. Note the bits are
  **packed LSB-first** and the symbol period is ~10 samples, so the header's
  `samplerate = 100 kHz` is not the bit rate.
* Reference packets recovered from the captures:

```
nanoeX off : 5A.6D.10.08.00.09.01.01.30.01.01.02.33.01.42.6C
nanoeX on  : 5A.6E.10.08.00.09.01.01.30.01.01.02.33.01.45.68
mode+temp  : 5A.6F.10.08.00.11.01.01.30.01.03.00.B0.01.41.02.31.01.2E.00.A0.01.41.A2
louver     : 5A.5F.10.08.00.0D.01.01.30.01.02.00.A5.01.5C.02.35.01.42.71
preset     : 5A.42.10.08.00.15.01.01.30.01.04.00.B0.01.42.00.B2.01.43.02.35.01.42.02.34.01.42.24
```

* `protocol/protocol_description_controller.ods` — the adapter vendor's own sheet.
  Agrees on power/mode/fan/preset and the `0x31` ×0.5 encoding; **wrong** on swing
  auto (claims `0x41` = horizontal; hardware says `0x41` = both, `0x44` = horizontal).
* Verification harness (local workspace, not part of this repo): scripted sweeps
  that drive one command per step and check the ack, the following report and the
  raw key deltas — including burst tests, which is how the counter-desync bug was
  found.

---

## 7. Component history for this work

| Version | Change |
|---|---|
| 2.6.0 | Fixed set-packet flag placement (nanoeX/iAUTO-X were never read-only), added working nanoeX + iAUTO-X control |
| 2.7.0 | Per-device `swing_modes:` option with masking; fan labels `Speed 1`–`Speed 5` |
| 2.8.0 | Expose the motorised flap (`0xA4`) and drop the inert axis; `swing_modes: [both]` |
