# RiseTogether Bela forwarder

The Bela is the hardware hub of the RiseTogether experiment: the link between the
experiment coordinator (a Raspberry Pi), the EEG recording, and the iPads that
present the stimuli.

```
   Raspberry Pi ───trigger──▶ ┌──────┐ ───forwarded verbatim───▶ EEG amp
                              │ Bela │ ───jittered ~1 Hz timer──▶ EEG amp
   iPads ────────photodiode──▶└──────┘
   (Polly, Pia, …)                └── logs/<session>/*.csv
```

Everything the Bela sees or emits lands on one clock — the audio frame counter,
22.7 µs per frame at 44.1 kHz. A latency between two parties is therefore a frame
difference, with no cross-device clock involved and nothing to correct for.

## What it does

**Forwards the Pi's trigger** to the EEG amp as a sample-accurate level mirror.
The output reproduces the Pi's pulse shape verbatim, delayed by exactly one block
— the PRU writes block *k*'s output word during block *k+1*, so 16 frames ≈
0.36 ms at `--period 16`. Constant, and logged as `output_block_delay_frames`.

**Emits its own trigger** at 1 Hz ± 200 ms with a 10 ms pulse. The jitter is
deliberate: a metronomic pulse train would beat against the EEG and inject a
correlated artefact. Intervals are drawn as `period + U(−J, +J)` and scheduled
from the previous *scheduled* frame, not the emitted one, so the mean rate stays
exactly 1 Hz and quantisation never accumulates. The xorshift32 seed is written
into `_meta.json`, so the entire schedule can be regenerated offline.

**Watches one photodiode per iPad**, each with a device name. A device's
photodiode edge minus the forwarded trigger frame is that device's display
latency. The photodiodes are read as digital comparator outputs, not analog.

**Logs everything, off the real-time path.** `render()` makes no clock call, no
allocation and no I/O — it pushes fixed-size records onto lock-free SPSC queues,
and a plain `std::thread` drains them to CSV on a 20 ms poll. Nothing about the
logging can stall the forward or shift a trigger.

The timer trigger appears in both the EEG record and the Bela log, which is what
aligns the two afterwards. Photodiode edges exist only on the Bela — the frame
axis is the bridge.

## Pin map

Pins are a compile-time table at the top of `render.cpp`. Adding an iPad means
adding a row; `setup()` refuses to start on a duplicated or out-of-range pin
rather than recording a session that looks fine and is quietly wrong.

| pin | direction | role | device |
|---|---|---|---|
| 0 | in | `photodiode` | Polly |
| 1 | in | `photodiode` | Pia |
| 4 | in | `trigger_in` | rpi |
| 12 | out | `trigger_fwd` | mirrors pin 4 → EEG |
| 13 | out | `trigger_timer` | jittered local trigger → EEG |

Two things are indexed by table position rather than by pin number — the
`gPinStatesAtomic` bitfield and `gRefractoryFrames[]` — so there is a hard limit
of 16 input rows.

`active_level` is the level that means "asserted"; it is per-device, because
comparator polarity is a wiring choice. `refractory_ms` only sets the `accepted`
flag in the log — **no edge is ever discarded**, and the Pi's trigger is
forwarded before the refractory check is even reached, so a chattering line can
never cost the EEG a trigger.

Both outputs are held idle for the first 250 ms while the PRU settles and the
startup pin scan runs.

## Layout

```
render.cpp                      the application
settings.json                   Bela CLArgs; period 16, digital on, analog off
build.sh                        cross-compiles and deploys via ../Bela/scripts
lsl_api.cfg                     liblsl config; only used if ENABLE_LSL is 1
docs/flutter_outlet_spec.md     the iPad-side LSL contract (not used at present)
analysis/                       from the old timing rig — see the note below
cross/build_liblsl.sh           cross-compiles liblsl for armhf into lib/
cross/bela-armhf.cmake          CMake toolchain file for the Bela target
cross/gcc6-compat.patch         C++17 constructs the Bela's gcc 6.3 lacks
```

Bela compiles every `*.cpp` in the project directory, so there must only ever be
one.

## LSL

`ENABLE_LSL` is `0`. LSL happens elsewhere in the stack now, so the Bela runs no
inlet and no outlet. Every line of that code, the headers, `lib/liblsl.so*`,
`lsl_api.cfg` and the link flags in `build.sh` are all kept, so restoring it is a
one-character change at the top of `render.cpp`. With LSL off, `nowClock()` falls
back to `CLOCK_MONOTONIC` and `_sync.csv` still yields a frame↔clock fit.

## Build

```sh
./build.sh          # cross-compiles and deploys via ../Bela/scripts
```

Requires the cross-toolchain and sysroot from `SyncBelaSysroot.sh`. The project
name in `build.sh` (`-p`) is the directory it deploys into on the board and must
match the three absolute paths in `-m`, or the rpath breaks.

To syntax-check against the board's own gcc 6.3.1 without deploying:

```sh
/usr/local/linaro/arm-bela-linux-gnueabihf/bin/arm-bela-linux-gnueabihf-g++ \
  -std=c++1z -fsyntax-only -Wall -Wextra \
  --sysroot=/usr/local/linaro/BelaSysroot \
  -isystem /usr/local/linaro/BelaSysroot/usr/include/arm-linux-gnueabihf \
  -I../Bela/include -I. -Iinclude render.cpp
```

### Rebuilding liblsl

`lib/liblsl.so` is built on the host with the same toolchain — nothing is
installed on the board:

```sh
cross/build_liblsl.sh [path-to-liblsl-source] [--install]
```

It stages a copy of the source, applies `cross/gcc6-compat.patch` (the Bela
toolchain's gcc 6.3.1 is a C++11/14 front-end and rejects three C++17 constructs
liblsl 1.17 uses: `inline` static data members, `if constexpr`, and
`std::string_view`), builds, strips, and with `--install` copies the library and
its public headers into `lib/` and `include/`. The source tree itself is never
modified.

## Output

One directory per session, `logs/<YYYYMMDD_HHMMSS>_<rand>/`:

| file | one row per |
|---|---|
| `*_edges.csv` | input edge (frame, pin, role, device, state, active, refractory flags) |
| `*_triggers.csv` | output level change (frame, source, seq, level, jitter, lateness) |
| `*_sync.csv` | frame↔clock pair with a `bracket_frames` staleness bound |
| `*_status.csv` | session start/end, XRUN, BLOCK_GAP, QUEUE_FULL |
| `*_meta.json` | pin map, device names, timer config, RNG seed, rates, file index |

`_triggers.csv` is the definitive record of what the EEG amp saw. `_edges.csv`
holds the Pi's input edges and every device's photodiode edges. All three share
the frame axis. `_meta.json` is written at *startup*, so it survives a hard kill.

`source` in `_triggers.csv` is `forward` or `timer`. On a timer rising edge,
`jitter_frames` is the offset drawn for the **next** interval, and `late_frames`
is non-zero only when a dropped block delayed the pulse.

All schemas are fixed-width — no ragged rows.

## Before a session

- **Confirm the pin map against the wiring.** The startup pin scan prints every
  digital pin with its role, device and 100 ms activity summary. A photodiode on
  the wrong pin looks exactly like a dead sensor.
- **Check comparator polarity per device.** `active_level` is per-row; a flipped
  comparator produces edges that are all logged and all inverted.
- `ntpdate` the Bela (no battery-backed RTC) so session directories sort. All
  data timestamps are frames or `local_clock()`, so this is cosmetic only.
- **Check `*_status.csv` shows zero XRUN and zero BLOCK_GAP** before trusting a
  session, and that `cleanup()` reported `dropped events: 0`. A dropped block
  means digital frames were never read, so an edge could be missing entirely and
  a timer pulse could have gone out late — which is the difference between "the
  iPad never flashed" and "the Bela missed it".

## Known confounds

- **The comparator threshold and the pixel rise time** give the photodiode edge a
  fixed bias (a few ms on LCD, ~1 ms on OLED). It is a bias, not jitter, so it is
  tolerable if measured once and documented.
- **Pin the iPad's display refresh rate.** A ProMotion iPad ramping 60→120 Hz on
  touch injects variable latency into every trial.
- **Fix the photodiode patch position** and record it — scanout is row-by-row, so
  vertical position costs up to a full refresh period.
- Mirroring the photodiodes to analog inputs would show the rise directly and
  remove the threshold guesswork. Digital-only by choice.

## A note on `analysis/`

`analysis/` belongs to the previous incarnation of this repo, a timing rig that
measured an iPad's input→display→network chain against an FSR and an LSL stream.
It computes T1–T4 and a clock-offset bracket from a schema that no longer exists
here: it expects an `fsr` role and `_lsl.csv`, and knows nothing about
`_triggers.csv`. It is kept for reference and does **not** run against a
forwarder session.

# Acknowledgements

- [Christian A. Kothe: liblsl](https://github.com/sccn/liblsl) for the LSL library
- [armlabs: OLED SSD1306 Linux driver](https://github.com/armlabs/ssd1306_linux) for the OLED driver
- [Liam Donovan <liam@bela.io>: Bela](https://bela.io) for the Bela platform
