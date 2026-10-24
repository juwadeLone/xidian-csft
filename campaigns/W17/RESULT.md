# W17 CRAM injection on the lab board — campaign W17_20261020

Task: `tasks/20261020-w17-cram/TASK.md`. 1200 uniform essential-bit shots
(scene `point`) + 60 control shots + pre/post no-fault checks on the
laboratory Kintex-7 board. Status: **complete — runner DONE, all checks
PASS.**

## Environment (task §4)

- Machine `cmp`, Ubuntu 22.04.5 LTS, kernel 6.8.0-136-generic
- Python 3.10.12; Vivado 2018.3 (SW Build 2405991); xsim n/a here
- UART port `/dev/ttyACM0`; board NIC `enp4s0` 192.168.1.100
  (mac 10:ff:e0:3b:35:0a — `checks/cram_pre_W17.console.log` line 34)
- `net.core.rmem_max`: raised to **33554432** before the campaign per §4
  (`sysctl -w`, runtime only, not written to /etc). The pre-change value
  was not saved to a file; current value still 33554432 (no reboot).
- hw_server reachable at localhost:3121 (Digilent H20026FBDFE8).
- SETUP OK — all target/ebd/bit hashes verified before start.

## Timeline (UTC)

- 10:18:57  §5-0b bit-identity check start (`check0/`)
- 10:20:45  campaign script start (`steps.log`)
- 10:22:14  `cram_pre_W17` PASS → campaign start
- 15:30:12  campaign runner rc=0 (last shot)
- 15:35:41  `cram_post_W17` PASS → DONE

## Step 0b — bit identity (before campaign)

- `check0/bare_chain_run_w17_lab.json` + `w17_lab.console.log`: point and
  GF-3, 2 frames each — all equal the frozen W17 values, flags 0.
- JTAG readback `K7_USR_ACCESS a53503e8` (`check0/usr_access.log`) —
  matches the W17 build.
- `check0/frames.sha256`: SHA-256 of the 4 stored frames (frames
  themselves not pushed per §7).

## Pre / post no-fault checks

- `checks/cram_pre_W17.console.log` → **PASS**: 3 point + 3 GF-3 frames
  equal frozen SHA-256 (steps.log 10:22:14Z).
- `checks/cram_post_W17.console.log` → **PASS**: same 6 frames after the
  campaign (15:35:41Z).

## Campaign records (summary/summary.json, summary.md)

Sampling pool (population_e0): **8,504,179** essential bits.

| batch    | records | valid | identical | changed | below_line | DUE | invalid |
|----------|--------:|------:|----------:|--------:|-----------:|----:|--------:|
| e0_point |    1200 |  1200 |       966 |     151 |         43 |  40 |       0 |
| control  |      60 |    55 |        55 |       0 |          0 |   0 |       5 |

Control invalids are INVALID_NO_DETECT at shots 31, 35, 42, 49, 51
(non-essential target class; counted by the runner's own rules).

e0_point rates (Wilson 95 %):

| class      | k/n      | share                | cross-section, bits              |
|------------|----------|----------------------|----------------------------------|
| below_line | 43/1200  | 3.58 % (2.67–4.79 %) | 304,733 (227,151–407,508)        |
| DUE        | 40/1200  | 3.33 % (2.46–4.51 %) | 283,473 (208,977–383,297)        |
| changed    | 151/1200 | 12.58 % (10.83–14.58%)| 1,070,109 (920,610–1,239,917)   |

SEM uncorrectable flag set after shots **189** and **933** (records kept;
each followed by an automatic JTAG reprogram — the D302 handling).

Per-shot landing sites for the 83 below-line/DUE shots:
`summary/shots.csv` (i, level, psnr_db, lfa, clock region, tile kind,
word/bit, overflow field, due_class).  Read-only lookup of each site
against the routed W17 checkpoint: `lookup/lookup.tsv` (8,719 rows;
routed dcp SHA-256 verified = `948ec7a2…37dc0a` before running).

## Reprogramming count

`RECOVER:` lines in `camp/W17_20261020.console.log`: **41** =
1 initial programming + **38** after a hung device
(`STOP device class=RECOVERY_FAIL`) + **2** after the SEM uncorrectable
flag (shots 189, 933).

## Anomalies

- UART NAK bursts at addr 201326592 precede each hung-device recovery
  (console log; expected symptom of a DUE taking down the readout path).
- No Traceback, no equipment stop, no write-failure retries beyond the
  NAK pattern above; first-10-shot inspection was clean (MASKED, SEM
  corrected).
- Control batch: 5 INVALID_NO_DETECT (listed above); e0 batch: 0 invalid.

## Provenance

- ub16 checkout `46285269e15c18276d20168ccb063f80c0e2bf42`
  (host scripts), w17 checkout `1e9acb3a2421ed8e33591fa61d37b893d6521579`
  (bitstream build inputs) — from `steps.log` start line.
- Bitstream `k7_udp_ddr_top.bit` SHA-256
  `02800d91386d3adf168f5f191cc56588be6a6dac913fc476f358053c23412e9c`
  (recorded in each check json).
- Injection targets: `camp/W17_20261020/manifest.json`; per-shot raw
  records `camp/W17_20261020/{e0_point,control}/injections.jsonl`
  (1200 + 60); kept changed frames `e0_point/frames/*.bin.gz` (46 files).
- Lab commit: this directory on branch `lab`.
