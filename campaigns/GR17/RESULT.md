# GR17fix CRAM injection campaign — COMPLETE, all gates passed

Task `tasks/20261021-gr17-cram/TASK.md` (updated revision, sha256
`54c23f03fba3ecadede5456832b45905c0e9bb9720a56d2ff2f04a37d98f06ae`,
lab `b80b0ab`). Authorizations D400/D415. Method identical to
`results/20261020-w17-cram/`; the differences per the task card:
corrected bitstream (`gr17@0e1372e`, `results/20261021-gr17-fix/`),
re-drawn targets (pool 23,859,950), new work dir `~/l/gr17fix-cram`,
results here. First-version directory `results/20261021-gr17-cram/`
untouched.

Machine: Ubuntu 22.04.5 LTS, kernel 6.8.0-136-generic, Python 3.10.12,
Vivado 2018.3 (SW 2405991), XC7K325T-FFG900-2, board via hw_server
localhost:3121 (Digilent H20026FBDFE8), serial /dev/ttyACM0,
enp4s0 192.168.1.100.

Timeline (UTC): setup/env ~00:1xZ, §0b 00:16–00:19Z,
campaign 00:22:51Z–05:02:50Z (~4 h 40 m), post-check PASS 05:04:44Z,
DONE 05:04:44Z; tile lookup finished 05:06Z.

## §4 environment (actual values)

| item | required | actual |
|---|---|---|
| python3 + numpy/scapy/serial | works | 3.10.12, import ok |
| /dev/ttyACM0, enp4s0 | present | present, 192.168.1.100 |
| `net.core.rmem_max` | 33554432 during run | **before 33554432, after 33554432** (already at target from previous campaigns; no change made) |
| Vivado 2018.3 + hw_server sees board | yes | `/tools/Xilinx/Vivado/2018.3/bin/vivado`, one xc7k325t |
| exclusivity / ~5 h window | yes | no other board task; apt-daily-upgrade next trigger 22 h away |
| checkouts | — | ub16 `46285269e15c18276d20168ccb063f80c0e2bf42` (clean), w17 `1e9acb3a2421ed8e33591fa61d37b893d6521579` (clean) |
| setup_data.py | `SETUP OK` | printed; pool 23,859,950 bits |

## §0b bit-consistency check — PASS (all four items)

- rc=0; last console line `PASS: 2 point + 2 GF-3 frame(s) equal the frozen SHA-256`.
- Saved frames sha256 (`check0/frames.sha256`): point ×2
  `79ca5c05a217e6e0be684ffd2d8a23a325ce909bb3cf6f40d157f2c510a0056c`,
  gf3 ×2 `3a794a1ad520ff8f8fca00634e6ca474e9b3dd11480b5d595a0c437ff4948d86`.
- Flags: `no protection-flag words in this build` (expected — sticky flag
  not readable).
- JTAG `K7_USR_ACCESS ad341b6b` == corrected build (`usr_access.log`).
- One clean attempt; no write-path misalignment this time.

## Campaign

Batches: `e0_point` 1,200 uniform-sampled essential-bit targets
(`targets_e0.json` `b3a8da86c434f94e…`) and `control` 60 non-essential
(`targets_control.json` `328540e4028187cc…`), both on the point
scenario; no GF-3 batch. Seed label 20261014. `--expect-vec-id none`.

Pre-campaign no-fault check `cram_pre_GR17`: **PASS** (3 point + 3 GF-3
== frozen). Post-campaign `cram_post_GR17`: **PASS** (3 + 3). First-10
watch: 0 INVALID, no Traceback, health `ok` — gate passed.

### summary.md (verbatim)

```
pool (population_e0): 23,859,950 bits

| batch    | records | valid | identical | changed | below_line | DUE | invalid |
| e0_point | 1200    | 1200  | 1134      | 59      | 4          | 3   | 0       |
| control  | 60      | 56    | 56        | 0       | 0          | 0   | 4       |

e0_point cross-sections (Wilson 95 %), pool 23,859,950:
  below_line  4/1200   0.33 % (0.13-0.85 %)  ->  79,533 (30,946-203,753) bits
  DUE         3/1200   0.25 % (0.09-0.73 %)  ->  59,650 (20,294-174,764) bits
  changed    59/1200   4.92 % (3.83-6.29 %)  -> 1,173,114 (914,000-1,500,881) bits

SEM uncorrectable flag after shots (e0_point): []
below-line and hang shots: 7 (shots.csv)
```

The 4 control `INVALID_NO_DETECT` (shots 31, 35, 42, 49) are the
non-essential-bit no-detection category recorded inside
`control/injections.jsonl` (no separate invalid.jsonl was written by
the runner — same layout as the W17 campaign).

### below-line shots (4) — output change below the SDC line

| shot | tile_kind @ clock_region (tile_x) | word.bit | psnr | voter overflow flags |
|---|---|---|---|---|
| 141 | INT_INTERFACE_L @ X1Y2 (94) | 46.30 | 30.82 | p1 p3 p4 p6 |
| 300 | CLBLM_R @ X1Y2 (59) | 42.29 | 40.90 | p3 p4 p6 |
| 365 | CLBLM_R @ X0Y4 (33) | 75.7 | 17.76 | p4 p6 |
| 1134 | CLBLM_L @ X1Y2 (72) | 23.7 | 43.12 | (none recorded) |

### DUE shots (3) — device stopped answering, recovered by JTAG reprogram

| shot | tile_kind @ clock_region (tile_x) | word.bit | class |
|---|---|---|---|
| 211 | CLBLM_L @ X1Y1 (72) | 77.11 | RECOVERY_FAIL clear=busy |
| 642 | CLBLM_L @ X1Y2 (64) | 24.20 | RECOVERY_FAIL clear=busy |
| 1181 | CLBLM_R @ X1Y3 (61) | 40.18 | RECOVERY_FAIL clear=busy |

### Reprogramming count

`RECOVER: JTAG reprogram` lines: 4 = **1 initial program** (campaign
start, expected) + **3 recoveries**, all DUE `RECOVERY_FAIL clear=busy`.
`STOP SEM uncorrectable` lines: **0** (SEM uncorrectable flag list empty).

### Read-only tile lookup (§5)

`lookup/lookup.tsv` — 7 below-line/hang shots, 1,013 tile-level rows,
routed dcp `/home/xdu/l/gr17/chain/build/snap/GR17/build/vivado/
PROT_GR17/PROT_GR17.runs/impl_1/k7_udp_ddr_top_routed.dcp`
sha256 `e40e8070555af4cb7e8ca05c0293f74ca76be6a9fa8ee29590c2dd68bfdaf772`
(matches the corrected-build value; checkpoint not modified).

## Notes

- Masked share 1134/1200 = 94.5 % (W17 campaign: 966/1200 = 80.5 %) —
  consistent with the RPR replica/voter coverage of stages 0–8.
- The `overflow` voter-flag column on below-line shots is recorded
  verbatim from the runner's scoring; flag words are not readable on
  this build (`no protection-flag words in this build` throughout).
- Frame dumps of changed outputs: `camp/GR17_20261021/e0_point/frames/`
  (7 `.bin.gz`).
- Board left configured with the corrected GR17 bitstream (volatile);
  no configuration flash touched; no injection into GF-3 scenario.
- Work dir `~/l/gr17fix-cram` kept on disk (inputs, golden, targets,
  4 §0b raw frames in `check0/`); not pushed per task card.

## Files

`check0/` — §0b json, console log, `usr_access.log`, `frames.sha256`
(raw 32 MB frames not pushed). `checks/` — pre/post no-fault json +
console logs. `camp/GR17_20261021/` — manifest, progress, per-batch
`injections.jsonl`, `e0_point/frames/*.bin.gz`, campaign console log.
`summary/` — summary.md/json, shots.csv, summarize.log.
`lookup/` — lookup.tsv, lookup.log. `steps.log`, `nohup.log`,
`COMMANDS.txt`, `SOURCE.txt`, `MANIFEST.sha256`.
