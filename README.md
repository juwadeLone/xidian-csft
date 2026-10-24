# xidian-csft

RTL sources of the seven 2048-point CSA FFT designs and the CRAM
configuration-bit injection campaign results.

Device: Kintex-7 XC7K325T-FFG900-2. Tool: Vivado 2018.3. CS clock 80 MHz.

## rtl/

`chain/rtl/` of each design, exported at the exact commit used for the
build whose bitstream was injected. Third-party UDP/Ethernet cores under
`rtl/board/net/` are excluded.

| Directory  | Design                                   | Source commit |
|------------|------------------------------------------|---------------|
| `GR17/`    | 17-bit replicas, grouped replica TMR     | `0e1372e`     |
| `GR17M1A/` | replicas 1 bit narrower, narrower products | `b868c86`   |
| `GR17M2/`  | replicas 2 bits narrower                 | `6569c42`     |
| `GR17P1/`  | replicas 1 bit wider (capped at 16)      | `d42e7c3`     |
| `GR17P2/`  | replicas 2 bits wider (capped at 16)     | `615f37d`     |
| `TMR17/`   | full TMR arm (a.k.a. GR17B)              | `f431c5f`     |
| `W17/`     | unprotected 17-bit reference             | `1e9acb3`     |

## campaigns/

Injection campaign result directories as handed off from the lab branch
(`results/` of 2048-2048cs-k7-sem_low). Each contains RESULT.md,
COMMANDS.txt, SOURCE.txt, MANIFEST.sha256, campaign logs/records
(`camp/`), pre/post no-fault checks (`check0/`, `checks/`), landing
lookups (`lookup/`), and `summary/`.

| Directory  | Campaign (lab result dir)                |
|------------|------------------------------------------|
| `GR17/`    | `results/20261021-gr17fix-cram`          |
| `GR17M2/`  | `results/20261022-gr17m2-cram`           |
| `GR17P1/`  | `results/20261023-gr17p1-cram`           |
| `GR17P2/`  | `results/20261022-gr17p2-cram`           |
| `TMR17/`   | `results/20261022-gr17b-cram`            |
| `W17/`     | `results/20261020-w17-cram`              |
| `GR17M1A/` | `results/20261023-gr17m1a-lookup` (landing lookup only; the injection campaign was run on a different machine and its raw records are not included here) |

Every figure in RESULT.md traces to the recorded command, raw output,
tool version and SHA-256 in the same directory.
