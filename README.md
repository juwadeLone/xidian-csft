# xidian-csft

RTL sources of seven builds of a 2048 x 2048 fixed-point chirp scaling
(CS) SAR chain, and the records of their configuration-memory (CRAM)
injection campaigns.

Device: Kintex-7 XC7K325T-FFG900-2. Tool: Vivado 2018.3. CS clock 80 MHz.

## rtl/

`chain/rtl/` of each design, exported at the exact commit used for the
build whose bitstream was injected. Third-party UDP/Ethernet cores under
`rtl/board/net/` are excluded.

| Directory  | Design                                     | Source commit |
|------------|--------------------------------------------|---------------|
| `GR17/`    | 17-bit replicas, grouped replica TMR       | `0e1372e`     |
| `GR17M1A/` | replicas 1 bit narrower, narrower products | `b868c86`     |
| `GR17M2/`  | replicas 2 bits narrower                   | `6569c42`     |
| `GR17P1/`  | replicas 1 bit wider (capped at 16)        | `d42e7c3`     |
| `GR17P2/`  | replicas 2 bits wider (capped at 16)       | `615f37d`     |
| `TMR17/`   | whole CS application triplicated           | `f431c5f`     |
| `W17/`     | unprotected 17-bit reference               | `1e9acb3`     |

## campaigns/

One directory per build, 1200 injected shots of the point-target scene
each. Every directory holds the campaign records (`camp/`: one JSON line
per shot with the flipped bit and its outcome, and under `frames/` the
board output of every shot below the line and of the first three changed
shots), the fault-free checks before and
after the campaign, and `summary/` with `shots.csv`. The lab campaigns
also hold RESULT.md, COMMANDS.txt, SOURCE.txt and MANIFEST.sha256, and
the lookup of the flipped bits on the routed design (`lookup/`).

| Directory  | Campaign                                   | Bitstream (SHA-256) |
|------------|--------------------------------------------|---------------------|
| `GR17/`    | lab `results/20261021-gr17fix-cram`        | `cc016c99...`       |
| `GR17M1A/` | `GR17M1A_20261023` (local), with the lab landing lookup `results/20261023-gr17m1a-lookup` (`lookup.tsv`, `lookup.log`) | `49a8bfab...` |
| `GR17M2/`  | lab `results/20261022-gr17m2-cram`         | `653a0127...`       |
| `GR17P1/`  | lab `results/20261023-gr17p1-cram`         | `7ff8c411...`       |
| `GR17P2/`  | lab `results/20261022-gr17p2-cram`         | `de2185d3...`       |
| `TMR17/`   | `TMR17_20261021` (local); `lookup/read_tmr17.txt`, bit-level lookup of its shots | `1d6661d5...` |
| `W17/`     | lab `results/20261020-w17-cram`            | `02800d91...`       |

Every figure in RESULT.md traces to the recorded command, raw output,
tool version and SHA-256 in the same directory.
