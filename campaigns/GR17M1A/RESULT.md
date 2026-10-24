# 20261023-gr17m1a-lookup：GR17M1A 本机注入 18 发的格子查询（只读，全完成）

- **任务单：** `tasks/20261023-gr17m1a-lookup/TASK.md`（lab `a664c13`），3 个附带文件按 `MANIFEST.sha256` 全部核对通过；`tile_lookup.tcl` 与各场注入第 5 节用的逐字节相同（`ef16526f…`）。
- **被测构建：** `gr17m1a` `b868c86`，即 `results/20261023-gr17m1a-build/`；比特流 `49a8bfab…`。清单是本机 `GR17M1A_20261023` 注入的汇总 shots.csv（CRLF→LF，内容不变）。
- **机器 / 工具：** Vivado 2018.3（SW 2405991），`-mode batch`。只读打开，没有 `write_checkpoint`，没有改动或存盘。不用板子，不开 hw_server。
- **时间（UTC，2026-10-23）：** 10:02–10:07。

## 检查点

`/home/xdu/l/gr17m1a/chain/build/snap/GR17M1A/build/vivado/PROT_GR17M1A/PROT_GR17M1A.runs/impl_1/k7_udp_ddr_top_routed.dcp`

- 实测 SHA-256：`5275468cfdfb369b8f3d2900a39661d684d125106e947d89b279a0424ad83fec`，与要求值一致（也与 `results/20261023-gr17m1a-build/checks/routed_dcp.sha256` 相同）。

## 查询结果

- `lookup.log` 最后一行：`TILE_LOOKUP done: 18 shots`（要求 18 发），`open_checkpoint` 成功，无 ERROR。
- `lookup.tsv`：3,163 数据行（含表头 1 行），18 发各自的上、下两格。

## 其他

- Vivado 在结果目录里生成的 `.Xil/`、`hd_visual/`（本次运行产生）已删除，没有推送。
- 与 `results/20261023-above-line-lookup/`、`results/20261023-final-bits/` 同窗口并行，都只读。

## 文件

`lookup.tsv`、`lookup.log`、`COMMANDS.txt`、`SOURCE.txt`、`RESULT.md`、`MANIFEST.sha256`。
