# 20261023-gr17p1-cram：GR17P1 上板配置位注入，全部完成，各道门都通过

## 基本信息

- **任务单：** `tasks/20261023-gr17p1-cram/TASK.md`，sha256 `0eb15cb3501ddf62473e3871bd24c4dff3943cb957ebe5f128c3675812ac3497`，lab `d350330`。附带的 16 个文件按 `MANIFEST.sha256` 全部核对通过。
- **做法：** 与 `results/20261022-gr17p2-cram/` 相同（点目标 1,200 发 + 对照 60 发，不打 GF-3）。
- **被测构建：** `gr17p1` `d42e7c3`，即 `results/20261023-gr17p1-build/`。`.bit` `7ff8c411…`，`.ebd` `b06c53cd…`，USR_ACCESS `bd341ba8`。
- **机器：**
  - Ubuntu 22.04.5 LTS，内核 6.8.0-136-generic；Python 3.10.12；Vivado 2018.3（SW 2405991）；
  - 器件 XC7K325T-FFG900-2，经 hw_server localhost:3121（Digilent H20026FBDFE8）；
  - 串口 `/dev/ttyACM0`；网口 enp4s0，192.168.1.100。

**时间线（UTC，2026-10-23）：**

| 步骤 | 时间 |
|---|---|
| 环境检查 / setup | 03:40–03:44（`SETUP OK`，抽样池 25,075,790 位） |
| `rmem_max` 调整 | 03:44 前后（作者执行） |
| 第 0b 步 | 03:47–03:52，JTAG 读 USR_ACCESS 随后完成 |
| runner 启动 | 03:54:26 |
| 打前检查 PASS | 03:55:50 |
| 注入 | 03:55:50–08:39:15（4 h 43 m） |
| 打后检查 PASS / DONE | 08:41:09 |
| 落点查询 | 09:28–09:34 |

系统于 2026-10-23 00:09:28 UTC 启动（本次开机，因此 `rmem_max`、hw_server、apt 定时器都是开跑前重新确认/处理的）。运行期间没有断电、没有重启。

## 第 4 节 环境（实际值）

| 项 | 要求 | 实际 |
|---|---|---|
| 1. Python 与依赖 | ≥ 3.9，三个包能导入 | 3.10.12；numpy、scapy、serial 导入正常 |
| 2. 串口、网口、发包权限 | 都在 | `/dev/ttyACM0`（dialout 组）；enp4s0 192.168.1.100；`/usr/bin/python3.10` 带 `cap_net_raw=ep` |
| 3. `net.core.rmem_max` | 运行期间 ≥ 33554432 | **调前 212992（开机默认值），调后 33554432**，`sudo sysctl -w` 只改运行值 |
| 4. Vivado 与 hw_server | 能看到唯一一片 xc7k325t | `/tools/Xilinx/Vivado/2018.3/bin/vivado`；开机后 hw_server 未运行，本次任务重启（2018.3，3121 监听，只看到一片 xc7k325t） |
| 5. 独占，以及运行期间不触发自动更新 | 满足 | 没有别的上板任务。定时器的情况见下 |

**定时器（第 5 项）：** 开跑前 2026-10-23 03:38 UTC（11:38 CST）查询，原样记录如下。

```
$ systemctl list-timers --all 'apt-daily*'
NEXT                        LEFT     LAST                        UNIT
Fri 2026-10-23 17:54:19 CST 6h left  Fri 2026-10-23 05:33:13 CST apt-daily.timer
Sat 2026-10-24 06:27:04 CST 18h left Fri 2026-10-23 06:02:09 CST apt-daily-upgrade.timer
2 timers listed.
```

- 两个定时器在本次开机后已恢复（ACTIVE，有 NEXT），两个服务查询时都是 inactive。
- 注入窗口 03:55:50–08:39:15 UTC = 11:55:50–16:39:15 CST；`apt-daily.timer` NEXT 17:54:19 CST，落在运行窗口之后约 1 小时 15 分；`apt-daily-upgrade.timer` NEXT 次日。两个都不在运行期间触发，按任务单照实记录，没有启动，也没有停止。
- 打完时（16:41 CST）两个定时器仍未触发过。

**检出与数据：**
- ub16 `46285269e15c18276d20168ccb063f80c0e2bf42`，w17 `1e9acb3a2421ed8e33591fa61d37b893d6521579`，两者 `git status --short` 都为空。
- `setup_data.py` 逐项核对后打印 `SETUP OK`：输入帧、`bare_chain_run.py` 冻结值、`.bit` `7ff8c411…`、`.ebd` `b06c53cd…`、`targets_e0.json` `f1d2fc72…`、`targets_control.json` `be1e3ce3…`；抽样池 25,075,790 位。

## 第 0b 步 比特一致检查：PASS（四项全部满足）

- `rc=0`；控制台最后一行是 `PASS: 2 point + 2 GF-3 frame(s) equal the frozen SHA-256`。
- 4 个存盘帧（`check0/frames.sha256`）：
  - 点目标两帧都是 `79ca5c05a217e6e0be684ffd2d8a23a325ce909bb3cf6f40d157f2c510a0056c`；
  - GF-3 两帧都是 `3a794a1ad520ff8f8fca00634e6ca474e9b3dd11480b5d595a0c437ff4948d86`。
- 保护标志：`no protection-flag words in this build`。`identity` 一行的 `USR_ACCESS 00000000` 属于正常（读回里没有身份字）。
- JTAG：`K7_USR_ACCESS bd341ba8`，与构建值一致。
- **重编程重试：0 次。**

## 无故障检查

| 检查 | 结果 | 重编程重试 |
|---|---|---|
| 打前 `cram_pre_GR17P1`（03:55:50Z） | **PASS**，3 点目标 + 3 GF-3 帧等于冻结值 | **0** |
| 打后 `cram_post_GR17P1`（08:41:09Z） | **PASS**，3 + 3 | **0** |

## 注入

**批次**（都在点目标场景上，不打 GF-3）：
- `e0_point`：1,200 发，`targets_e0.json` `f1d2fc72…`；
- `control`：60 发，`targets_control.json` `be1e3ce3…`；
- `--expect-vec-id none`。

**前 10 发的检查：** 实际看到第 18 发。INVALID 0、Traceback 0、equipment stop 0、输入写入失败 0，health 行是 `ok`。通过，继续打。

**整场统计：**
- Traceback 0、equipment stop 0；
- 输入写入失败 0；
- health 行 11 条，全部 `ok`。

### summary.md（原文）

```
pool (population_e0): 25,075,790 bits

| batch | records | valid | identical | changed | below_line | DUE | invalid |
|---|---|---|---|---|---|---|---|
| e0_point | 1200 | 1200 | 1121 | 63 | 7 | 9 | 0 |
| control | 60 | 55 | 55 | 0 | 0 | 0 | 5 |

| e0_point | k/n | share (95 % Wilson) | cross-section, bits (95 %) |
|---|---|---|---|
| below_line | 7/1200 | 0.58 % (0.28-1.20 %) | 146,275 (70,926-300,711) |
| DUE | 9/1200 | 0.75 % (0.40-1.42 %) | 188,068 (99,067-355,889) |
| changed | 63/1200 | 5.25 % (4.12-6.66 %) | 1,316,479 (1,034,332-1,670,244) |

SEM uncorrectable flag after shots (e0_point): []
below-line and hang shots: 16 (shots.csv)
```

**control 的 5 发无效：** shot 6、8、16、21、27，都是 `INVALID_NO_DETECT`，与前几次属于同一类。其余 55 发有效，全部 identical。

### 过线 7 发

| 发 | tile_kind @ 时钟区（tile_x） | word.bit | psnr | 表决器溢出标志 |
|---|---|---|---|---|
| 102 | CLBLM_L @ X1Y3（72） | 31.14 | 39.19 | （无） |
| 456 | DSP_R @ X1Y3（71） | 35.6 | 1.68 | （无） |
| 471 | BRAM_L @ X1Y2（74） | 57.30 | 29.11 | p1 p3 p4 p6 |
| 555 | CLBLM_L @ X1Y2（70） | 96.27 | 35.69 | （无） |
| 675 | CLBLL_L @ X1Y3（68） | 39.7 | 21.76 | p6 |
| 833 | CLBLM_R @ X1Y5（59） | 66.20 | 30.64 | p4 p6 |
| 1159 | CLBLM_R @ X1Y4（79） | 33.0 | 46.99 | （无） |

- 过线判定是 runner 的点目标分级，不是单看总 psnr：shot 1159 的 psnr 为 46.99，但点目标分析报 `line artifact -6.1 dB`、`ghost_db -28.69`、`old_tier T2`，分级 below_line。

### 卡死 9 发

9 发全部是 `RECOVERY_FAIL clear=busy`：设备不再应答，runner 用 JTAG 重新编程恢复。

| 发 | tile_kind @ 时钟区（tile_x） | word.bit | 恢复时刻（CST） |
|---|---|---|---|
| 125 | CLBLM_R @ X1Y3（61） | 80.25 | 12:24:25 |
| 262 | CLBLM_L @ X1Y4（66） | 17.14 | 12:55:02 |
| 299 | CLBLM_L @ X0Y0（34） | 93.17 | 13:03:40 |
| 691 | DSP_R @ X1Y5（65） | 53.20 | 14:30:24 |
| 696 | CLBLM_R @ X1Y3（55） | 71.27 | 14:32:21 |
| 898 | CLBLM_R @ X1Y3（63） | 84.28 | 15:17:24 |
| 1024 | CLBLM_R @ X1Y5（57） | 79.25 | 15:45:37 |
| 1041 | CLBLM_R @ X0Y4（7） | 91.4 | 15:50:10 |
| 1060 | CLBLL_L @ X1Y4（76） | 65.27 | 15:54:53 |

### 注入中的重新编程次数

- `RECOVER: JTAG reprogram` 共 **10** 行，其中 1 行是开打时的首次编程（11:55:50 CST）。
- 所以注入期间重新编程 **9 次**，全部是卡死后恢复（上表 9 发）；
- "纠不了的错"标志后的重编程：**0 次**（`SEM uncorrectable flag after shots: []`）。

## 第 5 节 只读落点查询

- 布线检查点：`/home/xdu/l/gr17p1/chain/build/snap/GR17P1/build/vivado/PROT_GR17P1/PROT_GR17P1.runs/impl_1/k7_udp_ddr_top_routed.dcp`，sha256 `eed7f01c080fb9e9565044c269b9117ba44ca92d66d36cf4c46c04bef34e3f86`，与要求值一致。只读打开，没有修改，也没有存盘。
- 输出：`lookup/lookup.tsv` 共 2,810 行（16 发），`lookup/lookup.log` 末行为 `TILE_LOOKUP done: 16 shots`。

下表按层级粗略统计各发上下两格里的单元和连线（`pN/g_sdf` 指该趟的数据通路，`u_post_compensation`/`u_pre_compensation` 是乘积补偿，`u_swap`/`u_pack`/`u_unpack`/`u_reader`/`u_writer` 是读写打包链，`u_core` 是顶层胶水）。这些是**候选**，不是确定翻转了的那个资源；计算节点内外的归类由本机做（第 8 节）。

| 发 | 格 | 格中单元（前几项） | 连线（前几项） |
|---|---|---|---|
| 102 | CLBLM_L_X72Y165/184 + INT | p6/g_sdf 19、p4/u_pack 2、p4/u_writer 1 | p6/g_sdf 69、p6/u_pack 33、u_swap 24 |
| 125 | CLBLM_R_X61Y160/189 + INT | p6/g_sdf 8、u_swap 7、p1/u_unpack 3 | p6/g_sdf 55、u_core 36、u_swap 17 |
| 262 | CLBLM_L_X66Y208/241 + INT | p3/u_unpack 8、p4/u_post_compensation 6、u_core 3 | u_core 46、p4/u_post_compensation 22、p1/u_post_compensation 20 |
| 299 | CLBLM_L_X34Y46 + INT | p1/g_sdf 14 | p1/g_sdf 83、u_core 4 |
| 456 | INT_R_X71Y167/182 | （无单元） | p6/g_sdf 52、u_swap 26、p6/u_pack 22 |
| 471 | BRAM_L_X74Y120/125 + INT | u_swap 2 | u_swap 61、p6/g_sdf 22、u_core 15 |
| 555 | CLBLM_L_X70Y102/147 + INT | p6/g_sdf 35 | p6/g_sdf 113、u_swap 17、u_core 15 |
| 675 | CLBLL_L_X68Y169/180 + INT | p1/u_pack 14、p4/u_pack 9、u_core 4 | u_core 48、p1/u_post_compensation 37、p6/g_sdf 34 |
| 691 | INT_R_X65Y273/276 | （无单元） | p6/g_sdf 20、p3/u_post_compensation 14、p6/u_pre_compensation 8 |
| 696 | CLBLM_R_X55Y164/185 + INT | p4/g_sdf 11、p6/g_sdf 6、p4/u_writer 2 | p6/g_sdf 90、u_core 24、p4/g_sdf 19 |
| 833 | CLBLM_R_X59Y267/282 + INT | p3/u_post_compensation 16、p6/g_sdf 10、p4/u_post_compensation 2 | p6/g_sdf 53、p3/u_post_compensation 45、p6/u_pre_compensation 15 |
| 898 | CLBLM_R_X63Y158/191 + INT | p6/g_sdf 15、p3/u_reader 5、p1/u_post_compensation 4 | p6/g_sdf 62、u_core 37、u_swap 27 |
| 1024 | CLBLM_R_X57Y260/289 + INT | p4/g_sdf 12、p3/u_post_compensation 9、p3/g_sdf 5 | p3/u_post_compensation 47、p4/g_sdf 44、p3/g_sdf 18 |
| 1041 | CLBLM_R_X7Y204/245 + INT | p4/g_sdf 19、u_core 9 | p4/g_sdf 61、u_core 33、u_seed_engine 20 |
| 1060 | CLBLL_L_X76Y217/232 + INT | p1/u_post_compensation 16、p4/g_sdf 12、p6/g_sdf 9 | p4/g_sdf 45、p6/g_sdf 29、p1/u_post_compensation 23 |
| 1159 | CLBLM_R_X79Y216/233 + INT | p4/g_sdf 9、p1/u_post_compensation 9、p4/u_post_compensation 3 | p4/g_sdf 52、p1/u_post_compensation 46、p4/u_post_compensation 20 |

## 其他

- **数字（只列出，不做判读）：**
  - e0 无变化 1,121/1,200 = 93.4%；
  - 过线 7 发、卡死 9 发，合计 16 发；
  - 对照（此前各场）：GR17 修正版 94.5%、过线 4、卡死 3；GR17B 93.6%、过线 5、卡死 2；GR17M2 92.5%、过线 5、卡死 8。
- **卡死聚簇（照实记录，未作判读）：** 691/696（相隔 2 分钟）、1024/1041/1060（10 分钟内 3 发）时间上相邻；落点各不相同。
- **表决器溢出标志列：** 按 runner 评分原样记录。这个构建读不到标志字，全程是 `no protection-flag words in this build`。
- **输出变化的帧：** 10 个 `.bin.gz`，在 `camp/GR17P1_20261023/e0_point/frames/`（过线 7 发全部，加 changed 的 24、44、58）。
- **板子状态：** 留在 GR17P1 的易失配置；没有动配置闪存；没有向 GF-3 场景注入。
- **工作目录：** `~/l/gr17p1-cram` 留在盘上，按任务单不推送。
- **清理：** Vivado 在结果目录里生成的 `.Xil/`、`hd_visual/`（本次运行产生）已删除，没有推送。

## 文件

- `check0/`：第 0b 步的 json、控制台日志、`usr_access.log`、`frames.sha256`（原始帧不推）。
- `checks/`：打前、打后的 json 和控制台日志。
- `camp/GR17P1_20261023/`：`manifest.json`、`progress.json`、两批的 `injections.jsonl`、`e0_point/frames/*.bin.gz`；另有 `camp/GR17P1_20261023.console.log`。没有单独的 `invalid.jsonl`：control 的 5 发无效记在 `injections.jsonl`（`device.detected=false`）和 `summary.json` 里。
- `summary/`、`lookup/`。
- `steps.log`、`nohup.log`、`COMMANDS.txt`、`SOURCE.txt`、`MANIFEST.sha256`。
