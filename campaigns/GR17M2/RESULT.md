# 20261022-gr17m2-cram：GR17M2 上板配置位注入，全部完成，各道门都通过

## 基本信息

- **任务单：** `tasks/20261022-gr17m2-cram/TASK.md`，sha256 `76dd3ec8fe99cd488dbf65056f3e43fa7c33a1b481172117b5150b3bb4c4d680`，lab `96519d0`。附带的 16 个文件按 `MANIFEST.sha256` 全部核对通过。
- **做法：** 与 `results/20261022-gr17b-cram/` 相同。
- **被测构建：** `gr17m2` `6569c42`，即 `results/20261022-gr17m2-build/`。`.bit` `653a0127…`，`.ebd` `706f07cb…`，USR_ACCESS `b534b18b`。
- **机器：**
  - Ubuntu 22.04.5 LTS，内核 6.8.0-136-generic；Python 3.10.12；Vivado 2018.3（SW 2405991）；
  - 器件 XC7K325T-FFG900-2，经 hw_server localhost:3121（Digilent H20026FBDFE8）；
  - 串口 `/dev/ttyACM0`；网口 enp4s0，192.168.1.100。

**时间线（UTC，2026-10-22）：**

| 步骤 | 时间 |
|---|---|
| 环境检查 / setup | 04:07 |
| 第 0b 步 | 04:08:01–04:09:07，JTAG 读 USR_ACCESS 随后完成 |
| runner 启动 | 04:09:32 |
| 打前检查 PASS | 04:10:57 |
| 注入 | 04:10:57–08:55:27（4 h 44 m） |
| 打后检查 PASS / DONE | 08:57:22 |
| 落点查询 | 08:58:09–08:58:47 |

系统自 2026-10-15 11:03 启动后没有重启，运行期间也没有断电。

## 第 4 节 环境（实际值）

| 项 | 要求 | 实际 |
|---|---|---|
| 1. Python 与依赖 | ≥ 3.9，三个包能导入 | 3.10.12；numpy 2.2.6、scapy 2.7.0、pyserial 3.5，导入正常 |
| 2. 串口、网口、发包权限 | 都在 | `/dev/ttyACM0`（dialout 组）；enp4s0 192.168.1.100；`/usr/bin/python3.10` 带 `cap_net_raw=ep` |
| 3. `net.core.rmem_max` | 运行期间 ≥ 33554432 | **调前 33554432，调后 33554432**，没有改动 |
| 4. Vivado 与 hw_server | 能看到唯一一片 xc7k325t | `/tools/Xilinx/Vivado/2018.3/bin/vivado`；hw_server 2018.3 在跑，只有一片 xc7k325t |
| 5. 独占，以及运行期间不触发自动更新 | 满足 | 没有别的上板任务。定时器的情况见下 |

**定时器（第 5 项）：** 开跑前 2026-10-22 04:07:26 UTC 查询，原样记录如下。

```
$ systemctl list-timers apt-daily.timer apt-daily-upgrade.timer
NEXT LEFT LAST PASSED UNIT ACTIVATES

0 timers listed.
Pass --all to see loaded but inactive timers, too.

$ systemctl list-timers --all apt-daily.timer apt-daily-upgrade.timer
NEXT LEFT LAST                        PASSED       UNIT                    ACTIVATES
n/a  n/a  Wed 2026-10-21 06:30:43 CST 1 day 5h ago apt-daily-upgrade.timer apt-daily-upgrade.service
n/a  n/a  Wed 2026-10-21 08:24:33 CST 1 day 3h ago apt-daily.timer         apt-daily.service
```

- 两个定时器都是 inactive，`InactiveEnterTimestamp` 为 2026-10-21 17:31:34 UTC，也就是 GR17B 那次停下后一直没有恢复。
- 两个服务也都是 inactive。
- NEXT 显示 n/a，运行期间没有触发。
- 按任务单原样记下，没有启动，也没有停止。

**检出与数据：**
- ub16 `46285269e15c18276d20168ccb063f80c0e2bf42`，w17 `1e9acb3a2421ed8e33591fa61d37b893d6521579`，两者 `git status --short` 都为空。
- `setup_data.py` 打印 `SETUP OK`，抽样池 20,283,038 位。

## 第 0b 步 比特一致检查：PASS（四项全部满足）

- `rc=0`；控制台最后一行是 `PASS: 2 point + 2 GF-3 frame(s) equal the frozen SHA-256`。
- 4 个存盘帧（`check0/frames.sha256`）：
  - 点目标两帧都是 `79ca5c05a217e6e0be684ffd2d8a23a325ce909bb3cf6f40d157f2c510a0056c`；
  - GF-3 两帧都是 `3a794a1ad520ff8f8fca00634e6ca474e9b3dd11480b5d595a0c437ff4948d86`。
- 保护标志：`no protection-flag words in this build`。`identity` 一行的 `USR_ACCESS 00000000` 属于正常（读回里没有身份字）。
- JTAG：`K7_USR_ACCESS b534b18b`，与构建值一致。
- **重编程重试：0 次。** `attempts` 只有 1 次，状态 pass。

## 无故障检查

| 检查 | 结果 | 重编程重试 |
|---|---|---|
| 打前 `cram_pre_GR17M2`（04:10:57Z） | **PASS**，3 点目标 + 3 GF-3 帧等于冻结值 | **0** |
| 打后 `cram_post_GR17M2`（08:57:22Z） | **PASS**，3 + 3 | **0** |

## 注入

**批次**（都在点目标场景上，不打 GF-3）：
- `e0_point`：1,200 发，`targets_e0.json` `c1664a43…`；
- `control`：60 发，`targets_control.json` `eec1aeff…`；
- 使用 `--expect-vec-id none`。

**前 10 发的检查：** 实际看到第 14 发。INVALID 0、Traceback 0、equipment stop 0、输入写入失败 0，health 行是 `ok`。通过，继续打。

**整场统计：**
- Traceback 0、equipment stop 0；
- 输入写入失败 0；
- health 行 12 条，全部 `ok`。

### summary.md（原文）

```
pool (population_e0): 20,283,038 bits

| batch | records | valid | identical | changed | below_line | DUE | invalid |
|---|---|---|---|---|---|---|---|
| e0_point | 1200 | 1200 | 1110 | 77 | 5 | 8 | 0 |
| control | 60 | 55 | 55 | 0 | 0 | 0 | 5 |

| e0_point | k/n | share (95 % Wilson) | cross-section, bits (95 %) |
|---|---|---|---|
| below_line | 5/1200 | 0.42 % (0.18-0.97 %) | 84,513 (36,124-197,087) |
| DUE | 8/1200 | 0.67 % (0.34-1.31 %) | 135,220 (68,595-265,708) |
| changed | 77/1200 | 6.42 % (5.16-7.95 %) | 1,301,495 (1,047,516-1,611,893) |

SEM uncorrectable flag after shots (e0_point): [274, 530]
below-line and hang shots: 13 (shots.csv)
```

**control 的 5 发无效：** shot 1、12、17、28、29，都是 `INVALID_NO_DETECT`，与前几次属于同一类。其余 55 发有效，全部 identical。

### 过线 5 发

| 发 | tile_kind @ 时钟区（tile_x） | word.bit | psnr | 表决器溢出标志 |
|---|---|---|---|---|
| 344 | CLBLM_R @ X1Y4（77） | 12.6 | 38.26 | p1 p3 p4 p6 |
| 410 | CLBLL_L @ X1Y5（60） | 18.26 | 31.47 | p4 p6 |
| 632 | CLBLM_R @ X1Y4（77） | 2.23 | 17.63 | p1 p3 p4 p6 |
| 711 | CLBLM_L @ X1Y2（72） | 24.25 | 8.20 | （无记录） |
| 1184 | CLBLM_R @ X1Y3（73） | 14.13 | 34.25 | p4 |

### 卡死 8 发

8 发全部是 `RECOVERY_FAIL clear=busy`：设备不再应答，runner 用 JTAG 重新编程恢复。

| 发 | tile_kind @ 时钟区（tile_x） | word.bit | 恢复时刻（CST） |
|---|---|---|---|
| 86 | CLBLM_R @ X1Y2（69） | 6.25 | 12:30:54 |
| 191 | CLBLM_L @ X0Y2（36） | 42.0 | 12:54:36 |
| 192 | CLBLM_R @ X1Y5（53） | 49.21 | 12:55:26 |
| 653 | CLBLM_L @ X1Y2（70） | 57.26 | 14:38:51 |
| 670 | CLBLM_R @ X1Y3（69） | 26.28 | 14:43:25 |
| 703 | CLBLM_L @ X1Y5（66） | 23.9 | 14:51:11 |
| 1117 | CLBLM_L @ X0Y2（36） | 42.13 | 16:22:53 |
| 1153 | CLBLM_L @ X0Y4（48） | 2.11 | 16:31:34 |

### 注入中的重新编程次数

- `RECOVER: JTAG reprogram` 共 **11** 行，其中 1 行是开打时的首次编程（12:10:57 CST）。
- 所以注入期间重新编程 **10 次**：
  - 卡死后：**8 次**（上表 8 发）；
  - "纠不了的错"标志后：**2 次**（shot 274、530 之后各一行 `STOP SEM uncorrectable flag set … (record kept)`，接着重新编程，属 D302 的正常处理）。

## 第 5 节 只读落点查询

- 布线检查点：`/home/xdu/l/gr17m2/chain/build/snap/GR17M2/build/vivado/PROT_GR17M2/PROT_GR17M2.runs/impl_1/k7_udp_ddr_top_routed.dcp`，sha256 `d3cdfdd43384a741a2070bb0df1478302dfd3febbfb6c76d9ec9839cfd396abf`，与要求值一致。只读打开，没有修改，也没有存盘。
- 输出：`lookup/lookup.tsv` 共 2,129 行（13 发），`lookup/lookup.log` 末行为 `TILE_LOOKUP done: 13 shots`。

下表按层级粗略统计各发上下两格里的单元和连线（`g_tail` 指第 9–10 级，单份）。这些是**候选**，不是确定翻转了的那个资源；计算节点内外的归类由本机做（第 8 节）。

| 发 | 上 / 下格 | 格中单元（前几项） | 连线（前几项） |
|---|---|---|---|
| 86 | CLBLM_R_X69Y103 / X69Y146 | p6/`u_reorder` 8、p1/`g_cctl` 6、p6/`u_rep` 2 | p1/`u_reorder` 19、`u_core` 17、p1/`g_tail` 16 |
| 191 | CLBLM_L_X36Y121 / INT_L_X36Y128 | p6/`u_prefetch` 18 | p6/`u_prefetch` 37、p6/`g_sdf_stage` 12、`u_axi` 11 |
| 192 | CLBLM_R_X53Y274 / X53Y275 | p3/`u_pdom` 17、p4 10 | p3/`u_pdom` 61、p4 8、p4/`g_cctl` 3 |
| 344 | CLBLM_R_X77Y206 / X77Y243 | p3/`u_unpack` 13、p4/`u_rep` 12、p3/`u_reader` 10 | p4/`u_rep` 53、p3/`u_unpack` 42、`u_core` 25 |
| 410 | CLBLL_L_X60Y259 / X60Y290 | p3/`u_pdom` 11、p3/`u_rep` 8 | p3/`u_pdom` 34、p3/`u_compensator` 20、p3/`u_rep` 8 |
| 632 | CLBLM_R_X77Y201 / X77Y248 | p4/`u_rep` 9、p3/`u_compensator` 8、p6/`u_unpack` 4 | `u_core` 30、p4/`u_rep` 16、p6/`u_unpack` 11 |
| 653 | CLBLM_L_X70Y128 / X70Y121 | p6/`g_tail` 18、p6/`u_rep` 11 | p6/`g_tail` 43、p6/`u_rep` 41、`u_axi` 14 |
| 670 | CLBLM_R_X69Y163 / X69Y186 | p1/`u_elastic` 16、p3/`u_writer` 5、p4/`u_rep` 4 | p1/`u_elastic` 28、`u_core` 23、p1/`u_writer` 19 |
| 703 | CLBLM_L_X66Y261 / X66Y288 | p3/`g_tail` 18、p3/`u_reorder` 5 | p3/`g_tail` 38、p3/`u_reorder` 21、p3/`u_rep` 19 |
| 711 | CLBLM_L_X72Y112 / X72Y137 | p4/`u_rep` 13、p6/`u_rep` 9、p6/`g_tail` 8 | p4/`u_rep` 36、p6/`g_tail` 32、`u_axi` 30 |
| 1117 | CLBLM_L_X36Y121 / INT_L_X36Y128 | p6/`u_prefetch` 18 | p6/`u_prefetch` 37、p6/`g_sdf_stage` 12、`u_axi` 11 |
| 1153 | CLBLM_L_X48Y201 / X48Y248 | p3/`u_compensator` 18、`u_seed_engine` 13 | p3/`u_compensator` 48、`u_seed_engine` 37 |
| 1184 | CLBLM_R_X73Y157 / X73Y192 | `u_reader` 8、p6/`u_pack` 7、p4/`u_pack` 2 | `u_core` 43、p6/`u_pack` 32、`u_reader` 30 |

## 其他

- **数字（只列出，不做判读）：**
  - e0 无变化 1,110/1,200 = 92.5%；
  - 过线 5 发、卡死 8 发，合计 13 发；
  - 对照：GR17 修正版为 94.5%、过线 4、卡死 3；GR17B 为 93.6%、过线 5、卡死 2。
- **两组相邻的卡死，照实记录，未作判读：**
  - shot 191 与 192 相邻：192 在 191 恢复后约 1 分钟内卡死，两发落点不同。
  - shot 191 与 1117 落在同一个 CLB 列位置（X36，word 42，bit 0 与 bit 13），两格的内容相同，都是 p6 的 `u_prefetch`。
- **表决器溢出标志列：** 按 runner 评分原样记录。这个构建读不到标志字，全程是 `no protection-flag words in this build`。
- **输出变化的帧：** 8 个 `.bin.gz`，在 `camp/GR17M2_20261022/e0_point/frames/`（过线 5 发全部，加 changed 的 7、16、36）。
- **板子状态：** 留在 GR17M2 的易失配置；没有动配置闪存；没有向 GF-3 场景注入。
- **工作目录：** `~/l/gr17m2-cram` 留在盘上，按任务单不推送。
- **清理：** Vivado 在结果目录里生成的 `.Xil/`（本次运行产生）已删除，没有推送。
- **lab 检出：** 本任务开始时 lab 检出在 `96519d0`；运行期间被并行的 20261022-gr17p2-build 任务推进到 `3efc4eb`。本结果只提交本目录。

## 文件

- `check0/`：第 0b 步的 json、控制台日志、`usr_access.log`、`frames.sha256`（原始帧不推）。
- `checks/`：打前、打后的 json 和控制台日志。
- `camp/GR17M2_20261022/`：`manifest.json`、`progress.json`、两批的 `injections.jsonl`、`e0_point/frames/*.bin.gz`；另有 `camp/GR17M2_20261022.console.log`。没有单独的 `invalid.jsonl`：control 的 5 发无效记在 `injections.jsonl` 和 `summary.json` 里。
- `summary/`、`lookup/`。
- `steps.log`、`nohup.log`、`COMMANDS.txt`、`SOURCE.txt`、`MANIFEST.sha256`。
