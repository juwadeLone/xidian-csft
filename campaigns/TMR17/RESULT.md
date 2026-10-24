# 20261022-gr17b-cram：GR17B 上板配置位注入，全部完成，各道门都通过

## 基本信息

- **任务单：** `tasks/20261022-gr17b-cram/TASK.md`，sha256 `41634c93849bfa7478b68a595c56104b365b0502fb1fcf64401239c7fb8cb0fa`，lab `3420c7e`。任务单附带的 16 个文件按 `MANIFEST.sha256` 全部核对通过。做法与 `results/20261021-gr17fix-cram/` 相同。
- **被测构建：** `gr17b` `7ae89e3`，即 `results/20261021-gr17b-build/build3_7ae89e3/`。
  - `.bit`：`0b7652dd…`
  - `.ebd`：`17e64177…`
  - USR_ACCESS：`b534019d`
- **机器：**
  - Ubuntu 22.04.5 LTS，内核 6.8.0-136-generic；Python 3.10.12；Vivado 2018.3（SW 2405991）；
  - 器件 XC7K325T-FFG900-2，经 hw_server localhost:3121 访问（Digilent H20026FBDFE8）；
  - 串口 `/dev/ttyACM0`；网口 enp4s0，192.168.1.100。

**时间线（UTC）：**

| 步骤 | 时间 |
|---|---|
| setup / 环境检查 | 2026-10-21 17:0x |
| 定时器停止 | 17:31:34 |
| 第 0b 步 | 17:32:23–17:33:28，JTAG 读 USR_ACCESS 随后完成 |
| runner 启动 | 17:34:10 |
| 打前检查 PASS | 17:35:34 |
| 注入 | 17:35:34–22:14:47（4 h 39 m） |
| 打后检查 PASS / DONE | 22:16:41 |
| 落点查询 | 02:04:02–02:04:44（10-22） |

系统自 2026-10-15 11:03 启动后没有重启，运行期间也没有断电。

## 第 4 节 环境（实际值）

| 项 | 要求 | 实际 |
|---|---|---|
| 1. Python 与依赖 | ≥ 3.9，三个包能导入 | 3.10.12；numpy 2.2.6、scapy 2.7.0、pyserial 3.5，导入正常 |
| 2. 串口、网口、发包权限 | 都在 | `/dev/ttyACM0`（dialout 组）；enp4s0 192.168.1.100；`/usr/bin/python3.10` 带 `cap_net_raw=ep` |
| 3. `net.core.rmem_max` | 运行期间 ≥ 33554432 | **调前 33554432，调后 33554432**（前几次已调到位，本次没有改动） |
| 4. Vivado 与 hw_server | 能看到唯一一片 xc7k325t | `/tools/Xilinx/Vivado/2018.3/bin/vivado`；hw_server 2018.3 在跑（自 10-19 起），只有一片 xc7k325t |
| 5. 独占，以及运行窗口内没有自动更新 | 满足 | 没有别的上板任务（一个 Vivado 2022.1 界面自 10-20 起开着，没有占用 JTAG）。自动更新的处理见下 |

**第 5 项的处理：**
- 开跑前查到 `apt-daily.timer` 的下次触发时间是 02:37:44 CST（18:37 UTC），落在运行窗口内；`apt-daily-upgrade.timer` 是 06:35:42 CST。按第 4 节，停下向用户报告。
- 用户执行了 `sudo systemctl stop apt-daily.timer apt-daily-upgrade.timer`，只停到注入结束，没有改配置。
- 两个定时器在 **2026-10-21 17:31:34 UTC** 变为 inactive，`list-timers` 的 NEXT 显示 n/a。
- 停之前，`apt-daily.service` 和 `apt-daily-upgrade.service` 都是 **inactive**。
- 截至交付时（2026-10-22 02:06 UTC），两个定时器仍是停的，**等用户执行 `sudo systemctl start apt-daily.timer apt-daily-upgrade.timer` 恢复**。
- 注入和打后检查都在 22:16:41 UTC 前结束。

**检出与数据：**
- ub16 `46285269e15c18276d20168ccb063f80c0e2bf42`，w17 `1e9acb3a2421ed8e33591fa61d37b893d6521579`，两者 `git status --short` 都为空。
- `setup_data.py` 打印 `SETUP OK`，抽样池 26,447,347 位。

## 第 0b 步 比特一致检查：PASS（四项全部满足）

- `rc=0`；控制台最后一行是 `PASS: 2 point + 2 GF-3 frame(s) equal the frozen SHA-256`。
- 4 个存盘帧（见 `check0/frames.sha256`）：
  - 点目标两帧都是 `79ca5c05a217e6e0be684ffd2d8a23a325ce909bb3cf6f40d157f2c510a0056c`；
  - GF-3 两帧都是 `3a794a1ad520ff8f8fca00634e6ca474e9b3dd11480b5d595a0c437ff4948d86`。
- 保护标志：`no protection-flag words in this build`。`identity` 一行的 `USR_ACCESS 00000000` 属于正常（读回里没有身份字）。
- JTAG：`K7_USR_ACCESS b534019d`，与构建值一致。
- **重编程重试：0 次。** `grep -c "programming again"` 为 0，JSON `attempts` 只有 1 次，状态 pass。

## 无故障检查

| 检查 | 结果 | 重编程重试 |
|---|---|---|
| 打前 `cram_pre_GR17B`（17:35:34Z） | **PASS**，3 点目标 + 3 GF-3 帧等于冻结值 | **0** |
| 打后 `cram_post_GR17B`（22:16:41Z） | **PASS**，3 + 3 | **0** |

## 注入

**批次**（都在点目标场景上，不打 GF-3）：
- `e0_point`：1,200 个均匀抽样的关键位，`targets_e0.json` `060d5c74…`；
- `control`：60 个非关键位，`targets_control.json` `53aa7863…`；
- 使用 `--expect-vec-id none`。

**前 10 发的检查：** 实际看到第 18 发。INVALID 0、Traceback 0、equipment stop 0、输入写入失败 0，health 行是 `ok`。通过，继续打。

**整场统计：**
- Traceback 0、equipment stop 0；
- 输入写入失败 0（没有 `short read`，也没有 `input write failed`）；
- health 行 3 条，全部 `ok`；
- `STOP SEM uncorrectable` 0 次。

### summary.md（原文）

```
pool (population_e0): 26,447,347 bits

| batch | records | valid | identical | changed | below_line | DUE | invalid |
|---|---|---|---|---|---|---|---|
| e0_point | 1200 | 1200 | 1123 | 70 | 5 | 2 | 0 |
| control | 60 | 55 | 55 | 0 | 0 | 0 | 5 |

| e0_point | k/n | share (95 % Wilson) | cross-section, bits (95 %) |
|---|---|---|---|
| below_line | 5/1200 | 0.42 % (0.18-0.97 %) | 110,197 (47,103-256,985) |
| DUE | 2/1200 | 0.17 % (0.05-0.61 %) | 44,079 (12,091-160,182) |
| changed | 70/1200 | 5.83 % (4.64-7.31 %) | 1,542,762 (1,227,903-1,932,171) |

SEM uncorrectable flag after shots (e0_point): []
below-line and hang shots: 7 (shots.csv)
```

**control 的 5 发无效：** shot 3、19、20、35、54，都是 `INVALID_NO_DETECT`，与 W17、GR17 两次属于同一类（非关键位）。其余 55 发有效，全部 identical。

### 过线 5 发（输出变化，但低于 SDC 线）

| 发 | tile_kind @ 时钟区（tile_x） | word.bit | psnr | 表决器溢出标志 |
|---|---|---|---|---|
| 497 | CLBLL_R @ X0Y2（39） | 7.23 | 26.77 | p3 p4 p6 |
| 777 | CLBLM_R @ X1Y3（69） | 66.30 | 34.02 | p6 |
| 1004 | CLBLM_L @ X0Y4（44） | 5.7 | 49.75 | （无记录） |
| 1141 | INT_INTERFACE_L @ X1Y2（94） | 11.23 | 1.71 | p1 p3 p4 p6 |
| 1145 | CLBLM_L @ X1Y3（72） | 95.29 | 29.67 | （无记录） |

### 卡死 2 发

| 发 | tile_kind @ 时钟区（tile_x） | word.bit | 类别 | 怎么恢复的 |
|---|---|---|---|---|
| 508 | INT_INTERFACE_L @ X1Y1（94） | 77.17 | `RECOVERY_FAIL clear=busy` | 设备不再应答（UART NAK），runner 用 JTAG 重新编程（03:28:29 CST） |
| 1103 | CLBLM_R @ X1Y3（59） | 10.30 | DUE，`clear=cleared` | 清零后即恢复，没有重新编程 |

### 注入中的重新编程次数

- `RECOVER: JTAG reprogram` 共 **2** 行，其中 1 行是开打时的首次编程（01:35:34 CST）。
- 所以注入期间重新编程 **1 次**：
  - 卡死后：**1 次**（shot 508，`RECOVERY_FAIL`）；
  - "纠不了的错"标志后：**0 次**（`STOP SEM uncorrectable` 0 行，SEM 不可纠列表为空）。

## 第 5 节 只读落点查询

- 布线检查点：`/home/xdu/l/gr17/chain/build/snap/GR17B/build/vivado/PROT_GR17B/PROT_GR17B.runs/impl_1/k7_udp_ddr_top_routed.dcp`，sha256 `c77d1dbfa27d2a69cd4bf891148566c48c4b5461b583d350b3669d62b5779034`，与要求值一致。只读打开，没有修改，也没有存盘。
- 输出：`lookup/lookup.tsv` 共 1,630 行（7 发：每发上下两格里的单元 + 互连格里穿过的全部连线），`lookup/lookup.log` 末行为 `TILE_LOOKUP done: 7 shots`。

下表按层级粗略统计各发所在格里的内容。这些是**候选**，不是确定翻转了的那个资源；计算节点内外的归类由本机做（第 8 节）。

| 发 | 上 / 下格 | 格中单元（按层级计数，前几项） | 连线（按层级，前几项） |
|---|---|---|---|
| 497 | CLBLL_R_X39Y103 / X39Y146 | p1/`u_rep` 12、p4/`g_octl` 8、p3/`g_pctl_tmr` 6 | p1/`u_rep` 86、p3/`g_pctl_tmr` 31、`u_axi` 19、p4/`g_octl` 13 |
| 508 | INT_L_X94Y88 / X94Y61 | 无 | 只有 `u_mig`（180 条） |
| 777 | CLBLM_R_X69Y182 / X69Y167 | `u_swap` 5，p3/p4/p6 `u_pack` 各 4，`u_reader` 4 | `u_core` 70、`u_swap` 37、`u_reader` 34、p6/`u_pack` 22 |
| 1004 | CLBLM_L_X44Y202 / X44Y247 | p6/`u_compensator` 18、p3 8、p3/`u_compensator` 5、p3/`g_pctl_tmr` 4 | p6/`u_compensator` 51、p3 32、p3/`u_compensator` 23 |
| 1103 | CLBLM_R_X59Y155 / X59Y194 | p1/`u_rep` 6、`u_reader` 5、p3/`u_pdom` 5 | p3/`u_pdom` 60、p1/`u_rep` 39、p6/`u_reader` 35 |
| 1141 | INT_L_X94Y105 / X94Y144 | 无 | 只有 `u_mig`（161 条） |
| 1145 | CLBLM_L_X72Y197 / X72Y152 | p1/`u_rep` 13、p1/`u_prefetch` 6、p1/`u_writer` 5 | p1/`u_rep` 81、`u_core` 43、p1/`u_writer` 30 |

## 其他

- **数字（只列出，不做判读）：**
  - e0 无变化 1,123/1,200 = 93.6%；
  - 过线 5 发、卡死 2 发，合计 7 发；
  - 对照：GR17 修正版为 94.5%、过线 4、卡死 3；W17 为 80.5%。
- **表决器溢出标志列：** 按 runner 评分原样记录。这个构建读不到标志字，全程是 `no protection-flag words in this build`。
- **输出变化的帧：** 8 个 `.bin.gz`，在 `camp/GR17B_20261022/e0_point/frames/`（过线 5 发全部，加 changed 的 42、50、102）。
- **板子状态：** 留在 GR17B 的易失配置；没有动配置闪存；没有向 GF-3 场景注入。
- **工作目录：** `~/l/gr17b-cram` 留在盘上（输入、黄金帧、靶点，以及第 0b 步的 4 个原始帧），按任务单不推送。
- **清理：** Vivado 在结果目录里生成的 `.Xil/`、`hd_visual/`（本次运行产生）已删除，没有推送。

## 文件

- `check0/`：第 0b 步的 json、控制台日志、`usr_access.log`、`frames.sha256`（32 MB 的原始帧不推）。
- `checks/`：打前、打后的 json 和控制台日志。
- `camp/GR17B_20261022/`：`manifest.json`、`progress.json`、两批的 `injections.jsonl`、`e0_point/frames/*.bin.gz`；另有 `camp/GR17B_20261022.console.log`。没有单独的 `invalid.jsonl`：control 的 5 发无效记在 `injections.jsonl` 和 `summary.json` 里。
- `summary/`、`lookup/`。
- `steps.log`、`nohup.log`、`COMMANDS.txt`、`SOURCE.txt`、`MANIFEST.sha256`。
