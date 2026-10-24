# 20261022-gr17p2-cram：GR17P2 上板配置位注入，全部完成，各道门都通过

## 基本信息

- **任务单：** `tasks/20261022-gr17p2-cram/TASK.md`，sha256 `8e7cb4ef22585aa5e2ffdbbd724146d0bc6659b7f925c70408852c7ddf02509c`，lab `c347167`。附带的 16 个文件按 `MANIFEST.sha256` 全部核对通过。
- **做法：** 与 `results/20261022-gr17m2-cram/` 相同（同一份 runner，仅工程名换 GR17P2）。
- **被测构建：** `gr17p2` `615f37d`，即 `results/20261022-gr17p2-build/`。`.bit` `de2185d3…`，`.ebd` `fb2c14bb…`，USR_ACCESS `b534d034`，PROT_VEC_ID `4befc553`。
- **靶点：** 任务单附带，`targets_e0.json` `a8bb9ea3…`（1,200 发均匀抽样）、`targets_control.json` `18d6741c…`（60 发非关键位）；抽样池 27,481,825 位（setup 实测一致）。
- **机器：**
  - Ubuntu 22.04.5 LTS，内核 6.8.0-136-generic；Python 3.10.12；Vivado 2018.3（SW 2405991）；
  - 器件 XC7K325T-FFG900-2，经 hw_server localhost:3121（Digilent H20026FBDFE8）；
  - 串口 `/dev/ttyACM0`；网口 enp4s0，192.168.1.100。

**时间线（UTC，2026-10-22）：**

| 步骤 | 时间 |
|---|---|
| 环境检查 / setup | 09:47–09:48 |
| 第 0b 步 | 09:48:29–09:49:10（含 JTAG 读 USR_ACCESS） |
| runner 启动 | 09:49:25 |
| 打前检查 PASS | 09:50:49（3 point + 3 GF-3，0 重试） |
| 注入 | 09:50:49–14:36:55（4 h 46 m，含 control） |
| 打后检查 PASS / DONE | 14:38:49（3 + 3，0 重试） |
| 落点查询 | 10-23 00:14–00:15 |

系统自 2026-10-15 11:03 启动后没有重启，运行期间也没有断电。

## 第 4 节 环境（实际值）

| 项 | 要求 | 实际 |
|---|---|---|
| 1. Python 与依赖 | ≥ 3.9，三个包能导入 | 3.10.12；numpy/scapy/pyserial 导入正常 |
| 2. 串口、网口、发包权限 | 都在 | `/dev/ttyACM0`（dialout 组）；enp4s0 192.168.1.100；`/usr/bin/python3.10` 带 `cap_net_raw=ep` |
| 3. `net.core.rmem_max` | 运行期间 ≥ 33554432 | **调前 33554432，调后 33554432**，没有改动（上次运行遗留的临时值仍在生效） |
| 4. Vivado 与 hw_server | 能看到唯一一片 xc7k325t | `/tools/Xilinx/Vivado/2018.3/bin/vivado`；hw_server 2018.3 在跑（PID 2027098），板上唯一一片 xc7k325t（第 0b 步 JTAG 读 USR_ACCESS 成功即为实证） |
| 5. 独占，以及运行期间不触发自动更新 | 满足 | 没有别的上板任务。定时器情况见下 |

**定时器（第 5 项）：** 开跑前查询，原样记录如下。

```
$ systemctl list-timers apt-daily.timer apt-daily-upgrade.timer
NEXT LEFT LAST PASSED UNIT ACTIVATES

0 timers listed.
Pass --all to see loaded but inactive timers, too.

$ systemctl list-timers --all apt-daily.timer apt-daily-upgrade.timer
NEXT LEFT LAST                        PASSED        UNIT                    ACTIVATES
n/a  n/a  Wed 2026-10-21 06:30:43 CST 1 day 11h ago apt-daily-upgrade.timer apt-daily-upgrade.service
n/a  n/a  Wed 2026-10-21 08:24:33 CST 1 day 9h ago  apt-daily.timer         apt-daily.service
```

- 两个定时器均 inactive（自 2026-10-21 17:31:34 UTC 停起后未恢复），NEXT=n/a，运行期间没有触发。
- 按任务单原样记下：没有启动，也没有停止。

**检出与数据：**
- ub16 `46285269e15c18276d20168ccb063f80c0e2bf42`，w17 `1e9acb3a2421ed8e33591fa61d37b893d6521579`，两者 `git status --short` 都为空（复用 `~/l/w17-cram/` 下检出，提交号核对一致）。
- `setup_data.py` 打印 `SETUP OK`；抽样池 27,481,825 位。

## 第 0b 步 比特一致检查：PASS（四项全部满足）

- `rc=0`；控制台最后一行是 `PASS: 2 point + 2 GF-3 frame(s) equal the frozen SHA-256`。
- 4 个存盘帧（`check0/frames.sha256`）：
  - 点目标两帧都是 `79ca5c05a217e6e0be684ffd2d8a23a325ce909bb3cf6f40d157f2c510a0056c`；
  - GF-3 两帧都是 `3a794a1ad520ff8f8fca00634e6ca474e9b3dd11480b5d595a0c437ff4948d86`。
- 保护标志：`no protection-flag words in this build`。`identity` 一行的 `USR_ACCESS 00000000` 属于正常（读回里没有身份字）。
- JTAG：`K7_USR_ACCESS b534d034`，与构建值一致。
- **重编程重试：0 次**（`programming again` 0 次）。

## 无故障检查

| 检查 | 结果 | 重编程重试 |
|---|---|---|
| 打前 `cram_pre_GR17P2`（09:50:49Z） | **PASS**，3 点目标 + 3 GF-3 帧等于冻结值 | **0** |
| 打后 `cram_post_GR17P2`（14:38:49Z） | **PASS**，3 + 3 | **0** |

## 注入

**批次**（都在点目标场景上，不打 GF-3）：
- `e0_point`：1,200 发，`targets_e0.json` `a8bb9ea3…`；
- `control`：60 发，`targets_control.json` `18d6741c…`；
- 使用 `--expect-vec-id none`。

**前 10 发的检查：** 0 INVALID、0 Traceback、0 equipment stop、0 输入写入失败，health 行是 `ok`。通过，继续打。

**整场统计：**
- Traceback 0、equipment stop 0；
- 输入写入失败 0；
- health 行 12 条，全部 `ok`；
- 控制台 `programming again` 0 次。

### summary.md（原文）

```
pool (population_e0): 27,481,825 bits

| batch | records | valid | identical | changed | below_line | DUE | invalid |
|---|---|---|---|---|---|---|---|
| e0_point | 1200 | 1200 | 1121 | 62 | 6 | 11 | 0 |
| control | 60 | 53 | 53 | 0 | 0 | 0 | 7 |

| e0_point | k/n | share (95 % Wilson) | cross-section, bits (95 %) |
|---|---|---|---|
| below_line | 6/1200 | 0.50 % (0.23-1.09 %) | 137,409 (63,029-298,610) |
| DUE | 11/1200 | 0.92 % (0.51-1.63 %) | 251,917 (140,874-449,049) |
| changed | 62/1200 | 5.17 % (4.05-6.57 %) | 1,419,894 (1,113,331-1,805,093) |

SEM uncorrectable flag after shots (e0_point): [42, 106]
below-line and hang shots: 17 (shots.csv)
```

**control 的 7 发无效：** shot 4、8、16、19、35、36、40，都是 `INVALID_NO_DETECT`（非关键位 SEM 未检出，与前几次同类）。其余 53 发有效，全部 identical。

### 过线 6 发

| 发 | tile_kind @ 时钟区（tile_x） | word.bit | psnr | 表决器溢出标志 |
|---|---|---|---|---|
| 444 | BRAM_L @ X1Y3（62） | 5.0 | 24.06 | p6 |
| 454 | CLBLM_R @ X1Y3（59） | 36.25 | 33.51 | p4 p6 |
| 487 | CLBLM_R @ X1Y5（77） | 69.15 | 40.88 | （无） |
| 937 | BRAM_R @ X0Y3（17） | 19.2 | 43.94 | p3 p4 p6 |
| 1005 | CLBLM_L @ X0Y4（8） | 50.14 | 31.54 | p3 p4 |
| 1180 | CLBLM_L @ X1Y3（72） | 5.4 | 36.71 | p1 p3 p4 |

输出帧已存 `camp/GR17P2_20261022/e0_point/frames/i00*_point_below_line.bin.gz`（另有 3 个 changed 帧存盘）。

### 卡死 11 发

| 发 | tile_kind @ 时钟区（tile_x） | word.bit | 结局 | 恢复时刻（CST） |
|---|---|---|---|---|
| 122 | CLBLM_L @ X1Y3（72） | 35.23 | RECOVERY_FAIL → JTAG | 18:19:45 |
| 216 | CLBLM_R @ X1Y2（57） | 47.6 | RECOVERY_FAIL → JTAG | 18:41:10 |
| 283 | CLBLL_L @ X1Y3（68） | 13.3 | RECOVERY_FAIL → JTAG | 18:56:44 |
| 300 | CLBLM_R @ X1Y3（73） | 57.1 | clear=cleared，自清 | — |
| 363 | CLBLM_L @ X1Y2（64） | 84.19 | RECOVERY_FAIL → JTAG | 19:14:58 |
| 404 | CLBLL_L @ X1Y2（68） | 85.11 | clear=cleared，自清 | — |
| 491 | CLBLL_L @ X1Y2（68） | 96.16 | clear=cleared，自清 | — |
| 737 | CLBLM_L @ X1Y5（72） | 50.14 | RECOVERY_FAIL → JTAG | 20:38:59 |
| 938 | CLBLL_L @ X1Y2（60） | 83.7 | RECOVERY_FAIL → JTAG | 21:24:08 |
| 979 | CLBLM_R @ X1Y4（79） | 94.22 | RECOVERY_FAIL → JTAG | 21:33:53 |
| 1055 | CLBLM_L @ X1Y3（72） | 34.20 | RECOVERY_FAIL → JTAG | 21:51:28 |

8 发 `RECOVERY_FAIL clear=busy`：设备不再应答，runner 用 JTAG 重新编程恢复。3 发 `clear=cleared`：超时的状态位随后自行清掉，未触发重编程。

### 注入中的重新编程次数

- `RECOVER: JTAG reprogram` 共 **11** 行，其中 1 行是开打时的首次编程（17:50:50 CST）。
- 所以注入期间重新编程 **10 次**：
  - 卡死后：**8 次**（上表 8 发 RECOVERY_FAIL）；
  - "纠不了的错"标志后：**2 次**（shot 42、106 之后各一行 `STOP SEM uncorrectable flag set … (record kept)`，接着重新编程，属 D302 的正常处理）。

## 第 5 节 只读落点查询

- 布线检查点：`/home/xdu/l/gr17p2/chain/build/snap/GR17P2/build/vivado/PROT_GR17P2/PROT_GR17P2.runs/impl_1/k7_udp_ddr_top_routed.dcp`，sha256 `af9011fd3d6786a768cc2e14ed5e6f0239fe8e5459dc2546395363f9623a7b40`，与任务单要求值一致。只读打开，没有修改，也没有存盘。
- 输出：`lookup/lookup.tsv` 共 3,044 行（17 发），`lookup/lookup.log` 末行为 `TILE_LOOKUP done: 17 shots`。

## 其他异常

无。control 批的 7 发 `INVALID_NO_DETECT` 是预期内的非关键位未检出，不算异常。
