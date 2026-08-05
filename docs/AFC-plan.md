# Sample-Clock Timing Loop：问题分析与 AFC 改造计划

> 状态：**问题未完全解决**。本文记录 545 长时间解码中反复出现的周期性解调崩溃的完整调查过程、根因分析、已尝试修复及其局限，以及下一步 AFC（Automatic Frequency Control）式闭环的设计方向。

## 1. 背景与现象

### 1.1 测试素材

- `545M-DVB-T-公視-3.cs16`：363.9 GB，10 Msps CS16，`airspy_rx` 录制，约 2.5 小时真实 DVB-T 信号（含真实时钟漂移、动态多径、fading）。
- `--decode-iq` 直接回放，`--decoder-threads 16`。
- 事件调试：`AIRSPYTV_EVENT_DEBUG=1`，stderr 输出 `[evt]` 行。

### 1.2 用户观察

- DVB-T 成功解码后，过几分钟 TS 串流突然没掉，CPU 使用率掉得很低（解码线程停摆）。
- TS buffer（mpv 播放 queue）水位开始下降，最后 run-out，mpv 无反应。
- **`tau`（timing 测值）爬到 ~171 左右时就炸掉**——只是把崩溃推迟了，没有根治。
- Viterbi / RS FEC 在崩溃时都还正常，BER 在合理范围。

### 1.3 日志证据（fi2.log，5 次 badlock）

```
[evt] badlock enter fi=0.830 off=0 sym=187887 discont=138 timing=28.85 hopeless=4
[evt] badlock enter fi=0.903 off=0 sym=189995 discont=436 timing=79.65 hopeless=4
[evt] badlock enter fi=0.819 off=0 sym=190742 discont=439 timing=15.77 hopeless=4
[evt] badlock enter fi=0.706 off=0 sym=312259 discont=615 timing=115.75 hopeless=4
[evt] badlock enter fi=0.796 off=0 sym=314503 discont=840 timing=-13.92 hopeless=4
```

timing 环路失控的关键序列（tloop，文件尾部）：

```
[evt] tloop tau=170.66 shift=46.0 drift=0.804 smooth=0.358 frac=0.46
[evt] tloop tau=171.11 shift=46.0 drift=0.449 smooth=0.359 frac=0.82
[evt] tloop tau=170.41 shift=47.0 drift=-0.700 smooth=0.360 frac=0.18
[evt] tloop tau=-163.89 shift=47.0 drift=-334.307 smooth=0.026 frac=0.20   ← 测值翻转！
[evt] tloop tau=-29.16  shift=47.0 drift=134.738 smooth=-0.447 frac=-0.24   ← 环路失控
[evt] tloop tau=10.90   shift=47.0 drift=40.052 smooth=-0.985 frac=-1.23
...
[evt] tloop tau=35.94   shift=-29.0 drift=4.494 smooth=-5.255 frac=-4.00   ← clamp 撞底，窗口乱移
```

## 2. 三层问题

调查发现"545 周期崩溃"实际是**三个独立问题**叠加，前两个与环路有关，第三个是信道现象：

### 2.1 时钟漂移本体（✅ 已解决）

- 545 的录制时钟漂移约 **0.36 样本/窗口**（1 窗口 = 68 符号 ≈ 0.1 s）。
- pilot phase-slope（相邻 scattered pilot 的相位差）每符号测一次 tau，窗口平均。
- 只跟踪**窗口间差分**（慢漂移），不追绝对值——绝对值被多径 group delay 主导（545 上约 +128 样本恒偏），追绝对值等于追信道。
- 补偿：`fractional_timing` 累积，跨 ±0.5 时对符号周期 ±1 样本（`advance_symbol`），clamp ±4。
- **现状**：差分测量 + 补偿基本闭合（tloop 显示 `tau − shift ≈ 常数` 时窗口追上了漂移）。

### 2.2 环路回归被离群点击垮（🔧 已修复，但只验证了 300 s）

- **现象**：`tau=171.11 → -163.89` 单窗口翻转，`smooth` 从 0.36 崩到 0.026，随后 `frac` 撞 clamp、窗口在约 1 秒内移动上百样本。
- **根因**：这不是 arg 的 π 包装（1.57 rad 离 π 还远），而是 **545 动态多径的 group-delay 突变**——某个窗口内多径相位变化把相邻 pilot 相位差推过了 ±π，测值瞬间跳 ~335 样本。**最小二乘回归对 24 点历史里的单个离群点毫无抵抗力**：一个离群点把斜率拉偏 ~100/24 ≈ 4，`smooth` 被拖成负值，环路失去补偿方向。
- **修复**：把最小二乘回归换成**中位数连续差**（对每窗口的相邻 tau 差取中位数，1-2 个离群窗口无法移动中位数），并对 `smoothed_timing_drift` 加 ±4 clamp 双保险。
- **验证**：545 跑 300 s（约 5 分钟，覆盖 1+ 个崩溃周期）零 badlock / 零 hopeless / phase-jump 1 次，tau 平滑爬过 175 不翻。
- **局限**：300 s 只覆盖约 1 个崩溃周期，**不足以证明长期稳定**。用户更长实测仍观察到"差不多的问题"。

### 2.3 verify 瞬时 ramp 估计的 π 边界（❌ 未解决）

- `lock_phase_at_offset` 的去旋转 ramp 是**每符号用相邻 pilot 相位差瞬时估计**的。
- 该估计的模是 **N/24 = 341 样本**（12 载波间距对应 π）：`tau` 爬过 341 后 arg 必然包装，估计失效 → verify 选错 phase → MER 崩。
- tau 绝对值无界爬升（每 ~8 分钟约 341 样本），所以**最终必然撞上这个边界**——中位数回归只是让环路在撞边界前不失控，撞上后仍要靠 hopeless → re-anchor 兜底（丢 ~0.4 s）。
- 更糟：**多径 group-delay 突变也会让瞬时估计翻 π**（见 2.2），这是 badlock 段 phase-jump 风暴（841 次，集中在崩溃段）的直接来源之一。

### 2.4 badlock = 真实信道退化（fading / 动态多径）（⚠️ 期望行为，非 bug）

- badlock 时 `fi=0.83/0.90/0.82/0.71/0.80`——**continual-carrier 相关本身下降**，说明信号真的在退化（fading / ISI），不是环路假象。
- hopeless 机制（窗口 MER < floor 连续 4 窗口 → 强制 re-acquisition）按设计工作：re-anchor score 0.99，恢复后 MER 回 20 dB。
- 这是**深衰落时的正确行为**（丢 ~0.4 s 恢复），不应消除，只能缩短恢复时间。

## 3. 为什么短时验证无效

崩溃周期 ~4.6 分钟（badlock 段 sym≈187K 与 312K，间隔约 186 K 符号 ≈ 4.6 分钟）。300 s 测试恰好在崩溃周期边缘，**即使过了也不代表长期稳定**。验证必须满足：

- 覆盖**至少 3 个完整崩溃周期**（≥ 15 分钟墙钟 / 或直接跑全文件 2.5 小时）。
- 关键观测点：**tau 爬过 341**（arg π 边界）时 verify 是否崩；**GD 突变窗口**出现时环路是否受扰。
- 区分两类事件：**环路自身崩溃**（应归零）vs **信道退化恢复**（fading 时的 re-anchor，允许存在，但恢复应 < 1 s）。

## 4. 当前架构（as of 1cb5a50 之后）

```text
pilot phase-slope（每符号）
        │ timing_acc（窗口平均）
        ▼
   tau 测值（含 group-delay 偏置 ~+128）
        │ rebase CIR slide
        ▼
   drift = tau − tau_prev          ← 差分，只留慢漂移
        │ tau_history[24]
        ▼
   中位数连续差 → smoothed_drift   ← 当前修复点（抗离群）
        │ clamp ±4
        ▼
   fractional_timing（±0.5 阈值）
        ▼
   advance_symbol：next_symbol_start ± 1（bang-bang 步进）
```

执行器是**积分器 + 阈值步进**（bang-bang），不是连续比例反馈。tau 绝对值无界爬升（窗口只追漂移、不归零），这是 2.3 的根。

## 5. 设计方向（AFC 式闭环）

目标是把"只追踪、不修正"改成真正的闭环：误差检测 → 环路滤波 → 反馈到执行器 → 误差归零。

| 层次 | 现状 | AFC 式目标 |
|---|---|---|
| 漂移闭环 | 差分测量 + 中位数回归（已修） | 保持差分（group delay bias 免疫），执行器改比例反馈 |
| 位置归零 | 故意不追绝对值（bias 风险） | 不归零到 0，而是把窗口移到 **CIR 能量重心**（多径质心） |
| 亚样本 | 整数步进（±1 sample） | 可选：polyphase 小数延迟，窗口连续移动 |

具体候选改造（按优先级）：

1. **verify 共享环路 tau（模 N 去旋转）**——消除 2.3 的 π 边界。verify 不再每符号瞬时估计 ramp，改用环路平滑后的 `tau mod N`（`N = fft_size`）做去旋转。环路 tau 平滑、无瞬时噪声，且模 N 后无包装问题。**注意**：verify 的 phase 区分对 tau 误差敏感（dephase = 2π·k·τ/N，k 最大 6816，τ 误差 1 样本就错 5.2 rad），所以环路 tau 必须足够平滑，且 GD 突变时也不能引入大误差——可能需要**多符号中位数**的 ramp 估计。
2. **tau unwrap**——维护无界累积的展开值，避免测值在 ±341 处翻转；回归/差分全部在展开域做。GD 突变跳变仍需 2 的方案（中位数/限幅）过滤。
3. **执行器比例化**——`next_symbol_start` 直接按 `W += K·drift` 连续调整（K≈2.6 抵消 0.38 的步进衰减，见历史测量），替代"累积到 ±0.5 才跳"的 bang-bang。锯齿消失、无 clamp。
4. **CIR 重心定位**——窗口放在 guard 内多径能量质心（ROADMAP 已有 adaptive window 条目），解决 group-delay 偏置 + 深衰落时窗口位置不佳的问题。注意与 timing 环路解耦（CIR slide 需要 rebase 测量基准，历史教训：rebase 系数 1.0 过补偿、实测 0.38）。
5. **群延迟偏置估计**——把恒定 GD（~+128）从 tau 测值中分离（长时间平均），让测值反映纯窗口偏移，绝对值才有意义。

## 6. 已验证的事实（供设计参考）

- 漂移率：0.36 样本/窗口（545）；0.5 ppm TCXO @ 48/7 MSPS ≈ 3.4 样本/秒 ≈ 0.005 样本/符号。
- 步进响应：`next_symbol_start` 移动 1 样本，tau 测值只降 **0.38**（多径衰减了 phase-slope 灵敏度）——所以 rebase 系数不能用 1.0。
- 窗口滑动 d 样本，timing 测值移动 **−d**（需 rebase 到窗口平均位置）。
- arg 包装：相邻 pilot（间距 12）相位差 = 2π·12·τ/N，包装在 τ = N/24 = 341 样本（8K 模式）。
- group delay 偏置：545 上约 +128 样本恒偏（tau 测值 = 窗口偏移 + GD）。
- verify 在 tau 172（150 s 测试）下工作正常；phase-jump 只在信道退化段（fi < 0.9）密集出现。

## 7. 验收标准

- 545 全文件（2.5 小时）离线解码：**环路相关崩溃归零**（无 `tau 翻转`、无 smooth 失控、无 clamp 撞击）。
- tau 爬过 341 时不触发 verify 崩溃（2.3 修复后）。
- fading 段 re-anchor 恢复 < 1 s，恢复后 MER 回正常值。
- 581 / 557 回归不劣化（byte 级或 sync 率 100%）。
- ctest 全绿。
