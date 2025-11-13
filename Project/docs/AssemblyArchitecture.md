# Assembly 模块架构梳理与优化

## 1. 总览
`CAssemblyWeld` 是装配焊接应用的顶层调度者：
- **UI/交互**：继承自 `CDialog`，负责主面板、Tab 页等界面事件。
- **控制单元 (`CUnit`)**：每个控制单元封装了机器人、相机、外轴、IO 等硬件对象，是焊接业务的最小执行单元。
- **运行时资源容器**：`m_vpRobotDriver`、`m_vtRobotThread`、`m_vpLaserLineScreen`、`m_vpScanInit`、`m_vpShowLaserImgBuff` 等向量保存了对硬件驱动、扫描线程与图像缓冲的引用，支撑扫描、焊接以及状态回显。
- **业务线程**：如扫描、焊接、回零等 `Thread*` 方法负责不同的流程执行，均依赖于上面的运行时资源。

整体结构如下：

```
CAssemblyWeld (UI + 流程)
├─ LoadControlUnitInfos()  —— 配置解析
├─ CUnit (硬件聚合)
│  ├─ CRobotDriverAdaptor
│  ├─ 摄像机/外轴驱动
│  └─ 轨迹/焊接数据缓存
├─ CLaserLineScreen/CScanInitModule —— 视觉&扫描工具
└─ 业务线程 —— 焊接 / 扫描 / 回零 / 教导
```

## 2. 初始化流程
新版初始化由四个步骤构成，对应 `InitAllUnit()` 中的 4 个子流程：

1. **配置解析 (`LoadControlUnitInfos`)**
   - 从 `CONTRAL_UNIT_INFO_INI` 读取控制单元数量、名称、类型等信息，并确保数据目录存在。
2. **运行时清理 (`ResetUnitRuntimeState`)**
   - 释放旧的机器人线程、扫描模块、激光屏以及显示缓冲，避免重复初始化导致的资源泄露。
3. **控制单元生命周期 (`DestroyUnits`)**
   - 删除上一次创建的 `CUnit` 实例，让其内部（`CContralUnit`）负责释放机器人、相机等底层驱动，彻底回收硬件对象。
4. **实例化与绑定 (`InitializeUnitRuntimeArtifacts`)**
   - 为每个 `CUnit` 设置机器人编号、补齐线程、激光屏、扫描初始化模块、图像缓冲等运行时资源，同时依据调试开关控制伺服上电。

上述流程确保 `InitAllUnit()` 在任何时候都能重复执行，并保持资源状态一致。

## 3. 架构优化要点
- **职责拆分**：新增的 `LoadControlUnitInfos`、`ResetUnitRuntimeState`、`DestroyUnits`、`InitializeUnitRuntimeArtifacts` 将原先 200+ 行的 `InitAllUnit` 拆分成独立职责，代码更易于维护与复用。
- **生命周期安全**：通过 `DestroyUnits` 统一释放 `CUnit`，不再在析构函数中手动逐个释放 `CRobotDriverAdaptor`，避免重复释放/遗留资源的问题。
- **可重复初始化**：清理步骤保证再次进入装配流程时不会残留旧线程或图像缓冲，从而提升系统长时间运行的稳定性。
- **文档化架构**：本文档总结了关键结构与初始化时序，方便后续团队成员快速理解装配模块的依赖关系。

## 4. 后续建议
- 根据不同 `nContralUnitType` 的分支补充专属的初始化策略，避免空 `case`。
- 将扫描/焊接线程统一注册到一个生命周期管理器，以进一步降低 `CAssemblyWeld` 的复杂度。
- 在新的 helper 中加入日志，帮助定位配置解析或硬件初始化失败的具体原因。
