# 稀疏 Otsu 二值感知权威

本文描述当前 active 实现。历史 V9 局部 Y 边界记录保持不变，仅用于追溯。

## 唯一阈值事实

`SteeringFramePipeline` 每个捕获帧只调用一次 `ComputeSparseOtsuThreshold()`。算法把原图划分为 `80x60` 个等面积单元，读取每个单元中心的 Y，共 4800 个样本。只有直方图能形成两个非空类别时，当前阈值有效；相同最大类间方差选择最小阈值。

`OtsuThresholdTracker` 发布完整状态：`valid`、`threshold`、`source` 和 `stale_frames`。当前帧有效时立即发布 `current`；连续无效的前三帧发布最近有效阈值为 `cached`；第四帧清空为 `none`。配置、感知记忆重置和帧源重建必须清空该缓存。

唯一分类谓词是 `Y > threshold` 为白，`Y <= threshold` 为黑。active 路径不读取 confidence 字段，也没有局部 Y 阈值回退。

## 两个直接 owner

`BEVSparseRowScanner` 把同一 Otsu 状态交给边界行提取器。只有横向索引严格相邻的两个有效 BEV 样本发生黑白转换时才产生 jump；索引缺口或不可采样点会中断邻接。相邻 rising/falling jump 继续组成现有 span，边缘白区不会合成未观测边界。

`BEVImageSegmentConnectivity` 使用同一状态，把 BEV 线段投影到原图并以 supercover 遍历覆盖像素。任一黑像素为 `blocked`，全部可观察像素为白才是 `connected`；阈值无效、投影失败或可见性合同不满足为 `unobservable`。首个候选允许从画外 `(0,0)` 起点裁剪可见后缀，目标必须可见；后续线段要求两端完整可见。

## 下游边界

普通路径、cross、CircleV2、ML path、reference 仲裁和控制继续消费 jump/span、trace、connectivity 与路径事实。它们不计算 Otsu、不解释阈值，也不得恢复另一套分类规则来补偿上游输出。

遥测和 steering media 统一发布 `otsu` 对象。上位机二值视图仅对与快照对齐的 `gray8` 使用该阈值在 host 生成；低位深图像不作为精确二值证据。性能阶段分别记录 `perception.otsu` 和 `bev.connectivity`。
