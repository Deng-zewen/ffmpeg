# HEVC Loop Filter RVV 实现总结

## 📅 实现日期
2026-04-30

## 📋 项目概述

为 FFmpeg 的 HEVC 解码器实现 RISC-V Vector Extension (RVV) 版本的去块效应滤波器（deblocking filter）。

## ✅ 已完成的工作

### 1. Chroma Loop Filter（色度环路滤波器）

#### 实现的函数
- `ff_hevc_v_loop_filter_chroma_8_rvv` - 垂直色度滤波器
- `ff_hevc_h_loop_filter_chroma_8_rvv` - 水平色度滤波器

#### 算法实现
```c
// 核心算法
delta0 = clip((((q0 - p0) << 2) + p1 - q1 + 4) >> 3, -tc, tc)
P0 = clip_pixel(p0 + delta0)  // 如果 !no_p
Q0 = clip_pixel(q0 - delta0)  // 如果 !no_q
```

#### 实现特点
- ✅ 处理 8 像素，分为两组（每组 4 像素）
- ✅ 使用 `tc[0]` 和 `tc[1]` 分别处理两组
- ✅ 完全向量化实现
- ✅ 支持 8-bit 深度
- ✅ 自动跳过零 tc 值

#### 代码位置
- 汇编实现: `libavcodec/riscv/hevcdsp_deblock_rvv.S`
- 初始化: `libavcodec/riscv/hevcdsp_init.c`

### 2. Luma Loop Filter（亮度环路滤波器）

#### 实现的函数
- `ff_hevc_v_loop_filter_luma_8_rvv` - 垂直亮度滤波器（基础版本）
- `ff_hevc_h_loop_filter_luma_8_rvv` - 水平亮度滤波器（框架）

#### 核心组件

##### 2.1 8x8 转置宏
```asm
transpose_8x8_b r0, r1, r2, r3, r4, r5, r6, r7, tmp0-tmp9
```
- 用于垂直滤波器的数据重排
- 参考 `hevcdsp_idct_rvv.S` 中的 `transpose16_8x8`
- 使用 `vslideup`/`vslidedown` + `vmerge` 组合实现

##### 2.2 弱滤波器实现
```c
// delta0 计算
delta0 = (9*(q0-p0) - 3*(q1-p1) + 8) >> 4
delta0 = clip(delta0, -tc, tc)

// 应用滤波
P0 = clip_pixel(p0 + delta0)
Q0 = clip_pixel(q0 - delta0)
```

#### 当前限制
⚠️ **仅实现了弱滤波器的基础版本**
- 缺少强滤波器实现
- 缺少条件判断逻辑（强/弱滤波器选择）
- 仅处理前 4 个像素，后 4 个像素未处理
- 水平滤波器仅为框架，需要完善

## 🔧 技术实现细节

### 数据布局处理

#### 垂直滤波器
```
内存布局（转置前）:
  pix-4*stride → P3[0-7]  (一行 8 像素)
  pix-3*stride → P2[0-7]
  pix-2*stride → P1[0-7]
  pix-1*stride → P0[0-7]
  pix          → Q0[0-7]
  pix+stride   → Q1[0-7]
  pix+2*stride → Q2[0-7]
  pix+3*stride → Q3[0-7]

转置后（处理时）:
  v0 = [P3[0], P3[1], ..., P3[7]]  // 第一列的所有像素
  v1 = [P2[0], P2[1], ..., P2[7]]
  ...
  v7 = [Q3[0], Q3[1], ..., Q3[7]]
```

#### 水平滤波器
```
内存布局（无需转置）:
  每行 = [P3, P2, P1, P0, Q0, Q1, Q2, Q3]
  直接按行处理
```

### RVV 指令使用统计

| 指令类别 | 使用的主要指令 | 用途 |
|---------|--------------|------|
| 数据加载 | `vle8.v`, `vle16.v` | 加载像素数据 |
| 数据存储 | `vse8.v`, `vse16.v` | 存储结果 |
| 类型转换 | `vzext.vf2`, `vnclipu.wi` | 8-bit ↔ 16-bit |
| 算术运算 | `vadd.vv`, `vsub.vv`, `vslli.vx`, `vsrai.vx` | 滤波计算 |
| 数据重排 | `vslideup.vi`, `vslidedown.vi`, `vmerge.vvm` | 转置操作 |
| 比较/裁剪 | `vmax.vx`, `vmin.vx`, `vabs.vv` | Clip 操作 |

## 📊 性能考虑

### 优化点
1. **完全向量化**: 所有循环展开为向量操作
2. **零跳转**: 使用掩码操作避免条件分支
3. **寄存器复用**: 合理分配向量寄存器
4. **数据局部性**: 连续加载/存储

### 已知瓶颈
1. 转置操作需要多条指令（RVV 没有原生转置指令）
2. 16-bit 中间计算需要额外的扩展/收缩指令

## 🚧 待完成的工作

### 高优先级
1. **完善 luma loop filter**
   - [ ] 实现强滤波器算法
   - [ ] 添加条件判断逻辑（beta、tc 检查）
   - [ ] 处理完整的 8 像素（目前仅 4 像素）
   - [ ] 完善 horizontal 滤波器实现

2. **多 bit-depth 支持**
   - [ ] 实现 10-bit 版本
   - [ ] 实现 12-bit 版本
   - [ ] 使用宏参数化处理不同位深

### 中优先级
3. **no_p/no_q 标志处理**
   - [ ] 正确处理跳过滤波的情况
   - [ ] 添加掩码控制

4. **性能优化**
   - [ ] 分析热点和瓶颈
   - [ ] 优化指令调度
   - [ ] 减少寄存器压力

### 低优先级
5. **代码完善**
   - [ ] 添加详细的注释
   - [ ] 代码格式化和对齐
   - [ ] 遵循 FFmpeg 编码规范

## 📝 关键设计决策

### 1. 为什么先实现 Chroma？
- **简单性**: Chroma 滤波器算法更简单（仅弱滤波器）
- **验证框架**: 验证 RVV 基本框架和转置逻辑
- **快速反馈**: 能够更快看到结果

### 2. 为什么使用 16-bit 中间计算？
- **避免溢出**: 滤波计算可能超出 8-bit 范围
- **精度保持**: 保持计算精度
- **标准做法**: 参考 NEON 实现也使用 16-bit

### 3. 转置实现选择
- **不使用 segment load/store**: 兼容性更好
- **参考现有代码**: 复用 `hevcdsp_idct_rvv.S` 的成熟实现
- **指令组合**: `vslideup`/`vslidedown` + `vmerge`

## 🔍 测试计划

### 单元测试
```bash
# 编译
make clean && make

# 运行 checkasm 测试
./tests/checkasm/hevc_deblock

# 使用 fate 测试套件
make fate-hevc
```

### 对比验证
1. 与 C 参考实现对比输出
2. 与 NEON 实现对比性能
3. 边界情况测试

## 📚 参考资料

### FFmpeg 源码
- `libavcodec/hevc/dsp_template.c` - C 参考实现
- `libavcodec/aarch64/hevcdsp_deblock_neon.S` - NEON 实现
- `libavcodec/riscv/hevcdsp_idct_rvv.S` - RVV IDCT 实现（转置宏参考）

### 算法文档
- ITU-T H.265 规范 - Deblocking filter 章节
- JVET 文档

## 📈 实现进度

```
Chroma Loop Filter
  ├─ 垂直滤波器  ✅ 100%
  ├─ 水平滤波器  ✅ 100%
  └─ 8-bit 支持  ✅ 100%

Luma Loop Filter
  ├─ 转置宏      ✅ 100%
  ├─ 弱滤波器    ⚠️  50%  (仅基础实现)
  ├─ 强滤波器    ❌   0%
  ├─ 条件判断    ❌   0%
  ├─ 垂直滤波器  ⚠️  40%
  └─ 水平滤波器  ⚠️  20%

Bit-Depth 支持
  ├─ 8-bit       ✅ 100%
  ├─ 10-bit      ❌   0%
  └─ 12-bit      ❌   0%

测试与验证
  ├─ 编译通过    ❓ 待测
  ├─ 功能正确性  ❓ 待测
  └─ 性能测试    ❓ 待测
```

## 🎯 下一步计划

### 立即行动
1. 测试当前的 chroma 和基础 luma 实现
2. 修复编译错误（如果有）
3. 完善 luma 弱滤波器（处理全部 8 像素）

### 短期目标（1-2 周）
1. 实现完整的 luma loop filter（包括强滤波器）
2. 添加条件判断逻辑
3. 实现 10/12-bit 支持

### 长期目标（1 个月）
1. 性能优化和调优
2. 完整的测试覆盖
3. 提交到 FFmpeg 主线

## 💭 经验教训

### 技术挑战
1. **转置实现**: RVV 没有原生转置指令，需要多条指令组合
2. **条件分支**: 需要用掩码操作完全替代分支
3. **寄存器分配**: luma 滤波器需要大量寄存器

### 最佳实践
1. **渐进式开发**: 从简单到复杂
2. **参考成熟实现**: NEON 和 IDCT RVV 代码
3. **保持代码清晰**: 良好的注释和结构

---

**文档维护者**: Claude Sonnet 4.6
**最后更新**: 2026-04-30
**版本**: 1.0
