# RVV IDCT 优化改进方案

本文档详细说明 RVV IDCT 实现相对于 NEON 的改进方案。

## 改进 1: 使用常量表（Constant Table）

### 问题分析
当前 RVV 实现每次调用都使用多个 `li` 指令加载常量：

```assembly
// 当前实现（低效）
li      a1, 64
li      a2, 83
li      a3, 36
li      t1, -64
li      t2, -83
...     // 总共 12+ 条指令
```

### NEON 的做法
```assembly
const trans, align=4
    .short  64, 83, 64, 36
    .short  89, 75, 50, 18
endconst

movrel      x1, trans
ld1         {v4.4h}, [x1]      // 一次加载所有系数！
```

### 改进方案

```assembly
const .Lidct_coeffs_4x8, align=4
    .short  64, 83, 64, 36
    .short  89, 75, 50, 18
    .short  -64, -83, -89, -50
    .short  -18, 0, 0, 0
endconst

// 使用时
lla             t5, .Lidct_coeffs_4x8
vle16.v         v20, (t5)         // 一次加载 8 个系数
```

**收益：**
- 减少约 10-12 条 `li` 指令
- 对于小尺寸变换（4x4, 8x8）收益明显
- 代码更简洁，易于维护

---

## 改进 2: 使用 vlseg 批量加载数据

### 问题分析
当前 RVV 需要逐行加载数据：

```assembly
// 4x4 IDCT
vle16.v     v4, (a0)
addi        t0, a0, 4 * 2 * 1
vle16.v     v1, (t0)
addi        t0, a0, 4 * 2 * 2
vle16.v     v2, (t0)
addi        t0, a0, 4 * 2 * 3
vle16.v     v3, (t0)           // 需要 4 次 vle16.v + 4 次 addi
```

### NEON 的优势
```assembly
ld1         {v0.4h-v3.4h}, [x0]    // 一次加载 4 个向量！
```

### 改进方案

RVV 提供了 `vlseg` 系列指令，可以一次加载多个向量：

```assembly
// 优化方案
vsetivli        zero, 4, e16, mf2, ta, ma
vlseg4e16.v     v4, (a0)          // 一次加载 v4, v5, v6, v7！
```

**指令对比：**

| 尺寸 | 原实现 | 优化后 | 减少 |
|------|-------|-------|------|
| 4x4 | 4 vle16 + 4 addi = 8 条 | 1 vlseg4e16 = 1 条 | -87.5% |
| 8x8 | 8 vle16 + 8 addi = 16 条 | 1 vlseg8e16 = 1 条 | -93.8% |

**注意事项：**
- `vlseg` 会使用连续的向量寄存器（v4, v5, v6, v7...）
- 需要调整寄存器分配策略
- VLEN 需要足够大以支持所需的 LMUL

---

## 改进 3: 特化 4x4 转置

### 问题分析
当前 RVV 使用通用的位合并法进行转置，对小尺寸不够优化。

### NEON 的做法
```assembly
transpose_4x8H  v16, v17, v18, v19, v26, v27, v28, v29
// 使用专用的 trn 指令，硬件优化
```

### 改进方案 A: 使用 vrgather（适合 4x4）

```assembly
const .Ltranspose_4x4_idx, align=4
    .byte  0, 4, 8, 12          // Column 0
    .byte  1, 5, 9, 13          // Column 1
    .byte  2, 6, 10, 14         // Column 2
    .byte  3, 7, 11, 15         // Column 3
endconst

vsetivli    zero, 4, e16, m1, ta, ma
vle8.v      v20, (transpose_table)
vzext.vf2   v20, v20           // 扩展到 16-bit 索引

vrgather.vv v16, v0, v20       // 第0列
vrgather.vv v17, v1, v20       // 第1列
vrgather.vv v18, v2, v20       // 第2列
vrgather.vv v19, v3, v20       // 第3列
```

**指令数对比：**
- 位合并法：~30 条指令
- vrgather 法：4 条 vrgather + 加载表 = 更简洁

### 改进方案 B: 完全消除转置

对于 4x4，可以重新设计算法，直接在一维数组上操作：

```assembly
// 加载所有 16 个元素到一个向量
vsetivli        zero, 16, e16, m1, ta, ma
vle16.v         v0, (a0)

// 直接进行行列混合的蝶形运算
// 避免显式的转置操作
```

**收益：**
- 完全消除转置开销
- 但需要重新设计算法

---

## 改进 4: 减少寄存器压力

### 问题分析
当前实现大量使用通用寄存器存储常量：

```assembly
li      a1, 64      // 使用 a1
li      a2, 83      // 使用 a2
li      a3, 36      // 使用 a3
...                 // 占用大量通用寄存器
```

### 改进方案

**方案 1: 预加载到向量寄存器**

```assembly
// 在函数开始时一次性加载
lla             t0, .Lidct_coeffs
vle16.v         v24, (t0)

// 后续从向量寄存器读取
vmv.x.s         a1, v24           // 提取第1个元素
vslidedown.vi   v24, v24, 1
vmv.x.s         a2, v24           // 提取第2个元素
```

**方案 2: 使用向量乘法**

```assembly
// 直接使用向量-向量乘法
vwmul.vv        v0, v8, v24       // v24 预加载了系数
```

**收益：**
- 减少通用寄存器占用
- 释放更多寄存器用于其他用途

---

## 改进 5: 优化数据存储

### 问题分析
8x8 IDCT 的存储效率：

```assembly
li              t3, 8*2
vssseg4e16.v    v24, (a0), t3      // 存储 4 个向量
add             t0, a0, 2*4
vssseg4e16.v    v28, (t0), t3      // 再存储 4 个向量
```

### 改进方案

使用 `vsseg` 系列指令批量存储：

```assembly
vsetivli        zero, 8, e16, m1, ta, ma
vsseg8e16.v     v24, (a0)          // 一次存储 8 个向量！
```

**指令数对比：**
- 原实现：2 条 vssseg4e16 + 1 条 add = 3 条
- 优化后：1 条 vsseg8e16 = 1 条

---

## 改进 6: add_residual 优化

### 问题分析
当前实现频繁切换 vtype：

```assembly
vsetivli        zero, 4, e8, mf4, ta, ma
vle8.v          v2, (a0)

vsetivli        zero, 4, e16, mf2, ta, ma    // 切换 vtype
vle16.v         v0, (a1)

vsetivli        zero, 4, e8, mf2, ta, ma     // 再切换
vnclipu.wi      v0, v0, 0
```

### 改进方案

**方案 1: 预加载常量**

```assembly
li      t4, 255               // 函数开始时加载一次

// 循环中使用
vmin.vx         v0, v0, t4    // 直接使用
```

**方案 2: 减少类型转换**

```assembly
// 如果残差值范围已知较小，可以优化
vle8.v          v2, (a0)          // 加载 8-bit 像素
vle8.v          v3, (a1)          // 假设残差也是 8-bit（需要验证）
vsadd.vv        v0, v2, v3        // 饱和加法
vse8.v          v0, (a0)          // 存储
```

**收益：**
- 减少 vtype 切换次数
- 提高流水线效率

---

## 性能预期

| 改进项 | 预期提升 | 适用尺寸 |
|--------|---------|---------|
| 常量表 | 5-10% | 所有 |
| vlseg 批量加载 | 15-25% | 4x4, 8x8 |
| 特化转置 | 10-15% | 4x4 |
| 减少寄存器压力 | 2-5% | 所有 |
| vsseg 批量存储 | 5-10% | 8x8 |
| add_residual 优化 | 3-8% | 所有 |

**总体预期：小尺寸 IDCT (4x4, 8x8) 性能提升 20-40%**

---

## 实现建议

### 优先级 1（高收益，低风险）
1. ✅ 使用常量表
2. ✅ 使用 vlseg/vsseg 批量加载/存储

### 优先级 2（中收益，中风险）
3. ⚠️ 4x4 转置特化（需要验证哪种方法最优）
4. ⚠️ 减少通用寄存器占用

### 优先级 3（高收益，高风险）
5. ⚠️ 完全消除 4x4 转置（需要算法重设计）
6. ⚠️ 向量-向量乘法代替标量乘法

---

## 验证方法

### 功能验证
```bash
make fate-hevc-idct
```

### 性能测试
```bash
# 使用 checkasm
./ffmpeg -benchmark -i hevc_test.mp4 -f null -

# 对比原实现 vs 优化实现
perf stat -e cycles,instructions ./ffmpeg ...
```

---

## 注意事项

1. **VLEN 依赖性**
   - `vlseg8e16` 需要 VLEN >= 128
   - 需要添加 fallback 实现支持小 VLEN

2. **寄存器分配**
   - `vlseg` 会占用连续向量寄存器
   - 需要重新规划寄存器使用

3. **兼容性**
   - 需要确保 RVV 1.0 支持
   - 测试不同硬件实现

---

## 下一步工作

1. 实现并测试改进 1-2（常量表 + vlseg）
2. 性能对比测试
3. 根据结果决定是否继续改进 3-6
4. 提交补丁到 FFmpeg 社区

---

## 参考代码

优化实现见：`hevcdsp_idct_rvv_optimized.S`

对比测试见：`hevcdsp_idct_benchmark.c`（待创建）
