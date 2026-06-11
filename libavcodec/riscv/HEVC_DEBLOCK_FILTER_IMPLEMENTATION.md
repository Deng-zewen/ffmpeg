# HEVC去块滤波器实现说明 - RISC-V RVV vs ARM NEON

## 目录
1. [概述](#概述)
2. [HEVC去块滤波器原理](#hevc去块滤波器原理)
3. [ARM NEON实现分析](#arm-neon实现分析)
4. [RISC-V RVV实现分析](#risc-v-rvv实现分析)
5. [关键技术对比](#关键技术对比)
6. [性能优化策略](#性能优化策略)
7. [实现细节对比](#实现细节对比)

---

## 概述

本文档详细说明了FFmpeg中HEVC去块滤波器的RISC-V RVV实现，并与成熟的ARM NEON实现进行对比分析。HEVC去块滤波器是视频解码中计算密集型的关键模块，其优化程度直接影响解码性能。

### 实现版本
- **RISC-V RVV**: `/libavcodec/riscv/hevcdsp_deblock_rvv.S`
- **ARM NEON**: `/libavcodec/aarch64/hevcdsp_deblock_neon.S`

### 关键差异
| 特性 | ARM NEON | RISC-V RVV |
|------|----------|------------|
| 向量长度 | 固定128位 | 可变长度(VLEN) |
| 编程模型 | 固定向量寄存器 | 可变LMUL配置 |
| 数据处理 | SIMD特定指令 | 灵活的向量指令 |
| 转置操作 | 专用指令 | 软件实现 |

---

## HEVC去块滤波器原理

### 滤波器类型

HEVC定义了两种去块滤波器：

#### 1. 色度滤波器 (Chroma Loop Filter)
- 处理4x4块边界
- 仅修改P0和Q0两个像素
- 使用简单的3抽头滤波器
- 公式：`delta0 = clip((((q0 - p0) << 2) + p1 - q1 + 4) >> 3, -tc, tc)`

#### 2. 亮度滤波器 (Luma Loop Filter)
- 处理8x8块边界
- 可能修改P2, P1, P0, Q0, Q1, Q2六个像素
- 包含强滤波和弱滤波两种模式
- 需要根据条件动态选择滤波强度

### 滤波器决策流程

```
1. 计算导数：
   dp0 = abs(P2 - 2*P1 + P0)
   dq0 = abs(Q2 - 2*Q1 + Q0)
   d0  = dp0 + dq0

2. 判断是否滤波：
   if (d0 < beta) {
       检查强滤波条件
       if (满足强滤波条件) {
           应用强滤波器
       } else {
           应用弱滤波器
       }
   }

3. 强滤波条件：
   - abs(P0 - Q0) < (beta >> 3) + (beta >> 5)
   - abs(P1 - P0) < (beta >> 2) + (beta >> 4)
   - abs(Q1 - Q0) < (beta >> 2) + (beta >> 4)
```

---

## ARM NEON实现分析

### 架构特点

ARM NEON使用固定128位向量寄存器，每个寄存器可以存储：
- 16个8位元素 (v0.16b)
- 8个16位元素 (v0.8h)
- 4个32位元素 (v0.4s)
- 2个64位元素 (v0.2d)

### 核心技术

#### 1. 转置优化

**色度垂直滤波器转置 (8位数据)**:
```asm
// 加载4行8字节数据
ld1             {v0.s}[0], [x0], x1  // 行0
ld1             {v1.s}[0], [x3], x1  // 行1
ld1             {v2.s}[0], [x0], x1  // 行2
ld1             {v3.s}[0], [x3], x1  // 行3

// 使用transpose_4x8B宏转置
transpose_4x8B  v0, v1, v2, v3, v28, v29, v30, v31
```

**转置宏实现原理**:
```asm
.macro transpose_4x8B r0, r1, r2, r3, r4, r5, r6, r7
        trn1    r4.8b, r0.8b, r1.8b   // 交叉元素
        trn2    r5.8b, r0.8b, r1.8b
        trn1    r6.8b, r2.8b, r3.8b
        trn2    r7.8b, r2.8b, r3.8b
        trn1    r0.8h, r4.8b, r6.8b   // 交叉对
        trn2    r1.8h, r4.8b, r6.8b
        trn1    r2.8h, r5.8b, r7.8b
        trn2    r3.8h, r5.8b, r7.8b
.endm
```

**关键优势**:
- 使用专用的`trn1`/`trn2`指令高效转置
- 单指令完成多个元素的交叉
- 无需额外的临时存储

#### 2. 强滤波器实现

**强滤波器公式** (NEON优化版本):
```asm
// P0' = (p2 + 2*p1 + 2*p0 + 2*q0 + q1 + 4) >> 3
add             v21.8h, v2.8h, v3.8h   // (p1 + p0)
add             v21.8h, v4.8h, v21.8h  //     + q0
shl             v21.8h, v21.8h, #1     //           * 2
add             v22.8h, v1.8h, v5.8h   //   (p2 + q1)
add             v21.8h, v22.8h, v21.8h // +
srshr           v21.8h, v21.8h, #3     //               >> 3
sub             v21.8h, v21.8h, v3.8h  //                    - p0
```

**优化技巧**:
- 减少乘法操作，使用移位和加法
- 合并计算减少指令数
- 利用向量并行性

#### 3. 条件判断优化

**掩码生成和应用**:
```asm
// 生成条件掩码
cmgt            v23.8h, v17.8h, v25.8h  // 比较结果生成掩码

// 合并多个条件
and             v23.16b, v23.16b, v21.16b

// 应用掩码选择滤波结果
bit             v21.16b, v22.16b, v16.16b  // 按位选择
```

### 完整亮度滤波器流程

```asm
function hevc_loop_filter_luma_body_8_neon
    // 1. 数据加载和扩展
    uxtl        v0.8h, v0.8b   // P3 -> 16位
    uxtl        v1.8h, v1.8b   // P2 -> 16位
    ...

    // 2. 计算导数
    sabd        v30.8h, v22.8h, v23.8h  // dp0
    sabd        v31.8h, v21.8h, v26.8h  // dq0

    // 3. 条件判断
    cmgt        v23.8h, v17.8h, v25.8h  // 生成掩码

    // 4. 根据掩码选择滤波器
    b.eq        1f  // 跳转到弱滤波
    // 强滤波代码...
    b           2f

1:  // 弱滤波代码...

2:  // 结果合并和存储
endfunc
```

---

## RISC-V RVV实现分析

### 架构特点

RISC-V向量扩展具有以下特点：
- **可变向量长度**: VLEN可以是128、256、512位或更多
- **灵活的LMUL**: 向量寄存器可以组合使用(LMUL=1,2,4,8)
- **动态配置**: 通过vsetvli指令动态设置向量参数

### 核心技术

#### 1. 转置实现

**8x8转置宏** (RVV实现):
```asm
.macro transpose_8x8_b r0, r1, r2, r3, r4, r5, r6, r7, tmp...
        // Step 1: 使用slide和merge交错相邻元素
        vid.v           tmp9            // 生成索引
        vand.vi         tmp8, tmp9, 0x1
        vmsne.vi        v0, tmp8, 0     // 生成掩码

        vslideup.vi     tmp0, r1, 1     // 滑动元素
        vmerge.vvm      tmp4, r0, tmp0, v0  // 合并

        // Step 2-3: 重复交错过程完成转置
        ...
.endm
```

**实现原理**:
1. 使用`vid.v`生成元素索引
2. 使用位运算生成选择掩码
3. 使用`vslideup`/`vslidedown`移动元素
4. 使用`vmerge`按掩码合并

**对比NEON**:
| 特性 | NEON | RVV |
|------|------|-----|
| 指令数 | ~10条 | ~40条 |
| 转置方式 | 专用trn指令 | 软件slide+merge |
| 临时寄存器 | 4个 | 10个 |
| 性能 | 高 | 中等 |

#### 2. 强滤波器实现

**强滤波器公式** (RVV完整实现):
```asm
// P0' = (P3 + 2*P2 + 2*P1 + 2*P0 + Q0 + 4) >> 3
vslli.vx        v1, v9, 1           // 2*P2
vadd.vv         v1, v1, v8          // + P3
vslli.vx        v2, v10, 1          // 2*P1
vadd.vv         v1, v1, v2          // + 2*P1
vslli.vx        v2, v11, 1          // 2*P0
vadd.vv         v1, v1, v2          // + 2*P0
vadd.vv         v1, v1, v12         // + Q0
vadd.vi         v1, v1, 4           // + 4
vsrai.vx        v1, v1, 3           // >> 3
```

**优化策略**:
- 分解乘法为移位和加法
- 逐步累加中间结果
- 使用向量寄存器避免内存访问

#### 3. 条件判断实现

**掩码生成和分支**:
```asm
// 计算比较值
vsub.vv         v1, v11, v12        // P0 - Q0
vabs.vv         v1, v1              // abs(P0-Q0)

// 生成掩码
vmslt.vx        v0, v1, t2          // abs(P0-Q0) < threshold1?
vmslt.vx        v1, v2, t3          // abs(P1-P0) < threshold2?
vmand.mm        v0, v0, v1          // 合并条件

// 根据掩码分支
vfirst.m        t5, v0              // 提取第一个匹配
bltz            t5, .Lweak_filter   // 跳转到弱滤波
```

### 完整亮度垂直滤波器流程

```asm
func ff_hevc_v_loop_filter_luma_8_rvv
    // 1. 保存寄存器和栈分配
    addi        sp, sp, -112
    sd          ra, 104(sp)
    ...

    // 2. 加载参数
    mv          s0, a0          // pix
    mv          s1, a1          // stride
    mv          s2, a2          // beta

    // 3. 加载8行数据
    vle8.v      v0, (s0)        // P3
    add         s0, s0, s1
    vle8.v      v1, (s0)        // P2
    ...

    // 4. 转置8x8矩阵
    transpose_8x8_b v0,v1,v2,v3,v4,v5,v6,v7,...

    // 5. 扩展到16位
    vzext.vf2   v16, v0         // P3 16位
    ...

    // 6. 计算导数和条件判断
    vadd.vv     v24, v17, v19   // P2 + P0
    vslli.vx    v25, v18, 1     // 2 * P1
    vsub.vv     v24, v24, v25
    vabs.vv     v24, v24        // dp0

    // 7. 处理Group 0 (列0-3)
    vsetivli    zero, 4, e16, mf2, ta, ma
    vslidedown.vi v8, v16, 0    // 提取元素

    // 8. 强/弱滤波器决策
    vmslt.vx    v0, v1, t2      // 条件检查
    vfirst.m    t5, v0
    bltz        t5, .Lweak_filter_group0

    // 9. 强滤波器实现
.Lstrong_filter_group0:
    // P0', P1', P2', Q0', Q1', Q2'计算
    ...

    // 10. 弱滤波器实现
.Lweak_filter_group0:
    // delta0计算和clip
    ...

    // 11. 处理Group 1 (列4-7)
    // 类似Group 0的处理

    // 12. 合并结果和转置回
    vnclipu.wi  v20, v8, 0      // 缩减到8位
    vslideup.vi v20, v26, 4     // 合并

    // 13. 存储结果
    vse8.v      v1, (s0)        // 存储P2'
    ...

    // 14. 恢复寄存器
    ld          ra, 104(sp)
    addi        sp, sp, 112
    ret
endfunc
```

---

## 关键技术对比

### 1. 数据加载和存储

#### NEON方式
```asm
// 连续加载
ld1         {v0.8b}, [x0], x1    // 加载后自动增加指针

// 交错加载
ld2         {v0.8b, v1.8b}, [x0] // 加载并解交错

// 转置加载(垂直滤波器)
ld1         {v0.d}[0], [x0], x1  // 按列加载
ld1         {v0.d}[1], [x0], x1
```

**优势**:
- 丰富的寻址模式
- 自动指针递增
- 交错加载指令

#### RVV方式
```asm
// 基本加载
vle8.v      v0, (a0)             // 加载向量
add         a0, a0, a1           // 手动增加指针

// 无交错加载指令
// 需要软件实现转置
```

**挑战**:
- 缺少交错加载指令
- 需要手动管理指针
- 转置需要软件实现

### 2. 数据类型转换

#### NEON方式
```asm
// 扩展到16位
uxtl        v20.8h, v0.8b        // 无符号扩展

// 缩减到8位
sqxtun      v1.8b, v1.8h         // 饱和截断

// 饱和操作
sqadd       v1.8h, v1.8h, v5.8h  // 饱和加法
sqsub       v2.8h, v2.8h, v5.8h  // 饱和减法
```

#### RVV方式
```asm
// 扩展到16位
vzext.vf2   v16, v0              // 零扩展

// 缩减到8位
vnclipu.wi  v12, v8, 0           // 向量窄化

// 饱和需要手动clip
vmax.vx     v4, v4, zero         // 下限clip
vmin.vx     v4, v4, t2           // 上限clip
```

**对比**:
| 操作 | NEON | RVV |
|------|------|-----|
| 扩展 | 1条指令 | 1条指令 |
| 缩减 | 1条指令(饱和) | 1条指令 |
| 饱和加法 | 1条指令 | 3条指令(add+max+min) |
| Clip到范围 | 内置 | 手动实现 |

### 3. 分组处理策略

两种架构都采用分组处理：

**NEON Group处理**:
```asm
// 加载tc值
ldr         w7, [x3]             // tc[0]
ldr         w8, [x3, #4]         // tc[1]

// 复制到向量
dup         v18.4h, w7           // 前4个元素
dup         v19.4h, w8           // 后4个元素
trn1        v18.2d, v18.2d, v19.2d  // 合并
```

**RVV Group处理**:
```asm
// 加载tc值
lw          t0, 0(a3)            // tc[0]
lw          t1, 4(a3)            // tc[1]

// 提取前4个元素
vsetivli    zero, 4, e16, mf2, ta, ma
vslidedown.vi v8, v16, 0         // Group 0

// 提取后4个元素
vslidedown.vi v8, v16, 4         // Group 1
```

### 4. 水平滤波器处理

#### NEON实现
```asm
// 直接加载行数据
ld1         {v0.8b}, [x0], x1    // 行0
ld1         {v1.8b}, [x0], x1    // 行1
ld1         {v2.8b}, [x0], x1    // 行2
ld1         {v3.8b}, [x0]        // 行3

// 水平行已在向量中正确排列
// 直接处理，无需转置
```

**优势**: 内存布局天然匹配向量布局

#### RVV实现
```asm
// 加载8行
vle8.v      v0, (s0)             // 行0
add         s0, s0, s1
vle8.v      v1, (s0)             // 行1
...

// 每行单独处理
vslidedown.vi v24, v16, 3        // 提取P0
vslidedown.vi v25, v16, 4        // 提取Q0
```

**挑战**: 需要逐行处理，代码较长

---

## 性能优化策略

### NEON优化技术

#### 1. 指令级并行
```asm
// 多条独立指令并行执行
add         v21.8h, v2.8h, v3.8h
add         v22.8h, v6.8h, v5.8h
shl         v23.8h, v0.8h, #1
```

#### 2. 宏定义减少代码重复
```asm
.macro hevc_loop_filter_luma_body bitdepth
    // 参数化实现
    ...
.endm

// 实例化多个位深版本
hevc_loop_filter_luma_body 8
hevc_loop_filter_luma_body 10
hevc_loop_filter_luma_body 12
```

#### 3. 分支预测优化
```asm
// 使用条件执行减少分支
cmp         w12, w13
b.hi        0f  // 预测跳转
```

### RVV优化技术

#### 1. 向量长度利用
```asm
// 动态设置最优向量长度
vsetivli    zero, 8, e8, m1, ta, ma   // 8元素，8位，LMUL=1
vsetivli    zero, 4, e16, mf2, ta, ma // 4元素，16位，LMUL=0.5
```

#### 2. 寄存器分配优化
```asm
// 使用v16-v23作为临时寄存器，避免溢出
vzext.vf2   v16, v0     // P3
vzext.vf2   v17, v1     // P2
...
// v0-v7保持原始数据
// v24-v31用于中间计算
```

#### 3. 循环展开
```asm
// 手动展开所有8行处理，避免循环开销
// Row 0
vslidedown.vi v24, v16, 3
...
// Row 1
vslidedown.vi v24, v17, 3
...
```

#### 4. 掩码优化
```asm
// 生成掩码一次，多次使用
vmslt.vx    v0, v1, t2      // 生成掩码
vmand.mm    v0, v0, v1      // 合并条件
vfirst.m    t5, v0          // 提取结果
```

---

## 实现细节对比

### 1. 代码复杂度

| 指标 | NEON | RVV |
|------|------|-----|
| 色度滤波器行数 | ~180行 | ~870行 |
| 亮度滤波器行数 | ~300行 | ~600行 |
| 宏定义数量 | 4个 | 1个 |
| 临时寄存器使用 | 8个 | 16个 |

### 2. 内存访问模式

#### NEON
```
垂直滤波器：
加载 -> 转置 -> 处理 -> 转置 -> 存储
      ^专用指令

水平滤波器：
加载 -> 处理 -> 存储
      ^直接处理
```

#### RVV
```
垂直滤波器：
加载 -> 转置(软件) -> 处理 -> 转置(软件) -> 存储
       ^开销大

水平滤波器：
加载 -> 处理 -> 存储
       ^逐行处理
```

### 3. 强滤波器公式对比

#### P0'计算

**NEON优化**:
```asm
// 5条指令
add     v21.8h, v2.8h, v3.8h
add     v21.8h, v4.8h, v21.8h
shl     v21.8h, v21.8h, #1
add     v22.8h, v1.8h, v5.8h
add     v21.8h, v22.8h, v21.8h
srshr   v21.8h, v21.8h, #3
```

**RVV实现**:
```asm
// 7条指令
vslli.vx    v1, v9, 1
vadd.vv     v1, v1, v8
vslli.vx    v2, v10, 1
vadd.vv     v1, v1, v2
vslli.vx    v2, v11, 1
vadd.vv     v1, v1, v2
vadd.vv     v1, v1, v12
vadd.vi     v1, v1, 4
vsrai.vx    v1, v1, 3
```

### 4. 弱滤波器delta0计算

**公式**: `delta0 = (9*(q0-p0) - 3*(q1-p1) + 8) >> 4`

#### NEON实现
```asm
sub         v27.8h, v4.8h, v3.8h   // q0 - p0
shl         v30.8h, v27.8h, #3     // * 8
add         v27.8h, v27.8h, v30.8h // * 9

sub         v30.8h, v5.8h, v2.8h   // q1 - p1
shl         v31.8h, v30.8h, #1     // * 2
sub         v27.8h, v27.8h, v31.8h
sub         v27.8h, v27.8h, v30.8h // - 3*(q1-p1)
srshr       v27.8h, v27.8h, #4
```

#### RVV实现
```asm
vsub.vv     v0, v12, v11           // q0 - p0
vslli.vx    v1, v0, 3              // * 8
vadd.vv     v1, v1, v0             // * 9

vsub.vv     v2, v13, v10           // q1 - p1
vslli.vx    v3, v2, 1              // * 2
vadd.vv     v3, v3, v2             // * 3

vsub.vv     v1, v1, v3             // 9*(q0-p0) - 3*(q1-p1)
vadd.vi     v1, v1, 8
vsrai.vx    v1, v1, 4
```

**指令数**: NEON 7条 vs RVV 9条

### 5. Clip操作对比

#### NEON内置Clip
```asm
// 单指令clip到[-tc, tc]
neg         v17.8h, v16.8h
clip        v17.8h, v16.8h, v5.8h
```

#### RVV手动Clip
```asm
// 3条指令clip到[-tc, tc]
neg         t2, t0
vmax.vx     v1, v1, t2
vmin.vx     v1, v1, t0
```

---

## 性能分析

### 理论分析

假设处理一个8x8块的亮度滤波器：

| 操作 | NEON周期 | RVV周期 | 比率 |
|------|----------|---------|------|
| 数据加载 | 16 | 16 | 1.0x |
| 数据扩展 | 8 | 8 | 1.0x |
| 转置(垂直) | 20 | 80 | 4.0x |
| 滤波计算 | 100 | 120 | 1.2x |
| 转置回(垂直) | 20 | 80 | 4.0x |
| 数据存储 | 16 | 16 | 1.0x |
| **总计(垂直)** | **180** | **320** | **1.78x** |
| **总计(水平)** | **140** | **160** | **1.14x** |

### 瓶颈分析

#### NEON瓶颈
1. 分支预测失败
2. 内存延迟
3. 指令依赖链

#### RVV瓶颈
1. **转置开销** (最大瓶颈)
2. 向量配置开销
3. 指令数较多

### 优化建议

#### 对于RVV实现

1. **转置优化**:
   - 考虑使用LMUL=2增加向量长度
   - 探索新的转置算法
   - 可能使用查表法加速

2. **指令调度**:
   ```asm
   // 重排指令以隐藏延迟
   vle8.v      v0, (s0)        // 加载
   add         s0, s0, s1      // 计算地址(并行)
   vle8.v      v1, (s0)        // 加载
   ```

3. **批量处理**:
   - 使用LMUL=2一次处理16个元素
   - 减少循环开销

4. **专用路径**:
   ```asm
   // 为不同VLEN提供专用实现
   .if vlenb >= 32
       // 256位向量优化版本
   .else
       // 128位向量标准版本
   .endif
   ```

---

## 实现要点总结

### NEON优势

1. **专用指令**:
   - `trn1`/`trn2`高效转置
   - `ld2`/`ld3`交错加载
   - `clip`内置饱和操作

2. **指令密度**:
   - 复杂操作编码为单条指令
   - 代码更紧凑

3. **成熟优化**:
   - 经过长期优化
   - 大量实测验证

### RVV优势

1. **灵活性**:
   - 可变向量长度
   - LMUL配置灵活
   - 未来扩展性好

2. **可移植性**:
   - 不依赖固定向量长度
   - 适应不同硬件实现

3. **正交设计**:
   - 指令功能正交
   - 组合能力强

### 改进方向

#### 短期改进
1. ✅ 实现完整的强滤波器
2. ✅ 添加条件判断逻辑
3. ✅ 完善水平滤波器8行处理
4. ⚠️ 测试和验证正确性
5. ⚠️ 性能调优

#### 长期优化
1. 探索新的转置算法
2. 使用LMUL=2优化
3. 批量处理多个块
4. 针对特定VLEN优化

---

## 附录

### A. 关键指令对照表

| 操作 | NEON | RVV | 说明 |
|------|------|-----|------|
| 向量加载 | ld1 | vle8.v | 加载向量 |
| 向量存储 | st1 | vse8.v | 存储向量 |
| 零扩展 | uxtl | vzext.vf2 | 8位→16位 |
| 窄化 | sqxtun | vnclipu.wi | 16位→8位 |
| 绝对差 | sabd | vsub+vabs | 绝对差值 |
| 左移 | shl | vslli.vx | 向量左移 |
| 右移 | srshr | vsrai.vx | 算术右移 |
| 比较 | cmgt | vmslt.vx | 大于比较 |
| 掩码操作 | and | vmand.mm | 掩码与 |
| 转置 | trn1/trn2 | slide+merge | 元素重排 |

### B. 寄存器使用对比

#### NEON寄存器分配
```
v0-v7:   加载的原始数据(P3,Q3等)
v8-v15:  tc值, beta值等参数
v16-v23: 展开后的16位数据
v24-v31: 临时计算寄存器
```

#### RVV寄存器分配
```
v0-v7:   原始数据/转置后数据
v8-v15:  提取的分组数据
v16-v23: 展开后的16位数据
v24-v31: 中间计算结果
```

### C. 性能计数器建议

#### 测量指标
1. 执行周期数
2. 指令数
3. 缓存命中率
4. 分支预测成功率

#### NEON测量
```bash
perf stat -e cycles,instructions,cache-misses,branch-misses ./ffmpeg ...
```

#### RVV测量
```bash
# 使用RISC-V性能计数器
perf stat -e r10,r11 ./ffmpeg ...
```

---

## 参考文档

1. **HEVC标准**: ITU-T H.265 (08/2021)
2. **ARM NEON参考**: ARM Architecture Reference Manual
3. **RISC-V V扩展**: RISC-V V Vector Extension Specification v1.0
4. **FFmpeg源码**: https://github.com/FFmpeg/FFmpeg

---

**文档版本**: 1.0
**最后更新**: 2026-04-30
**作者**: FFmpeg RISC-V优化团队
