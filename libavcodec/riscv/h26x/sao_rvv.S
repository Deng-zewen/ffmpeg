/*
 * Copyright (c) 2025 Institute of Software, Chinese Academy of Sciences (ISCAS).
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/riscv/asm.S"

#define HEVC_MAX_PB_SIZE 64
#define AV_INPUT_BUFFER_PADDING_SIZE 64
#define HEVC_SAO_STRIDE (2 * HEVC_MAX_PB_SIZE + AV_INPUT_BUFFER_PADDING_SIZE)

const .Lhevc_sao_edge_pos, align=4
.4byte 1                                        // horizontal
.4byte HEVC_SAO_STRIDE                          // vertical
.4byte HEVC_SAO_STRIDE + 1                      // 45 degree
.4byte HEVC_SAO_STRIDE - 1                      // 135 degree
endconst

const .Lhevc_edge_idx, align=0
        .byte  2, 4, 0, 6, 8
endconst

func ff_hevc_sao_edge_filter_8_rvv, zve32x, b
        lpad            0

        # 常量加载
        lla             a7, .Lhevc_sao_edge_pos
        li              t5, HEVC_SAO_STRIDE
        sh2add          a4, a4, a7
        lw              a4, (a4)                # a4 = 方向步长

        lla             t0, .Lhevc_edge_idx
        vsetivli        zero, 5, e8, m1, ta, ma
        vle16.v         v4, (t0)
        vluxei8.v       v28, (a3), v4           # 重排偏移表

        # 128-bit VLEN 固定配置m1
        sub             t5, t5, a5              # stride_src - width
        sub             t6, a2, a5              # stride_dst - width

1:      # 外层：按行
        mv              t4, a5
        sub             t2, a1, a4
        add             t3, a1, a4

2:      # 内层：向量处理
        vsetvli         t1, t4, e8, m1, ta, mu     # narrow: e8 + m1
        vle8.v          v8,  (a1)
        vle8.v          v16, (t2)
        vle8.v          v24, (t3)

        add             a1, a1, t1
        add             t2, t2, t1
        add             t3, t3, t1

        vmv.v.i         v12, 0
        vmsgtu.vv       v0, v8, v16
        vadd.vi         v12, v12, 1, v0.t
        vmsgtu.vv       v0, v16, v8
        vadd.vi         v12, v12, -1, v0.t
        vmsgtu.vv       v0, v8, v24
        vadd.vi         v12, v12, 1, v0.t
        vmsgtu.vv       v0, v24, v8
        vadd.vi         v12, v12, -1, v0.t

        li              t0, 2
        vadd.vx         v24, v12, t0

        vrgather.vv     v16, v28, v24
        vwcvtu.x.x.v    v0, v8
        vwadd.wv        v0, v0, v16

        vsetvli         zero, zero, e16, m2, ta, ma   # wide: e16 + m2（LMUL+1）
        li              t0, 255
        vmax.vx         v0, v0, zero
        vmin.vx         v0, v0, t0

        vsetvli         zero, zero, e8, m1, ta, ma    # 切回 narrow
        vncvt.x.x.w     v16, v0
        vse8.v          v16, (a0)

        add             a0, a0, t1
        sub             t4, t4, t1
        bnez            t4, 2b

        # 下一行
        add             a1, a1, t5
        add             a0, a0, t6
        addi            a6, a6, -1
        bnez            a6, 1b

        ret
endfunc

.macro sao_band_filter_wrap_around modular
1:
        mv              t3, a6              // 当前行剩余列数 = 图像宽度
2:
        vsetvli         t6, t3, e8, m1, ta, ma
        vle8.v          v8, (a1)           // 加载当前行像素
        add             a1, a1, t6          // 源指针前进

        vwcvtu.x.x.v    v24, v8            // 零扩展为16位 (v24~v25)
        vsrl.vi         v0, v8, 3          // 计算 band 索引 (0~31)

.if \modular
        vadd.vx         v0, v0, a5          // 环绕模式：索引 += (32 - left_class)
        vand.vx         v0, v0, t5          // 模 32
.else
        vsub.vx         v0, v0, a5          // 非环绕模式：索引 -= left_class
.endif

        li              t0, 4
        vmin.vx         v0, v0, t0          // 限制索引 ≤ 4，避免越界

        vrgather.vv     v4, v16, v0         // 查偏移表，v4 为 8bit 偏移

        vwadd.wv        v24, v24, v4        // 16bit 加宽累加

        vsetvli         zero, zero, e16, m2, ta, ma
        vmax.vx         v24, v24, zero      // 下钳位

        vsetvli         zero, zero, e8, m1, ta, ma
        vnclipu.wi      v8, v24, 0          // 无符号饱和窄化（自动上钳位至255）

        vse8.v          v8, (a0)            // 存储结果
        add             a0, a0, t6

        sub             t3, t3, t6
        bnez            t3, 2b              // 处理完一行

        add             a0, a0, a2          // 目标指针跳至下一行开头
        add             a1, a1, a3          // 源指针跳至下一行开头
        addi            a7, a7, -1          // 行数减1
        bnez            a7, 1b
.endm

func ff_hevc_sao_band_filter_8_rvv, zve32x, zbb, zba
        lpad            0
        vsetvli         t0, zero, e8, m1, ta, ma
        vmv.v.x         v16, zero

        li              t0, 4
        vsetivli        t0, 4, e8, m1, tu, ma
        add             t0, a4, 2
        li              t6, 2
        vlse8.v         v16, (t0), t6      // sao_offset_val[1~4]

        sub             a2, a2, a6         // dst_stride -= width
        sub             a3, a3, a6         // src_stride -= width

        li              t0, 28
        bgt             a5, t0, 0f         // left_class > 28 则使用环绕模式
        sao_band_filter_wrap_around 0
        ret
0:
        li              t5, 0x1F
        li              t0, 0x20
        sub             a5, t0, a5         // a5 = 32 - left_class
        sao_band_filter_wrap_around 1
        ret
endfunc
