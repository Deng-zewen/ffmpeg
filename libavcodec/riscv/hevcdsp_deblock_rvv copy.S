# =============================================================================
# ff_hevc_h_loop_filter_luma_8_rvv
# HEVC 水平边界亮度环路滤波实现，适用于 8-bit 位深
# 使用 RISC-V 向量扩展 (RVV) 1.0
# 该实现严格复刻了 AArch64 NEON 版本的掩码逻辑与分支行为。
#
# 调用约定 (LP64)：
#   a0 : uint8_t *pix        – 指向当前 8×8 块边界的像素指针
#   a1 : ptrdiff_t stride    – 图像行步长（以字节为单位）
#   a2 : int beta            – 滤波阈值 β
#   a3 : const int32_t *tc   – 指向包含两个 tC 值的数组
#   a4 : const uint8_t *no_p – 未使用（与 NEON 版本一致）
#   a5 : const uint8_t *no_q – 未使用（与 NEON 版本一致）
# =============================================================================

#include "libavutil/riscv/asm.S"

.macro vabs d0, s0, t0
    vrsub.vi    \t0, \s0, 0
    vmax.vv     \d0, \s0, \t0
.endm

func ff_hevc_h_loop_filter_luma_8_rvv
        # ---------- 保存被调用者保存的寄存器 ----------
        addi        sp, sp, -112
        sd          ra, 104(sp)
        sd          s0,  96(sp)
        sd          s1,  88(sp)
        sd          s2,  80(sp)
        sd          s3,  72(sp)
        sd          s4,  64(sp)
        sd          s5,  56(sp)
        sd          s6,  48(sp)
        sd          s7,  40(sp)

        # ---- 指针调整：pix -= 4 * stride ----
        # 水平滤波时，以当前行为基准向上移动4行，使后续8行加载能覆盖到
        # P侧的上方像素和Q侧的下方像素。
        slli        t0, a1, 2
        sub         a0, a0, t0
        mv          s0, a0              # s0 保存滤波起始地址，后续存储时使用

        # ---- 加载8行像素（每行8个字节）----
        vsetivli    zero, 8, e8, m1, ta, ma   # 设置向量长度：8个元素，8-bit
        vle8.v      v0, (a0)            # row 0 (对应 P3 列)
        add         a0, a0, a1
        vle8.v      v1, (a0)            # row 1 (P2)
        add         a0, a0, a1
        vle8.v      v2, (a0)            # row 2 (P1)
        add         a0, a0, a1
        vle8.v      v3, (a0)            # row 3 (P0)
        add         a0, a0, a1
        vle8.v      v4, (a0)            # row 4 (Q0)
        add         a0, a0, a1
        vle8.v      v5, (a0)            # row 5 (Q1)
        add         a0, a0, a1
        vle8.v      v6, (a0)            # row 6 (Q2)
        add         a0, a0, a1
        vle8.v      v7, (a0)            # row 7 (Q3)

        # ---- 将所有像素扩展为 16-bit，方便后续运算 ----
        vsetivli    zero, 8, e16, m1, ta, ma
        vzext.vf2   v8,  v0   # P3
        vzext.vf2   v9,  v1   # P2
        vzext.vf2   v10, v2   # P1
        vzext.vf2   v11, v3   # P0
        vzext.vf2   v12, v4   # Q0
        vzext.vf2   v13, v5   # Q1
        vzext.vf2   v14, v6   # Q2
        vzext.vf2   v15, v7   # Q3

        # ---------- 准备参数 ----------
        lw          s1, 0(a3)          # tc[0]
        lw          s2, 4(a3)          # tc[1]
        or          t0, s1, s2
        beqz        t0, .Lexit         # 若两个 tC 都为0，则无需滤波，直接退出

        vmv.v.x     v16, a2            # beta 广播到所有向量元素

        # 构造 tC 向量：低4元素为 tc[0]，高4元素为 tc[1]
        vmv.v.x     v17, s1
        vsetivli    zero, 4, e16, mf2, ta, ma
        vmv.v.x     v18, s2
        vsetivli    zero, 8, e16, m1, ta, ma
        vslideup.vi v17, v18, 4        # v17 = [tc0, tc0, tc0, tc0, tc1, tc1, tc1, tc1]

        # tc25 = (tc * 5 + 1) >> 1
        vsll.vi     v18, v17, 2        # v18 = 4 * tc
        vadd.vv     v18, v18, v17      # v18 = 5 * tc
        vadd.vi     v18, v18, 1
        vsrl.vi     v19, v18, 1        # v19 = tc25

        # beta2 = beta >> 2
        vsrl.vi     v20, v16, 2        # v20 = beta2

        # ----- 计算 dp0, dq0, d0（每行单独计算）-----
        # dp0 = |P2 - 2*P1 + P0|
        vadd.vv     v21, v11, v9       # v21 = P0 + P2
        vsll.vi     v22, v10, 1        # v22 = 2*P1
        vsub.vv     v21, v21, v22      # v21 = (P0 + P2) - 2*P1
        vabs      v21, v21, v30           # v21 = dp0

        # dq0 = |Q2 - 2*Q1 + Q0|
        vadd.vv     v22, v14, v12      # v22 = Q2 + Q0
        vsll.vi     v23, v13, 1        # v23 = 2*Q1
        vsub.vv     v22, v22, v23
        vabs      v22, v22, v30           # v22 = dq0

        # d0 = dp0 + dq0
        vadd.vv     v23, v21, v22      # v23 = d0

        # (d0 << 1) < beta2 判断，作为强滤波条件之一
        vsll.vi     v24, v23, 1        # v24 = d0 << 1
        vmsltu.vv   v0, v24, v20       # v0 中满足条件的 lane 置 1
        vmv.v.i     v24, 0
        vmerge.vim  v24, v24, -1, v0   # 将 v0 掩码扩展为 0xFFFF 或 0

        # ----- 分组边界检查（使用 d0 + d3 < beta）-----
        # HEVC 以 4 行为一组进行决策，第一组用行0和行3的 d0/d3，
        # 第二组用行4和行7的 d0/d3（这里分别对应向量元素下标0,3和4,7）。
        vsetivli    zero, 1, e16, m1, ta, ma
        vmv.x.s     t0, v23            # t0 = d0 of row 0
        vsetivli    zero, 8, e16, m1, ta, ma
        vslidedown.vi v25, v23, 3
        vmv.x.s     t1, v25            # t1 = d3 of row 3
        add         t0, t0, t1         # t0 = d0 + d3 (group 0)

        vslidedown.vi v25, v23, 4
        vmv.x.s     t2, v25            # t2 = d0 of row 4
        vslidedown.vi v25, v23, 7
        vmv.x.s     t3, v25            # t3 = d3 of row 7
        add         t2, t2, t3         # t2 = d0 + d3 (group 1)

        mv          t4, a2             # t4 = beta
        sltu        t0, t0, t4         # 若 group 0 的和 < beta，则 t0=1
        sltu        t2, t2, t4         # group 1 同理
        or          t5, t0, t2
        beqz        t5, .Lexit         # 两组都不满足，直接退出

        # 生成 "滤波使能" 掩码 v26，高4元素使能 group 1，低4元素使能 group 0
        neg         t0, t0             # 将 0/1 转换为 0 / 0xFFFFFFFF（64位）
        li          t1, 0xFFFF
        and         t0, t0, t1         # 仅取低 16 位得到 0xFFFF 或 0
        neg         t2, t2
        and         t2, t2, t1
        vmv.v.x     v26, t0            # 临时设置所有元素 = group 0 使能
        vsetivli    zero, 4, e16, mf2, ta, ma
        vmv.v.x     v25, t2            # 设置低 4 元素 = group 1 使能
        vsetivli    zero, 8, e16, m1, ta, ma
        vslideup.vi v26, v25, 4        # 插入高 4 元素，完成使能掩码

        # ----- 强滤波条件检查 -----
        vsrl.vi     v27, v20, 1        # beta_3 = beta2 >> 1

        # (|P3 - P0| + |Q3 - Q0|) < beta_3
        vsub.vv     v28, v8, v11
        vabs      v28, v28, v30
        vsub.vv     v29, v15, v12
        vabs      v29, v29, v30
        vadd.vv     v28, v28, v29
        vmsltu.vv   v0, v28, v27
        vmv.v.i     v28, 0
        vmerge.vim  v28, v28, -1, v0   # v28 = 条件掩码
        vand.vv     v24, v24, v28      # 结合之前的 (d0<<1)<beta2 掩码

        # |P0 - Q0| < tc25
        vsub.vv     v28, v11, v12
        vabs      v28, v28, v30
        vmsltu.vv   v0, v28, v19       # v19 = tc25
        vmv.v.i     v28, 0
        vmerge.vim  v28, v28, -1, v0
        vand.vv     v24, v24, v28      # v24 即逐行的 "强滤波候选" 掩码

        # 每组强滤波的条件是：该组内的行0和行3（或行4和行7）都满足候选条件
        vmv.x.s     t0, v24            # row0 的候选标志
        vslidedown.vi v28, v24, 3
        vmv.x.s     t1, v28            # row3 的候选标志
        and         t0, t0, t1         # group 0 强滤波标志（0xFFFF 或 0）
        vslidedown.vi v28, v24, 4
        vmv.x.s     t2, v28            # row4 的候选标志
        vslidedown.vi v28, v24, 7
        vmv.x.s     t3, v28            # row7 的候选标志
        and         t2, t2, t3         # group 1 强滤波标志

        mv          s3, t0             # s3 = group 0 强标志 (0xFFFF 或 0)
        mv          s4, t2             # s4 = group 1 强标志

        # 构造 "强/弱选择" 掩码 v17（与 NEON 版本作用相同）
        vmv.v.x     v17, s3
        vsetivli    zero, 4, e16, mf2, ta, ma
        vmv.v.x     v18, s4
        vsetivli    zero, 8, e16, m1, ta, ma
        vslideup.vi v17, v18, 4        # v17 = 每组的强/弱选择 (1=强, 0=弱)

        # ----- 若两组都是强滤波，则不需要计算 nd_p/nd_q -----
        li          t4, 0xFFFF
        bne         s3, t4, .Lneed_weak
        bne         s4, t4, .Lneed_weak
        j           .Lstrong_only

.Lneed_weak:
        # 弱滤波需要用到 nd_p / nd_q，先计算阈值
        # threshold = (beta + (beta >> 1)) >> 3
        srli        t5, a2, 1
        add         t5, t5, a2
        srli        t5, t5, 3

        # 计算 group0 和 group1 的 dp 和 dq 之和（仅使用关键行0,3,4,7）
        # dp 和 dq 已分别位于 v21 和 v22 中
        vmv.x.s     t0, v21            # dp row0
        vslidedown.vi v25, v21, 3
        vmv.x.s     t1, v25            # dp row3
        add         t0, t0, t1         # dp_sum0
        vslidedown.vi v25, v21, 4
        vmv.x.s     t2, v25            # dp row4
        vslidedown.vi v25, v21, 7
        vmv.x.s     t3, v25            # dp row7
        add         t2, t2, t3         # dp_sum1

        vmv.x.s     t6, v22            # dq row0
        vslidedown.vi v25, v22, 3
        vmv.x.s     a6, v25            # dq row3 (使用 a6 临时)
        add         t6, t6, a6         # dq_sum0
        vslidedown.vi v25, v22, 4
        vmv.x.s     a7, v25            # dq row4
        vslidedown.vi v25, v22, 7
        vmv.x.s     a4, v25            # dq row7 (复用 a4 寄存器)
        add         a7, a7, a4         # dq_sum1

        # 比较得到 nd_p 和 nd_q 标志（1 表示满足条件需要修正）
        sltu        t0, t0, t5         # nd_p0
        sltu        t2, t2, t5         # nd_p1
        sltu        t6, t6, t5         # nd_q0
        sltu        a7, a7, t5         # nd_q1

        # 将 0/1 扩张为全位掩码 (0 或 0xFFFF)
        neg         t0, t0
        neg         t2, t2
        neg         t6, t6
        neg         a7, a7

        # 构造 nd_p 向量 v28（低4元素对应 group0，高4元素对应 group1）
        vmv.v.x     v28, t0
        vsetivli    zero, 4, e16, mf2, ta, ma
        vmv.v.x     v25, t2
        vsetivli    zero, 8, e16, m1, ta, ma
        vslideup.vi v28, v25, 4        # v28 = nd_p 掩码

        # 构造 nd_q 向量 v29
        vmv.v.x     v29, t6
        vsetivli    zero, 4, e16, mf2, ta, ma
        vmv.v.x     v25, a7
        vsetivli    zero, 8, e16, m1, ta, ma
        vslideup.vi v29, v25, 4        # v29 = nd_q 掩码

        # ============ 弱滤波计算 ============
        # delta0 = (9*(q0-p0) - 3*(q1-p1) + 8) >> 4
        vsub.vv     v30, v12, v11      # q0 - p0
        vsll.vi     v31, v30, 3
        vadd.vv     v30, v30, v31      # 9*(q0-p0)
        vsub.vv     v31, v13, v10      # q1 - p1
        vsll.vi     v27, v31, 1
        vadd.vv     v31, v27, v31      # 3*(q1-p1)
        vsub.vv     v30, v30, v31      # 9*(q0-p0) - 3*(q1-p1)
        vadd.vi     v30, v30, 8
        vsra.vi     v30, v30, 4        # delta0 (带符号)

        # 弱滤波还需满足：|delta0| < 10*tc
        vsll.vi     v27, v17, 1        # 2*tc
        vsll.vi     v24, v17, 3        # 8*tc
        vadd.vv     v27, v27, v24      # 10*tc
        vabs      v24, v30, v31
        vmsltu.vv   v0, v24, v27
        vmv.v.i     v27, 0
        vmerge.vim  v27, v27, -1, v0   # 弱滤波应用掩码 v27
        vand.vv     v27, v27, v26      # 与组使能掩码结合

        # delta0 裁剪到 [-tc, tc]
        vneg.v      v31, v17
        vmax.vv     v30, v30, v31
        vmin.vv     v30, v30, v17

        # 计算 delta_p1 和 delta_q1
        # deltap1 = Clip(( (p2+p0+1)>>1 - p1 + delta0 ) >> 1, -tc_2, tc_2)
        vadd.vv     v24, v9, v11       # p2+p0
        vadd.vi     v24, v24, 1
        vsrl.vi     v24, v24, 1
        vsub.vv     v24, v24, v10
        vadd.vv     v24, v24, v30
        vsra.vi     v24, v24, 1

        # deltaq1 = Clip(( (q2+q0+1)>>1 - q1 - delta0 ) >> 1, -tc_2, tc_2)
        vadd.vv     v25, v14, v12
        vadd.vi     v25, v25, 1
        vsrl.vi     v25, v25, 1
        vsub.vv     v25, v25, v13
        vsub.vv     v25, v25, v30
        vsra.vi     v25, v25, 1

        # 裁剪到 [-tc_2, tc_2] （tc_2 = tc >> 1）
        vsrl.vi     v31, v17, 1        # tc_2
        vneg.v      v18, v31
        vmax.vv     v24, v24, v18
        vmin.vv     v24, v24, v31
        vmax.vv     v25, v25, v18
        vmin.vv     v25, v25, v31

        # 应用 nd_p, nd_q 和弱滤波使能掩码
        vand.vv     v24, v24, v28      # deltap1 仅当 nd_p=1 时有效
        vand.vv     v25, v25, v29      # deltaq1 仅当 nd_q=1 时有效
        vand.vv     v24, v24, v27
        vand.vv     v30, v30, v27
        vand.vv     v25, v25, v27

        # 计算弱滤波结果（临时使用 v18~v21）
        vadd.vv     v18, v10, v24      # P1' weak
        vadd.vv     v19, v11, v30      # P0' weak
        vsub.vv     v20, v12, v30      # Q0' weak
        vadd.vv     v21, v13, v25      # Q1' weak
        # P2 和 Q2 保持原值（弱滤波不修改它们）

.Lstrong_only:
        # ============ 强滤波计算 ============
        # tc2 = 2 * tc
        vsll.vi     v24, v17, 1
        vneg.v      v25, v24           # -tc2

        # P0' = p0 + Clip( (p2+2*p1+2*p0+2*q0+q1+4)>>3 - p0, -tc2, tc2 )
        vadd.vv     v26, v10, v11      # p1 + p0
        vadd.vv     v26, v26, v12      # + q0
        vsll.vi     v26, v26, 1        # 2*(…)
        vadd.vv     v27, v9, v13       # p2 + q1
        vadd.vv     v26, v26, v27
        vadd.vi     v26, v26, 4
        vsrl.vi     v26, v26, 3
        vsub.vv     v26, v26, v11      # delta P0

        # P1' = p1 + Clip( (p2+p1+p0+q0+2)>>2 - p1, -tc2, tc2 )
        vadd.vv     v27, v9, v10
        vadd.vv     v28, v11, v12
        vadd.vv     v27, v27, v28
        vadd.vi     v27, v27, 2
        vsrl.vi     v27, v27, 2
        vsub.vv     v27, v27, v10

        # P2' = p2 + Clip( (2*p3+3*p2+p1+p0+q0+4)>>3 - p2, -tc2, tc2 )
        vadd.vv     v28, v8, v9        # p3+p2
        vsll.vi     v28, v28, 1        # 2*(p3+p2)
        vadd.vv     v29, v11, v12      # p0+q0
        vadd.vv     v28, v28, v29
        vadd.vv     v29, v9, v10       # p2+p1
        vadd.vv     v28, v28, v29
        vadd.vi     v28, v28, 4
        vsrl.vi     v28, v28, 3
        vsub.vv     v28, v28, v9

        # Q0' = q0 + Clip( (p1+2*p0+2*q0+2*q1+q2+4)>>3 - q0, -tc2, tc2 )
        vadd.vv     v29, v11, v12
        vadd.vv     v29, v29, v13
        vsll.vi     v29, v29, 1
        vadd.vv     v30, v10, v14
        vadd.vv     v29, v29, v30
        vadd.vi     v29, v29, 4
        vsrl.vi     v29, v29, 3
        vsub.vv     v29, v29, v12

        # Q1' = q1 + Clip( (p0+q0+q1+q2+2)>>2 - q1, -tc2, tc2 )
        vadd.vv     v30, v14, v13
        vadd.vv     v31, v11, v12
        vadd.vv     v30, v30, v31
        vadd.vi     v30, v30, 2
        vsrl.vi     v30, v30, 2
        vsub.vv     v30, v30, v13

        # Q2' = q2 + Clip( (2*q3+3*q2+q1+q0+p0+4)>>3 - q2, -tc2, tc2 )
        vadd.vv     v31, v15, v14
        vsll.vi     v31, v31, 1
        vadd.vv     v0,  v14, v13      # 复用 v0 做临时
        vadd.vv     v31, v31, v0
        vadd.vv     v0,  v11, v12
        vadd.vv     v31, v31, v0
        vadd.vi     v31, v31, 4
        vsrl.vi     v31, v31, 3
        vsub.vv     v31, v31, v14

        # 将各 delta 裁剪到 [-tc2, tc2]
        vmax.vv     v26, v26, v25
        vmin.vv     v26, v26, v24
        vmax.vv     v27, v27, v25
        vmin.vv     v27, v27, v24
        vmax.vv     v28, v28, v25
        vmin.vv     v28, v28, v24
        vmax.vv     v29, v29, v25
        vmin.vv     v29, v29, v24
        vmax.vv     v30, v30, v25
        vmin.vv     v30, v30, v24
        vmax.vv     v31, v31, v25
        vmin.vv     v31, v31, v24

        # 加上原始像素得到强滤波结果
        vadd.vv     v28, v28, v9       # P2' strong
        vadd.vv     v27, v27, v10      # P1' strong
        vadd.vv     v26, v26, v11      # P0' strong
        vadd.vv     v29, v29, v12      # Q0' strong
        vadd.vv     v30, v30, v13      # Q1' strong
        vadd.vv     v31, v31, v14      # Q2' strong

        # 若两组都是强滤波，则直接使用强滤波结果，无需合并
        li          t4, 0xFFFF
        bne         s3, t4, .Lmerge
        bne         s4, t4, .Lmerge
        vmv.v.v     v18, v28           # P2'
        vmv.v.v     v19, v27           # P1'
        vmv.v.v     v20, v26           # P0'
        vmv.v.v     v21, v29           # Q0'
        vmv.v.v     v22, v30           # Q1'
        vmv.v.v     v23, v31           # Q2'
        j           .Lfinal_store

.Lmerge:
        # ------- 根据掩码 v17 合并强/弱滤波结果 -------
        vmsne.vi    v0, v17, 0          # v0 中对应强滤波组的 lane 置1
        # 对于 P2：强 = v28，弱 = v9（原始值）
        vmerge.vvm  v1,  v9,  v28, v0
        # 对于 P1：强 = v27，弱 = v18（弱滤波结果）
        vmerge.vvm  v2,  v18, v27, v0
        # 对于 P0：强 = v26，弱 = v19
        vmerge.vvm  v3,  v19, v26, v0
        # 对于 Q0：强 = v29，弱 = v20
        vmerge.vvm  v4,  v20, v29, v0
        # 对于 Q1：强 = v30，弱 = v21
        vmerge.vvm  v5,  v21, v30, v0
        # 对于 Q2：强 = v31，弱 = v14
        vmerge.vvm  v6,  v14, v31, v0

        # 将合并结果转移到统一寄存器（v18~v23）
        vmv.v.v     v18, v1
        vmv.v.v     v19, v2
        vmv.v.v     v20, v3
        vmv.v.v     v21, v4
        vmv.v.v     v22, v5
        vmv.v.v     v23, v6

.Lfinal_store:
        # ---- 裁剪到 [0, 255] 并缩小为 8-bit ----
        li          t0, 255
        vmax.vx     v18, v18, zero     # 下限 0
        vmax.vx     v19, v19, zero
        vmax.vx     v20, v20, zero
        vmax.vx     v21, v21, zero
        vmax.vx     v22, v22, zero
        vmax.vx     v23, v23, zero
        vmin.vx     v18, v18, t0       # 上限 255
        vmin.vx     v19, v19, t0
        vmin.vx     v20, v20, t0
        vmin.vx     v21, v21, t0
        vmin.vx     v22, v22, t0
        vmin.vx     v23, v23, t0

        vsetivli    zero, 8, e8, mf2, ta, ma
        vnclipu.wi  v0, v8, 0          # P3 不变
        vnclipu.wi  v1, v18, 0         # P2'
        vnclipu.wi  v2, v19, 0         # P1'
        vnclipu.wi  v3, v20, 0         # P0'
        vnclipu.wi  v4, v21, 0         # Q0'
        vnclipu.wi  v5, v22, 0         # Q1'
        vnclipu.wi  v6, v23, 0         # Q2'
        vnclipu.wi  v7, v15, 0         # Q3 不变

        # ---------- 存储回内存 ----------
        mv          a0, s0              # 恢复起始地址
        vse8.v      v0, (a0)
        add         a0, a0, a1
        vse8.v      v1, (a0)
        add         a0, a0, a1
        vse8.v      v2, (a0)
        add         a0, a0, a1
        vse8.v      v3, (a0)
        add         a0, a0, a1
        vse8.v      v4, (a0)
        add         a0, a0, a1
        vse8.v      v5, (a0)
        add         a0, a0, a1
        vse8.v      v6, (a0)
        add         a0, a0, a1
        vse8.v      v7, (a0)

.Lexit:
        # ---------- 恢复寄存器并返回 ----------
        ld          ra, 104(sp)
        ld          s0,  96(sp)
        ld          s1,  88(sp)
        ld          s2,  80(sp)
        ld          s3,  72(sp)
        ld          s4,  64(sp)
        ld          s5,  56(sp)
        ld          s6,  48(sp)
        ld          s7,  40(sp)
        addi        sp, sp, 112
        ret
endfunc