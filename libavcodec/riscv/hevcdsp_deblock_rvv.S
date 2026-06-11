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

.macro transpose16_8x8 r0, r1, r2, r3, r4, r5, r6, r7, tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7,        tmp8, tmp9
        vid.v           \tmp9
        vand.vi         \tmp8, \tmp9, 0x1
        vmsne.vi        v0, \tmp8, 0

        // 0 8  2 10  4 12  6  14
        vslideup.vi     \tmp0, \r1, 1
        vmerge.vvm      \tmp4, \r0, \tmp0, v0
        // 16 24  18 26  20 28  22 30
        vslideup.vi     \tmp1, \r3, 1
        vmerge.vvm      \tmp5, \r2, \tmp1, v0

        // 32 40  34 42   36 44  38 46
        vslideup.vi     \tmp2, \r5, 1
        vmerge.vvm      \tmp6, \r4, \tmp2, v0
        // 48 56  50 58  52 60  54 62
        vslideup.vi     \tmp3, \r7, 1
        vmerge.vvm      \tmp7, \r6, \tmp3, v0

        // 1 9  3 11 5 13  7  15
        vslidedown.vi   \tmp0, \r0, 1
        vmerge.vvm      \r0, \tmp0, \r1, v0
        //17 25  19 27  21  29  23 31
        vslidedown.vi   \tmp1, \r2, 1
        vmerge.vvm      \r2, \tmp1, \r3, v0

        // 33 41  35 43   37 45  39 47
        vslidedown.vi   \tmp2, \r4, 1
        vmerge.vvm      \r4, \tmp2, \r5, v0
        // 49 57  51 59  53 61  55 63
        vslidedown.vi   \tmp3, \r6, 1
        vmerge.vvm      \r6, \tmp3, \r7, v0

        vand.vi         \tmp8, \tmp9, 0x2
        vmsne.vi        v0, \tmp8, 0

        // 0 8 16 24    4 12  20 28
        vslideup.vi     \tmp0, \tmp5, 2
        vmerge.vvm      \tmp0, \tmp4, \tmp0, v0
        // 32 40  48 56   36 44  52 60
        vslideup.vi     \tmp1, \tmp7, 2
        vmerge.vvm      \tmp1, \tmp6, \tmp1, v0

        // 2 10  18 26  6  14 22 30
        vslidedown.vi   \tmp2, \tmp4, 2
        vmerge.vvm      \tmp2, \tmp2, \tmp5, v0
        // 34 42  50 58  38 46  54 62
        vslidedown.vi   \tmp3, \tmp6, 2
        vmerge.vvm      \tmp3, \tmp3, \tmp7, v0

        // 1 9  17 25   5 13  21 29
        vslideup.vi     \r1, \r2, 2
        vmerge.vvm      \r1, \r0, \r1, v0
        // 33 41 49 57  37 45 53 61
        vslideup.vi     \r3, \r6, 2
        vmerge.vvm      \r3, \r4, \r3, v0

        // 3 11 19 27  7 15  23 31
        vslidedown.vi   \r5, \r0, 2
        vmerge.vvm      \r5, \r5, \r2, v0
        // 35 43 51 59  39  47 55 63
        vslidedown.vi   \r7, \r4, 2
        vmerge.vvm      \r7, \r7, \r6, v0

        vand.vi         \tmp8, \tmp9, 0x4
        vmsne.vi        v0, \tmp8, 0

        // 0 8 16 24    32 40 48 56
        vslideup.vi     \tmp4, \tmp1, 4
        vmerge.vvm      \tmp4, \tmp0, \tmp4, v0
        // 2 10  18 26   34 42  50 58
        vslideup.vi     \tmp5, \tmp3, 4
        vmerge.vvm      \tmp5, \tmp2, \tmp5, v0

        //4 12  20 28    36 44 52 60
        vslidedown.vi   \tmp6, \tmp0, 4
        vmerge.vvm      \tmp6, \tmp6, \tmp1, v0
        //6 14 22 30     38 46 54 62
        vslidedown.vi   \tmp7, \tmp2, 4
        vmerge.vvm      \tmp7, \tmp7, \tmp3, v0

        // 1 9  17 25   33 41 49 57
        vslideup.vi     \tmp0, \r3, 4
        vmerge.vvm      \tmp0, \r1, \tmp0, v0
        //  3 11 19 27  35 43 51 59
        vslideup.vi     \tmp1, \r7, 4
        vmerge.vvm      \tmp1, \r5, \tmp1, v0

        // 5 13  21 29  37 45 53 61
        vslidedown.vi   \tmp2, \r1, 4
        vmerge.vvm      \tmp2, \tmp2, \r3, v0
        // 7 15  23 31  39  47 55 63
        vslidedown.vi   \tmp3, \r5, 4
        vmerge.vvm      \tmp3, \tmp3, \r7, v0
.endm

.macro hevc_loop_filter_luma_body
       vmv.v.x     v16, a2            # beta 广播到所有向量元素

        # 构造 tC 向量：低4元素为 tc[0]，高4元素为 tc[1]
        vmv.v.x     v17, s1
        vmv.v.x     v24, s2
        vslideup.vi v17, v24, 4         # v17 = [tc0, tc0, tc0, tc0, tc1, tc1, tc1, tc1]

        # tc25 = (tc * 5 + 1) >> 1
        vsll.vi     v18, v17, 2         # v18 = 4 * tc
        vadd.vv     v18, v18, v17       # v18 = 5 * tc
        vadd.vi     v18, v18, 1
        vsra.vi     v19, v18, 1         # v19 = tc25

        # beta2 = beta >> 2
        vsra.vi     v20, v16, 2         # v20 = beta2

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
        vmv.x.s     t0, v23            # t0 = d0 of row 0
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
        beqz        t5, 4f         # 两组都不满足，直接退出

        # 生成 "滤波使能" 掩码 v26，低4元素使能 group 0，高4元素使能 group 1
        neg         t0, t0             # 将 0/1 转换为 0 / 0xFFFFFFFF（64位）
        li          t1, 0xFFFF
        and         t0, t0, t1         # 仅取低 16 位得到 0xFFFF 或 0
        neg         t2, t2
        and         t2, t2, t1
        vmv.v.x     v26, t0            # v26 所有元素 = t0 (group 0 标志)
        vmv.v.x     v25, t2            # v25 低4元素 = t2 (group 1 标志)
        vslideup.vi v26, v25, 4        # v26[4..7] = v25[0..3]，完成使能掩码

        # ----- 强滤波条件检查 -----
        vsra.vi     v27, v20, 1        # beta_3 = beta2 >> 1

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

        # 构造 "强/弱选择" 掩码 v1（与 NEON 版本作用相同）
        # 注意：不能覆盖 v26，它在弱滤波中作为组使能掩码
        vmv.v.x     v1, s3
        vmv.v.x     v25, s4
        vslideup.vi v1, v25, 4        # v1 = 每组的强/弱选择 (1=强, 0=弱)

        # ----- 若两组都是强滤波，则不需要计算 nd_p/nd_q -----
        li          t4, 0xFFFF
        bne         s3, t4, 0f
        bne         s4, t4, 0f
        j           1f

0:
        # 弱滤波需要用到 nd_p / nd_q，先计算阈值
        # threshold = (beta + (beta >> 1)) >> 3
        srli        t5, a2, 1
        add         t5, t5, a2
        srli        t5, t5, 3

        # 计算 group0 和 group1 的 dp 和 dq 之和（仅使用关键行0,3,4,7）
        # dp 和 dq 已分别位于 v21 和 v22 中
        
        vslidedown.vi v25, v21, 3
        vadd.vv      v25, v25, v21         # 
        vmv.x.s     t0, v25            # dp_sum0
        vslidedown.vi v2, v25, 4
        vmv.x.s     t1, v2            # dp_sum1

        vslidedown.vi v25, v22, 3
        vadd.vv      v25, v25, v22         # 
        vmv.x.s     t2, v25            # dp_sum0
        vslidedown.vi v2, v25, 4
        vmv.x.s     t3, v2            # dp_sum1

        # 比较得到 nd_p 和 nd_q 标志（1 表示满足条件需要修正）
        sltu        t0, t0, t5         # nd_p0
        sltu        t1, t1, t5         # nd_p1
        sltu        t2, t2, t5         # nd_q0
        sltu        t3, t3, t5         # nd_q1

        # 将 0/1 扩张为全位掩码 (0 或 0xFFFF)
        neg         t0, t0
        neg         t1, t1
        neg         t2, t2
        neg         t3, t3

        # 构造 nd_p 向量 v28（低4元素对应 group0，高4元素对应 group1）
        vmv.v.x     v28, t0
        vmv.v.x     v25, t1
        vslideup.vi v28, v25, 4        # v28 = nd_p 掩码

        # 构造 nd_q 向量 v29
        vmv.v.x     v29, t2
        vmv.v.x     v25, t3
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
        vsra.vi     v31, v17, 1        # tc_2 (使用算术右移)
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

1:
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
        vsra.vi     v26, v26, 3
        vsub.vv     v26, v26, v11      # delta P0

        # P1' = p1 + Clip( (p2+p1+p0+q0+2)>>2 - p1, -tc2, tc2 )
        vadd.vv     v27, v9, v10
        vadd.vv     v28, v11, v12
        vadd.vv     v27, v27, v28
        vadd.vi     v27, v27, 2
        vsra.vi     v27, v27, 2
        vsub.vv     v27, v27, v10

        # P2' = p2 + Clip( (2*p3+3*p2+p1+p0+q0+4)>>3 - p2, -tc2, tc2 )
        vadd.vv     v28, v8, v9        # p3+p2
        vsll.vi     v28, v28, 1        # 2*(p3+p2)
        vadd.vv     v29, v11, v12      # p0+q0
        vadd.vv     v28, v28, v29
        vadd.vv     v29, v9, v10       # p2+p1
        vadd.vv     v28, v28, v29
        vadd.vi     v28, v28, 4
        vsra.vi     v28, v28, 3
        vsub.vv     v28, v28, v9

        # Q0' = q0 + Clip( (p1+2*p0+2*q0+2*q1+q2+4)>>3 - q0, -tc2, tc2 )
        vadd.vv     v29, v11, v12
        vadd.vv     v29, v29, v13
        vsll.vi     v29, v29, 1
        vadd.vv     v30, v10, v14
        vadd.vv     v29, v29, v30
        vadd.vi     v29, v29, 4
        vsra.vi     v29, v29, 3
        vsub.vv     v29, v29, v12

        # Q1' = q1 + Clip( (p0+q0+q1+q2+2)>>2 - q1, -tc2, tc2 )
        vadd.vv     v30, v14, v13
        vadd.vv     v31, v11, v12
        vadd.vv     v30, v30, v31
        vadd.vi     v30, v30, 2
        vsra.vi     v30, v30, 2
        vsub.vv     v30, v30, v13

        # Q2' = q2 + Clip( (2*q3+3*q2+q1+q0+p0+4)>>3 - q2, -tc2, tc2 )
        vadd.vv     v31, v15, v14
        vsll.vi     v31, v31, 1
        vadd.vv     v0,  v14, v13      # 复用 v0 做临时
        vadd.vv     v31, v31, v0
        vadd.vv     v0,  v11, v12
        vadd.vv     v31, v31, v0
        vadd.vi     v31, v31, 4
        vsra.vi     v31, v31, 3
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
        bne         s3, t4, 2f
        bne         s4, t4, 2f
        vmv.v.v     v1, v28           # P2'
        vmv.v.v     v2, v27           # P1'
        vmv.v.v     v3, v26           # P0'
        vmv.v.v     v4, v29           # Q0'
        vmv.v.v     v5, v30           # Q1'
        vmv.v.v     v6, v31           # Q2'
        j           3f

2:
        # ------- 根据掩码 v25 合并强/弱滤波结果 -------
        vmsne.vi    v0, v1, 0          # v0 中对应强滤波组的 lane 置1
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

3:
        # ---- 裁剪到 [0, 255] 并缩小为 8-bit ----
        li          t0, 255
        vmax.vx     v18, v1, zero     # 下限 0
        vmax.vx     v19, v2, zero
        vmax.vx     v20, v3, zero
        vmax.vx     v21, v4, zero
        vmax.vx     v22, v5, zero
        vmax.vx     v23, v6, zero
.endm

.macro hevc_loop_filter_luma_8_rvv dir
func ff_hevc_\dir\()_loop_filter_luma_8_rvv, zve32x
        # ---------- 保存被调用者保存的寄存器 ----------
        addi        sp, sp, -88
        sd          ra,  80(sp)
        sd          s0,  72(sp)
        sd          s1,  64(sp)
        sd          s2,  56(sp)
        sd          s3,  48(sp)
        sd          s4,  40(sp)
.ifc \dir, v
        # ---- 指针调整：pix = a0-4 ----
        # 垂直滤波时，以当前列为基准边界左侧 4 列
        addi        a0, a0, -4
.else
        # ---- 指针调整：pix -= 4 * stride ----
        # 水平滤波时，以当前行为基准向上移动4行，使后续8行加载能覆盖到
        # P侧的上方像素和Q侧的下方像素。
        slli        t0, a1, 2
        sub         a0, a0, t0
.endif
        # ---------- 准备参数 ----------
        mv          s0, a0              # s0 保存滤波起始地址，后续存储时使用
        lw          s1, 0(a3)          # tc[0]
        lw          s2, 4(a3)          # tc[1]
        or          t0, s1, s2
        beqz        t0, 4f         # 若两个 tC 都为0，则无需滤波，直接退出

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
.ifc \dir, v
        # ---- 将所有像素扩展为 16-bit，方便后续运算 ----
        vsetivli    zero, 8, e16, m1, ta, ma
        vzext.vf2   v16,  v0   # P3
        vzext.vf2   v17,  v1   # P2
        vzext.vf2   v18, v2   # P1
        vzext.vf2   v19, v3   # P0
        vzext.vf2   v20, v4   # Q0
        vzext.vf2   v21, v5   # Q1
        vzext.vf2   v22, v6   # Q2
        vzext.vf2   v23, v7   # Q3

        # ---- 转置 8x8，使垂直边界变成水平边界 ----
        transpose16_8x8 v16, v17, v18, v19, v20, v21, v22, v23, v9, v11, v13, v15, v8, v10, v12, v14, v1, v2
.else
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
.endif
        hevc_loop_filter_luma_body
.ifc \dir, v
        # ---- 转置 8x8 并窄化----
        transpose16_8x8 v8, v18, v19, v20, v21, v22, v23, v15, v25, v27, v29, v31, v24, v26, v28, v30, v1, v2

        vsetivli    zero, 8, e8, mf2, ta, ma
        vnclipu.wi  v0, v24, 0         # P3'
        vnclipu.wi  v1, v25, 0         # P2'
        vnclipu.wi  v2, v26, 0         # P1'
        vnclipu.wi  v3, v27, 0         # P0'
        vnclipu.wi  v4, v28, 0         # Q0'
        vnclipu.wi  v5, v29, 0         # Q1'
        vnclipu.wi  v6, v30, 0         # Q2'
        vnclipu.wi  v7, v31, 0         # Q3'

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
.else
        vsetivli    zero, 8, e8, mf2, ta, ma
        vnclipu.wi  v1, v18, 0         # P2'
        vnclipu.wi  v2, v19, 0         # P1'
        vnclipu.wi  v3, v20, 0         # P0'
        vnclipu.wi  v4, v21, 0         # Q0'
        vnclipu.wi  v5, v22, 0         # Q1'
        vnclipu.wi  v6, v23, 0         # Q2'

        # ---------- 存储回内存 ----------
        mv          a0, s0              # 恢复起始地址
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
.endif
4:
        # ---------- 恢复寄存器并返回 ----------
        ld          ra,  80(sp)
        ld          s0,  72(sp)
        ld          s1,  64(sp)
        ld          s2,  56(sp)
        ld          s3,  48(sp)
        ld          s4,  40(sp)
        addi        sp, sp, 88
        ret
endfunc
.endm

hevc_loop_filter_luma_8_rvv h
hevc_loop_filter_luma_8_rvv v

.macro hevc_loop_filter_chroma_8_rvv dir
func ff_hevc_\dir\()_loop_filter_chroma_8_rvv, zve32x
        # 加载 tc[0] 和 tc[1]
        lw          t0, 0(a2)
        lw          t1, 4(a2)
        or          t2, t0, t1
        beqz        t2, 1f              # 如果两个tc都为0，跳过滤波

.ifc \dir, v
        # 调整指针: pix -= 2 (指向左侧2列，即p1列)
        addi        a0, a0, -2
.else
        # 调整指针: pix -= 2 * stride (指向 p1 行)
        slli        t2, a1, 1
        sub         a0, a0, t2
.endif
        # 保存起始地址
        mv          t3, a0

        # 加载4列/行像素: p1, p0, q0, q1
        vsetivli    zero, 8, e8, m1, ta, ma
.ifc \dir, v
        vlse8.v     v0, (a0), a1        # p1 列
        addi        a0, a0, 1
        vlse8.v     v1, (a0), a1        # p0 列
        addi        a0, a0, 1
        vlse8.v     v2, (a0), a1        # q0 列
        addi        a0, a0, 1
        vlse8.v     v3, (a0), a1        # q1 列
.else
        vle8.v      v0, (a0)            # p1
        add         a0, a0, a1
        vle8.v      v1, (a0)            # p0
        add         a0, a0, a1
        vle8.v      v2, (a0)            # q0
        add         a0, a0, a1
        vle8.v      v3, (a0)            # q1
.endif
        # 扩展到16-bit进行计算
        vsetivli    zero, 8, e16, m1, ta, ma
        vzext.vf2   v20, v0             # p1 (16-bit)
        vzext.vf2   v21, v1             # p0 (16-bit)
        vzext.vf2   v22, v2             # q0 (16-bit)
        vzext.vf2   v23, v3             # q1 (16-bit)

        # 构造 tc 向量: v16 = [tc0, tc0, tc0, tc0, tc1, tc1, tc1, tc1]
        vmv.v.x     v16, t0
        vmv.v.x     v17, t1
        vslideup.vi v16, v17, 4

        # -tc 用于裁剪
        vneg.v      v17, v16

        # delta = ((q0 - p0) << 2) + (p1 - q1) + 4) >> 3
        # HEVC 规范要求舍入右移
        vsub.vv     v24, v22, v21       # q0 - p0
        vsll.vi     v24, v24, 2         # (q0 - p0) << 2
        vsub.vv     v25, v20, v23       # p1 - q1
        vadd.vv     v24, v24, v25       # delta = ((q0-p0) << 2) + (p1-q1)
        vadd.vi     v24, v24, 4         # + 4 用于舍入
        vsra.vi     v24, v24, 3         # delta >>= 3 (舍入右移)

        # 裁剪 delta 到 [-tc, tc]
        vmax.vv     v24, v24, v17       # delta = max(delta, -tc)
        vmin.vv     v24, v24, v16       # delta = min(delta, tc)

        # p0' = Clip(p0 + delta)
        # q0' = Clip(q0 - delta)
        vadd.vv     v21, v21, v24       # p0 + delta
        vsub.vv     v22, v22, v24       # q0 - delta

        # 裁剪到 [0, 255]
        vmax.vx     v21, v21, zero
        vmax.vx     v22, v22, zero

        # 窄化回8-bit
        vsetivli    zero, 8, e8, mf2, ta, ma
        vnclipu.wi  v1, v21, 0          # p0' (8-bit)
        vnclipu.wi  v2, v22, 0          # q0' (8-bit)

.ifc \dir, v
        # 存储回内存（列存储）
        addi        a0, t3, 1           # 指向 p0 列
        vsse8.v     v1, (a0), a1        # 存储 p0' 列
        addi        a0, a0, 1           # 指向 q0 列
        vsse8.v     v2, (a0), a1        # 存储 q0' 列
.else
        # 存储 p0' 和 q0'
        add         a0, t3, a1          # 指向 p0 行
        vse8.v      v1, (a0)
        add         a0, a0, a1          # 指向 q0 行
        vse8.v      v2, (a0)
.endif

1:
        ret
endfunc
.endm

hevc_loop_filter_chroma_8_rvv h
hevc_loop_filter_chroma_8_rvv v
