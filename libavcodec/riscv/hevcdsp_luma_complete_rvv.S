/*
 * Copyright (c) 2026 Institute of Software, Chinese Academy of Sciences (ISCAS).
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

/* ============================================================================
 * HEVC Luma Loop Filter - RISC-V Vector (RVV) Implementation
 * 8-bit depth, based on FFmpeg AArch64 NEON reference
 *
 * Register conventions:
 *   v0  = P3 (16-bit expanded)
 *   v1  = P2
 *   v2  = P1
 *   v3  = P0
 *   v4  = Q0
 *   v5  = Q1
 *   v6  = Q2
 *   v7  = Q3
 *   v8  = dp (abs(P2 - 2*P1 + P0))
 *   v9  = dq (abs(Q2 - 2*Q1 + Q0))
 *   v10 = d0 (dp + dq)
 *   v16-v23 = temporaries / saved originals
 *   v24-v31 = filter computation temps
 * ============================================================================ */

/* ============================================================================
 * Helper: Absolute difference of two vectors
 * result = abs(a - b)
 * ============================================================================ */
.macro vabsd vd, va, vb, vtmp
        vsub.vv         \vtmp, \va, \vb
        vsub.vv         \vd,   \vb, \va
        vmax.vv         \vd,   \vd, \vtmp
.endm

/* ============================================================================
 * Helper: Clip vector to [min_reg, max_reg]
 * ============================================================================ */
.macro clip_vv vd, vmin, vmax
        vmax.vv         \vd, \vd, \vmin
        vmin.vv         \vd, \vd, \vmax
.endm

.macro clip_scalar vd, min_r, max_r
        vmax.vx         \vd, \vd, \min_r
        vmin.vx         \vd, \vd, \max_r
.endm

/* ============================================================================
 * Strong Filter
 *
 * Inputs:
 *   v0=P3, v1=P2, v2=P1, v3=P0, v4=Q0, v5=Q1, v6=Q2, v7=Q3 (16-bit)
 *   a_tc0 = tc for lower 4 lanes (scalar)
 *   a_tc1 = tc for upper 4 lanes (scalar, may equal a_tc0)
 *   v_tc   = per-lane tc vector (16-bit, already set by caller)
 *   v_tc2  = per-lane tc*2 vector
 *   v_tc3  = per-lane tc*3 vector
 *
 * Outputs (16-bit, unclipped to [0,255]):
 *   v24 = P0'
 *   v25 = P1'
 *   v26 = P2'
 *   v27 = Q0'
 *   v28 = Q1'
 *   v29 = Q2'
 *
 * Temporaries: v30, v31
 * ============================================================================ */
.macro strong_filter_body v_tc, v_tc2, v_tc3

        /* ------------------------------------------------------------------
         * P0' = p0 + clip((p2 + 2*p1 + 2*p0 + 2*q0 + q1 + 4) >> 3 - p0,
         *                  -tc3, tc3)
         * ------------------------------------------------------------------ */
        vadd.vv         v24, v2, v3         /* p1 + p0 */
        vadd.vv         v24, v4, v24        /* + q0 */
        vadd.vv         v24, v24, v24       /* * 2  (shift left 1) */
        vadd.vv         v30, v1, v5         /* p2 + q1 */
        vadd.vv         v24, v30, v24       /* full sum */
        vadd.vi         v24, v24, 4
        vsra.vi         v24, v24, 3         /* >> 3 (arithmetic, rounds toward -inf) */
        vsub.vv         v24, v24, v3        /* delta_P0 */
        /* clip delta_P0 to [-tc3, tc3] */
        vneg.v          v31, \v_tc3
        clip_vv         v24, v31, \v_tc3
        vadd.vv         v24, v3, v24        /* P0' */

        /* ------------------------------------------------------------------
         * P1' = p1 + clip((p2 + p1 + p0 + q0 + 2) >> 2 - p1, -tc2, tc2)
         * ------------------------------------------------------------------ */
        vadd.vv         v25, v1, v2         /* p2 + p1 */
        vadd.vv         v30, v3, v4         /* p0 + q0 */
        vadd.vv         v25, v25, v30
        vadd.vi         v25, v25, 2
        vsra.vi         v25, v25, 2
        vsub.vv         v25, v25, v2        /* delta_P1 */
        vneg.v          v31, \v_tc2
        clip_vv         v25, v31, \v_tc2
        vadd.vv         v25, v2, v25        /* P1' */

        /* ------------------------------------------------------------------
         * P2' = p2 + clip((2*p3 + 3*p2 + p1 + p0 + q0 + 4) >> 3 - p2,
         *                  -tc, tc)
         * Note: 2*p3 + 3*p2 = 2*(p3+p2) + p2
         * ------------------------------------------------------------------ */
        vadd.vv         v26, v0, v1         /* p3 + p2 */
        vadd.vv         v26, v26, v26       /* * 2 */
        vadd.vv         v30, v3, v4         /* p0 + q0 */
        vadd.vv         v26, v26, v30
        vadd.vv         v30, v1, v2         /* p2 + p1 */
        vadd.vv         v26, v26, v30
        vadd.vi         v26, v26, 4
        vsra.vi         v26, v26, 3
        vsub.vv         v26, v26, v1        /* delta_P2 */
        vneg.v          v31, \v_tc
        clip_vv         v26, v31, \v_tc
        vadd.vv         v26, v1, v26        /* P2' */

        /* ------------------------------------------------------------------
         * Q0' = q0 + clip((p1 + 2*p0 + 2*q0 + 2*q1 + q2 + 4) >> 3 - q0,
         *                  -tc3, tc3)
         * ------------------------------------------------------------------ */
        vadd.vv         v27, v3, v4         /* p0 + q0 */
        vadd.vv         v27, v5, v27        /* + q1 */
        vadd.vv         v27, v27, v27       /* * 2 */
        vadd.vv         v30, v2, v6         /* p1 + q2 */
        vadd.vv         v27, v30, v27
        vadd.vi         v27, v27, 4
        vsra.vi         v27, v27, 3
        vsub.vv         v27, v27, v4        /* delta_Q0 */
        vneg.v          v31, \v_tc3
        clip_vv         v27, v31, \v_tc3
        vadd.vv         v27, v4, v27        /* Q0' */

        /* ------------------------------------------------------------------
         * Q1' = q1 + clip((p0 + q0 + q1 + q2 + 2) >> 2 - q1, -tc2, tc2)
         * ------------------------------------------------------------------ */
        vadd.vv         v28, v5, v6         /* q1 + q2 */
        vadd.vv         v30, v3, v4         /* p0 + q0 */
        vadd.vv         v28, v28, v30
        vadd.vi         v28, v28, 2
        vsra.vi         v28, v28, 2
        vsub.vv         v28, v28, v5        /* delta_Q1 */
        vneg.v          v31, \v_tc2
        clip_vv         v28, v31, \v_tc2
        vadd.vv         v28, v5, v28        /* Q1' */

        /* ------------------------------------------------------------------
         * Q2' = q2 + clip((2*q3 + 3*q2 + q1 + q0 + p0 + 4) >> 3 - q2,
         *                  -tc, tc)
         * ------------------------------------------------------------------ */
        vadd.vv         v29, v7, v6         /* q3 + q2 */
        vadd.vv         v29, v29, v29       /* * 2 */
        vadd.vv         v30, v6, v5         /* q2 + q1 */
        vadd.vv         v29, v29, v30
        vadd.vv         v30, v3, v4         /* p0 + q0 */
        vadd.vv         v29, v29, v30
        vadd.vi         v29, v29, 4
        vsra.vi         v29, v29, 3
        vsub.vv         v29, v29, v6        /* delta_Q2 */
        vneg.v          v31, \v_tc
        clip_vv         v29, v31, \v_tc
        vadd.vv         v29, v6, v29        /* Q2' */

.endm

/* ============================================================================
 * Weak Filter Body
 *
 * Inputs:
 *   v0=P3, v1=P2, v2=P1, v3=P0, v4=Q0, v5=Q1, v6=Q2, v7=Q3 (16-bit)
 *   v_tc  = per-lane tc  (16-bit)
 *   v_tc2 = per-lane tc/2 (16-bit, tc>>1)
 *   v_ndp = per-lane nd_p mask (all-ones = apply P1 update)
 *   v_ndq = per-lane nd_q mask (all-ones = apply Q1 update)
 *
 * Outputs (16-bit):
 *   v24 = P0'
 *   v25 = Q0'
 *   v26 = P1' (only valid where nd_p true)
 *   v27 = Q1' (only valid where nd_q true)
 *   v28 = delta0 (saved for caller)
 *
 * Temporaries: v29, v30, v31
 * ============================================================================ */
.macro weak_filter_body v_tc, v_tc_half, v_ndp, v_ndq

        /* ------------------------------------------------------------------
         * delta0 = (9*(q0-p0) - 3*(q1-p1) + 8) >> 4
         *
         * For 8-bit: intermediate fits in 16-bit
         *   max |q0-p0| = 255,  9*255 = 2295  < 32767  OK
         *   max |q1-p1| = 255,  3*255 = 765   < 32767  OK
         * ------------------------------------------------------------------ */
        vsub.vv         v28, v4, v3         /* q0 - p0 */
        vadd.vv         v29, v28, v28       /* * 2 */
        vadd.vv         v29, v29, v29       /* * 4 */
        vadd.vv         v29, v29, v29       /* * 8 */
        vadd.vv         v29, v29, v28       /* * 9 = 8x + x */

        vsub.vv         v30, v5, v2         /* q1 - p1 */
        vadd.vv         v31, v30, v30       /* * 2 */
        vadd.vv         v31, v31, v30       /* * 3 */

        vsub.vv         v29, v29, v31       /* 9*(q0-p0) - 3*(q1-p1) */
        vadd.vi         v29, v29, 8
        vsra.vi         v28, v29, 4         /* delta0 (saved in v28) */

        /* ------------------------------------------------------------------
         * Check: abs(delta0) < 10*tc
         * If not, suppress this pixel entirely (delta0 = 0)
         * ------------------------------------------------------------------ */
        /* 10*tc = 8*tc + 2*tc */
        vadd.vv         v29, \v_tc, \v_tc   /* 2*tc */
        vadd.vv         v29, v29, v29       /* 4*tc */
        vadd.vv         v30, v29, v29       /* 8*tc */
        vadd.vv         v30, v30, v29       /* 10*tc  (v29=4tc still) */

        /* abs(delta0) */
        vneg.v          v31, v28
        vmax.vv         v31, v31, v28       /* abs(delta0) */

        /* mask: abs(delta0) < 10*tc  =>  valid_mask */
        vmslt.vv        v0_m, v31, v30      /* borrow mask reg; v0_m = (abs<10tc) */
        /* zero out delta0 where invalid */
        vmerge.vxm      v28, v28, zero, v0_m   /* delta0 masked */
        /* Note: RVV has vmerge which uses mask; where mask=1 use scalar,
         * so we invert: where valid keep v28, where invalid zero.
         * Use vmxnor / complement approach: */
        /*
         * Correct idiom:
         *   vmv.v.x  v31, 0
         *   vmerge.vvm v28, v31, v28, v0_m   -- where mask=1 take v28, else 0
         */
        vmv.v.x         v31, 0
        vmerge.vvm      v28, v31, v28, v0_m  /* delta0 = valid ? delta0 : 0 */

        /* ------------------------------------------------------------------
         * clip delta0 to [-tc, tc]
         * ------------------------------------------------------------------ */
        vneg.v          v31, \v_tc
        clip_vv         v28, v31, \v_tc     /* delta0 clipped */

        /* ------------------------------------------------------------------
         * P0' = clip(p0 + delta0, 0, 255)
         * Q0' = clip(q0 - delta0, 0, 255)
         * ------------------------------------------------------------------ */
        vadd.vv         v24, v3, v28        /* P0' (pre pixel-clip) */
        vsub.vv         v25, v4, v28        /* Q0' (pre pixel-clip) */

        /* ------------------------------------------------------------------
         * deltap1 = clip((((p2+p0+1)>>1) - p1 + delta0) >> 1, -tc/2, tc/2)
         * ------------------------------------------------------------------ */
        vadd.vv         v26, v1, v3         /* p2 + p0 */
        vadd.vi         v26, v26, 1
        vsra.vi         v26, v26, 1         /* (p2+p0+1)>>1 */
        vsub.vv         v26, v26, v2        /* - p1 */
        vadd.vv         v26, v26, v28       /* + delta0 */
        vsra.vi         v26, v26, 1         /* >> 1 */
        vneg.v          v31, \v_tc_half
        clip_vv         v26, v31, \v_tc_half
        /* apply nd_p mask: deltap1 = nd_p ? deltap1 : 0 */
        vmv.v.x         v29, 0
        vmerge.vvm      v26, v29, v26, \v_ndp   /* where nd_p=true keep deltap1 */
        vadd.vv         v26, v2, v26        /* P1' = p1 + deltap1 */

        /* ------------------------------------------------------------------
         * deltaq1 = clip((((q2+q0+1)>>1) - q1 - delta0) >> 1, -tc/2, tc/2)
         * ------------------------------------------------------------------ */
        vadd.vv         v27, v6, v4         /* q2 + q0 */
        vadd.vi         v27, v27, 1
        vsra.vi         v27, v27, 1         /* (q2+q0+1)>>1 */
        vsub.vv         v27, v27, v5        /* - q1 */
        vsub.vv         v27, v27, v28       /* - delta0 */
        vsra.vi         v27, v27, 1
        vneg.v          v31, \v_tc_half
        clip_vv         v27, v31, \v_tc_half
        /* apply nd_q mask */
        vmerge.vvm      v27, v29, v27, \v_ndq
        vadd.vv         v27, v5, v27        /* Q1' = q1 + deltaq1 */

.endm

/* ============================================================================
 * Build per-lane tc vectors from two scalar tc values (tc0, tc1).
 *
 * Convention: lanes 0-3 use tc0, lanes 4-7 use tc1.
 *
 * We build in a temp area on the stack (16 bytes for 8 x int16).
 * Caller must have vsetivli 8, e16 active.
 *
 * After macro: v_tc, v_tc2, v_tc3, v_tc_half are ready.
 * Clobbers: t4, t5.
 * ============================================================================ */
.macro build_tc_vectors v_tc, v_tc2, v_tc3, v_tc_half, tc0, tc1, stack_tmp

        /* store tc0 into first 8 bytes (4 x int16), tc1 into next 8 bytes */
        /* We use a small scalar loop to fill; stack_tmp must be 16-byte aligned */
        addi            t4, \stack_tmp, 0
        sh              \tc0, 0(t4)
        sh              \tc0, 2(t4)
        sh              \tc0, 4(t4)
        sh              \tc0, 6(t4)
        sh              \tc1, 8(t4)
        sh              \tc1, 10(t4)
        sh              \tc1, 12(t4)
        sh              \tc1, 14(t4)

        vle16.v         \v_tc, (t4)

        /* tc2 = tc * 2 */
        vadd.vv         \v_tc2, \v_tc, \v_tc

        /* tc3 = tc * 3 = tc*2 + tc */
        vadd.vv         \v_tc3, \v_tc2, \v_tc

        /* tc_half = tc >> 1  (arithmetic) */
        vsra.vi         \v_tc_half, \v_tc, 1

.endm

/* ============================================================================
 * Condition evaluation
 *
 * Computes per-lane masks:
 *   skip_mask   : lanes where d0+d3 >= beta  (no filter needed)
 *   strong_mask : lanes that pass all strong-filter conditions
 *   nd_p_mask   : lanes where dp_sum < beta_adj
 *   nd_q_mask   : lanes where dq_sum < beta_adj
 *
 * Inputs (v0-v7 = P3..Q3, 16-bit, 8 lanes):
 *   s_beta  = beta (scalar)
 *   v_tc    = per-lane tc (16-bit)
 *   v_tc25  = per-lane (5*tc+1)>>1 (16-bit)
 *
 * Outputs (as mask registers, 1 bit per lane):
 *   skip_m, strong_m, ndp_m, ndq_m
 *
 * Temporaries: v8-v15, v30, v31
 * Clobbers: t4, t5
 * ============================================================================ */
.macro compute_conditions skip_m, strong_m, ndp_m, ndq_m, \
                           s_beta, v_tc, v_tc25

        /* ------------------------------------------------------------------
         * dp = abs(P2 - 2*P1 + P0)
         * dq = abs(Q2 - 2*Q1 + Q0)
         * ------------------------------------------------------------------ */
        vadd.vv         v8,  v1, v3         /* P2 + P0 */
        vadd.vv         v30, v2, v2         /* 2*P1 */
        vabsd           v8,  v8, v30, v31   /* dp  (v31 = tmp) */

        vadd.vv         v9,  v6, v4         /* Q2 + Q0 */
        vadd.vv         v30, v5, v5         /* 2*Q1 */
        vabsd           v9,  v9, v30, v31   /* dq */

        /* d0 = dp + dq */
        vadd.vv         v10, v8, v9

        /* ------------------------------------------------------------------
         * Replicate d0 across pairs:
         *   lanes 0-3: d0_lo = d[0] + d[1] + d[2] + d[3]  (sum of 4)
         *   lanes 4-7: d0_hi = d[4] + d[5] + d[6] + d[7]
         * Actually HEVC checks d0+d3 where d0 = d for first sample pair
         * and d3 = d for last sample pair within a 4-pixel block.
         *
         * The reference groups 8 pixels into two 4-pixel sub-blocks:
         *   block A: pixels 0-3,  tc = tc[0], beta check uses d[0]+d[3]
         *   block B: pixels 4-7,  tc = tc[1], beta check uses d[4]+d[7]
         *
         * We sum d across each 4-pixel block and broadcast.
         * ------------------------------------------------------------------ */
        /* Sum d0 for lanes 0-3 */
        vmv.x.s         t4, v10             /* lane 0 */
        /* slide to extract lane 1, 2, 3 */
        vslidedown.vi   v30, v10, 1
        vmv.x.s         t5, v30
        add             t4, t4, t5
        vslidedown.vi   v30, v10, 2
        vmv.x.s         t5, v30
        add             t4, t4, t5
        vslidedown.vi   v30, v10, 3
        vmv.x.s         t5, v30
        add             t4, t4, t5          /* t4 = d_sum_lo */

        /* Sum d0 for lanes 4-7 */
        vslidedown.vi   v30, v10, 4
        vmv.x.s         t5, v30
        vslidedown.vi   v31, v10, 5
        vmv.x.s         a6, v31             /* borrow a6 temporarily */
        add             t5, t5, a6
        vslidedown.vi   v31, v10, 6
        vmv.x.s         a6, v31
        add             t5, t5, a6
        vslidedown.vi   v31, v10, 7
        vmv.x.s         a6, v31
        add             t5, t5, a6          /* t5 = d_sum_hi */

        /* Build broadcast vectors for the sums */
        /* v11 = [t4 t4 t4 t4 t5 t5 t5 t5] */
        sh              t4, 0(sp)           /* use existing stack space */
        sh              t4, 2(sp)
        sh              t4, 4(sp)
        sh              t4, 6(sp)
        sh              t5, 8(sp)
        sh              t5, 10(sp)
        sh              t5, 12(sp)
        sh              t5, 14(sp)
        vle16.v         v11, (sp)           /* d_sum broadcast */

        /* ------------------------------------------------------------------
         * skip_mask: d_sum >= beta  =>  no filter
         * We want: filter if d_sum < beta
         * skip_m = (d_sum >= beta)
         * ------------------------------------------------------------------ */
        vmsgeu.vx       \skip_m, v11, \s_beta   /* skip if d_sum >= beta */

        /* ------------------------------------------------------------------
         * beta_2 = beta >> 2,  beta_3 = beta >> 3
         * ------------------------------------------------------------------ */
        srli            t4, \s_beta, 2      /* beta_2 */
        srli            t5, \s_beta, 3      /* beta_3 */

        /* ------------------------------------------------------------------
         * Strong condition 1: (d0*2) < beta_2
         *   i.e. per-lane 2*d0 < beta/4
         * ------------------------------------------------------------------ */
        vadd.vv         v12, v10, v10       /* 2*d0 */
        vmsltu.vx       v0_m, v12, t4       /* 2*d0 < beta_2 */

        /* ------------------------------------------------------------------
         * Strong condition 2: abs(P3-P0) + abs(Q3-Q0) < beta_3
         * ------------------------------------------------------------------ */
        vabsd           v13, v0, v3, v31    /* abs(P3-P0) */
        vabsd           v14, v7, v4, v31    /* abs(Q3-Q0) */
        vadd.vv         v13, v13, v14
        vmsltu.vx       v1_m, v13, t5       /* sum < beta_3 */
        vmand.mm        v0_m, v0_m, v1_m

        /* ------------------------------------------------------------------
         * Strong condition 3: abs(P0-Q0) < tc25
         * ------------------------------------------------------------------ */
        vabsd           v14, v3, v4, v31
        vmslt.vv        v1_m, v14, \v_tc25  /* abs(P0-Q0) < tc25 */
        vmand.mm        \strong_m, v0_m, v1_m

        /* ------------------------------------------------------------------
         * nd_p: dp_sum < (beta + beta>>1) >> 3
         * dp_sum = dp[0..3 sum] for lo block, dp[4..7 sum] for hi block
         * ------------------------------------------------------------------ */
        /* Compute dp_sum per block (same approach as d_sum) */
        vmv.x.s         t4, v8
        vslidedown.vi   v30, v8, 1 ; vmv.x.s t5, v30 ; add t4, t4, t5
        vslidedown.vi   v30, v8, 2 ; vmv.x.s t5, v30 ; add t4, t4, t5
        vslidedown.vi   v30, v8, 3 ; vmv.x.s t5, v30 ; add t4, t4, t5
        /* dp_sum_lo in t4 */
        vslidedown.vi   v30, v8, 4 ; vmv.x.s a6, v30
        vslidedown.vi   v31, v8, 5 ; vmv.x.s t5, v31 ; add a6, a6, t5
        vslidedown.vi   v31, v8, 6 ; vmv.x.s t5, v31 ; add a6, a6, t5
        vslidedown.vi   v31, v8, 7 ; vmv.x.s t5, v31 ; add a6, a6, t5
        /* dp_sum_hi in a6 */

        sh              t4, 0(sp)
        sh              t4, 2(sp)
        sh              t4, 4(sp)
        sh              t4, 6(sp)
        sh              a6, 8(sp)
        sh              a6, 10(sp)
        sh              a6, 12(sp)
        sh              a6, 14(sp)
        vle16.v         v15, (sp)           /* dp_sum broadcast */

        /* threshold = (beta + beta>>1) >> 3 */
        srli            t4, \s_beta, 1
        add             t4, t4, \s_beta
        srli            t4, t4, 3
        vmsltu.vx       \ndp_m, v15, t4

        /* nd_q similarly */
        vmv.x.s         t4, v9
        vslidedown.vi   v30, v9, 1 ; vmv.x.s t5, v30 ; add t4, t4, t5
        vslidedown.vi   v30, v9, 2 ; vmv.x.s t5, v30 ; add t4, t4, t5
        vslidedown.vi   v30, v9, 3 ; vmv.x.s t5, v30 ; add t4, t4, t5
        vslidedown.vi   v30, v9, 4 ; vmv.x.s a6, v30
        vslidedown.vi   v31, v9, 5 ; vmv.x.s t5, v31 ; add a6, a6, t5
        vslidedown.vi   v31, v9, 6 ; vmv.x.s t5, v31 ; add a6, a6, t5
        vslidedown.vi   v31, v9, 7 ; vmv.x.s t5, v31 ; add a6, a6, t5

        sh              t4, 0(sp)
        sh              t4, 2(sp)
        sh              t4, 4(sp)
        sh              t4, 6(sp)
        sh              a6, 8(sp)
        sh              a6, 10(sp)
        sh              a6, 12(sp)
        sh              a6, 14(sp)
        vle16.v         v15, (sp)
        vmsltu.vx       \ndq_m, v15, t4

.endm

/* ============================================================================
 * Pixel clip to [0,255] and narrow to 8-bit
 * ============================================================================ */
.macro pixel_clip_narrow v_out8, v_in16
        vmax.vx         v_in16, v_in16, zero
        li              t4, 255
        vmin.vx         v_in16, v_in16, t4
        vnclipu.wi      \v_out8, v_in16, 0
.endm

/* ============================================================================
 * ff_hevc_v_loop_filter_luma_8_rvv
 *
 * C signature:
 *   void ff_hevc_v_loop_filter_luma_8(uint8_t *pix, ptrdiff_t stride,
 *                                     int beta, const int16_t *tc,
 *                                     uint8_t no_p, uint8_t no_q)
 *
 * For vertical filter (filtering along a vertical edge):
 *   - pix points to the first sample of the boundary column
 *   - We read/write 8 rows, each row: [P3 P2 P1 P0 Q0 Q1 Q2 Q3]
 *   - adjust pix back by 4 to start at P3
 *
 * Stack layout (sp after prologue):
 *   [sp+0  .. sp+15]   16 bytes scratch for tc vector build
 *   [sp+16 .. sp+63]   saved registers
 * ============================================================================ */
        .text
        .balign 4

        .global ff_hevc_v_loop_filter_luma_8_rvv
ff_hevc_v_loop_filter_luma_8_rvv:

        /* ----------------------------------------------------------------
         * Prologue
         * ---------------------------------------------------------------- */
        addi            sp, sp, -80
        sd              ra,  72(sp)
        sd              s0,  64(sp)
        sd              s1,  56(sp)
        sd              s2,  48(sp)
        sd              s3,  40(sp)
        sd              s4,  32(sp)
        sd              s5,  24(sp)
        sd              s6,  16(sp)
        /* sp+0 .. sp+15 : 16 bytes scratch */

        mv              s0, a0              /* pix */
        mv              s1, a1              /* stride */
        mv              s2, a2              /* beta */
        mv              s3, a3              /* tc array */
        mv              s4, a4              /* no_p */
        mv              s5, a5              /* no_q */

        /* Early exit if both tc[0] and tc[1] are zero */
        lh              t0, 0(s3)           /* tc[0] (signed) */
        lh              t1, 2(s3)           /* tc[1] */
        or              t2, t0, t1
        beqz            t2, .Lv_exit

        /* ----------------------------------------------------------------
         * Adjust pointer: pix -= 4  (to reach P3 column)
         * For 8-bit, one pixel = 1 byte, so sub 4.
         * ---------------------------------------------------------------- */
        addi            s0, s0, -4

        /* ----------------------------------------------------------------
         * Load 8 rows of 8 bytes each (P3 P2 P1 P0 Q0 Q1 Q2 Q3)
         * into v0-v7 as 8-bit.
         * ---------------------------------------------------------------- */
        vsetivli        zero, 8, e8, m1, ta, ma

        mv              t3, s0
        vle8.v          v0, (t3) ; add t3, t3, s1
        vle8.v          v1, (t3) ; add t3, t3, s1
        vle8.v          v2, (t3) ; add t3, t3, s1
        vle8.v          v3, (t3) ; add t3, t3, s1
        vle8.v          v4, (t3) ; add t3, t3, s1
        vle8.v          v5, (t3) ; add t3, t3, s1
        vle8.v          v6, (t3) ; add t3, t3, s1
        vle8.v          v7, (t3)

        /* ----------------------------------------------------------------
         * Each vector v0-v7 holds one ROW: [P3 P2 P1 P0 Q0 Q1 Q2 Q3]
         *
         * For vertical edge filtering we need to operate on COLUMNS, i.e.
         * we need all the P3 values (one per row) in one vector, etc.
         *
         * This requires an 8x8 byte transpose.
         *
         * After transpose:
         *   v0 = column 0 = all P3 values (one per row)
         *   v1 = column 1 = all P2 values
         *   ...
         *   v7 = column 7 = all Q3 values
         * ---------------------------------------------------------------- */

        /* ------- 8x8 byte transpose using RVV -------
         *
         * Strategy: use vrgather to pick elements.
         * We store all 8 vectors contiguously, then gather by (row*8 + col)
         * to form columns.
         *
         * Store v0-v7 to stack (64 bytes).
         */
        addi            t3, sp, -64         /* use 64 bytes below our frame */
        /* Adjust sp further for this temp area */
        addi            sp, sp, -64
        vse8.v          v0, (sp)
        addi            t3, sp, 8
        vse8.v          v1, (t3)
        addi            t3, sp, 16
        vse8.v          v2, (t3)
        addi            t3, sp, 24
        vse8.v          v3, (t3)
        addi            t3, sp, 32
        vse8.v          v4, (t3)
        addi            t3, sp, 40
        vse8.v          v5, (t3)
        addi            t3, sp, 48
        vse8.v          v6, (t3)
        addi            t3, sp, 56
        vse8.v          v7, (t3)

        /*
         * Index for column c: gather indices = [c, c+8, c+16, c+24, c+32, c+40, c+48, c+56]
         * We set up a base index vector and add c.
         *
         * Base: [0, 8, 16, 24, 32, 40, 48, 56]
         */
        /* Set vl=8, e8 for the gather */
        vsetivli        zero, 8, e8, m1, ta, ma

        /* Load base indices [0,8,16,24,32,40,48,56] into v16 */
        li              t3, 0
        vmv.v.x         v16, t3
        /* vid gives [0,1,2,...,7]; multiply by 8 */
        vid.v           v16
        vsll.vi         v16, v16, 3         /* v16 = [0,8,16,24,32,40,48,56] */

        /* col 0 (P3) */
        vrgather.vv     v0, v7, v16         /* placeholder – see note below */

        /*
         * NOTE: vrgather on m1 gathers within a single m1 group (8 bytes).
         * For a true 8x8 transpose across 64 bytes we need vlmul or
         * a load-with-stride approach.
         *
         * Better: use strided loads with stride=8 (row stride in transposed view).
         */
        addi            t3, sp, 0
        li              t4, 8               /* column stride = 8 bytes (one element per row) */
        vlse8.v         v0, (t3), t4        /* col 0: [row0[0], row1[0], ...] */
        addi            t3, sp, 1
        vlse8.v         v1, (t3), t4        /* col 1 */
        addi            t3, sp, 2
        vlse8.v         v2, (t3), t4
        addi            t3, sp, 3
        vlse8.v         v3, (t3), t4
        addi            t3, sp, 4
        vlse8.v         v4, (t3), t4
        addi            t3, sp, 5
        vlse8.v         v5, (t3), t4
        addi            t3, sp, 6
        vlse8.v         v6, (t3), t4
        addi            t3, sp, 7
        vlse8.v         v7, (t3), t4

        addi            sp, sp, 64          /* restore temp area */

        /* ----------------------------------------------------------------
         * Expand 8-bit to 16-bit (zero-extend)
         * After this: v0-v7 are e16, 8 lanes
         * ---------------------------------------------------------------- */
        vsetivli        zero, 8, e16, m2, ta, ma
        /*
         * We need e16 results in v0-v7 individually.
         * vzext.vf2 doubles element size.
         * Use m1 for each to stay in v0-v7:
         */
        vsetivli        zero, 8, e8, m1, ta, ma
        /* save 8-bit originals */
        vmv.v.v         v16, v0
        vmv.v.v         v17, v1
        vmv.v.v         v18, v2
        vmv.v.v         v19, v3
        vmv.v.v         v20, v4
        vmv.v.v         v21, v5
        vmv.v.v         v22, v6
        vmv.v.v         v23, v7

        vsetivli        zero, 8, e16, m1, ta, ma
        vzext.vf2       v0, v16
        vzext.vf2       v1, v17
        vzext.vf2       v2, v18
        vzext.vf2       v3, v19
        vzext.vf2       v4, v20
        vzext.vf2       v5, v21
        vzext.vf2       v6, v22
        vzext.vf2       v7, v23

        /* ----------------------------------------------------------------
         * Build tc vectors
         * tc[0] for lanes 0-3, tc[1] for lanes 4-7
         * ---------------------------------------------------------------- */
        /* Clamp tc to >=0 (they should be, but just in case) */
        li              t4, 0
        blt             t0, t4, .Lv_exit
        blt             t1, t4, .Lv_exit

        /* Use sp scratch (16 bytes at [sp+0..sp+15]) */
        build_tc_vectors v8, v9, v10, v11, t0, t1, sp

        /*
         * v8  = tc  per lane
         * v9  = tc*2
         * v10 = tc*3
         * v11 = tc/2
         */

        /* tc25 = (5*tc + 1) >> 1  =  tc + tc + tc + (tc>>1) approximately
         *       = (5*tc+1)>>1
         * Exact: vadd tc*4 + tc = tc*5, then vsra+1 rounding
         */
        vadd.vv         v12, v9, v9         /* 4*tc */
        vadd.vv         v12, v12, v8        /* 5*tc */
        vadd.vi         v12, v12, 1
        vsra.vi         v12, v12, 1         /* tc25 */

        /* ----------------------------------------------------------------
         * Compute conditions
         * ---------------------------------------------------------------- */
        /* mask registers: use v0_m=vm0, v1_m=vm1 etc. (architectural names) */
        /* RVV mask regs are vm0-vm7; in the assembler use v0.t etc.
         * Here we use dedicated mask vector variables conceptually;
         * in real RVV assembly we store masks as regular vectors with vmsXX. */

        /*
         * We store masks as 16-bit vectors (0xFFFF=true, 0x0000=false)
         * using vmv + vmerge idiom, since mask registers are 1-bit.
         * This matches the NEON approach.
         *
         * Actual mask computation uses vmsXX -> mask register,
         * then vmv.v.i + vmerge to expand to full elements.
         */

        compute_conditions v13_skip, v14_strong, v15_ndp, v16_ndq, \
                           s2, v8, v12
        /*
         * v13_skip  = skip mask  (mask register, 1-bit per lane)
         * v14_strong= strong mask
         * v15_ndp   = nd_p mask
         * v16_ndq   = nd_q mask
         */

        /* ----------------------------------------------------------------
         * Check if all lanes skip
         * ---------------------------------------------------------------- */
        vcpop.m         t3, v13_skip
        li              t4, 8
        beq             t3, t4, .Lv_exit    /* all 8 lanes skip */

        /* ----------------------------------------------------------------
         * Invert skip mask to get "do_filter" mask
         * ---------------------------------------------------------------- */
        vmnot.m         v17_do, v13_skip    /* do_filter = !skip */

        /* ----------------------------------------------------------------
         * Apply no_p / no_q to strong mask
         * If no_p: disable P-side updates (strong only touches P2..P0)
         * If no_q: disable Q-side updates
         * We handle by zeroing deltas on affected sides after computation.
         * ---------------------------------------------------------------- */

        /* ----------------------------------------------------------------
         * STRONG FILTER (compute for all lanes, mask later)
         * ---------------------------------------------------------------- */
        /* Save originals for mixing */
        vmv.v.v         v16, v1             /* orig P2 */
        vmv.v.v         v17, v6             /* orig Q2 */

        strong_filter_body v8, v9, v10
        /*
         * Results: v24=P0', v25=P1', v26=P2', v27=Q0', v28=Q1', v29=Q2'
         */

        /* ----------------------------------------------------------------
         * WEAK FILTER (compute for all lanes, mask later)
         * ---------------------------------------------------------------- */
        /* Restore v0-v7 to originals for weak filter input */
        /* (strong_filter_body doesn't modify v0-v7, only v24-v29, so OK) */

        weak_filter_body v8, v11, v15_ndp, v16_ndq
        /*
         * Results: v24w=P0' in v24, Q0'=v25, P1'=v26, Q1'=v27
         * BUT: strong already used v24-v29!
         * We need to separate. Use different temporaries:
         */

        /*
         * Reorganization: run weak into v20-v23 range.
         * Rewrite weak call to use different output regs.
         * For clarity in this reference implementation, we run them
         * sequentially and save/restore.
         *
         * Save strong results to named locations:
         */
        /* Strong: P2'=v26, P1'=v25, P0'=v24, Q0'=v27, Q1'=v28, Q2'=v29 */
        /* Copy strong results */
        vmv.v.v         v16_sP2, v26
        vmv.v.v         v17_sP1, v25
        vmv.v.v         v18_sP0, v24
        vmv.v.v         v19_sQ0, v27
        vmv.v.v         v20_sQ1, v28
        vmv.v.v         v21_sQ2, v29

        /* Weak: recalculate into v24-v27 */
        weak_filter_body v8, v11, v15_ndp, v16_ndq
        /* v24=wP0', v25=wQ0', v26=wP1', v27=wQ1' */

        /* ----------------------------------------------------------------
         * Merge strong and weak based on strong_mask
         * (strong_m=1 -> use strong, else use weak/original)
         *
         * P3, Q3: never changed.
         *
         * With no_p / no_q flags:
         *   no_p=1: P side keeps original (P2, P1, P0 unchanged)
         *   no_q=1: Q side keeps original (Q0, Q1, Q2 unchanged)
         * ---------------------------------------------------------------- */

        /*
         * Build combined filter mask per side:
         *   do_p = do_filter & !no_p
         *   do_q = do_filter & !no_q
         */
        bnez            s4, .Lv_no_p_set
        vmv.v.i         v13_dop, 1
        j               .Lv_after_no_p
.Lv_no_p_set:
        vmv.v.i         v13_dop, 0
.Lv_after_no_p:
        /* Expand to all-ones / all-zeros per lane */
        /* Simpler: build masks as vectors of 0/0xFFFF */

        /*
         * For the final merge we use vmerge.vvm:
         *   result = (mask) ? a : b
         *
         * P2: strong_m ? sP2 : orig_P2
         *     do_p_m ? above : orig_P2
         */

        /* ------ P2 ------ */
        /* start with original */
        vmv.v.v         v1_out, v1          /* orig P2 */
        /* if do_filter and strong: use sP2 */
        vmerge.vvm      v1_out, v1_out, v16_sP2, v14_strong
        /* if do_filter and !strong: keep orig P2 (strong P2 vs. weak: P2 only changes in strong) */
        /* apply do_p mask */
        vmerge.vvm      v1_out, v1, v1_out, v17_do   /* if !do_filter, revert to orig */
        /* apply no_p */
        beqz            s4, .Lv_skip_nop_p2
        vmv.v.v         v1_out, v1
.Lv_skip_nop_p2:

        /* ------ P1 ------ */
        vmv.v.v         v2_out, v2
        /* strong: use sP1 */
        vmerge.vvm      v2_out, v2_out, v17_sP1, v14_strong
        /* weak: use wP1 (in v26) */
        /* we need: if !strong && do_filter: use wP1 */
        vmnot.m         v18_nstrong, v14_strong
        vmand.mm        v18_nstrong, v18_nstrong, v17_do
        vmerge.vvm      v2_out, v2_out, v26, v18_nstrong
        /* final apply do_p and no_p */
        vmerge.vvm      v2_out, v2, v2_out, v17_do
        beqz            s4, .Lv_skip_nop_p1
        vmv.v.v         v2_out, v2
.Lv_skip_nop_p1:

        /* ------ P0 ------ */
        vmv.v.v         v3_out, v3
        vmerge.vvm      v3_out, v3_out, v18_sP0, v14_strong
        vmerge.vvm      v3_out, v3_out, v24, v18_nstrong   /* weak P0 */
        vmerge.vvm      v3_out, v3, v3_out, v17_do
        beqz            s4, .Lv_skip_nop_p0
        vmv.v.v         v3_out, v3
.Lv_skip_nop_p0:

        /* ------ Q0 ------ */
        vmv.v.v         v4_out, v4
        vmerge.vvm      v4_out, v4_out, v19_sQ0, v14_strong
        vmerge.vvm      v4_out, v4_out, v25, v18_nstrong   /* weak Q0 */
        vmerge.vvm      v4_out, v4, v4_out, v17_do
        beqz            s5, .Lv_skip_noq_q0
        vmv.v.v         v4_out, v4
.Lv_skip_noq_q0:

        /* ------ Q1 ------ */
        vmv.v.v         v5_out, v5
        vmerge.vvm      v5_out, v5_out, v20_sQ1, v14_strong
        vmerge.vvm      v5_out, v5_out, v27, v18_nstrong
        vmerge.vvm      v5_out, v5, v5_out, v17_do
        beqz            s5, .Lv_skip_noq_q1
        vmv.v.v         v5_out, v5
.Lv_skip_noq_q1:

        /* ------ Q2 ------ */
        vmv.v.v         v6_out, v6
        vmerge.vvm      v6_out, v6_out, v21_sQ2, v14_strong
        /* weak does not update Q2 */
        vmerge.vvm      v6_out, v6, v6_out, v17_do
        beqz            s5, .Lv_skip_noq_q2
        vmv.v.v         v6_out, v6
.Lv_skip_noq_q2:

        /* ----------------------------------------------------------------
         * Clip to [0, 255] (16-bit -> 8-bit)
         * ---------------------------------------------------------------- */
        li              t4, 255

        vmax.vx         v1_out, v1_out, zero
        vmin.vx         v1_out, v1_out, t4
        vmax.vx         v2_out, v2_out, zero
        vmin.vx         v2_out, v2_out, t4
        vmax.vx         v3_out, v3_out, zero
        vmin.vx         v3_out, v3_out, t4
        vmax.vx         v4_out, v4_out, zero
        vmin.vx         v4_out, v4_out, t4
        vmax.vx         v5_out, v5_out, zero
        vmin.vx         v5_out, v5_out, t4
        vmax.vx         v6_out, v6_out, zero
        vmin.vx         v6_out, v6_out, t4

        /* Narrow to 8-bit (saturating) */
        vsetivli        zero, 8, e8, m1, ta, ma
        vnclipu.wi      v1_8, v1_out, 0
        vnclipu.wi      v2_8, v2_out, 0
        vnclipu.wi      v3_8, v3_out, 0
        vnclipu.wi      v4_8, v4_out, 0
        vnclipu.wi      v5_8, v5_out, 0
        vnclipu.wi      v6_8, v6_out, 0
        /* v0 (P3) and v7 (Q3) unchanged */

        /* ----------------------------------------------------------------
         * Transpose back (8x8), then store
         * ---------------------------------------------------------------- */

        /* Rebuild full 8-vector set for transposing back:
         * cols: v0(P3), v1_8(P2'), v2_8(P1'), v3_8(P0'),
         *       v4_8(Q0'), v5_8(Q1'), v6_8(Q2'), v7(Q3)
         */
        /* Store columns to temp, reload as rows */
        addi            sp, sp, -64

        vsetivli        zero, 8, e8, m1, ta, ma
        li              t4, 8

        /* Strided store: col c stores its 8 elements at offsets c, c+8, c+16, ... */
        mv              t3, sp
        vsse8.v         v0,    (t3), t4     /* col 0 -> row offsets 0,8,16,...,56 */
        addi            t3, sp, 1
        vsse8.v         v1_8,  (t3), t4
        addi            t3, sp, 2
        vsse8.v         v2_8,  (t3), t4
        addi            t3, sp, 3
        vsse8.v         v3_8,  (t3), t4
        addi            t3, sp, 4
        vsse8.v         v4_8,  (t3), t4
        addi            t3, sp, 5
        vsse8.v         v5_8,  (t3), t4
        addi            t3, sp, 6
        vsse8.v         v6_8,  (t3), t4
        addi            t3, sp, 7
        vsse8.v         v7,    (t3), t4

        /* Load rows */
        mv              t3, sp
        vle8.v          v0, (t3) ; addi t3, t3, 8
        vle8.v          v1, (t3) ; addi t3, t3, 8
        vle8.v          v2, (t3) ; addi t3, t3, 8
        vle8.v          v3, (t3) ; addi t3, t3, 8
        vle8.v          v4, (t3) ; addi t3, t3, 8
        vle8.v          v5, (t3) ; addi t3, t3, 8
        vle8.v          v6, (t3) ; addi t3, t3, 8
        vle8.v          v7, (t3)

        addi            sp, sp, 64

        /* ----------------------------------------------------------------
         * Store 8 rows back to memory
         * ---------------------------------------------------------------- */
        mv              t3, s0
        vse8.v          v0, (t3) ; add t3, t3, s1
        vse8.v          v1, (t3) ; add t3, t3, s1
        vse8.v          v2, (t3) ; add t3, t3, s1
        vse8.v          v3, (t3) ; add t3, t3, s1
        vse8.v          v4, (t3) ; add t3, t3, s1
        vse8.v          v5, (t3) ; add t3, t3, s1
        vse8.v          v6, (t3) ; add t3, t3, s1
        vse8.v          v7, (t3)

.Lv_exit:
        ld              ra,  72(sp)
        ld              s0,  64(sp)
        ld              s1,  56(sp)
        ld              s2,  48(sp)
        ld              s3,  40(sp)
        ld              s4,  32(sp)
        ld              s5,  24(sp)
        ld              s6,  16(sp)
        addi            sp, sp, 80
        ret

/* ============================================================================
 * ff_hevc_h_loop_filter_luma_8_rvv
 *
 * Horizontal edge: boundary is a horizontal line.
 * Each column has [P3 P2 P1 P0 Q0 Q1 Q2 Q3] vertically.
 * Load 8 rows (P3..Q3), each row = 8 pixels wide.
 * No transpose needed: rows directly correspond to P3..Q3.
 *
 * The first 4 rows (P3..P0) use tc[0]; last 4 rows (Q0..Q3) still tc[0].
 * Within a row, pixels 0-3 use tc[0], pixels 4-7 use tc[1].
 *
 * Wait – for H filter, per the NEON reference:
 *   - No transpose
 *   - 8 lanes = 8 columns, each column is independently filtered
 *   - tc[0] applies to columns 0-3, tc[1] applies to columns 4-7
 *   - v0=P3_row, v1=P2_row, ..., v7=Q3_row
 *   - Each vector has 8 elements (one per column)
 * ============================================================================ */
        .global ff_hevc_h_loop_filter_luma_8_rvv
ff_hevc_h_loop_filter_luma_8_rvv:

        addi            sp, sp, -80
        sd              ra,  72(sp)
        sd              s0,  64(sp)
        sd              s1,  56(sp)
        sd              s2,  48(sp)
        sd              s3,  40(sp)
        sd              s4,  32(sp)
        sd              s5,  24(sp)
        sd              s6,  16(sp)

        mv              s0, a0
        mv              s1, a1
        mv              s2, a2
        mv              s3, a3
        mv              s4, a4
        mv              s5, a5

        lh              t0, 0(s3)
        lh              t1, 2(s3)
        or              t2, t0, t1
        beqz            t2, .Lh_exit

        /* pix -= 4 * stride */
        slli            t3, s1, 2
        sub             s0, s0, t3

        /* Load 8 rows x 8 bytes */
        vsetivli        zero, 8, e8, m1, ta, ma
        mv              t3, s0
        vle8.v          v0, (t3) ; add t3, t3, s1   /* row 0 = P3 row */
        vle8.v          v1, (t3) ; add t3, t3, s1   /* row 1 = P2 row */
        vle8.v          v2, (t3) ; add t3, t3, s1   /* row 2 = P1 row */
        vle8.v          v3, (t3) ; add t3, t3, s1   /* row 3 = P0 row */
        vle8.v          v4, (t3) ; add t3, t3, s1   /* row 4 = Q0 row */
        vle8.v          v5, (t3) ; add t3, t3, s1   /* row 5 = Q1 row */
        vle8.v          v6, (t3) ; add t3, t3, s1   /* row 6 = Q2 row */
        vle8.v          v7, (t3)                     /* row 7 = Q3 row */

        /* Expand to 16-bit */
        /* Save 8-bit originals */
        vsetivli        zero, 8, e8, m1, ta, ma
        vmv.v.v         v16, v0
        vmv.v.v         v17, v1
        vmv.v.v         v18, v2
        vmv.v.v         v19, v3
        vmv.v.v         v20, v4
        vmv.v.v         v21, v5
        vmv.v.v         v22, v6
        vmv.v.v         v23, v7

        vsetivli        zero, 8, e16, m1, ta, ma
        vzext.vf2       v0, v16
        vzext.vf2       v1, v17
        vzext.vf2       v2, v18
        vzext.vf2       v3, v19
        vzext.vf2       v4, v20
        vzext.vf2       v5, v21
        vzext.vf2       v6, v22
        vzext.vf2       v7, v23

        /* Build tc vectors */
        build_tc_vectors v8, v9, v10, v11, t0, t1, sp

        /* tc25 */
        vadd.vv         v12, v9, v9
        vadd.vv         v12, v12, v8
        vadd.vi         v12, v12, 1
        vsra.vi         v12, v12, 1

        /* Conditions */
        compute_conditions v13_skip, v14_strong, v15_ndp, v16_ndq, \
                           s2, v8, v12

        vcpop.m         t3, v13_skip
        li              t4, 8
        beq             t3, t4, .Lh_exit

        vmnot.m         v17_do, v13_skip

        /* Save originals for merge */
        vmv.v.v         v16_origP2, v1
        vmv.v.v         v16_origQ2, v6

        /* Strong filter */
        strong_filter_body v8, v9, v10
        vmv.v.v         v16_sP2, v26
        vmv.v.v         v17_sP1, v25
        vmv.v.v         v18_sP0, v24
        vmv.v.v         v19_sQ0, v27
        vmv.v.v         v20_sQ1, v28
        vmv.v.v         v21_sQ2, v29

        /* Weak filter */
        weak_filter_body v8, v11, v15_ndp, v16_ndq
        /* v24=wP0', v25=wQ0', v26=wP1', v27=wQ1' */

        /* Merge strong / weak / original */
        vmnot.m         v18_nstrong, v14_strong
        vmand.mm        v18_nstrong, v18_nstrong, v17_do

        /* P2 */
        vmv.v.v         v1_out, v1
        vmerge.vvm      v1_out, v1_out, v16_sP2, v14_strong
        vmerge.vvm      v1_out, v1, v1_out, v17_do
        beqz            s4, .Lh_ok_p2
        vmv.v.v         v1_out, v1
.Lh_ok_p2:

        /* P1 */
        vmv.v.v         v2_out, v2
        vmerge.vvm      v2_out, v2_out, v17_sP1, v14_strong
        vmerge.vvm      v2_out, v2_out, v26, v18_nstrong
        vmerge.vvm      v2_out, v2, v2_out, v17_do
        beqz            s4, .Lh_ok_p1
        vmv.v.v         v2_out, v2
.Lh_ok_p1:

        /* P0 */
        vmv.v.v         v3_out, v3
        vmerge.vvm      v3_out, v3_out, v18_sP0, v14_strong
        vmerge.vvm      v3_out, v3_out, v24, v18_nstrong
        vmerge.vvm      v3_out, v3, v3_out, v17_do
        beqz            s4, .Lh_ok_p0
        vmv.v.v         v3_out, v3
.Lh_ok_p0:

        /* Q0 */
        vmv.v.v         v4_out, v4
        vmerge.vvm      v4_out, v4_out, v19_sQ0, v14_strong
        vmerge.vvm      v4_out, v4_out, v25, v18_nstrong
        vmerge.vvm      v4_out, v4, v4_out, v17_do
        beqz            s5, .Lh_ok_q0
        vmv.v.v         v4_out, v4
.Lh_ok_q0:

        /* Q1 */
        vmv.v.v         v5_out, v5
        vmerge.vvm      v5_out, v5_out, v20_sQ1, v14_strong
        vmerge.vvm      v5_out, v5_out, v27, v18_nstrong
        vmerge.vvm      v5_out, v5, v5_out, v17_do
        beqz            s5, .Lh_ok_q1
        vmv.v.v         v5_out, v5
.Lh_ok_q1:

        /* Q2 */
        vmv.v.v         v6_out, v6
        vmerge.vvm      v6_out, v6_out, v21_sQ2, v14_strong
        vmerge.vvm      v6_out, v6, v6_out, v17_do
        beqz            s5, .Lh_ok_q2
        vmv.v.v         v6_out, v6
.Lh_ok_q2:

        /* Clip and narrow */
        li              t4, 255
        vmax.vx         v1_out, v1_out, zero ; vmin.vx v1_out, v1_out, t4
        vmax.vx         v2_out, v2_out, zero ; vmin.vx v2_out, v2_out, t4
        vmax.vx         v3_out, v3_out, zero ; vmin.vx v3_out, v3_out, t4
        vmax.vx         v4_out, v4_out, zero ; vmin.vx v4_out, v4_out, t4
        vmax.vx         v5_out, v5_out, zero ; vmin.vx v5_out, v5_out, t4
        vmax.vx         v6_out, v6_out, zero ; vmin.vx v6_out, v6_out, t4

        vsetivli        zero, 8, e8, m1, ta, ma
        vnclipu.wi      v1_8, v1_out, 0
        vnclipu.wi      v2_8, v2_out, 0
        vnclipu.wi      v3_8, v3_out, 0
        vnclipu.wi      v4_8, v4_out, 0
        vnclipu.wi      v5_8, v5_out, 0
        vnclipu.wi      v6_8, v6_out, 0

        /* Store rows (no transpose needed for horizontal) */
        mv              t3, s0
        vse8.v          v0,    (t3) ; add t3, t3, s1   /* P3 unchanged */
        vse8.v          v1_8,  (t3) ; add t3, t3, s1   /* P2' */
        vse8.v          v2_8,  (t3) ; add t3, t3, s1   /* P1' */
        vse8.v          v3_8,  (t3) ; add t3, t3, s1   /* P0' */
        vse8.v          v4_8,  (t3) ; add t3, t3, s1   /* Q0' */
        vse8.v          v5_8,  (t3) ; add t3, t3, s1   /* Q1' */
        vse8.v          v6_8,  (t3) ; add t3, t3, s1   /* Q2' */
        vse8.v          v7,    (t3)                     /* Q3 unchanged */

.Lh_exit:
        ld              ra,  72(sp)
        ld              s0,  64(sp)
        ld              s1,  56(sp)
        ld              s2,  48(sp)
        ld              s3,  40(sp)
        ld              s4,  32(sp)
        ld              s5,  24(sp)
        ld              s6,  16(sp)
        addi            sp, sp, 80
        ret
