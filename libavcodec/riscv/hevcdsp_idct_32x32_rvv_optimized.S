/*
 * Copyright (c) 2026 Institute of Software, Chinese Academy of Sciences (ISCAS).
 *
 * Optimized version of hevcdsp_idct_rvv.S focusing on:
 * 1. Reducing function call overhead by combining passes
 * 2. Optimizing memory access patterns with stride loads
 *
 * This file is part of FFmpeg.
 */

#include "libavutil/riscv/asm.S"

.macro  lx rd, addr
#if (__riscv_xlen == 32)
        lw      \rd, \addr
#elif (__riscv_xlen == 64)
        ld      \rd, \addr
#else
        lq      \rd, \addr
#endif
.endm

.macro  sx rd, addr
#if (__riscv_xlen == 32)
        sw      \rd, \addr
#elif (__riscv_xlen == 64)
        sd      \rd, \addr
#else
        sq      \rd, \addr
#endif
.endm

.macro butterfly e, o, tmp_p, tmp_m
        vadd.vv         \tmp_p, \e, \o
        vsub.vv         \tmp_m, \e, \o
.endm

.macro add_member_16xN  in, tt0, tt1, tt2, tt3, tt4, tt5, tt6, tt7
        vwmacc.vx       v0, \tt0, \in
        vwmacc.vx       v16, \tt1, \in
        vwmacc.vx       v18, \tt2, \in
        vwmacc.vx       v2, \tt3, \in

        vwmacc.vx       v4, \tt4, \in
        vwmacc.vx       v20, \tt5, \in
        vwmacc.vx       v22, \tt6, \in
        vwmacc.vx       v6, \tt7, \in
.endm

/*
 * Optimization 2: Improved memory access pattern
 * Use stride-based loading to reduce address calculation overhead
 */
.macro tr_16xN_firstpass_optimized step
        // Pre-calculate base addresses to reduce immediate calculations
        // Base address for odd lines (1, 3, 5, 7, 9, 11, 13, 15)
        add             t0, a0, \step * 32          // Line 1 base
        li              t1, \step * 64              // Stride: 2 lines apart

        // Load odd lines using stride pattern
        vle16.v         v8, (t0)                    // Line 1
        add             t0, t0, t1
        vle16.v         v10, (t0)                   // Line 3
        add             t0, t0, t1
        vle16.v         v9, (t0)                    // Line 5
        add             t0, t0, t1
        vle16.v         v29, (t0)                   // Line 7
        add             t0, t0, t1
        vle16.v         v28, (t0)                   // Line 9
        add             t0, t0, t1
        vle16.v         v30, (t0)                   // Line 11
        add             t0, t0, t1
        vle16.v         v11, (t0)                   // Line 13
        add             t0, t0, t1
        vle16.v         v31, (t0)                   // Line 15

        // Base address for even lines (2, 6, 10, 14, 0, 4, 8, 12)
        add             t0, a0, \step * 64          // Line 2 base
        li              t1, \step * 128             // Stride: 4 lines apart

        vle16.v         v8, (t0)                    // Line 2
        add             t0, t0, t1
        vle16.v         v10, (t0)                   // Line 6
        add             t0, t0, t1
        vle16.v         v24, (t0)                   // Line 10
        add             t0, t0, t1
        vle16.v         v26, (t0)                   // Line 14

        // Lines 0, 4, 8, 12
        vle16.v         v10, (a0)                   // Line 0
        addi            t0, a0, \step * 32 * 4
        vle16.v         v28, (t0)                   // Line 4
        addi            t0, a0, \step * 32 * 8
        vle16.v         v26, (t0)                   // Line 8
        addi            t0, a0, \step * 32 * 12
        vle16.v         v29, (t0)                   // Line 12

        // Load transformation constants
        li              a2, 90
        li              a3, 87
        li              a4, 80
        li              a5, 70

        li              a6, 57
        li              a7, 43
        li              s2, 25
        li              s3, 9

        li              s4, -90
        li              s5, -87
        li              s6, -80
        li              s7, -70

        li              s8, -57
        li              s9, -43
        li              s10, -25
        li              s11, -9

        // O[k] calculations (same as before but with optimized loads)
        vwmul.vx        v0, v8, a2
        vwmul.vx        v16, v8, a3
        vwmul.vx        v18, v8, a4
        vwmul.vx        v2, v8, a5
        vwmul.vx        v4, v8, a6
        vwmul.vx        v20, v8, a7
        vwmul.vx        v22, v8, s2
        vwmul.vx        v6, v8, s3

        add_member_16xN         v10,    a3, a6, s3, s9, s6, s4, s7, s10
        add_member_16xN         v9,    a4, s3, s7, s5, s10, a6, a2, a7
        add_member_16xN         v29,    a5, s9, s5, s3, a2, s2, s6, s8
        add_member_16xN         v28,    a6, s6, s10, a2, s11, s5, a7, a5
        add_member_16xN         v30,    a7, s4, a6, s2, s5, a5, s3, s6
        add_member_16xN         v11,    s2, s7, a2, s6, a7, s3, s8, a3
        add_member_16xN         v31,    s3, s10, a7, s8, a5, s6, a3, s4

        // EO[k] calculations
        li              s2, -89
        li              s4, -50
        li              s5, -18
        li              s6, 89
        li              s7, 75
        li              s8, 50
        li              s9, 18

        vwmul.vx        v12, v8, s6
        vwmul.vx        v28, v8, s7
        vwmul.vx        v30, v8, s8
        vwmul.vx        v14, v8, s9

        vwmacc.vx       v12, s7, v10
        vwmacc.vx       v28, s5, v10
        vwmacc.vx       v30, s2, v10
        vwmacc.vx       v14, s4, v10

        vwmacc.vx       v12, s8, v24
        vwmacc.vx       v28, s2, v24
        vwmacc.vx       v30, s9, v24
        vwmacc.vx       v14, s7, v24

        vwmacc.vx       v12, s9, v26
        vwmacc.vx       v28, s4, v26
        vwmacc.vx       v30, s7, v26
        vwmacc.vx       v14, s2, v26

        // Store EO[1] and EO[2] to stack
        vse32.v         v28, (a1)
        sh1add          t0, s1, a1
        vse32.v         v30, (t0)

        // EEE and EEO calculations
        li              a2, 64
        li              a3, 83
        li              a5, 36
        li              a6, -64
        li              a7, -83

        vwmul.vx        v8, v10, a2
        vwmul.vx        v24, v10, a2
        vwmul.vx        v10, v28, a3
        vwmacc.vx       v8, a2, v26
        vwmacc.vx       v24, a6, v26
        vwmacc.vx       v10, a5, v29
        vwmul.vx        v26, v28, a5
        vwmacc.vx       v26, a7, v29
.endm

/*
 * Optimization 1: Combined function to eliminate call overhead
 * Merges func_tr_16xN_32x32_noscale_rvv + func_tr_32xN_firstpass_rvv
 */
.macro tr_16xN_32x32_combined_rvv name, step
func func_tr_16xN_32x32_combined_\name\()_rvv, zve32x, zba
        lpad            0

        // Inline tr_16xN_firstpass_optimized
        tr_16xN_firstpass_optimized \step

        // Calculate destination addresses for E values
        sh3add          a5, s1, t4          // E4
        sh1add          a6, s1, a5          // E5
        sh1add          a7, s1, a6          // E6
        sh1add          s2, s1, a7          // E7
        sh1add          s3, s1, s2          // E8
        sh1add          s4, s1, s3          // E9
        sh1add          s5, s1, s4          // E10
        sh1add          s6, s1, s5          // E11
        sh1add          s7, s1, s6          // E12
        sh1add          s8, s1, s7          // E13
        sh1add          s9, s1, s8          // E14
        sh1add          s10, s1, s9         // E15

        // First butterfly stage (EEE/EEO)
        vsetvli         zero, zero, e32, m2, ta, ma
        butterfly       v8, v10, v28, v30

        // Second butterfly stage (EE/EO)
        butterfly       v28, v12, v8, v10
        butterfly       v30, v14, v28, v12

        // Third butterfly stage (E/O)
        butterfly       v8, v0, v30, v14

        // Store E15 (keep E0 in v30)
        vse32.v         v14, (s10)

        butterfly       v28, v2, v8, v0
        vse32.v         v0, (s7)            // E12

        butterfly       v12, v4, v28, v2
        vse32.v         v28, (a5)           // E4
        vse32.v         v2, (s6)            // E11

        butterfly       v10, v6, v12, v4
        vse32.v         v12, (s2)           // E7
        vse32.v         v4, (s3)            // E8

        // Restore EEO[1] and EEO[2]
        vle32.v         v2, (a1)
        sh1add          t0, s1, a1
        vle32.v         v4, (t0)

        // Second path of butterfly operations
        butterfly       v24, v26, v6, v10
        butterfly       v6, v2, v24, v26
        butterfly       v10, v4, v6, v2

        butterfly       v24, v16, v28, v4
        vse32.v         v4, (s9)            // E14

        butterfly       v6, v18, v24, v16
        vse32.v         v16, (s8)           // E13

        butterfly       v2, v20, v6, v18
        vse32.v         v6, (a6)            // E5
        vse32.v         v18, (s5)           // E10

        butterfly       v26, v22, v2, v20
        vmv.v.v         v26, v8             // Move E3 to v26

        vse32.v         v2, (a7)            // E6
        vse32.v         v20, (s4)           // E9

        // E0 in v30, E1 in v28, E2 in v24, E3 in v26
        ret
endfunc
.endm

tr_16xN_32x32_combined_rvv firstpass, 1

/*
 * Optimized 32xN transformation (combined with scale and store)
 */
.macro dst_add_member32_m1_m2   in, tmp0, tmp1, tmp2, tmp3, op0, op1, op2, op3, first
        .ifc \op0, -
                neg    t0, \tmp0
        .endif
        .ifc \op1, -
                neg    t1, \tmp1
        .endif
        .ifc \op2, -
                neg    t2, \tmp2
        .endif
        .ifc \op3, -
                neg    t3, \tmp3
        .endif

.if \first == 1
        .ifc \op0, -
                vwmul.vx    v22, \in, t0
        .else
                vwmul.vx     v22, \in, \tmp0
        .endif
        .ifc \op1, -
                vwmul.vx     v20, \in, t1
        .else
                vwmul.vx     v20, \in, \tmp1
        .endif
        .ifc \op2, -
                vwmul.vx     v18, \in, t2
        .else
                vwmul.vx     v18, \in, \tmp2
        .endif
        .ifc \op3, -
                vwmul.vx     v16, \in, t3
        .else
                vwmul.vx     v16, \in, \tmp3
        .endif
.else
        .ifc \op0, -
                vwmacc.vx    v22, t0, \in
        .else
                vwmacc.vx     v22, \tmp0, \in
        .endif
        .ifc \op1, -
                vwmacc.vx     v20, t1, \in
        .else
                vwmacc.vx     v20, \tmp1, \in
        .endif
        .ifc \op2, -
                vwmacc.vx     v18, t2, \in
        .else
                vwmacc.vx     v18, \tmp2, \in
        .endif
        .ifc \op3, -
                vwmacc.vx     v16, t3, \in
        .else
                vwmacc.vx     v16, \tmp3, \in
        .endif
.endif
.endm

.macro add_member32_m1_m2 first, start_index, in, \
                                tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7, \
                                tmp8, tmp9, tmp10, tmp11, tmp12, tmp13, tmp14, tmp15, \
                                op0, op1, op2, op3, op4, op5, op6, op7, op8, op9, op10, op11, op12, op13, op14, op15
.if \start_index == 0
    dst_add_member32_m1_m2 \in, \tmp0, \tmp1, \tmp2, \tmp3, \op0, \op1, \op2, \op3, \first
.elseif \start_index == 4
    dst_add_member32_m1_m2 \in, \tmp4, \tmp5, \tmp6, \tmp7, \op4, \op5, \op6, \op7, \first
.elseif \start_index == 8
    dst_add_member32_m1_m2 \in, \tmp8, \tmp9, \tmp10, \tmp11, \op8, \op9, \op10, \op11, \first
.else
    dst_add_member32_m1_m2 \in, \tmp12, \tmp13, \tmp14, \tmp15, \op12, \op13, \op14, \op15, \first
.endif
.endm

.macro tr_block  start_index
        add_member32_m1_m2      1, \start_index, v0, a2, a3, a4, a5, a6, a7, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, +, +, +, +, +, +, +, +, +, +, +, +, +, +, +, +
        add_member32_m1_m2      0, \start_index, v1, a2, a6, s3, s6, s9, s11, s8, s5, s2, a5, a3, a4, a7, s4, s7, s10, +, +, +, +, +, -, -, -, -, -, -, -, -, -, -, -
        add_member32_m1_m2      0, \start_index, v2, a4, s3, s8, s10, s5, a6, a2, a7, s6, s11, s7, s2, a3, a5, s4, s9, +, +, +, -, -, -, -, -, -, -, +, +, +, +, +, +
        add_member32_m1_m2      0, \start_index, v3, a5, s6, s10, s3, a2, s2, s9, s7, a6, a4, s5, s11, s4, a3, a7, s8, +, +, -, -, -, -, -, +, +, +, +, -, -, -, -, -

        add_member32_m1_m2      0, \start_index, v4, a6, s9, s5, a2, s4, s10, a7, a5, s8, s6, a3, s3, s11, s2, a4, s7, +, +, -, -, -, +, +, +, +, -, -, -, +, +, +, +
        add_member32_m1_m2      0, \start_index, v5, a7, s11, a6, s2, s10, a5, s3, s9, a4, s4, s8, a3, s5, s7, a2, s6, +, -, -, -, +, +, +, -, -, -, +, +, +, -, -, -
        add_member32_m1_m2      0, \start_index, v6, s2, s8, a3, s9, a7, s3, s7, a2, s10, a6, s4, s6, a4, s11, a5, s5, +, -, -, -, +, +, -, -, -, +, +, -, -, -, +, +
        add_member32_m1_m2      0, \start_index, v7, s3, s5, a7, s7, a5, s9, a2, s11, a3, s10, a4, s8, a6, s6, s2, s4, +, -, -, +, +, -, -, +, +, +, -, -, +, +, -, -

        add_member32_m1_m2      0, \start_index, v8, s4, s2, s6, a6, s8, a4, s10, a3, s11, a2, s9, a5, s7, a7, s5, s3, +, -, -, +, +, -, -, +, -, -, +, +, -, -, +, +
        add_member32_m1_m2      0, \start_index, v9, s5, a5, s11, a4, s6, s4, a6, s10, a2, s7, s3, a7, s9, a3, s8, s2, +, -, -, +, -, -, +, +, -, +, +, -, -, +, -, -
        add_member32_m1_m2      0, \start_index, v10, s6, a2, s7, s5, a3, s8, s4, a4, s9, s3, a5, s10, s2, a6, s11, a7, +, -, +, +, -, +, +, -, +, +, -, +, +, -, +, +
        add_member32_m1_m2      0, \start_index, v11, s7, a4, s2, s11, s3, a2, s6, s8, a5, a7, s10, s4, a3, s5, s9, a6, +, -, +, -, -, +, -, -, +, -, +, +, -, +, +, -

        add_member32_m1_m2      0, \start_index, v12, s8, a7, a3, s4, s11, s5, a4, a6, s7, s9, s2, a2, s3, s10, s6, a5, +, -, +, -, +, +, -, +, -, -, +, -, +, -, -, +
        add_member32_m1_m2      0, \start_index, v13, s9, s4, a5, a2, s2, s7, s11, s6, a7, a3, a6, s5, s10, s8, s3, a4, +, -, +, -, +, -, -, +, -, +, -, +, -, -, +, -
        add_member32_m1_m2      0, \start_index, v14, s10, s7, s4, a7, a4, a2, a5, s2, s5, s8, s11, s9, s6, s3, a6, a3, +, -, +, -, +, -, +, -, +, -, +, +, -, +, -, +
        add_member32_m1_m2      0, \start_index, v15, s11, s10, s9, s8, s7, s6, s5, s4, s3, s2, a7, a6, a5, a4, a3, a2, +, -, +, -, +, -, +, -, +, -, +, -, +, -, +, -
.endm

.macro scale_store_optimized shift, off
.if \off != 24
        vse16.v         v14, (t4)
        add             t1, t4, s1
        vse16.v         v15, (t1)
.endif

        vsetvli         zero, zero, e32, m2, ta, ma
        butterfly       v30, v22, v14, v30
        butterfly       v28, v20, v22, v28
        butterfly       v24, v18, v20, v24
        butterfly       v26, v16, v18, v26

        vsetvli         zero, zero, e16, m1, ta, ma

        vnclip.wi       v16, v26, \shift
        vnclip.wi       v17, v24, \shift
        vnclip.wi       v24, v14, \shift
        vnclip.wi       v25, v22, \shift
        vnclip.wi       v26, v20, \shift
        vnclip.wi       v27, v18, \shift
        vnclip.wi       v18, v28, \shift
        vnclip.wi       v19, v30, \shift

        li              t2, 32*2
        add             t3, a1, \off
        vssseg4e16.v    v24, (t3), t2

        add             t3, a1, (64-4*2)-\off
        vssseg4e16.v    v16, (t3), t2

.if \off != 24
        vle16.v          v14, (t4)
        vle16.v          v15, (t1)
.endif
.endm

.macro tr_32xN_secondpass_optimized_rvv name, shift
func func_tr_32xN_secondpass_optimized_\name\()_rvv, zve32x, zba
        lpad            0

        // Optimization 2: Use stride-based loading pattern
        // Pre-calculate stride and base addresses
        li              t2, 64                      // Stride between consecutive lines
        add             t0, a0, t2                  // Line 1 base

        // Load all 16 lines using stride pattern (odd lines first)
        vle16.v         v0, (t0)                    // Line 1
        add             t0, t0, t2
        add             t0, t0, t2
        vle16.v         v1, (t0)                    // Line 3
        add             t0, t0, t2
        add             t0, t0, t2
        vle16.v         v2, (t0)                    // Line 5
        add             t0, t0, t2
        add             t0, t0, t2
        vle16.v         v3, (t0)                    // Line 7
        add             t0, t0, t2
        add             t0, t0, t2
        vle16.v         v4, (t0)                    // Line 9
        add             t0, t0, t2
        add             t0, t0, t2
        vle16.v         v5, (t0)                    // Line 11
        add             t0, t0, t2
        add             t0, t0, t2
        vle16.v         v6, (t0)                    // Line 13
        add             t0, t0, t2
        add             t0, t0, t2
        vle16.v         v7, (t0)                    // Line 15
        add             t0, t0, t2
        add             t0, t0, t2
        vle16.v         v8, (t0)                    // Line 17
        add             t0, t0, t2
        add             t0, t0, t2
        vle16.v         v9, (t0)                    // Line 19
        add             t0, t0, t2
        add             t0, t0, t2
        vle16.v         v10, (t0)                   // Line 21
        add             t0, t0, t2
        add             t0, t0, t2
        vle16.v         v11, (t0)                   // Line 23
        add             t0, t0, t2
        add             t0, t0, t2
        vle16.v         v12, (t0)                   // Line 25
        add             t0, t0, t2
        add             t0, t0, t2
        vle16.v         v13, (t0)                   // Line 27
        add             t0, t0, t2
        add             t0, t0, t2
        vle16.v         v14, (t0)                   // Line 29
        add             t0, t0, t2
        add             t0, t0, t2
        vle16.v         v15, (t0)                   // Line 31

        vsetvli         zero, zero, e16, m1, ta, ma

        li              a2, 90
        li              a3, 90
        li              a4, 88
        li              a5, 85
        li              a6, 82
        li              a7, 78
        li              s2, 73
        li              s3, 67
        li              s4, 61
        li              s5, 54
        li              s6, 46
        li              s7, 38
        li              s8, 31
        li              s9, 22
        li              s10, 13
        li              s11, 4

        tr_block        0
        scale_store_optimized \shift, 0

        // E4 - E7
        sh3add          t0, s1, t4
        vle32.v         v30, (t0)
        sh1add          t1, s1, t0
        vle32.v         v28, (t1)
        sh1add          t2, s1, t1
        vle32.v         v24, (t2)
        sh1add          t3, s1, t2
        vle32.v         v26, (t3)

        tr_block        4
        scale_store_optimized \shift, 8

        // E8 - E11
        add             t0, t4, t5
        vle32.v         v30, (t0)
        sh1add          t1, s1, t0
        vle32.v         v28, (t1)
        sh1add          t2, s1, t1
        vle32.v         v24, (t2)
        sh1add          t3, s1, t2
        vle32.v         v26, (t3)

        tr_block        8
        scale_store_optimized \shift, 16

        // E12 - E15
        slli            t3, s1, 3
        sh1add          t3, t3, t3
        add             t0, t4, t3
        vle32.v         v30, (t0)
        sh1add          t1, s1, t0
        vle32.v         v28, (t1)
        sh1add          t2, s1, t1
        vle32.v         v24, (t2)
        sh1add          t3, s1, t2
        vle32.v         v26, (t3)

        tr_block        12
        scale_store_optimized \shift, 24

        ret
endfunc
.endm

tr_32xN_secondpass_optimized_rvv secondpass_8, 20 - 8

/*
 * Main 32x32 IDCT function using optimized passes
 */
.macro idct_32x32_optimized bitdepth
func ff_hevc_idct_32x32_\bitdepth\()_rvv_optimized, zve32x, zba
        lpad    0
        addi    sp, sp, -(__riscv_xlen / 8)*12
        sx      ra, (__riscv_xlen / 8)*(11)(sp)
.irp i, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
        sx      s\i, (__riscv_xlen / 8)*(11-\i)(sp)
.endr
        mv      t6, a0
        csrwi   vxrm, 0

        // 16xN stack address
        addi    t4, sp, -2048
        addi    sp, t4, -2048

        li      t0, 32
        // Set the maximum number of elements that a vector register can process.
        vsetvli t0, t0, e16, m1, ta, ma
        // Number of bytes loaded into a vector register
        slli    s1, t0, 1
        // Number of bytes processed in one loop
        slli    t5, t0, 5

        mv      a1, sp
1:
        // Optimization 1: Single function call instead of two
        jal     func_tr_16xN_32x32_combined_firstpass_rvv
        add     a0, a0, s1
        sh1add  a1, t5, a1
        addi    a3, t6, (32 * 2)
        bgt     a3, a0, 1b

        mv      a0, sp
        mv      a1, t6
1:
        // Optimization 1: Single function call instead of two
        jal     func_tr_32xN_secondpass_optimized_secondpass_\bitdepth\()_rvv
        add     a0, a0, s1
        sh1add  a1, t5, a1
        addi    a3, sp, (32 * 2)
        bgt     a3, a0, 1b

        li      t0, 4096
        add     sp, sp,  t0

        lx      ra, (__riscv_xlen / 8)*(11)(sp)
.irp i, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
        lx      s\i, (__riscv_xlen / 8)*(11-\i)(sp)
.endr
        addi    sp, sp, (__riscv_xlen / 8)*12
        ret
endfunc
.endm

idct_32x32_optimized 8
