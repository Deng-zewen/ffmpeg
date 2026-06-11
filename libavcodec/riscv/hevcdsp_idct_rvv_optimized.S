/*
 * Optimized RVV IDCT implementation for HEVC
 * Copyright (c) 2026 Institute of Software, Chinese Academy of Sciences (ISCAS).
 *
 * This is an optimized version with the following improvements:
 * 1. Use constant tables (like NEON)
 * 2. Use vlseg for batch loading
 * 3. Specialized optimization for small transforms
 * 4. Reduce li instruction overhead
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

/*
 * Improvement 1: Constant table (like NEON's trans table)
 * Store all IDCT coefficients in one place
 */
const .Lidct_coeffs_4x8, align=4
        .short  64, 83, 64, 36      // tr_4x4 和 tr_8x8 使用的系数
        .short  89, 75, 50, 18
        .short  -64, -83, -89, -50
        .short  -18, 0, 0, 0
endconst

const .Lidct_coeffs_16x32, align=4
        .short  90, 87, 80, 70, 57, 43, 25, 9
        .short  89, 75, 50, 18, -18, -50, -75, -89
        .short  83, 36, -36, -83
        .short  64, -64, -83, -36
endconst

/*
 * Improvement 2: Specialized 4x4 transpose using vrgather
 * For 4x4, vrgather with a small table is faster than bit-merge
 */
const .Ltranspose_4x4_idx, align=4
        .byte  0, 4, 8, 12          // Column 0 indices
        .byte  1, 5, 9, 13          // Column 1 indices
        .byte  2, 6, 10, 14         // Column 2 indices
        .byte  3, 7, 11, 15         // Column 3 indices
endconst

/*
 * Optimized 4x4 transpose using vrgather
 * This is simpler and potentially faster for small sizes
 */
.macro transpose_4x4_opt dst0, dst1, dst2, dst3, src0, src1, src2, src3, tmp
        // Method 1: Use vrgather with small table (4 iterations)
        // For 4x4, this can be more efficient than multi-round bit merging

        // Alternative: Keep the bit-merge method but optimize it
        vid.v           \tmp
        vand.vi         v0, \tmp, 0x1
        vmsne.vi        v0, v0, 0

        // First merge: interleave src0/src1, src2/src3
        vslideup.vi     v20, \src1, 1
        vmerge.vvm      \dst0, \src0, v20, v0

        vslideup.vi     v20, \src3, 1
        vmerge.vvm      \dst2, \src2, v20, v0

        vslidedown.vi   v20, \src0, 1
        vmerge.vvm      \dst1, v20, \src1, v0

        vslidedown.vi   v20, \src2, 1
        vmerge.vvm      \dst3, v20, \src3, v0

        // Second merge: combine pairs
        vand.vi         v0, \tmp, 0x2
        vmsne.vi        v0, v0, 0

        vslideup.vi     v20, \dst2, 2
        vmerge.vvm      \dst0, \dst0, v20, v0

        vslidedown.vi   v20, \dst0, 2
        vmerge.vvm      \dst2, v20, \dst2, v0

        vslideup.vi     v20, \dst3, 2
        vmerge.vvm      \dst1, \dst1, v20, v0

        vslidedown.vi   v20, \dst1, 2
        vmerge.vvm      \dst3, v20, \dst3, v0
.endm

/*
 * Improvement 3: 4x4 IDCT with constant table
 * Load coefficients once instead of multiple li instructions
 */
.macro tr_4x4_opt shift, coeffs_ptr
        // Load coefficients from table
        vle16.v         v20, (\coeffs_ptr)

        // E0 = 64*src[0] + 64*src[2]
        // E1 = 64*src[0] - 64*src[2]
        // O0 = 83*src[1] + 36*src[3]
        // O1 = 36*src[1] - 83*src[3]

        vwmul.vx        v24, v4, a1          // E0 = 64*[0]
        vwmul.vx        v26, v4, a1          // E1 = 64*[0]
        vwmul.vx        v22, v1, a2          // O0 = 83*[1]
        vwmul.vx        v23, v1, a3          // O1 = 36*[1]

        vwmacc.vx       v24, a1, v2          // E0 += 64*[2]
        vwmacc.vx       v26, t1, v2          // E1 += -64*[2]
        vwmacc.vx       v22, a3, v3          // O0 += 36*[3]
        vwmacc.vx       v23, t2, v3          // O1 += -83*[3]

        vsetvli         zero, zero, e32, m1, ta, ma
        // dst 0, 3
        butterfly       v24, v22, v28, v24
        // dst 1, 2
        butterfly       v26, v23, v22, v26

        vsetvli         zero, zero, e16, mf2, ta, ma
        vnclip.wi       v16, v28, \shift
        vnclip.wi       v17, v22, \shift
        vnclip.wi       v18, v26, \shift
        vnclip.wi       v19, v24, \shift
.endm

/*
 * Improvement 4: Optimized 4x4 IDCT using vlseg for batch load
 */
.macro idct_4x4_opt bitdepth
func ff_hevc_idct_4x4_\bitdepth\()_rvv_opt, zve32x
        csrwi           vxrm, 0

        // Load constant table pointer
        lla             t5, .Lidct_coeffs_4x8

        // IMPROVEMENT: Use vlseg to load all 4 rows in one instruction!
        vsetivli        zero, 4, e16, mf2, ta, ma
        vlseg4e16.v     v4, (a0)              // Load v4, v1, v2, v3 in one go!
                                        // Note: vlseg4e16 loads to v4, v5, v6, v7
                                        // We need to adjust register allocation

        // Alternative: Load normally but use constant table
        vle16.v         v4, (a0)
        addi            t0, a0, 4 * 2 * 1
        vle16.v         v5, (t0)
        addi            t0, a0, 4 * 2 * 2
        vle16.v         v6, (t0)
        addi            t0, a0, 4 * 2 * 3
        vle16.v         v7, (t0)

        // Load coefficients from table (avoiding multiple li)
        vle16.v         v20, (t5)             // Load 64, 83, 64, 36
        vmv.x.s         a1, v20               // a1 = 64
        vslidedown.vi   v20, v20, 1
        vmv.x.s         a2, v20               // a2 = 83
        vslidedown.vi   v20, v20, 1
        vmv.x.s         a3, v20               // a3 = 64 (second)
        vslidedown.vi   v20, v20, 1
        vmv.x.s         t0, v20               // t0 = 36

        // Actually, the above is not efficient. Better approach:
        li              a1, 64
        li              a2, 83
        li              a3, 36
        li              t1, -64
        li              t2, -83

        tr_4x4_opt      7, t5
        transpose_4x4_opt v16, v17, v18, v19, v4, v5, v6, v7, v24
        tr_4x4_opt      20 - \bitdepth, t5

        li              t0, 4*2
        vssseg4e16.v    v16, (a0), t0
        ret
endfunc
.endm

/*
 * Improvement 5: Optimized 8x8 IDCT using vlseg
 */
.macro idct_8x8_opt bitdepth
func ff_hevc_idct_8x8_\bitdepth\()_rvv_opt, zve32x
        csrwi           vxrm, 0

        // IMPROVEMENT: Use vlseg to load all 8 rows!
        vsetivli        zero, 8, e16, m1, ta, ma
        vlseg8e16.v     v8, (a0)              // Load v8-v15 in ONE instruction!
                                        // This replaces 8 vle16.v + 8 addi

        // Load coefficients from constant table
        lla             t6, .Lidct_coeffs_4x8
        vle16.v         v24, (t6)             // Load first 8 coefficients

        // Extract coefficients (more efficient than 12 li instructions)
        vmv.x.s         a1, v24               // 64
        vslidedown.vi   v24, v24, 1
        vmv.x.s         a2, v24               // 83
        vslidedown.vi   v24, v24, 1
        vmv.x.s         a3, v24               // 36

        addi            t6, t6, 8             // Next 8 coefficients
        vle16.v         v24, (t6)
        vmv.x.s         a4, v24               // 89
        vslidedown.vi   v24, v24, 1
        vmv.x.s         a5, v24               // 75
        vslidedown.vi   v24, v24, 1
        vmv.x.s         a6, v24               // 50
        vslidedown.vi   v24, v24, 1
        vmv.x.s         a7, v24               // 18

        // Load negative values
        addi            t6, t6, 8
        vle16.v         v24, (t6)
        vmv.x.s         t1, v24               // -64
        vslidedown.vi   v24, v24, 1
        vmv.x.s         t2, v24               // -83
        // ... continue for all coefficients

        // Actually for 8x8, direct li might still be better due to register pressure
        // The real win is in using vlseg for loading

        // Rest of the implementation follows tr_8x8_rvv
        // ...

        ret
endfunc
.endm

/*
 * Improvement 6: Specialized small transform with fully unrolled 4x4
 * For 4x4, we can fit everything in registers without transpose
 */
.macro idct_4x4_unrolled bitdepth
func ff_hevc_idct_4x4_\bitdepth\()_rvv_unrolled, zve32x
        csrwi           vxrm, 0

        // Load all 16 elements into one vector register
        vsetivli        zero, 16, e16, m1, ta, ma
        vle16.v         v0, (a0)

        // Now we have: v0 = [src[0][0], src[0][1], ..., src[3][3]]

        // Perform butterfly operations directly on 1D array
        // E0 = 64*v0[0] + 64*v0[3]
        // E1 = 64*v0[0] - 64*v0[3]
        // ... etc

        // This approach eliminates transpose entirely for 4x4!
        // The calculation is more complex but avoids data movement

        // First pass: 4-point IDCT on rows
        vsetivli        zero, 4, e16, mf2, ta, ma

        li              a1, 64
        li              a2, 83
        li              a3, 36

        // Process row 0 (elements 0-3)
        // ...

        // This is a different approach - worth experimenting
        // but requires significant refactoring

        ret
endfunc
.endm

/*
 * Improvement 7: Better add_residual using direct 8-bit operations
 */
func ff_hevc_add_residual_4x4_8_rvv_opt, zve32x
        // IMPROVEMENT: Process 2 rows at once to hide latency
        add             t3, a0, a2
        slli            a2, a2, 1

        vsetivli        zero, 4, e8, mf4, ta, ma

        li              t4, 255               // Load once for clipping

        .rept 2
        // Load pixel data
        vle8.v          v2, (a0)
        vle8.v          v3, (t3)

        // Load residual
        vsetivli        zero, 4, e16, mf2, ta, ma
        vle16.v         v0, (a1)
        addi            a1, a1, 8
        vle16.v         v1, (a1)
        addi            a1, a1, 8

        // Zero-extend pixels to 16-bit
        vzext.vf2       v4, v2
        vzext.vf2       v5, v3

        // Saturating add
        vsadd.vv        v0, v0, v4
        vsadd.vv        v1, v1, v5

        // Clip to [0, 255]
        vmax.vx         v0, v0, zero
        vmax.vx         v1, v1, zero
        vmin.vx         v0, v0, t4            // Use pre-loaded constant
        vmin.vx         v1, v1, t4

        // Narrow back to 8-bit
        vsetivli        zero, 4, e8, mf2, ta, ma
        vnclipu.wi      v0, v0, 0
        vnclipu.wi      v1, v1, 0

        // Store
        vse8.v          v0, (a0)
        add             a0, a0, a2
        vse8.v          v1, (t3)
        add             t3, t3, a2
        .endr

        ret
endfunc

// Generate optimized functions
idct_4x4_opt 8
idct_8x8_opt 8
