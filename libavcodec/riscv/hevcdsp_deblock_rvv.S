/* -*-riscv64-*-
 *
 * Copyright (c) 2026 Deng Zewen <deng.zewen@zte.com.cn>
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

.macro vabs d0, s0, t0
    vrsub.vi    \t0, \s0, 0
    vmax.vv     \d0, \s0, \t0
.endm

.macro uabd d0, s0, s1, t0
    vmaxu.vv        \d0, \s0, \s1
    vminu.vv        \t0, \s0, \s1
    vsub.vv         \d0, \d0, \t0    
.endm

.macro transpose16_8x8 r0, r1, r2, r3, r4, r5, r6, r7, tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7, tmp8, tmp9
        vid.v           \tmp9
        vand.vi         \tmp8, \tmp9, 0x1
        vmsne.vi        v0, \tmp8, 0

        // Stage 1: interleave even/odd rows
        vslideup.vi     \tmp0, \r1, 1
        vmerge.vvm      \tmp4, \r0, \tmp0, v0
        vslideup.vi     \tmp1, \r3, 1
        vmerge.vvm      \tmp5, \r2, \tmp1, v0

        vslideup.vi     \tmp2, \r5, 1
        vmerge.vvm      \tmp6, \r4, \tmp2, v0
        vslideup.vi     \tmp3, \r7, 1
        vmerge.vvm      \tmp7, \r6, \tmp3, v0

        vslidedown.vi   \tmp0, \r0, 1
        vmerge.vvm      \r0, \tmp0, \r1, v0
        vslidedown.vi   \tmp1, \r2, 1
        vmerge.vvm      \r2, \tmp1, \r3, v0

        vslidedown.vi   \tmp2, \r4, 1
        vmerge.vvm      \r4, \tmp2, \r5, v0
        vslidedown.vi   \tmp3, \r6, 1
        vmerge.vvm      \r6, \tmp3, \r7, v0

        // Stage 2: interleave pairs
        vand.vi         \tmp8, \tmp9, 0x2
        vmsne.vi        v0, \tmp8, 0

        vslideup.vi     \tmp0, \tmp5, 2
        vmerge.vvm      \tmp0, \tmp4, \tmp0, v0
        vslideup.vi     \tmp1, \tmp7, 2
        vmerge.vvm      \tmp1, \tmp6, \tmp1, v0

        vslidedown.vi   \tmp2, \tmp4, 2
        vmerge.vvm      \tmp2, \tmp2, \tmp5, v0
        vslidedown.vi   \tmp3, \tmp6, 2
        vmerge.vvm      \tmp3, \tmp3, \tmp7, v0

        vslideup.vi     \r1, \r2, 2
        vmerge.vvm      \r1, \r0, \r1, v0
        vslideup.vi     \r3, \r6, 2
        vmerge.vvm      \r3, \r4, \r3, v0

        vslidedown.vi   \r5, \r0, 2
        vmerge.vvm      \r5, \r5, \r2, v0
        vslidedown.vi   \r7, \r4, 2
        vmerge.vvm      \r7, \r7, \r6, v0

        // Stage 3: interleave quads
        vand.vi         \tmp8, \tmp9, 0x4
        vmsne.vi        v0, \tmp8, 0

        vslideup.vi     \tmp4, \tmp1, 4
        vmerge.vvm      \tmp4, \tmp0, \tmp4, v0
        vslideup.vi     \tmp5, \tmp3, 4
        vmerge.vvm      \tmp5, \tmp2, \tmp5, v0

        vslidedown.vi   \tmp6, \tmp0, 4
        vmerge.vvm      \tmp6, \tmp6, \tmp1, v0
        vslidedown.vi   \tmp7, \tmp2, 4
        vmerge.vvm      \tmp7, \tmp7, \tmp3, v0

        vslideup.vi     \tmp0, \r3, 4
        vmerge.vvm      \tmp0, \r1, \tmp0, v0
        vslideup.vi     \tmp1, \r7, 4
        vmerge.vvm      \tmp1, \r5, \tmp1, v0

        vslidedown.vi   \tmp2, \r1, 4
        vmerge.vvm      \tmp2, \tmp2, \r3, v0
        vslidedown.vi   \tmp3, \r5, 4
        vmerge.vvm      \tmp3, \tmp3, \r7, v0
.endm

.macro hevc_loop_filter_luma_body
       vmv.v.x     v16, a2            // beta

        // v17 = [tc0 x4, tc1 x4]; v19 = tc25 = (tc*5+1)>>1; v20 = beta>>2
        vmv.v.x     v17, a3
        vmv.v.x     v24, t6
        vslideup.vi v17, v24, 4
        vsll.vi     v18, v17, 2
        vadd.vv     v18, v18, v17
        vadd.vi     v18, v18, 1
        vsra.vi     v19, v18, 1
        vsra.vi     v20, v16, 2

        // dp0 = |P2 - 2*P1 + P0|, dq0 = |Q2 - 2*Q1 + Q0|, d0 = dp0 + dq0
        vadd.vv     v21, v11, v9
        vsll.vi     v22, v10, 1
        vsub.vv     v21, v21, v22
        vabs      v21, v21, v30

        vadd.vv     v22, v14, v12
        vsll.vi     v23, v13, 1
        vsub.vv     v22, v22, v23
        vabs      v22, v22, v30

        vadd.vv     v23, v21, v22

        // v24: per-lane mask for (d0<<1) < beta2
        vsll.vi     v24, v23, 1
        vmsltu.vv   v0, v24, v20
        vmv.v.i     v24, 0
        vmerge.vim  v24, v24, -1, v0

        // Group decision: 4 lines per group; check d0+d3 < beta for rows {0,3} and {4,7}
        vmv.x.s     t0, v23
        vslidedown.vi v25, v23, 3
        vmv.x.s     t1, v25
        add         t0, t0, t1

        vslidedown.vi v25, v23, 4
        vmv.x.s     t2, v25
        vslidedown.vi v25, v23, 7
        vmv.x.s     t3, v25
        add         t2, t2, t3

        sltu        t0, t0, a2
        sltu        t2, t2, a2
        or          t5, t0, t2
        beqz        t5, 4f

        // v26: group enable mask (low 4 = group0, high 4 = group1)
        neg         t0, t0
        li          t1, 0xFFFF
        and         t0, t0, t1
        neg         t2, t2
        and         t2, t2, t1
        vmv.v.x     v26, t0
        vmv.v.x     v25, t2
        vslideup.vi v26, v25, 4

        // Strong filter conditions
        vsra.vi     v27, v20, 1        // beta_3 = beta2 >> 1

        // |P3-P0| + |Q3-Q0| < beta_3
        uabd      v28, v8, v11, v30
        uabd      v29, v15, v12, v30
        vadd.vv     v28, v28, v29
        vmsltu.vv   v0, v28, v27
        vmv.v.i     v28, 0
        vmerge.vim  v28, v28, -1, v0
        vand.vv     v24, v24, v28

        // |P0-Q0| < tc25
        uabd      v28, v11, v12, v30
        vmsltu.vv   v0, v28, v19
        vmv.v.i     v28, 0
        vmerge.vim  v28, v28, -1, v0
        vand.vv     v24, v24, v28

        // Per-group strong flag: both row0 and row3 (or row4 and row7) must pass
        vmv.x.s     t0, v24
        vslidedown.vi v28, v24, 3
        vmv.x.s     t1, v28
        and         t0, t0, t1
        vslidedown.vi v28, v24, 4
        vmv.x.s     t2, v28
        vslidedown.vi v28, v24, 7
        vmv.x.s     t3, v28
        and         t2, t2, t3

        mv          a3, t0             // group0 strong flag
        mv          a4, t2             // group1 strong flag

        // v1: per-lane strong/weak selector (1=strong, 0=weak)
        vmv.v.x     v1, a3
        vmv.v.x     v25, a4
        vslideup.vi v1, v25, 4

        // Skip nd_p/nd_q if both groups are strong
        li          t4, 0xFFFF
        bne         a3, t4, 0f
        bne         a4, t4, 0f
        j           1f

0:
        // Weak filter: compute nd_p, nd_q
        srli        t5, a2, 1
        add         t5, t5, a2
        srli        t5, t5, 3          // threshold = (beta + beta/2) >> 3

        vslidedown.vi v25, v21, 3
        vadd.vv      v25, v25, v21
        vmv.x.s     t0, v25
        vslidedown.vi v2, v25, 4
        vmv.x.s     t1, v2

        vslidedown.vi v25, v22, 3
        vadd.vv      v25, v25, v22
        vmv.x.s     t2, v25
        vslidedown.vi v2, v25, 4
        vmv.x.s     t3, v2

        sltu        t0, t0, t5
        sltu        t1, t1, t5
        sltu        t2, t2, t5
        sltu        t3, t3, t5

        neg         t0, t0
        neg         t1, t1
        neg         t2, t2
        neg         t3, t3

        // v28 = nd_p mask, v29 = nd_q mask
        vmv.v.x     v28, t0
        vmv.v.x     v25, t1
        vslideup.vi v28, v25, 4

        vmv.v.x     v29, t2
        vmv.v.x     v25, t3
        vslideup.vi v29, v25, 4

        // delta0 = Clip((9*(q0-p0) - 3*(q1-p1) + 8) >> 4, -tc, tc)
        vsub.vv     v30, v12, v11
        vsll.vi     v31, v30, 3
        vadd.vv     v30, v30, v31
        vsub.vv     v31, v13, v10
        vsll.vi     v27, v31, 1
        vadd.vv     v31, v27, v31
        vsub.vv     v30, v30, v31
        vadd.vi     v30, v30, 8
        vsra.vi     v30, v30, 4

        // Apply only if |delta0| < 10*tc and group enabled
        vsll.vi     v27, v17, 1
        vsll.vi     v24, v17, 3
        vadd.vv     v27, v27, v24      // 10*tc
        vabs      v24, v30, v31
        vmsltu.vv   v0, v24, v27
        vmv.v.i     v27, 0
        vmerge.vim  v27, v27, -1, v0
        vand.vv     v27, v27, v26

        vneg.v      v31, v17
        vmax.vv     v30, v30, v31
        vmin.vv     v30, v30, v17

        // deltap1 = Clip(((p2+p0+1)>>1 - p1 + delta0) >> 1, -tc/2, tc/2)
        vadd.vv     v24, v9, v11
        vadd.vi     v24, v24, 1
        vsrl.vi     v24, v24, 1
        vsub.vv     v24, v24, v10
        vadd.vv     v24, v24, v30
        vsra.vi     v24, v24, 1

        // deltaq1 = Clip(((q2+q0+1)>>1 - q1 - delta0) >> 1, -tc/2, tc/2)
        vadd.vv     v25, v14, v12
        vadd.vi     v25, v25, 1
        vsrl.vi     v25, v25, 1
        vsub.vv     v25, v25, v13
        vsub.vv     v25, v25, v30
        vsra.vi     v25, v25, 1

        vsra.vi     v31, v17, 1        // tc/2
        vneg.v      v18, v31
        vmax.vv     v24, v24, v18
        vmin.vv     v24, v24, v31
        vmax.vv     v25, v25, v18
        vmin.vv     v25, v25, v31

        vand.vv     v24, v24, v28      // apply nd_p
        vand.vv     v25, v25, v29      // apply nd_q
        vand.vv     v24, v24, v27
        vand.vv     v30, v30, v27
        vand.vv     v25, v25, v27

        // Weak filter results
        vadd.vv     v18, v10, v24      // P1' weak
        vadd.vv     v19, v11, v30      // P0' weak
        vsub.vv     v20, v12, v30      // Q0' weak
        vadd.vv     v21, v13, v25      // Q1' weak

1:
        // Strong filter: tc2 = 2*tc
        vsll.vi     v24, v17, 1
        vneg.v      v25, v24

        // P0' = p0 + Clip((p2+2*p1+2*p0+2*q0+q1+4)>>3 - p0, -tc2, tc2)
        vadd.vv     v26, v10, v11
        vadd.vv     v26, v26, v12
        vsll.vi     v26, v26, 1
        vadd.vv     v27, v9, v13
        vadd.vv     v26, v26, v27
        vadd.vi     v26, v26, 4
        vsra.vi     v26, v26, 3
        vsub.vv     v26, v26, v11

        // P1' = p1 + Clip((p2+p1+p0+q0+2)>>2 - p1, -tc2, tc2)
        vadd.vv     v27, v9, v10
        vadd.vv     v28, v11, v12
        vadd.vv     v27, v27, v28
        vadd.vi     v27, v27, 2
        vsra.vi     v27, v27, 2
        vsub.vv     v27, v27, v10

        // P2' = p2 + Clip((2*p3+3*p2+p1+p0+q0+4)>>3 - p2, -tc2, tc2)
        vadd.vv     v28, v8, v9
        vsll.vi     v28, v28, 1
        vadd.vv     v29, v11, v12
        vadd.vv     v28, v28, v29
        vadd.vv     v29, v9, v10
        vadd.vv     v28, v28, v29
        vadd.vi     v28, v28, 4
        vsra.vi     v28, v28, 3
        vsub.vv     v28, v28, v9

        // Q0' = q0 + Clip((p1+2*p0+2*q0+2*q1+q2+4)>>3 - q0, -tc2, tc2)
        vadd.vv     v29, v11, v12
        vadd.vv     v29, v29, v13
        vsll.vi     v29, v29, 1
        vadd.vv     v30, v10, v14
        vadd.vv     v29, v29, v30
        vadd.vi     v29, v29, 4
        vsra.vi     v29, v29, 3
        vsub.vv     v29, v29, v12

        // Q1' = q1 + Clip((p0+q0+q1+q2+2)>>2 - q1, -tc2, tc2)
        vadd.vv     v30, v14, v13
        vadd.vv     v31, v11, v12
        vadd.vv     v30, v30, v31
        vadd.vi     v30, v30, 2
        vsra.vi     v30, v30, 2
        vsub.vv     v30, v30, v13

        // Q2' = q2 + Clip((2*q3+3*q2+q1+q0+p0+4)>>3 - q2, -tc2, tc2)
        vadd.vv     v31, v15, v14
        vsll.vi     v31, v31, 1
        vadd.vv     v0,  v14, v13
        vadd.vv     v31, v31, v0
        vadd.vv     v0,  v11, v12
        vadd.vv     v31, v31, v0
        vadd.vi     v31, v31, 4
        vsra.vi     v31, v31, 3
        vsub.vv     v31, v31, v14

        // Clip all deltas to [-tc2, tc2]
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

        vadd.vv     v28, v28, v9       // P2' strong
        vadd.vv     v27, v27, v10      // P1' strong
        vadd.vv     v26, v26, v11      // P0' strong
        vadd.vv     v29, v29, v12      // Q0' strong
        vadd.vv     v30, v30, v13      // Q1' strong
        vadd.vv     v31, v31, v14      // Q2' strong

        // If both groups strong, use strong results directly
        bne         a3, t4, 2f
        bne         a4, t4, 2f
        vmv.v.v     v1, v28
        vmv.v.v     v2, v27
        vmv.v.v     v3, v26
        vmv.v.v     v4, v29
        vmv.v.v     v5, v30
        vmv.v.v     v6, v31
        j           3f

2:
        // Merge strong/weak results using v1 selector mask
        vmsne.vi    v0, v1, 0
        vmerge.vvm  v1,  v9,  v28, v0  // P2: strong=v28, weak=orig
        vmerge.vvm  v2,  v18, v27, v0  // P1
        vmerge.vvm  v3,  v19, v26, v0  // P0
        vmerge.vvm  v4,  v20, v29, v0  // Q0
        vmerge.vvm  v5,  v21, v30, v0  // Q1
        vmerge.vvm  v6,  v14, v31, v0  // Q2: strong=v31, weak=orig

3:
        // Clamp to [0, 255]
        vmax.vx     v18, v1, zero
        vmax.vx     v19, v2, zero
        vmax.vx     v20, v3, zero
        vmax.vx     v21, v4, zero
        vmax.vx     v22, v5, zero
        vmax.vx     v23, v6, zero
.endm

.macro hevc_loop_filter_luma_8_rvv dir
	func ff_hevc_\dir\()_loop_filter_luma_8_rvv, zve32x
        lpad            0
.ifc \dir, v
        addi        a0, a0, -4
.else
        slli        t0, a1, 2
        sub         a0, a0, t0
.endif
        mv          a5, a0              // save base address for store
        lw          t6, 4(a3)           // tc[1]
        lw          a3, 0(a3)           // tc[0]
        or          t0, a3, t6
        beqz        t0, 4f              // skip if both tc == 0

        // Load 8 rows of 8 pixels
        vsetivli    zero, 8, e8, m1, ta, ma
        vle8.v      v0, (a0)
        add         a0, a0, a1
        vle8.v      v1, (a0)
        add         a0, a0, a1
        vle8.v      v2, (a0)
        add         a0, a0, a1
        vle8.v      v3, (a0)
        add         a0, a0, a1
        vle8.v      v4, (a0)
        add         a0, a0, a1
        vle8.v      v5, (a0)
        add         a0, a0, a1
        vle8.v      v6, (a0)
        add         a0, a0, a1
        vle8.v      v7, (a0)
.ifc \dir, v
        // Widen to 16-bit and transpose (vertical boundary -> horizontal)
        vsetivli    zero, 8, e16, m1, ta, ma
        vzext.vf2   v16,  v0
        vzext.vf2   v17,  v1
        vzext.vf2   v18, v2
        vzext.vf2   v19, v3
        vzext.vf2   v20, v4
        vzext.vf2   v21, v5
        vzext.vf2   v22, v6
        vzext.vf2   v23, v7

        transpose16_8x8 v16, v17, v18, v19, v20, v21, v22, v23, v9, v11, v13, v15, v8, v10, v12, v14, v1, v2
.else
        // Widen to 16-bit (horizontal boundary: pixels already in order)
        vsetivli    zero, 8, e16, m1, ta, ma
        vzext.vf2   v8,  v0   // P3
        vzext.vf2   v9,  v1   // P2
        vzext.vf2   v10, v2   // P1
        vzext.vf2   v11, v3   // P0
        vzext.vf2   v12, v4   // Q0
        vzext.vf2   v13, v5   // Q1
        vzext.vf2   v14, v6   // Q2
        vzext.vf2   v15, v7   // Q3
.endif
        hevc_loop_filter_luma_body
.ifc \dir, v
        // Transpose back and narrow to 8-bit
        transpose16_8x8 v8, v18, v19, v20, v21, v22, v23, v15, v25, v27, v29, v31, v24, v26, v28, v30, v1, v2

        vsetivli    zero, 8, e8, mf2, ta, ma
        vnclipu.wi  v0, v24, 0
        vnclipu.wi  v1, v25, 0
        vnclipu.wi  v2, v26, 0
        vnclipu.wi  v3, v27, 0
        vnclipu.wi  v4, v28, 0
        vnclipu.wi  v5, v29, 0
        vnclipu.wi  v6, v30, 0
        vnclipu.wi  v7, v31, 0

        // Store
        mv          a0, a5
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
        vnclipu.wi  v1, v18, 0
        vnclipu.wi  v2, v19, 0
        vnclipu.wi  v3, v20, 0
        vnclipu.wi  v4, v21, 0
        vnclipu.wi  v5, v22, 0
        vnclipu.wi  v6, v23, 0

        // Store (skip P3 row)
        mv          a0, a5
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
        ret
endfunc
.endm

hevc_loop_filter_luma_8_rvv h
hevc_loop_filter_luma_8_rvv v

.macro hevc_loop_filter_chroma_8_rvv dir
	func ff_hevc_\dir\()_loop_filter_chroma_8_rvv, zve32x
        lpad            0
        lw          t0, 0(a2)           // tc[0]
        lw          t1, 4(a2)           // tc[1]
        or          t2, t0, t1
        beqz        t2, 1f              // skip if both tc == 0

.ifc \dir, v
        addi        a0, a0, -2
.else
        slli        t2, a1, 1
        sub         a0, a0, t2
.endif
        mv          t3, a0              // save base address

        // Load p1, p0, q0, q1
        vsetivli    zero, 8, e8, m1, ta, ma
.ifc \dir, v
        vlse8.v     v0, (a0), a1
        addi        a0, a0, 1
        vlse8.v     v1, (a0), a1
        addi        a0, a0, 1
        vlse8.v     v2, (a0), a1
        addi        a0, a0, 1
        vlse8.v     v3, (a0), a1
.else
        vle8.v      v0, (a0)
        add         a0, a0, a1
        vle8.v      v1, (a0)
        add         a0, a0, a1
        vle8.v      v2, (a0)
        add         a0, a0, a1
        vle8.v      v3, (a0)
.endif
        vsetivli    zero, 8, e16, m1, ta, ma
        vzext.vf2   v20, v0             // p1
        vzext.vf2   v21, v1             // p0
        vzext.vf2   v22, v2             // q0
        vzext.vf2   v23, v3             // q1

        // v16 = [tc0 x4, tc1 x4], v17 = -tc
        vmv.v.x     v16, t0
        vmv.v.x     v17, t1
        vslideup.vi v16, v17, 4
        vneg.v      v17, v16

        // delta = Clip((((q0-p0) << 2) + (p1-q1) + 4) >> 3, -tc, tc)
        vsub.vv     v24, v22, v21
        vsll.vi     v24, v24, 2
        vsub.vv     v25, v20, v23
        vadd.vv     v24, v24, v25
        vadd.vi     v24, v24, 4
        vsra.vi     v24, v24, 3
        vmax.vv     v24, v24, v17
        vmin.vv     v24, v24, v16

        vadd.vv     v21, v21, v24       // p0 + delta
        vsub.vv     v22, v22, v24       // q0 - delta

        // Clamp to [0, 255] and narrow to 8-bit
        vmax.vx     v21, v21, zero
        vmax.vx     v22, v22, zero

        vsetivli    zero, 8, e8, mf2, ta, ma
        vnclipu.wi  v1, v21, 0
        vnclipu.wi  v2, v22, 0

.ifc \dir, v
        addi        a0, t3, 1           // p0 column
        vsse8.v     v1, (a0), a1
        addi        a0, a0, 1           // q0 column
        vsse8.v     v2, (a0), a1
.else
        add         a0, t3, a1          // p0 row
        vse8.v      v1, (a0)
        add         a0, a0, a1          // q0 row
        vse8.v      v2, (a0)
.endif

1:
        ret
endfunc
.endm

hevc_loop_filter_chroma_8_rvv h
hevc_loop_filter_chroma_8_rvv v
