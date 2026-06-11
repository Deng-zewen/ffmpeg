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
const int16_t g_t16[16][16] =
{
0    { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 },
1    { 90, 87, 80, 70, 57, 43, 25,  9, -9, -25, -43, -57, -70, -80, -87, -90 },
2    { 89, 75, 50, 18, -18, -50, -75, -89, -89, -75, -50, -18, 18, 50, 75, 89 },
3    { 87, 57,  9, -43, -80, -90, -70, -25, 25, 70, 90, 80, 43, -9, -57, -87 },
4    { 83, 36, -36, -83, -83, -36, 36, 83, 83, 36, -36, -83, -83, -36, 36, 83 },
5    { 80,  9, -70, -87, -25, 57, 90, 43, -43, -90, -57, 25, 87, 70, -9, -80 },
6    { 75, -18, -89, -50, 50, 89, 18, -75, -75, 18, 89, 50, -50, -89, -18, 75 },
7    { 70, -43, -87,  9, 90, 25, -80, -57, 57, 80, -25, -90, -9, 87, 43, -70 },
8    { 64, -64, -64, 64, 64, -64, -64, 64, 64, -64, -64, 64, 64, -64, -64, 64 },
9    { 57, -80, -25, 90, -9, -87, 43, 70, -70, -43, 87,  9, -90, 25, 80, -57 },
10    { 50, -89, 18, 75, -75, -18, 89, -50, -50, 89, -18, -75, 75, 18, -89, 50 },
11    { 43, -90, 57, 25, -87, 70,  9, -80, 80, -9, -70, 87, -25, -57, 90, -43 },
12    { 36, -83, 83, -36, -36, 83, -83, 36, 36, -83, 83, -36, -36, 83, -83, 36 },
13    { 25, -70, 90, -80, 43,  9, -57, 87, -87, 57, -9, -43, 80, -90, 70, -25 },
14    { 18, -50, 75, -89, 89, -75, 50, -18, -18, 50, -75, 89, -89, 75, -50, 18 },
15    {  9, -25, 43, -57, 70, -80, 87, -90, 90, -87, 80, -70, 57, -43, 25, -9 }
};
*/

.macro tr_16xN_firstpass_rvv step
        // v0 = O0 =  90*[1]+87*[3]+80*[5]+70*[7]+57*[9]+43*[11]+25*[13]+ 9*[15]
        // v16 = O1 = 87*[1]+57*[3]+ 9*[5]-43*[7]-80*[9]-90*[11]-70*[13]-25*[15]
        // v18 = O2 = 80*[1]+ 9*[3]-70*[5]-87*[7]-25*[9]+57*[11]+90*[13]+43*[15]
        // v2 = O3 =  70*[1]-43*[3]-87*[5]+ 9*[7]+90*[9]+25*[11]-80*[13]-57*[15]
        // v4 = O4 =  57*[1]-80*[3]-25*[5]+90*[7]- 9*[9]-87*[11]+43*[13]+70*[15]
        // v20 = O5 = 43*[1]-90*[3]+57*[5]+25*[7]-87*[9]+70*[11]+ 9*[13]-80*[15]
        // v22 = O6 = 25*[1]-70*[3]+90*[5]-80*[7]+43*[9]+ 9*[11]-57*[13]+87*[15]
        // v6 = O7 =  9*[1]-25*[3]+43*[5]-57*[7]+70*[9]-80*[11]+87*[13]-90*[15]

        // L 1
        addi            t0, a0, \step * 32
        vle16.v         v8, (t0)

        // L 3
        addi            t0, a0, \step * 32 * 3
        vle16.v         v10, (t0)

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

        // O[k] = g_t16[1][k] * src[1 * line
        // 90, 87, 80, 70, 57, 43, 25,  9
        // O[0] =  90*[1]
        vwmul.vx        v0, v8, a2
        // O[1] = 87*[1]
        vwmul.vx        v16, v8, a3
        // O[2] = 80*[1]
        vwmul.vx        v18, v8, a4
        // O[3] = 70*[1]
        vwmul.vx        v2, v8, a5
        // O[4] =  57*[1]
        vwmul.vx        v4, v8, a6
        // O[5] = 43*[1]
        vwmul.vx        v20, v8, a7
        // O[6] = 25*[1]
        vwmul.vx        v22, v8, s2
        // O[7] =  9*[1]
        vwmul.vx        v6, v8, s3

        // L 5
        addi            t0, a0, \step * 32 * 5
        vle16.v         v9, (t0)
        // L 7
        addi            t0, a0, \step * 32 * 7
        vle16.v         v29, (t0)

        // O[k] += g_t16[3][k] * src[3 * line]
        //  87, 57,  9, -43, -80, -90, -70, -25,
        add_member_16xN         v10,    a3, a6, s3, s9, s6, s4, s7, s10

        // L 9
        addi            t0, a0, \step * 32 * 9
        vle16.v         v28, (t0)
        // L 11
        addi            t0, a0, \step * 32 * 11
        vle16.v         v30, (t0)

        // O[k] += g_t16[5][k] * src[5 * line]
        // 80,  9, -70, -87, -25, 57, 90, 43,
        add_member_16xN         v9,    a4, s3, s7, s5, s10, a6, a2, a7

        // L 13
        addi            t0, a0, \step * 32 * 13
        vle16.v         v11, (t0)
        // L 15
        addi            t0, a0, \step * 32 * 15
        vle16.v         v31, (t0)

        // O[k] += g_t16[7][k] * src[7 * line]
        // 70, -43, -87,  9, 90, 25, -80, -57,
        add_member_16xN         v29,    a5, s9, s5, s3, a2, s2, s6, s8

        // L 2
        addi            t0, a0, \step * 64
        vle16.v         v8, (t0)

        // O[k] += g_t16[9][k] * src[9 * line]
        // 57, -80, -25, 90, -9, -87, 43, 70,
        add_member_16xN         v28,    a6, s6, s10, a2, s11, s5, a7, a5

        // L 6
        addi            t0, a0, \step * 32 * 6
        vle16.v         v10, (t0)

        // O[k] += g_t16[11][k] * src[11 * line]
        // 43, -90, 57, 25, -87, 70,  9, -80,
        add_member_16xN         v30,    a7, s4, a6, s2, s5, a5, s3, s6

        // L 10
        addi            t0, a0, \step * 32 * 10
        vle16.v         v24, (t0)

        // O[k] += g_t16[13][k] * src[13 * line]
        // 25, -70, 90, -80, 43,  9, -57, 87,
        add_member_16xN         v11,    s2, s7, a2, s6, a7, s3, s8, a3

        // L 14
        addi            t0, a0, \step * 32 * 14
        vle16.v         v26, (t0)

        // O[k] += g_t16[15][k] * src[15 * line]
        // 9, -25, 43, -57, 70, -80, 87, -90,
        add_member_16xN         v31,    s3, s10, a7, s8, a5, s6, a3, s4

        // EO[k] = g_t16[2][k] * src[2 * line]
        // EO0 = 89*src[ 2] + 75*src[ 6] + 50*src[10] + 18*src[14];
        // EO1 = 75*src[ 2] - 18*src[ 6] - 89*src[10] - 50*src[14];
        // EO2 = 50*src[ 2] - 89*src[ 6] + 18*src[10] + 75*src[14];
        // EO3 = 18*src[ 2] - 50*src[ 6] + 75*src[10] - 89*src[14];

        li              s2, -89
        li              s4, -50
        li              s5, -18
        li              s6, 89
        li              s7, 75
        li              s8, 50
        li              s9, 18
        // 89, 75, 50, 18
        // EO[0] = 89*src[ 2]
        vwmul.vx        v12, v8, s6
        // EO[1] = 75*src[ 2]
        vwmul.vx        v28, v8, s7
        // EO[2] = 50*src[ 2]
        vwmul.vx        v30, v8, s8
        // EO[3] = 18*src[ 2]
        vwmul.vx        v14, v8, s9

        // EO[k] += g_t16[6][k] * src[6 * line]
        // 75, -18, -89, -50
        // EO[0] = 75 * src[6]
        vwmacc.vx       v12, s7, v10
        // EO[1] = -18 * src[6]
        vwmacc.vx       v28, s5, v10
        // EO[2] = -89 * src[6]
        vwmacc.vx       v30, s2, v10
        // EO[3] = -50 * src[6]
        vwmacc.vx       v14, s4, v10

        // EO[k] += g_t16[10][k] * src[10 * line]
        // 50, -89, 18, 75,
        vwmacc.vx       v12, s8, v24
        vwmacc.vx       v28, s2, v24
        vwmacc.vx       v30, s9, v24
        vwmacc.vx       v14, s7, v24

        // EO[k] += g_t16[14][k] * src[14 * line]
        // 18, -50, 75, -89
        vwmacc.vx       v12, s9, v26
        vwmacc.vx       v28, s4, v26
        vwmacc.vx       v30, s7, v26
        vwmacc.vx       v14, s2, v26

        // EO[1] EO[2] are temporarily stored in the stack.
        vse32.v         v28, (a1)
        sh1add          t0, s1, a1
        vse32.v         v30, (t0)

        // L 0
        vle16.v         v10, (a0)
        // L 4
        addi            t0, a0, \step * 32 * 4
        vle16.v         v28, (t0)
        // L 8
        addi            a3, a0, \step * 32 * 8
        vle16.v         v26, (a3)
        // L 12
        addi            t0, a0, \step * 32 * 12
        vle16.v         v29, (t0)

        // EEE0 = 64*src[0] + 64*src[ 8];
        // EEE1 = 64*src[0] - 64*src[ 8];
        // EEO0 = 83*src[4] + 36*src[12];
        // EEO1 = 36*src[4] - 83*src[12];

        li              a2, 64
        li              a3, 83
        li              a5, 36
        li              a6, -64
        li              a7, -83

        // EEE0 = 64*[0]
        vwmul.vx        v8, v10, a2
        // EEE1 = 64*[0]
        vwmul.vx        v24, v10, a2
        // EEO0 = 83*[4]
        vwmul.vx        v10, v28, a3
        // EEE0 += 64*[0]
        vwmacc.vx       v8, a2, v26
        // EEE1 += -64*[8]
        vwmacc.vx       v24, a6, v26
        // EEO0 += 36*[12]
        vwmacc.vx       v10, a5, v29
        // EEO1 = 36*[4]
        vwmul.vx        v26, v28, a5
        // EEO1 += -83*[12]
        vwmacc.vx       v26, a7, v29
.endm

.macro tr_16xN_rvv name, shift, step
func func_tr_16xN_\name\()_rvv, zve32x, zba
        lpad            0
        tr_16xN_firstpass_rvv \step

        vsetvli         zero, zero, e32, m2, ta, ma
        // EE[0] = EEE[0] + EEO[0];
        // EE[3] = EEE[0] - EEO[0];
        butterfly       v8, v10, v28, v30

        // E[0] = EE[0] + EO[0]
        // E[7] = EE[0] - EO[0]
        butterfly       v28, v12, v8, v10
        // E[3] = EE[3] + EO[3]
        // E[4] = EE[3] - EO[3]
        butterfly       v30, v14, v28, v12

        // dst[0] = E[0] + O[0]
        // dst[15] = E[0] - O[0]
        butterfly       v8, v0, v14, v30

        # dst[3] = E[3] + O[3]
        # dst[12] = E[3] - O[3]
        butterfly       v28, v2, v8, v0

        # dst[4] = E[4] + O[4]
        # dst[11] = E[4] - O[4]
        butterfly       v12, v4, v28, v2

        # dst[7] = E[7] + O[7]
        # dst[8] = E[7] - O[7]
        butterfly       v10, v6, v12, v4

        vsetvli         zero, zero, e16, m1, ta, ma
        // dst[0]
        vnclip.wi       v6, v14, \shift
        // dst[7]
        vnclip.wi       v7, v12, \shift
        // dst[3]
        vnclip.wi       v13, v8, \shift
        // dst[4]
        vnclip.wi       v10, v28, \shift

        // Restore EO[1] from the stack
        vle32.v         v28, (a1)

        // dst[8]
        vnclip.wi       v14, v4, \shift
        // dst[11]
        vnclip.wi       v11, v2, \shift
        // dst[12]
        vnclip.wi       v12, v0, \shift
        // dst[15]
        vnclip.wi       v15, v30, \shift

        // Restore EO[2] from the stack
        sh1add          t0, s1, a1
        vle32.v         v30, (t0)

        vsetvli         zero, zero, e32, m2, ta, ma
        // EE[1] = EEE[1] + EEO[1];
        // EE[2] = EEE[1] - EEO[1];
        butterfly       v24, v26, v2, v4

        // E[1] = EE[1] + EO[1]
        // E[6] = EE[1] - EO[1]
        butterfly       v2, v28, v24, v26
        // E[2] = EE[2] + EO[2]
        // E[5] = EE[2] - EO[2]
        butterfly       v4, v30, v2, v28

        // dst[1] = E[1] + O[1]
        // dst[14] = E[1] - O[1]
        butterfly       v24, v16, v4, v30
        // dst[2] = E[2] + O[2]
        // dst[13] = E[2] - O[2]
        butterfly       v2, v18, v24, v16
        // dst[5] = E[5] + O[5]
        // dst[10] = E[5] - O[5]
        butterfly       v28, v20, v2, v18
        // dst[6] = E[6] + O[6]
        // dst[9] = E[6] - O[6]
        butterfly       v26, v22, v28, v20

        vsetvli         zero, zero, e16, m1, ta, ma

        li              t0, 16*2
        addi            t1, a1, 8
        addi            t2, a1, 16
        addi            t3, a1, 24

        // 0
        vmv.v.v         v0, v6
        // dst[1]
        vnclip.wi       v1, v4, \shift
        // dst[5]
        vnclip.wi       v5, v2, \shift
        // dst[2]
        vnclip.wi       v2, v24, \shift
        // 3
        vmv.v.v         v3, v13

        vssseg4e16.v    v0, (a1), t0

        // 4
        vmv.v.v         v4, v10
        // 5
        // dst[6]
        vnclip.wi       v6, v28, \shift
        // 7

        vssseg4e16.v    v4, (t1), t0

        // 8
        vmv.v.v         v8, v14
        // dst[9]
        vnclip.wi       v9, v20, \shift

        // dst[10]
        vnclip.wi       v10, v18, \shift
        // 11

        vssseg4e16.v    v8, (t2), t0

        // 12
        // dst[13]
        vnclip.wi       v13, v16, \shift
        // dst[14]
        vnclip.wi       v14, v30, \shift
        // 15

        vssseg4e16.v    v12, (t3), t0

        ret
endfunc
.endm

tr_16xN_rvv firstpass, 7, 1
tr_16xN_rvv secondpass_8, 20 - 8, 1

.macro idct_16x16 bitdepth
func ff_hevc_idct_16x16_\bitdepth\()_rvv, zve32x
        lpad            0
        addi            sp, sp, -(__riscv_xlen / 8)*12
        sx              ra, (__riscv_xlen / 8)*(11)(sp)
.irp i, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
        sx              s\i, (__riscv_xlen / 8)*(11-\i)(sp)
.endr
        mv              t6, a0
        csrwi           vxrm, 0
        addi            sp, sp, -512

        // Set the maximum number of elements that a vector register can process.
        vsetivli        t0, 16, e16, m1, ta, ma
        // Number of bytes loaded into a vector register
        slli            s1, t0, 1
        // Number of bytes processed in one loop
        slli            t5, t0, 5

        mv              a1, sp
1:
        jal             func_tr_16xN_firstpass_rvv
        add             a0, a0, s1
        add             a1, a1, t5
        addi            a3, t6, (16 * 2)
        bgt             a3, a0, 1b

        mv              a0, sp
        mv              a1, t6
1:
        jal             func_tr_16xN_secondpass_\bitdepth\()_rvv
        add             a0, a0, s1
        add             a1, a1, t5
        addi            a3, sp, (16 * 2)
        bgt             a3, a0, 1b

        addi            sp, sp, 512
        lx              ra, (__riscv_xlen / 8)*(11)(sp)
.irp i, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
        lx              s\i, (__riscv_xlen / 8)*(11-\i)(sp)
.endr
        addi            sp, sp, (__riscv_xlen / 8)*12
        ret
endfunc
.endm

idct_16x16 8

.macro dst_add_member32_m1_m2   in, tmp0, tmp1, tmp2, tmp3, op0, op1, op2, op3, first
        // By replacing neg, vwmacc.vx with vsext.vf2, vnmsac.vx, vmacc.vx, the performance is degraded.
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
        // O[k] = g_t32[1][k] * src[line] + g_t32[3][k] * src[3 * line] + g_t32[5][k] * src[5 * line] + g_t32[7][k] * src[7 * line] +
        //        g_t32[9][k] * src[9 * line] + g_t32[11][k] * src[11 * line] + g_t32[13][k] * src[13 * line] + g_t32[15][k] * src[15 * line] +
        //        g_t32[17][k] * src[17 * line] + g_t32[19][k] * src[19 * line] + g_t32[21][k] * src[21 * line] + g_t32[23][k] * src[23 * line] +
        //        g_t32[25][k] * src[25 * line] + g_t32[27][k] * src[27 * line] + g_t32[29][k] * src[29 * line] + g_t32[31][k] * src[31 * line];

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

.macro tr_16xN_32x32_rvv name, step
func func_tr_16xN_32x32_\name\()_rvv, zve32x, zba
        lpad            0

        tr_16xN_firstpass_rvv \step

        // 4
        sh3add          a5, s1, t4
        // 5
        sh1add          a6, s1, a5
        // 6
        sh1add          a7, s1, a6
        // 7
        sh1add          s2, s1, a7

        // 8
        sh1add          s3, s1, s2
        // 9
        sh1add          s4, s1, s3
        // 10
        sh1add          s5, s1, s4
        // 11
        sh1add          s6, s1, s5

        // 12
        sh1add          s7, s1, s6
        // 13
        sh1add          s8, s1, s7
        // 14
        sh1add          s9, s1, s8
        // 15
        sh1add          s10, s1, s9

        vsetvli         zero, zero, e32, m2, ta, ma
        // EEE[0] = EEEE[0] + EEEO[0];
        // EEE[3] = EEEE[0] - EEEO[0];
        butterfly       v8, v10, v28, v30

        // EE[0] = EEE[0] + EEO[0]
        // EE[7] = EEE[0] - EEO[0]
        butterfly       v28, v12, v8, v10

        // EE[3] = EEE[3] + EEO[3]
        // EE[4] = EEE[3] - EEO[3]
        butterfly       v30, v14, v28, v12

        //E[0] = EE[0] + EO[0]
        //E[15] = EE[0] - EO[0]
        butterfly       v8, v0, v30, v14

        // E0  Do not store, used on the first tr_block.
        // E15
        vse32.v         v14, (s10)

        # E[3] = EE[3] + EO[3]
        # E[12] = EE[3] - EO[3]
        butterfly       v28, v2, v8, v0

        // E3 Do not store, used on the first tr_block.
        // E12
        vse32.v         v0, (s7)

        # E[4] = EE[4] + EO[4]
        # E[11] = EE[4] - EO[4]
        butterfly       v12, v4, v28, v2

        // E4
        vse32.v         v28, (a5)
        // E11
        vse32.v         v2, (s6)

        # E[7] = EE[7] + EO[7]
        # E[8] = EE[7] - EO[7]
        butterfly       v10, v6, v12, v4

        // E7
        vse32.v         v12, (s2)
        // E8
        vse32.v         v4, (s3)

        // Restore EEO[1] to V2
        vle32.v         v2, (a1)
        // Restore EEO[2] to V4
        sh1add          t0, s1, a1
        vle32.v         v4, (t0)

        // EEE[1] = EEEE[1] + EEEO[1];
        // EEE[2] = EEEE[1] - EEEO[1];
        butterfly       v24, v26, v6, v10

        // EE[1] = EEE[1] + EEO[1]
        // EE[6] = EEE[1] - EEO[1]
        butterfly       v6, v2, v24, v26
        // EE[2] = EEE[2] + EEO[2]
        // EE[5] = EEE[2] - EEO[2]
        butterfly       v10, v4, v6, v2

        // E[1] = EE[1] + EO[1]
        // E[14] = EE[1] - EO[1]
        butterfly       v24, v16, v28, v4
        // E1  Do not store, used on the first tr_block.
        // 14
        vse32.v         v4, (s9)

        // E[2] = EE[2] + EO[2]
        // E[13] = EE[2] - EO[2]
        butterfly       v6, v18, v24, v16

        // E2  Do not store, used on the first tr_block.
        // E13
        vse32.v         v16, (s8)

        // E[5] = EE[5] + EO[5]
        // E[10] = EE[5] - EO[5]
        butterfly       v2, v20, v6, v18

        // E5
        vse32.v         v6, (a6)
        // E10
        vse32.v         v18, (s5)

        // E[6] = EE[6] + EO[6]
        // E[9] = EE[6] - EO[6]
        butterfly       v26, v22, v2, v20
        // move E3 from V8 to v26
        vmv.v.v         v26, v8

        // E6
        vse32.v         v2, (a7)
        // E9
        vse32.v         v20, (s4)

        // E0 v30 E1 v28 E2 v24 E3 v26
        ret
endfunc
.endm

tr_16xN_32x32_rvv noscale, 4

.macro scale_store_rvv shift, off
.if \off != 24
        vse16.v         v14, (t4)
        add             t1, t4, s1
        vse16.v         v15, (t1)
.endif

        vsetvli         zero, zero, e32, m2, ta, ma
        // dst[0] = E[0] + O[0]
        // dst[31] = E[0] - O[0]
        butterfly       v30, v22, v14, v30

        // dst[1] = E[1] + O[1]
        // dst[30] = E[1] - O[1]
        butterfly       v28, v20, v22, v28

        // dst[2] = E[2] + O[2]
        // dst[29] = E[2] - O[2]
        butterfly       v24, v18, v20, v24

        // dst[3] = E[3] + O[3]
        // dst[28] = E[3] - O[3]
        butterfly       v26, v16, v18, v26

        vsetvli         zero, zero, e16, m1, ta, ma

        // dst 28
        vnclip.wi       v16, v26, \shift
        // dst 29
        vnclip.wi       v17, v24, \shift

        // dst 0
        vnclip.wi       v24, v14, \shift
        // dst 1
        vnclip.wi       v25, v22, \shift

        // dst 2
        vnclip.wi       v26, v20, \shift
        // dst 3
        vnclip.wi       v27, v18, \shift

        // dst 30
        vnclip.wi       v18, v28, \shift
        // dst 31
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

.macro tr_32xN_rvv name, shift
func func_tr_32xN_\name\()_rvv, zve32x, zba
        lpad            0

        // line 1
        addi            t0, a0, 64 * 1
        vle16.v         v0, (t0)
        // line 3
        addi            t0, a0, 64 * 3
        vle16.v         v1, (t0)
        // line 5
        addi            t0, a0, 64 * 5
        vle16.v         v2, (t0)
        // line 7
        addi            t0, a0, 64 * 7
        vle16.v         v3, (t0)

        // line 9
        addi            t0, a0, 64 * 9
        vle16.v         v4, (t0)
        // line 11
        addi            t0, a0, 64 * 11
        vle16.v         v5, (t0)
        // line 13
        addi            t0, a0, 64 * 13
        vle16.v         v6, (t0)
        // line 15
        addi            t0, a0, 64 * 15
        vle16.v         v7, (t0)

        // line 17
        addi            t0, a0, 64 * 17
        vle16.v         v8, (t0)
        // line 19
        addi            t0, a0, 64 * 19
        vle16.v         v9, (t0)
        // line 21
        addi            t0, a0, 64 * 21
        vle16.v         v10, (t0)
        // line 23
        addi            t0, a0, 64 * 23
        vle16.v         v11, (t0)

        // line 25
        addi            t0, a0, 64 * 25
        vle16.v         v12, (t0)
        // line 27
        addi            t0, a0, 64 * 27
        vle16.v         v13, (t0)
        // line 29
        addi            t0, a0, 64 * 29
        vle16.v         v14, (t0)
        // line 31
        addi            t0, a0, 64 * 31
        vle16.v         v15, (t0)

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
        scale_store_rvv \shift, 0

        // E4
        sh3add          t0, s1, t4
        vle32.v         v30, (t0)
        // E5
        sh1add          t1, s1, t0
        vle32.v         v28, (t1)
        // E6
        sh1add          t2, s1, t1
        vle32.v         v24, (t2)
        // E7
        sh1add          t3, s1, t2
        vle32.v         v26, (t3)

        tr_block        4
        scale_store_rvv \shift, 8

        // E8
        add             t0, t4, t5
        vle32.v         v30, (t0)
        // E9
        sh1add          t1, s1, t0
        vle32.v         v28, (t1)
        // E10
        sh1add          t2, s1, t1
        vle32.v         v24, (t2)
        // E11
        sh1add          t3, s1, t2
        vle32.v         v26, (t3)

        tr_block        8
        scale_store_rvv \shift, 16

        slli            t3, s1, 3
        sh1add          t3, t3, t3
        // E12
        add             t0, t4, t3
        vle32.v         v30, (t0)
        // E13
        sh1add          t1, s1, t0
        vle32.v         v28, (t1)
        // E14
        sh1add          t2, s1, t1
        vle32.v         v24, (t2)
        // E15
        sh1add          t3, s1, t2
        vle32.v         v26, (t3)

        tr_block        12
        scale_store_rvv \shift, 24

        ret
endfunc
.endm

tr_32xN_rvv firstpass, 7
tr_32xN_rvv secondpass_8, 20 - 8

.macro idct_32x32 bitdepth
func ff_hevc_idct_32x32_\bitdepth\()_rvv, zve32x, zba
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
        jal     func_tr_16xN_32x32_noscale_rvv
        jal     func_tr_32xN_firstpass_rvv
        add     a0, a0, s1
        sh1add  a1, t5, a1
        addi    a3, t6, (32 * 2)
        bgt     a3, a0, 1b

        mv      a0, sp
        mv      a1, t6
1:
        jal     func_tr_16xN_32x32_noscale_rvv
        jal     func_tr_32xN_secondpass_\bitdepth\()_rvv
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

idct_32x32 8

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

.macro tr_8x8_rvv shift
        // EO[0] = g_t8[2][0] * src[2 * line] + g_t8[6][0] * src[6 * line];
        // EO[1] = g_t8[2][1] * src[2 * line] + g_t8[6][1] * src[6 * line];
        // EE[0] = g_t8[0][0] * src[0] + g_t8[4][0] * src[4 * line];
        // EE[1] = g_t8[0][1] * src[0] + g_t8[4][1] * src[4 * line];

        // EE[0]
        vwmul.vx        v24, v8, a1
        // EE[1]
        vwmul.vx        v26, v8, a1
        // EO[0]
        vwmul.vx        v28, v10, a2
        // EO[1]
        vwmul.vx        v30, v10, a3

        vwmacc.vx       v24, a1, v12
        vwmacc.vx       v26, t1, v12
        vwmacc.vx       v28, a3, v14
        vwmacc.vx       v30, t2, v14

        // O[k] = g_t8[1][k] * src[line] + g_t8[3][k] * src[3 * line]
        //      + g_t8[5][k] * src[5 * line] + g_t8[7][k] * src[7 * line];
        // O0
        vwmul.vx        v0, v9, a4
        // O1
        vwmul.vx        v2, v9, a5
        // O2
        vwmul.vx        v4, v9, a6
        // O3
        vwmul.vx        v6, v9, a7

        // O0
        vwmacc.vx       v0, a5, v11
        // O1
        vwmacc.vx       v2, t5, v11
        // O2
        vwmacc.vx       v4, t4, v11
        // O3
        vwmacc.vx       v6, t6, v11

        // O0
        vwmacc.vx       v0, a6, v13
        // O1
        vwmacc.vx       v2, t4, v13
        // O2
        vwmacc.vx       v4, a7, v13
        // O3
        vwmacc.vx       v6, a5, v13

        // O0
        vwmacc.vx       v0, a7, v15
        // O1
        vwmacc.vx       v2, t6, v15
        // O2
        vwmacc.vx       v4, a5, v15
        // O3
        vwmacc.vx       v6, t4, v15

        vsetvli         zero, zero, e32, m2, ta, ma
        // E[0] = EE[0] + EO[0];
        // E[3] = EE[0] - EO[0];
        butterfly       v24, v28, v16, v22
        // E[1] = EE[1] + EO[1];
        // E[2] = EE[1] - EO[1];
        butterfly       v26, v30, v18, v20

        // dst[k] = E[k] + O[k]
        // dst[7-k] = E[k] + O[k]
        //  dst 0 7
        butterfly       v16, v0, v14, v0
        //  dst 1 6
        butterfly       v18, v2, v16, v18
        //  dst 2 5
        butterfly       v20, v4, v2, v20
        //  dst 3 4
        butterfly       v22, v6, v4, v22

        // clip dst
        vsetvli         zero, zero,  e16, m1, ta, ma
        vnclip.wi       v24, v14, \shift
        vnclip.wi       v25, v16, \shift

        vnclip.wi       v26, v2, \shift
        vnclip.wi       v27, v4, \shift

        vnclip.wi       v28, v22, \shift
        vnclip.wi       v29, v20, \shift

        vnclip.wi       v30, v18, \shift
        vnclip.wi       v31, v0, \shift
.endm

.macro idct_8x8 bitdepth
func ff_hevc_idct_8x8_\bitdepth\()_rvv, zve32x
        csrwi           vxrm, 0

        vsetivli        zero, 8, e16, m1, ta, ma
        // L 0
        vle16.v         v8, (a0)
        // L 1
        addi            t0, a0, 8*2*1
        vle16.v         v9, (t0)
        // L 2
        addi            t0, a0, 8*2*2
        vle16.v         v10, (t0)
        // L 3
        addi            t0, a0, 8*2*3
        vle16.v         v11, (t0)
        // L 4
        addi            t0, a0, 8*2*4
        vle16.v         v12, (t0)
        // L 5
        addi            t0, a0, 8*2*5
        vle16.v         v13, (t0)
        // L 6
        addi            t0, a0, 8*2*6
        vle16.v         v14, (t0)
        // L 7
        addi            t0, a0, 8*2*7
        vle16.v         v15, (t0)

        li              a1, 64
        li              a2, 83
        li              a3, 36
        li              t1, -64
        li              t2, -83
        li              a4, 89
        li              a5, 75
        li              a6, 50
        li              a7, 18
        li              t4, -89
        li              t6, -50
        li              t5, -18

        tr_8x8_rvv      7
        transpose16_8x8 v24, v25, v26, v27, v28, v29, v30, v31,  v9, v11, v13, v15,  v8, v10, v12, v14,  v7, v6

        tr_8x8_rvv      20 - \bitdepth\()
        li              t3, 8*2
        vssseg4e16.v    v24, (a0), t3
        add             t0, a0, 2*4
        vssseg4e16.v    v28, (t0), t3
        ret
endfunc
.endm

idct_8x8 8

.macro transpose16_4x4 r0, r1, r2, r3, tmp0, tmp1, tmp2, tmp3, tmp4, tmp5
        vid.v           \tmp5
        vand.vi         \tmp4, \tmp5, 0x1
        vmsne.vi        v0, \tmp4, 0

        // 0 4  2 6
        vslideup.vi     \tmp0, \r1, 1
        vmerge.vvm      \tmp2, \r0, \tmp0, v0
        // 8 12 10 14
        vslideup.vi     \tmp1, \r3, 1
        vmerge.vvm      \tmp3, \r2, \tmp1, v0

        // 1 5 3 7
        vslidedown.vi   \tmp0, \r0, 1
        vmerge.vvm      \r0, \tmp0, \r1, v0
        //9 13 11 15
        vslidedown.vi   \tmp1, \r2, 1
        vmerge.vvm      \r2, \tmp1, \r3, v0

        vand.vi         \tmp4, \tmp5, 0x2
        vmsne.vi        v0, \tmp4, 0

        // 0 4 8 12
        vslideup.vi     \tmp0, \tmp3, 2
        vmerge.vvm      \tmp0, \tmp2, \tmp0, v0
        // 2 6 10 14
        vslidedown.vi   \tmp1, \tmp2, 2
        vmerge.vvm      \tmp2, \tmp1, \tmp3, v0

        // 1 5 9 13
        vslideup.vi     \tmp1, \r2, 2
        vmerge.vvm      \tmp1, \r0, \tmp1, v0

        // 3 7 11 15
        vslidedown.vi   \tmp3, \r0, 2
        vmerge.vvm      \tmp3, \tmp3, \r2, v0
.endm

.macro tr_4x4 shift
        // E0
        vwmul.vx        v20, v4, a1
        // O0
        vwmul.vx        v22, v1, a2
        // E1
        vwmul.vx        v21, v4, a1
        // O1
        vwmul.vx        v23, v1, a3

        vwmacc.vx       v20, a1, v2
        vwmacc.vx       v22, a3, v3
        vwmacc.vx       v21, t1, v2
        vwmacc.vx       v23, t2, v3

        vsetvli         zero, zero, e32, m1, ta, ma
        // dst 0 3
        butterfly       v20, v22, v24, v20
        // dst 1 2
        butterfly       v21, v23, v22, v21

        vsetvli         zero, zero, e16, mf2, ta, ma
        // clip dst 0 1 2 3
        vnclip.wi       v16, v24, \shift
        vnclip.wi       v17, v22, \shift
        vnclip.wi       v18, v21, \shift
        vnclip.wi       v19, v20, \shift
.endm

.macro idct_4x4 bitdepth
func ff_hevc_idct_4x4_\bitdepth\()_rvv, zve32x
        csrwi           vxrm, 0

        vsetivli        zero, 4, e16, mf2, ta, ma
        // L 0
        vle16.v         v4, (a0)
        // L 1
        addi            t0, a0, 4 * 2 * 1
        vle16.v         v1, (t0)
        // L 2
        addi            t0, a0, 4 * 2 * 2
        vle16.v         v2, (t0)
        // L 3
        addi            t0, a0, 4 * 2 * 3
        vle16.v         v3, (t0)

        li              a1, 64
        li              a2, 83
        li              a3, 36
        li              t1, -64
        li              t2, -83

        tr_4x4          7
        transpose16_4x4 v16, v17, v18, v19, v4, v1, v2, v3, v5, v6
        tr_4x4          20 - \bitdepth
        li              t0, 4*2
        vssseg4e16.v    v16, (a0), t0
        ret
endfunc
.endm

idct_4x4 8


func ff_hevc_add_residual_4x4_8_rvv, zve32x
        mv              t1, a0
        add             t3, a0, a2          # t3 = 第2行指针（代替 a12）
        slli            a2, a2, 1
        vsetivli        zero, 4, e8, mf4, ta, ma
.rept 2
        vle8.v          v2, (a0)
        vle8.v          v3, (t3)

        vsetivli        zero, 4, e16, mf2, ta, ma
        vle16.v         v0, (a1)
        addi            a1, a1, 8
        vle16.v         v1, (a1)
        addi            a1, a1, 8

        vzext.vf2       v4, v2
        vzext.vf2       v5, v3

        vsadd.vv        v0, v0, v4
        vsadd.vv        v1, v1, v5

        vmax.vx         v0, v0, zero
        vmax.vx         v1, v1, zero

        vsetivli        zero, 4, e8, mf2, ta, ma
        vnclipu.wi      v0, v0, 0
        vnclipu.wi      v1, v1, 0

        vse8.v          v0, (a0)
        add             a0, a0, a2
        vse8.v          v1, (t3)
        add             t3, t3, a2
.endr
        ret
endfunc


func ff_hevc_add_residual_8x8_8_rvv, zve32x
        add             t3, a0, a2          # t3 = 第2行指针（代替 a12）
        slli            a2, a2, 1
        vsetivli        zero, 8, e8, mf2, ta, ma
.rept 4
        vle8.v          v2, (a0)
        vle8.v          v3, (t3)

        vsetivli        zero, 8, e16, m1, ta, ma
        vle16.v         v0, (a1)
        addi            a1, a1, 16
        vle16.v         v1, (a1)
        addi            a1, a1, 16

        vzext.vf2       v4, v2
        vzext.vf2       v5, v3

        vsadd.vv        v0, v0, v4
        vsadd.vv        v1, v1, v5

        vmax.vx         v0, v0, zero
        vmax.vx         v1, v1, zero

        vsetivli        zero, 8, e8, mf2, ta, ma
        vnclipu.wi      v0, v0, 0
        vnclipu.wi      v1, v1, 0

        vse8.v          v0, (a0)
        add             a0, a0, a2
        vse8.v          v1, (t3)
        add             t3, t3, a2
.endr
        ret
endfunc


func ff_hevc_add_residual_16x16_8_rvv, zve32x
        vsetivli        zero, 16, e8, m1, ta, ma
.rept 16
        vle8.v          v2, (a0)

        vsetivli        zero, 16, e16, m2, ta, ma
        vle16.v         v0, (a1)
        addi            a1, a1, 32

        vzext.vf2       v4, v2
        vsadd.vv        v0, v0, v4
        vmax.vx         v0, v0, zero
        vsetivli        zero, 16, e8, m1, ta, ma
        vnclipu.wi      v0, v0, 0

        vse8.v          v0, (a0)
        add             a0, a0, a2
.endr
        ret
endfunc


func ff_hevc_add_residual_32x32_8_rvv, zve32x
        li              t0, 32
        vsetvli         zero, t0, e8, m2, ta, ma
.rept 32
        vle8.v          v4, (a0)

        vsetvli         zero, t0, e16, m4, ta, ma
        vle16.v         v0, (a1)
        addi            a1, a1, 64

        vzext.vf2       v8, v4
        vsadd.vv        v0, v0, v8
        vmax.vx         v0, v0, zero
        vsetvli         zero, t0, e8, m2, ta, ma
        vnclipu.wi      v0, v0, 0
        
        vse8.v          v0, (a0)
        add             a0, a0, a2
.endr
        ret
endfunc

