/*
 * Copyright (C) 2026 Alibaba Group Holding Limited.
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
#define XLEN_BYTE __riscv_xlen/8

const qpel_filters
    .byte  0,  0,  0,  0,  0,  0, 0,  0
    .byte -1,  4, -10, 58, 17, -5, 1,  0
    .byte -1,  4, -11, 40, 40, -11, 4, -1
    .byte  0,  1, -5, 17, 58, -10, 4, -1
endconst

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

/* clobbers t0, t1 */
.macro load_filter m
    la          t0, qpel_filters
    sh3add      t0, \m, t0
    lb          s1, 0(t0)
    lb          s2, 1(t0)
    lb          s3, 2(t0)
    lb          s4, 3(t0)
    lb          s5, 4(t0)
    lb          s6, 5(t0)
    lb          s7, 6(t0)
    lb          s8, 7(t0)
.endm

/* output is unclipped; clobbers t4 */
.macro filter_h         vdst, vsrc0, vsrc1, vsrc2, vsrc3, vsrc4, vsrc5, vsrc6, vsrc7, src
    addi             t4, \src, -3
    vle8.v           \vsrc0, (t4)
    addi             t4, \src, -2
    vle8.v           \vsrc1, (t4)
    addi             t4, \src, -1
    vle8.v           \vsrc2, (t4)
    vwmulu.vx        \vdst, \vsrc1, s2
    vwmaccsu.vx      \vdst, s1, \vsrc0
    vle8.v           \vsrc3, (\src)
    addi             t4, \src, 1
    vwmaccsu.vx      \vdst, s3, \vsrc2
    vle8.v           \vsrc4, (t4)
    addi             t4, \src, 2
    vwmaccsu.vx      \vdst, s4, \vsrc3
    vle8.v           \vsrc5, (t4)
    addi             t4, \src, 3
    vwmaccsu.vx      \vdst, s5, \vsrc4
    vle8.v           \vsrc6, (t4)
    addi             t4, \src, 4
    vwmaccsu.vx      \vdst, s6, \vsrc5
    vle8.v           \vsrc7, (t4)
    vwmaccsu.vx      \vdst, s7, \vsrc6
    vwmaccsu.vx      \vdst, s8, \vsrc7
.endm

func ff_hevc_put_qpel_h_8_m1_rvv, zve32x, zba
    addi        sp, sp, -8*XLEN_BYTE
    sx          s1, 0(sp)
    sx          s2, XLEN_BYTE(sp)
    sx          s3, 2*XLEN_BYTE(sp)
    sx          s4, 3*XLEN_BYTE(sp)
    sx          s5, 4*XLEN_BYTE(sp)
    sx          s6, 5*XLEN_BYTE(sp)
    sx          s7, 6*XLEN_BYTE(sp)
    sx          s8, 7*XLEN_BYTE(sp)
    load_filter a4
    mv          t3, a6
    li          t1, 0       # offset

1:
    vsetvli     t6, t3, e8, m1, ta, ma
    add         t2, a1, t1
    filter_h    v0, v16, v18, v20, v22, v24, v26, v28, v30, t2
    vsetvli     zero, zero, e16, m2, ta, ma
    sh1add      t2, t1, a0
    vse16.v     v0, (t2)
    sub         t3, t3, t6
    add         t1, t1, t6
    bnez        t3, 1b
    addi        a3, a3, -1
    mv          t3, a6
    add         a1, a1, a2
    addi        a0, a0, 2*HEVC_MAX_PB_SIZE
    li          t1, 0
    bnez        a3, 1b

    lx          s1, 0(sp)
    lx          s2, XLEN_BYTE(sp)
    lx          s3, 2*XLEN_BYTE(sp)
    lx          s4, 3*XLEN_BYTE(sp)
    lx          s5, 4*XLEN_BYTE(sp)
    lx          s6, 5*XLEN_BYTE(sp)
    lx          s7, 6*XLEN_BYTE(sp)
    lx          s8, 7*XLEN_BYTE(sp)
    addi        sp, sp, 8*XLEN_BYTE
    ret
endfunc

func ff_hevc_put_qpel_uni_h_8_m1_rvv, zve32x, zba
    csrwi       vxrm, 0
    addi        sp, sp, -8*XLEN_BYTE
    sx          s1, 0(sp)
    sx          s2, XLEN_BYTE(sp)
    sx          s3, 2*XLEN_BYTE(sp)
    sx          s4, 3*XLEN_BYTE(sp)
    sx          s5, 4*XLEN_BYTE(sp)
    sx          s6, 5*XLEN_BYTE(sp)
    sx          s7, 6*XLEN_BYTE(sp)
    sx          s8, 7*XLEN_BYTE(sp)
    load_filter a5
    mv          t3, a7
    li          t1, 0       # offset

1:
    vsetvli     t6, t3, e8, m1, ta, ma
    add         t2, a2, t1
    filter_h    v0, v16, v18, v20, v22, v24, v26, v28, v30, t2
    vsetvli     zero, zero, e16, m2, ta, ma
    vmax.vx     v0, v0, zero
    vsetvli     zero, zero, e8, m1, ta, ma
    vnclipu.wi  v0, v0, 6
    add         t2, a0, t1
    vse8.v      v0, (t2)
    sub         t3, t3, t6
    add         t1, t1, t6
    bnez        t3, 1b
    addi        a4, a4, -1
    mv          t3, a7
    add         a2, a2, a3
    add         a0, a0, a1
    li          t1, 0
    bnez        a4, 1b

    lx          s1, 0(sp)
    lx          s2, XLEN_BYTE(sp)
    lx          s3, 2*XLEN_BYTE(sp)
    lx          s4, 3*XLEN_BYTE(sp)
    lx          s5, 4*XLEN_BYTE(sp)
    lx          s6, 5*XLEN_BYTE(sp)
    lx          s7, 6*XLEN_BYTE(sp)
    lx          s8, 7*XLEN_BYTE(sp)
    addi        sp, sp, 8*XLEN_BYTE
    ret
endfunc

func ff_hevc_put_qpel_uni_w_h_8_m1_rvv, zve32x, zba
    csrwi       vxrm, 0
    lx          t2, 0(sp)       # mx
    addi        a5, a5, 6       # shift
    lw          t3, 2*XLEN_BYTE(sp)       # width

    addi        sp, sp, -8*XLEN_BYTE
    sx          s1, 0(sp)
    sx          s2, XLEN_BYTE(sp)
    sx          s3, 2*XLEN_BYTE(sp)
    sx          s4, 3*XLEN_BYTE(sp)
    sx          s5, 4*XLEN_BYTE(sp)
    sx          s6, 5*XLEN_BYTE(sp)
    sx          s7, 6*XLEN_BYTE(sp)
    sx          s8, 7*XLEN_BYTE(sp)
    load_filter t2
    li          t2, 0           # offset

1:
    vsetvli     t6, t3, e8, m1, ta, ma
    add         t1, a2, t2
    filter_h    v8, v16, v18, v20, v22, v24, v26, v28, v30, t1
    vsetvli     zero, zero, e16, m2, ta, ma
    vwmul.vx    v0, v8, a6
    vsetvli     zero, zero, e32, m4, ta, ma
    vssra.vx    v0, v0, a5
    vsadd.vx    v0, v0, a7
    vmax.vx     v0, v0, zero
    vsetvli     zero, zero, e16, m2, ta, ma
    vnclip.wi   v0, v0, 0
    vsetvli     zero, zero, e8, m1, ta, ma
    vnclipu.wi  v0, v0, 0
    add         t1, a0, t2
    vse8.v      v0, (t1)
    sub         t3, t3, t6
    add         t2, t2, t6
    bnez        t3, 1b
    addi        a4, a4, -1
    lw          t3, 10*XLEN_BYTE(sp)
    add         a2, a2, a3
    add         a0, a0, a1
    li          t2, 0
    bnez        a4, 1b

    lx          s1, 0(sp)
    lx          s2, XLEN_BYTE(sp)
    lx          s3, 2*XLEN_BYTE(sp)
    lx          s4, 3*XLEN_BYTE(sp)
    lx          s5, 4*XLEN_BYTE(sp)
    lx          s6, 5*XLEN_BYTE(sp)
    lx          s7, 6*XLEN_BYTE(sp)
    lx          s8, 7*XLEN_BYTE(sp)
    addi        sp, sp, 8*XLEN_BYTE
    ret
endfunc

func ff_hevc_put_qpel_bi_h_8_m1_rvv, zve32x, zba
    csrwi       vxrm, 0
    lw          t3, 0(sp)      # width
    addi        sp, sp, -8*XLEN_BYTE
    sx          s1, 0(sp)
    sx          s2, XLEN_BYTE(sp)
    sx          s3, 2*XLEN_BYTE(sp)
    sx          s4, 3*XLEN_BYTE(sp)
    sx          s5, 4*XLEN_BYTE(sp)
    sx          s6, 5*XLEN_BYTE(sp)
    sx          s7, 6*XLEN_BYTE(sp)
    sx          s8, 7*XLEN_BYTE(sp)
    load_filter a6
    li          t1, 0          # offset

1:
    vsetvli     t6, t3, e16, m2, ta, ma
    sh1add      t2, t1, a4
    vle16.v     v12, (t2)
    vsetvli     zero, zero, e8, m1, ta, ma
    add         t2, a2, t1
    filter_h    v0, v16, v18, v20, v22, v24, v26, v28, v30, t2
    vsetvli     zero, zero, e16, m2, ta, ma
    vsadd.vv    v0, v0, v12
    vmax.vx     v0, v0, zero
    vsetvli     zero, zero, e8, m1, ta, ma
    vnclipu.wi  v0, v0, 7
    add         t2, a0, t1
    vse8.v      v0, (t2)
    sub         t3, t3, t6
    add         t1, t1, t6
    bnez        t3, 1b
    addi        a5, a5, -1
    lw          t3, 8*XLEN_BYTE(sp)
    add         a2, a2, a3
    add         a0, a0, a1
    addi        a4, a4, 2*HEVC_MAX_PB_SIZE
    li          t1, 0
    bnez        a5, 1b

    lx          s1, 0(sp)
    lx          s2, XLEN_BYTE(sp)
    lx          s3, 2*XLEN_BYTE(sp)
    lx          s4, 3*XLEN_BYTE(sp)
    lx          s5, 4*XLEN_BYTE(sp)
    lx          s6, 5*XLEN_BYTE(sp)
    lx          s7, 6*XLEN_BYTE(sp)
    lx          s8, 7*XLEN_BYTE(sp)
    addi        sp, sp, 8*XLEN_BYTE
    ret
endfunc

/* output is unclipped;*/
.macro filter_v         vdst, vsrc0, vsrc1, vsrc2, vsrc3, vsrc4, vsrc5, vsrc6, vsrc7
    vwmulu.vx        \vdst, \vsrc1, s2
    vwmaccsu.vx      \vdst, s1, \vsrc0
    vmv.v.v          \vsrc0, \vsrc1
    vwmaccsu.vx      \vdst, s3, \vsrc2
    vmv.v.v          \vsrc1, \vsrc2
    vwmaccsu.vx      \vdst, s4, \vsrc3
    vmv.v.v          \vsrc2, \vsrc3
    vwmaccsu.vx      \vdst, s5, \vsrc4
    vmv.v.v          \vsrc3, \vsrc4
    vwmaccsu.vx      \vdst, s6, \vsrc5
    vmv.v.v          \vsrc4, \vsrc5
    vwmaccsu.vx      \vdst, s7, \vsrc6
    vmv.v.v          \vsrc5, \vsrc6
    vwmaccsu.vx      \vdst, s8, \vsrc7
    vmv.v.v          \vsrc6, \vsrc7
.endm

func ff_hevc_put_qpel_v_8_m1_rvv, zve32x, zba
    addi        sp, sp, -8*XLEN_BYTE
    sx          s1, 0(sp)
    sx          s2, XLEN_BYTE(sp)
    sx          s3, 2*XLEN_BYTE(sp)
    sx          s4, 3*XLEN_BYTE(sp)
    sx          s5, 4*XLEN_BYTE(sp)
    sx          s6, 5*XLEN_BYTE(sp)
    sx          s7, 6*XLEN_BYTE(sp)
    sx          s8, 7*XLEN_BYTE(sp)
    load_filter a5
    sh1add      t1, a2, a2
    sub         a1, a1, t1      # src - 3 * src_stride
    li          t1, 0           # offset
    mv          t4, a3

1:
    add         t2, a1, t1
    sh1add      t3, t1, a0

    vsetvli     t5, a6, e8, m1, ta, ma
    vle8.V      v16, (t2)
    add         t2, t2, a2
    vle8.V      v18, (t2)
    add         t2, t2, a2
    vle8.V      v20, (t2)
    add         t2, t2, a2
    vle8.V      v22, (t2)
    add         t2, t2, a2
    vle8.V      v24, (t2)
    add         t2, t2, a2
    vle8.V      v26, (t2)
    add         t2, t2, a2
    vle8.V      v28, (t2)
    add         t2, t2, a2

2:
    vsetvli     zero, zero, e8, m1, ta, ma
    vle8.V      v30, (t2)
    add         t2, t2, a2
    filter_v    v0, v16, v18, v20, v22, v24, v26, v28, v30
    vsetvli     zero, zero, e16, m2, ta, ma
    vse16.v     v0, (t3)
    add         t3, t3, 2*HEVC_MAX_PB_SIZE
    addi        a3, a3, -1
    bnez        a3, 2b
    add         t1, t1, t5
    sub         a6, a6, t5
    mv          a3, t4
    bnez        a6, 1b

    lx          s1, 0(sp)
    lx          s2, XLEN_BYTE(sp)
    lx          s3, 2*XLEN_BYTE(sp)
    lx          s4, 3*XLEN_BYTE(sp)
    lx          s5, 4*XLEN_BYTE(sp)
    lx          s6, 5*XLEN_BYTE(sp)
    lx          s7, 6*XLEN_BYTE(sp)
    lx          s8, 7*XLEN_BYTE(sp)
    addi        sp, sp, 8*XLEN_BYTE
    ret
endfunc

func ff_hevc_put_qpel_uni_v_8_m1_rvv, zve32x, zba
    csrwi       vxrm, 0
    addi        sp, sp, -8*XLEN_BYTE
    sx          s1, 0(sp)
    sx          s2, XLEN_BYTE(sp)
    sx          s3, 2*XLEN_BYTE(sp)
    sx          s4, 3*XLEN_BYTE(sp)
    sx          s5, 4*XLEN_BYTE(sp)
    sx          s6, 5*XLEN_BYTE(sp)
    sx          s7, 6*XLEN_BYTE(sp)
    sx          s8, 7*XLEN_BYTE(sp)
    load_filter a6
    sh1add      t1, a3, a3
    sub         a2, a2, t1      # src - 3 * src_stride
    li          t1, 0           # offset
    mv          t4, a4

1:
    add         t2, a2, t1
    add         t3, a0, t1

    vsetvli     t5, a7, e8, m1, ta, ma
    vle8.V      v16, (t2)
    add         t2, t2, a3
    vle8.V      v18, (t2)
    add         t2, t2, a3
    vle8.V      v20, (t2)
    add         t2, t2, a3
    vle8.V      v22, (t2)
    add         t2, t2, a3
    vle8.V      v24, (t2)
    add         t2, t2, a3
    vle8.V      v26, (t2)
    add         t2, t2, a3
    vle8.V      v28, (t2)
    add         t2, t2, a3

2:
    vsetvli     zero, zero, e8, m1, ta, ma
    vle8.V      v30, (t2)
    add         t2, t2, a3
    filter_v    v0, v16, v18, v20, v22, v24, v26, v28, v30
    vsetvli     zero, zero, e16, m2, ta, ma
    vmax.vx     v0, v0, zero
    vsetvli     zero, zero, e8, m1, ta, ma
    vnclipu.wi  v0, v0, 6
    vse8.v      v0, (t3)
    add         t3, t3, a1
    addi        a4, a4, -1
    bnez        a4, 2b
    add         t1, t1, t5
    sub         a7, a7, t5
    mv          a4, t4
    bnez        a7, 1b

    lx          s1, 0(sp)
    lx          s2, XLEN_BYTE(sp)
    lx          s3, 2*XLEN_BYTE(sp)
    lx          s4, 3*XLEN_BYTE(sp)
    lx          s5, 4*XLEN_BYTE(sp)
    lx          s6, 5*XLEN_BYTE(sp)
    lx          s7, 6*XLEN_BYTE(sp)
    lx          s8, 7*XLEN_BYTE(sp)
    addi        sp, sp, 8*XLEN_BYTE
    ret
endfunc

func ff_hevc_put_qpel_uni_w_v_8_m1_rvv, zve32x, zba
    csrwi       vxrm, 0
    lw          t1, XLEN_BYTE(sp)       # my
    lw          t6, 2*XLEN_BYTE(sp)     # width
    
    addi        sp, sp, -8*XLEN_BYTE
    sx          s1, 0(sp)
    sx          s2, XLEN_BYTE(sp)
    sx          s3, 2*XLEN_BYTE(sp)
    sx          s4, 3*XLEN_BYTE(sp)
    sx          s5, 4*XLEN_BYTE(sp)
    sx          s6, 5*XLEN_BYTE(sp)
    sx          s7, 6*XLEN_BYTE(sp)
    sx          s8, 7*XLEN_BYTE(sp)
    load_filter t1
    addi        a5, a5, 6       # shift
    sh1add      t1, a3, a3
    sub         a2, a2, t1      # src - 3 * src_stride
    li          t1, 0           # offset
    mv          t4, a4

1:
    add         t2, a2, t1
    add         t3, a0, t1

    vsetvli     t5, t6, e8, m1, ta, ma
    vle8.V      v16, (t2)
    add         t2, t2, a3
    vle8.V      v18, (t2)
    add         t2, t2, a3
    vle8.V      v20, (t2)
    add         t2, t2, a3
    vle8.V      v22, (t2)
    add         t2, t2, a3
    vle8.V      v24, (t2)
    add         t2, t2, a3
    vle8.V      v26, (t2)
    add         t2, t2, a3
    vle8.V      v28, (t2)
    add         t2, t2, a3

2:
    vsetvli     zero, zero, e8, m1, ta, ma
    vle8.V      v30, (t2)
    add         t2, t2, a3
    filter_v    v0, v16, v18, v20, v22, v24, v26, v28, v30
    vsetvli     zero, zero, e16, m2, ta, ma
    vwmul.vx    v8, v0, a6
    vsetvli     zero, zero, e32, m4, ta, ma
    vssra.vx    v0, v8, a5
    vsadd.vx    v0, v0, a7
    vmax.vx     v0, v0, zero
    vsetvli     zero, zero, e16, m2, ta, ma
    vnclip.wi   v0, v0, 0
    vsetvli     zero, zero, e8, m1, ta, ma
    vnclipu.wi  v0, v0, 0
    vse8.v      v0, (t3)
    add         t3, t3, a1
    addi        a4, a4, -1
    bnez        a4, 2b
    add         t1, t1, t5
    sub         t6, t6, t5
    mv          a4, t4
    bnez        t6, 1b

    lx          s1, 0(sp)
    lx          s2, XLEN_BYTE(sp)
    lx          s3, 2*XLEN_BYTE(sp)
    lx          s4, 3*XLEN_BYTE(sp)
    lx          s5, 4*XLEN_BYTE(sp)
    lx          s6, 5*XLEN_BYTE(sp)
    lx          s7, 6*XLEN_BYTE(sp)
    lx          s8, 7*XLEN_BYTE(sp)
    addi        sp, sp, 8*XLEN_BYTE
    ret
endfunc

func ff_hevc_put_qpel_bi_v_8_m1_rvv, zve32x, zba
    csrwi       vxrm, 0
    lw          t6, 0(sp)      # width
    addi        sp, sp, -8*XLEN_BYTE
    sx          s1, 0(sp)
    sx          s2, XLEN_BYTE(sp)
    sx          s3, 2*XLEN_BYTE(sp)
    sx          s4, 3*XLEN_BYTE(sp)
    sx          s5, 4*XLEN_BYTE(sp)
    sx          s6, 5*XLEN_BYTE(sp)
    sx          s7, 6*XLEN_BYTE(sp)
    sx          s8, 7*XLEN_BYTE(sp)
    load_filter a7
    sh1add      t1, a3, a3
    sub         a2, a2, t1      # src - 3 * src_stride
    li          t1, 0           # offset
    mv          t4, a5

1:
    add         t2, a2, t1
    add         t3, a0, t1
    sh1add      t0, t1, a4

    vsetvli     t5, t6, e8, m1, ta, ma
    vle8.V      v16, (t2)
    add         t2, t2, a3
    vle8.V      v18, (t2)
    add         t2, t2, a3
    vle8.V      v20, (t2)
    add         t2, t2, a3
    vle8.V      v22, (t2)
    add         t2, t2, a3
    vle8.V      v24, (t2)
    add         t2, t2, a3
    vle8.V      v26, (t2)
    add         t2, t2, a3
    vle8.V      v28, (t2)
    add         t2, t2, a3

2:
    vsetvli     zero, zero, e8, m1, ta, ma
    vle8.V      v30, (t2)
    add         t2, t2, a3
    filter_v    v0, v16, v18, v20, v22, v24, v26, v28, v30
    vsetvli     zero, zero, e16, m2, ta, ma
    vle16.v     v8, (t0)
    addi        t0, t0, 2*HEVC_MAX_PB_SIZE
    vsadd.vv    v0, v0, v8
    vmax.vx     v0, v0, zero
    vsetvli     zero, zero, e8, m1, ta, ma
    vnclipu.wi  v0, v0, 7
    vse8.v      v0, (t3)
    add         t3, t3, a1
    addi        a5, a5, -1
    bnez        a5, 2b
    add         t1, t1, t5
    sub         t6, t6, t5
    mv          a5, t4
    bnez        t6, 1b

    lx          s1, 0(sp)
    lx          s2, XLEN_BYTE(sp)
    lx          s3, 2*XLEN_BYTE(sp)
    lx          s4, 3*XLEN_BYTE(sp)
    lx          s5, 4*XLEN_BYTE(sp)
    lx          s6, 5*XLEN_BYTE(sp)
    lx          s7, 6*XLEN_BYTE(sp)
    lx          s8, 7*XLEN_BYTE(sp)
    addi        sp, sp, 8*XLEN_BYTE
    ret
endfunc

/* clobbers reg t4 */
.macro filter_v_s         vdst, vsrc0, vsrc1, vsrc2, vsrc3, vsrc4, vsrc5, vsrc6, vsrc7, vf
    vwmul.vx       \vdst, \vsrc0, s0
    vwmacc.vx      \vdst, s9, \vsrc1
    vmv.v.v        \vsrc0, \vsrc1
    vwmacc.vx      \vdst, s10, \vsrc2
    vmv.v.v        \vsrc1, \vsrc2
    vwmacc.vx      \vdst, s11, \vsrc3
    vmv.v.v        \vsrc2, \vsrc3
    lb             t4, 4(\vf)
    vwmacc.vx      \vdst, t4, \vsrc4
    lb             t4, 5(\vf)
    vmv.v.v        \vsrc3, \vsrc4
    vwmacc.vx      \vdst, t4, \vsrc5
    lb             t4, 6(\vf)
    vmv.v.v        \vsrc4, \vsrc5
    vwmacc.vx      \vdst, t4, \vsrc6
    lb             t4, 7(\vf)
    vmv.v.v        \vsrc5, \vsrc6
    vwmacc.vx      \vdst, t4, \vsrc7
    vmv.v.v        \vsrc6, \vsrc7
.endm

/* output \m as t0; clobbers t0, t1, reg not enough for all coef */
.macro load_filter2 m
    la          t0, qpel_filters
    sh3add      t0, \m, t0
    lb          s0, 0(t0)
    lb          s9, 1(t0)
    lb          s10, 2(t0)
    lb          s11, 3(t0)
    mv          \m, t0
.endm

func ff_hevc_put_qpel_hv_8_m1_rvv, zve32x, zba
    csrwi       vxrm, 2
    addi        sp, sp, -12*XLEN_BYTE
    sx          s0, 0(sp)
    sx          s1, XLEN_BYTE(sp)
    sx          s2, 2*XLEN_BYTE(sp)
    sx          s3, 3*XLEN_BYTE(sp)
    sx          s4, 4*XLEN_BYTE(sp)
    sx          s5, 5*XLEN_BYTE(sp)
    sx          s6, 6*XLEN_BYTE(sp)
    sx          s7, 7*XLEN_BYTE(sp)
    sx          s8, 8*XLEN_BYTE(sp)
    sx          s9, 9*XLEN_BYTE(sp)
    sx          s10, 10*XLEN_BYTE(sp)
    sx          s11, 11*XLEN_BYTE(sp)
    load_filter  a4
    load_filter2 a5
    sh1add      t1, a2, a2
    sub         a1, a1, t1     # src - 3 * src_stride
    mv          t0, a3
    li          t1, 0          # offset

1:
    add         t2, a1, t1
    sh1add      t3, t1, a0

    vsetvli     t6, a6, e8, m1, ta, ma
    filter_h    v4, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a2
    filter_h    v6, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a2
    filter_h    v8, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a2
    filter_h    v10, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a2
    filter_h    v12, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a2
    filter_h    v14, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a2
    filter_h    v16, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a2

2:
    vsetvli     zero, zero, e8, m1, ta, ma
    filter_h    v18, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a2

    vsetvli     zero, zero, e16, m2, ta, ma
    filter_v_s  v0, v4, v6, v8, v10, v12, v14, v16, v18, a5
    vnclip.wi   v0, v0, 6
    vse16.v     v0, (t3)
    addi        a3, a3, -1
    addi        t3, t3, 2*HEVC_MAX_PB_SIZE
    bnez        a3, 2b
    mv          a3, t0
    add         t1, t1, t6
    sub         a6, a6, t6
    bnez        a6, 1b

    lx          s0, 0(sp)
    lx          s1, XLEN_BYTE(sp)
    lx          s2, 2*XLEN_BYTE(sp)
    lx          s3, 3*XLEN_BYTE(sp)
    lx          s4, 4*XLEN_BYTE(sp)
    lx          s5, 5*XLEN_BYTE(sp)
    lx          s6, 6*XLEN_BYTE(sp)
    lx          s7, 7*XLEN_BYTE(sp)
    lx          s8, 8*XLEN_BYTE(sp)
    lx          s9, 9*XLEN_BYTE(sp)
    lx          s10, 10*XLEN_BYTE(sp)
    lx          s11, 11*XLEN_BYTE(sp)
    addi        sp, sp, 12*XLEN_BYTE
    ret
endfunc

func ff_hevc_put_qpel_uni_hv_8_m1_rvv, zve32x, zba
    csrwi       vxrm, 0
    addi        sp, sp, -12*XLEN_BYTE
    sx          s0, 0(sp)
    sx          s1, XLEN_BYTE(sp)
    sx          s2, 2*XLEN_BYTE(sp)
    sx          s3, 3*XLEN_BYTE(sp)
    sx          s4, 4*XLEN_BYTE(sp)
    sx          s5, 5*XLEN_BYTE(sp)
    sx          s6, 6*XLEN_BYTE(sp)
    sx          s7, 7*XLEN_BYTE(sp)
    sx          s8, 8*XLEN_BYTE(sp)
    sx          s9, 9*XLEN_BYTE(sp)
    sx          s10, 10*XLEN_BYTE(sp)
    sx          s11, 11*XLEN_BYTE(sp)
    load_filter  a5
    load_filter2 a6
    sh1add      t1, a3, a3
    sub         a2, a2, t1     # src - 3 * src_stride
    mv          t0, a4
    li          t1, 0          # offset

1:
    add         t2, a2, t1
    add         t3, a0, t1

    vsetvli     t6, a7, e8, m1, ta, ma
    filter_h    v4, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v6, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v8, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v10, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v12, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v14, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v16, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3

2:
    vsetvli     zero, zero, e8, m1, ta, ma
    filter_h    v18, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3

    vsetvli     zero, zero, e16, m2, ta, ma
    filter_v_s  v0, v4, v6, v8, v10, v12, v14, v16, v18, a6
    vsetvli     zero, zero, e32, m4, ta, ma
    vsra.vi     v0, v0, 6
    vmax.vx     v0, v0, zero
    vsetvli     zero, zero, e16, m2, ta, ma
    vnclipu.wi   v0, v0, 6
    vsetvli     zero, zero, e8, m1, ta, ma
    vnclipu.wi  v0, v0, 0
    vse8.v     v0, (t3)
    addi        a4, a4, -1
    add         t3, t3, a1
    bnez        a4, 2b
    mv          a4, t0
    add         t1, t1, t6
    sub         a7, a7, t6
    bnez        a7, 1b

    lx          s0, 0(sp)
    lx          s1, XLEN_BYTE(sp)
    lx          s2, 2*XLEN_BYTE(sp)
    lx          s3, 3*XLEN_BYTE(sp)
    lx          s4, 4*XLEN_BYTE(sp)
    lx          s5, 5*XLEN_BYTE(sp)
    lx          s6, 6*XLEN_BYTE(sp)
    lx          s7, 7*XLEN_BYTE(sp)
    lx          s8, 8*XLEN_BYTE(sp)
    lx          s9, 9*XLEN_BYTE(sp)
    lx          s10, 10*XLEN_BYTE(sp)
    lx          s11, 11*XLEN_BYTE(sp)
    addi        sp, sp, 12*XLEN_BYTE
    ret
endfunc

func ff_hevc_put_qpel_uni_w_hv_8_m1_rvv, zve32x, zba
    csrwi       vxrm, 0
    lx          t2, 0(sp)               # mx
    lw          t4, XLEN_BYTE(sp)       # my
    lw          t5, 2*XLEN_BYTE(sp)     # width
    addi        a5, a5, 6               # shift
    addi        sp, sp, -13*XLEN_BYTE
    sx          s0, 0(sp)
    sx          s1, XLEN_BYTE(sp)
    sx          s2, 2*XLEN_BYTE(sp)
    sx          s3, 3*XLEN_BYTE(sp)
    sx          s4, 4*XLEN_BYTE(sp)
    sx          s5, 5*XLEN_BYTE(sp)
    sx          s6, 6*XLEN_BYTE(sp)
    sx          s7, 7*XLEN_BYTE(sp)
    sx          s8, 8*XLEN_BYTE(sp)
    sx          s9, 9*XLEN_BYTE(sp)
    sx          s10, 10*XLEN_BYTE(sp)
    sx          s11, 11*XLEN_BYTE(sp)
    sx          ra, 12*XLEN_BYTE(sp)
    mv          ra, t4
    load_filter  t2
    load_filter2 ra
    sh1add      t1, a3, a3
    sub         a2, a2, t1     # src - 3 * src_stride
    mv          t0, a4
    li          t1, 0          # offset

1:
    add         t2, a2, t1
    add         t3, a0, t1

    vsetvli     t6, t5, e8, m1, ta, ma
    filter_h    v4, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v6, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v8, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v10, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v12, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v14, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v16, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3

2:
    vsetvli     zero, zero, e8, m1, ta, ma
    filter_h    v18, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3

    vsetvli     zero, zero, e16, m2, ta, ma
    filter_v_s  v0, v4, v6, v8, v10, v12, v14, v16, v18, ra
    vsetvli     zero, zero, e32, m4, ta, ma
    vsra.vi     v0, v0, 6
    vmul.vx     v0, v0, a6
    vssra.vx    v0, v0, a5
    vsadd.vx    v0, v0, a7
    vmax.vx     v0, v0, zero
    vsetvli     zero, zero, e16, m2, ta, ma
    vnclip.wi   v0, v0, 0
    vsetvli     zero, zero, e8, m1, ta, ma
    vnclipu.wi  v0, v0, 0
    vse8.v      v0, (t3)
    addi        a4, a4, -1
    add         t3, t3, a1
    bnez        a4, 2b
    mv          a4, t0
    add         t1, t1, t6
    sub         t5, t5, t6
    bnez        t5, 1b

    lx          s0, 0(sp)
    lx          s1, XLEN_BYTE(sp)
    lx          s2, 2*XLEN_BYTE(sp)
    lx          s3, 3*XLEN_BYTE(sp)
    lx          s4, 4*XLEN_BYTE(sp)
    lx          s5, 5*XLEN_BYTE(sp)
    lx          s6, 6*XLEN_BYTE(sp)
    lx          s7, 7*XLEN_BYTE(sp)
    lx          s8, 8*XLEN_BYTE(sp)
    lx          s9, 9*XLEN_BYTE(sp)
    lx          s10, 10*XLEN_BYTE(sp)
    lx          s11, 11*XLEN_BYTE(sp)
    lx          ra, 12*XLEN_BYTE(sp)
    addi        sp, sp, 13*XLEN_BYTE
    ret
endfunc

func ff_hevc_put_qpel_bi_hv_8_m1_rvv, zve32x, zba
    csrwi       vxrm, 0
    lw          t3, 0(sp)      # width
    addi        sp, sp, -12*XLEN_BYTE
    sx          s0, 0(sp)
    sx          s1, XLEN_BYTE(sp)
    sx          s2, 2*XLEN_BYTE(sp)
    sx          s3, 3*XLEN_BYTE(sp)
    sx          s4, 4*XLEN_BYTE(sp)
    sx          s5, 5*XLEN_BYTE(sp)
    sx          s6, 6*XLEN_BYTE(sp)
    sx          s7, 7*XLEN_BYTE(sp)
    sx          s8, 8*XLEN_BYTE(sp)
    sx          s9, 9*XLEN_BYTE(sp)
    sx          s10, 10*XLEN_BYTE(sp)
    sx          s11, 11*XLEN_BYTE(sp)
    load_filter  a6
    load_filter2 a7
    mv          a6, t3
    sh1add      t1, a3, a3
    sub         a2, a2, t1     # src - 3 * src_stride
    mv          t0, a5
    li          t1, 0          # offset

1:
    add         t2, a2, t1
    add         t3, a0, t1
    sh1add      t5, t1, a4

    vsetvli     t6, a6, e8, m1, ta, ma
    filter_h    v4, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v6, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v8, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v10, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v12, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v14, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3
    filter_h    v16, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3

2:
    vsetvli     zero, zero, e8, m1, ta, ma
    filter_h    v18, v24, v25, v26, v27, v28, v29, v30, v31, t2
    add         t2, t2, a3

    vsetvli     zero, zero, e16, m2, ta, ma
    vle16.V     v24, (t5)
    addi        t5, t5, 2*HEVC_MAX_PB_SIZE
    filter_v_s  v0, v4, v6, v8, v10, v12, v14, v16, v18, a7
    vsetvli     zero, zero, e32, m4, ta, ma
    vsra.vi     v0, v0, 6
    vsetvli     zero, zero, e16, m2, ta, ma
    vwadd.wv    v0, v0, v24
    vnclip.wi   v0, v0, 7
    vmax.vx     v0, v0, zero
    vsetvli     zero, zero, e8, m1, ta, ma
    vnclipu.wi  v0, v0, 0
    vse8.v     v0, (t3)
    addi        a5, a5, -1
    add         t3, t3, a1
    bnez        a5, 2b
    mv          a5, t0
    add         t1, t1, t6
    sub         a6, a6, t6
    bnez        a6, 1b

    lx          s0, 0(sp)
    lx          s1, XLEN_BYTE(sp)
    lx          s2, 2*XLEN_BYTE(sp)
    lx          s3, 3*XLEN_BYTE(sp)
    lx          s4, 4*XLEN_BYTE(sp)
    lx          s5, 5*XLEN_BYTE(sp)
    lx          s6, 6*XLEN_BYTE(sp)
    lx          s7, 7*XLEN_BYTE(sp)
    lx          s8, 8*XLEN_BYTE(sp)
    lx          s9, 9*XLEN_BYTE(sp)
    lx          s10, 10*XLEN_BYTE(sp)
    lx          s11, 11*XLEN_BYTE(sp)
    addi        sp, sp, 12*XLEN_BYTE
    ret
endfunc

func ff_hevc_put_pel_pixels_8_m1_rvv, zve32x, zba
    mv          t3, a6
    li          t1, 0       # offset
    add         t4, a1, a2
    slli        a2, a2, 1

1:
    vsetvli     t6, t3, e8, m1, ta, ma
    add         t2, a1, t1
    add         t5, t4, t1
    vle8.V      v16, (t2)
    vle8.V      v20, (t5)
    vwaddu.vx   v24, v16, zero
    vwaddu.vx   v28, v20, zero
    vsetvli     zero, zero, e16, m2, ta, ma
    vsll.vi     v24, v24, 6
    vsll.vi     v28, v28, 6
    slli        t2, t1, 1
    add         t2, a0, t2
    addi        t5, t2, 2*HEVC_MAX_PB_SIZE
    vse16.v     v24, (t2)
    vse16.v     v28, (t5)
    sub         t3, t3, t6
    add         t1, t1, t6
    bgt         t3, zero, 1b
    addi        a3, a3, -2
    mv          t3, a6
    add         a1, a1, a2
    add         t4, t4, a2
    addi        a0, a0, 4*HEVC_MAX_PB_SIZE
    li          t1, 0
    bgt         a3, zero, 1b

    ret
endfunc

func ff_hevc_put_pel_pixels_uni_8_m1_rvv, zve32x, zba
    mv          t3, a7
    li          t1, 0       # offset

1:
    vsetvli     t6, t3, e8, m1, ta, ma
    add         t2, a2, t1
    add         t4, t2, a3
    vle8.V      v16, (t2)
    vle8.V      v20, (t4)
    add         t0, a0, t1
    add         t5, t0, a1
    vse8.v     v16, (t0)
    vse8.v     v20, (t5)
    sub         t3, t3, t6
    add         t1, t1, t6
    bgt         t3, zero, 1b
    addi        a4, a4, -2
    mv          t3, a7
    slli        t2, a3, 1
    add         a2, a2, t2
    slli        t2, a1, 1
    add         a0, a0, t2
    li          t1, 0
    bgt         a4, zero, 1b

    ret
endfunc

func ff_hevc_put_pel_pixels_uni_w_8_m1_rvv, zve32x, zba
    csrwi       vxrm, 0
#if (__riscv_xlen == 32)
    lw          t5, 8(sp)       # width
#elif (__riscv_xlen == 64)
    lw          t5, 16(sp)
#endif
    mv          t3, t5
    addi        a5, a5, 6       # shift
    li          t1, 0       # offset

1:
    vsetvli     t6, t3, e8, m1, ta, ma
    add         t2, a2, t1
    add         t4, t2, a3
    vle8.V      v20, (t2)
    vle8.V      v22, (t4)
    vwaddu.vx   v24, v20, zero
    vwaddu.vx   v28, v22, zero
    vsetvli     zero, zero, e16, m2, ta, ma
    vsll.vi     v24, v24, 6
    vsll.vi     v28, v28, 6
    vwmul.vx    v0, v24, a6
    vwmul.vx    v8, v28, a6
    vsetvli     zero, zero, e32, m4, ta, ma
    vssra.vx    v0, v0, a5
    vssra.vx    v8, v8, a5
    vsadd.vx    v0, v0, a7
    vsadd.vx    v8, v8, a7
    vmax.vx     v0, v0, zero
    vmax.vx     v8, v8, zero
    vsetvli     zero, zero, e16, m2, ta, ma
    vnclip.wi   v0, v0, 0
    vnclip.wi   v8, v8, 0
    vsetvli     t6, t3, e8, m1, ta, ma
    vnclipu.wi  v0, v0, 0
    vnclipu.wi  v8, v8, 0
    add         t2, a0, t1
    add         t4, t2, a1
    vse8.v     v0, (t2)
    vse8.v     v8, (t4)
    sub         t3, t3, t6
    add         t1, t1, t6
    bgt         t3, zero, 1b
    addi        a4, a4, -2
    mv          t3, t5
    slli        t2, a3, 1
    add         a2, a2, t2
    slli        t2, a1, 1
    add         a0, a0, t2
    li          t1, 0
    bgt         a4, zero, 1b

    ret
endfunc

func ff_hevc_put_pel_pixels_bi_8_m1_rvv, zve32x, zba
    csrwi       vxrm, 0
    lw          t5, 0(sp)      # width
    mv          t3, t5
    li          t1, 0       # offset

1:
    vsetvli     t6, t3, e8, m1, ta, ma
    add         t2, a2, t1
    add         t4, t2, a3
    vle8.V      v16, (t2)
    vle8.V      v20, (t4)
    vwaddu.vx   v24, v16, zero
    vwaddu.vx   v28, v20, zero
    vsetvli     zero, zero, e16, m2, ta, ma
    vsll.vi     v24, v24, 6
    vsll.vi     v28, v28, 6
    slli        t2, t1, 1
    add         t2, a4, t2
    add         t4, t2, 2*HEVC_MAX_PB_SIZE
    vle16.v     v4, (t2)
    vle16.v     v8, (t4)
    vsadd.vv    v24, v24, v4
    vsadd.vv    v28, v28, v8
    vmax.vx     v24, v24, zero
    vmax.vx     v28, v28, zero
    vsetvli     zero, zero, e8, m1, ta, ma
    vnclipu.wi  v24, v24, 7
    vnclipu.wi  v28, v28, 7
    add         t2, a0, t1
    add         t4, t2, a1
    vse8.v      v24, (t2)
    vse8.v      v28, (t4)
    sub         t3, t3, t6
    add         t1, t1, t6
    bgt         t3, zero, 1b
    addi        a5, a5, -2
    mv          t3, t5
    slli        t2, a3, 1
    add         a2, a2, t2
    slli        t2, a1, 1
    add         a0, a0, t2
    addi        a4, a4, 4*HEVC_MAX_PB_SIZE
    li          t1, 0
    bgt         a5, zero, 1b

    ret
endfunc