/*******************************************************************************
**+--------------------------------------------------------------------------+**
**|                            ****                                          |**
**|                            ****                                          |**
**|                            ******o***                                    |**
**|                      ********_///_****                                   |**
**|                      ***** /_//_/ ****                                   |**
**|                       ** ** (__/ ****                                    |**
**|                           *********                                      |**
**|                            ****                                          |**
**|                            ***                                           |**
**|                                                                          |**
**|         Copyright (c) 2007-2012 Texas Instruments Incorporated           |**
**|                        ALL RIGHTS RESERVED                               |**
**|                                                                          |**
**| Permission to use, copy, modify, or distribute this software,            |**
**| whether in part or in whole, for any purpose is forbidden without        |**
**| a signed licensing agreement and NDA from Texas Instruments              |**
**| Incorporated (TI).                                                       |**
**|                                                                          |**
**| TI makes no representation or warranties with respect to the             |**
**| performance of this computer program, and specifically disclaims         |**
**| any responsibility for any damages, special or consequential,            |**
**| connected with the use of this program.                                  |**
**|                                                                          |**
**+--------------------------------------------------------------------------+**
*******************************************************************************/

#include "VLIB_utility.h"

CODE_SECTION(VLIB_divS32, ".text:optimized")
CODE_SECTION(VLIB_divS16,  ".text:optimized")
CODE_SECTION(VLIB_divU16,  ".text:optimized")
CODE_SECTION(VLIB_matrix_inverse_4x4_F32,  ".text:optimized")
CODE_SECTION(VLIB_isqrt,  ".text:optimized")
CODE_SECTION(VLIB_matrixInverse_4x4,  ".text:optimized")
CODE_SECTION(VLIB_cORBFeatures_insertionsort_x,  ".text:optimized")
CODE_SECTION(VLIB_orb_make_offsets_9,  ".text:optimized")
CODE_SECTION(VLIB_orb_make_offsets_12,  ".text:optimized")
CODE_SECTION(VLIB_compareShort,  ".text:optimized")

/* Division Utilities:
*
*    VLIB_divS32
*
*/

#if MISRA

int32_t VLIB_divS32(int32_t num, int32_t den)
{
    int32_t    k0, shift;
    int32_t    status=0;

    num = _abs(num);
    den = _abs(den);
    if( num == 0 ) {
        status =  0;
    } else if( den == 0 ) {
        status =  -1;
    } else {
        int32_t     temp = (int32_t)(_lmbd(1U, (uint32_t)num));
        int32_t     temp0 = (int32_t)(_lmbd(1U, (uint32_t)den));
        uint32_t    shift_var;
        shift = (temp0 - temp);

        den = (int32_t)((uint32_t)den << shift); /* align denominator */

        for( k0=0; k0 <= shift; k0++ ) {
            num = (int32_t)_subc((uint32_t)num, (uint32_t)den);
        }

        shift_var = (uint32_t)((uint32_t)32 - ((uint32_t)shift + 1U));
        num = (int32_t)_extu((uint32_t)num, shift_var, shift_var);
        status = num;
    }
    return (status);
}

#else

int32_t VLIB_divS32 (int32_t num, int32_t den)
{
    int32_t    i, shift;

    num = _abs(num);
    den = _abs(den);
    if( num == 0 ) {
        return (0);
    }
    if( den == 0 ) {
        return (-1);
    }

    shift = _lmbd(1, den) - _lmbd(1, num);

    den <<= shift; /* align denominator */

    for( i=0; i <= shift; i++ ) {
        num = _subc(num, den);
    }

    num = _extu(num, (32 - (shift + 1)), ((32 - (shift + 1))));

    return (num);
}

#endif

/* Division Utilities:
*
*    VLIB_divS16
*
*/
int16_t VLIB_divS16 (int16_t num, int16_t den)
{
    int32_t    i_s16, shift_s16;
    int16_t    status_s16=0;

    num = (int16_t)_abs((int32_t)num);
    den = (int16_t)_abs((int32_t)den);

    if( num == 0 ) {
        status_s16 =  0;
    } else if( den == 0 ) {
        status_s16 =  -1;
    } else {
        int32_t     temp_s16 = (int32_t)(_lmbd(1U, (uint32_t)num));
        int32_t     temp0_s16 = (int32_t)(_lmbd(1U, (uint32_t)den));
        uint32_t    den_tmp, shift_var_s16;
        shift_s16 = (temp0_s16 - temp_s16);

        den_tmp = ((uint32_t)den << shift_s16); /* align denominator */
        den = (int16_t)den_tmp;

        for( i_s16=0; i_s16 <= shift_s16; i_s16++ ) {
            num = (int16_t)_subc((uint32_t)num, (uint32_t)den);
        }

        shift_var_s16 = (uint32_t)((uint32_t)32 - ((uint32_t)shift_s16 + 1U));
        num = (int16_t)_extu((uint32_t)num, shift_var_s16, shift_var_s16);
        status_s16 = num;
    }
    return (status_s16);
}

/* Division Utilities:
*
*    VLIB_divU16
*
*/
uint16_t VLIB_divU16 (uint16_t num, uint16_t den)
{
    uint16_t    i_u16, shift_u16;
    uint32_t    inum = num;

    uint16_t    temp_u16 = (uint16_t)(_lmbd(1U, inum));
    uint16_t    temp0_u16 = (uint16_t)(_lmbd(1U, (uint32_t)den));
    uint32_t    shift_var_u16;

    shift_u16 = (temp0_u16 - temp_u16);

    den <<= shift_u16; /* align denominator */

    for( i_u16=0U; i_u16 <= shift_u16; i_u16++ ) {
        inum = _subc(inum, (uint32_t)den);
    }

    shift_var_u16 = (uint32_t)((uint32_t)32 - ((uint32_t)shift_u16 + 1U));
    num = (uint16_t)_extu(inum, shift_var_u16, shift_var_u16);

    return (num);
}

int32_t VLIB_isqrt (int32_t x)
{
    uint32_t    guess = 1U;
    uint32_t    oldguess = 0U;
    int32_t     numGuess = 0;
    int32_t     ret_val = (int32_t)guess;

    while( guess != oldguess ) {
        oldguess = guess;
        guess = (guess + ((uint32_t)x / guess)) >> 1U;
        numGuess++;

        ret_val =  (int32_t)guess;
        if( numGuess > 100 ) {
            ret_val =  (int32_t)guess;
            break;
        }
    }

    return (ret_val);
}

void VLIB_matrixInverse_4x4(int16_t *output, int16_t *source, int32_t *buffer, int16_t factor)
{
    int32_t    det, oodet;
    int32_t    col, index1, index2;
    int32_t   *buffer1;

    int64_t   *sourcePtr = (int64_t *)source;
    int64_t    temp_4x4, row0_dcba, row1_hgfe, row2_lkji, row3_ponm;
    int64_t    prod_jp_io, prod_ln_km, prod_bh_ag, prod_df_ce;
    int32_t    prod_lo, prod_kp, prod_jo, prod_kn, prod_dg, prod_ch, prod_bg, prod_cf,
               prod_ip, prod_lm, prod_ah, prod_de, prod_in, prod_jm, prod_af, prod_be;
    int32_t    sub_kp_lo, sub_jo_kn, sub_jp_ln, sub_ch_dg, sub_bg_cf, sub_bh_df,
               sub_ip_lm, sub_io_km, sub_ah_de, sub_ag_ce, sub_in_jm, sub_af_be;
    int32_t     tmp_hi, tmp_lo;
    int32_t     tmp0, tmp1, tmp2;
    int64_t     prod, prod0, prod1, prod2;
    int32_t     row0dcba_hi, row0dcba_lo, row0_temp;
    uint32_t    prod_jp_io_tmp, prod_bh_ag_tmp, oodet_temp;

    /* Step 1 : Compute cofactor of source
    a b c d
    e f g h
    i j k l
    m n o p
    read in each row of source*/

    row0_dcba = _mem8(sourcePtr);
    sourcePtr++;
    row1_hgfe = _mem8(sourcePtr);
    sourcePtr++;
    row2_lkji = _mem8(sourcePtr);
    sourcePtr++;
    row3_ponm = _mem8(sourcePtr);

    tmp_hi = (int32_t)_hill(row2_lkji);
    tmp_lo = (int32_t)_loll(row2_lkji);

    prod_lo = _mpyhl(tmp_hi, (int32_t)_hill(row3_ponm));

    /*  kp*/
    prod_kp = _mpylh(tmp_hi, (int32_t)_hill(row3_ponm));
    /*  io, jp*/
    prod_jp_io = _mpy2ll(tmp_lo, (int32_t)_hill(row3_ponm));

    /*   km, ln*/
    prod_ln_km = _mpy2ll(tmp_hi, (int32_t)_loll(row3_ponm));

    /*  jo*/
    prod_jo = _mpyhl(tmp_lo, (int32_t)_hill(row3_ponm));

    /*  kn*/
    prod_kn = _mpylh(tmp_hi, (int32_t)_loll(row3_ponm));

    sub_kp_lo = prod_kp - prod_lo;
    sub_jo_kn = prod_jo - prod_kn;

    tmp0 =(int32_t) _hill(prod_jp_io);
    sub_jp_ln =  tmp0 - (int32_t)_hill(prod_ln_km);

    /*  16 by 32 multiplies*/
    tmp1 = (int32_t)_loll(row1_hgfe);
    tmp2 = (int32_t)_hill(row1_hgfe);

    prod =  _mpyhill(tmp1, sub_kp_lo);
    prod0 = _mpylill(tmp2, sub_jp_ln);
    buffer[0] = (int32_t)((prod - prod0) + _mpyhill(tmp2, sub_jo_kn));

    prod1 = _mpyhill((int32_t)_loll(row0_dcba), sub_kp_lo);
    prod2 = _mpylill((int32_t)_hill(row0_dcba), sub_jp_ln);
    buffer[4] = -(int32_t)((prod1 - prod2) + _mpyhill((int32_t)_hill(row0_dcba), sub_jo_kn));

    row0dcba_hi = (int32_t)_hill(row0_dcba);
    row0dcba_lo = (int32_t)_loll(row0_dcba);
    prod_dg = _mpyhl(row0dcba_hi, tmp2);
    /*  ch*/
    prod_ch = _mpylh(row0dcba_hi, tmp2);
    /*  bh, ag*/
    prod_bh_ag = _mpy2ll(row0dcba_lo, tmp2);
    /*  df, ce*/
    prod_df_ce = _mpy2ll(row0dcba_hi, tmp1);
    /*  bg*/
    prod_bg = _mpyhl(row0dcba_lo, tmp2);
    /*   cf*/
    prod_cf = _mpylh(row0dcba_hi, tmp1);

    sub_ch_dg = prod_ch - prod_dg;
    sub_bg_cf = prod_bg - prod_cf;
    row0_temp = (int32_t)_hill(prod_bh_ag);
    sub_bh_df = (int32_t)row0_temp - (int32_t)_hill(prod_df_ce);

    prod = _mpyhill((int32_t)_loll(row3_ponm), sub_ch_dg);
    prod0 = _mpylill((int32_t)_hill(row3_ponm), sub_bh_df);

    buffer[8] =(int32_t)((prod - prod0) + _mpyhill((int32_t)_hill(row3_ponm), sub_bg_cf));

    prod1 = _mpyhill(tmp_lo, sub_ch_dg);
    prod2 = _mpylill(tmp_hi, sub_bh_df);

    buffer[12] = -(int32_t)((prod1 - prod2) + _mpyhill(tmp_hi, sub_bg_cf));

    prod_ip = _mpylh(tmp_lo, (int32_t)_hill(row3_ponm));
    /*  lm*/
    prod_lm = _mpyhl(tmp_hi, (int32_t)_loll(row3_ponm));

    sub_ip_lm = prod_ip - prod_lm;
    prod_jp_io_tmp = _loll(prod_jp_io);
    sub_io_km =  (int32_t)prod_jp_io_tmp - (int32_t)_loll(prod_ln_km);

    prod = _mpylill(tmp1, sub_kp_lo);
    prod0 = _mpylill(tmp2, sub_ip_lm);

    buffer[1] = -(int32_t)((prod   - prod0) + _mpyhill(tmp2, sub_io_km));

    prod1 = _mpylill(row0dcba_lo, sub_kp_lo);
    prod2 = _mpylill(row0dcba_hi, sub_ip_lm);

    buffer[5] =(int32_t)((prod1 - prod2) + _mpyhill(row0dcba_hi, sub_io_km));

    prod_ah =_mpylh(row0dcba_lo, tmp2);
    prod_de = _mpyhl(row0dcba_hi, tmp1);

    sub_ah_de = prod_ah - prod_de;

    prod_bh_ag_tmp = _loll(prod_bh_ag);

    sub_ag_ce = (int32_t)prod_bh_ag_tmp - (int32_t)_loll(prod_df_ce);

    prod = _mpylill((int32_t)_loll(row3_ponm), sub_ch_dg);
    prod0 = _mpylill((int32_t)_hill(row3_ponm), sub_ah_de);

    buffer[9] = -(int32_t)((prod - prod0) + _mpyhill((int32_t)_hill(row3_ponm), sub_ag_ce));

    prod1 = _mpylill(tmp_lo, sub_ch_dg);
    prod2 = _mpylill(tmp_hi, sub_ah_de);

    buffer[13] =(int32_t)((prod1 - prod2) + _mpyhill(tmp_hi, sub_ag_ce));

    prod_in = _mpylh(tmp_lo, (int32_t)_loll(row3_ponm));
    prod_jm = _mpyhl(tmp_lo, (int32_t)_loll(row3_ponm));

    sub_in_jm = prod_in - prod_jm;

    prod = _mpylill(tmp1, sub_jp_ln);
    prod0 = _mpyhill(tmp1, sub_ip_lm);

    buffer[2] = (int32_t)((prod  - prod0) + _mpyhill(tmp2, sub_in_jm));

    prod1 = _mpylill(row0dcba_lo, sub_jp_ln);
    prod2 = _mpyhill(row0dcba_lo, sub_ip_lm);

    buffer[6] = -(int32_t)((prod1 - prod2) + _mpyhill(row0dcba_hi, sub_in_jm));

    prod_af = _mpylh(row0dcba_lo, tmp1);
    prod_be = _mpyhl(row0dcba_lo, tmp1);

    sub_af_be = prod_af - prod_be;

    prod = _mpylill((int32_t)_loll(row3_ponm), sub_bh_df);
    prod0 = _mpyhill((int32_t)_loll(row3_ponm), sub_ah_de);

    buffer[10] =(int32_t)((prod - prod0) + _mpyhill((int32_t)_hill(row3_ponm), sub_af_be));

    prod1 = _mpylill(tmp_lo, sub_bh_df);
    prod2 = _mpyhill(tmp_lo, sub_ah_de);

    buffer[14] = -(int32_t)((prod1 - prod2) + _mpyhill(tmp_hi, sub_af_be));

    prod = _mpylill(tmp1, sub_jo_kn);
    prod0 = _mpyhill(tmp1, sub_io_km);

    buffer[3] = -(int32_t)((prod - prod0) + _mpylill(tmp2, sub_in_jm));

    prod1 =  _mpylill(row0dcba_lo, sub_jo_kn);
    prod2 = _mpyhill(row0dcba_lo, sub_io_km);

    buffer[7] = (int32_t)((prod1 - prod2) +  _mpylill(row0dcba_hi, sub_in_jm));

    prod = _mpylill((int32_t)_loll(row3_ponm), sub_bg_cf);
    prod0 = _mpyhill((int32_t)_loll(row3_ponm), sub_ag_ce);

    buffer[11] = -(int32_t)((prod - prod0) + _mpylill((int32_t)_hill(row3_ponm), sub_af_be));

    prod1 = _mpylill(tmp_lo, sub_bg_cf);
    prod2 = _mpyhill(tmp_lo, sub_ag_ce);

    buffer[15] =(int32_t)((prod1 - prod2) + _mpylill(tmp_hi, sub_af_be));
    /* Step 2 : Compute determinant*/
    det = 0;

    for( col = 0; col < 4; col++ ) {
        det += ((int32_t)source[col + (4 * 0)] * buffer[col + (4 * 0)]);
    }

    /*  Step 3: Compute reiprocal of determinant
    get reciprocal of determinant
    S0.31*/
    oodet = VLIB_divS32(0x7fffffff, det);

    oodet_temp = ((((uint32_t)oodet * (uint32_t)factor)) >> 15);

    /*  scale the determinant using provided scaling factor
    S0.31 * S0.15 = SS0.46 -> S0.31*/
    oodet = (int32_t)oodet_temp;


    /*  Step 4: Compute transpose of cofactor
    compute transpose of cofactor*/
    buffer1 = buffer + 16;

    for( index1 = 0; index1 < 4; index1++ ) {
        for( index2 = 0; index2 < 4; index2++ ) {
            buffer1[index2 + ((int32_t)4 * index1)] = buffer[index1 + ((int32_t)4 * index2)];
        }
    }

#ifdef VLIB_CGT6X_COMPILER
    /*  Step 5: Multiply determinannt with transposed cofactor*/
    /*WORD_ALIGNED(output);*/
    (_nassert(((int32_t)(output) & 0x3) == 0));
    /*WORD_ALIGNED(buffer1);*/
    (_nassert(((int32_t)(buffer1) & 0x3) == 0));
    /*  S31.0 * S0.31 = SS31.31 -> S0.15*/
#pragma UNROLL(16)
#endif

    for( index1 = 0; index1 < 16; index1++ ) {
        /*S31.0 * S0.31 = SS31.31*/
        temp_4x4 =((int64_t)buffer1[index1] * (int64_t)oodet);
        output[index1] = (int16_t)_packh2(0x0000ffffU, _loll(temp_4x4));

    }
}

void VLIB_matrix_inverse_4x4_F32(VLIB_F32 *temp2_F32,
                                 int16_t factor,
                                 VLIB_F32 *inv_temp)
{
    VLIB_D64    inv[16], det_F32;
    int32_t     im_F32;

    __float2_t    row_0_ba, row_0_dc, row_1_ba, row_1_dc,
                  row_2_ba, row_2_dc, row_3_ba, row_3_dc;

    VLIB_F32    t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, tA, tB, tC, tD, tE, tF;

    VLIB_F32    diff_FA_EB, diff_F6_E7, diff_B6_A7, diff_E5_D6,
                diff_F9_DB, diff_F5_D7, diff_B5_97, diff_A5_96,
                diff_B2_A3, diff_F2_E3, diff_F1_D3, diff_B1_93,
                diff_A1_92, diff_E9_DA, diff_72_63, diff_71_53,
                diff_E1_D2, diff_61_52;
    VLIB_D64    p_05, p_09, p_0D, p_44, p_48, p_4C, p_84, p_88, p_8C, p_C4, p_C8, p_CC,
                p_11, p_19, p_1D, p_50, p_58, p_5C, p_90, p_98, p_9C, p_D8, p_DC, p_D0,
                p_21, p_25, p_2D, p_60, p_64, p_6C, p_A0, p_A4, p_AC, p_E0, p_EC, p_E4,
                p_35, p_31, p_39, p_70, p_74, p_78, p_B0, p_B4, p_B8, p_F0, p_F4, p_F8;
    VLIB_F32    t_FA, t_EB, t_F6, t_E7, t_B6, t_A7, t_F9, t_DB, t_F5, t_D7, t_B5, t_97,
                t_E9, t_DA, t_E5, t_D6, t_A5, t_96, t_F2, t_E3, t_B2, t_A3, t_D3, t_F1,
                t_B1, t_93, t_D2, t_E1, t_92, t_A1, t_72, t_63, t_71, t_53, t_61, t_52;

    VLIB_F32    det_1;

    row_0_ba=_mem8_f2(temp2_F32);
    temp2_F32+=2;
    row_0_dc=_mem8_f2(temp2_F32);
    temp2_F32+=2;
    row_1_ba=_mem8_f2(temp2_F32);
    temp2_F32+=2;
    row_1_dc=_mem8_f2(temp2_F32);
    temp2_F32+=2;
    row_2_ba=_mem8_f2(temp2_F32);
    temp2_F32+=2;
    row_2_dc=_mem8_f2(temp2_F32);
    temp2_F32+=2;
    row_3_ba=_mem8_f2(temp2_F32);
    temp2_F32+=2;
    row_3_dc=_mem8_f2(temp2_F32);

    t0 = _lof2(row_0_ba);
    t2  = _lof2(row_0_dc);
    t4 = _lof2(row_1_ba);
    t6  = _lof2(row_1_dc);
    t8 = _lof2(row_2_ba);
    tA  = _lof2(row_2_dc);
    tC = _lof2(row_3_ba);
    tE  = _lof2(row_3_dc);

    t1 = _hif2(row_0_ba);
    t3  = _hif2(row_0_dc);
    t5 = _hif2(row_1_ba);
    t7  = _hif2(row_1_dc);
    t9 = _hif2(row_2_ba);
    tB  = _hif2(row_2_dc);
    tD = _hif2(row_3_ba);
    tF  = _hif2(row_3_dc);

    /*no intrinsic MPYSP inst*/
    t_FA = tF * tA;
    t_EB = tE * tB;
    t_F6 = tF * t6;
    t_E7 = tE * t7;
    t_B6 = tB * t6;
    t_A7 = tA * t7;
    t_F9 = tF * t9;
    t_DB = tD * tB;
    t_F5 = tF * t5;
    t_D7 = tD * t7;
    t_B5 = tB * t5;
    t_97 = t9 * t7;
    t_E9 = tE * t9;
    t_DA = tD * tA;
    t_E5 = tE * t5;
    t_D6 = tD * t6;
    t_A5 = tA * t5;
    t_96 = t9 * t6;
    t_F2 = tF * t2;
    t_E3 = tE * t3;
    t_B2 = tB * t2;
    t_A3 = tA * t3;
    t_D3 = tD * t3;
    t_F1 = tF * t1;
    t_B1 = tB * t1;
    t_93 = t9 * t3;
    t_D2 = tD * t2;
    t_E1 = tE * t1;
    t_92 = t9 * t2;
    t_A1 = tA * t1;
    t_63 = t6 * t3;
    t_72 = t7 * t2;
    t_71 = t7 * t1;
    t_53 = t5 * t3;
    t_61 = t6 * t1;
    t_52 = t5 * t2;

    diff_FA_EB= t_FA - t_EB;
    diff_F6_E7= t_F6 - t_E7;
    diff_B6_A7= t_B6 - t_A7;
    diff_F9_DB= t_F9 - t_DB;
    diff_F5_D7= t_F5 - t_D7;
    diff_B5_97= t_B5 - t_97;
    diff_E5_D6= t_E5 - t_D6;
    diff_A5_96= t_A5 - t_96;
    diff_F2_E3= t_F2 - t_E3;
    diff_B2_A3= t_B2 - t_A3;
    diff_F1_D3= t_F1 - t_D3;
    diff_B1_93= t_B1 - t_93;
    diff_A1_92= t_A1 - t_92;
    diff_72_63= t_72 - t_63;
    diff_71_53= t_71 - t_53;
    diff_E1_D2= t_E1 - t_D2;
    diff_61_52= t_61 - t_52;
    diff_E9_DA= t_E9 - t_DA;

    p_31 = _mpysp2dp(diff_B6_A7, t1);
    p_35 = _mpysp2dp(diff_B2_A3, t5);
    p_39 = _mpysp2dp(diff_72_63, t9);
    p_05 = _mpysp2dp(diff_FA_EB, t5);
    p_09 = _mpysp2dp(diff_F6_E7, t9);
    p_0D = _mpysp2dp(diff_B6_A7, tD);
    p_44 = _mpysp2dp(diff_FA_EB, t4);
    p_48 = _mpysp2dp(diff_F6_E7, t8);
    p_4C = _mpysp2dp(diff_B6_A7, tC);
    p_84 = _mpysp2dp(diff_F9_DB, t4);
    p_88 = _mpysp2dp(diff_F5_D7, t8);
    p_8C = _mpysp2dp(diff_B5_97, tC);
    p_C4 = _mpysp2dp(diff_E9_DA, t4);
    p_C8 = _mpysp2dp(diff_E5_D6, t8);
    p_CC = _mpysp2dp(diff_A5_96, tC);
    p_11 = _mpysp2dp(diff_FA_EB, t1);
    p_19 = _mpysp2dp(diff_F2_E3, t9);
    p_1D = _mpysp2dp(diff_B2_A3, tD);
    p_50 = _mpysp2dp(diff_FA_EB, t0);
    p_58 = _mpysp2dp(diff_F2_E3, t8);
    p_5C = _mpysp2dp(diff_B2_A3, tC);
    p_90 = _mpysp2dp(diff_F9_DB, t0);
    p_98 = _mpysp2dp(diff_F1_D3, t8);
    p_9C = _mpysp2dp(diff_B1_93, tC);
    p_D0 = _mpysp2dp(diff_E9_DA, t0);
    p_D8 = _mpysp2dp(diff_E1_D2, t8);
    p_DC = _mpysp2dp(diff_A1_92, tC);
    p_21 = _mpysp2dp(diff_F6_E7, t1);
    p_25 = _mpysp2dp(diff_F2_E3, t5);
    p_2D = _mpysp2dp(diff_72_63, tD);
    p_60 = _mpysp2dp(diff_F6_E7, t0);
    p_64 = _mpysp2dp(diff_F2_E3, t4);
    p_6C = _mpysp2dp(diff_72_63, tC);
    p_A0 = _mpysp2dp(diff_F5_D7, t0);
    p_A4 = _mpysp2dp(diff_F1_D3, t4);
    p_AC = _mpysp2dp(diff_71_53, t4);
    p_E0 = _mpysp2dp(diff_E5_D6, t0);
    p_E4 = _mpysp2dp(diff_E1_D2, t4);
    p_EC = _mpysp2dp(diff_61_52, tC);
    p_70 = _mpysp2dp(diff_B6_A7, t0);
    p_74 = _mpysp2dp(diff_B2_A3, t4);
    p_78 = _mpysp2dp(diff_72_63, t8);
    p_B0 = _mpysp2dp(diff_B5_97, t0);
    p_B4 = _mpysp2dp(diff_B1_93, t4);
    p_B8 = _mpysp2dp(diff_71_53, t8);
    p_F0 = _mpysp2dp(diff_A5_96, t0);
    p_F4 = _mpysp2dp(diff_A1_92, t4);
    p_F8 = _mpysp2dp(diff_61_52, t8);



    inv[0]  =  (p_05 - p_09)  + p_0D;
    inv[4]  =  (-p_44 + p_48) - p_4C;
    inv[8]  =  (p_84 - p_88)  + p_8C;
    inv[12] =  (-p_C4 + p_C8) - p_CC;
    inv[1]  =  (-p_11 - p_19) - p_1D;
    inv[5]  =  (p_50 - p_58)  + p_5C;
    inv[9]  =  (-p_90 + p_98) - p_9C;
    inv[13] =  (p_D0 - p_D8)  + p_DC;
    inv[2]  =  (p_21 - p_25)  + p_2D;
    inv[6]  =  (-p_60 + p_64) - p_6C;
    inv[10] =  (p_A0 - p_A4)  + p_AC;
    inv[14] =  (-p_E0 + p_E4) - p_EC;
    inv[3]  =  (-p_31 + p_35) - p_39;
    inv[7]  =  (p_70 - p_74)  + p_78;
    inv[11] =  (-p_B0 + p_B4) - p_B8;
    inv[15] =  (p_F0 - p_F4)  + p_F8;

    det_F32 =((VLIB_D64)t0 * inv[0]) + ((VLIB_D64)t1 * inv[4]) + ((VLIB_D64)t2 * inv[8]) +
              ((VLIB_D64)t3 * inv[12]);


    det_F32 = 1.0 / ((VLIB_D64)factor * det_F32);
    det_1=(VLIB_F32) det_F32;

    for( im_F32 = 0; im_F32 < 16; im_F32++ ) {
        inv_temp[im_F32] = ((VLIB_F32)inv[im_F32] * det_1);
    }
}

void VLIB_orb_make_offsets_9(int32_t pixl[], int32_t row_stride)
{
    pixl[0] = (int32_t)0 + ((int32_t)row_stride * (int32_t)3);
    pixl[1] = (int32_t)1 + ((int32_t)row_stride * (int32_t)3);
    pixl[2] = (int32_t)2 + ((int32_t)row_stride * (int32_t)2);
    pixl[3] = (int32_t)3 + ((int32_t)row_stride * (int32_t)1);
    pixl[4] = (int32_t)3 + ((int32_t)row_stride * (int32_t)0);
    pixl[5] = (int32_t)3 + ((int32_t)row_stride * (int32_t)-1);
    pixl[6] = (int32_t)2 + ((int32_t)row_stride * (int32_t)-2);
    pixl[7] = (int32_t)1 + ((int32_t)row_stride * (int32_t)-3);
    pixl[8] = (int32_t)0 + ((int32_t)row_stride * (int32_t)-3);
    pixl[9] = (int32_t)-1 + ((int32_t)row_stride * (int32_t)-3);
    pixl[10] = (int32_t)-2 + ((int32_t)row_stride * (int32_t)-2);
    pixl[11] = (int32_t)-3 + ((int32_t)row_stride * (int32_t)-1);
    pixl[12] = (int32_t)-3 + ((int32_t)row_stride * (int32_t)0);
    pixl[13] = (int32_t)-3 + ((int32_t)row_stride * (int32_t)1);
    pixl[14] = (int32_t)-2 + ((int32_t)row_stride * (int32_t)2);
    pixl[15] = (int32_t)-1 + ((int32_t)row_stride * (int32_t)3);
}

void VLIB_orb_make_offsets_12(int32_t pixel[], int32_t row_stride)
{
    pixel[0] = (int32_t)0 + ((int32_t)row_stride * (int32_t)3);
    pixel[1] = (int32_t)1 + ((int32_t)row_stride * (int32_t)3);
    pixel[2] = (int32_t)2 + ((int32_t)row_stride * (int32_t)2);
    pixel[3] = (int32_t)3 + ((int32_t)row_stride * (int32_t)1);
    pixel[4] = (int32_t)3 + ((int32_t)row_stride * (int32_t)0);
    pixel[5] = (int32_t)3 + ((int32_t)row_stride * (int32_t)-1);
    pixel[6] = (int32_t)2 + ((int32_t)row_stride * (int32_t)-2);
    pixel[7] = (int32_t)1 + ((int32_t)row_stride * (int32_t)-3);
    pixel[8] = (int32_t)0 + ((int32_t)row_stride * (int32_t)-3);
    pixel[9] = (int32_t)-1 + ((int32_t)row_stride * (int32_t)-3);
    pixel[10] = (int32_t)-2 + ((int32_t)row_stride * (int32_t)-2);
    pixel[11] = (int32_t)-3 + ((int32_t)row_stride * (int32_t)-1);
    pixel[12] = (int32_t)-3 + ((int32_t)row_stride * (int32_t)0);
    pixel[13] = (int32_t)-3 + ((int32_t)row_stride * (int32_t)1);
    pixel[14] = (int32_t)-2 + ((int32_t)row_stride * (int32_t)2);
    pixel[15] = (int32_t)-1 + ((int32_t)row_stride * (int32_t)3);
}

void VLIB_cORBFeatures_insertionsort_x(CORBFeature *features, int32_t n_features)
{
    int32_t    i;

    for( i=1; i < n_features; i++ ) {
        CORBFeature    value = features[i];

        int32_t      j = i - 1;
        CORB_Bool    done = BOOL_FALSE;

        do {
            if( features[j].x > value.x ) {
                features[j + 1] = features[j];
                j = j - 1;

                if( j < 0 ) {
                    done = BOOL_TRUE;
                }
            } else {
                done = BOOL_TRUE;
            }
        } while( !done );

        features[j + 1] = value;
    }
}

int32_t VLIB_compareShort(const void *a, const void *b)
{
    const int32_t    sa = *((const int16_t *)a);
    const int32_t    sb = *((const int16_t *)b);

    return ((int32_t)(sa - sb));
}

/* ======================================================================== */
/*  End of file:  VLIB_utility.c                                            */
/* ======================================================================== */

