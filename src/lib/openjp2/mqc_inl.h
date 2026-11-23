/*
 * The copyright in this software is being made available under the 2-clauses
 * BSD License, included below. This software may be subject to other third
 * party and contributor rights, including patent rights, and no such rights
 * are granted under this license.
 *
 * Copyright (c) 2002-2014, Universite catholique de Louvain (UCL), Belgium
 * Copyright (c) 2002-2014, Professor Benoit Macq
 * Copyright (c) 2001-2003, David Janssens
 * Copyright (c) 2002-2003, Yannick Verschueren
 * Copyright (c) 2003-2007, Francois-Olivier Devaux
 * Copyright (c) 2003-2014, Antonin Descampe
 * Copyright (c) 2005, Herve Drolon, FreeImage Team
 * Copyright (c) 2008, Jerome Fimes, Communications & Systemes <jerome.fimes@c-s.fr>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS `AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OPJ_MQC_INL_H
#define OPJ_MQC_INL_H

/**
Decode a symbol using raw-decoder. Cfr p.506 TAUBMAN
@param mqc MQC handle
@return Returns the decoded symbol (0 or 1)
*/
static INLINE OPJ_UINT32 opj_mqc_raw_decode(opj_mqc_t *mqc)
{
    OPJ_UINT32 d;
    if (mqc->ct == 0) {
        /* Given opj_mqc_raw_init_dec() we know that at some point we will */
        /* have a 0xFF 0xFF artificial marker */
        if (mqc->c == 0xff) {
            if (*mqc->bp  > 0x8f) {
                mqc->c = 0xff;
                mqc->ct = 8;
            } else {
                mqc->c = *mqc->bp;
                mqc->bp ++;
                mqc->ct = 7;
            }
        } else {
            mqc->c = *mqc->bp;
            mqc->bp ++;
            mqc->ct = 8;
        }
    }
    mqc->ct--;
    d = ((OPJ_UINT32)mqc->c >> mqc->ct) & 0x01U;

    return d;
}


#define opj_mqc_bytein_macro(mqc, c, ct) \
{ \
        OPJ_UINT32 l_c;  \
        /* Given opj_mqc_init_dec() we know that at some point we will */ \
        /* have a 0xFF 0xFF artificial marker */ \
        l_c = *(mqc->bp + 1); \
        if (*mqc->bp == 0xff) { \
            if (l_c > 0x8f) { \
                c += 0xff00; \
                ct = 8; \
                mqc->end_of_byte_stream_counter ++; \
            } else { \
                mqc->bp++; \
                c += l_c << 9; \
                ct = 7; \
            } \
        } else { \
            mqc->bp++; \
            c += l_c << 8; \
            ct = 8; \
        } \
}

/* For internal use of opj_mqc_decode_macro() */
/* Input bytes before c runs out of bits, while there is room for them in */
/* c (ct <= 8). This stops at a 0xFF byte, as the byte that follows it can */
/* carry a bit into the previous ones, or be a marker: such bytes are left */
/* to opj_mqc_bytein_macro(), when ct reaches 0. So c ends up with the same */
/* value as if the bytes had been input one by one when needed. */
/* Bytes read ahead (ct >= 8) are not counted as used by the PTERM check */
/* of opj_t1_decode_cblk(). */
#define opj_mqc_bytein_ahead_macro(mqc, c, ct) \
{ \
    while (ct <= 8U && *mqc->bp != 0xff) { \
        mqc->bp++; \
        c += (OPJ_UINT32)(*mqc->bp) << (8U - ct); \
        ct += 8U; \
    } \
}

/* For internal use of opj_mqc_decode_macro() */
#define opj_mqc_renormd_macro(mqc, a, c, ct) \
{ \
    do { \
        if (ct == 0) { \
            opj_mqc_bytein_macro(mqc, c, ct); \
        } \
        a <<= 1; \
        c <<= 1; \
        ct--; \
    } while (a < 0x8000); \
}

/* For internal use of opj_mqc_decode_macro() */
/* Shift a and c left by n bits, n being the number of bits that */
/* renormalize a. */
#define opj_mqc_renormd_n_macro(mqc, a, c, ct, n) \
{ \
    if (ct < (n)) { \
        opj_mqc_bytein_ahead_macro(mqc, c, ct); \
    } \
    if (ct >= (n)) { \
        a <<= (n); \
        c <<= (n); \
        ct -= (n); \
    } else { \
        opj_mqc_renormd_macro(mqc, a, c, ct); \
    } \
}

/* Context states of the decoder, in opj_mqc_t::dec_ctxs[]: */
/* qeval in bits 16 to 31, in bits 8 to 11 the number of left shifts that */
/* renormalize qeval, and in bits 0 to 7 twice the index of the state in */
/* the state table, which puts mps in bit 1. */
/* opj_mqc_t::dec_trans[(state & 0xFF) + 0] is the next state after a MPS */
/* and opj_mqc_t::dec_trans[(state & 0xFF) + 1] the next state after a LPS */
#define opj_mqc_dec_state_qeval(state) ((state) >> 16)
#define opj_mqc_dec_state_nlps_shift(state) (((state) >> 8) & 0xFU)
#define opj_mqc_dec_state_mps(state) (((state) >> 1) & 1U)

#define opj_mqc_decode_macro(d, mqc, curctx, a, c, ct) \
{ \
    /* Implements ISO 15444-1 C.3.2 Decoding a decision (DECODE) */ \
    /* Note: alternate "J.2 - Decoding an MPS or an LPS in the */ \
    /* software-conventions decoder" has been tried, but does not bring any */ \
    /* improvement. See https://github.com/uclouvain/openjpeg/issues/921 */ \
    const OPJ_UINT32 l_state = *(curctx); \
    const OPJ_UINT32 l_qeval = opj_mqc_dec_state_qeval(l_state); \
    a -= l_qeval; \
    /* (c >> 16) < qeval */ \
    if (c < (l_state & 0xFFFF0000U)) { \
        /* LPS exchange */ \
        const OPJ_UINT32 l_lps = (a >= l_qeval) ? 1U : 0U; \
        const OPJ_UINT32 l_n = opj_mqc_dec_state_nlps_shift(l_state); \
        d = opj_mqc_dec_state_mps(l_state) ^ l_lps; \
        *(curctx) = (mqc)->dec_trans[(l_state & 0xFFU) + l_lps]; \
        a = l_qeval; \
        opj_mqc_renormd_n_macro(mqc, a, c, ct, l_n); \
    } else { \
        c -= l_state & 0xFFFF0000U; \
        if ((a & 0x8000) == 0) { \
            /* MPS exchange */ \
            const OPJ_UINT32 l_lps = (a < l_qeval) ? 1U : 0U; \
            d = opj_mqc_dec_state_mps(l_state) ^ l_lps; \
            *(curctx) = (mqc)->dec_trans[(l_state & 0xFFU) + l_lps]; \
            opj_mqc_renormd_macro(mqc, a, c, ct); \
        } else { \
            d = opj_mqc_dec_state_mps(l_state); \
        } \
    } \
}

/* Same as opj_mqc_decode_macro(), but the MPS / LPS decision and the */
/* conditional exchange are done with masks instead of branches. Faster for */
/* the contexts whose decisions are hard to predict (sign, refinement, */
/* significance propagation), slower for the very skewed ones. */
#define opj_mqc_decode_nobranch_macro(d, mqc, curctx, a, c, ct) \
{ \
    const OPJ_UINT32 l_state = *(curctx); \
    const OPJ_UINT32 l_qeval = opj_mqc_dec_state_qeval(l_state); \
    /* all ones on the LPS path, (c >> 16) < qeval */ \
    const OPJ_UINT32 l_lps_mask = 0U - ((c < (l_state & 0xFFFF0000U)) ? 1U : 0U); \
    OPJ_UINT32 l_lps, l_n, l_next, l_renorm_mask; \
    a -= l_qeval; \
    /* 1 if the decoded symbol is the LPS */ \
    l_lps = (l_lps_mask & 1U) ^ ((a < l_qeval) ? 1U : 0U); \
    d = opj_mqc_dec_state_mps(l_state) ^ l_lps; \
    /* number of bits to renormalize: on the MPS path, 0 if a >= 0x8000, */ \
    /* else 1 or 2 as a >= 0x8000 - 0x5601 */ \
    l_n = (opj_mqc_dec_state_nlps_shift(l_state) & l_lps_mask) | \
          ((((a < 0x8000U) ? 1U : 0U) + ((a < 0x4000U) ? 1U : 0U)) & ~l_lps_mask); \
    /* the state changes when there is a renormalization */ \
    l_next = (mqc)->dec_trans[(l_state & 0xFFU) + l_lps]; \
    l_renorm_mask = 0U - ((l_n != 0U) ? 1U : 0U); \
    *(curctx) = l_state ^ ((l_state ^ l_next) & l_renorm_mask); \
    c -= (l_state & 0xFFFF0000U) & ~l_lps_mask; \
    a ^= (a ^ l_qeval) & l_lps_mask; \
    opj_mqc_renormd_n_macro(mqc, a, c, ct, l_n); \
}

#define DOWNLOAD_MQC_DEC_VARIABLES(mqc, curctx, a, c, ct) \
        register OPJ_UINT32 *curctx = mqc->dec_curctx; \
        register OPJ_UINT32 c = mqc->c; \
        register OPJ_UINT32 a = mqc->a; \
        register OPJ_UINT32 ct = mqc->ct

#define UPLOAD_MQC_DEC_VARIABLES(mqc, curctx, a, c, ct) \
        mqc->dec_curctx = curctx; \
        mqc->c = c; \
        mqc->a = a; \
        mqc->ct = ct;

#define DOWNLOAD_MQC_VARIABLES(mqc, curctx, a, c, ct) \
        register const opj_mqc_state_t **curctx = mqc->curctx; \
        register OPJ_UINT32 c = mqc->c; \
        register OPJ_UINT32 a = mqc->a; \
        register OPJ_UINT32 ct = mqc->ct

#define UPLOAD_MQC_VARIABLES(mqc, curctx, a, c, ct) \
        mqc->curctx = curctx; \
        mqc->c = c; \
        mqc->a = a; \
        mqc->ct = ct;

/**
Input a byte
@param mqc MQC handle
*/
static INLINE void opj_mqc_bytein(opj_mqc_t *const mqc)
{
    opj_mqc_bytein_macro(mqc, mqc->c, mqc->ct);
}

/**
Renormalize mqc->a and mqc->c while decoding
@param mqc MQC handle
*/
#define opj_mqc_renormd(mqc) \
    opj_mqc_renormd_macro(mqc, mqc->a, mqc->c, mqc->ct)

/**
Decode a symbol
@param d OPJ_UINT32 value where to store the decoded symbol
@param mqc MQC handle
@return Returns the decoded symbol (0 or 1) in d
*/
#define opj_mqc_decode(d, mqc) \
    opj_mqc_decode_macro(d, mqc, mqc->dec_curctx, mqc->a, mqc->c, mqc->ct)

/**
Output a byte, doing bit-stuffing if necessary.
After a 0xff byte, the next byte must be smaller than 0x90.
@param mqc MQC handle
*/
void opj_mqc_byteout(opj_mqc_t *mqc);

/**
Renormalize mqc->a and mqc->c while encoding, so that mqc->a stays between 0x8000 and 0x10000
@param mqc MQC handle
@param a_ value of mqc->a
@param c_ value of mqc->c_
@param ct_ value of mqc->ct_
*/
#define opj_mqc_renorme_macro(mqc, a_, c_, ct_) \
{ \
    do { \
        a_ <<= 1; \
        c_ <<= 1; \
        ct_--; \
        if (ct_ == 0) { \
            mqc->c = c_; \
            opj_mqc_byteout(mqc); \
            c_ = mqc->c; \
            ct_ = mqc->ct; \
        } \
    } while( (a_ & 0x8000) == 0); \
}

#define opj_mqc_codemps_macro(mqc, curctx, a, c, ct) \
{ \
    a -= (*curctx)->qeval; \
    if ((a & 0x8000) == 0) { \
        if (a < (*curctx)->qeval) { \
            a = (*curctx)->qeval; \
        } else { \
            c += (*curctx)->qeval; \
        } \
        *curctx = (*curctx)->nmps; \
        opj_mqc_renorme_macro(mqc, a, c, ct); \
    } else { \
        c += (*curctx)->qeval; \
    } \
}

#define opj_mqc_codelps_macro(mqc, curctx, a, c, ct) \
{ \
    a -= (*curctx)->qeval; \
    if (a < (*curctx)->qeval) { \
        c += (*curctx)->qeval; \
    } else { \
        a = (*curctx)->qeval; \
    } \
    *curctx = (*curctx)->nlps; \
    opj_mqc_renorme_macro(mqc, a, c, ct); \
}

#define opj_mqc_encode_macro(mqc, curctx, a, c, ct, d) \
{ \
    if ((*curctx)->mps == (d)) { \
        opj_mqc_codemps_macro(mqc, curctx, a, c, ct); \
    } else { \
        opj_mqc_codelps_macro(mqc, curctx, a, c, ct); \
    } \
}


#define opj_mqc_bypass_enc_macro(mqc, c, ct, d) \
{\
    if (ct == BYPASS_CT_INIT) {\
        ct = 8;\
    }\
    ct--;\
    c = c + ((d) << ct);\
    if (ct == 0) {\
        *mqc->bp = (OPJ_BYTE)c;\
        ct = 8;\
        /* If the previous byte was 0xff, make sure that the next msb is 0 */ \
        if (*mqc->bp == 0xff) {\
            ct = 7;\
        }\
        mqc->bp++;\
        c = 0;\
    }\
}

#endif /* OPJ_MQC_INL_H */
