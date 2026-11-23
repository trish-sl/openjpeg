; Copyright (c) 2026, the OpenJPEG contributors
; All rights reserved.
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
; 1. Redistributions of source code must retain the above copyright
;    notice, this list of conditions and the following disclaimer.
; 2. Redistributions in binary form must reproduce the above copyright
;    notice, this list of conditions and the following disclaimer in the
;    documentation and/or other materials provided with the distribution.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS `AS IS'
; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
; ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
; POSSIBILITY OF SUCH DAMAGE.
;
; Tier-1 decoding passes of 64x64 code-blocks of code-block style 0 for
; x86-64 with BMI1 and BMI2, Windows x64 calling convention (MASM, ml64).
; Bit exact with the C code of t1.c: see opj_t1_dec_refpass_mqc_64x64()
; and the MQ decoder macros of mqc_inl.h, that they implement.
; The caller (t1.c) checks with cpuid that the CPU has BMI1 and BMI2.
;
; The state is passed in an opj_t1_dec_asm_t structure (t1.c), so that
; this code does not depend on the layout of opj_t1_t and opj_mqc_t.

; opj_t1_dec_asm_t
ST_DATA         equ 0       ; OPJ_INT32 *: the code-block coefficients
ST_FLAGSP       equ 8       ; opj_flag_t *: &flags[66 + 1]
ST_CTXS         equ 16      ; OPJ_UINT32 *: the packed context states
ST_TRANS        equ 24      ; const OPJ_UINT32 *: the state transitions
ST_BP           equ 32      ; OPJ_BYTE *
ST_A            equ 40      ; OPJ_UINT32
ST_C            equ 44      ; OPJ_UINT32
ST_CT           equ 48      ; OPJ_UINT32
ST_EOBS         equ 52      ; OPJ_UINT32 end_of_byte_stream_counter
ST_HALF         equ 56      ; OPJ_INT32: poshalf (refinement) or oneplushalf (significance)
ST_LUTZC        equ 64      ; const OPJ_BYTE *: lut_ctxno_zc of the band
ST_LUTSC        equ 72      ; const OPJ_BYTE *: lut_ctxno_sc
ST_LUTSPB       equ 80      ; const OPJ_BYTE *: lut_spb

; flags (t1.h)
T1_SIGMA_THIS   equ 10h
T1_SIGMA_NEIGH  equ 1EFh
T1_MU_THIS      equ 100000h
T1_PI_THIS      equ 200000h
CTX_MAG         equ 14

; stack frame, after the pushes
L_STATE         equ 0       ; opj_t1_dec_asm_t *
L_TRANS         equ 8
L_HALF          equ 16
L_COLEND        equ 24      ; end of the flags of the current stripe
L_STRIPES       equ 32      ; stripes left
L_LUTZC         equ 40
L_LUTSPB        equ 48
L_SPB           equ 56
FRAME_SIZE      equ 72

; registers
;   r8d a, r9d c, r10d ct, r11 bp, rsi flagsp, rdi data, ebx flags,
;   refinement pass: r12d r13d r14d the states of the contexts CTX_MAG + 0, 1, 2,
;   significance pass: r12 the context states, r13 the current one, r14 lut_ctxno_sc,
;   rax rcx rdx rbp r15 temporaries

; Decodes a decision of the MQ decoder, as opj_mqc_decode_nobranch_macro()
; in: eax packed state
; out: ebp decision, ecx new packed state
MQ_DECODE_NOBRANCH macro
    local fast, slow_loop, nobyte, b_normal, b_stuff, ahead, ahead_done, done
    mov     edx, eax
    shr     edx, 16                     ; qeval
    mov     ebp, eax
    and     ebp, 0FFFF0000h             ; qeval << 16
    sub     r8d, edx                    ; a -= qeval
    xor     r15d, r15d
    cmp     r9d, ebp
    sbb     r15d, r15d                  ; all ones on the LPS path
    andn    ebp, r15d, ebp
    sub     r9d, ebp                    ; c -= qeval << 16 on the MPS path
    xor     ecx, ecx
    cmp     r8d, edx
    setb    cl
    mov     ebp, r15d
    and     ebp, 1
    xor     ecx, ebp                    ; 1 if the symbol is the LPS
    xor     ebp, ebp
    cmp     r8d, 8000h
    adc     ebp, 0
    cmp     r8d, 4000h
    adc     ebp, 0                      ; bits to renormalize on the MPS path
    test    r15d, r15d
    cmovnz  r8d, edx                    ; a = qeval on the LPS path
    mov     edx, eax
    shr     edx, 8
    and     edx, 0Fh
    test    r15d, r15d
    cmovz   edx, ebp                    ; edx = bits to renormalize
    mov     ebp, eax
    shr     ebp, 1
    and     ebp, 1
    xor     ebp, ecx                    ; the decision
    movzx   r15d, al
    add     r15d, ecx
    mov     rcx, qword ptr [rsp + L_TRANS]
    mov     ecx, dword ptr [rcx + r15 * 4]
    test    edx, edx
    cmovz   ecx, eax                    ; the state changes when renormalizing
    cmp     r10d, edx
    jae     fast
    ; opj_mqc_bytein_ahead_macro()
ahead:
    cmp     r10d, 8
    ja      ahead_done
    cmp     byte ptr [r11], 0FFh
    je      ahead_done
    inc     r11
    movzx   eax, byte ptr [r11]
    mov     r15d, 8
    sub     r15d, r10d
    shlx    eax, eax, r15d
    add     r9d, eax
    add     r10d, 8
    jmp     ahead
ahead_done:
    cmp     r10d, edx
    jae     fast
    ; opj_mqc_renormd_macro()
slow_loop:
    test    r10d, r10d
    jnz     nobyte
    movzx   eax, byte ptr [r11 + 1]
    cmp     byte ptr [r11], 0FFh
    jne     b_normal
    cmp     eax, 8Fh
    jbe     b_stuff
    add     r9d, 0FF00h
    mov     r10d, 8
    mov     rax, qword ptr [rsp + L_STATE]
    inc     dword ptr [rax + ST_EOBS]
    jmp     nobyte
b_stuff:
    inc     r11
    shl     eax, 9
    add     r9d, eax
    mov     r10d, 7
    jmp     nobyte
b_normal:
    inc     r11
    shl     eax, 8
    add     r9d, eax
    mov     r10d, 8
nobyte:
    add     r8d, r8d
    add     r9d, r9d
    dec     r10d
    cmp     r8d, 8000h
    jb      slow_loop
    jmp     done
fast:
    shlx    r8d, r8d, edx
    shlx    r9d, r9d, edx
    sub     r10d, edx
done:
endm

; One sample of the refinement pass, as opj_t1_dec_refpass_step_mqc_macro()
REFPASS_STEP macro ci
    local skip
    mov     eax, ebx
    and     eax, (T1_SIGMA_THIS or T1_PI_THIS) shl (3 * ci)
    cmp     eax, T1_SIGMA_THIS shl (3 * ci)
    jne     skip
    ; context: opj_t1_getctxno_mag()
    mov     eax, r12d
    test    ebx, T1_SIGMA_NEIGH shl (3 * ci)
    cmovnz  eax, r13d
    test    ebx, T1_MU_THIS shl (3 * ci)
    cmovnz  eax, r14d
    MQ_DECODE_NOBRANCH
    ; new state of the context
    mov     eax, r12d
    mov     edx, r13d
    test    ebx, T1_SIGMA_NEIGH shl (3 * ci)
    cmovz   eax, ecx
    cmovnz  edx, ecx
    test    ebx, T1_MU_THIS shl (3 * ci)
    cmovz   r12d, eax
    cmovz   r13d, edx
    cmovnz  r14d, ecx
    ; data += (v ^ (data < 0)) ? poshalf : -poshalf
    mov     eax, dword ptr [rdi + 256 * ci]
    mov     r15d, eax
    shr     r15d, 31
    xor     r15d, ebp
    dec     r15d
    mov     edx, dword ptr [rsp + L_HALF]
    xor     edx, r15d
    sub     edx, r15d
    add     eax, edx
    mov     dword ptr [rdi + 256 * ci], eax
    or      ebx, T1_MU_THIS shl (3 * ci)
skip:
endm

.code

; void opj_t1_dec_refpass_64x64_x64(opj_t1_dec_asm_t *st)
opj_t1_dec_refpass_64x64_x64 proc frame
    push    rbx
    .pushreg rbx
    push    rbp
    .pushreg rbp
    push    rsi
    .pushreg rsi
    push    rdi
    .pushreg rdi
    push    r12
    .pushreg r12
    push    r13
    .pushreg r13
    push    r14
    .pushreg r14
    push    r15
    .pushreg r15
    sub     rsp, FRAME_SIZE
    .allocstack FRAME_SIZE
    .endprolog

    mov     qword ptr [rsp + L_STATE], rcx
    mov     rax, qword ptr [rcx + ST_TRANS]
    mov     qword ptr [rsp + L_TRANS], rax
    mov     eax, dword ptr [rcx + ST_HALF]
    mov     dword ptr [rsp + L_HALF], eax
    mov     rdi, qword ptr [rcx + ST_DATA]
    mov     rsi, qword ptr [rcx + ST_FLAGSP]
    mov     r11, qword ptr [rcx + ST_BP]
    mov     r8d, dword ptr [rcx + ST_A]
    mov     r9d, dword ptr [rcx + ST_C]
    mov     r10d, dword ptr [rcx + ST_CT]
    mov     rax, qword ptr [rcx + ST_CTXS]
    mov     r12d, dword ptr [rax + 4 * CTX_MAG]
    mov     r13d, dword ptr [rax + 4 * CTX_MAG + 4]
    mov     r14d, dword ptr [rax + 4 * CTX_MAG + 8]
    lea     rax, [rsi + 4 * 64]
    mov     qword ptr [rsp + L_COLEND], rax
    mov     dword ptr [rsp + L_STRIPES], 16

refpass_col:
    mov     ebx, dword ptr [rsi]
    test    ebx, ebx
    jz      refpass_next
    REFPASS_STEP 0
    REFPASS_STEP 1
    REFPASS_STEP 2
    REFPASS_STEP 3
    mov     dword ptr [rsi], ebx
refpass_next:
    add     rsi, 4
    add     rdi, 4
    cmp     rsi, qword ptr [rsp + L_COLEND]
    jb      refpass_col
    ; next stripe: 2 border flags, 3 more rows of data
    add     rsi, 8
    add     rdi, 3 * 64 * 4
    add     qword ptr [rsp + L_COLEND], 4 * 66
    dec     dword ptr [rsp + L_STRIPES]
    jnz     refpass_col

    mov     rcx, qword ptr [rsp + L_STATE]
    mov     rax, qword ptr [rcx + ST_CTXS]
    mov     dword ptr [rax + 4 * CTX_MAG], r12d
    mov     dword ptr [rax + 4 * CTX_MAG + 4], r13d
    mov     dword ptr [rax + 4 * CTX_MAG + 8], r14d
    mov     qword ptr [rcx + ST_BP], r11
    mov     dword ptr [rcx + ST_A], r8d
    mov     dword ptr [rcx + ST_C], r9d
    mov     dword ptr [rcx + ST_CT], r10d

    add     rsp, FRAME_SIZE
    pop     r15
    pop     r14
    pop     r13
    pop     r12
    pop     rdi
    pop     rsi
    pop     rbp
    pop     rbx
    ret
opj_t1_dec_refpass_64x64_x64 endp

; One sample of the significance pass, as opj_t1_dec_sigpass_step_mqc_macro()
; without vertically causal context
SIGPASS_STEP macro ci
    local skip, done_sig
    test    ebx, (T1_SIGMA_THIS or T1_PI_THIS) shl (3 * ci)
    jnz     skip
    test    ebx, T1_SIGMA_NEIGH shl (3 * ci)
    jz      skip
    ; zero coding context: opj_t1_getctxno_zc()
    mov     eax, ebx
if ci ne 0
    shr     eax, 3 * ci
endif
    and     eax, T1_SIGMA_NEIGH
    mov     rdx, qword ptr [rsp + L_LUTZC]
    movzx   eax, byte ptr [rdx + rax]
    lea     r13, [r12 + rax * 4]
    mov     eax, dword ptr [r13]
    MQ_DECODE_NOBRANCH
    mov     dword ptr [r13], ecx
    test    ebp, ebp
    jz      done_sig
    ; sign coding: opj_t1_getctxtno_sc_or_spb_index()
    mov     eax, ebx
if ci ne 0
    shr     eax, 3 * ci
endif
    and     eax, 0AAh
    mov     edx, dword ptr [rsi - 4]
    shr     edx, 19 + 3 * ci
    and     edx, 1
    or      eax, edx
    mov     edx, dword ptr [rsi + 4]
    shr     edx, 17 + 3 * ci
    and     edx, 4
    or      eax, edx
    mov     edx, ebx
if ci eq 0
    shr     edx, 14
else
    shr     edx, 15 + (ci - 1) * 3
endif
    and     edx, 16
    or      eax, edx
    mov     edx, ebx
    shr     edx, 16 + 3 * ci
    and     edx, 64
    or      eax, edx
    movzx   ecx, byte ptr [r14 + rax]
    lea     r13, [r12 + rcx * 4]
    mov     rdx, qword ptr [rsp + L_LUTSPB]
    movzx   eax, byte ptr [rdx + rax]
    mov     dword ptr [rsp + L_SPB], eax
    mov     eax, dword ptr [r13]
    MQ_DECODE_NOBRANCH
    mov     dword ptr [r13], ecx
    xor     ebp, dword ptr [rsp + L_SPB]
    ; data = v ? -oneplushalf : oneplushalf
    mov     eax, dword ptr [rsp + L_HALF]
    mov     edx, eax
    neg     edx
    test    ebp, ebp
    cmovnz  eax, edx
    mov     dword ptr [rdi + 256 * ci], eax
    ; opj_t1_update_flags_macro()
    or      dword ptr [rsi - 4], 20h shl (3 * ci)
    or      dword ptr [rsi + 4], 8h shl (3 * ci)
    mov     eax, ebp
    shl     eax, 19 + 3 * ci
    or      eax, T1_SIGMA_THIS shl (3 * ci)
    or      ebx, eax
if ci eq 0
    mov     eax, ebp
    shl     eax, 31
    or      eax, 10000h
    or      dword ptr [rsi - 4 * 66], eax
    or      dword ptr [rsi - 4 * 66 - 4], 20000h
    or      dword ptr [rsi - 4 * 66 + 4], 8000h
endif
if ci eq 3
    mov     eax, ebp
    shl     eax, 18
    or      eax, 2
    or      dword ptr [rsi + 4 * 66], eax
    or      dword ptr [rsi + 4 * 66 - 4], 4
    or      dword ptr [rsi + 4 * 66 + 4], 1
endif
done_sig:
    or      ebx, T1_PI_THIS shl (3 * ci)
skip:
endm

; void opj_t1_dec_sigpass_64x64_x64(opj_t1_dec_asm_t *st)
opj_t1_dec_sigpass_64x64_x64 proc frame
    push    rbx
    .pushreg rbx
    push    rbp
    .pushreg rbp
    push    rsi
    .pushreg rsi
    push    rdi
    .pushreg rdi
    push    r12
    .pushreg r12
    push    r13
    .pushreg r13
    push    r14
    .pushreg r14
    push    r15
    .pushreg r15
    sub     rsp, FRAME_SIZE
    .allocstack FRAME_SIZE
    .endprolog

    mov     qword ptr [rsp + L_STATE], rcx
    mov     rax, qword ptr [rcx + ST_TRANS]
    mov     qword ptr [rsp + L_TRANS], rax
    mov     eax, dword ptr [rcx + ST_HALF]
    mov     dword ptr [rsp + L_HALF], eax
    mov     rax, qword ptr [rcx + ST_LUTZC]
    mov     qword ptr [rsp + L_LUTZC], rax
    mov     rax, qword ptr [rcx + ST_LUTSPB]
    mov     qword ptr [rsp + L_LUTSPB], rax
    mov     r14, qword ptr [rcx + ST_LUTSC]
    mov     r12, qword ptr [rcx + ST_CTXS]
    mov     rdi, qword ptr [rcx + ST_DATA]
    mov     rsi, qword ptr [rcx + ST_FLAGSP]
    mov     r11, qword ptr [rcx + ST_BP]
    mov     r8d, dword ptr [rcx + ST_A]
    mov     r9d, dword ptr [rcx + ST_C]
    mov     r10d, dword ptr [rcx + ST_CT]
    lea     rax, [rsi + 4 * 64]
    mov     qword ptr [rsp + L_COLEND], rax
    mov     dword ptr [rsp + L_STRIPES], 16

sigpass_col:
    mov     ebx, dword ptr [rsi]
    test    ebx, ebx
    jz      sigpass_next
    SIGPASS_STEP 0
    SIGPASS_STEP 1
    SIGPASS_STEP 2
    SIGPASS_STEP 3
    mov     dword ptr [rsi], ebx
sigpass_next:
    add     rsi, 4
    add     rdi, 4
    cmp     rsi, qword ptr [rsp + L_COLEND]
    jb      sigpass_col
    add     rsi, 8
    add     rdi, 3 * 64 * 4
    add     qword ptr [rsp + L_COLEND], 4 * 66
    dec     dword ptr [rsp + L_STRIPES]
    jnz     sigpass_col

    mov     rcx, qword ptr [rsp + L_STATE]
    mov     qword ptr [rcx + ST_BP], r11
    mov     dword ptr [rcx + ST_A], r8d
    mov     dword ptr [rcx + ST_C], r9d
    mov     dword ptr [rcx + ST_CT], r10d

    add     rsp, FRAME_SIZE
    pop     r15
    pop     r14
    pop     r13
    pop     r12
    pop     rdi
    pop     rsi
    pop     rbp
    pop     rbx
    ret
opj_t1_dec_sigpass_64x64_x64 endp

end
