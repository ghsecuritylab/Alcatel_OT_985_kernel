
#ifndef __ASM_OFFSETS_H__
#define __ASM_OFFSETS_H__

#define PT_orig_r10 0 /* offsetof(struct pt_regs, orig_r10) */
#define PT_r13 56 /* offsetof(struct pt_regs, r13) */
#define PT_r12 52 /* offsetof(struct pt_regs, r12) */
#define PT_r11 48 /* offsetof(struct pt_regs, r11) */
#define PT_r10 44 /* offsetof(struct pt_regs, r10) */
#define PT_r9 40 /* offsetof(struct pt_regs, r9) */
#define PT_acr 60 /* offsetof(struct pt_regs, acr) */
#define PT_srs 64 /* offsetof(struct pt_regs, srs) */
#define PT_mof 68 /* offsetof(struct pt_regs, mof) */
#define PT_ccs 76 /* offsetof(struct pt_regs, ccs) */
#define PT_srp 80 /* offsetof(struct pt_regs, srp) */

#define TI_task 0 /* offsetof(struct thread_info, task) */
#define TI_flags 8 /* offsetof(struct thread_info, flags) */
#define TI_preempt_count 16 /* offsetof(struct thread_info, preempt_count) */

#define THREAD_ksp 0 /* offsetof(struct thread_struct, ksp) */
#define THREAD_usp 4 /* offsetof(struct thread_struct, usp) */
#define THREAD_ccs 8 /* offsetof(struct thread_struct, ccs) */

#define TASK_pid 151 /* offsetof(struct task_struct, pid) */

#define LCLONE_VM 256 /* CLONE_VM */
#define LCLONE_UNTRACED 8388608 /* CLONE_UNTRACED */

#endif
