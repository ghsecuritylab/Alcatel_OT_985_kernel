
#ifndef _ASM_POWERPC_PTRACE_H
#define _ASM_POWERPC_PTRACE_H


#include <linux/types.h>

#ifndef __ASSEMBLY__

struct pt_regs {
	unsigned long gpr[32];
	unsigned long nip;
	unsigned long msr;
	unsigned long orig_gpr3;	/* Used for restarting system calls */
	unsigned long ctr;
	unsigned long link;
	unsigned long xer;
	unsigned long ccr;
#ifdef __powerpc64__
	unsigned long softe;		/* Soft enabled/disabled */
#else
	unsigned long mq;		/* 601 only (not used at present) */
					/* Used on APUS to hold IPL value. */
#endif
	unsigned long trap;		/* Reason for being here */
	/* N.B. for critical exceptions on 4xx, the dar and dsisr
	   fields are overloaded to hold srr0 and srr1. */
	unsigned long dar;		/* Fault registers */
	unsigned long dsisr;		/* on 4xx/Book-E used for ESR */
	unsigned long result;		/* Result of a system call */
};

#endif /* __ASSEMBLY__ */

#ifdef __KERNEL__

#ifdef __powerpc64__

#define STACK_FRAME_OVERHEAD	112	/* size of minimum stack frame */
#define STACK_FRAME_LR_SAVE	2	/* Location of LR in stack frame */
#define STACK_FRAME_REGS_MARKER	ASM_CONST(0x7265677368657265)
#define STACK_INT_FRAME_SIZE	(sizeof(struct pt_regs) + \
					STACK_FRAME_OVERHEAD + 288)
#define STACK_FRAME_MARKER	12

/* Size of dummy stack frame allocated when calling signal handler. */
#define __SIGNAL_FRAMESIZE	128
#define __SIGNAL_FRAMESIZE32	64

#else /* __powerpc64__ */

#define STACK_FRAME_OVERHEAD	16	/* size of minimum stack frame */
#define STACK_FRAME_LR_SAVE	1	/* Location of LR in stack frame */
#define STACK_FRAME_REGS_MARKER	ASM_CONST(0x72656773)
#define STACK_INT_FRAME_SIZE	(sizeof(struct pt_regs) + STACK_FRAME_OVERHEAD)
#define STACK_FRAME_MARKER	2

/* Size of stack frame allocated when calling signal handler. */
#define __SIGNAL_FRAMESIZE	64

#endif /* __powerpc64__ */

#ifndef __ASSEMBLY__

#define instruction_pointer(regs) ((regs)->nip)
#define user_stack_pointer(regs) ((regs)->gpr[1])
#define kernel_stack_pointer(regs) ((regs)->gpr[1])
#define regs_return_value(regs) ((regs)->gpr[3])

#ifdef CONFIG_SMP
extern unsigned long profile_pc(struct pt_regs *regs);
#else
#define profile_pc(regs) instruction_pointer(regs)
#endif

#ifdef __powerpc64__
#define user_mode(regs) ((((regs)->msr) >> MSR_PR_LG) & 0x1)
#else
#define user_mode(regs) (((regs)->msr & MSR_PR) != 0)
#endif

#define force_successful_syscall_return()   \
	do { \
		set_thread_flag(TIF_NOERROR); \
	} while(0)

struct task_struct;
extern unsigned long ptrace_get_reg(struct task_struct *task, int regno);
extern int ptrace_put_reg(struct task_struct *task, int regno,
			  unsigned long data);

#define FULL_REGS(regs)		(((regs)->trap & 1) == 0)
#ifndef __powerpc64__
#define IS_CRITICAL_EXC(regs)	(((regs)->trap & 2) != 0)
#define IS_MCHECK_EXC(regs)	(((regs)->trap & 4) != 0)
#define IS_DEBUG_EXC(regs)	(((regs)->trap & 8) != 0)
#endif /* ! __powerpc64__ */
#define TRAP(regs)		((regs)->trap & ~0xF)
#ifdef __powerpc64__
#define CHECK_FULL_REGS(regs)	BUG_ON(regs->trap & 1)
#else
#define CHECK_FULL_REGS(regs)						      \
do {									      \
	if ((regs)->trap & 1)						      \
		printk(KERN_CRIT "%s: partial register set\n", __func__); \
} while (0)
#endif /* __powerpc64__ */

#define arch_has_single_step()	(1)
#define arch_has_block_step()	(!cpu_has_feature(CPU_FTR_601))
#define ARCH_HAS_USER_SINGLE_STEP_INFO


#include <linux/stddef.h>
#include <linux/thread_info.h>
extern int regs_query_register_offset(const char *name);
extern const char *regs_query_register_name(unsigned int offset);
#define MAX_REG_OFFSET (offsetof(struct pt_regs, dsisr))

static inline unsigned long regs_get_register(struct pt_regs *regs,
						unsigned int offset)
{
	if (unlikely(offset > MAX_REG_OFFSET))
		return 0;
	return *(unsigned long *)((unsigned long)regs + offset);
}


static inline bool regs_within_kernel_stack(struct pt_regs *regs,
						unsigned long addr)
{
	return ((addr & ~(THREAD_SIZE - 1))  ==
		(kernel_stack_pointer(regs) & ~(THREAD_SIZE - 1)));
}

static inline unsigned long regs_get_kernel_stack_nth(struct pt_regs *regs,
						      unsigned int n)
{
	unsigned long *addr = (unsigned long *)kernel_stack_pointer(regs);
	addr += n;
	if (regs_within_kernel_stack(regs, (unsigned long)addr))
		return *addr;
	else
		return 0;
}

#endif /* __ASSEMBLY__ */

#endif /* __KERNEL__ */

#define PT_R0	0
#define PT_R1	1
#define PT_R2	2
#define PT_R3	3
#define PT_R4	4
#define PT_R5	5
#define PT_R6	6
#define PT_R7	7
#define PT_R8	8
#define PT_R9	9
#define PT_R10	10
#define PT_R11	11
#define PT_R12	12
#define PT_R13	13
#define PT_R14	14
#define PT_R15	15
#define PT_R16	16
#define PT_R17	17
#define PT_R18	18
#define PT_R19	19
#define PT_R20	20
#define PT_R21	21
#define PT_R22	22
#define PT_R23	23
#define PT_R24	24
#define PT_R25	25
#define PT_R26	26
#define PT_R27	27
#define PT_R28	28
#define PT_R29	29
#define PT_R30	30
#define PT_R31	31

#define PT_NIP	32
#define PT_MSR	33
#define PT_ORIG_R3 34
#define PT_CTR	35
#define PT_LNK	36
#define PT_XER	37
#define PT_CCR	38
#ifndef __powerpc64__
#define PT_MQ	39
#else
#define PT_SOFTE 39
#endif
#define PT_TRAP	40
#define PT_DAR	41
#define PT_DSISR 42
#define PT_RESULT 43
#define PT_REGS_COUNT 44

#define PT_FPR0	48	/* each FP reg occupies 2 slots in this space */

#ifndef __powerpc64__

#define PT_FPR31 (PT_FPR0 + 2*31)
#define PT_FPSCR (PT_FPR0 + 2*32 + 1)

#else /* __powerpc64__ */

#define PT_FPSCR (PT_FPR0 + 32)	/* each FP reg occupies 1 slot in 64-bit space */

#ifdef __KERNEL__
#define PT_FPSCR32 (PT_FPR0 + 2*32 + 1)	/* each FP reg occupies 2 32-bit userspace slots */
#endif

#define PT_VR0 82	/* each Vector reg occupies 2 slots in 64-bit */
#define PT_VSCR (PT_VR0 + 32*2 + 1)
#define PT_VRSAVE (PT_VR0 + 33*2)

#ifdef __KERNEL__
#define PT_VR0_32 164	/* each Vector reg occupies 4 slots in 32-bit */
#define PT_VSCR_32 (PT_VR0 + 32*4 + 3)
#define PT_VRSAVE_32 (PT_VR0 + 33*4)
#endif

#define PT_VSR0 150	/* each VSR reg occupies 2 slots in 64-bit */
#define PT_VSR31 (PT_VSR0 + 2*31)
#ifdef __KERNEL__
#define PT_VSR0_32 300 	/* each VSR reg occupies 4 slots in 32-bit */
#endif
#endif /* __powerpc64__ */

#define PTRACE_GETVRREGS	18
#define PTRACE_SETVRREGS	19

#define PTRACE_GETEVRREGS	20
#define PTRACE_SETEVRREGS	21

/* Get the first 32 128bit VSX registers */
#define PTRACE_GETVSRREGS	27
#define PTRACE_SETVSRREGS	28

#define PTRACE_GET_DEBUGREG	25
#define PTRACE_SET_DEBUGREG	26

#define PTRACE_GETREGS            12
#define PTRACE_SETREGS            13
#define PTRACE_GETFPREGS          14
#define PTRACE_SETFPREGS          15
#define PTRACE_GETREGS64	  22
#define PTRACE_SETREGS64	  23

/* (old) PTRACE requests with inverted arguments */
#define PPC_PTRACE_GETREGS	0x99	/* Get GPRs 0 - 31 */
#define PPC_PTRACE_SETREGS	0x98	/* Set GPRs 0 - 31 */
#define PPC_PTRACE_GETFPREGS	0x97	/* Get FPRs 0 - 31 */
#define PPC_PTRACE_SETFPREGS	0x96	/* Set FPRs 0 - 31 */

/* Calls to trace a 64bit program from a 32bit program */
#define PPC_PTRACE_PEEKTEXT_3264 0x95
#define PPC_PTRACE_PEEKDATA_3264 0x94
#define PPC_PTRACE_POKETEXT_3264 0x93
#define PPC_PTRACE_POKEDATA_3264 0x92
#define PPC_PTRACE_PEEKUSR_3264  0x91
#define PPC_PTRACE_POKEUSR_3264  0x90

#define PTRACE_SINGLEBLOCK	0x100	/* resume execution until next branch */

#define PPC_PTRACE_GETHWDBGINFO	0x89
#define PPC_PTRACE_SETHWDEBUG	0x88
#define PPC_PTRACE_DELHWDEBUG	0x87

#ifndef __ASSEMBLY__

struct ppc_debug_info {
	__u32 version;			/* Only version 1 exists to date */
	__u32 num_instruction_bps;
	__u32 num_data_bps;
	__u32 num_condition_regs;
	__u32 data_bp_alignment;
	__u32 sizeof_condition;		/* size of the DVC register */
	__u64 features;
};

#endif /* __ASSEMBLY__ */

#define PPC_DEBUG_FEATURE_INSN_BP_RANGE		0x0000000000000001
#define PPC_DEBUG_FEATURE_INSN_BP_MASK		0x0000000000000002
#define PPC_DEBUG_FEATURE_DATA_BP_RANGE		0x0000000000000004
#define PPC_DEBUG_FEATURE_DATA_BP_MASK		0x0000000000000008

#ifndef __ASSEMBLY__

struct ppc_hw_breakpoint {
	__u32 version;		/* currently, version must be 1 */
	__u32 trigger_type;	/* only some combinations allowed */
	__u32 addr_mode;	/* address match mode */
	__u32 condition_mode;	/* break/watchpoint condition flags */
	__u64 addr;		/* break/watchpoint address */
	__u64 addr2;		/* range end or mask */
	__u64 condition_value;	/* contents of the DVC register */
};

#endif /* __ASSEMBLY__ */

#define PPC_BREAKPOINT_TRIGGER_EXECUTE	0x00000001
#define PPC_BREAKPOINT_TRIGGER_READ	0x00000002
#define PPC_BREAKPOINT_TRIGGER_WRITE	0x00000004
#define PPC_BREAKPOINT_TRIGGER_RW	\
	(PPC_BREAKPOINT_TRIGGER_READ | PPC_BREAKPOINT_TRIGGER_WRITE)

#define PPC_BREAKPOINT_MODE_EXACT		0x00000000
#define PPC_BREAKPOINT_MODE_RANGE_INCLUSIVE	0x00000001
#define PPC_BREAKPOINT_MODE_RANGE_EXCLUSIVE	0x00000002
#define PPC_BREAKPOINT_MODE_MASK		0x00000003

#define PPC_BREAKPOINT_CONDITION_MODE	0x00000003
#define PPC_BREAKPOINT_CONDITION_NONE	0x00000000
#define PPC_BREAKPOINT_CONDITION_AND	0x00000001
#define PPC_BREAKPOINT_CONDITION_EXACT	PPC_BREAKPOINT_CONDITION_AND
#define PPC_BREAKPOINT_CONDITION_OR	0x00000002
#define PPC_BREAKPOINT_CONDITION_AND_OR	0x00000003
#define PPC_BREAKPOINT_CONDITION_BE_ALL	0x00ff0000
#define PPC_BREAKPOINT_CONDITION_BE_SHIFT	16
#define PPC_BREAKPOINT_CONDITION_BE(n)	\
	(1<<((n)+PPC_BREAKPOINT_CONDITION_BE_SHIFT))

#endif /* _ASM_POWERPC_PTRACE_H */
