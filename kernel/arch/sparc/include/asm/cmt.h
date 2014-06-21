
#ifndef _SPARC64_CMT_H
#define _SPARC64_CMT_H


/* ASI_CORE_ID - private */
#define LP_ID		0x0000000000000010UL
#define  LP_ID_MAX	0x00000000003f0000UL
#define  LP_ID_ID	0x000000000000003fUL

/* ASI_INTR_ID - private */
#define LP_INTR_ID	0x0000000000000000UL
#define  LP_INTR_ID_ID	0x00000000000003ffUL

/* ASI_CESR_ID - private */
#define CESR_ID		0x0000000000000040UL
#define  CESR_ID_ID	0x00000000000000ffUL

/* ASI_CORE_AVAILABLE - shared */
#define LP_AVAIL	0x0000000000000000UL
#define  LP_AVAIL_1	0x0000000000000002UL
#define  LP_AVAIL_0	0x0000000000000001UL

/* ASI_CORE_ENABLE_STATUS - shared */
#define LP_ENAB_STAT	0x0000000000000010UL
#define  LP_ENAB_STAT_1	0x0000000000000002UL
#define  LP_ENAB_STAT_0	0x0000000000000001UL

/* ASI_CORE_ENABLE - shared */
#define LP_ENAB		0x0000000000000020UL
#define  LP_ENAB_1	0x0000000000000002UL
#define  LP_ENAB_0	0x0000000000000001UL

/* ASI_CORE_RUNNING - shared */
#define LP_RUNNING_RW	0x0000000000000050UL
#define LP_RUNNING_W1S	0x0000000000000060UL
#define LP_RUNNING_W1C	0x0000000000000068UL
#define  LP_RUNNING_1	0x0000000000000002UL
#define  LP_RUNNING_0	0x0000000000000001UL

/* ASI_CORE_RUNNING_STAT - shared */
#define LP_RUN_STAT	0x0000000000000058UL
#define  LP_RUN_STAT_1	0x0000000000000002UL
#define  LP_RUN_STAT_0	0x0000000000000001UL

/* ASI_XIR_STEERING - shared */
#define LP_XIR_STEER	0x0000000000000030UL
#define  LP_XIR_STEER_1	0x0000000000000002UL
#define  LP_XIR_STEER_0	0x0000000000000001UL

/* ASI_CMT_ERROR_STEERING - shared */
#define CMT_ER_STEER	0x0000000000000040UL
#define  CMT_ER_STEER_1	0x0000000000000002UL
#define  CMT_ER_STEER_0	0x0000000000000001UL

#endif /* _SPARC64_CMT_H */
