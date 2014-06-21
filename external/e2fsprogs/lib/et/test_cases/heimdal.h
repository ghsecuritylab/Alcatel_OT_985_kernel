
#include <et/com_err.h>

#define KRBET_KSUCCESS                           (39525376L)
#define KRBET_KDC_NAME_EXP                       (39525377L)
#define KRBET_KDC_SERVICE_EXP                    (39525378L)
#define KRBET_KDC_AUTH_EXP                       (39525379L)
#define KRBET_KDC_PKT_VER                        (39525380L)
#define KRBET_KDC_P_MKEY_VER                     (39525381L)
#define KRBET_KDC_S_MKEY_VER                     (39525382L)
#define KRBET_KDC_BYTE_ORDER                     (39525383L)
#define KRBET_KDC_PR_UNKNOWN                     (39525384L)
#define KRBET_KDC_PR_N_UNIQUE                    (39525385L)
#define KRBET_KDC_NULL_KEY                       (39525386L)
#define KRBET_KDC_GEN_ERR                        (39525396L)
#define KRBET_GC_TKFIL                           (39525397L)
#define KRBET_GC_NOTKT                           (39525398L)
#define KRBET_MK_AP_TGTEXP                       (39525402L)
#define KRBET_RD_AP_UNDEC                        (39525407L)
#define KRBET_RD_AP_EXP                          (39525408L)
#define KRBET_RD_AP_NYV                          (39525409L)
#define KRBET_RD_AP_REPEAT                       (39525410L)
#define KRBET_RD_AP_NOT_US                       (39525411L)
#define KRBET_RD_AP_INCON                        (39525412L)
#define KRBET_RD_AP_TIME                         (39525413L)
#define KRBET_RD_AP_BADD                         (39525414L)
#define KRBET_RD_AP_VERSION                      (39525415L)
#define KRBET_RD_AP_MSG_TYPE                     (39525416L)
#define KRBET_RD_AP_MODIFIED                     (39525417L)
#define KRBET_RD_AP_ORDER                        (39525418L)
#define KRBET_RD_AP_UNAUTHOR                     (39525419L)
#define KRBET_GT_PW_NULL                         (39525427L)
#define KRBET_GT_PW_BADPW                        (39525428L)
#define KRBET_GT_PW_PROT                         (39525429L)
#define KRBET_GT_PW_KDCERR                       (39525430L)
#define KRBET_GT_PW_NULLTKT                      (39525431L)
#define KRBET_SKDC_RETRY                         (39525432L)
#define KRBET_SKDC_CANT                          (39525433L)
#define KRBET_INTK_W_NOTALL                      (39525437L)
#define KRBET_INTK_BADPW                         (39525438L)
#define KRBET_INTK_PROT                          (39525439L)
#define KRBET_INTK_ERR                           (39525446L)
#define KRBET_AD_NOTGT                           (39525447L)
#define KRBET_NO_TKT_FIL                         (39525452L)
#define KRBET_TKT_FIL_ACC                        (39525453L)
#define KRBET_TKT_FIL_LCK                        (39525454L)
#define KRBET_TKT_FIL_FMT                        (39525455L)
#define KRBET_TKT_FIL_INI                        (39525456L)
#define KRBET_KNAME_FMT                          (39525457L)
extern const struct error_table et_krb_error_table;
extern void initialize_krb_error_table(void);

/* For compatibility with Heimdal */
extern void initialize_krb_error_table_r(struct et_list **list);

#define ERROR_TABLE_BASE_krb (39525376L)

/* for compatibility with older versions... */
#define init_krb_err_tbl initialize_krb_error_table
#define krb_err_base ERROR_TABLE_BASE_krb
