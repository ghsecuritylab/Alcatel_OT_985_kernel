





#ifndef _GL_WEXT_PRIV_H
#define _GL_WEXT_PRIV_H


#define IOCTL_SET_INT                   (SIOCIWFIRSTPRIV + 0)
#define IOCTL_GET_INT                   (SIOCIWFIRSTPRIV + 1)

#define IOCTL_SET_ADDRESS               (SIOCIWFIRSTPRIV + 2)
#define IOCTL_GET_ADDRESS               (SIOCIWFIRSTPRIV + 3)
#define IOCTL_SET_STR                   (SIOCIWFIRSTPRIV + 4)
#define IOCTL_GET_STR                   (SIOCIWFIRSTPRIV + 5)
#define IOCTL_SET_KEY                   (SIOCIWFIRSTPRIV + 6)
#define IOCTL_GET_KEY                   (SIOCIWFIRSTPRIV + 7)
#define IOCTL_SET_STRUCT                (SIOCIWFIRSTPRIV + 8)
#define IOCTL_GET_STRUCT                (SIOCIWFIRSTPRIV + 9)
#define IOCTL_SET_STRUCT_FOR_EM         (SIOCIWFIRSTPRIV + 11)

#define PRIV_CMD_REG_DOMAIN             0
#define PRIV_CMD_BEACON_PERIOD          1
#define PRIV_CMD_ADHOC_MODE             2

#if CFG_TCP_IP_CHKSUM_OFFLOAD
    #define PRIV_CMD_CSUM_OFFLOAD       3
#endif /* CFG_TCP_IP_CHKSUM_OFFLOAD */

#define PRIV_CMD_ROAMING                4
#define PRIV_CMD_VOIP_DELAY             5
#define PRIV_CMD_POWER_MODE             6

#define PRIV_CMD_WMM_PS                 7
#define PRIV_CMD_BT_COEXIST             8
#define PRIV_GPIO2_MODE                 9

#define PRIV_CUSTOM_SET_PTA        		10
#define PRIV_CUSTOM_CONTINUOUS_POLL     11
#define PRIV_CUSTOM_SINGLE_ANTENNA		12
#define PRIV_CUSTOM_BWCS_CMD			13
#define PRIV_CUSTOM_DISABLE_BEACON_DETECTION	14//later
#define PRIV_CMD_OID                15
#define PRIV_SEC_MSG_OID            16

/* General Operational Characteristics */
#define OID_GEN_LINK_SPEED                      0x00010107

/* 802.3 Objects (Ethernet) */
#define OID_802_3_CURRENT_ADDRESS           0x01010102

/* IEEE 802.11 OIDs */
#define OID_802_11_RSSI                         0x0D010206
#define OID_802_11_SUPPORTED_RATES              0x0D01020E
#define OID_802_11_DESIRED_RATES                0x0D010210
#define OID_802_11_CONFIGURATION                0x0D010211

/* PnP and PM OIDs, NDIS default OIDS */
#define OID_PNP_SET_POWER                               0xFD010101

#define OID_CUSTOM_OID_INTERFACE_VERSION                0xFFA0C000

/* MT5921 specific OIDs */
#define OID_CUSTOM_BT_COEXIST_CTRL                      0xFFA0C580
#define OID_CUSTOM_POWER_MANAGEMENT_PROFILE             0xFFA0C581
#define OID_CUSTOM_PATTERN_CONFIG                       0xFFA0C582
#define OID_CUSTOM_BG_SSID_SEARCH_CONFIG                0xFFA0C583
#define OID_CUSTOM_VOIP_SETUP                           0xFFA0C584
#define OID_CUSTOM_ADD_TS                               0xFFA0C585
#define OID_CUSTOM_DEL_TS                               0xFFA0C586
#define OID_CUSTOM_SLT                               0xFFA0C587
#define OID_CUSTOM_ROAMING_EN                           0xFFA0C588
#define OID_CUSTOM_WMM_PS_TEST                          0xFFA0C589
#define OID_CUSTOM_COUNTRY_STRING                       0xFFA0C58A
#define OID_CUSTOM_MULTI_DOMAIN_CAPABILITY              0xFFA0C58B
#define OID_CUSTOM_GPIO2_MODE                           0xFFA0C58C
#define OID_CUSTOM_CONTINUOUS_POLL                      0xFFA0C58D
#define OID_CUSTOM_DISABLE_BEACON_DETECTION             0xFFA0C58E

/* CR1460, WPS privacy bit check disable */
#define OID_CUSTOM_DISABLE_PRIVACY_CHECK                0xFFA0C600

/* Precedent OIDs */
#define OID_CUSTOM_MCR_RW                               0xFFA0C801
#define OID_CUSTOM_EEPROM_RW                            0xFFA0C803


/* RF Test specific OIDs */
#define OID_CUSTOM_TEST_MODE                            0xFFA0C901
#define OID_CUSTOM_TEST_RX_STATUS                       0xFFA0C903
#define OID_CUSTOM_TEST_TX_STATUS                       0xFFA0C905
#define OID_CUSTOM_ABORT_TEST_MODE                      0xFFA0C906
#define OID_CUSTOM_MTK_WIFI_TEST                        0xFFA0C911

/* BWCS */
#define OID_CUSTOM_BWCS_CMD                             0xFFA0C931
#define OID_CUSTOM_SINGLE_ANTENNA                       0xFFA0C932
#define OID_CUSTOM_SET_PTA                              0xFFA0C933
#define OID_CUSTOM_FIXED_RX_GAIN                        0xFFA0C934

/* Config Source */
#define OID_CUSTOM_MTK_EXT_CONFIG                       0xFFA0C940
#define OID_CUSTOM_MTK_NVRAM_RW                         0xFFA0C941
#define OID_CUSTOM_CFG_SRC_TYPE                         0xFFA0C942
#define OID_CUSTOM_EEPROM_TYPE                          0xFFA0C943

#define OID_CUSTOM_PREFER_ANT                           0xFFA0C945

#if SUPPORT_WAPI
#define OID_CUSTOM_WAPI_CAPABILITY                      0xFFA0C950
#define OID_802_11_WAPI_MODE                            0xFFA0CA00
#define OID_802_11_WAPI_ASSOC_INFO                      0xFFA0CA01
#define OID_802_11_SET_WAPI_KEY                         0xFFA0CA02
#endif

#define OID_CUSTOM_WMMPS_MODE                           0xFFA0CA20


/* NIC BBCR configuration entry structure */
typedef struct _PRIV_CONFIG_ENTRY {
    UINT_8      ucOffset;
    UINT_8      ucValue;
} PRIV_CONFIG_ENTRY, *PPRIV_CONFIG_ENTRY;

typedef WLAN_STATUS (*PFN_OID_HANDLER_FUNC_REQ) (
    IN  PVOID       prAdapter,
    IN OUT PVOID    pvBuf,
    IN  UINT_32     u4BufLen,
    OUT PUINT_32    pu4OutInfoLen
    );

typedef enum _ENUM_OID_METHOD_T {
    ENUM_OID_GLUE_ONLY,
    ENUM_OID_GLUE_EXTENSION,
    ENUM_OID_DRIVER_CORE
} ENUM_OID_METHOD_T, *P_ENUM_OID_METHOD_T;

/* OID set/query processing entry */
typedef struct _WLAN_REQ_ENTRY {
    UINT_32            rOid;            /* OID */
    PUINT_8             pucOidName;      /* OID name text */
    BOOLEAN             fgQryBufLenChecking;
    BOOLEAN             fgSetBufLenChecking;
    ENUM_OID_METHOD_T   eOidMethod;
    UINT_32             u4InfoBufLen;
    PFN_OID_HANDLER_FUNC_REQ    pfOidQueryHandler; /*  PFN_OID_HANDLER_FUNC*/
    PFN_OID_HANDLER_FUNC_REQ    pfOidSetHandler; /* PFN_OID_HANDLER_FUNC */
} WLAN_REQ_ENTRY, *P_WLAN_REQ_ENTRY;

typedef struct _NDIS_TRANSPORT_STRUCT {
    UINT_32 ndisOidCmd;
    UINT_32 inNdisOidlength;
    UINT_32 outNdisOidLength;
    UINT_8 ndisOidContent[32];
} NDIS_TRANSPORT_STRUCT, *P_NDIS_TRANSPORT_STRUCT;





int
priv_set_int(
    IN struct net_device *prNetDev,
    IN struct iw_request_info *prIwReqInfo,
    IN union iwreq_data *prIwReqData,
    IN char *pcExtra
    );

int
priv_get_int(
    IN struct net_device *prNetDev,
    IN struct iw_request_info *prIwReqInfo,
    IN union iwreq_data *prIwReqData,
    IN OUT char *pcExtra
    );

int
priv_set_struct(
    IN struct net_device *prNetDev,
    IN struct iw_request_info *prIwReqInfo,
    IN union iwreq_data *prIwReqData,
    IN char *pcExtra
    );

int
priv_get_struct (
    IN struct net_device *prNetDev,
    IN struct iw_request_info *prIwReqInfo,
    IN union iwreq_data *prIwReqData,
    IN OUT char *pcExtra
    );

int
priv_support_ioctl (
    IN struct net_device *prDev,
    IN OUT struct ifreq *prReq,
    IN int i4Cmd
    );


#endif /* _GL_WEXT_PRIV_H */

