

#ifndef IWLIB_H
#define IWLIB_H

/*#include "CHANGELOG.h"*/

/***************************** INCLUDES *****************************/

/* Standard headers */
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>		/* gethostbyname, getnetbyname */
#include <net/if_ether.h>	/* struct ether_addr */
#include <sys/time.h>		/* struct timeval */
#include <unistd.h>


/* Set of headers proposed by Dr. Michael Rietz <rietz@mail.amps.de>, 27.3.2 */
#include <net/if_arp.h>		/* For ARPHRD_ETHER */
#include <sys/socket.h>		/* For AF_INET & struct sockaddr */
#include <netinet/in.h>         /* For struct sockaddr_in */
#include <netinet/if_ether.h>

#ifndef __user
#define __user
#endif

#include <linux/types.h>		/* for "caddr_t" et al		*/

/* Glibc systems headers are supposedly less problematic than kernel ones */
#include <sys/socket.h>			/* for "struct sockaddr" et al	*/
#include <net/if.h>			/* for IFNAMSIZ and co... */

/* Private copy of Wireless extensions (in this directoty) */
#include "wireless.h"

#undef IW_GCC_HAS_BROKEN_INLINE
#if __GNUC__ == 3
#if __GNUC_MINOR__ >= 1 && __GNUC_MINOR__ < 4
#define IW_GCC_HAS_BROKEN_INLINE	1
#endif	/* __GNUC_MINOR__ */
#endif	/* __GNUC__ */
#if __GNUC__ == 4
#define IW_GCC_HAS_BROKEN_INLINE	1
#endif	/* __GNUC__ */
/* Now, really fix the inline */
#ifdef IW_GCC_HAS_BROKEN_INLINE
#ifdef inline
#undef inline
#endif	/* inline */
#define inline		inline		__attribute__((always_inline))
#endif	/* IW_GCC_HAS_BROKEN_INLINE */

#ifdef __cplusplus
extern "C" {
#endif

/****************************** DEBUG ******************************/

//#define DEBUG 1

/************************ CONSTANTS & MACROS ************************/

/* Various versions information */
/* Recommended Wireless Extension version */
#define WE_VERSION	21
/* Maximum forward compatibility built in this version of WT */
#define WE_MAX_VERSION	22
/* Version of Wireless Tools */
#define WT_VERSION	29

/* Paths */
#define PROC_NET_WIRELESS	"/proc/net/wireless"
#define PROC_NET_DEV		"/proc/net/dev"

/* Some usefull constants */
#define KILO	1e3
#define MEGA	1e6
#define GIGA	1e9
/* For doing log10/exp10 without libm */
#define LOG10_MAGIC	1.25892541179

/* Backward compatibility for network headers */
#ifndef ARPHRD_IEEE80211
#define ARPHRD_IEEE80211 801		/* IEEE 802.11			*/
#endif /* ARPHRD_IEEE80211 */

#ifndef IW_EV_LCP_PK_LEN
/* Size of the Event prefix when packed in stream */
#define IW_EV_LCP_PK_LEN	(4)
/* Size of the various events when packed in stream */
#define IW_EV_CHAR_PK_LEN	(IW_EV_LCP_PK_LEN + IFNAMSIZ)
#define IW_EV_UINT_PK_LEN	(IW_EV_LCP_PK_LEN + sizeof(__u32))
#define IW_EV_FREQ_PK_LEN	(IW_EV_LCP_PK_LEN + sizeof(struct iw_freq))
#define IW_EV_PARAM_PK_LEN	(IW_EV_LCP_PK_LEN + sizeof(struct iw_param))
#define IW_EV_ADDR_PK_LEN	(IW_EV_LCP_PK_LEN + sizeof(struct sockaddr))
#define IW_EV_QUAL_PK_LEN	(IW_EV_LCP_PK_LEN + sizeof(struct iw_quality))
#define IW_EV_POINT_PK_LEN	(IW_EV_LCP_PK_LEN + 4)
#endif	/* IW_EV_LCP_PK_LEN */

struct iw_pk_event
{
	__u16		len;			/* Real lenght of this stuff */
	__u16		cmd;			/* Wireless IOCTL */
	union iwreq_data	u;		/* IOCTL fixed payload */
} __attribute__ ((packed));
struct	iw_pk_point
{
  void __user	*pointer;	/* Pointer to the data  (in user space) */
  __u16		length;		/* number of fields or size in bytes */
  __u16		flags;		/* Optional params */
} __attribute__ ((packed));

#define IW_EV_LCP_PK2_LEN	(sizeof(struct iw_pk_event) - sizeof(union iwreq_data))
#define IW_EV_POINT_PK2_LEN	(IW_EV_LCP_PK2_LEN + sizeof(struct iw_pk_point) - IW_EV_POINT_OFF)

/****************************** TYPES ******************************/

/* Shortcuts */
typedef struct iw_statistics	iwstats;
typedef struct iw_range		iwrange;
typedef struct iw_param		iwparam;
typedef struct iw_freq		iwfreq;
typedef struct iw_quality	iwqual;
typedef struct iw_priv_args	iwprivargs;
typedef struct sockaddr		sockaddr;

typedef struct wireless_config
{
  char		name[IFNAMSIZ + 1];	/* Wireless/protocol name */
  int		has_nwid;
  iwparam	nwid;			/* Network ID */
  int		has_freq;
  double	freq;			/* Frequency/channel */
  int		freq_flags;
  int		has_key;
  unsigned char	key[IW_ENCODING_TOKEN_MAX];	/* Encoding key used */
  int		key_size;		/* Number of bytes */
  int		key_flags;		/* Various flags */
  int		has_essid;
  int		essid_on;
  char		essid[IW_ESSID_MAX_SIZE + 1];	/* ESSID (extended network) */
  int		has_mode;
  int		mode;			/* Operation mode */
} wireless_config;

typedef struct wireless_info
{
  struct wireless_config	b;	/* Basic information */

  int		has_sens;
  iwparam	sens;			/* sensitivity */
  int		has_nickname;
  char		nickname[IW_ESSID_MAX_SIZE + 1]; /* NickName */
  int		has_ap_addr;
  sockaddr	ap_addr;		/* Access point address */
  int		has_bitrate;
  iwparam	bitrate;		/* Bit rate in bps */
  int		has_rts;
  iwparam	rts;			/* RTS threshold in bytes */
  int		has_frag;
  iwparam	frag;			/* Fragmentation threshold in bytes */
  int		has_power;
  iwparam	power;			/* Power management parameters */
  int		has_txpower;
  iwparam	txpower;		/* Transmit Power in dBm */
  int		has_retry;
  iwparam	retry;			/* Retry limit or lifetime */

  /* Stats */
  iwstats	stats;
  int		has_stats;
  iwrange	range;
  int		has_range;

  /* Auth params for WPA/802.1x/802.11i */
  int		auth_key_mgmt;
  int		has_auth_key_mgmt;
  int		auth_cipher_pairwise;
  int		has_auth_cipher_pairwise;
  int		auth_cipher_group;
  int		has_auth_cipher_group;
} wireless_info;

typedef struct wireless_scan
{
  /* Linked list */
  struct wireless_scan *	next;

  /* Cell identifiaction */
  int		has_ap_addr;
  sockaddr	ap_addr;		/* Access point address */

  /* Other information */
  struct wireless_config	b;	/* Basic information */
  iwstats	stats;			/* Signal strength */
  int		has_stats;
  iwparam	maxbitrate;		/* Max bit rate in bps */
  int		has_maxbitrate;
} wireless_scan;

typedef struct wireless_scan_head
{
  wireless_scan *	result;		/* Result of the scan */
  int			retry;		/* Retry level */
} wireless_scan_head;

typedef struct stream_descr
{
  char *	end;		/* End of the stream */
  char *	current;	/* Current event in stream of events */
  char *	value;		/* Current value in event */
} stream_descr;

typedef int (*iw_enum_handler)(int	skfd,
			       char *	ifname,
			       char *	args[],
			       int	count);

/* Describe a modulation */
typedef struct iw_modul_descr
{
  unsigned int		mask;		/* Modulation bitmask */
  char			cmd[8];		/* Short name */
  char *		verbose;	/* Verbose description */
} iw_modul_descr;

/**************************** PROTOTYPES ****************************/

/* ---------------------- SOCKET SUBROUTINES -----------------------*/
int
	iw_sockets_open(void);
void
	iw_enum_devices(int		skfd,
			iw_enum_handler fn,
			char *		args[],
			int		count);
/* --------------------- WIRELESS SUBROUTINES ----------------------*/
int
	iw_get_kernel_we_version(void);
int
	iw_print_version_info(const char *	toolname);
int
	iw_get_range_info(int		skfd,
			  const char *	ifname,
			  iwrange *	range);
int
	iw_get_priv_info(int		skfd,
			 const char *	ifname,
			 iwprivargs **	ppriv);
int
	iw_get_basic_config(int			skfd,
			    const char *	ifname,
			    wireless_config *	info);
int
	iw_set_basic_config(int			skfd,
			    const char *	ifname,
			    wireless_config *	info);
/* --------------------- PROTOCOL SUBROUTINES --------------------- */
int
	iw_protocol_compare(const char *	protocol1,
			    const char *	protocol2);
/* -------------------- FREQUENCY SUBROUTINES --------------------- */
void
	iw_float2freq(double	in,
		      iwfreq *	out);
double
	iw_freq2float(const iwfreq *	in);
void
	iw_print_freq_value(char *	buffer,
			    int		buflen,
			    double	freq);
void
	iw_print_freq(char *	buffer,
		      int	buflen,
		      double	freq,
		      int	channel,
		      int	freq_flags);
int
	iw_freq_to_channel(double			freq,
			   const struct iw_range *	range);
int
	iw_channel_to_freq(int				channel,
			   double *			pfreq,
			   const struct iw_range *	range);
void
	iw_print_bitrate(char *	buffer,
			 int	buflen,
			 int	bitrate);
/* ---------------------- POWER SUBROUTINES ----------------------- */
int
	iw_dbm2mwatt(int	in);
int
	iw_mwatt2dbm(int	in);
void
	iw_print_txpower(char *			buffer,
			 int			buflen,
			 struct iw_param *	txpower);
/* -------------------- STATISTICS SUBROUTINES -------------------- */
int
	iw_get_stats(int		skfd,
		     const char *	ifname,
		     iwstats *		stats,
		     const iwrange *	range,
		     int		has_range);
void
	iw_print_stats(char *		buffer,
		       int		buflen,
		       const iwqual *	qual,
		       const iwrange *	range,
		       int		has_range);
/* --------------------- ENCODING SUBROUTINES --------------------- */
void
	iw_print_key(char *			buffer,
		     int			buflen,
		     const unsigned char *	key,
		     int			key_size,
		     int			key_flags);
int
	iw_in_key(const char *		input,
		  unsigned char *	key);
int
	iw_in_key_full(int		skfd,
		       const char *	ifname,
		       const char *	input,
		       unsigned char *	key,
		       __u16 *		flags);
/* ----------------- POWER MANAGEMENT SUBROUTINES ----------------- */
void
	iw_print_pm_value(char *	buffer,
			  int		buflen,
			  int		value,
			  int		flags,
			  int		we_version);
void
	iw_print_pm_mode(char *		buffer,
			 int		buflen,
			 int		flags);
/* --------------- RETRY LIMIT/LIFETIME SUBROUTINES --------------- */
void
	iw_print_retry_value(char *	buffer,
			     int	buflen,
			     int	value,
			     int	flags,
			     int	we_version);
/* ----------------------- TIME SUBROUTINES ----------------------- */
void
	iw_print_timeval(char *				buffer,
			 int				buflen,
			 const struct timeval *		time,
			 const struct timezone *	tz);
/* --------------------- ADDRESS SUBROUTINES ---------------------- */
int
	iw_check_mac_addr_type(int		skfd,
			       const char *	ifname);
int
	iw_check_if_addr_type(int		skfd,
			      const char *	ifname);
#if 0
int
	iw_check_addr_type(int		skfd,
			   const char *	ifname);
#endif
#if 0
int
	iw_get_mac_addr(int			skfd,
			const char *		name,
			struct ether_addr *	eth,
			unsigned short *	ptype);
#endif
char *
	iw_mac_ntop(const unsigned char *	mac,
		    int				maclen,
		    char *			buf,
		    int				buflen);
void
	iw_ether_ntop(const struct ether_addr *	eth,
		      char *			buf);
char *
	iw_sawap_ntop(const struct sockaddr *	sap,
		      char *			buf);
int
	iw_mac_aton(const char *	orig,
		    unsigned char *	mac,
		    int			macmax);
int
	iw_ether_aton(const char* bufp, struct ether_addr* eth);
int
	iw_in_inet(char *bufp, struct sockaddr *sap);
int
	iw_in_addr(int			skfd,
		   const char *		ifname,
		   char *		bufp,
		   struct sockaddr *	sap);
/* ----------------------- MISC SUBROUTINES ------------------------ */
int
	iw_get_priv_size(int		args);

/* ---------------------- EVENT SUBROUTINES ---------------------- */
void
	iw_init_event_stream(struct stream_descr *	stream,
			     char *			data,
			     int			len);
int
	iw_extract_event_stream(struct stream_descr *	stream,
				struct iw_event *	iwe,
				int			we_version);
/* --------------------- SCANNING SUBROUTINES --------------------- */
int
	iw_process_scan(int			skfd,
			char *			ifname,
			int			we_version,
			wireless_scan_head *	context);
int
	iw_scan(int			skfd,
		char *			ifname,
		int			we_version,
		wireless_scan_head *	context);

/**************************** VARIABLES ****************************/

/* Modes as human readable strings */
extern const char * const	iw_operation_mode[];
#define IW_NUM_OPER_MODE	7
#define IW_NUM_OPER_MODE_EXT	8

/* Modulations as human readable strings */
extern const struct iw_modul_descr	iw_modul_list[];
#define IW_SIZE_MODUL_LIST	16

/************************* INLINE FUNTIONS *************************/


/*------------------------------------------------------------------*/
static inline int
iw_set_ext(int			skfd,		/* Socket to the kernel */
	   const char *		ifname,		/* Device name */
	   int			request,	/* WE ID */
	   struct iwreq *	pwrq)		/* Fixed part of the request */
{
  /* Set device name */
  strncpy(pwrq->ifr_name, ifname, IFNAMSIZ);
  /* Do the request */
  return(ioctl(skfd, request, pwrq));
}

/*------------------------------------------------------------------*/
static inline int
iw_get_ext(int			skfd,		/* Socket to the kernel */
	   const char *		ifname,		/* Device name */
	   int			request,	/* WE ID */
	   struct iwreq *	pwrq)		/* Fixed part of the request */
{
  /* Set device name */
  strncpy(pwrq->ifr_name, ifname, IFNAMSIZ);
  /* Do the request */
  return(ioctl(skfd, request, pwrq));
}

/*------------------------------------------------------------------*/
static inline void
iw_sockets_close(int	skfd)
{
  close(skfd);
}

/*------------------------------------------------------------------*/
static inline char *
iw_saether_ntop(const struct sockaddr *sap, char* bufp)
{
  iw_ether_ntop((const struct ether_addr *) sap->sa_data, bufp);
  return bufp;
}
/*------------------------------------------------------------------*/
static inline int
iw_saether_aton(const char *bufp, struct sockaddr *sap)
{
  sap->sa_family = ARPHRD_ETHER;
  return iw_ether_aton(bufp, (struct ether_addr *) sap->sa_data);
}

/*------------------------------------------------------------------*/
static inline void
iw_broad_ether(struct sockaddr *sap)
{
  sap->sa_family = ARPHRD_ETHER;
  memset((char *) sap->sa_data, 0xFF, ETH_ALEN);
}

/*------------------------------------------------------------------*/
static inline void
iw_null_ether(struct sockaddr *sap)
{
  sap->sa_family = ARPHRD_ETHER;
  memset((char *) sap->sa_data, 0x00, ETH_ALEN);
}

/*------------------------------------------------------------------*/
static inline int
iw_ether_cmp(const struct ether_addr* eth1, const struct ether_addr* eth2)
{
  return memcmp(eth1, eth2, sizeof(*eth1));
}

#ifdef __cplusplus
}
#endif

#endif	/* IWLIB_H */
