

#ifndef _MTK_RTC_H_
#define _MTK_RTC_H_

#include <linux/kernel.h>
#include <linux/rtc.h>

#define RTC_PWRON_SEC		1	/* must be > 0 */

#define RTC_GPIO_USER_MASK	(((1U << 12) - 1) & 0xff00)

typedef enum {
	RTC_GPIO_USER_WIFI	= 8,
	RTC_GPIO_USER_GPS	= 9,
	RTC_GPIO_USER_BT	= 10,
	RTC_GPIO_USER_FM	= 11,
} rtc_gpio_user_t;

#ifdef CONFIG_MTK_RTC

extern void rtc_gpio_enable_32k(rtc_gpio_user_t user);
extern void rtc_gpio_disable_32k(rtc_gpio_user_t user);

/* NOTE: used in Sleep driver to workaround Vrtc-Vore level shifter issue */
extern void rtc_enable_writeif(void);
extern void rtc_disable_writeif(void);

extern void rtc_mark_recovery(void);
extern void rtc_mark_swreset(void);

extern u16 rtc_rdwr_uart_bits(u16 *val);

extern void rtc_bbpu_power_down(void);

extern void rtc_read_pwron_alarm(struct rtc_wkalrm *alm);

#else

#define rtc_gpio_enable_32k(user)	do {} while (0)
#define rtc_gpio_disable_32k(user)	do {} while (0)
#define rtc_enable_writeif()		do {} while (0)
#define rtc_disable_writeif()		do {} while (0)
#define rtc_mark_recovery()		do {} while (0)
#define rtc_mark_swreset()		do {} while (0)
#define rtc_rdwr_uart_bits(val)		({ 0; })
#define rtc_bbpu_power_down()		do {} while (0)
#define rtc_read_pwron_alarm(alm)	do {} while (0)

#endif

#endif
