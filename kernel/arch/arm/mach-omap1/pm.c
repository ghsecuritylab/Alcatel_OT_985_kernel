

#include <linux/suspend.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <asm/atomic.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>

#include <plat/cpu.h>
#include <mach/irqs.h>
#include <plat/clock.h>
#include <plat/sram.h>
#include <plat/tc.h>
#include <plat/mux.h>
#include <plat/dma.h>
#include <plat/dmtimer.h>

#include "pm.h"

static unsigned int arm_sleep_save[ARM_SLEEP_SAVE_SIZE];
static unsigned short dsp_sleep_save[DSP_SLEEP_SAVE_SIZE];
static unsigned short ulpd_sleep_save[ULPD_SLEEP_SAVE_SIZE];
static unsigned int mpui7xx_sleep_save[MPUI7XX_SLEEP_SAVE_SIZE];
static unsigned int mpui1510_sleep_save[MPUI1510_SLEEP_SAVE_SIZE];
static unsigned int mpui1610_sleep_save[MPUI1610_SLEEP_SAVE_SIZE];

#ifdef CONFIG_OMAP_32K_TIMER

static unsigned short enable_dyn_sleep = 1;

static ssize_t idle_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	return sprintf(buf, "%hu\n", enable_dyn_sleep);
}

static ssize_t idle_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char * buf, size_t n)
{
	unsigned short value;
	if (sscanf(buf, "%hu", &value) != 1 ||
	    (value != 0 && value != 1)) {
		printk(KERN_ERR "idle_sleep_store: Invalid value\n");
		return -EINVAL;
	}
	enable_dyn_sleep = value;
	return n;
}

static struct kobj_attribute sleep_while_idle_attr =
	__ATTR(sleep_while_idle, 0644, idle_show, idle_store);

#endif

static void (*omap_sram_suspend)(unsigned long r0, unsigned long r1) = NULL;

void omap1_pm_idle(void)
{
	extern __u32 arm_idlect1_mask;
	__u32 use_idlect1 = arm_idlect1_mask;
	int do_sleep = 0;

	local_irq_disable();
	local_fiq_disable();
	if (need_resched()) {
		local_fiq_enable();
		local_irq_enable();
		return;
	}

#ifdef CONFIG_OMAP_MPU_TIMER
#warning Enable 32kHz OS timer in order to allow sleep states in idle
	use_idlect1 = use_idlect1 & ~(1 << 9);
#else

	while (enable_dyn_sleep) {

#ifdef CONFIG_CBUS_TAHVO_USB
		extern int vbus_active;
		/* Clock requirements? */
		if (vbus_active)
			break;
#endif
		do_sleep = 1;
		break;
	}

#endif

#ifdef CONFIG_OMAP_DM_TIMER
	use_idlect1 = omap_dm_timer_modify_idlect_mask(use_idlect1);
#endif

	if (omap_dma_running())
		use_idlect1 &= ~(1 << 6);

	/* We should be able to remove the do_sleep variable and multiple
	 * tests above as soon as drivers, timer and DMA code have been fixed.
	 * Even the sleep block count should become obsolete. */
	if ((use_idlect1 != ~0) || !do_sleep) {

		__u32 saved_idlect1 = omap_readl(ARM_IDLECT1);
		if (cpu_is_omap15xx())
			use_idlect1 &= OMAP1510_BIG_SLEEP_REQUEST;
		else
			use_idlect1 &= OMAP1610_IDLECT1_SLEEP_VAL;
		omap_writel(use_idlect1, ARM_IDLECT1);
		__asm__ volatile ("mcr	p15, 0, r0, c7, c0, 4");
		omap_writel(saved_idlect1, ARM_IDLECT1);

		local_fiq_enable();
		local_irq_enable();
		return;
	}
	omap_sram_suspend(omap_readl(ARM_IDLECT1),
			  omap_readl(ARM_IDLECT2));

	local_fiq_enable();
	local_irq_enable();
}

static void omap_pm_wakeup_setup(void)
{
	u32 level1_wake = 0;
	u32 level2_wake = OMAP_IRQ_BIT(INT_UART2);

	/*
	 * Turn off all interrupts except GPIO bank 1, L1-2nd level cascade,
	 * and the L2 wakeup interrupts: keypad and UART2. Note that the
	 * drivers must still separately call omap_set_gpio_wakeup() to
	 * wake up to a GPIO interrupt.
	 */
	if (cpu_is_omap7xx())
		level1_wake = OMAP_IRQ_BIT(INT_7XX_GPIO_BANK1) |
			OMAP_IRQ_BIT(INT_7XX_IH2_IRQ);
	else if (cpu_is_omap15xx())
		level1_wake = OMAP_IRQ_BIT(INT_GPIO_BANK1) |
			OMAP_IRQ_BIT(INT_1510_IH2_IRQ);
	else if (cpu_is_omap16xx())
		level1_wake = OMAP_IRQ_BIT(INT_GPIO_BANK1) |
			OMAP_IRQ_BIT(INT_1610_IH2_IRQ);

	omap_writel(~level1_wake, OMAP_IH1_MIR);

	if (cpu_is_omap7xx()) {
		omap_writel(~level2_wake, OMAP_IH2_0_MIR);
		omap_writel(~(OMAP_IRQ_BIT(INT_7XX_WAKE_UP_REQ) |
				OMAP_IRQ_BIT(INT_7XX_MPUIO_KEYPAD)),
				OMAP_IH2_1_MIR);
	} else if (cpu_is_omap15xx()) {
		level2_wake |= OMAP_IRQ_BIT(INT_KEYBOARD);
		omap_writel(~level2_wake,  OMAP_IH2_MIR);
	} else if (cpu_is_omap16xx()) {
		level2_wake |= OMAP_IRQ_BIT(INT_KEYBOARD);
		omap_writel(~level2_wake, OMAP_IH2_0_MIR);

		/* INT_1610_WAKE_UP_REQ is needed for GPIO wakeup... */
		omap_writel(~OMAP_IRQ_BIT(INT_1610_WAKE_UP_REQ),
			    OMAP_IH2_1_MIR);
		omap_writel(~0x0, OMAP_IH2_2_MIR);
		omap_writel(~0x0, OMAP_IH2_3_MIR);
	}

	/*  New IRQ agreement, recalculate in cascade order */
	omap_writel(1, OMAP_IH2_CONTROL);
	omap_writel(1, OMAP_IH1_CONTROL);
}

#define EN_DSPCK	13	/* ARM_CKCTL */
#define EN_APICK	6	/* ARM_IDLECT2 */
#define DSP_EN		1	/* ARM_RSTCT1 */

void omap1_pm_suspend(void)
{
	unsigned long arg0 = 0, arg1 = 0;

	printk(KERN_INFO "PM: OMAP%x is trying to enter deep sleep...\n",
		omap_rev());

	omap_serial_wake_trigger(1);

	if (!cpu_is_omap15xx())
		omap_writew(0xffff, ULPD_SOFT_DISABLE_REQ_REG);

	/*
	 * Step 1: turn off interrupts (FIXME: NOTE: already disabled)
	 */

	local_irq_disable();
	local_fiq_disable();

	/*
	 * Step 2: save registers
	 *
	 * The omap is a strange/beautiful device. The caches, memory
	 * and register state are preserved across power saves.
	 * We have to save and restore very little register state to
	 * idle the omap.
         *
	 * Save interrupt, MPUI, ARM and UPLD control registers.
	 */

	if (cpu_is_omap7xx()) {
		MPUI7XX_SAVE(OMAP_IH1_MIR);
		MPUI7XX_SAVE(OMAP_IH2_0_MIR);
		MPUI7XX_SAVE(OMAP_IH2_1_MIR);
		MPUI7XX_SAVE(MPUI_CTRL);
		MPUI7XX_SAVE(MPUI_DSP_BOOT_CONFIG);
		MPUI7XX_SAVE(MPUI_DSP_API_CONFIG);
		MPUI7XX_SAVE(EMIFS_CONFIG);
		MPUI7XX_SAVE(EMIFF_SDRAM_CONFIG);

	} else if (cpu_is_omap15xx()) {
		MPUI1510_SAVE(OMAP_IH1_MIR);
		MPUI1510_SAVE(OMAP_IH2_MIR);
		MPUI1510_SAVE(MPUI_CTRL);
		MPUI1510_SAVE(MPUI_DSP_BOOT_CONFIG);
		MPUI1510_SAVE(MPUI_DSP_API_CONFIG);
		MPUI1510_SAVE(EMIFS_CONFIG);
		MPUI1510_SAVE(EMIFF_SDRAM_CONFIG);
	} else if (cpu_is_omap16xx()) {
		MPUI1610_SAVE(OMAP_IH1_MIR);
		MPUI1610_SAVE(OMAP_IH2_0_MIR);
		MPUI1610_SAVE(OMAP_IH2_1_MIR);
		MPUI1610_SAVE(OMAP_IH2_2_MIR);
		MPUI1610_SAVE(OMAP_IH2_3_MIR);
		MPUI1610_SAVE(MPUI_CTRL);
		MPUI1610_SAVE(MPUI_DSP_BOOT_CONFIG);
		MPUI1610_SAVE(MPUI_DSP_API_CONFIG);
		MPUI1610_SAVE(EMIFS_CONFIG);
		MPUI1610_SAVE(EMIFF_SDRAM_CONFIG);
	}

	ARM_SAVE(ARM_CKCTL);
	ARM_SAVE(ARM_IDLECT1);
	ARM_SAVE(ARM_IDLECT2);
	if (!(cpu_is_omap15xx()))
		ARM_SAVE(ARM_IDLECT3);
	ARM_SAVE(ARM_EWUPCT);
	ARM_SAVE(ARM_RSTCT1);
	ARM_SAVE(ARM_RSTCT2);
	ARM_SAVE(ARM_SYSST);
	ULPD_SAVE(ULPD_CLOCK_CTRL);
	ULPD_SAVE(ULPD_STATUS_REQ);

	/* (Step 3 removed - we now allow deep sleep by default) */

	/*
	 * Step 4: OMAP DSP Shutdown
	 */

	/* stop DSP */
	omap_writew(omap_readw(ARM_RSTCT1) & ~(1 << DSP_EN), ARM_RSTCT1);

		/* shut down dsp_ck */
	if (!cpu_is_omap7xx())
		omap_writew(omap_readw(ARM_CKCTL) & ~(1 << EN_DSPCK), ARM_CKCTL);

	/* temporarily enabling api_ck to access DSP registers */
	omap_writew(omap_readw(ARM_IDLECT2) | 1 << EN_APICK, ARM_IDLECT2);

	/* save DSP registers */
	DSP_SAVE(DSP_IDLECT2);

	/* Stop all DSP domain clocks */
	__raw_writew(0, DSP_IDLECT2);

	/*
	 * Step 5: Wakeup Event Setup
	 */

	omap_pm_wakeup_setup();

	/*
	 * Step 6: ARM and Traffic controller shutdown
	 */

	/* disable ARM watchdog */
	omap_writel(0x00F5, OMAP_WDT_TIMER_MODE);
	omap_writel(0x00A0, OMAP_WDT_TIMER_MODE);

	/*
	 * Step 6b: ARM and Traffic controller shutdown
	 *
	 * Step 6 continues here. Prepare jump to power management
	 * assembly code in internal SRAM.
	 *
	 * Since the omap_cpu_suspend routine has been copied to
	 * SRAM, we'll do an indirect procedure call to it and pass the
	 * contents of arm_idlect1 and arm_idlect2 so it can restore
	 * them when it wakes up and it will return.
	 */

	arg0 = arm_sleep_save[ARM_SLEEP_SAVE_ARM_IDLECT1];
	arg1 = arm_sleep_save[ARM_SLEEP_SAVE_ARM_IDLECT2];

	/*
	 * Step 6c: ARM and Traffic controller shutdown
	 *
	 * Jump to assembly code. The processor will stay there
	 * until wake up.
	 */
	omap_sram_suspend(arg0, arg1);

	/*
	 * If we are here, processor is woken up!
	 */

	/*
	 * Restore DSP clocks
	 */

	/* again temporarily enabling api_ck to access DSP registers */
	omap_writew(omap_readw(ARM_IDLECT2) | 1 << EN_APICK, ARM_IDLECT2);

	/* Restore DSP domain clocks */
	DSP_RESTORE(DSP_IDLECT2);

	/*
	 * Restore ARM state, except ARM_IDLECT1/2 which omap_cpu_suspend did
	 */

	if (!(cpu_is_omap15xx()))
		ARM_RESTORE(ARM_IDLECT3);
	ARM_RESTORE(ARM_CKCTL);
	ARM_RESTORE(ARM_EWUPCT);
	ARM_RESTORE(ARM_RSTCT1);
	ARM_RESTORE(ARM_RSTCT2);
	ARM_RESTORE(ARM_SYSST);
	ULPD_RESTORE(ULPD_CLOCK_CTRL);
	ULPD_RESTORE(ULPD_STATUS_REQ);

	if (cpu_is_omap7xx()) {
		MPUI7XX_RESTORE(EMIFS_CONFIG);
		MPUI7XX_RESTORE(EMIFF_SDRAM_CONFIG);
		MPUI7XX_RESTORE(OMAP_IH1_MIR);
		MPUI7XX_RESTORE(OMAP_IH2_0_MIR);
		MPUI7XX_RESTORE(OMAP_IH2_1_MIR);
	} else if (cpu_is_omap15xx()) {
		MPUI1510_RESTORE(MPUI_CTRL);
		MPUI1510_RESTORE(MPUI_DSP_BOOT_CONFIG);
		MPUI1510_RESTORE(MPUI_DSP_API_CONFIG);
		MPUI1510_RESTORE(EMIFS_CONFIG);
		MPUI1510_RESTORE(EMIFF_SDRAM_CONFIG);
		MPUI1510_RESTORE(OMAP_IH1_MIR);
		MPUI1510_RESTORE(OMAP_IH2_MIR);
	} else if (cpu_is_omap16xx()) {
		MPUI1610_RESTORE(MPUI_CTRL);
		MPUI1610_RESTORE(MPUI_DSP_BOOT_CONFIG);
		MPUI1610_RESTORE(MPUI_DSP_API_CONFIG);
		MPUI1610_RESTORE(EMIFS_CONFIG);
		MPUI1610_RESTORE(EMIFF_SDRAM_CONFIG);

		MPUI1610_RESTORE(OMAP_IH1_MIR);
		MPUI1610_RESTORE(OMAP_IH2_0_MIR);
		MPUI1610_RESTORE(OMAP_IH2_1_MIR);
		MPUI1610_RESTORE(OMAP_IH2_2_MIR);
		MPUI1610_RESTORE(OMAP_IH2_3_MIR);
	}

	if (!cpu_is_omap15xx())
		omap_writew(0, ULPD_SOFT_DISABLE_REQ_REG);

	/*
	 * Re-enable interrupts
	 */

	local_irq_enable();
	local_fiq_enable();

	omap_serial_wake_trigger(0);

	printk(KERN_INFO "PM: OMAP%x is re-starting from deep sleep...\n",
		omap_rev());
}

#if defined(DEBUG) && defined(CONFIG_PROC_FS)
static int g_read_completed;

static int omap_pm_read_proc(
	char *page_buffer,
	char **my_first_byte,
	off_t virtual_start,
	int length,
	int *eof,
	void *data)
{
	int my_buffer_offset = 0;
	char * const my_base = page_buffer;

	ARM_SAVE(ARM_CKCTL);
	ARM_SAVE(ARM_IDLECT1);
	ARM_SAVE(ARM_IDLECT2);
	if (!(cpu_is_omap15xx()))
		ARM_SAVE(ARM_IDLECT3);
	ARM_SAVE(ARM_EWUPCT);
	ARM_SAVE(ARM_RSTCT1);
	ARM_SAVE(ARM_RSTCT2);
	ARM_SAVE(ARM_SYSST);

	ULPD_SAVE(ULPD_IT_STATUS);
	ULPD_SAVE(ULPD_CLOCK_CTRL);
	ULPD_SAVE(ULPD_SOFT_REQ);
	ULPD_SAVE(ULPD_STATUS_REQ);
	ULPD_SAVE(ULPD_DPLL_CTRL);
	ULPD_SAVE(ULPD_POWER_CTRL);

	if (cpu_is_omap7xx()) {
		MPUI7XX_SAVE(MPUI_CTRL);
		MPUI7XX_SAVE(MPUI_DSP_STATUS);
		MPUI7XX_SAVE(MPUI_DSP_BOOT_CONFIG);
		MPUI7XX_SAVE(MPUI_DSP_API_CONFIG);
		MPUI7XX_SAVE(EMIFF_SDRAM_CONFIG);
		MPUI7XX_SAVE(EMIFS_CONFIG);
	} else if (cpu_is_omap15xx()) {
		MPUI1510_SAVE(MPUI_CTRL);
		MPUI1510_SAVE(MPUI_DSP_STATUS);
		MPUI1510_SAVE(MPUI_DSP_BOOT_CONFIG);
		MPUI1510_SAVE(MPUI_DSP_API_CONFIG);
		MPUI1510_SAVE(EMIFF_SDRAM_CONFIG);
		MPUI1510_SAVE(EMIFS_CONFIG);
	} else if (cpu_is_omap16xx()) {
		MPUI1610_SAVE(MPUI_CTRL);
		MPUI1610_SAVE(MPUI_DSP_STATUS);
		MPUI1610_SAVE(MPUI_DSP_BOOT_CONFIG);
		MPUI1610_SAVE(MPUI_DSP_API_CONFIG);
		MPUI1610_SAVE(EMIFF_SDRAM_CONFIG);
		MPUI1610_SAVE(EMIFS_CONFIG);
	}

	if (virtual_start == 0) {
		g_read_completed = 0;

		my_buffer_offset += sprintf(my_base + my_buffer_offset,
		   "ARM_CKCTL_REG:            0x%-8x     \n"
		   "ARM_IDLECT1_REG:          0x%-8x     \n"
		   "ARM_IDLECT2_REG:          0x%-8x     \n"
		   "ARM_IDLECT3_REG:	      0x%-8x     \n"
		   "ARM_EWUPCT_REG:           0x%-8x     \n"
		   "ARM_RSTCT1_REG:           0x%-8x     \n"
		   "ARM_RSTCT2_REG:           0x%-8x     \n"
		   "ARM_SYSST_REG:            0x%-8x     \n"
		   "ULPD_IT_STATUS_REG:       0x%-4x     \n"
		   "ULPD_CLOCK_CTRL_REG:      0x%-4x     \n"
		   "ULPD_SOFT_REQ_REG:        0x%-4x     \n"
		   "ULPD_DPLL_CTRL_REG:       0x%-4x     \n"
		   "ULPD_STATUS_REQ_REG:      0x%-4x     \n"
		   "ULPD_POWER_CTRL_REG:      0x%-4x     \n",
		   ARM_SHOW(ARM_CKCTL),
		   ARM_SHOW(ARM_IDLECT1),
		   ARM_SHOW(ARM_IDLECT2),
		   ARM_SHOW(ARM_IDLECT3),
		   ARM_SHOW(ARM_EWUPCT),
		   ARM_SHOW(ARM_RSTCT1),
		   ARM_SHOW(ARM_RSTCT2),
		   ARM_SHOW(ARM_SYSST),
		   ULPD_SHOW(ULPD_IT_STATUS),
		   ULPD_SHOW(ULPD_CLOCK_CTRL),
		   ULPD_SHOW(ULPD_SOFT_REQ),
		   ULPD_SHOW(ULPD_DPLL_CTRL),
		   ULPD_SHOW(ULPD_STATUS_REQ),
		   ULPD_SHOW(ULPD_POWER_CTRL));

		if (cpu_is_omap7xx()) {
			my_buffer_offset += sprintf(my_base + my_buffer_offset,
			   "MPUI7XX_CTRL_REG	     0x%-8x \n"
			   "MPUI7XX_DSP_STATUS_REG:      0x%-8x \n"
			   "MPUI7XX_DSP_BOOT_CONFIG_REG: 0x%-8x \n"
			   "MPUI7XX_DSP_API_CONFIG_REG:  0x%-8x \n"
			   "MPUI7XX_SDRAM_CONFIG_REG:    0x%-8x \n"
			   "MPUI7XX_EMIFS_CONFIG_REG:    0x%-8x \n",
			   MPUI7XX_SHOW(MPUI_CTRL),
			   MPUI7XX_SHOW(MPUI_DSP_STATUS),
			   MPUI7XX_SHOW(MPUI_DSP_BOOT_CONFIG),
			   MPUI7XX_SHOW(MPUI_DSP_API_CONFIG),
			   MPUI7XX_SHOW(EMIFF_SDRAM_CONFIG),
			   MPUI7XX_SHOW(EMIFS_CONFIG));
		} else if (cpu_is_omap15xx()) {
			my_buffer_offset += sprintf(my_base + my_buffer_offset,
			   "MPUI1510_CTRL_REG             0x%-8x \n"
			   "MPUI1510_DSP_STATUS_REG:      0x%-8x \n"
			   "MPUI1510_DSP_BOOT_CONFIG_REG: 0x%-8x \n"
			   "MPUI1510_DSP_API_CONFIG_REG:  0x%-8x \n"
			   "MPUI1510_SDRAM_CONFIG_REG:    0x%-8x \n"
			   "MPUI1510_EMIFS_CONFIG_REG:    0x%-8x \n",
			   MPUI1510_SHOW(MPUI_CTRL),
			   MPUI1510_SHOW(MPUI_DSP_STATUS),
			   MPUI1510_SHOW(MPUI_DSP_BOOT_CONFIG),
			   MPUI1510_SHOW(MPUI_DSP_API_CONFIG),
			   MPUI1510_SHOW(EMIFF_SDRAM_CONFIG),
			   MPUI1510_SHOW(EMIFS_CONFIG));
		} else if (cpu_is_omap16xx()) {
			my_buffer_offset += sprintf(my_base + my_buffer_offset,
			   "MPUI1610_CTRL_REG             0x%-8x \n"
			   "MPUI1610_DSP_STATUS_REG:      0x%-8x \n"
			   "MPUI1610_DSP_BOOT_CONFIG_REG: 0x%-8x \n"
			   "MPUI1610_DSP_API_CONFIG_REG:  0x%-8x \n"
			   "MPUI1610_SDRAM_CONFIG_REG:    0x%-8x \n"
			   "MPUI1610_EMIFS_CONFIG_REG:    0x%-8x \n",
			   MPUI1610_SHOW(MPUI_CTRL),
			   MPUI1610_SHOW(MPUI_DSP_STATUS),
			   MPUI1610_SHOW(MPUI_DSP_BOOT_CONFIG),
			   MPUI1610_SHOW(MPUI_DSP_API_CONFIG),
			   MPUI1610_SHOW(EMIFF_SDRAM_CONFIG),
			   MPUI1610_SHOW(EMIFS_CONFIG));
		}

		g_read_completed++;
	} else if (g_read_completed >= 1) {
		 *eof = 1;
		 return 0;
	}
	g_read_completed++;

	*my_first_byte = page_buffer;
	return  my_buffer_offset;
}

static void omap_pm_init_proc(void)
{
	struct proc_dir_entry *entry;

	entry = create_proc_read_entry("driver/omap_pm",
				       S_IWUSR | S_IRUGO, NULL,
				       omap_pm_read_proc, NULL);
}

#endif /* DEBUG && CONFIG_PROC_FS */

static void (*saved_idle)(void) = NULL;

static int omap_pm_prepare(void)
{
	/* We cannot sleep in idle until we have resumed */
	saved_idle = pm_idle;
	pm_idle = NULL;

	return 0;
}



static int omap_pm_enter(suspend_state_t state)
{
	switch (state)
	{
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		omap1_pm_suspend();
		break;
	default:
		return -EINVAL;
	}

	return 0;
}



static void omap_pm_finish(void)
{
	pm_idle = saved_idle;
}


static irqreturn_t omap_wakeup_interrupt(int irq, void *dev)
{
	return IRQ_HANDLED;
}

static struct irqaction omap_wakeup_irq = {
	.name		= "peripheral wakeup",
	.flags		= IRQF_DISABLED,
	.handler	= omap_wakeup_interrupt
};



static struct platform_suspend_ops omap_pm_ops ={
	.prepare	= omap_pm_prepare,
	.enter		= omap_pm_enter,
	.finish		= omap_pm_finish,
	.valid		= suspend_valid_only_mem,
};

static int __init omap_pm_init(void)
{

#ifdef CONFIG_OMAP_32K_TIMER
	int error;
#endif

	printk("Power Management for TI OMAP.\n");

	/*
	 * We copy the assembler sleep/wakeup routines to SRAM.
	 * These routines need to be in SRAM as that's the only
	 * memory the MPU can see when it wakes up.
	 */
	if (cpu_is_omap7xx()) {
		omap_sram_suspend = omap_sram_push(omap7xx_cpu_suspend,
						   omap7xx_cpu_suspend_sz);
	} else if (cpu_is_omap15xx()) {
		omap_sram_suspend = omap_sram_push(omap1510_cpu_suspend,
						   omap1510_cpu_suspend_sz);
	} else if (cpu_is_omap16xx()) {
		omap_sram_suspend = omap_sram_push(omap1610_cpu_suspend,
						   omap1610_cpu_suspend_sz);
	}

	if (omap_sram_suspend == NULL) {
		printk(KERN_ERR "PM not initialized: Missing SRAM support\n");
		return -ENODEV;
	}

	pm_idle = omap1_pm_idle;

	if (cpu_is_omap7xx())
		setup_irq(INT_7XX_WAKE_UP_REQ, &omap_wakeup_irq);
	else if (cpu_is_omap16xx())
		setup_irq(INT_1610_WAKE_UP_REQ, &omap_wakeup_irq);

	/* Program new power ramp-up time
	 * (0 for most boards since we don't lower voltage when in deep sleep)
	 */
	omap_writew(ULPD_SETUP_ANALOG_CELL_3_VAL, ULPD_SETUP_ANALOG_CELL_3);

	/* Setup ULPD POWER_CTRL_REG - enter deep sleep whenever possible */
	omap_writew(ULPD_POWER_CTRL_REG_VAL, ULPD_POWER_CTRL);

	/* Configure IDLECT3 */
	if (cpu_is_omap7xx())
		omap_writel(OMAP7XX_IDLECT3_VAL, OMAP7XX_IDLECT3);
	else if (cpu_is_omap16xx())
		omap_writel(OMAP1610_IDLECT3_VAL, OMAP1610_IDLECT3);

	suspend_set_ops(&omap_pm_ops);

#if defined(DEBUG) && defined(CONFIG_PROC_FS)
	omap_pm_init_proc();
#endif

#ifdef CONFIG_OMAP_32K_TIMER
	error = sysfs_create_file(power_kobj, &sleep_while_idle_attr.attr);
	if (error)
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
#endif

	if (cpu_is_omap16xx()) {
		/* configure LOW_PWR pin */
		omap_cfg_reg(T20_1610_LOW_PWR);
	}

	return 0;
}
__initcall(omap_pm_init);
