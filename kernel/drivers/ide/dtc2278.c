

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/blkdev.h>
#include <linux/ide.h>
#include <linux/init.h>

#include <asm/io.h>

#define DRV_NAME "dtc2278"

#undef ALWAYS_SET_DTC2278_PIO_MODE


static void sub22 (char b, char c)
{
	int i;

	for(i = 0; i < 3; ++i) {
		inb(0x3f6);
		outb_p(b,0xb0);
		inb(0x3f6);
		outb_p(c,0xb4);
		inb(0x3f6);
		if(inb(0xb4) == c) {
			outb_p(7,0xb0);
			inb(0x3f6);
			return;	/* success */
		}
	}
}

static DEFINE_SPINLOCK(dtc2278_lock);

static void dtc2278_set_pio_mode(ide_hwif_t *hwif, ide_drive_t *drive)
{
	unsigned long flags;

	if (drive->pio_mode >= XFER_PIO_3) {
		spin_lock_irqsave(&dtc2278_lock, flags);
		/*
		 * This enables PIO mode4 (3?) on the first interface
		 */
		sub22(1,0xc3);
		sub22(0,0xa0);
		spin_unlock_irqrestore(&dtc2278_lock, flags);
	} else {
		/* we don't know how to set it back again.. */
		/* Actually we do - there is a data sheet available for the
		   Winbond but does anyone actually care */
	}
}

static const struct ide_port_ops dtc2278_port_ops = {
	.set_pio_mode		= dtc2278_set_pio_mode,
};

static const struct ide_port_info dtc2278_port_info __initdata = {
	.name			= DRV_NAME,
	.chipset		= ide_dtc2278,
	.port_ops		= &dtc2278_port_ops,
	.host_flags		= IDE_HFLAG_SERIALIZE |
				  IDE_HFLAG_NO_UNMASK_IRQS |
				  IDE_HFLAG_IO_32BIT |
				  /* disallow ->io_32bit changes */
				  IDE_HFLAG_NO_IO_32BIT |
				  IDE_HFLAG_NO_DMA |
				  IDE_HFLAG_DTC2278,
	.pio_mask		= ATA_PIO4,
};

static int __init dtc2278_probe(void)
{
	unsigned long flags;

	local_irq_save(flags);
	/*
	 * This enables the second interface
	 */
	outb_p(4,0xb0);
	inb(0x3f6);
	outb_p(0x20,0xb4);
	inb(0x3f6);
#ifdef ALWAYS_SET_DTC2278_PIO_MODE
	/*
	 * This enables PIO mode4 (3?) on the first interface
	 * and may solve start-up problems for some people.
	 */
	sub22(1,0xc3);
	sub22(0,0xa0);
#endif
	local_irq_restore(flags);

	return ide_legacy_device_add(&dtc2278_port_info, 0);
}

static int probe_dtc2278;

module_param_named(probe, probe_dtc2278, bool, 0);
MODULE_PARM_DESC(probe, "probe for DTC2278xx chipsets");

static int __init dtc2278_init(void)
{
	if (probe_dtc2278 == 0)
		return -ENODEV;

	if (dtc2278_probe()) {
		printk(KERN_ERR "dtc2278: ide interfaces already in use!\n");
		return -EBUSY;
	}
	return 0;
}

module_init(dtc2278_init);

MODULE_AUTHOR("See Local File");
MODULE_DESCRIPTION("support of DTC-2278 VLB IDE chipsets");
MODULE_LICENSE("GPL");
