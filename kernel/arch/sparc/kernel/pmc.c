

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <asm/io.h>
#include <asm/oplib.h>
#include <asm/uaccess.h>
#include <asm/auxio.h>


#define PMC_OBPNAME	"SUNW,pmc"
#define PMC_DEVNAME	"pmc"

#define PMC_IDLE_REG	0x00
#define PMC_IDLE_ON	0x01

static u8 __iomem *regs;

#define pmc_readb(offs)		(sbus_readb(regs+offs))
#define pmc_writeb(val, offs)	(sbus_writeb(val, regs+offs))

static void pmc_swift_idle(void)
{
#ifdef PMC_DEBUG_LED
	set_auxio(0x00, AUXIO_LED);
#endif

	pmc_writeb(pmc_readb(PMC_IDLE_REG) | PMC_IDLE_ON, PMC_IDLE_REG);

#ifdef PMC_DEBUG_LED
	set_auxio(AUXIO_LED, 0x00);
#endif
}

static int __devinit pmc_probe(struct of_device *op,
			       const struct of_device_id *match)
{
	regs = of_ioremap(&op->resource[0], 0,
			  resource_size(&op->resource[0]), PMC_OBPNAME);
	if (!regs) {
		printk(KERN_ERR "%s: unable to map registers\n", PMC_DEVNAME);
		return -ENODEV;
	}

#ifndef PMC_NO_IDLE
	/* Assign power management IDLE handler */
	pm_idle = pmc_swift_idle;
#endif

	printk(KERN_INFO "%s: power management initialized\n", PMC_DEVNAME);
	return 0;
}

static struct of_device_id __initdata pmc_match[] = {
	{
		.name = PMC_OBPNAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, pmc_match);

static struct of_platform_driver pmc_driver = {
	.driver = {
		.name = "pmc",
		.owner = THIS_MODULE,
		.of_match_table = pmc_match,
	},
	.probe		= pmc_probe,
};

static int __init pmc_init(void)
{
	return of_register_driver(&pmc_driver, &of_bus_type);
}

__initcall(pmc_init);
