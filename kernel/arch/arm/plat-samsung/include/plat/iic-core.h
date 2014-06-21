

#ifndef __ASM_ARCH_IIC_CORE_H
#define __ASM_ARCH_IIC_CORE_H __FILE__


/* re-define device name depending on support. */
static inline void s3c_i2c0_setname(char *name)
{
	/* currently this device is always compiled in */
	s3c_device_i2c0.name = name;
}

static inline void s3c_i2c1_setname(char *name)
{
#ifdef CONFIG_S3C_DEV_I2C1
	s3c_device_i2c1.name = name;
#endif
}

static inline void s3c_i2c2_setname(char *name)
{
#ifdef CONFIG_S3C_DEV_I2C2
	s3c_device_i2c2.name = name;
#endif
}

#endif /* __ASM_ARCH_IIC_H */
