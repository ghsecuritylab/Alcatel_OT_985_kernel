Fully working Alcatel_OT_985 kernel source.
Usage:
1. cd /OT_985/kernel
2. export PATH=/path_to_your_toolchain/android_prebuilt_toolchains/arm-eabi-4.4.3/bin/:$PATH
3. export TARGET_PRODUCT=jrd73_gb
4. ./build.sh

Known bugs:
1. In /kernel/drivers/char the consolemap_deftbl.c the first array's size became a huge negative or a huge positive number. If negative the build fails, but if positive the build fails just at the and with final link error. Fix: If the build does not fail at the genarating of consolmap_deftbl.c, than stop building with Ctrl+c and modify the /kernel/drivers/char/consolemap_deftbl.c file's firs array size to [256].
2. Notify led turned off, because when it turn on the flashlight also turn on and phone became hot while charging.
3. Camera button not works.
