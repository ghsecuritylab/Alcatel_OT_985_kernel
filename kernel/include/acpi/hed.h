

#ifndef ACPI_HED_H
#define ACPI_HED_H

#include <linux/notifier.h>

int register_acpi_hed_notifier(struct notifier_block *nb);
void unregister_acpi_hed_notifier(struct notifier_block *nb);

#endif
