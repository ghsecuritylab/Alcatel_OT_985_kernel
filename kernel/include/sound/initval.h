
#ifndef __SOUND_INITVAL_H
#define __SOUND_INITVAL_H


#define SNDRV_AUTO_PORT		1
#define SNDRV_AUTO_IRQ		0xffff
#define SNDRV_AUTO_DMA		0xffff
#define SNDRV_AUTO_DMA_SIZE	(0x7fffffff)

#define SNDRV_DEFAULT_IDX1	(-1)
#define SNDRV_DEFAULT_STR1	NULL
#define SNDRV_DEFAULT_ENABLE1	1
#define SNDRV_DEFAULT_PORT1	SNDRV_AUTO_PORT
#define SNDRV_DEFAULT_IRQ1	SNDRV_AUTO_IRQ
#define SNDRV_DEFAULT_DMA1	SNDRV_AUTO_DMA
#define SNDRV_DEFAULT_DMA_SIZE1	SNDRV_AUTO_DMA_SIZE
#define SNDRV_DEFAULT_PTR1	SNDRV_DEFAULT_STR1

#define SNDRV_DEFAULT_IDX	{ [0 ... (SNDRV_CARDS-1)] = -1 }
#define SNDRV_DEFAULT_STR	{ [0 ... (SNDRV_CARDS-1)] = NULL }
#define SNDRV_DEFAULT_ENABLE	{ 1, [1 ... (SNDRV_CARDS-1)] = 0 }
#define SNDRV_DEFAULT_ENABLE_PNP { [0 ... (SNDRV_CARDS-1)] = 1 }
#ifdef CONFIG_PNP
#define SNDRV_DEFAULT_ENABLE_ISAPNP SNDRV_DEFAULT_ENABLE_PNP
#else
#define SNDRV_DEFAULT_ENABLE_ISAPNP SNDRV_DEFAULT_ENABLE
#endif
#define SNDRV_DEFAULT_PORT	{ [0 ... (SNDRV_CARDS-1)] = SNDRV_AUTO_PORT }
#define SNDRV_DEFAULT_IRQ	{ [0 ... (SNDRV_CARDS-1)] = SNDRV_AUTO_IRQ }
#define SNDRV_DEFAULT_DMA	{ [0 ... (SNDRV_CARDS-1)] = SNDRV_AUTO_DMA }
#define SNDRV_DEFAULT_DMA_SIZE	{ [0 ... (SNDRV_CARDS-1)] = SNDRV_AUTO_DMA_SIZE }
#define SNDRV_DEFAULT_PTR	SNDRV_DEFAULT_STR

#ifdef SNDRV_LEGACY_FIND_FREE_IRQ
#include <linux/interrupt.h>

static irqreturn_t snd_legacy_empty_irq_handler(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static int snd_legacy_find_free_irq(int *irq_table)
{
	while (*irq_table != -1) {
		if (!request_irq(*irq_table, snd_legacy_empty_irq_handler,
				 IRQF_DISABLED | IRQF_PROBE_SHARED, "ALSA Test IRQ",
				 (void *) irq_table)) {
			free_irq(*irq_table, (void *) irq_table);
			return *irq_table;
		}
		irq_table++;
	}
	return -1;
}
#endif

#ifdef SNDRV_LEGACY_FIND_FREE_DMA
static int snd_legacy_find_free_dma(int *dma_table)
{
	while (*dma_table != -1) {
		if (!request_dma(*dma_table, "ALSA Test DMA")) {
			free_dma(*dma_table);
			return *dma_table;
		}
		dma_table++;
	}
	return -1;
}
#endif

#endif /* __SOUND_INITVAL_H */
