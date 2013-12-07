#ifndef __ASM_ARCH_NUC900_KEYPAD_H
#define __ASM_ARCH_NUC900_KEYPAD_H

#include <linux/input/matrix_keypad.h>

extern void mfp_set_groupi(struct device *dev);

struct nuc900_keypad_platform_data {
	const struct matrix_keymap_data *keymap_data;

	unsigned int	prescale;
	unsigned int	debounce;
};

#endif /* __ASM_ARCH_NUC900_KEYPAD_H */
