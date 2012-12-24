/*
 * Driver for the remote control of the TT6400 DVB-S2 card
 *
 * Copyright (C) 2010 Oliver Endriss <o.endriss@gmx.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 * Or, point your browser to http://www.gnu.org/copyleft/gpl.html
 *
 */

#include <linux/types.h>
#include <linux/input.h>

#include "saa716x_spi.h"
#include "saa716x_priv.h"
#include "saa716x_ff.h"


struct ir_keymap {
	u16			keys[128];
};

/* infrared remote control */
struct infrared {
	struct ir_keymap	*key_maps[33];
	struct input_dev	*input_dev;
	char			input_phys[32];
	struct timer_list	keyup_timer;
	struct tasklet_struct	tasklet;
	u32			command;
	u32			device_mask;
	u8			protocol;
	u16			last_key;
	u16			last_toggle;
	bool			delay_timer_finished;
};

#define IR_RC5		0
#define UP_TIMEOUT	(HZ*7/25)


/* key-up timer */
static void ir_emit_keyup(unsigned long parm)
{
	struct infrared *ir = (struct infrared *) parm;

	if (!ir || !test_bit(ir->last_key, ir->input_dev->key))
		return;

	input_report_key(ir->input_dev, ir->last_key, 0);
	input_sync(ir->input_dev);
}

static unsigned int ir_keymap_calculate_index(struct ir_keymap /*const*/ *maps[],
					      size_t addr, size_t data)
{
	unsigned int	res = 0;
	size_t		i;

	BUG_ON(data >= ARRAY_SIZE(maps[0]->keys));
	BUG_ON(maps[addr] == NULL);

	for (i = 0; i+1 < addr; ++i) {
		size_t		j;

		if (maps[i] == NULL)
			continue;

		for (j = 0; j < ARRAY_SIZE(maps[i]->keys); ++j) {
			if (maps[i]->keys[j] != 0)
				++res;
		}
	}

	for (i = 0; i <= data; ++i) {
		if (maps[addr]->keys[i] != 0 || i == data)
			++res;
	}

	return res;
}

static int ir_keymap_find_by_index(struct ir_keymap /*const*/ *maps[],
				   size_t num_maps, unsigned int idx,
				   size_t *addr, size_t *data)
{
	size_t		i;

	++idx;
	for (i = 0; i < num_maps && idx > 0; ++i) {
		size_t	j;

		if (maps[i] == NULL)
			continue;

		for (j = 0; j < ARRAY_SIZE(maps[i]->keys) && idx > 0; ++j) {
			if (maps[i]->keys[j] != 0)
				--idx;
		}

		if (idx == 0) {
			*addr = i;
			*data = j;
			break;
		}
	}

	return (idx == 0) ? 0 : -ENOENT;
}


static int ir_scancode_to_keycode(struct infrared *ir, unsigned int scancode,
				  struct input_keymap_entry *ke)
{
	unsigned int		addr = scancode >> 16;
	unsigned int		data = scancode & 0xffff;
	struct ir_keymap const	*map;

	if (addr >= ARRAY_SIZE(ir->key_maps)) {
		dev_warn(&ir->input_dev->dev,
			 "address in scancode %08x out of range\n", scancode);
		return -EINVAL;
	}

	if (!(ir->device_mask & (1 << addr)))
		return 0;

	map = ir->key_maps[addr];

	if (data >= ARRAY_SIZE(map->keys)) {
		dev_warn(&ir->input_dev->dev,
			 "data in scancode %08x out of range\n", scancode);
		return -EINVAL;
	}

	if (map == NULL) {
		dev_warn(&ir->input_dev->dev,
			 "no keymap for scancode %08x\n", scancode);
		return 0;
	}

	dev_dbg(&ir->input_dev->dev, "map[%u]->keys[%u] => %04x\n",
		addr, data, map->keys[data]);

	if (ke) {
		ke->keycode = map->keys[data];
		ke->len = sizeof scancode;
		ke->index = ir_keymap_calculate_index(ir->key_maps, addr, data);
	}

	return map->keys[data];
}

/* tasklet */
static void ir_emit_key(unsigned long parm)
{
	struct saa716x_dev *saa716x = (struct saa716x_dev *) parm;
	struct infrared *ir = saa716x->ir_priv;
	u32 ircom = ir->command;
	u8 data;
	u8 addr;
	u16 toggle;
	u16 keycode;
	u32 scancode;
	bool send_scancode;

	/* extract device address and data */
	if (ircom & 0x80000000) { /* CEC remote command */
		addr = 0;
		data = ircom & 0x7F;
		toggle = 0;
	} else {
		switch (ir->protocol) {
		case IR_RC5: /* extended RC5: 5 bits device address, 7 bits data */
			addr = (ircom >> 6) & 0x1f;
			/* data bits 1..6 */
			data = ircom & 0x3f;
			/* data bit 7 (inverted) */
			if (!(ircom & 0x1000))
				data |= 0x40;
			toggle = ircom & 0x0800;
			break;

		default:
			printk(KERN_ERR "%s: invalid protocol %x\n",
				__func__, ir->protocol);
			return;
		}
	}

	scancode = (addr << 16) | data;
	keycode = ir_scancode_to_keycode(ir, scancode, NULL);

	dprintk(SAA716x_DEBUG, 0,
		"%s: code %08x -> addr %i data 0x%02x -> keycode %i\n",
		__func__, ircom, addr, data, keycode);

	send_scancode = true;
	/* \todo: there is a race with the 'ir->delay_timer_finished' flag;
	 * when the repeat timer is running and this interrupt routine is
	 * called, the flag is reset by this function but set immediately by
	 * the timer causing a too fast repeat event */
	if (del_timer(&ir->keyup_timer) == 0) {
		/* no timer active yet (e.g. not previously pressed key which
		 * must be handled by sending a release or repeat event; just
		 * reset variables */
	} else if (ir->last_key != keycode || toggle != ir->last_toggle) {
		/* another key was pressed; signal the release event of the
		 * previos one */
		input_event(ir->input_dev, EV_KEY, ir->last_key, 0);
	} else if (ir->delay_timer_finished) {
		/* repeat last key event */
		input_event(ir->input_dev, EV_KEY, keycode, 2);
		send_scancode = false;
	} else {
		/* repeat timer was running but no action was required; do
		 * nothing */
		send_scancode = false;
	}

	if (send_scancode) {
		input_event(ir->input_dev, EV_MSC, MSC_RAW,  ircom);
		input_event(ir->input_dev, EV_MSC, MSC_SCAN, scancode);

		if (keycode > 0) {
			ir->delay_timer_finished = false;
			input_event(ir->input_dev, EV_KEY, keycode, 1);
		} else {
			dev_info(&ir->input_dev->dev,
				 "unknown key %04x (scancode %08x)\n",
				 ircom, scancode);
		}
	}

	input_sync(ir->input_dev);

	if (keycode > 0) {
		ir->last_key = keycode;
		ir->last_toggle = toggle;

		ir->keyup_timer.expires = jiffies + UP_TIMEOUT;
		add_timer(&ir->keyup_timer);
	}
}

static unsigned int ir_ke_scancode(struct input_keymap_entry const *ke)
{
	unsigned int	v = 0;

	if (ke->len != sizeof v) {
		printk(KERN_WARNING "%s: bad length %u of scancode\n",
		       __func__, ke->len);
		return ~0u;
	}

	memcpy(&v, &ke->scancode, sizeof v);
	return v;
}

static int ir_setkeycode(struct input_dev *dev,
			 const struct input_keymap_entry *ke,
			 unsigned int *old_keycode)
{
	struct infrared		*ir = input_get_drvdata(dev);
	unsigned int		scancode = ir_ke_scancode(ke);
	unsigned int		addr = scancode >> 16;
	unsigned int		data = scancode & 0xffff;
	struct ir_keymap	*map;
	unsigned int		old_code;

	if (ke->flags & INPUT_KEYMAP_BY_INDEX) {
		dev_warn(&ir->input_dev->dev,
			 "setkeycode by index not supported\n");
		return -EINVAL;
	}

	if (addr >= ARRAY_SIZE(ir->key_maps)) {
		dev_warn(&ir->input_dev->dev,
			 "address in scancode %08x out of range\n", scancode);
		return -EINVAL;
	}

	map = ir->key_maps[addr];

	if (data >= ARRAY_SIZE(map->keys)) {
		dev_warn(&ir->input_dev->dev,
			 "data in scancode %08x out of range\n", scancode);
		return -EINVAL;
	}

	if (map == NULL) {
		dev_dbg(&ir->input_dev->dev,
			"allocating map for scancode %08x\n", scancode);

		/* we have to use GFP_ATOMIC here because setkeycode is called
		 * with a hold spinlock */
		map = devm_kzalloc(&dev->dev, sizeof *map, GFP_ATOMIC);
		ir->key_maps[addr] = map;
	}

	if (!map)
		return -ENOMEM;

	old_code = map->keys[data];

	if (old_keycode)
		*old_keycode = old_code;

	dev_dbg(&ir->input_dev->dev, "map[%u]->keys[%u] <= %04x\n",
		addr, data, ke->keycode);

	map->keys[data] = ke->keycode;

	__clear_bit(old_code, dev->keybit);
	__set_bit(ke->keycode, dev->keybit);

	return 0;
}

static int ir_getkeycode(struct input_dev *dev,
			 struct input_keymap_entry *ke)
{
	struct infrared		*ir = input_get_drvdata(dev);
	int			rc;

	if (ke->flags & INPUT_KEYMAP_BY_INDEX) {
		size_t		addr;
		size_t		data;

		rc = ir_keymap_find_by_index(ir->key_maps,
					     ARRAY_SIZE(ir->key_maps),
					     ke->index,
					     &addr, &data);


		if (!rc) {
			unsigned int	v = data;

			ke->keycode = ir->key_maps[addr]->keys[data];
			ke->len = sizeof v;
			memcpy(ke->scancode, &v, sizeof v);
		}
	} else {
		unsigned int	scancode = ir_ke_scancode(ke);
		rc = ir_scancode_to_keycode(ir, scancode, ke);
	}

	return rc;
}

/* register with input layer */
static void ir_register_keys(struct infrared *ir)
{
	set_bit(EV_KEY, ir->input_dev->evbit);
	set_bit(EV_REP, ir->input_dev->evbit);
	set_bit(EV_MSC, ir->input_dev->evbit);

	set_bit(MSC_RAW, ir->input_dev->mscbit);
	set_bit(MSC_SCAN, ir->input_dev->mscbit);

	memset(ir->input_dev->keybit, 0, sizeof(ir->input_dev->keybit));

	ir->input_dev->setkeycode = ir_setkeycode;
	ir->input_dev->getkeycode = ir_getkeycode;
}


/* called by the input driver after rep[REP_DELAY] ms */
static void ir_repeat_key(unsigned long parm)
{
	struct infrared *ir = (struct infrared *) parm;

	ir->delay_timer_finished = true;
}


/* interrupt handler */
void saa716x_ir_handler(struct saa716x_dev *saa716x, u32 ir_cmd)
{
	struct infrared *ir = saa716x->ir_priv;

	if (!ir)
		return;

	ir->command = ir_cmd;
	tasklet_schedule(&ir->tasklet);
}


int saa716x_ir_init(struct saa716x_dev *saa716x)
{
	struct input_dev *input_dev;
	struct infrared *ir;
	int rc;

	if (!saa716x)
		return -ENOMEM;

	ir = kzalloc(sizeof(struct infrared), GFP_KERNEL);
	if (!ir)
		return -ENOMEM;

	init_timer(&ir->keyup_timer);
	ir->keyup_timer.function = ir_emit_keyup;
	ir->keyup_timer.data = (unsigned long) ir;

	input_dev = input_allocate_device();
	if (!input_dev)
		goto err;

	ir->input_dev = input_dev;
	input_dev->name = "TT6400 DVB IR receiver";
	snprintf(ir->input_phys, sizeof(ir->input_phys),
		"pci-%s/ir0", pci_name(saa716x->pdev));
	input_dev->phys = ir->input_phys;
	input_dev->id.bustype = BUS_PCI;
	input_dev->id.version = 1;
	input_dev->id.vendor = saa716x->pdev->subsystem_vendor;
	input_dev->id.product = saa716x->pdev->subsystem_device;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 22)
	input_dev->dev.parent = &saa716x->pdev->dev;
#else
	input_dev->cdev.dev = &saa716x->pdev->dev;
#endif
	input_set_drvdata(input_dev, ir);
	rc = input_register_device(input_dev);
	if (rc)
		goto err;

	/* TODO: fix setup/keymap */
	ir->protocol = IR_RC5;
	ir->device_mask = 0xffffffff;
	ir_register_keys(ir);

	/* override repeat timer */
	input_dev->timer.function = ir_repeat_key;
	input_dev->timer.data = (unsigned long) ir;

	tasklet_init(&ir->tasklet, ir_emit_key, (unsigned long) saa716x);
	saa716x->ir_priv = ir;

	return 0;

err:
	if (ir->input_dev)
		input_free_device(ir->input_dev);
	kfree(ir);
	return -ENOMEM;
}


void saa716x_ir_exit(struct saa716x_dev *saa716x)
{
	struct infrared *ir = saa716x->ir_priv;

	saa716x->ir_priv = NULL;
	tasklet_kill(&ir->tasklet);
	del_timer_sync(&ir->keyup_timer);
	input_unregister_device(ir->input_dev);
	kfree(ir);
}
