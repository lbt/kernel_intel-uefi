/*
 * Copyright © 2013 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Brad Volkin <bradley.d.volkin@intel.com>
 *
 */
#include "i915_drv.h"

#define LENGTH_BIAS 2

static const struct drm_i915_cmd_descriptor*
find_cmd_in_table(const struct drm_i915_cmd_table *table,
		  unsigned int cmd_header)
{
	int i;

	for (i = 0; i < table->count; i++) {
		const struct drm_i915_cmd_descriptor *desc = &table->table[i];
		unsigned int masked_cmd = desc->cmd.mask & cmd_header;
		unsigned int masked_value = desc->cmd.value & desc->cmd.mask;

		if (masked_cmd == masked_value)
			return desc;
	}

	return NULL;
}

static const struct drm_i915_cmd_descriptor*
find_cmd(struct intel_ring_buffer *ring, unsigned int cmd_header)
{
	static struct drm_i915_cmd_descriptor default_desc = {
		.flags = CMD_DESC_SKIP
	};

	unsigned int mask;
	int i;

	for (i = 0; i < ring->cmd_table_count; i++) {
		const struct drm_i915_cmd_descriptor *desc;

		desc = find_cmd_in_table(&ring->cmd_tables[i], cmd_header);
		if (desc)
			return desc;
	}

	mask = ring->get_cmd_length_mask(cmd_header);
	if (!mask)
		return NULL;

	default_desc.length.mask = mask;

	return &default_desc;
}

static int valid_reg(const unsigned int *table, int count, unsigned int addr)
{
	if (table && count != 0) {
		int i;

		for (i = 0; i < count; i++) {
			if (table[i] == addr)
				return 1;
		}
	}

	return 0;
}

/* TODO: merge with vmap code for batch copy */
static unsigned int *vmap_batch(struct drm_i915_gem_object *obj)
{
	int i;
	void *addr = NULL;
	struct sg_page_iter sg_iter;
	struct page **pages;

	pages = drm_malloc_ab(obj->base.size >> PAGE_SHIFT, sizeof(*pages));
	if (pages == NULL) {
		DRM_DEBUG("Failed to get space for pages\n");
		goto finish;
	}

	i = 0;
	for_each_sg_page(obj->pages->sgl, &sg_iter, obj->pages->nents, 0) {
		pages[i] = sg_page_iter_page(&sg_iter);
		i++;
	}

	/* XXX: WC mapping used to avoid caching issues on non-LLC platforms */
	addr = vmap(pages, i, VM_MAP, pgprot_writecombine(PAGE_KERNEL));
	if (addr == NULL) {
		DRM_DEBUG("Failed to vmap pages\n");
		goto finish;
	}

finish:
	if (pages)
		drm_free_large(pages);
	return addr;
}

int i915_parse_cmds(struct intel_ring_buffer *ring,
		    struct drm_i915_gem_object *batch_obj,
		    u32 batch_start_offset)
{
	int ret = 0;
	unsigned int *cmd, *batch_base, *batch_end;

	/* Needed because find_cmd() currently uses a static variable and
	 * upstream may rework the locking such that multiple execbuffer2
	 * calls may execute in parallel. We would need to rework find_cmd
	 * at that point.
	 */
	WARN_ON(!mutex_is_locked(&ring->dev->struct_mutex));

	/* No command tables currently indicates a platform without parsing */
	if (!ring->cmd_tables)
		return 0;

	batch_base = vmap_batch(batch_obj);
	if (!batch_base) {
		DRM_DEBUG_DRIVER("CMD: Failed to vmap batch\n");
		return -ENOMEM;
	}

	cmd = batch_base + (batch_start_offset / sizeof(*cmd));
	batch_end = cmd + (batch_obj->base.size / sizeof(*batch_end));

	while (cmd < batch_end) {
		const struct drm_i915_cmd_descriptor *desc;
		unsigned int length;

		if (*cmd == MI_BATCH_BUFFER_END)
			break;

		desc = find_cmd(ring, *cmd);
		if (!desc) {
			DRM_DEBUG_DRIVER("CMD: Unrecognized command: 0x%08X\n",
					 *cmd);
			ret = -EINVAL;
			break;
		}

		if (desc->flags & CMD_DESC_FIXED)
			length = desc->length.fixed;
		else
			length = ((*cmd & desc->length.mask) + LENGTH_BIAS);

		if ((batch_end - cmd) < length) {
			DRM_DEBUG_DRIVER("CMD: Command length exceeds batch length: 0x%08X length=%d batchlen=%ld\n",
					 *cmd,
					 length,
					 batch_end - cmd);
			ret = -EINVAL;
			break;
		}

		if (desc->flags & CMD_DESC_REJECT) {
			DRM_DEBUG_DRIVER("CMD: Rejected command: 0x%08X\n", *cmd);
			ret = -EINVAL;
			break;
		}

		if (desc->flags & CMD_DESC_REGISTER) {
			unsigned int reg_addr =
				cmd[desc->reg.offset] & desc->reg.mask;

			if (!valid_reg(ring->reg_table,
				       ring->reg_count, reg_addr)) {
				DRM_DEBUG_DRIVER("CMD: Rejected register 0x%08X in command: 0x%08X (ring=%d)\n",
						 reg_addr, *cmd, ring->id);
				ret = -EINVAL;
				break;
			}
		}

		cmd += length;
	}

	if (cmd >= batch_end) {
		DRM_DEBUG_DRIVER("CMD: Got to the end of the buffer w/o a BBE cmd!\n");
		ret = -EINVAL;
	}

	vunmap(batch_base);

	return ret;
}
