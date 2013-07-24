/*
 *
 * Intel Management Engine Interface (Intel MEI) Linux driver
 * Copyright (c) 2003-2012, Intel Corporation.

 * Based on: videobuf2-dma-sg.c
 * Author: Andrzej Pietrasiewicz <andrzej.p@samsung.com>
 *  Copyright (C) 2010 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>

#include "dma-sg.h"

struct mei_dma_sg_buf {
	void				*vaddr;
	struct page			**pages;
	int				write;
	int				offset;
	struct mei_dma_sg_desc		sg_desc;
	atomic_t			refcount;
};

void *mei_dma_sg_get_userptr(unsigned long vaddr, unsigned long size, int write)
{
	struct mei_dma_sg_buf *buf;
	unsigned long first, last;
	int num_pages_from_user, i;

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return NULL;

	buf->vaddr = NULL;
	buf->write = write;
	buf->offset = vaddr & ~PAGE_MASK;
	buf->sg_desc.size = size;

	first = (vaddr           & PAGE_MASK) >> PAGE_SHIFT;
	last  = ((vaddr + size - 1) & PAGE_MASK) >> PAGE_SHIFT;
	buf->sg_desc.num_pages = last - first + 1;

	buf->sg_desc.sglist = vzalloc(
		buf->sg_desc.num_pages * sizeof(*buf->sg_desc.sglist));
	if (!buf->sg_desc.sglist)
		goto userptr_fail_sglist_alloc;

	sg_init_table(buf->sg_desc.sglist, buf->sg_desc.num_pages);

	buf->pages = kzalloc(buf->sg_desc.num_pages * sizeof(struct page *),
			     GFP_KERNEL);
	if (!buf->pages)
		goto userptr_fail_pages_array_alloc;

	num_pages_from_user = get_user_pages(current, current->mm,
					     vaddr & PAGE_MASK,
					     buf->sg_desc.num_pages,
					     write,
					     1, /* force */
					     buf->pages,
					     NULL);

	if (num_pages_from_user != buf->sg_desc.num_pages)
		goto userptr_fail_get_user_pages;

	sg_set_page(&buf->sg_desc.sglist[0], buf->pages[0],
		    PAGE_SIZE - buf->offset, buf->offset);
	size -= PAGE_SIZE - buf->offset;
	for (i = 1; i < buf->sg_desc.num_pages; ++i) {
		sg_set_page(&buf->sg_desc.sglist[i], buf->pages[i],
			    min_t(size_t, PAGE_SIZE, size), 0);
		size -= min_t(size_t, PAGE_SIZE, size);
	}

	return buf;

userptr_fail_get_user_pages:
	pr_debug("get_user_pages requested/got: %d/%d]\n",
	       num_pages_from_user, buf->sg_desc.num_pages);
	while (--num_pages_from_user >= 0)
		put_page(buf->pages[num_pages_from_user]);
	kfree(buf->pages);

userptr_fail_pages_array_alloc:
	vfree(buf->sg_desc.sglist);

userptr_fail_sglist_alloc:
	kfree(buf);
	return NULL;
}

/*
 * @put_userptr: inform the allocator that a USERPTR buffer will no longer
 *		 be used
 */
void mei_dma_sg_put_userptr(void *buf_priv)
{
	struct mei_dma_sg_buf *buf = buf_priv;
	int i = buf->sg_desc.num_pages;

	pr_debug("Releasing userspace buffer of %d pages\n",
			buf->sg_desc.num_pages);
	if (buf->vaddr)
		vm_unmap_ram(buf->vaddr, buf->sg_desc.num_pages);
	while (--i >= 0) {
		if (buf->write)
			set_page_dirty_lock(buf->pages[i]);
		put_page(buf->pages[i]);
	}
	vfree(buf->sg_desc.sglist);
	kfree(buf->pages);
	kfree(buf);
}

int mei_sg_buf_prepare(struct device *dev, void *buf_priv)
{
	struct mei_dma_sg_buf *buf = buf_priv;
	struct mei_dma_sg_desc *sgd = &buf->sg_desc;
	int nent;

	nent = dma_map_sg(dev, sgd->sglist, sgd->num_pages, DMA_BIDIRECTIONAL);
	if (nent <= 0)
		return -EIO;
	return nent;
}

int mei_sg_buf_to_sgl(struct device *dev, void *buf_priv,
		     struct mei_sgl_entry *e, int cnt)
{
	struct mei_dma_sg_buf *buf = buf_priv;
	struct mei_dma_sg_desc *sgd = &buf->sg_desc;
	struct scatterlist *sg;
	size_t segment_len;
	int i = 0;

	for_each_sg(sgd->sglist, sg, cnt, i) {
		dma_addr_t dma_addr = sg_dma_address(sg);
		segment_len = sg_dma_len(sg);
		e->lo = lower_32_bits(dma_addr);
		e->hi = upper_32_bits(dma_addr);
		e->sz = sg_dma_len(sg);
		dev_dbg(dev, "sg : %ld, %zd\n",
			(unsigned long)dma_addr, segment_len);
		e++;
	}
	return 0;
}

int mei_sg_buf_finish(struct device *dev, void *buf_priv)
{
	struct mei_dma_sg_buf *buf = buf_priv;
	struct mei_dma_sg_desc *sgd = &buf->sg_desc;
	dma_unmap_sg(dev, sgd->sglist, sgd->num_pages, DMA_BIDIRECTIONAL);
	dev_dbg(dev, "sg :  unmap");
	return 0;
}
