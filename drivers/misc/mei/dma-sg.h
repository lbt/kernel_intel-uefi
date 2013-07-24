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
#ifndef _MEI_DMA_BUF_H_
#define _MEI_DMA_BUF_H_

#include <linux/mm_types.h>
#include <linux/types.h>


struct mei_dma_sg_desc {
	unsigned long		size;
	unsigned int		num_pages;
	struct scatterlist	*sglist;
};

struct mei_sgl_entry {
	u32 lo;
	u32 hi;
	u32 sz;
};

void *mei_dma_sg_get_userptr(unsigned long vaddr,
			unsigned long size, int write);
void mei_dma_sg_put_userptr(void *buf_priv);
int mei_sg_buf_prepare(struct device *dev, void *buf_priv);
int mei_sg_buf_finish(struct device *dev, void *buf_priv);
int mei_sg_buf_to_sgl(struct device *dev, void *buf_priv,
		      struct mei_sgl_entry *e, int cnt);

#endif /* _MEI_DMA_BUF_H_ */
