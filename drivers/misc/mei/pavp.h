#ifndef __MEI_PAVP__
#define __MEI_PAVP__

#include "dma-sg.h"

static const uuid_le pavp_uuid = UUID_LE(0xfbf6fcf1, 0x96cf, 0x4e2e,
		0xa6, 0xa6, 0x1b, 0xab, 0x8c, 0xbe, 0x36, 0xb1);

/* support 16 concurrent frames + 1 for metadata and 1 for parsed header */
#define KB(_x)                          (_x << 10)
#define WV_MAX_NUM_SGL                  18
#define WV_MAX_PACKET_SIZE              KB(64)
#define WV_MAX_PACKETS_IN_FRAME         20 /* 20*64K=1280K, max frame size */
#define WV_MAX_NUM_ENTRIES_IN_SGL       (WV_MAX_PACKET_SIZE * \
					 WV_MAX_PACKETS_IN_FRAME/PAGE_SIZE)

struct mei_pavp_header {
	u32 api_version;
	u32 command_id;
	u32 status;
	u32 len;
};

struct mei_pavp_sgl {
	u32 num;
	struct mei_sgl_entry e[0];
};

struct mei_pavp_init_dma_req {
	struct mei_pavp_header hdr;
	union {
		struct mei_pavp_sgl sgl;
		u32 handle;
	};
};

struct mei_pavp_init_dma_res {
	struct mei_pavp_header hdr;
	u32 handle;
};

struct mei_pavp_uninit_dma_req {
	struct mei_pavp_header hdr;
	u32 handle;
};

struct mei_pavp_uninit_dma_rsp {
	struct mei_pavp_header hdr;
};


#define MEI_WV_API_VERSION        0x00010000
#define MEI_WV_INIT_DMA_CMD_ID    0x000A0002
#define MEI_WV_UNINIT_DMA_CMD_ID  0x000A000B

#define MEI_WV_INIT_DMA_REQ_SIZE(cnt) (\
	sizeof(struct mei_pavp_sgl) + \
	(sizeof(struct mei_sgl_entry) * (cnt)))
#define MEI_WV_INIT_DMA_RSP_SIZE() (sizeof(u32))

#define MEI_WV_UNINIT_DMA_REQ_SIZE() (sizeof(u32))
#define MEI_WV_UNINIT_DMA_RES_SIZE() 0

static ssize_t mei_pavp_write(struct mei_cl *cl, struct mei_cl_cb *cb)
{

	struct device *device = &cl->dev->pdev->dev;
	struct mei_pavp_header *hdr;
	size_t length, newlen;
	ssize_t rets;

	hdr = (struct mei_pavp_header *)cb->request_buffer.data;
	length  = cb->request_buffer.size;

	if (length < sizeof(struct mei_pavp_header)) {
		dev_err(device, "pavp buffer too small to parse\n");
		rets = -EINVAL;
		goto out;
	}

	newlen = length;

	dev_dbg(device, "PAVP Command %X\n", hdr->command_id);

	if (hdr->command_id == MEI_WV_INIT_DMA_CMD_ID) {
		struct mei_pavp_init_dma_req *init_dma;
		void *buf;
		int cnt;

		if (length < sizeof(struct mei_pavp_init_dma_req)) {
			dev_err(device, "pavp buffer too small to parse\n");
			rets = -EINVAL;
			goto out;
		}


		init_dma = (struct mei_pavp_init_dma_req *)hdr;
		buf = mei_cl_dmabuf_get(cl, init_dma->handle);
		dev_dbg(device, "PAVP Init DMA %d\n", init_dma->handle);

		if (!buf) {
			dev_err(device, "cannot locate dma buffer\n");
			rets = -EINVAL;
			goto out;
		}

		cnt = mei_cl_dmabuf_map(cl, buf);
		if (cnt < 0) {
			rets = -ENOMEM;
			dev_err(device, "dma mapping failed with status = %d\n",
				cnt);
			goto out;
		}

		newlen = MEI_WV_INIT_DMA_REQ_SIZE(cnt) + sizeof(*hdr);

		init_dma = kmalloc(newlen, GFP_KERNEL);
		if (!init_dma) {
			dev_err(device, "cannot allocate cmd buffer\n");
			rets = -ENOMEM;
			goto out;
		}

		init_dma->hdr.api_version = hdr->api_version;
		init_dma->hdr.command_id  = hdr->command_id;
		init_dma->hdr.status      = 0;
		init_dma->hdr.len         = MEI_WV_INIT_DMA_REQ_SIZE(cnt);
		init_dma->sgl.num         = cnt;
		mei_sg_buf_to_sgl(device, buf, init_dma->sgl.e, cnt);

		cb->request_buffer.data = (unsigned char *)init_dma;
		cb->request_buffer.size = newlen;
		cb->internal = 1;

		kfree(hdr);

		dynamic_hex_dump("init dma: ",
				DUMP_PREFIX_OFFSET, 16, 4,
				cb->request_buffer.data,
				cb->request_buffer.size,
				false);
	} else if (hdr->command_id == MEI_WV_UNINIT_DMA_CMD_ID) {

		if (length < sizeof(struct mei_pavp_uninit_dma_req)) {
			rets = -EINVAL;
			dev_err(device, "pavp buffer too small to parse\n");
			goto out;
		}
		cb->internal = 1;
	}

	rets = mei_cl_write(cl, cb, false);
	/* we have to fake this length */
	if (rets == newlen)
		rets = length;
out:
	return rets;
}

#endif /* __MEI_PAVP__ */
