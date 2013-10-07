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

/**
 * This file is a convenience to avoid throwing a bunch of large tables in other
 * files. It should only be included from intel_ringbuffer.c, where we connect
 * the tables to the appropriate rings.
 */

#define STD_MI_OPCODE_MASK  0xFF800000
#define STD_3D_OPCODE_MASK  0xFFFF0000
#define STD_2D_OPCODE_MASK  0xFFC00000
#define STD_MFX_OPCODE_MASK 0xFFFF0000

#define CMD(op, opm, f, lm, fl, ...)		\
	{					\
		.flags = (fl) | (f),		\
		.cmd = { (op), (opm) },		\
		.length = { (lm) },		\
		__VA_ARGS__			\
	}

/* Convenience macros to compress the tables */
#define SMI STD_MI_OPCODE_MASK
#define S3D STD_3D_OPCODE_MASK
#define S2D STD_2D_OPCODE_MASK
#define SMFX STD_MFX_OPCODE_MASK
#define F CMD_DESC_FIXED
#define S CMD_DESC_SKIP
#define R CMD_DESC_REJECT

/*            Command                          Mask   Fixed Len   Action
	      ---------------------------------------------------------- */
static const struct drm_i915_cmd_descriptor common_cmds[] = {
	CMD(  MI_NOOP,                          SMI,    F,  1,      S  ),
	CMD(  MI_USER_INTERRUPT,                SMI,    F,  1,      S  ),
	CMD(  MI_WAIT_FOR_EVENT,                SMI,    F,  1,      S  ),
	CMD(  MI_ARB_CHECK,                     SMI,    F,  1,      S  ),
	CMD(  MI_REPORT_HEAD,                   SMI,    F,  1,      S  ),
	CMD(  MI_SUSPEND_FLUSH,                 SMI,    F,  1,      S  ),
	CMD(  MI_SEMAPHORE_MBOX,                SMI,   !F,  0xFF,   R  ),
	CMD(  MI_STORE_DWORD_IMM,               SMI,   !F,  0x1FF,  S  ),
	CMD(  MI_STORE_DWORD_INDEX,             SMI,   !F,  0xFF,   R  ),
	CMD(  MI_LOAD_REGISTER_IMM(1),          SMI,   !F,  0xFF,   S  ),
	CMD(  MI_UPDATE_GTT,                    SMI,   !F,  0xFF,   R  ),
	CMD(  MI_STORE_REGISTER_MEM(1),		SMI,   !F,  0xFF,   S  ),
	CMD(  MI_LOAD_REGISTER_MEM,             SMI,   !F,  0xFF,   S  ),
	CMD(  MI_BATCH_BUFFER_START,            SMI,   !F,  0xFF,   S  ),
};

static const struct drm_i915_cmd_descriptor render_cmds[] = {
	CMD(  MI_FLUSH,                         SMI,    F,  1,      S  ),
	CMD(  MI_ARB_ON_OFF,                    SMI,    F,  1,      R  ),
	CMD(  MI_DISPLAY_FLIP,                  SMI,   !F,  0xFF,   R  ),
	CMD(  MI_PREDICATE,                     SMI,    F,  1,      S  ),
	CMD(  MI_TOPOLOGY_FILTER,               SMI,    F,  1,      S  ),
	CMD(  MI_CLFLUSH,                       SMI,   !F,  0x3FF,  S  ),
	CMD(  GFX_OP_3DSTATE_VF_STATISTICS,     S3D,    F,  1,      S  ),
	CMD(  PIPELINE_SELECT,                  S3D,    F,  1,      S  ),
	CMD(  GPGPU_OBJECT,                     S3D,   !F,  0xFF,   S  ),
	CMD(  GPGPU_WALKER,                     S3D,   !F,  0xFF,   S  ),
	CMD(  GFX_OP_3DSTATE_SO_DECL_LIST,      S3D,   !F,  0x1FF,  S  ),
};

static struct drm_i915_cmd_descriptor hsw_render_cmds[] = {
	CMD(  MI_SET_PREDICATE,                 SMI,    F,  1,      S  ),
	CMD(  MI_RS_CONTROL,                    SMI,    F,  1,      S  ),
	CMD(  MI_URB_ATOMIC_ALLOC,              SMI,    F,  1,      S  ),
	CMD(  MI_RS_CONTEXT,                    SMI,    F,  1,      S  ),
	CMD(  MI_MATH,                          SMI,   !F,  0x3F,   S  ),
	CMD(  MI_LOAD_REGISTER_REG,             SMI,    F,  3,      S  ),
	CMD(  MI_LOAD_URB_MEM,                  SMI,   !F,  0xFF,   S  ),
	CMD(  MI_STORE_URB_MEM,                 SMI,   !F,  0xFF,   S  ),
	CMD(  GFX_OP_3DSTATE_DX9_CONSTANTF_VS,  S3D,   !F,  0x7FF,  S  ),
	CMD(  GFX_OP_3DSTATE_DX9_CONSTANTF_PS,  S3D,   !F,  0x7FF,  S  ),
};

static const struct drm_i915_cmd_descriptor video_cmds[] = {
	CMD(  MI_ARB_ON_OFF,                    SMI,    F,  1,      R  ),
	CMD(  MFX_WAIT,                         SMFX,  !F,  0x3F,   S  ),
};

static const struct drm_i915_cmd_descriptor blt_cmds[] = {
	CMD(  MI_DISPLAY_FLIP,                  SMI,   !F,  0xFF,   R  ),
	CMD(  COLOR_BLT,                        S2D,   !F,  0x1F,   S  ),
	CMD(  SRC_COPY_BLT,                     S2D,   !F,  0x1F,   S  ),
};

#undef CMD
#undef SMI
#undef S3D
#undef S2D
#undef SMFX
#undef F
#undef S
#undef R

static const struct drm_i915_cmd_table gen7_render_cmds[] = {
	{ common_cmds, ARRAY_SIZE(common_cmds) },
	{ render_cmds, ARRAY_SIZE(render_cmds) },
};

static struct drm_i915_cmd_table hsw_render_ring_cmds[] = {
	{ common_cmds, ARRAY_SIZE(common_cmds) },
	{ render_cmds, ARRAY_SIZE(render_cmds) },
	{ hsw_render_cmds, ARRAY_SIZE(hsw_render_cmds) },
};

static const struct drm_i915_cmd_table gen7_video_cmds[] = {
	{ common_cmds, ARRAY_SIZE(common_cmds) },
	{ video_cmds, ARRAY_SIZE(video_cmds) },
};

static struct drm_i915_cmd_table hsw_vebox_cmds[] = {
	{ common_cmds, ARRAY_SIZE(common_cmds) },
};

static const struct drm_i915_cmd_table gen7_blt_cmds[] = {
	{ common_cmds, ARRAY_SIZE(common_cmds) },
	{ blt_cmds, ARRAY_SIZE(blt_cmds) },
};
