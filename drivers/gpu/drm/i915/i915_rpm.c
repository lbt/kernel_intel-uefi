/*
 * Copyright 2013 Intel Corporation
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
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Author:
 * Naresh Kumar Kachhi <naresh.kumar.kachhi@intel.com>
 */

#include "i915_drv.h"
#include "i915_reg.h"
#include "intel_drv.h"
#include "drmP.h"
#include <linux/console.h>
#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif

#define RPM_AUTOSUSPEND_DELAY 500

#define RPM_SYNC 0x1
#define RPM_NO_OP 0x2
#define RPM_AUTOSUSPEND 0x4

#ifdef CONFIG_PM_RUNTIME

/**
 *   Get/put should be used very carefully to avoid any race conditions.
 *   (see the Note below). Main idea is to cover all the acesses that
 *   might result in accessing rings/display/registers/gtt etc .
 *   Mostly covering ioctls and tracking gpu idleness should be enough.
 *
 *   WQ and interrupts should be taken care in suspend path. We should
 *   disable all the interrupts and cancel any pending WQs. Do not
 *   cover interrupt/WQ with get/put unless you are sure about it.
 *
 * Note:Following scenarios should be strictly avoided while using get_sync
 * 1. Calling get_sync with struct_mutex or mode_config.mutex locked
 *    - we acquire these locks in runtime_resume, so any call to get_sync
 *    with these mutex locked might end up in a dead lock.
 *    - Or let's say thread1 has done get_sync and is currently executing
 *    runtime_resume function. Before thread1 is able to acquire these
 *    mutex, thread2 acquires the mutex and does a get_sync. Now thread1
 *    is waiting for mutex and thread2 is waiting for dev->power.lock
 *    resulting in a deadlock.
 * 2. Calling get_sync from runtime_resume path runtime_resume is called
 *    with dev->power.lock held. doing get_sync from this function will
 *    end up in deadlock
 *
 *	Everytime a get/put is added, make sure you to do the testing with
 *  following configs enabled
 *   - CONFIG_LOCKDEP_SUPPORT=y
 *   - CONFIG_PROVE_LOCKING=y
 */

int i915_rpm_init(struct drm_device *drm_dev)
{
	int ret = 0;
	struct device *dev = drm_dev->dev;
	struct drm_i915_private *dev_priv = drm_dev->dev_private;
	ret = pm_runtime_set_active(dev);
	dev_priv->pm.rpm.ring_active = false;
	pm_runtime_allow(dev);
	/* enable Auto Suspend */
	pm_runtime_set_autosuspend_delay(dev, RPM_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(dev);
	if (dev->power.runtime_error)
		DRM_ERROR("rpm init: error = %d\n", dev->power.runtime_error);

	/* Device is expected to call "pm_runtime_put_noidle" to make sure
	 * usage_counter is set to zero after this. In our case Gfx is already
	 * on and will go idle on first power button press (once display goes
	 * idle). So expectations are to do put in display idle.
	 */

	return ret;
}

int i915_rpm_deinit(struct drm_device *drm_dev)
{
	struct device *dev = drm_dev->dev;
	pm_runtime_forbid(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_get_noresume(dev);
	if (dev->power.runtime_error)
		DRM_ERROR("rpm init: error = %d\n", dev->power.runtime_error);

	return 0;
}

/**
 * We have different flavour of get/put based on access type (ring/disp/
 * vxd etc). this is done based on different requirements and to make
 * debugging a little easier. Debugfs introduces seperate counter for
 * each type.
 */

/* wrappers for get/put functions */
int i915_rpm_get(struct drm_device *drm_dev, u32 flags)
{
	/* Note: if device is already on, return value will be 1 */
	if (flags & RPM_SYNC)
		return pm_runtime_get_sync(drm_dev->dev);
	else if (flags & RPM_NO_OP)
		pm_runtime_get_noresume(drm_dev->dev);
	else
		return pm_runtime_get(drm_dev->dev);

	return 0;
}

int i915_rpm_put(struct drm_device *drm_dev, u32 flags)
{
	if (flags & RPM_AUTOSUSPEND) {
		pm_runtime_mark_last_busy(drm_dev->dev);
		return pm_runtime_put_autosuspend(drm_dev->dev);
	} else if (flags & RPM_NO_OP)
		pm_runtime_put_noidle(drm_dev->dev);
	else
		return pm_runtime_put(drm_dev->dev);

	return 0;
}

/**
 * Once we have scheduled commands on GPU, it might take a while GPU
 * to execute them. Following is done to make sure Gfx is in D0i0 while
 * GPU is executing the commands.
 * 1. For IOCTLS make sure we are in D0i0 by calling "get_ioctl".
 * 2. if IOCTL scheudles GPU commands using rings do the following
 *  a. For all ring accesses make sure we add a request in the request
 *     list and schedule a work item to track the "seq no". This
 *     is done by using "i915_add_request" or "i915_add_request_no_flush"
 *     functions.
 *  b. If request list was empty, we do a "get_ring". This will increment
 *     ref count to make sure GPU will be in D0 state.
 *  c. Once the list becomes empty call put_ring
 *
 * Note: All the ring accesses are covered with struct_mutex. So we
 * don't need any synchronization to protect dev_priv->pm.rpm.ring_active.
 */
int i915_rpm_get_ring(struct drm_device *drm_dev)
{
	struct intel_ring_buffer *ring;
	struct drm_i915_private *dev_priv = drm_dev->dev_private;
	int i;
	bool idle = true;

	for_each_ring(ring, dev_priv, i)
		idle &= list_empty(&ring->request_list);

	if (idle) {
		if (!dev_priv->pm.rpm.ring_active) {
			dev_priv->pm.rpm.ring_active = true;
			i915_rpm_get(drm_dev, RPM_NO_OP);
		}
	}

	return 0;
}

int i915_rpm_put_ring(struct drm_device *drm_dev)
{
	struct drm_i915_private *dev_priv = drm_dev->dev_private;
	if (dev_priv->pm.rpm.ring_active) {
		/* Mark last time it was busy and schedule a autosuspend */
		i915_rpm_put(drm_dev, RPM_AUTOSUSPEND);
		dev_priv->pm.rpm.ring_active = false;
	}
	return 0;
}

/**
 * To cover the function pointers that are assigned to drm structures
 * and can be called from drm
 */
int i915_rpm_get_callback(struct drm_device *drm_dev)
{
	return i915_rpm_get(drm_dev, RPM_SYNC);
}

int i915_rpm_put_callback(struct drm_device *drm_dev)
{
	return i915_rpm_put(drm_dev, RPM_AUTOSUSPEND);
}

/**
 * early_suspend/DSR should call this function to notify PM Core about
 * display idleness
 */
int i915_rpm_get_disp(struct drm_device *drm_dev)
{
	return i915_rpm_get(drm_dev, RPM_SYNC);
}

int i915_rpm_put_disp(struct drm_device *drm_dev)
{
	return i915_rpm_put(drm_dev, RPM_AUTOSUSPEND);
}

/** to cover the ioctls with get/put*/
int i915_rpm_get_ioctl(struct drm_device *drm_dev)
{
	/* Don't do anything if device is not ready */
	if (drm_device_is_unplugged(drm_dev))
		return 0;

	return i915_rpm_get(drm_dev, RPM_SYNC);
}

int i915_rpm_put_ioctl(struct drm_device *drm_dev)
{
	/* Don't do anything if device is not ready */
	if (drm_device_is_unplugged(drm_dev))
		return 0;

	return i915_rpm_put(drm_dev, RPM_AUTOSUSPEND);
}

/**
 * VXD driver need to call this to make sure Gfx is in D0i0
 * while VXD is on
 */
#ifdef CONFIG_DRM_VXD_BYT
int i915_rpm_get_vxd(struct drm_device *drm_dev)
{
	return i915_rpm_get(drm_dev, RPM_SYNC);
}
EXPORT_SYMBOL(i915_rpm_get_vxd);

/**
 * VXD driver need to call this to notify Gfx that it is
 * done with HW accesses
 */
int i915_rpm_put_vxd(struct drm_device *drm_dev)
{
	return i915_rpm_put(drm_dev, RPM_AUTOSUSPEND);
}
EXPORT_SYMBOL(i915_rpm_put_vxd);
#endif

#else /*CONFIG_PM_RUNTIME*/
int i915_rpm_init(struct drm_device *dev) {return 0; }
int i915_rpm_deinit(struct drm_device *dev) {return 0; }
int i915_rpm_get_ring(struct drm_device *dev) {return 0; }
int i915_rpm_put_ring(struct drm_device *dev) {return 0; }
int i915_rpm_get_callback(struct drm_device *dev) {return 0; }
int i915_rpm_put_callback(struct drm_device *dev) {return 0; }
int i915_rpm_get_ioctl(struct drm_device *dev) {return 0; }
int i915_rpm_put_ioctl(struct drm_device *dev) {return 0; }
int i915_rpm_get_disp(struct drm_device *dev) {return 0; }
int i915_rpm_put_disp(struct drm_device *dev) {return 0; }
#ifdef CONFIG_DRM_VXD_BYT
int i915_rpm_get_vxd(struct drm_device *dev) {return 0; }
int i915_rpm_put_vxd(struct drm_device *dev) {return 0; }
#endif

#endif /*CONFIG_PM_RUNTIME*/
