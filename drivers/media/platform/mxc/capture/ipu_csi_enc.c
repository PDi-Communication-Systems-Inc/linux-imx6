/*
 * Copyright 2009-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file ipu_csi_enc.c
 *
 * @brief CSI Use case for video capture
 *
 * @ingroup IPU
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/ipu.h>
#include <linux/mipi_csi2.h>
#include "mxc_v4l2_capture.h"
#include "ipu_prp_sw.h"


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kprobes.h>
#include <linux/ktime.h>
#include <linux/limits.h>
#include <linux/sched.h>
#include <linux/math64.h>

#include <linux/mxcfb.h>
#include <linux/console.h>
static ktime_t start, last;
static s64 total_time;
static unsigned int no_of_frame;
static unsigned int overall_time;
static int irq_start;
static int measure_in_ms;

#ifdef CAMERA_DBG
	#define CAMERA_TRACE(x) (printk)x
#else
	#define CAMERA_TRACE(x)
#endif

static int buffer_num;
static struct ipu_soc *disp_ipu;
static struct fb_info *fbi;
static struct fb_var_screeninfo fbvar;
static u32 vf_out_format;
static u32 ub_in_format;

/*
 * Function definitions
 */

/*
* FPS Measurement APIs
*/
static void dbg_show_in_fps()
{
	s64 temp_time;
	s64 rem;

	if (!no_of_frame)
		return;

	temp_time = no_of_frame;
	if (measure_in_ms)
		temp_time = temp_time * MSEC_PER_SEC;
	else
		temp_time = temp_time * NSEC_PER_SEC;

	rem = div64_ul(temp_time, total_time);

	pr_err("NXP Total F:%d, IN@ %lld fps Total time %lld\n",
			no_of_frame, rem, total_time);
}

static void dbg_measure_in_fps()
{
	s64 actual_time;
	ktime_t end;

	no_of_frame++;
	if (!irq_start) {
		last = start = ktime_get();
		irq_start = 1;
		pr_err("CSI0 IRQ STARTED\n");
		return;
	}
	end = ktime_get();
	if (measure_in_ms) {
		/*Measure in MS*/
		actual_time = ktime_to_ms(ktime_sub(end, last));
		total_time = ktime_to_ms(ktime_sub(end, start));
/*
		pr_err("F:%d, IRQ:@ %u ms\n",
				no_of_frame, (unsigned int)actual_time);
*/
	} else {
		/*Measure in NS*/
		actual_time = ktime_to_ns(ktime_sub(end, last));
		total_time = ktime_to_ns(ktime_sub(end, start));
/*
		pr_err("C==F:%d, IRQ: @ %lld ns\n",
				no_of_frame, (long long)actual_time);
*/
	}

	last = end;
}

static void directly_display(cam_data *cam)
{
	++buffer_num;
	buffer_num %= 3;

	if (ipu_update_channel_buffer(disp_ipu, MEM_FG_SYNC, IPU_INPUT_BUFFER,
			buffer_num, mxc_get_disp_buf(cam, buffer_num)) == 0) {
		ipu_select_buffer(disp_ipu, MEM_FG_SYNC, IPU_INPUT_BUFFER, buffer_num);
	} else {
		pr_err("ERROR:: error in update FG buff%d\n", buffer_num);
	}
}

/*!
 * csi ENC callback function.
 *
 * @param irq       int irq line
 * @param dev_id    void * device id
 *
 * @return status   IRQ_HANDLED for handled
 */
static irqreturn_t csi_enc_callback(int irq, void *dev_id)
{
	cam_data *cam = (cam_data *) dev_id;

	if (cam->enc_callback == NULL)
		return IRQ_HANDLED;

	dbg_measure_in_fps();
	if (cam->usefg == 1) {
		directly_display(cam);

		cam->direct_callback(0, cam);
		return IRQ_HANDLED;
	} else {
		cam->enc_callback(irq, dev_id);
	}
	return IRQ_HANDLED;
}

/*!
 * CSI ENC enable channel setup function
 *
 * @param cam       struct cam_data * mxc capture instance
 *
 * @return  status
 */
static int csi_enc_setup(cam_data *cam)
{
	ipu_channel_params_t params;
	u32 pixel_fmt;
	int err = 0, sensor_protocol = 0;
	dma_addr_t dummy = cam->dummy_frame.buffer.m.offset;
#ifdef CONFIG_MXC_MIPI_CSI2
	void *mipi_csi2_info;
	int ipu_id;
	int csi_id;
#endif

	CAMERA_TRACE("In csi_enc_setup\n");
	if (!cam) {
		printk(KERN_ERR "cam private is NULL\n");
		return -ENXIO;
	}

	memset(&params, 0, sizeof(ipu_channel_params_t));
	params.csi_mem.csi = cam->csi;

	sensor_protocol = ipu_csi_get_sensor_protocol(cam->ipu, cam->csi);
	switch (sensor_protocol) {
	case IPU_CSI_CLK_MODE_GATED_CLK:
	case IPU_CSI_CLK_MODE_NONGATED_CLK:
	case IPU_CSI_CLK_MODE_CCIR656_PROGRESSIVE:
	case IPU_CSI_CLK_MODE_CCIR1120_PROGRESSIVE_DDR:
	case IPU_CSI_CLK_MODE_CCIR1120_PROGRESSIVE_SDR:
		params.csi_mem.interlaced = false;
		break;
	case IPU_CSI_CLK_MODE_CCIR656_INTERLACED:
	case IPU_CSI_CLK_MODE_CCIR1120_INTERLACED_DDR:
	case IPU_CSI_CLK_MODE_CCIR1120_INTERLACED_SDR:
		params.csi_mem.interlaced = true;
		break;
	default:
		printk(KERN_ERR "sensor protocol unsupported\n");
		return -EINVAL;
	}

	if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
		pixel_fmt = IPU_PIX_FMT_YUV420P;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_YVU420)
		pixel_fmt = IPU_PIX_FMT_YVU420P;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV422P)
		pixel_fmt = IPU_PIX_FMT_YUV422P;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY)
		pixel_fmt = IPU_PIX_FMT_UYVY;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
		pixel_fmt = IPU_PIX_FMT_YUYV;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_NV12)
		pixel_fmt = IPU_PIX_FMT_NV12;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_BGR24)
		pixel_fmt = IPU_PIX_FMT_BGR24;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
		pixel_fmt = IPU_PIX_FMT_RGB24;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB565)
		pixel_fmt = IPU_PIX_FMT_RGB565;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_BGR32)
		pixel_fmt = IPU_PIX_FMT_BGR32;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB32)
		pixel_fmt = IPU_PIX_FMT_RGB32;
	else {
		printk(KERN_ERR "format not supported\n");
		return -EINVAL;
	}

#ifdef CONFIG_MXC_MIPI_CSI2
	mipi_csi2_info = mipi_csi2_get_info();

	if (mipi_csi2_info) {
		if (mipi_csi2_get_status(mipi_csi2_info)) {
			ipu_id = mipi_csi2_get_bind_ipu(mipi_csi2_info);
			csi_id = mipi_csi2_get_bind_csi(mipi_csi2_info);

			if (cam->ipu == ipu_get_soc(ipu_id)
				&& cam->csi == csi_id) {
				params.csi_mem.mipi_en = true;
				params.csi_mem.mipi_vc =
				mipi_csi2_get_virtual_channel(mipi_csi2_info);
				params.csi_mem.mipi_id =
				mipi_csi2_get_datatype(mipi_csi2_info);

				mipi_csi2_pixelclk_enable(mipi_csi2_info);
			} else {
				params.csi_mem.mipi_en = false;
				params.csi_mem.mipi_vc = 0;
				params.csi_mem.mipi_id = 0;
			}
		} else {
			params.csi_mem.mipi_en = false;
			params.csi_mem.mipi_vc = 0;
			params.csi_mem.mipi_id = 0;
		}
	}
#endif

	err = ipu_init_channel(cam->ipu, CSI_MEM, &params);
	if (err != 0) {
		printk(KERN_ERR "ipu_init_channel %d\n", err);
		return err;
	}

	ub_in_format = pixel_fmt;
	printk("\nCSI_ENC: IDMAC: Crop width : %d\tCrop height : %d\tPIX FMT : %x\tstride:%d\n",
			UB940_WIDTH,cam->v2f.fmt.pix.height,pixel_fmt, cam->v2f.fmt.pix.bytesperline);

	err = ipu_init_channel_buffer(cam->ipu, CSI_MEM, IPU_OUTPUT_BUFFER,
				      pixel_fmt, UB940_WIDTH/*cam->v2f.fmt.pix.width*/,
				      cam->v2f.fmt.pix.height,
				      cam->v2f.fmt.pix.bytesperline,
				      IPU_ROTATE_NONE,
				      dummy, dummy, 0,
				      cam->offset.u_offset,
				      cam->offset.v_offset);
	if (err != 0) {
		printk(KERN_ERR "CSI_MEM output buffer\n");
		return err;
	}
	err = ipu_enable_channel(cam->ipu, CSI_MEM);
	if (err < 0) {
		printk(KERN_ERR "ipu_enable_channel CSI_MEM\n");
		return err;
	}

	return err;
}

/*!
 * function to update physical buffer address for encorder IDMA channel
 *
 * @param eba         physical buffer address for encorder IDMA channel
 * @param buffer_num  int buffer 0 or buffer 1
 *
 * @return  status
 */
static int csi_enc_eba_update(struct ipu_soc *ipu, dma_addr_t eba,
			      int *buffer_num)
{
	int err = 0;

	pr_debug("eba %x\n", eba);
	err = ipu_update_channel_buffer(ipu, CSI_MEM, IPU_OUTPUT_BUFFER,
					*buffer_num, eba);
	if (err != 0) {
		ipu_clear_buffer_ready(ipu, CSI_MEM, IPU_OUTPUT_BUFFER,
				       *buffer_num);

		err = ipu_update_channel_buffer(ipu, CSI_MEM, IPU_OUTPUT_BUFFER,
						*buffer_num, eba);
		if (err != 0) {
			pr_err("ERROR: v4l2 capture: fail to update "
			       "buf%d\n", *buffer_num);
			return err;
		}
	}

	ipu_select_buffer(ipu, CSI_MEM, IPU_OUTPUT_BUFFER, *buffer_num);

	*buffer_num = (*buffer_num == 0) ? 1 : 0;

	return 0;
}

static void get_disp_ipu(cam_data *cam)
{
		disp_ipu = ipu_get_soc(1); /* using DISP4 */
}

/*!
 * foreground_start - start the vf task
 *
 * @param private    cam_data * mxc v4l2 main structure
 *
 */
static int foreground_start(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0, i = 0, screen_size;
	char *base;
	int bpp;

	if (!cam) {
		printk(KERN_ERR "private is NULL\n");
		return -EIO;
	}

	get_disp_ipu(cam);

	for (i = 0; i < num_registered_fb; i++) {
		char *idstr = registered_fb[i]->fix.id;
		if (((strcmp(idstr, "DISP3 FG") == 0)) ||
		    ((strcmp(idstr, "DISP4 FG") == 0))) {
			fbi = registered_fb[i];
			break;
		}
	}

	if (fbi == NULL) {
		printk(KERN_ERR "DISP FG fb not found\n");
		return -EPERM;
	}

	fbvar = fbi->var;

	/* Store the overlay frame buffer's original std */
	cam->fb_origin_std = fbvar.nonstd;

	if (cam->devtype == IMX5_V4L2 || cam->devtype == IMX6_V4L2) {
		/* Use DP to do CSC so that we can get better performance */
		if(cam->usefg == 1)
			vf_out_format = IPU_PIX_FMT_NV12;
		else
			vf_out_format = IPU_PIX_FMT_UYVY;
		fbvar.nonstd = vf_out_format;
	} else {
		vf_out_format = IPU_PIX_FMT_RGB565;
		fbvar.nonstd = 0;
	}

	if(vf_out_format == IPU_PIX_FMT_NV12)
		bpp = 12;
	else
		bpp = bytes_per_pixel(vf_out_format) * 8;

	fbvar.bits_per_pixel = bpp;
	fbvar.xres = fbvar.xres_virtual = FOR_ANDROID_WIDTH;
	fbvar.yres = 768;//cam->win.w.height;
	fbvar.yres_virtual = 768 * 3;//cam->win.w.height * 3;
	fbvar.yoffset = 0;
	fbvar.vmode &= ~FB_VMODE_YWRAP;
	fbvar.accel_flags = FB_ACCEL_TRIPLE_FLAG;
	fbvar.activate |= FB_ACTIVATE_FORCE;
	fb_set_var(fbi, &fbvar);

	ipu_disp_set_window_pos(disp_ipu, MEM_FG_SYNC, cam->win.w.left,
			cam->win.w.top);

	/* Fill black color for framebuffer */
	base = (char *) fbi->screen_base;
	if (cam->devtype == IMX5_V4L2 || cam->devtype == IMX6_V4L2) {
		for (i = 0; i < fbi->fix.smem_len; i++, base++)
			*base = 0x80;
	} else {
		for (i = 0; i < screen_size * 2; i++, base++)
			*base = 0x00;
	}
	console_lock();
	fb_blank(fbi, FB_BLANK_UNBLANK);
	console_unlock();

	cam->disp_phyaddr_0 = fbi->fix.smem_start + (((fbi->fix.line_length * fbvar.yres)
			+ (fbi->fix.line_length * fbvar.yres)));
	cam->disp_phyaddr_1 = fbi->fix.smem_start + ((fbi->fix.line_length * fbvar.yres));
	cam->disp_phyaddr_2 = fbi->fix.smem_start;

	buffer_num = 2;

	/* correct display ch buffer address */
	ipu_update_channel_buffer(disp_ipu, MEM_FG_SYNC, IPU_INPUT_BUFFER,
			0, cam->disp_phyaddr_0);
	ipu_update_channel_buffer(disp_ipu, MEM_FG_SYNC, IPU_INPUT_BUFFER,
			1, cam->disp_phyaddr_1);
	ipu_update_channel_buffer(disp_ipu, MEM_FG_SYNC, IPU_INPUT_BUFFER,
			2, cam->disp_phyaddr_2);
	return err;

}

/*!
 * foreground_stop - stop the vf task
 *
 * @param private    cam_data * mxc v4l2 main structure
 *
 */
static int foreground_stop(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0, i = 0;
	struct fb_info *fbi = NULL;
	struct fb_var_screeninfo fbvar;

	/*csi_buffer_num = 0;*/
	buffer_num = 0;

	for (i = 0; i < num_registered_fb; i++) {
		char *idstr = registered_fb[i]->fix.id;
		if (((strcmp(idstr, "DISP3 FG") == 0)) ||
		    ((strcmp(idstr, "DISP4 FG") == 0))) {
			fbi = registered_fb[i];
			break;
		}
	}

	if (fbi == NULL) {
		printk(KERN_ERR "DISP FG fb not found\n");
		return -EPERM;
	}

	console_lock();
	fb_blank(fbi, FB_BLANK_POWERDOWN);
	console_unlock();

	/* Set the overlay frame buffer std to what it is used to be */
	fbvar = fbi->var;
	fbvar.accel_flags = FB_ACCEL_TRIPLE_FLAG;
	fbvar.nonstd = cam->fb_origin_std;
	fbvar.activate |= FB_ACTIVATE_FORCE;
	fb_set_var(fbi, &fbvar);
	return err;
}


/*!
 * Enable encoder task
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  status
 */
static int csi_enc_enabling_tasks(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;
	CAMERA_TRACE("IPU:In csi_enc_enabling_tasks\n");

	cam->dummy_frame.vaddress = dma_alloc_coherent(0,
			       PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage),
			       &cam->dummy_frame.paddress,
			       GFP_DMA | GFP_KERNEL);
	if (cam->dummy_frame.vaddress == 0) {
		pr_err("ERROR: v4l2 capture: Allocate dummy frame "
		       "failed.\n");
		return -ENOBUFS;
	}
	cam->dummy_frame.buffer.type = V4L2_BUF_TYPE_PRIVATE;
	cam->dummy_frame.buffer.length =
	    PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage);
	cam->dummy_frame.buffer.m.offset = cam->dummy_frame.paddress;

	ipu_clear_irq(cam->ipu, IPU_IRQ_CSI0_OUT_EOF);
	err = ipu_request_irq(cam->ipu, IPU_IRQ_CSI0_OUT_EOF,
			      csi_enc_callback, 0, "Mxc Camera", cam);
	if (err != 0) {
		printk(KERN_ERR "Error registering rot irq\n");
		return err;
	}

	if(cam->usefg)
		foreground_start(cam);

	err = csi_enc_setup(cam);
	if (err != 0) {
		printk(KERN_ERR "csi_enc_setup %d\n", err);
		return err;
	}

	return err;
}

/*!
 * Disable encoder task
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  int
 */
static int csi_enc_disabling_tasks(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;
#ifdef CONFIG_MXC_MIPI_CSI2
	void *mipi_csi2_info;
	int ipu_id;
	int csi_id;
#endif

	err = ipu_disable_channel(cam->ipu, CSI_MEM, true);

	ipu_uninit_channel(cam->ipu, CSI_MEM);

	if (cam->dummy_frame.vaddress != 0) {
		dma_free_coherent(0, cam->dummy_frame.buffer.length,
				  cam->dummy_frame.vaddress,
				  cam->dummy_frame.paddress);
		cam->dummy_frame.vaddress = 0;
	}

	if(cam->usefg)
		foreground_stop(cam);

#ifdef CONFIG_MXC_MIPI_CSI2
	mipi_csi2_info = mipi_csi2_get_info();

	if (mipi_csi2_info) {
		if (mipi_csi2_get_status(mipi_csi2_info)) {
			ipu_id = mipi_csi2_get_bind_ipu(mipi_csi2_info);
			csi_id = mipi_csi2_get_bind_csi(mipi_csi2_info);

			if (cam->ipu == ipu_get_soc(ipu_id)
				&& cam->csi == csi_id)
				mipi_csi2_pixelclk_disable(mipi_csi2_info);
		}
	}
#endif

	return err;
}

/*!
 * Enable csi
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  status
 */
static int csi_enc_enable_csi(void *private)
{
	cam_data *cam = (cam_data *) private;
	pr_err("+++++++++%s\n", __func__);
	irq_start = 0;
	no_of_frame = 0;
	measure_in_ms = 0;

	return ipu_enable_csi(cam->ipu, cam->csi);
}

/*!
 * Disable csi
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  status
 */
static int csi_enc_disable_csi(void *private)
{
	cam_data *cam = (cam_data *) private;

	/* free csi eof irq firstly.
	 * when disable csi, wait for idmac eof.
	 * it requests eof irq again */
	ipu_free_irq(cam->ipu, IPU_IRQ_CSI0_OUT_EOF, cam);

	pr_err("#####%s\n", __func__);
	dbg_show_in_fps();

	return ipu_disable_csi(cam->ipu, cam->csi);
}

/*!
 * function to select CSI ENC as the working path
 *
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  int
 */
int csi_enc_select(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;

	if (cam) {
		cam->enc_update_eba = csi_enc_eba_update;
		cam->enc_enable = csi_enc_enabling_tasks;
		cam->enc_disable = csi_enc_disabling_tasks;
		cam->enc_enable_csi = csi_enc_enable_csi;
		cam->enc_disable_csi = csi_enc_disable_csi;
	} else {
		err = -EIO;
	}

	return err;
}
EXPORT_SYMBOL(csi_enc_select);

/*!
 * function to de-select CSI ENC as the working path
 *
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  int
 */
int csi_enc_deselect(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;

	if (cam) {
		cam->enc_update_eba = NULL;
		cam->enc_enable = NULL;
		cam->enc_disable = NULL;
		cam->enc_enable_csi = NULL;
		cam->enc_disable_csi = NULL;
	}

	return err;
}
EXPORT_SYMBOL(csi_enc_deselect);

/*!
 * Init the Encorder channels
 *
 * @return  Error code indicating success or failure
 */
__init int csi_enc_init(void)
{
	return 0;
}

/*!
 * Deinit the Encorder channels
 *
 */
void __exit csi_enc_exit(void)
{
}

module_init(csi_enc_init);
module_exit(csi_enc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("CSI ENC Driver");
MODULE_LICENSE("GPL");
