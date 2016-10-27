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

#ifdef CAMERA_DBG
	#define CAMERA_TRACE(x) (printk)x
#else
	#define CAMERA_TRACE(x)
#endif

/*
 * Function definitions
 */
extern struct mxc_v4l_frame * mxc_get_internal_frame(cam_data *cam);
extern struct mxc_v4l_frame * mxc_get_Nth_frame(cam_data *cam,int delta);
int g_dump_flag=0;
static void NXP_dump_ipu_task(struct ipu_task *t)
{
    pr_err("====== ipu task ======\n");
    pr_err("input:\n");
    pr_err("\tforamt: 0x%x\n", t->input.format);
    pr_err("\twidth: %d\n", t->input.width);
    pr_err("\theight: %d\n", t->input.height);
    pr_err("\tcrop.w = %d\n", t->input.crop.w);
    pr_err("\tcrop.h = %d\n", t->input.crop.h);
    pr_err("\tcrop.pos.x = %d\n", t->input.crop.pos.x);
    pr_err("\tcrop.pos.y = %d\n", t->input.crop.pos.y);
    pr_err("\tinput.paddr_n = %x\n", t->input.paddr_n);
    pr_err("\tinput.paddr = %x\n", t->input.paddr);

    if (t->input.deinterlace.enable) {
        pr_err("deinterlace enabled with:\n");
        if (t->input.deinterlace.motion != HIGH_MOTION)
            pr_err("\tlow/medium motion\n");
        else
            pr_err("\thigh motion\n");
    }
    pr_err("output:\n");
    pr_err("\tforamt: 0x%x\n", t->output.format);
    pr_err("\twidth: %d\n", t->output.width);
    pr_err("\theight: %d\n", t->output.height);
    pr_err("\troate: %d\n", t->output.rotate);
    pr_err("\tcrop.w = %d\n", t->output.crop.w);
    pr_err("\tcrop.h = %d\n", t->output.crop.h);
    pr_err("\tcrop.pos.x = %d\n", t->output.crop.pos.x);
    pr_err("\tcrop.pos.y = %d\n", t->output.crop.pos.y);
    pr_err("\toutput.paddr = %x\n", t->output.paddr);
    if (t->overlay_en) {
        pr_err("overlay:\n");
        pr_err("\tforamt: 0x%x\n", t->overlay.format);
        pr_err("\twidth: %d\n", t->overlay.width);
        pr_err("\theight: %d\n", t->overlay.height);
        pr_err("\tcrop.w = %d\n", t->overlay.crop.w);
        pr_err("\tcrop.h = %d\n", t->overlay.crop.h);
        pr_err("\tcrop.pos.x = %d\n", t->overlay.crop.pos.x);
        pr_err("\tcrop.pos.y = %d\n", t->overlay.crop.pos.y);
        if (t->overlay.alpha.mode == IPU_ALPHA_MODE_LOCAL)
            pr_err("combine with local alpha\n");
        else
            pr_err("combine with global alpha %d\n", t->overlay.alpha.gvalue);
        if (t->overlay.colorkey.enable)
            pr_err("colorkey enabled with 0x%x\n", t->overlay.colorkey.value);
    }
}

static void csi_buf_work_func(struct work_struct *work)
{
	int err;
	struct mxc_v4l_frame *done_frame;
	struct mxc_v4l_frame *frame;
	struct ipu_task	task;

	cam_data *cam =
			container_of(work, struct _cam_data, csi_work_struct);

	if (!list_empty(&cam->working_q) &&  cam->frame_delay == 0)
	{
		done_frame = list_entry(cam->working_q.next,
				struct mxc_v4l_frame,
				queue);

		memset(&task, 0, sizeof(task));
		/* Grab one frame from work queue queue */
		task.output.paddr = done_frame->buffer.m.offset;

		frame = mxc_get_Nth_frame(cam,-2);
		task.input.paddr = frame->buffer.m.offset;

		task.input.deinterlace.enable = false;

		task.input.width = CAM_WIDTH;
		task.input.height = CAM_HEIGHT;
		task.input.format = V4L2_PIX_FMT_UYVY;

		task.output.rotate = 0;
		task.output.width = CAM_WIDTH_NEW;
		task.output.height = CAM_HEIGHT;
		task.output.format = V4L2_PIX_FMT_UYVY;

		/* Parameter validation */
		err = ipu_check_task(&task);
		if (err != IPU_CHECK_OK) {
			pr_err("%s: ipu_check_task failed\n", __func__);
		} else {
			if(g_dump_flag<3){
				NXP_dump_ipu_task(&task);
				g_dump_flag++;
			}
			err = ipu_queue_task(&task);
			if (err < 0)
				pr_err("queue ipu task error\n");
			/* Update required information in application buffer */

		}
	}

	if ( cam->frame_delay > 0 )
		cam->frame_delay--;

	cam->enc_callback(0, cam);
	cam->frame_out_cnt++;
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

	//cam->enc_callback(irq, dev_id);
	schedule_work(&cam->csi_work_struct);
	
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
	int width = CAM_WIDTH;
	int bytesperline = width*2;

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

#if 1
	pr_err("NXP Debug pixel_fmt 0x%x \n ",pixel_fmt);
	pr_err("NXP Debug cam->v2f.fmt.pix.width %d \n ",cam->v2f.fmt.pix.width);
	pr_err("NXP Debug cam->v2f.fmt.pix.bytesperline %d \n ",cam->v2f.fmt.pix.bytesperline);
	pr_err("NXP Debug cam->offset.u_offset %d, cam->offset.v_offset %d \n ",cam->offset.u_offset, cam->offset.v_offset);
	pr_err("NXP Debug Handle stride \n");
	err = ipu_init_channel_buffer(cam->ipu,
				      CSI_MEM,
				      IPU_OUTPUT_BUFFER,
				      pixel_fmt, width,
				      cam->v2f.fmt.pix.height,
				      bytesperline,
				      IPU_ROTATE_NONE,
				      dummy, dummy, 0,
				      cam->offset.u_offset,
				      cam->offset.v_offset);
#else
	err = ipu_init_channel_buffer(cam->ipu, CSI_MEM, IPU_OUTPUT_BUFFER,
				      pixel_fmt, cam->v2f.fmt.pix.width,
				      cam->v2f.fmt.pix.height,
				      cam->v2f.fmt.pix.bytesperline,
				      IPU_ROTATE_NONE,
				      dummy, dummy, 0,
				      cam->offset.u_offset,
				      cam->offset.v_offset);
#endif
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
	g_dump_flag =0;
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

	/* Add a work queue */
	pr_err("NXP debug Register csi_buf_work_func \n");
	INIT_WORK(&cam->csi_work_struct, csi_buf_work_func);

	ipu_clear_irq(cam->ipu, IPU_IRQ_CSI0_OUT_EOF);
	err = ipu_request_irq(cam->ipu, IPU_IRQ_CSI0_OUT_EOF,
			      csi_enc_callback, 0, "Mxc Camera", cam);
	if (err != 0) {
		printk(KERN_ERR "Error registering rot irq\n");
		return err;
	}

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
	
	pr_err("NXP Debug %s cancle csi_buf_work_func \n", __func__);
	flush_work(&cam->csi_work_struct);
	cancel_work_sync(&cam->csi_work_struct);

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
