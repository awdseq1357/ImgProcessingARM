/////////////////////////////////////////////////////////////////////////////////////////
//V4L2.cpp
//符合V4L2协议的摄像机的接口函数
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
#include "V4L2.h"

#define RECORD_SINGLE_FILE_FRAMES 			2000
#define MAX_FAILURE_COUNT					50

/////////////////////////////////////////////////////////////////////////////////////////
//打开摄像头设备文件
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int open_device(V4L2_data *camera)
{
	if (-1 == (camera->fd = open (camera->dev_name, O_RDWR | O_NONBLOCK, 0)))
		return -1;

	return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////
//初始化摄像头设备
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int init_device(V4L2_data* camera)
{
	//查询设备驱动的功能
	v4l2_capability cap;
	if (-1 == ioctl (camera->fd, VIDIOC_QUERYCAP, &cap))
		return -11;
	//若不支持视频捕获功能则退出
	if (! (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
		return -12;
	if (! (cap.capabilities & V4L2_CAP_STREAMING))
		return -13;
		
	//设置视频设备的视频数据格式
	v4l2_format fmt;
	memset(&fmt, 0, sizeof(v4l2_format));
	fmt.type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width	= IMAGE_WIDTH;
	fmt.fmt.pix.height	= IMAGE_HEIGHT;
	fmt.fmt.pix.pixelformat	= PIXEL_FORMAT;
	fmt.fmt.pix.field	= V4L2_FIELD_INTERLACED;
	if (-1 == ioctl (camera->fd, VIDIOC_S_FMT, &fmt))
		return -14;

	return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////
//分配视频缓冲区
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int init_mmap(V4L2_data *camera)
{
	//请求V4L2驱动分配视频缓冲区（申请V4L2视频驱动分配内存）
	v4l2_requestbuffers req;
	memset(&req, 0, sizeof(v4l2_requestbuffers));
	req.count	=	2;									//申请的缓冲区的个数
	req.type	=	V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory	=	V4L2_MEMORY_MMAP;					//mmap方式
	if(-1 == ioctl (camera->fd, VIDIOC_REQBUFS, &req))
		return -21;
	//若只申请到1个或没有申请到缓冲区，则返回
	if(req.count < 2)
		return -22;

	//将内核空间地址映射到用户空间，方便应用程序的访问
	v4l2_buffer v4l2_buf;
	camera->n_buffers = req.count;
	camera->buffers = (image_buffer *)calloc (camera->n_buffers, sizeof (image_buffer));
	for (int i = 0; i < camera->n_buffers; ++i )
	{
		memset(&v4l2_buf, 0, sizeof(v4l2_buffer));
		v4l2_buf.type	=	V4L2_BUF_TYPE_VIDEO_CAPTURE;
		v4l2_buf.memory	=	V4L2_MEMORY_MMAP;
		v4l2_buf.index	=	i;
		if (-1 == ioctl (camera->fd, VIDIOC_QUERYBUF, &v4l2_buf))
			return -23;

		camera->buffers[i].length	=	v4l2_buf.length;
		camera->buffers[i].start	= mmap(NULL,		/*start anywhere*/
				v4l2_buf.length,
				PROT_READ | PROT_WRITE,
				MAP_SHARED,
				camera->fd,
				v4l2_buf.m.offset);
		if (MAP_FAILED == camera->buffers[i].start)
			return -24;	
			
		//投放空的视频缓冲区到视频缓冲区输入队列中
		if (-1 == ioctl (camera->fd, VIDIOC_QBUF, &v4l2_buf))
			return -25;
	}
	
	return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////
//启动视频采集
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int start_capturing(V4L2_data *camera)
{
	//启动视频采集
	enum v4l2_buf_type type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == ioctl (camera->fd, VIDIOC_STREAMON, &type))
		return -31;

	return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////
//打开并初始化摄像头设备，启动视频采集
//打开成功返回1，打开失败返回小于0的错误代码
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int init_dev (V4L2_data *camera)
{
	//若采集的图像是MJPEG格式，需要开辟缓存
	camera->frame_index = 0;
	if(V4L2_PIX_FMT_MJPEG == PIXEL_FORMAT)
	{
		camera->framebuffer = (unsigned char*) calloc(IMAGE_WIDTH*(IMAGE_HEIGHT+8)*2, sizeof(unsigned char));
		camera->prvious_buffer_yuyv = (unsigned char*) calloc(IMAGE_WIDTH*IMAGE_HEIGHT*2, sizeof(unsigned char));
	}
	
	camera->avifile = NULL;
	camera->failure_count = 0;
	
	//打开摄像头设备
	int error_code = open_device(camera);
	if(error_code < 0 )
		return error_code;
	
	//设置分辨率等参数
	error_code = init_device(camera);
	if(error_code < 0 )
		return error_code;

	//分配缓存
	error_code = init_mmap(camera);
	if(error_code < 0 )
		return error_code;

	//启动视频采集
	error_code = start_capturing(camera);
	if(error_code < 0 )
		return error_code;

	return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////
//停止摄像头设备
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void stop_dev (V4L2_data *camera)
{
	//停止视频采集
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ioctl (camera->fd, VIDIOC_STREAMOFF, &type);

	//释放分配的空间
	for (int i = 0; i < camera->n_buffers; ++i)
		munmap(camera->buffers[i].start, camera->buffers[i].length);
		
	//释放MJPEG缓存
	if(V4L2_PIX_FMT_MJPEG == PIXEL_FORMAT)
	{
		free(camera->framebuffer);
		free(camera->prvious_buffer_yuyv);
	}
	
	if(NULL != camera->avifile)
	{
		AVI_close(camera->avifile);
		camera->avifile = NULL;
	}
	
	//关闭摄像头
	close(camera->fd);
}

/////////////////////////////////////////////////////////////////////////////////////////
//判定相机是否出现了故障
//参数：img_8u2_yuyv1——输入图像YUYV
//      img_8u2_yuyv2——输入图像YUYV
//      img_width——图像宽度
//      img_height——图像高度
//返回值：非0表明出现故障
/////////////////////////////////////////////////////////////////////////////////////////
int IsBreakdown(const unsigned char *img_8u2_yuyv1,
				const unsigned char *img_8u2_yuyv2,
				const int img_width, 
				const int img_height)
{
	int yuyv_size = img_width * img_height * 2;
	int offset;
	for(offset = 0; offset < yuyv_size; offset ++) 
	{
		if(img_8u2_yuyv1[offset] != img_8u2_yuyv2[offset])
			return 0;
	}
	
	return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////
//重新启动摄像头设备
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void restart_dev (V4L2_data *camera)
{
	//停止视频采集
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ioctl (camera->fd, VIDIOC_STREAMOFF, &type);

	//释放分配的空间
	for (int i = 0; i < camera->n_buffers; ++i)
		munmap(camera->buffers[i].start, camera->buffers[i].length);
	
	//关闭摄像头
	close(camera->fd);
	
	//启动摄像头
	if(open_device(camera) < 0 )
		return;
	if(init_device(camera) < 0 )
		return;
	if(init_mmap(camera) < 0 )
		return;
	start_capturing(camera);
	
	camera->failure_count = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
//读取帧图像
//读取成功返回1，读取失败返回0
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int read_frame(V4L2_data *camera,
                unsigned char *img_8u2_yuyv)				/*输出的灰度图像数据*/
{
	//当前拍摄的视频数据的缓冲区
	v4l2_buffer			current_buf;			
	memset(&current_buf, 0, sizeof(v4l2_buffer));
	current_buf.type	=	V4L2_BUF_TYPE_VIDEO_CAPTURE;
	current_buf.memory	= 	V4L2_MEMORY_MMAP;
	ioctl (camera->fd, VIDIOC_DQBUF, &current_buf);
	
	//缓冲区地址
	unsigned char *img_buffer = (unsigned char *)(camera->buffers[current_buf.index].start);
    int image_size = IMAGE_WIDTH*IMAGE_HEIGHT*2;
    //若采集的图像是YUYV格式
    if(current_buf.bytesused != image_size)
    {
        //释放当前读取图像的缓存
        printf("yuyv size Error! \n");
        camera->failure_count++;
        ioctl(camera->fd, VIDIOC_QBUF, &current_buf);

        //若多次出现yuyv size Error，说明摄像头出现了故障，重启摄像头
        if(camera->failure_count > MAX_FAILURE_COUNT)
        {
            printf("yuyv capturing error, restart! \n");
            restart_dev(camera);
        }
        return 0;
    }
    camera->failure_count = 0;


    for(int offset = 0; offset < image_size; offset ++)
        img_8u2_yuyv[offset] = img_buffer[offset];

//    //保存当前帧图像
//    if(1 == camera->snap_mode)
//    {
//        char file_name[1024];

//        if(NULL != img_8u3_yuv)
//        {
//            sprintf(file_name, "%d_rgb.bmp", camera->frame_index);
//            qsBmpRgbSave(img_8u3_yuv, IMAGE_WIDTH, IMAGE_HEIGHT, file_name, 1);
//        }

//        if(NULL != img_8u1_gray)
//        {
//            sprintf(file_name, "%d_gray.bmp", camera->frame_index);
//            qsBmpGraySave(img_8u1_gray, IMAGE_WIDTH, IMAGE_HEIGHT, file_name, 1);
//        }

//        camera->snap_mode = 0;
//    }

	//释放当前读取图像的缓存
	ioctl(camera->fd, VIDIOC_QBUF, &current_buf);
	
	camera->frame_index++;
	
	return 1;
}

