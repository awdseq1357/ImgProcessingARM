/////////////////////////////////////////////////////////////////////////////////////////
//V4L2.h
//符合V4L2协议的摄像机的接口函数
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
#ifndef		__V4L2_H__
#define		__V4L2_H__

#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <malloc.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <time.h>
#include "./qsLib/qsImgLib.h"
#include "./qsLib/avilib.h"

//图像宽度和高度(2592, 1944), (1600, 1200), (640, 480)
#define IMAGE_WIDTH						1600	
#define IMAGE_HEIGHT					1200

//图像采集方式
#define PIXEL_FORMAT					V4L2_PIX_FMT_YUYV
//#define PIXEL_FORMAT					V4L2_PIX_FMT_MJPEG

#ifdef __cplusplus
extern "C" {
#endif

//图像缓冲区结构体
typedef	struct
{
	void				*start;					//起始地址
	size_t				length;					//缓冲区大小
}image_buffer;

//符合V4L2协议的摄像机对象的结构体
typedef	struct
{
	//用户交互变量
	char*				dev_name;				//设备名称/dev/vidio*
	int					snap_mode;				//是否抓拍当前帧图像，1抓拍，其他值不抓拍
	int					avi_record_mode;		//是否录像，1录像，其他值不录像
	
	//自己管理的变量及缓存
	int					fd;						//设备句柄
	int					n_buffers;				//视频缓冲区的数量
	image_buffer*		buffers;				//图像缓冲区的地址
	int					frame_index;			//采集到的帧的序号
	avi_t*				avifile;				//avi录像对象
	int					failure_count;			//连续采集失败的次数,，用于判定相机是否故障
	
	unsigned char*		framebuffer;			//mjpeg临时缓冲区，存放解码得到的yuyv数据
	unsigned char*		prvious_buffer_yuyv;	//mjpeg之前采集到的缓存，用于判定相机是否故障
}V4L2_data;

/////////////////////////////////////////////////////////////////////////////////////////
//打开并初始化摄像头设备，启动视频采集
//打开成功返回1，打开失败返回小于0的错误代码
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int init_dev (V4L2_data *camera);

/////////////////////////////////////////////////////////////////////////////////////////
//停止摄像头设备
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void stop_dev (V4L2_data *camera);

/////////////////////////////////////////////////////////////////////////////////////////
//读取帧图像
//读取成功返回1，读取失败返回0
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int read_frame(V4L2_data *camera, 
				unsigned char *img_8u3_rgb888, 				/*输出的RGB888数据*/
				unsigned char *img_8u1_gray = NULL);		/*输出的灰度图像数据*/


#ifdef __cplusplus
}
#endif


#endif
