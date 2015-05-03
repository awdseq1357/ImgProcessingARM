/////////////////////////////////////////////////////////////////////////////////////////
//V4L2.h
//����V4L2Э���������Ľӿں���
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

//ͼ���Ⱥ͸߶�(2592, 1944), (1600, 1200), (640, 480)
#define IMAGE_WIDTH						1600	
#define IMAGE_HEIGHT					1200

//ͼ��ɼ���ʽ
#define PIXEL_FORMAT					V4L2_PIX_FMT_YUYV
//#define PIXEL_FORMAT					V4L2_PIX_FMT_MJPEG

#ifdef __cplusplus
extern "C" {
#endif

//ͼ�񻺳����ṹ��
typedef	struct
{
	void				*start;					//��ʼ��ַ
	size_t				length;					//��������С
}image_buffer;

//����V4L2Э������������Ľṹ��
typedef	struct
{
	//�û���������
	char*				dev_name;				//�豸����/dev/vidio*
	int					snap_mode;				//�Ƿ�ץ�ĵ�ǰ֡ͼ��1ץ�ģ�����ֵ��ץ��
	int					avi_record_mode;		//�Ƿ�¼��1¼������ֵ��¼��
	
	//�Լ�����ı���������
	int					fd;						//�豸���
	int					n_buffers;				//��Ƶ������������
	image_buffer*		buffers;				//ͼ�񻺳����ĵ�ַ
	int					frame_index;			//�ɼ�����֡�����
	avi_t*				avifile;				//avi¼�����
	int					failure_count;			//�����ɼ�ʧ�ܵĴ���,�������ж�����Ƿ����
	
	unsigned char*		framebuffer;			//mjpeg��ʱ����������Ž���õ���yuyv����
	unsigned char*		prvious_buffer_yuyv;	//mjpeg֮ǰ�ɼ����Ļ��棬�����ж�����Ƿ����
}V4L2_data;

/////////////////////////////////////////////////////////////////////////////////////////
//�򿪲���ʼ������ͷ�豸��������Ƶ�ɼ�
//�򿪳ɹ�����1����ʧ�ܷ���С��0�Ĵ������
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int init_dev (V4L2_data *camera);

/////////////////////////////////////////////////////////////////////////////////////////
//ֹͣ����ͷ�豸
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void stop_dev (V4L2_data *camera);

/////////////////////////////////////////////////////////////////////////////////////////
//��ȡ֡ͼ��
//��ȡ�ɹ�����1����ȡʧ�ܷ���0
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int read_frame(V4L2_data *camera, 
				unsigned char *img_8u3_rgb888, 				/*�����RGB888����*/
				unsigned char *img_8u1_gray = NULL);		/*����ĻҶ�ͼ������*/


#ifdef __cplusplus
}
#endif


#endif
