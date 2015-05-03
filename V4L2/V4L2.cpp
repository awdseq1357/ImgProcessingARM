/////////////////////////////////////////////////////////////////////////////////////////
//V4L2.cpp
//����V4L2Э���������Ľӿں���
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
#include "V4L2.h"

#define RECORD_SINGLE_FILE_FRAMES 			2000
#define MAX_FAILURE_COUNT					20

/////////////////////////////////////////////////////////////////////////////////////////
//������ͷ�豸�ļ�
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int open_device(V4L2_data *camera)
{
	if (-1 == (camera->fd = open (camera->dev_name, O_RDWR | O_NONBLOCK, 0)))
		return -1;

	return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////
//��ʼ������ͷ�豸
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int init_device(V4L2_data* camera)
{
	//��ѯ�豸�����Ĺ���
	v4l2_capability cap;
	if (-1 == ioctl (camera->fd, VIDIOC_QUERYCAP, &cap))
		return -11;
	//����֧����Ƶ���������˳�
	if (! (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
		return -12;
	if (! (cap.capabilities & V4L2_CAP_STREAMING))
		return -13;
		
	//������Ƶ�豸����Ƶ���ݸ�ʽ
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
//������Ƶ������
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int init_mmap(V4L2_data *camera)
{
	//����V4L2����������Ƶ������������V4L2��Ƶ���������ڴ棩
	v4l2_requestbuffers req;
	memset(&req, 0, sizeof(v4l2_requestbuffers));
	req.count	=	2;									//����Ļ������ĸ���
	req.type	=	V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory	=	V4L2_MEMORY_MMAP;					//mmap��ʽ
	if(-1 == ioctl (camera->fd, VIDIOC_REQBUFS, &req))
		return -21;
	//��ֻ���뵽1����û�����뵽���������򷵻�
	if(req.count < 2)
		return -22;

	//���ں˿ռ��ַӳ�䵽�û��ռ䣬����Ӧ�ó���ķ���
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
			
		//Ͷ�ſյ���Ƶ����������Ƶ���������������
		if (-1 == ioctl (camera->fd, VIDIOC_QBUF, &v4l2_buf))
			return -25;
	}
	
	return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////
//������Ƶ�ɼ�
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int start_capturing(V4L2_data *camera)
{
	//������Ƶ�ɼ�
	enum v4l2_buf_type type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == ioctl (camera->fd, VIDIOC_STREAMON, &type))
		return -31;

	return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////
//�򿪲���ʼ������ͷ�豸��������Ƶ�ɼ�
//�򿪳ɹ�����1����ʧ�ܷ���С��0�Ĵ������
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int init_dev (V4L2_data *camera)
{
	//���ɼ���ͼ����MJPEG��ʽ����Ҫ���ٻ���
	camera->frame_index = 0;
	if(V4L2_PIX_FMT_MJPEG == PIXEL_FORMAT)
	{
		camera->framebuffer = (unsigned char*) calloc(IMAGE_WIDTH*(IMAGE_HEIGHT+8)*2, sizeof(unsigned char));
		camera->prvious_buffer_yuyv = (unsigned char*) calloc(IMAGE_WIDTH*IMAGE_HEIGHT*2, sizeof(unsigned char));
	}
	
	camera->avifile = NULL;
	camera->failure_count = 0;
	
	//������ͷ�豸
	int error_code = open_device(camera);
	if(error_code < 0 )
		return error_code;
	
	//���÷ֱ��ʵȲ���
	error_code = init_device(camera);
	if(error_code < 0 )
		return error_code;

	//���仺��
	error_code = init_mmap(camera);
	if(error_code < 0 )
		return error_code;

	//������Ƶ�ɼ�
	error_code = start_capturing(camera);
	if(error_code < 0 )
		return error_code;

	return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////
//ֹͣ����ͷ�豸
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void stop_dev (V4L2_data *camera)
{
	//ֹͣ��Ƶ�ɼ�
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ioctl (camera->fd, VIDIOC_STREAMOFF, &type);

	//�ͷŷ���Ŀռ�
	for (int i = 0; i < camera->n_buffers; ++i)
		munmap(camera->buffers[i].start, camera->buffers[i].length);
		
	//�ͷ�MJPEG����
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
	
	//�ر�����ͷ
	close(camera->fd);
}

/////////////////////////////////////////////////////////////////////////////////////////
//�ж�����Ƿ�����˹���
//������img_8u2_yuyv1��������ͼ��YUYV
//      img_8u2_yuyv2��������ͼ��YUYV
//      img_width����ͼ����
//      img_height����ͼ��߶�
//����ֵ����0�������ֹ���
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
//������������ͷ�豸
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
void restart_dev (V4L2_data *camera)
{
	//ֹͣ��Ƶ�ɼ�
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ioctl (camera->fd, VIDIOC_STREAMOFF, &type);

	//�ͷŷ���Ŀռ�
	for (int i = 0; i < camera->n_buffers; ++i)
		munmap(camera->buffers[i].start, camera->buffers[i].length);
	
	//�ر�����ͷ
	close(camera->fd);
	
	//��������ͷ
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
//��ȡ֡ͼ��
//��ȡ�ɹ�����1����ȡʧ�ܷ���0
//Author: EmbedVision
/////////////////////////////////////////////////////////////////////////////////////////
int read_frame(V4L2_data *camera, 
				unsigned char *img_8u3_rgb888, 				/*�����RGB888����*/
				unsigned char *img_8u1_gray)				/*����ĻҶ�ͼ������*/
{
	//��ǰ�������Ƶ���ݵĻ�����
	v4l2_buffer			current_buf;			
	memset(&current_buf, 0, sizeof(v4l2_buffer));
	current_buf.type	=	V4L2_BUF_TYPE_VIDEO_CAPTURE;
	current_buf.memory	= 	V4L2_MEMORY_MMAP;
	ioctl (camera->fd, VIDIOC_DQBUF, &current_buf);
	
	//��������ַ
	unsigned char *img_buffer = (unsigned char *)(camera->buffers[current_buf.index].start);
	
	//���ɼ���ͼ����YUYV��ʽ
	if(V4L2_PIX_FMT_YUYV == PIXEL_FORMAT)
	{
		if(current_buf.bytesused != IMAGE_WIDTH*IMAGE_HEIGHT*2)
		{
			//�ͷŵ�ǰ��ȡͼ��Ļ���
			printf("yuyv size Error! \n");
			camera->failure_count++;
			ioctl(camera->fd, VIDIOC_QBUF, &current_buf);
			
			//����γ���yuyv size Error��˵������ͷ�����˹��ϣ���������ͷ
			if(camera->failure_count > MAX_FAILURE_COUNT)
			{
				printf("yuyv capturing error, restart! \n");
				restart_dev(camera);
			}
			return 0;
		}
		camera->failure_count = 0;
	
		//ͼ���ʽת����YUV----RGB
		if(NULL != img_8u3_rgb888)
		{
			qsYuv2Rgb(
				img_buffer, 
				img_8u3_rgb888, 
				IMAGE_WIDTH, 
				IMAGE_HEIGHT);
		}
	
		//ͼ���ʽת����YUV----GRAY
		if(NULL != img_8u1_gray)
		{
			qsYuv2Gray(
				img_buffer, 
				img_8u1_gray, 
				IMAGE_WIDTH, 
				IMAGE_HEIGHT);
		}
		
		//���浱ǰ֡ͼ��
		if(1 == camera->snap_mode)
		{
			char file_name[1024];
			
			if(NULL != img_8u3_rgb888)
			{
				sprintf(file_name, "%d_rgb.bmp", camera->frame_index);
				qsBmpRgbSave(img_8u3_rgb888, IMAGE_WIDTH, IMAGE_HEIGHT, file_name, 1);
			}
			
			if(NULL != img_8u1_gray)
			{
				sprintf(file_name, "%d_gray.bmp", camera->frame_index);
				qsBmpGraySave(img_8u1_gray, IMAGE_WIDTH, IMAGE_HEIGHT, file_name, 1);
			}
			
			camera->snap_mode = 0;
		}
	}
	
	//���ɼ���ͼ����MJPEG��ʽ
	if(V4L2_PIX_FMT_MJPEG == PIXEL_FORMAT)
	{
		//���浱ǰ֡ͼ��
		if(1 == camera->snap_mode)
		{
			char file_name[1024];
			sprintf(file_name, "%d.jpg", camera->frame_index);
			qsJpegSave(img_buffer, current_buf.bytesused, file_name);
			camera->snap_mode = 0;
		}
		
		//avi¼��
		if(1 == camera->avi_record_mode)
		{
			if(NULL == camera->avifile)
			{
				//�õ����õ��ļ���
				char file_name[1024];
				for(int i=1; i<10000; i++)
				{
					sprintf(file_name, "/udisk/qs%.4d.avi", i);
					FILE *fp;
					fp = fopen(file_name,"rb");
					if(NULL == fp)
						break;
					fclose(fp);
				}
				
				camera->avifile = AVI_open_output_file(file_name);
				if(NULL != camera->avifile)
					AVI_set_video(camera->avifile, IMAGE_WIDTH, IMAGE_HEIGHT, 15, (char*)("MJPG"));
			}
			else
			{
				AVI_write_frame(camera->avifile, (char*)img_buffer, current_buf.bytesused, camera->frame_index % RECORD_SINGLE_FILE_FRAMES);
			}
			
			//¼��RECORD_SINGLE_FILE_FRAMES֡������ļ�
			if(RECORD_SINGLE_FILE_FRAMES-1 == camera->frame_index % RECORD_SINGLE_FILE_FRAMES)
			{
				AVI_close(camera->avifile);
				camera->avifile = NULL;
			}
		}
		
		//��ѹ��
		int img_width = 0, img_height = 0;
		qsJpegDecode(&camera->framebuffer, img_buffer, &img_width, &img_height);
		if(IMAGE_WIDTH != img_width || IMAGE_HEIGHT != img_height)
		{
			//�ͷŵ�ǰ��ȡͼ��Ļ���
			printf("qsJpegDecode Error! \n");
			camera->failure_count++;
			ioctl(camera->fd, VIDIOC_QBUF, &current_buf);
			
			//����γ���qsJpegDecode Error��˵������ͷ�����˹��ϣ���������ͷ
			if(camera->failure_count > MAX_FAILURE_COUNT)
			{
				printf("Jpeg capturing Error, restart! \n");
				restart_dev(camera);
			}
			return 0;
		}
		
		//ͼ���ʽת����YUV----RGB
		if(NULL != img_8u3_rgb888)
		{
			qsYuv2Rgb(
				camera->framebuffer, 
				img_8u3_rgb888, 
				IMAGE_WIDTH, 
				IMAGE_HEIGHT);
		}
		
		//ͼ���ʽת����YUV----GRAY
		if(NULL != img_8u1_gray)
		{
			qsYuv2Gray(
				camera->framebuffer, 
				img_8u1_gray, 
				IMAGE_WIDTH, 
				IMAGE_HEIGHT);
		}
		
		//�ж�����Ƿ�����˹���
		if( IsBreakdown(camera->framebuffer, camera->prvious_buffer_yuyv, IMAGE_WIDTH, IMAGE_HEIGHT) )
		{
			camera->failure_count++;
			if(camera->failure_count > MAX_FAILURE_COUNT)
			{
				//�ͷŵ�ǰ��ȡͼ��Ļ���
				printf("image died, restart! \n");
				ioctl(camera->fd, VIDIOC_QBUF, &current_buf);
				
				restart_dev(camera);
				return 0;
			}
		}
		memcpy(camera->prvious_buffer_yuyv, camera->framebuffer, IMAGE_WIDTH*IMAGE_HEIGHT*2);
		
		camera->failure_count = 0;
	}
	
	//�ͷŵ�ǰ��ȡͼ��Ļ���
	ioctl(camera->fd, VIDIOC_QBUF, &current_buf);
	
	camera->frame_index++;
	
	return 1;
}

