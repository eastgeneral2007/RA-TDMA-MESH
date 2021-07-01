/**
 * video.h
 */

#ifndef _VIDEO_H_
#define _VIDEO_H_

#include <semaphore.h>

#include <linux/videodev2.h>


#define VIDEO_FORMAT		V4L2_PIX_FMT_UYVY /* https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/pixfmt-uyvy.html */ 
//#define VIDEO_FORMAT		V4L2_PIX_FMT_YUV420
#define BUF_TYPE			V4L2_BUF_TYPE_VIDEO_CAPTURE
#define CAMERA_DEVICE_BOTTOM "/dev/video2"
#define	VIDEO_WIDTH_BOTTOM	320UL //176  
#define	VIDEO_HEIGHT_BOTTOM	240UL  //144 
#define	BUFFER_COUNT_BOTTOM	20
#define	IMAGE_SIZE_BOTTOM	(VIDEO_WIDTH_BOTTOM*VIDEO_HEIGHT_BOTTOM)
#define	FRAME_SIZE_BOTTOM	(2L*IMAGE_SIZE_BOTTOM)

#define CAMERA_DEVICE_FRONT "/dev/videoC" // "/dev/video1"
//#define IMAGE_SIZE_FRONT  460800
//#define WIDTH_TIMES_HEIGHT_FRONT 307200
#define VIDEO_WIDTH_FRONT 	1280UL//640
#define VIDEO_HEIGHT_FRONT	720UL//480
#define	IMAGE_SIZE_FRONT	(VIDEO_WIDTH_FRONT*VIDEO_HEIGHT_FRONT)
#define	FRAME_SIZE_FRONT	(2UL*IMAGE_SIZE_FRONT)
#define BUFFER_COUNT_FRONT	1


/********* global varibels ***********/

// define buffer 
// used for V4L grab frames
struct fimc_buffer {
	int length;
	void *start;
} framebuf_bottom[BUFFER_COUNT_BOTTOM], framebuf_front[BUFFER_COUNT_FRONT];

// define buffers
// used for transfer buffers
//struct temp_buffers_bottom{
	//char buf[IMAGE_SIZE_BOTTOM];
	//sem_t mutex;	
//};

//struct temp_buffers_front{
	//char buf[IMAGE_SIZE_FRONT];
	//sem_t mutex;	
//};

// for communicationtion
// used for deliver pointers between two processes
//struct post_ptr_bottom{
	//struct temp_buffers_bottom  *pointer;
	//sem_t                 mutex;	
//};

//struct post_ptr_front{
	//struct temp_buffers_front  *pointer;
	//sem_t                 mutex;	
//};

////transmit frames
//struct temp_buffers_bottom *ptr1_bottom;            // buffer1
//struct temp_buffers_bottom *ptr2_bottom;            // buffer2
//struct temp_buffers_bottom *ptr3_bottom;            // buffer3
//struct post_ptr_bottom     *temp_ptr_bottom;        // for delivering pointer

//struct temp_buffers_front *ptr1_front;            // buffer1
//struct temp_buffers_front *ptr2_front;            // buffer2
//struct temp_buffers_front *ptr3_front;            // buffer3
//struct post_ptr_front     *temp_ptr_front;        // for delivering pointer

/********* functions declearation ***********/
int drk_video_close( void ) ;
int drk_video_init( char cam_id ) ;
int drk_video_front_init( unsigned int h, unsigned int w ) ;
int drk_video_bottom_init( unsigned int h, unsigned int w ) ;
int grab_frame( void *p , __u32 *len ) ;
//void drk_PID_control_bottom(unsigned char *ptr);
//void drk_write_ppm_front(unsigned char *ptr, char *ppm_name);
//void drk_write_ppm_bottom(unsigned char *ptr, char *ppm_name);
//void drk_PID_control_old(unsigned char *ptr);
//int drk_PID_set(float P_i, float I_i, float D_i, float P_j, float I_j, float D_j); 



#endif // _VIDEO_H_
