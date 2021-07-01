/**
 * video.c
 */

#include "drk/video.h"
#include "drk/utils.h"

#include <linux/version.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <inttypes.h> 
#include <unistd.h>




#define PREFIX "VIDEO"



/**************
 * Structs
 *************/
typedef struct buffer_t
{
	void*	start;
	__u32	length;
} buffer_t;


/********************
 * Globals
 *******************/
static enum v4l2_buf_type	Buf_type = BUF_TYPE ;
static enum v4l2_memory		Memory_type = V4L2_MEMORY_MMAP ;
static int 					fd; /* file descriptor of the camera */
static buffer_t* 			Buffers ;
static uint8_t 				Buffers_count = 1 ;
//static uint8_t vdebug = 1;

/* save here desired resolution */
static unsigned int H = 0 ;
static unsigned int W = 0 ;

//file to save nav-data
//FILE* nav_file;
//PID control config factor
//float P_factor_i, I_factor_i, D_factor_i, P_factor_j, I_factor_j, D_factor_j;


/* 
 * static prototypes 
 * */
static int 		read_frame( void *p , uint32_t *len ) ;
static int 		setFrameFormat( struct v4l2_fmtdesc *fmtdesc, char camera_id ) ;
//static void 	processImage(const void *p, uint32_t size) ;

/*************************
 * Function definitions
 ************************/
//void errno_exit(const char *s)
//{
  //printf("%s error %d, %s\n", s, errno, strerror(errno));
//}

#if 0
void processImage( const void *p , uint32_t size )
{
	return ;
	static uint16_t frame_id = 0;
	frame_id++ ;
	
	//printf("processing image (%dB)\n", size);
	
	FILE* pFile;
	char flnm[40] ;
	snprintf(flnm, 39 ,"rxshots/frame%03d.uyvy", frame_id ) ;
	pFile = fopen ( flnm , "w") ;
	int ret = fwrite( p, (size_t)size, 1, pFile);
	
	if ( 1 != ret )
		fprintf( stderr , "Failed to write frame\n" ) ;
	fclose(pFile);
}
#endif

int read_frame( void *p , uint32_t *len ) 
{
	#ifdef HIGHDEBUG


	#endif
	struct v4l2_buffer buf ;    
	CLEAR(buf);
	buf.type = Buf_type;
	buf.memory = Memory_type ;

	/* wait to read a frame (dequeue stuff) */
	if (-1 == ioctl( fd, VIDIOC_DQBUF, &buf) )
	{
		perror("VIDIOC_DQBUF");      
		return -1 ;
	}
	#ifdef HIGHDEBUG
	printf("bufindex %d, bytesused %d B\n",
		buf.index, buf.bytesused );
	#endif
	assert( buf.index < Buffers_count ) ;


	*len = buf.bytesused ;
	#if 0 // todo: test this further before use
	/* save into p not the whole frame, but a HxW frame (method: subsample . alternative would be: crop ) */
	if ( W != VIDEO_WIDTH_FRONT )
	{
		
		unsigned int step = 4* ( W / VIDEO_WIDTH_FRONT ) ; 
		PRINTF_FL("Subsampling...W %u , step: %u\n", W,  step ) ;
		unsigned int j = 0 ;
		unsigned int i = 0 ;
		unsigned int h = 0 ;
		for ( h = 0 ; h < VIDEO_HEIGHT_FRONT ; h++ )
		{
			for ( i = 0 ; i < VIDEO_WIDTH_FRONT ; i=i+step )
			{
				/* grab from Buffers 4 bytes , */
				memcpy ( (char*)p+j , (char*)Buffers[buf.index].start + i + h*VIDEO_WIDTH_FRONT*2 , 4 ) ;
				j = j + 4 ; /* ready to save 4 more bytes*/
				
			}
			PRINTF_FL("j %dB , %p\n", j ,(char*)p+j ) ;
		}
		*len = j ;
	}
	#else
	memcpy( p , Buffers[buf.index].start , *len ) ;
	#endif
	//processImage( Buffers[buf.index].start , buf.bytesused );
	
	/* Request a capture of new frame - enqueue a empty buffer */
	if (-1 == ioctl(fd, VIDIOC_QBUF, &buf)) 
	{
		perror("VIDIOC_QBUF");
		return -2;
	}
	return 1 ;
}

/* read <N_FRAMES> frames */
#ifdef MAINLOOP
#define N_FRAMES	200
void mainloop(void)
{
	unsigned int count;
	for (count = N_FRAMES; count > 0; --count)
	{
		while (true)
		{
			microsleep(100000);
			fd_set fds;
			struct timeval tv;
			int r;

			/* add <fd> to the set <fds> */
			FD_ZERO( &fds );
			FD_SET( fd, &fds );

			/* Read with Timeout */
			tv.tv_sec = 5;
			tv.tv_usec = 0;

			/* monitor ("read") file descriptor : fd 
			 * waits here until we have something to read */
			r = select(fd + 1, &fds, NULL, NULL, &tv);

			if (-1 == r)
			{
				if (EINTR == errno)
				  continue;
				errno_exit("select");
			}

			if (0 == r)
			{
				fprintf(stderr, "select timeout\n");
				//exit(EXIT_FAILURE);
			}

			/* read a frame */
			if ( read_frame() )
				break ; 
			
		}
		if ( count % 10 == 0 )
		{
				printf("frames = %d\n" , count ) ; 
				fflush ( stdout ) ;
		}
	}
}
#endif

int grab_frame( void *p , __u32 *len )
{


	fd_set fds;
	struct timeval tv;
	int r;

	/* add <fd> to the set <fds> */
	FD_ZERO( &fds );
	FD_SET( fd, &fds );

	/* Read with Timeout */
	tv.tv_sec = 5;
	tv.tv_usec = 0;

	/* monitor ("read") file descriptor : fd 
	 * waits here until we have something to read */
	r = select(fd + 1, &fds, NULL, NULL, &tv);

	if (-1 == r)
	{
		perror("select");
		if (EINTR == errno)
			return -1 ;
		return -2 ;
	}

	if (0 == r)
	{
		fprintf(stderr, "select timeout\n");
		return -3 ;
	}

	/* read a frame , and write to p */
	if ( read_frame( p , len ) < 0)
		return -4 ; 


	return 1 ;
}


int start_capturing(void)
{
	struct v4l2_buffer buf ;
	for (__u32 i = 0; i < Buffers_count; i++)
	{
		CLEAR(buf);
		buf.type = Buf_type ;
		buf.memory = Memory_type ;
		buf.index = i ;
		/* start grabing frames - enqueue ! */
		if (-1 == ioctl(fd, VIDIOC_QBUF, &buf))
		{
			perror("VIDIOC_QBUF");
			return -1 ;
		}
		//printf("Qbuffer[#%" PRIu32 "] - OK\n", i );
	}
  

	if ( -1 == ioctl(fd, VIDIOC_STREAMON, &Buf_type))
	{
		PRINTF_FL_ERR("VIDIOC_STREAMON: %s\n", strerror( errno) );
		return -2 ;
	}
	
	return 1 ;
}

int init_mmap(void)
{
	struct v4l2_requestbuffers req;

	CLEAR(req);

	req.count = Buffers_count ; // 4 
	req.type = Buf_type ; // V4L2_BUF_TYPE_VIDEO_CAPTURE
	req.memory =  Memory_type; // V4L2_MEMORY_USERPTR

	if (-1 == ioctl(fd, VIDIOC_REQBUFS, &req))
	{
		if ( EINVAL == errno )
		{
			PRINTF_FL_ERR( "Device does not support memory mapping");
			return -1 ;
		}
		else
		{
			PRINTF_FL_ERR("VIDIOC_REQBUFS");
			return -2 ;
		}
	}

	Buffers_count = req.count ;
	//PRINTF_FL("VIDIOC_REQBUFS - Done \t Total bufs =%d\n", Buffers_count ) ;

	//if ( Buffers_count < 2 )
	//{
		//perror(  "Insufficient buffer memory on the device\n");
		//return -3 ;
	//}

	Buffers = calloc( req.count , sizeof(buffer_t) );
	if ( Buffers == NULL)
	{
		PRINTF_FL_ERR( "Out of memory\n");
		return -4 ;
	}

	for (__u32 i = 0; i < Buffers_count; i++)
	{
		struct v4l2_buffer buf ;
		CLEAR(buf);

		buf.type    = req.type ;
		buf.memory  = req.memory ;
		buf.index   = i ;

		if ( -1 == ioctl(fd, VIDIOC_QUERYBUF, &buf) )
		{
			PRINTF_FL_ERR("VIDIOC_QUERYBUF");
			return -5 ;
		}

		
		
		Buffers[i].length = buf.length;
		Buffers[i].start = 
			(char*)mmap(
				NULL /* start anywhere */,
				buf.length,
				PROT_READ | PROT_WRITE /* required */,
				MAP_SHARED /* recommended */,
				fd , 
				buf.m.offset
				);
	
		//PRINTF_FL("buf[%d] length %dB, @<%p>\n",
			//i,
			//Buffers[i].length , 
			//Buffers[i].start ) ;
				
		if ( MAP_FAILED == Buffers[i].start )
		{
			PRINTF_FL_ERR("mmap");
			return -6 ;
		}
	}

	//PRINTF_FL("Whole MMAP initiated with success\n"  );
	return 1 ;
}

int open_camera( char F_or_B )
{
	char camera_device[20] ;
	switch ( F_or_B )
	{
		case 'F' :
			strncpy( camera_device , CAMERA_DEVICE_FRONT , 20 );
			break ;
		case 'B' :
			strncpy( camera_device , CAMERA_DEVICE_BOTTOM , 20 ) ;
			break ; 
		default :
			return -1 ;
	}
	// **Open Camera device**:
	fd = open( camera_device ,
		O_RDWR | /* mmap needs W persmissions  */
		O_NONBLOCK , 
		0 ) ; 
		
	if ( fd < 0 )
	{
		PRINTF_FL_ERR("Open %s failed - %s\n", camera_device , strerror( errno ) ) ;
		return -1 ;
	}
	
	PRINTF_FL("Camera %s - open success\n",camera_device);
	return 1 ;
}

int setFrameFormat( struct v4l2_fmtdesc *fmtdesc , char camera_id )
{
	 /* format used here should be what ENUM_FMT returs to fmtdesc */
	
	
	struct v4l2_format fmt ;  
	CLEAR(fmt);
	
	switch (camera_id)
	{
		case 'F':
			fmt.fmt.pix.width = VIDEO_WIDTH_FRONT ; 
			fmt.fmt.pix.height = VIDEO_HEIGHT_FRONT ;
			break;
		case 'B' :
			fmt.fmt.pix.width = VIDEO_WIDTH_BOTTOM ; 
			fmt.fmt.pix.height = VIDEO_HEIGHT_BOTTOM ;
			break;
		default:
			return 1 ;
	}
	
	if ( fmtdesc == NULL )
	{
		fmt.fmt.pix.pixelformat = VIDEO_FORMAT ; 
		fmt.type = Buf_type ;
	}
	else
	{
		fmt.type = fmtdesc->type ;
		fmt.fmt.pix.pixelformat = fmtdesc->pixelformat ;
		//fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB ;
		//fmt.fmt.pix.field = V4L2_FIELD_NONE ; V4L2_FIELD_TOP;  V4L2_FIELD_ANY, V4L2_FIELD_INTERLACED; 
	}

	if ( ioctl(fd, VIDIOC_S_FMT, &fmt )  < 0)
	{
		PRINTF_FL_ERR("S_FMT failed (%s)\n", strerror(errno));
		return -1 ;
	}
	else
	{
		//PRINTF_FL("S_FMT OK - Camera set up.\n");
	}

	/* *Get actual Stream Format */
	if ( ioctl(fd, VIDIOC_G_FMT, &fmt ) < 0)
	{
		PRINTF_FL_ERR("VIDIOC_G_FMT failed (%s)\n", strerror( errno) ) ;
		return -1 ;
	}


	//printf("Stream Format Information (G_FMT):\n");
	//printf(" type: %d (1=V4L2_BUF_TYPE_VIDEO_CAPTURE, 2=V4L2_BUF_TYPE_VIDEO_OUTPUT)\n", fmt.type); //http://linuxtv.org/downloads/v4l-dvb-apis/buffer.html#v4l2-buf-type
	//printf(" width: %d px\n", fmt.fmt.pix.width);
	//printf(" height: %d px\n", fmt.fmt.pix.height);
	//printf(" framesize w.h = %d px\n", fmt.fmt.pix.width*fmt.fmt.pix.height);
	///* build string */
	//printf(" pixelformat: %4s\n", (char*)&fmt.fmt.pix.pixelformat );
	//printf(" bytesperline: %d B \n", fmt.fmt.pix.bytesperline);
	//printf(" sizeimage: %0.1f KB\n", fmt.fmt.pix.sizeimage/1000.0);
	//printf(" colorspace: %d (8 = V4L2_COLORSPACE_SRGB )\n", fmt.fmt.pix.colorspace); //0 if there is no color, I suppose. cf. grey
	//printf(" field: %d (1= V4L2_FIELD_NONE , 8 = V4L2_FIELD_INTERLACED_TB)\n", fmt.fmt.pix.field);

	//printf("-------Frame Format -------------\n");
	//printf("\n") ;
	
	return 1 ;
}

int drk_video_front_init( unsigned int h, unsigned int w ) 
{
	if ( h==0 || w == 0 )
	{
		H = VIDEO_HEIGHT_FRONT ;
		W = VIDEO_WIDTH_FRONT ;
	}
	else
	{
		H = h ;
		W = w ;
	}
	PRINTF_FL("%ux%u px\n", H , W ) ;
	return drk_video_init( 'F' ) ;
}

int drk_video_bottom_init( unsigned int h, unsigned int w ) 
{
	if (h==0 || w == 0)
	{
		H = VIDEO_HEIGHT_BOTTOM ;
		W = VIDEO_WIDTH_BOTTOM ;
	}
	else
	{
		H = h ;
		W = w ;
	}
	PRINTF_FL("%ux%u px\n", H , W ) ;
	return drk_video_init( 'B' ) ;
}

int drk_video_close(void)
{
	return close( fd ) ;
}
int drk_video_init( char cam_id )
{
	
	char camera_id = cam_id ;
	if ( open_camera( camera_id ) < 0 )
		return -1 ;
		
	struct v4l2_capability cap;
  


		// **Query Capability**:
		//The VIDIOC_QUERYCAP ioctl is available to check if the kernel device is compatible with this specification, and to query the functions and I/O methods supported by the device.
		
	CLEAR(cap) ;
	if ( ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0 )
	{
		PRINTF_FL_ERR("VIDIOC_QUERYCAP failed (%s)\n" , strerror( errno ) );
		return -1 ;
	}

	

	//printf( "VIDIOC_QUERYCAP success: capabilities 0x%x\n", cap.capabilities); 
	//printf( "Driver:\t%s\n" ,cap.driver ) ;
	//printf( "Card:\t%s\n" ,cap.card ) ; 
	//printf( "Bus info:\t%s\n" ,cap.bus_info ) ; 
	////printf( "Flags:\t0x%x\n" ,cap.flags ) ; 
	//char ans[] = "_:_" ;
	//printf( "Dev Caps:\t%" PRIx32 "\n" , cap.device_caps ) ; 
	
	//memcpy( ans , (cap.capabilities & V4L2_CAP_STREAMING) ? "yes" : "noo" , 3) ; 	
	//printf( "Device Caps:\tStreaming? %s\n"  , ans ) ;

	//memcpy( ans , (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) ? "YES" : "noo" , 3) ; 	
	//printf( "Device Caps:\tVideo Capture? %s\n"  , ans ) ;
	

	//printf("-----Device -----------\n\n");
	
	


	if ( (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0 ) 
		return -1 ;
  //0x04000002 = V4L2_CAP_STREAMING & V4L2_CAP_VIDEO_OUTPUT
  //0x04000001 = V4L2_CAP_STREAMING & V4L2_CAP_VIDEO_CAPTURE
  
  //supports the STREAMING I/O method + VIDEO OUTPUT interface (or. VIDEO_CAPTURE)
  //http://linuxtv.org/downloads/v4l-dvb-apis/vidioc-querycap.html

	//printf("-------Frame Format -------------\n");
	struct v4l2_fmtdesc fmtdesc ;
	CLEAR(fmtdesc);
	fmtdesc.index = 0 ;
	fmtdesc.type = Buf_type  ;
	
	
	if ( ioctl( fd , VIDIOC_ENUM_FMT , &fmtdesc ) <0 )
	{
		if ( errno == EINVAL )
		{
			PRINTF_FL_WARN("VIDIOC_ENUM_FMT failed (%s) \n", 
				strerror(errno) );
			//exit(1); 
		}
		else
		{
			PRINTF_FL_ERR("VIDIOC_ENUM_FMT failed (%s)\n", 
				strerror(errno) ) ;
			exit(1); 
		}
	}

	/* lets enumerate the Formats */
	char px_fmt[5] ;
	px_fmt[4]='\0' ;
	memcpy( px_fmt, (void*)&fmtdesc.pixelformat , 4 ) ;
	//printf("V4L2_BUF_TYPE_VIDEO_CAPTURE \t%s (id %s)\n",
		//fmtdesc.description ,
		//px_fmt );

	//printf("------------\n\n");
	//Buf_type = fmtdesc.type  ; /* Global */
	
	


	/* image formats tested that do not work: */
	//uint32_t desired_pix_format = V4L2_PIX_FMT_RGB*; // none RGB format works 
	//uint32_t desired_pix_format = V4L2_PIX_FMT_YUV420; // does not work
	//uint32_t desired_pix_format = V4L2_PIX_FMT_YUV32; // does not work

	/* list that works: */
	//uint32_t desired_pix_format = fmtdesc.pixelformat ; // V4L2_PIX_FMT_YUYV YUV 4:2:2 
	//uint32_t desired_pix_format = V4L2_PIX_FMT_GREY; // Grey-scale

	/* Define our picture format */
	//setFrameFormat( &fmtdesc , camera_id );
	if ( setFrameFormat( NULL , camera_id ) < 0 )
		return -1 ;
	
	//PRINTF_FL_WARN("Setframeformat done\n");


	//init buffers array, and map the device to each of the buffers:
	if ( init_mmap() < 0 )
		return -1 ;
	
	//PRINTF_FL_WARN("init mmap done\n");
	
	/* create file */
	//FILE * pFile = fopen ("myfile.yuv","w");
	//fclose( pFile ) ;
	
	if( start_capturing() < 0 )
		return -1 ;
	
	return 1 ;
	// mainloop();
	
	
	
}
