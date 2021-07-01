/*********
 JPEG 
 compress and decompress
*/ 
#define IMG_PROC_IMPORT

#include "drk/image_process.h"
#include "drk/utils.h"
//#include "drk/video.h"


#include <math.h>
#include <inttypes.h> 
#include <string.h>
//#include <stdio.h>
//#include <stdlib.h>




//#define PI_EIGHTH	(0.392699)

#define PREFIX "IMG_PROC"

#if COMPRESS
uint_fast8_t Max_u = 0, Max_v = 0, Max_uv_dist = 0 ;
uint16_t Quality=0;
#endif

/** *********
 * PRIVATE prototypes
 *******************/
#if COMPRESS
#if METHOD1
static error_t serializeBlock( const int8_t const *in_G, int8_t *const out ) ;
#endif
#endif
 
 #if COMPRESS
 #if METHOD1
/** serialize matrix into 1 array 
 * grab most important cells out of 2D matrix BLOCKSIZExBLOCKSIZE
 * uses Zigzag LUT **/
static error_t serializeBlock( const int8_t const *in_G , int8_t *const out )
{
	//int8_t G[BLOCK_SIZE][BLOCK_SIZE];
	for (uint_fast16_t i = 0; i < Quality ; i++ )
	{
		//out[i] = G[ Zigzag[i][0] ][ Zigzag[i][1] ] ;
		memcpy( 
			out+i , 
			in_G+( Zigzag[i][0]* BLOCK_SIZE ) + Zigzag[i][1], 
			1 ) ;
		
		//printf("G%u%u = %d \n", 
		//zigzag[i][0], zigzag[i][1], G[ zigzag[i][0] -1][ zigzag[i][1] -1]  );
	}
	//printf("\n");
	return E_SUCCESS ;
}
#endif
#endif

#if COMPRESS
/** given quality, 
 * what is the max value of (u+v) to retrive from G[u][v] matrix ? 
 **/
const uint8_t UV_DIST[] ={
	255, /*0 -*/
	0, /*1 = quality*/
	255,/*2*/
	1,/*3 = quality */
	255,/*4*/
	255,/*5*/
	2,/*6 = quality */
	255,/*7*/
	255,/*8*/
	255,/*9*/
	3,/*10 = quality */
	255,/*11*/
	255,/*12*/
	255,/*13*/
	255,/*14*/
	4,/*15 = quality */
	255,/*16*/
	255,/*17*/
	255,/*18*/
	255,/*19*/
	255,/*20*/
	5,/*21 = quality */
	255,/*22*/
	255,/*23*/
	255,/*24*/
	255,/*25*/
	255,/*26*/
	255,/*27*/
	6,/*28 = quality */
};
error_t setup_compress( uint16_t quality )
{
	if (quality>ARRAYSIZE(UV_DIST))
	{
		PRINTF_FL_ERR(
			"invalid value for quality- choose 1,3,6,10,15\n");
		return E_INVALID_INPUT;
	}
	
	if (255 == UV_DIST[quality])
	{
		PRINTF_FL_ERR(
			"invalid value for quality- choose 1,3,6,10,15\n");
		return E_INVALID_INPUT;
	}
	

	
	/* we'll Compute G(u,v) only from 0 to max_u, max_v indexes */
	for ( uint_fast8_t i = 0 ; i < quality ; i++ )
	{
		if ( Zigzag[i][0] > Max_u )
			Max_u = Zigzag[i][0] ;
		if ( Zigzag[i][1] > Max_v )
			Max_v = Zigzag[i][1] ;
	}
	/* and only if u+v, in G(u,v) is less or equal to */
	Max_uv_dist = UV_DIST[quality];
	PRINTF_FL(
		"derivates: u=[0-%"PRIu8"],v=[0-%"PRIu8"], "
		"s.t. u+v <= %"PRIu8"\n", 
		Max_u, 
		Max_v,
		Max_uv_dist);

	Quality = quality ; /* global save */
	return E_SUCCESS ;
}
#endif

#if COMPRESS
#define METHOD2 1 

#define ZERO_THRS 3
/** 
 * Compress 
 * <in> points to an array of a black and white image
 * 1-byte-pixel
 * size: height by width . **/
inline error_t jpeg_compress( const uint8_t *const restrict in, 
	uint16_t img_height, uint16_t img_width, 
	int8_t *const restrict out )
{
		
	uint_fast16_t it = 0 ;
	#if METHOD1
	int8_t B[BLOCK_SIZE*BLOCK_SIZE] ; /* u,v */
	#endif
	//CLEAR(G) ;
	
	//PRINTF_FL_WARN("Size %"PRIu16 "x%"PRIu16" = "
		//"%" PRIu32 "px\n", 
		//img_height, img_width,
		//(uint32_t)img_height * img_width ) ; 
	//int8_t *array = (int8_t*)malloc(Quality) ;
	int8_t Barray[Quality];
	//printf("Blocks: " );
	
	/* double for loop */
	for (
		uint_fast16_t block_ri = 0 ; 
		block_ri <  img_height  ; 
		block_ri += BLOCK_SIZE )
	for (
		uint_fast16_t block_ci = 0 ; 
		block_ci <  img_width  ; 
		block_ci += BLOCK_SIZE )
	{
		//CLEAR(Barray);
		//uint_fast8_t zero_counter=0;
		//double init_time = getEpoch();
		//printf("G(%"PRIu16",%" PRIu16 ")=\n" , block_ri , block_ci ) ;
		for ( uint_fast8_t u = 0 ; u <= Max_u ; u++ )
		{
		for ( uint_fast8_t v = 0 ; v <= Max_v ; v++ )
		{
			if ( u+v > Max_uv_dist )
				continue;
			
			
			float Guv = 0.0 ;
			#if 0
			for ( uint_fast8_t x = 0 ; x < BLOCK_SIZE ; x++ )
			for ( uint_fast8_t y = 0 ; y < BLOCK_SIZE ; y++ )
			{
				//PRINTF_FL("xy %u %u \n", x, y ) ;
				
				
					
				#if 0		
				/* grab one pixel from the block, coords x,y */
				const uint8_t *pointer = in + 
					( (block_ri+x) * img_width + (block_ci+y) ) ;
						
				/* center it to 128.0 */
				const float pixel = (float)(*pointer) - 128.0 ;
				
				/* compute cos*cos multiplication */
				Guv += pixel * 
					KOS_LUT[u][v][x][y] ; //=> cos( PI_EIGHTH * (x + 0.5) * u ) * cos( PI_EIGHTH * (y + 0.5) * v ) ;
				#endif
				
				Guv += ( 
					(float)(*( in + ( (block_ri+x) * img_width + (block_ci+y) )))-/* pixel value */
					128.0 
						) *
					KOS_LUT[u][v][x][y] ;
				
			}
			
			#endif
			
			#if 1
			//const uint8_t *pointer = in + 
					//( (block_ri+x) * img_width + (block_ci+y) ) ;
			#define PIXEL(x,y)	\
						(*(in + ( (block_ri+x) * img_width + (block_ci+y) )))
			
			Guv = 
				cs[u][0]*(
				cos_pix[v][0][PIXEL(0,0)] + 
				cos_pix[v][1][PIXEL(0,1)] + 
				cos_pix[v][2][PIXEL(0,2)] + 
				cos_pix[v][3][PIXEL(0,3)] + 
				cos_pix[v][4][PIXEL(0,4)] + 
				cos_pix[v][5][PIXEL(0,5)] + 
				cos_pix[v][6][PIXEL(0,6)] + 
				cos_pix[v][7][PIXEL(0,7)]
				) + 
				cs[u][1]*(
				cos_pix[v][0][PIXEL(1,0)] + 
				cos_pix[v][1][PIXEL(1,1)] + 
				cos_pix[v][2][PIXEL(1,2)] + 
				cos_pix[v][3][PIXEL(1,3)] + 
				cos_pix[v][4][PIXEL(1,4)] + 
				cos_pix[v][5][PIXEL(1,5)] + 
				cos_pix[v][6][PIXEL(1,6)] + 
				cos_pix[v][7][PIXEL(1,7)]
				) + 
				cs[u][2]*(
				cos_pix[v][0][PIXEL(2,0)] + 
				cos_pix[v][1][PIXEL(2,1)] + 
				cos_pix[v][2][PIXEL(2,2)] + 
				cos_pix[v][3][PIXEL(2,3)] + 
				cos_pix[v][4][PIXEL(2,4)] + 
				cos_pix[v][5][PIXEL(2,5)] + 
				cos_pix[v][6][PIXEL(2,6)] + 
				cos_pix[v][7][PIXEL(2,7)]
				) + 
				cs[u][3]*(
				cos_pix[v][0][PIXEL(3,0)] + 
				cos_pix[v][1][PIXEL(3,1)] + 
				cos_pix[v][2][PIXEL(3,2)] + 
				cos_pix[v][3][PIXEL(3,3)] + 
				cos_pix[v][4][PIXEL(3,4)] + 
				cos_pix[v][5][PIXEL(3,5)] + 
				cos_pix[v][6][PIXEL(3,6)] + 
				cos_pix[v][7][PIXEL(3,7)]
				) + 
				cs[u][4]*(
				cos_pix[v][0][PIXEL(4,0)] + 
				cos_pix[v][1][PIXEL(4,1)] + 
				cos_pix[v][2][PIXEL(4,2)] + 
				cos_pix[v][3][PIXEL(4,3)] + 
				cos_pix[v][4][PIXEL(4,4)] + 
				cos_pix[v][5][PIXEL(4,5)] + 
				cos_pix[v][6][PIXEL(4,6)] + 
				cos_pix[v][7][PIXEL(4,7)]
				) + 
				cs[u][5]*(
				cos_pix[v][0][PIXEL(5,0)] + 
				cos_pix[v][1][PIXEL(5,1)] + 
				cos_pix[v][2][PIXEL(5,2)] + 
				cos_pix[v][3][PIXEL(5,3)] + 
				cos_pix[v][4][PIXEL(5,4)] + 
				cos_pix[v][5][PIXEL(5,5)] + 
				cos_pix[v][6][PIXEL(5,6)] + 
				cos_pix[v][7][PIXEL(5,7)]
				) + 
				cs[u][6]*(
				cos_pix[v][0][PIXEL(6,0)] + 
				cos_pix[v][1][PIXEL(6,1)] + 
				cos_pix[v][2][PIXEL(6,2)] + 
				cos_pix[v][3][PIXEL(6,3)] + 
				cos_pix[v][4][PIXEL(6,4)] + 
				cos_pix[v][5][PIXEL(6,5)] + 
				cos_pix[v][6][PIXEL(6,6)] + 
				cos_pix[v][7][PIXEL(6,7)]
				) + 
				cs[u][7]*(
				cos_pix[v][0][PIXEL(7,0)] + 
				cos_pix[v][1][PIXEL(7,1)] + 
				cos_pix[v][2][PIXEL(7,2)] + 
				cos_pix[v][3][PIXEL(7,3)] + 
				cos_pix[v][4][PIXEL(7,4)] + 
				cos_pix[v][5][PIXEL(7,5)] + 
				cos_pix[v][6][PIXEL(7,6)] + 
				cos_pix[v][7][PIXEL(7,7)]
				);

			#endif
			Guv *= aaQ[u][v] ; //Guv *= 0.25*ALPHA(u)*ALPHA(v) / jpegQ[u][v] ;
			
			#if METHOD2
			if (255==Anti_zigzag[u][v] )
			{
				PRINTF_FL_ERR(
					"unexpected u,v = [%d,%d]\n",
					u,v);
				return E_OTHER;
			}
			#endif
			//PRINTF_FL_ERR(
				//"G[%d,%d] => Ar[%d]\n",
				//u,v,
				//Anti_zigzag[u][v]);	
						
			#if METHOD2
			Barray[Anti_zigzag[u][v]] = (int8_t)Guv ;
			#endif
			#if METHOD1
			B[u*BLOCK_SIZE+v] = (int8_t)Guv ;
			#endif
			
			///* after finding some Zeros, skip, 
			 //* and go to next block, to speedup */
			//if (0 == (int8_t)Guv )
				//zero_counter++ ;
			//else
				//zero_counter=0;
				
			//if ( zero_counter > ZERO_THRS )
			//{
				//goto block_done;
			//}
			
			
			//printf(
				//"G[%"PRIu8"][%"PRIu8"] ="
				//" %" PRId8 "\n", 
				//u,v,
				//G[u][v] ) ;
		}
		
		}
		
		//block_done:
		//;

		//init_time = getEpoch();
		
		
		#if METHOD1
		serializeBlock( B , array ) ;
		#endif
		
	
		//PRINTF_FL( 
					//"serialize #%d: %.4fms\n" , 
					//it,
					//1000*(getEpoch() - init_time) ) ;
					
		
		
		//printf("\n");
		//PRINTF_FL_WARN("copin %dB to %p\n", 
			//Quality,
			//(void*)(out+Quality*it));
			
		memcpy( 
			(void*)(1+out+Quality*it), 
			(void*)Barray, 
			Quality );
		it++;
	}
	out[0]=Quality; 
	//PRINTF_FL("nblocks=%" PRIu32  "\n", it ) ;
	//free(array);
	//printf("\n");
	return E_SUCCESS ;
}
#endif




/** **/
error_t Y_subsample( 
	const uint8_t *const in_y, unsigned long in_len, /* in*/
	uint16_t width, uint8_t *const out_sub_y, uint8_t factor ) 
{

	unsigned long  j = 0 ;
	for ( unsigned long  i = 0 ; i < in_len ; i += factor , j++ )
	{
		if ( NULL==memcpy( out_sub_y+j , in_y+i, 1 ) ) 
			return E_NO_MEM ;		
		
		if ( ( (long)i % (long)(factor*width) ) == (long)(width-factor) ) /* 1280-2*/
		{
			i+=(factor-1)*width;
			
		}
			
	}
	
	
	#if 0 /* save gray format */
	int ret ;
	char filename[30];
	snprintf(
		filename, 
		sizeof(filename)-1,
		"image_%lupx.grayraw", j ) ;
	FILE *fd = fopen(filename,"wb");
	ret = (int)fwrite( out_sub_y, j, 1, fd );
	if (ret != 1 )
		PRINTF_FL_ERR("Failed to write file\n");
	fclose( fd ) ; 
	return ret;
	#endif /* save gray */
	
	//printf("all copied\n");
	return E_SUCCESS ;
}

/** **/
error_t UYVY_2_Y( const uint8_t *const in_uyvy, unsigned long len, 
	uint8_t *const out_gray ) 
{

	unsigned long  j = 0 ;
	for ( unsigned long  i = 1 ; i < len ; i = i + 2 )
	{
		if ( NULL==memcpy( out_gray+j , in_uyvy+i ,1 ) ) return -1 ;
		//printf("%d ",j);
		j++;
	}
	//printf("all copied\n");
	
	#if 0 /* save gray format */
	char filename[30];
	snprintf(
		filename, 
		sizeof(filename)-1,
		"image_%lupx.grayraw", j ) ;
	FILE* fd = fopen(filename,"wb");
	int ret = (int)fwrite( out_gray, j ,1,fd );
	if (1 =! ret)
		PRINTF_FL_ERR("Failed to write file\n");
	fclose( fd ) ; 
	return ret ;
	#endif
	
	return E_SUCCESS ;
	
}
