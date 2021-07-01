
#include <sys/shm.h>
#include <semaphore.h>
#include <errno.h>      /* not sure if needed */
#include <fcntl.h>		/* For O_* constants */

/* standard mex include */
#include "mex.h"


/* -------------------------------------------------------------------------*/
/* Matlab gateway function                                                  */
/*                                                                          */
/* (see semaphore.m for description)               	                       	*/
/* ---------------------------------------------------------------------	*/
/* do not use // for comments 												*/
void mexFunction( int nlhs,       mxArray *plhs[], 
                  int nrhs, const mxArray *prhs[]  )
{
     if( nrhs < 1 )
         mexErrMsgIdAndTxt("MATLAB:semaphore","Minimum input arguments missing; must supply directive");
     
     /* for storing directive (string) input */
     sem_t *clock_semaphore ;
     clock_semaphore = sem_open("clock_semaphore", O_CREAT , 0777, 1 ) ;
     if ( clock_semaphore < 0 )
     	mexErrMsgIdAndTxt("MATLAB:semaphore2","sem open failed");
     
     char *data = (char *)mxGetData(prhs[0]) ;
     switch  (*data )
     {
     case 'w':
         sem_wait( clock_semaphore );
         break;
     case 'p':
             
         sem_post( clock_semaphore ); 
         break;
     default:
        printf("input: [%d]\n", (int)*data ) ;
     }


    /*printf("returned\n");*/
}
