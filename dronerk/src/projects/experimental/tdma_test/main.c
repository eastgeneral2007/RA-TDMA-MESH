#include "drk/tdma.h"


#define PREFIX "MAIN_APP"

volatile int proceed=1;

void closeme()
{

	proceed=0;
	PRINTF_FL_WARN("i am terminating. Proceed %d\n", proceed);
}






int main(int argc, char *argv[])
{
	if (argc <3)
	{	 
		fprintf( stderr , "Usage: %s <myid> [<sender?> <dst]\n", argv[0] );
		return -1 ;
	}

	int my_id = (int)strtol( argv[1] , NULL, 0 ) ;
	PRINTF_FL("my id: #%" PRIu8 ".\n" , my_id );
	int sender = (int)strtol( argv[2] , NULL, 0 ) ; /* yes or no , 1 or 0 , bool */
	PRINTF_FL("Am I sender? %s.\n" , sender==0? "no":"yes" );
	int dst =  (int)strtol( argv[3], NULL, 0 ) ;
	if (argc == 4)
	{
		PRINTF_FL("Dst = #%d.\n" ,dst); 
	}
	
	if (drk_TDMA_init(my_id)< 0)
		exit(0);
	
	while ( proceed)
	{
		puts("coo coo");
		sleep(1);
	}
}
