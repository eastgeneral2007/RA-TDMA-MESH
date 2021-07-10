#include "main.h"
#define NUM_DRONES 3
#define TIMEOUT 300            //(TDMA_getRoundTime()*3) Milliseconds

typedef struct{
	uint8_t type; //Type of packet 1- MATRIX 0- STRING
	uint8_t matrix[NUM_DRONES][NUM_DRONES];
	int seqNumber[NUM_DRONES];
} Data;

Data data;
int myId;
int timeout[NUM_DRONES];
uint8_t **mst;

pthread_mutex_t mutex;

void printData(Data data) {

	PRINTF_FL("\n\n Matrix:\n\n");

	int i, x;
	for (i = 0; i < NUM_DRONES; i++) {
		for (x = 0; x < NUM_DRONES; x++) {
			printf("%"PRIu8 "\t", data.matrix[i][x]);
		}
		printf("\n");
	}

	printf("\n\n");

	for (i = 0; i < NUM_DRONES; i++) printf("%d\t%d\n", data.seqNumber[i], timeout[i]);

}

Data setDefaultData(){

	Data data;

	data.type = 1;

	int i,x;
	for (i = 0; i < NUM_DRONES; i++)
		for (x = 0; x < NUM_DRONES; x++)
			data.matrix[i][x] = 0;

	for (i = 0; i < NUM_DRONES; i++)
			data.seqNumber[i] = 0;

	return data;
}

int createSeqNum(int src_id, uint32_t SeqNumber){
	// "s# %02" PRIu8 "%02" PRIu8 "%06" PRIu32 " | " link tx-rx-seqnum

	char FinalSeqNumber[10];
	sprintf(FinalSeqNumber, "%d%.6u", src_id, SeqNumber);

	return atoi(FinalSeqNumber);
}

void updateMatrix(Data NewData, uint8_t senderIP){

	int i,x;
	for(i = 0; i < NUM_DRONES; i++){

			if((i+1) != myId && NewData.seqNumber[i] > data.seqNumber[i]){
				data.seqNumber[i] = NewData.seqNumber[i];

				pthread_mutex_lock(&mutex);
				timeout[i] = TIMEOUT;
				pthread_mutex_unlock(&mutex);

				for(x = 0; x < NUM_DRONES; x++){
					data.matrix[i][x] = NewData.matrix[i][x];
				}
			}
	}

	data.matrix[myId - 1][senderIP - 1] = 1; //PDR_get_in(senderIP);
}

void primAlgorithm(uint8_t matrix[NUM_DRONES][NUM_DRONES], uint8_t **mst){ // O(E log V)

	int i,z,x,y;

	//Reset MST	
	for(i = 0; i < NUM_DRONES; i++){
		for(x = 0; x < NUM_DRONES; x++){
			mst[i][x] = 0;
		}
	}

	//Prim Algorithm
	int visited[NUM_DRONES];

	for(i = 0; i < NUM_DRONES; i++){
		visited[i] = 0;
	}

	int numEdge = 0;
	uint8_t min;

	visited[0] = 1; //Iniciamos a pesquisa no primeiro no

	while(numEdge < NUM_DRONES-1){
		min = 2;
		x = 0; // row number
		y = 0; // col number

		for(i = 0; i < NUM_DRONES; i++){
			if(visited[i] == 1){
				for(z = 0; z < NUM_DRONES; z++){ //Vamos procurar todos os vizinhos do no
					if((visited[z] == 0) && (matrix[i][z] == 1)){
						if(min > matrix[i][z]){
							min = matrix[i][z];
							x = i;
							y = z;
						}
					}
				}
			}
		}

    mst[x][y] = matrix[x][y];
    mst[y][x] = matrix[x][y];
    visited[y] = 1;
    numEdge++;
  }
}

/* local var changed by threads */
static volatile int proceed = 1;

/*******************************************************************************
                STRUCT UPDATERS
*******************************************************************************/
/* Sends a command to the drone every ~1s */
static void* txthread() {  //(void* args)

	//PRINTF_FL("TX initiated. sending a msg every second\n");
	printf("TX initiated. sending a msg every 10ms\n");

	Data text;
	text.type = 0;

	void *pkt_text = (void *)&text;
	int pkt_text_size = sizeof(text);

	while (proceed) {

		error_t ret = TDMA_send(255, pkt_text_size, pkt_text, 0);

		if (E_SUCCESS != ret)
			PRINTF_FL_WARN("something failed: %s\n", getError(ret));	

		microsleep(6e5);
	}

	return NULL;
}

/* reads whatever is received from any drone */
static void *rxthread()//(void* args)
{

	PRINTF_FL("RX initiated. listening\n");
	printf("RX initiated. listening\n");
	Data *pkt_ptr = malloc(sizeof(Data));
	if (pkt_ptr == NULL) return NULL;

	uint8_t senderIP;
	int SeqNumber;
	while (proceed) {

			ssize_t ret = TDMA_receive(pkt_ptr, &senderIP);
			if (ret < 1)
				PRINTF_FL_WARN("something failed: %lu\n", (ret));
			dumpData((uint8_t*) pkt_ptr, ret);
	
			if(senderIP != myId){
				if(pkt_ptr->type == 1){
	
				SeqNumber = createSeqNum(senderIP, TDMA_getSeqNum());

				//                  Update line of sender                     
				pkt_ptr->seqNumber[senderIP - 1] = SeqNumber;
				//------------------------------------------------------------
				
				updateMatrix(*pkt_ptr, senderIP);

				primAlgorithm(data.matrix, mst);
				
				// Uncomment for synchronization
				drk_TDMA_setSpanningTree(mst, NUM_DRONES,1);

				printf("\nSPANNING TREE\n");

				int i,x;
				for(i = 0; i < NUM_DRONES; i++){
					for(x = 0; x < NUM_DRONES; x++){
						printf("%d\t", mst[i][x]);
					}
					printf("\n");
				}
				*/
				
				printf("\n\n");
				printData(data);

			}else{
				printf("\n\n\n NÃO É MATRIX!!!! \n\n\n");
			}
		}

	}

	return NULL;
}

/* control clock time */
static void* clockthread() {	//(void* args)


	while (proceed) {
		sleep(0.01);

		int i, x, y;
		for (i = 0; i < NUM_DRONES; i++) {
			if ((i + 1) != myId) {
				pthread_mutex_lock(&mutex);

				if (timeout[i] > -TIMEOUT)
					timeout[i] -= 1;

				if (timeout[i] == 0) //Remove adjacencies of node i (no more updates)
					for (x = 0; x < NUM_DRONES; x++)
						data.matrix[i][x] = 0;

				if (timeout[i] == -TIMEOUT) //Remove node i of matrix (DEAD)
					for (y = 0; y < NUM_DRONES; y++)
						data.matrix[y][i] = 0;

				pthread_mutex_unlock(&mutex);
			}
		}
	}

	return NULL;

}

/* send matrix packet */
static void* matrixthread(){    //(void* args)

	void* pkt_matrix = (void *)&data;
	int pkt_matrix_size;

	//microsleep(800000); 
	
	while (proceed) {

		pkt_matrix_size = sizeof(data);
		
		//sleepToStart();
		
		error_t ret = TDMA_send(255, pkt_matrix_size, pkt_matrix, 1);

		if (E_SUCCESS != ret)
			PRINTF_FL_WARN("something failed: %s\n", getError(ret));		
	
		microsleep(1e5);
	}

	return NULL;
}

void closeme()
{

	proceed=0;
	PRINTF_FL_WARN("i am terminating. Proceed %d\n", proceed);
}


int main(int argc, char *argv[])
{
	if ( argc > 2)
	{	 
		fprintf( stderr , "Usage: %s <myid>", argv[0] );
		return -1 ;
	}

	myId = (int)strtol( argv[1] , NULL, 0 );
	data = setDefaultData();

	int x;
	for (x = 0; x < NUM_DRONES; x++) {
		timeout[x] = -1;
	}

	// Uncomment for synchronization and go TDMA_slot.c function (int tdma_syncronizeSlot(int32_t *delta)) and comment the first line
	mst = (uint8_t**) malloc(sizeof(uint8_t*) * NUM_DRONES);

	for (x = 0; x < NUM_DRONES; x++) {
		mst[x] = (uint8_t*) malloc(sizeof(uint8_t) * NUM_DRONES);
	}
	
	for (int i = 0; i < NUM_DRONES; i++) {
		for (int x = 0; x < NUM_DRONES; x++) {
			mst[i][x] = 0;
		}
	}

	drk_TDMA_setSpanningTree(mst, NUM_DRONES,0);

	printf("Spanning tree:\n");
	for(int i = 0; i < NUM_DRONES; i++){
		for(int x = 0; x < NUM_DRONES; x++){
			printf("%d ", mst[i][x]);
		}
		printf("\n");
	}
	
	
	if (drk_TDMA_init(myId) < 0)
		exit(0);

	if(pthread_mutex_init(&mutex, NULL) != 0){
		PRINTF_FL_ERR("Mutex init has failed \n");
		return -1;
	}


	/*
	int sender = 1;
	if(sender){
		// create tx thread
		pthread_t tx_tid;
		pthread_attr_t act_attr;
		pthread_attr_init(&act_attr);
		size_t act_desired_stack_size = 1000000;
		pthread_attr_setstacksize(&act_attr, act_desired_stack_size);

		if (pthread_create(&tx_tid, &act_attr, &txthread, NULL) != 0) {
			PRINTF_FL_ERR("[TX] spawning thread: - Error - %s\n", strerror(errno));
			return -1;
		}PRINTF_FL("TX initiating\n");
	}
	*/
	
	/* create rx thread */
	pthread_t rx_tid;
	pthread_attr_t act_attr;
	pthread_attr_init(&act_attr);
	size_t act_desired_stack_size = 1000000;
	pthread_attr_setstacksize(&act_attr, act_desired_stack_size);

	if (pthread_create(&rx_tid, &act_attr, &rxthread, NULL) != 0) {
		PRINTF_FL_ERR("[RX] spawning thread: - Error - %s\n", strerror(errno));
		return -1;
	} PRINTF_FL("RX initiating\n");
	

	/* create clock thread */
	/*
	pthread_t clock;
	if (pthread_create(&clock, NULL, &clockthread, NULL) != 0) {
		printf("Clock thread: - Error - %s\n", strerror(errno));
		return -1;
	}
	*/
	
	/* create matrix thread */
	pthread_t matrix;
	if (pthread_create(&matrix, NULL, &matrixthread, NULL) != 0) {
		printf("Matrix thread: - Error - %s\n", strerror(errno));
		return -1;
	}
	

	/* we only close if Drklib lets us do it*/
	while(proceed)
	{
		if (!proceed) break;
		sleep(1);
	}

	/* never let it close this way. press CTRL+Z or X*/
}
