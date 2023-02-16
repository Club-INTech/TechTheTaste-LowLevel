
typedef struct {
	char Buffer[25];
	int BufferSize;
	int Head;
	int Tail;
	int BufferFullFlag;
	int BufferEmptyFlag;
	int BufferOrderNumber;
	} CircularBuffer;

extern  CircularBuffer buffer;
extern char order[5];
extern int orderExecuted;
extern unsigned int id;
extern unsigned int comp;
extern unsigned short arg0;
extern unsigned short arg1;


void uartInit();
void uartIrqSetup();
void receive();
void acknowledge(char order[5]);
void finish(char order[5]);
int WriteBuffer(char data, CircularBuffer *Buffer);
void BufferInit(CircularBuffer *Buffer);
int IsBufferEmpty(CircularBuffer *Buffer);
int ReadBuffer(char *data,  CircularBuffer *Buffer);
int ReadNewOrder(char order[5], CircularBuffer *Buffer); 
unsigned long extraire_bit(unsigned long data, int n); 
unsigned int concatene16(unsigned int octet1, unsigned int octet2);
unsigned int concatene32(unsigned int double_octet1, unsigned int double_octet2);
int getID(char octet0); 
unsigned int getCOMP(unsigned int octet0);
unsigned int getARG(unsigned int double_octet);
float getARG_float(unsigned short buffer_reception[]);




