 #include "pico/time.h"
void uartInit();
void uartIrqSetup();
void receiveParse();
void acknowledge(char order[5]);
void finish(char order[5]);
void sendVar(int data,int id,int comp);
int getId(char octet0); 
unsigned int getComp(unsigned int octet0);
unsigned short getArg0();
unsigned short getArg1();
float getFloat(unsigned short arg0 , unsigned short arg1);
int getInt(float floatArg);
int tabEqual(char tab1[], char tab2[],int lenght);
int ident(unsigned char type);
void lidarStop( unsigned int comp, unsigned short arg0, unsigned short arg1);
void move( unsigned int comp, unsigned short arg0, unsigned short arg1);
void rotatefunction( unsigned int comp, unsigned short arg0, unsigned short arg1);
void cancelMove( unsigned int comp, unsigned short arg0, unsigned short arg1);
void arm( unsigned int comp, unsigned short arg0, unsigned short arg1);
void motorTime( unsigned int comp, unsigned short arg0, unsigned short arg1);
void pumpsvalvemotors( unsigned int comp, unsigned short arg0, unsigned short arg1);
void motors( unsigned int comp, unsigned short arg0, unsigned short arg1);
void motorsArgs( unsigned int comp, unsigned short arg0, unsigned short arg1);
void setVar( unsigned int comp, unsigned short arg0, unsigned short arg1);
void getVar( unsigned int comp, unsigned short arg0, unsigned short arg1);
void track( unsigned int comp, unsigned short arg0, unsigned short arg1);
void identification( unsigned int comp, unsigned short arg0, unsigned short arg1); 
void receive();

