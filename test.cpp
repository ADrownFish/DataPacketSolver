#include <iostream>

#include "cpp/DataPacketSolver.h"

int mainCpp(int argc, char **argv){

  using namespace robot;
  using Solver = DataPacketSolver<10,1,robot::CRC32>;
  using Data = Solver::Data;

  // Constructing a data packet
	uint8_t RawData[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
	Solver solver(0xA,true);
	Data data;
  data.setHead(0x0A,0x01);
  data.appendData(RawData);

	const auto makeResult = solver.makeDataPacket(data);

  /////////////////////////
	printf("\n make result(%d bytes): ", makeResult.size);
	for (int i = 0; i < makeResult.size; i++)   printf("0x%x  ", makeResult.data[i]);
	printf("\n ");
  /////////////////////////

  // Parsing the data packet
	for (size_t j = 0; j < 1; j++)
    for (int i = 0; i < makeResult.size; i++)	
      solver.pushByte(makeResult.data[i]);

  while (solver.getAvailableSize()){
    auto value = solver.getFirstDataPacket();

    /////////////////////////
    printf("\n ================\n");
    printf("- srcID %d\n", value.head.srcID);
    printf("- dstID %d\n", value.head.dstID);
    printf("- DataID %d\n", value.head.DataID);
    printf("- length %d\n ", value.length);
    printf("- data");
    for (int i = 0; i < value.length; i++)      printf("0x%x ", value.data[i]);
    /////////////////////////
  }

  return 0;
}

#include "c/DataPacketSolver.h"

int mainC(int argc, char **argv){

  // Constructing a data packet
	uint8_t RawData[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
	
  //address, enable filter
  DPS_setSolver(0x0A,1);

	DPS_Data data;
  data.head.dstID = 0x0A;
  data.head.DataID = 0x01;
  data.length = sizeof(RawData);
  memcpy(data.data,RawData,data.length);

	const ByteArrayPointer* makeResult = DPS_makeDataPacket(&data);

  /////////////////////////
	printf("\n make result(%d bytes): ", makeResult->size);
	for (int i = 0; i < makeResult->size; i++)   printf("0x%x  ", makeResult->data[i]);
	printf("\n ");
  /////////////////////////

  // Parsing the data packet
	for (size_t j = 0; j < 1; j++)
    for (int i = 0; i < makeResult->size; i++){
      DPS_pushByte(makeResult->data[i]);
      if(DPS_readyRead()){
        const DPS_Data* value =  DPS_getDataPacket();
        
        /////////////////////////
        printf("\n ================\n");
        printf("- srcID %d\n", value->head.srcID);
        printf("- dstID %d\n", value->head.dstID);
        printf("- DataID %d\n", value->head.DataID);
        printf("- length %d\n ", value->length);
        printf("- data");

        for (int i = 0; i < value->length; i++)      printf("0x%x ", value->data[i]);
        printf("\n- length %d\n ", value->length);
        /////////////////////////
      }
    }
      
  return 0;
}

int main(int argc, char **argv){
  //Cpp
  printf(" \n********************* CPP *********************\n");
  mainCpp(argc,argv);

  //C
  printf(" \n\n********************* C *********************\n");
  mainC(argc,argv);

  printf(" \n\n********************* - *********************\n");
  return 0;
}