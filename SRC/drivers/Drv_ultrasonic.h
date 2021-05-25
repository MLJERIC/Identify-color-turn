#include "stm32f4xx.h"
#include "Ano_FlightCtrl.h"
static void UltraSonic_Data_Analysis(u8 *buf_data,u8 len);
void UltraSonic_Byte_Get(u8 bytedata);
typedef struct
{
	u8 offline;
	u16 distance;
}_UltraSonic_data_st;
//Êý¾ÝÉùÃ÷
extern _UltraSonic_data_st uls;
void ct_state_task();
