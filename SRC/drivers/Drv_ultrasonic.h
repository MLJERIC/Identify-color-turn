#include "stm32f4xx.h"
#include "Ano_FlightCtrl.h"
static void UltraSonic_Data_Analysis(u8 *buf_data,u8 len);
void UltraSonic_Byte_Get(u8 bytedata);
void ultras_tesk();
typedef struct
{
	u8 offline;
	u16 distance;
}_UltraSonic_data_st;
//��������
extern _UltraSonic_data_st uls;
void ct_state_task(void);
u8 ct_state_task1(void);
void ct_state_task2(void);
void ct_state_task3(void);
void ultras_tesk(void);