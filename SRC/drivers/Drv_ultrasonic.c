#include "Drv_ultrasonic.h"
#include "stm32f4xx.h" 
#include "Ano_FlightCtrl.h"
#include "ANO_FcData.h"
#include "Drv_OpenMV.h"
#include "Ano_ProgramCtrl_User.h"
//ioesr05���������ݽ��պ���
u8 UltraSonic_buf[10]; 

static void UltraSonic_Data_Analysis(u8 *buf_data,u8 len)
{
if(buf_data[0]==0xff)
{
	if(buf_data[1]<=7)
	{
		flag.offline=0;
		flag.distance= buf_data[1]*256+buf_data[2];

	}
	else
	{
		flag.offline=1;
		flag.distance= 0;
	}				
}

}

void UltraSonic_Byte_Get(u8 bytedata)
{
	static u8 rec_sta;
	u8 check_val=0;
	
	//
	UltraSonic_buf[rec_sta] = bytedata;
	//
	if(rec_sta==0)
	{
		if(bytedata==0xff)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}
	}
	else if(rec_sta==1)
	{
		if(bytedata >0x07)//(bytedata==0x29)δȷ��
		{
			rec_sta=0;
						flag.offline=1;
				flag.distance= 0xffff;
		}	
		else 
		{
			rec_sta++;
		}		
	}
	else if(rec_sta==2)
	{
			rec_sta++;
		
	}
	else if(rec_sta==3)
	{
			for(u8 i=0;i<3;i++)
		{
			check_val += UltraSonic_buf[i];
		}
		//
		if(check_val == bytedata)
		{
			//�����ɹ�
			UltraSonic_Data_Analysis(UltraSonic_buf,4);
			//
			rec_sta=0;
		}
		else
		{

			rec_sta=0;
		}		
	}
}

void ct_state_task()
{
	static u8 state=0;
	static u8 state_loss=0;
	if(state==0)
	{
		if(opmv.offline==0)//��Ұ���Ƿ���ְ���ɫ����
		{
			Program_Ctrl_User_Set_YAWdps(-4*opmv.cb.pos_x);//����ʶ�𣬿��Ƹ�������Ұ����
			if(ABS(opmv.cb.pos_x)<20)//�������Ұ���뷶Χ��
			{
				state++;		//������һ���׶�
			}		
		}
		else
		{		
			Program_Ctrl_User_Set_YAWdps(10);
		}		
	}
	else if(state ==1)//����׶�2���������
	{
		if(flag.offline)
		{
		if(flag.distance>350||flag.distance<250)//���ڿ�ת����Χ��ʱ
		{
			pc_user.vel_cmps_set_h[0] = 0.1*(flag.distance-350);		//���ƾ���
		}
		else 
		{
			state++;//���ڿ�ת����Χʱ��������һ���׶�
		}
		}
		if(opmv.offline)//�����ʱͻȻopmv���߻��������ߣ��ص�״̬1
		{
			state =0;		
		}		
	}
	else if(state ==2)
	{
		
			pc_user.vel_cmps_set_h[1]=10;//����������������ʱ����׶�3�������ƶ�
			if(opmv.offline)//�ƶ���opmvʧȥ�Ӿ�ʱ���ص���һ�׶�
			{
				state=0;
				pc_user.vel_cmps_set_h[1]=0;
			}	
	}

}

u8 ct_state_task1()//yaw�����
{
	if(opmv.offline==0)//���ӳ�������Ұ��
	{	
		if(ABS(opmv.cb.pos_x)>20)//���Ӳ�����Ұ����
		{
			Program_Ctrl_User_Set_YAWdps(-1*opmv.cb.pos_x);
			return 1;
		}
		else
		{	
			Program_Ctrl_User_Set_YAWdps(0);
			return 2;
		}
	}
	else
	{
		Program_Ctrl_User_Set_YAWdps(10);//��ʧ��Ұ��ʱ��ԭ������
		Program_Ctrl_User_Set_HXYcmps(0,0);
		return 0;
	}
}

void ct_state_task2()//ǰ���ƶ�����
{
	if(ct_state_task1()==1)//��Ұ��������������
	{		
		if(flag.offline==0)//��������ʶ��
		{
			pc_user.vel_cmps_set_h[0] = 0.1*(flag.distance-350);		//���ƾ���
		}
		else
		{
			pc_user.vel_cmps_set_h[0] = 0;
		}
	}
}

void ct_state_task3()
{
	if(ct_state_task1()==1)
	{
		pc_user.vel_cmps_set_h[1]= 0.1*(90-opmv.cb.pos_x);     //�����ƶ�
	}
	else
	{
		pc_user.vel_cmps_set_h[1]=0;
	}
}
s16 opmv_pox_pid(s16 ep_pox,s16 opmv_pox){//�����ƶ�pid

	s16 pox_pid,pox_pid_p,pox_pid_i,pox_pid_d,ep_pox_old=0,opmv_pox_old=0;
	pox_pid_p=(ep_pox-opmv_pox);
	pox_pid_i=ep_pox-((ep_pox-pox_pid_p)+opmv_pox);
	pox_pid_d=(ep_pox-pox_pid_p)-(ep_pox_old-opmv_pox_old);

	ep_pox_old=ep_pox;
	opmv_pox_old=opmv_pox;
	pox_pid=pox_pid_p+pox_pid_i+pox_pid_d;
	return pox_pid;
}


