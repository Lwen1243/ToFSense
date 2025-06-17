#ifndef __TOFSENSE_H
#define __TOFSENSE_H
#include "main.h"

#define AVOID_DANGER_SHIFT_SHORT_TIME 130     //�϶̵Ŀ������Ͽ���ʱ�䣬��λms
#define AVOID_DANGER_SHIFT_MEDIUM_TIME 300    //���еĿ������Ͽ���ʱ�䣬��λms
#define AVOID_DANGER_SHIFT_LONG_TIME 500      //�ϳ��Ŀ������Ͽ���ʱ�䣬��λms

#define TOF_M_POINT_GROUP_NUM 16              //TOF-Mϵ��64��λ������������ÿ��ĵ�λ����

void Avoid_Danger(void);                      //TOFSense���Ϻ���
void TOF_Application(void);                   //TOFSenseӦ�ú���

extern uint8_t TOF_series_switch;             //��ǰʹ�õ�TOF���⴫����ϵ�У�0ΪTOFSenseϵ�У�1ΪTOFSense-Fϵ�У�2ΪTOFSense-Mϵ�У�����Э�鹦�����Զ�ʶ��Fϵ�в�֧�ּ������ݲ�֧��

extern uint8_t u_rx_buf_3[1024];              //usart3���ջ�������
extern uint8_t u_tx_buf_3[8];                 //usart3���ͻ������飬���ڲ�ѯ������TOF

extern uint8_t TOF_unpack_correct_flag;       //TOFSenseһ֡���ݽ����ȷ��־λ
extern uint8_t TOF_DMA_receive_flag;          //DMA����һ֡TOFSenseЭ��������ɱ�־λ
extern uint16_t TOF_data_length;              //DMA����һ֡TOFSenseЭ�����ݳ���

extern uint8_t TOF_inquire_cycle_count;       //TOFSenseģ�鴮�ڲ�ѯģʽ��ѯ���ڼ���������5ms��ѯһ�Σ�
extern uint8_t TOF_inquire_number;            //TOFSenseģ�鴮�ڲ�ѯģʽ��ǰ��ѯ��ģ����

extern float TOF_FR_dis0;                   	//0�ţ�ǰ�ң�TOFSenseģ�����
extern float TOF_FM_dis1;                   	//1�ţ�ǰ�У�TOFSenseģ�����
extern float TOF_FL_dis2;                   	//2�ţ�ǰ��TOFSenseģ�����
extern float TOF_BL_dis3;                   	//3�ţ�����TOFSenseģ�����
extern float TOF_BM_dis4;                  	  //4�ţ����У�TOFSenseģ�����
extern float TOF_BR_dis5;                  	  //5�ţ����ң�TOFSenseģ�����
extern uint8_t TOF_status0;                   //0��TOFSenseģ�����״ָ̬ʾ
extern uint8_t TOF_status1;                   //1��TOFSenseģ�����״ָ̬ʾ
extern uint8_t TOF_status2;                   //2��TOFSenseģ�����״ָ̬ʾ
extern uint8_t TOF_status3;                   //3��TOFSenseģ�����״ָ̬ʾ
extern uint8_t TOF_status4;                   //4��TOFSenseģ�����״ָ̬ʾ
extern uint8_t TOF_status5;                   //5��TOFSenseģ�����״ָ̬ʾ
extern uint16_t TOF_signal_strength0;         //0��TOFSenseģ������ź�ǿ��
extern uint16_t TOF_signal_strength1;         //1��TOFSenseģ������ź�ǿ��
extern uint16_t TOF_signal_strength2;         //2��TOFSenseģ������ź�ǿ��
extern uint16_t TOF_signal_strength3;         //3��TOFSenseģ������ź�ǿ��
extern uint16_t TOF_signal_strength4;         //4��TOFSenseģ������ź�ǿ��
extern uint16_t TOF_signal_strength5;         //5��TOFSenseģ������ź�ǿ��
extern uint16_t TOF_signal_strength;          //�ź�ǿ����ֵ�������ź�ǿ�ȴ��ڵ��������ֵ��ִ�б��ϲ���
	
extern uint8_t avoid_danger_switch;           //����ģʽ���أ�0��1��
extern uint8_t avoid_danger_status;           //����ģʽ״̬��1��ʾ���ڱ����㷨������
extern float danger_distance;                 //���Ͼ�����ֵ,��λ����
extern float slow_down_distance;            	//���ϼ��ٻ�����������ֵ,��λ����
extern uint8_t avoid_danger_slow_down_speed;  //���ϼ��ٻ����������ٶȣ���Χ[0,127]
extern uint8_t avoid_danger_shift_speed;    	//���Ͽ������ƺ����ٶȣ���Χ[0,127]
extern uint16_t avoid_danger_shift_time;      //���Ͽ������ƺ���ʱ�䣬��λ��ms
extern uint8_t avoid_danger_turn_flag;        //ǰ��3TOF���붼С�ڱ�����ֵ����¿�ʼתȦ��־λ
extern uint16_t avoid_danger_turn_count;      //ǰ��3TOF���붼С�ڱ�����ֵ����¿�ʼתȦ��������


#endif


