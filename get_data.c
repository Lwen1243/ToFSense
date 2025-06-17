<<<<<<< HEAD
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

int main() {
    int serial_fd;
    struct termios tty;
    
    serial_fd = open("/dev/ttyCH343USB0", O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        perror("无法打开串口设备");
        return 1;
    }
    
    // 获取当前串口配置
    if (tcgetattr(serial_fd, &tty) != 0) {
        perror("获取串口配置失败");
        close(serial_fd);
        return 1;
    }
    
    // 设置波特率921600
    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);
    
    // 8位数据位，无校验，1位停止位
    tty.c_cflag &= ~PARENB;  // 禁用奇偶校验
    tty.c_cflag &= ~CSTOPB;  // 1位停止位
    tty.c_cflag &= ~CSIZE;   // 清除数据位设置
    tty.c_cflag |= CS8;      // 8位数据位
    
    // 禁用硬件流控
    tty.c_cflag &= ~CRTSCTS;
    
    // 启用接收
    tty.c_cflag |= CREAD | CLOCAL;
    
    // 禁用软件流控
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    // 原始输入模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;  // 原始输出模式
    
    // 设置超时 - 等待至少1个字符，每字符最多等待100ms
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    
    // 应用设置
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        perror("设置串口参数失败");
        close(serial_fd);
        return 1;
    }
    
    printf("开始读取16进制数据 (按Ctrl+C停止)...\n");
    
    // 读取数据
    unsigned char buffer[512];
    while (1) {
        int n = read(serial_fd, buffer, sizeof(buffer));
        if (n <= 26)
        {
            puts("");
        }
            

        if (n > 0)
        {
            for (int i = 0 ; i < n; i ++) {
                printf("%02x ", buffer[i]);
            }
        }
        else {
            printf("error");
            break;
        }
    }
    
    close(serial_fd);
    return 0;
}
=======
#include "get_data.h"
#include <stdio.h>

uint8_t TOF_series_switch=0;            // 0 is ToFSense; 1 is ToFSense-F; 2 is TOFSense-M 
uint8_t u_rx_buf_3[1024];                //uart3 getting cache
uint8_t u_tx_buf_3[8]={0x57,0x10,0xff,0xff,0x00,0xff,0xff,0x63}; //usart3 send ToFSense

uint8_t TOF_unpack_correct_flag=0;       
uint8_t TOF_DMA_receive_flag=0;         
uint16_t TOF_data_length=0;            

uint8_t TOF_inquire_cycle_count=0;    
uint8_t TOF_inquire_number=0;            

float TOF_FR_dis0=0.0;                  
float TOF_FM_dis1=0.0;                 
float TOF_FL_dis2=0.0;                
float TOF_BL_dis3=0.0;               
float TOF_BM_dis4=0.0;              
float TOF_BR_dis5=0.0;             
uint8_t TOF_status0=0;                   
uint8_t TOF_status1=0;                   //1��TOFSenseģ�����״ָ̬ʾ
uint8_t TOF_status2=0;                   //2��TOFSenseģ�����״ָ̬ʾ
uint8_t TOF_status3=0;                   //3��TOFSenseģ�����״ָ̬ʾ
uint8_t TOF_status4=0;                   //4��TOFSenseģ�����״ָ̬ʾ
uint8_t TOF_status5=0;                   //5��TOFSenseģ�����״ָ̬ʾ
uint16_t TOF_signal_strength0=0;         //0��TOFSenseģ������ź�ǿ��
uint16_t TOF_signal_strength1=0;         //1��TOFSenseģ������ź�ǿ��
uint16_t TOF_signal_strength2=0;         //2��TOFSenseģ������ź�ǿ��
uint16_t TOF_signal_strength3=0;         //3��TOFSenseģ������ź�ǿ��
uint16_t TOF_signal_strength4=0;         //4��TOFSenseģ������ź�ǿ��
uint16_t TOF_signal_strength5=0;         //5��TOFSenseģ������ź�ǿ��
uint16_t TOF_signal_strength=2;          //�ź�ǿ����ֵ�������ź�ǿ�ȴ��ڵ��������ֵ��ִ�б��ϲ���

uint8_t TOF_M_left_point_group[TOF_M_POINT_GROUP_NUM]={1,0,9,8,17,16,25,24,33,32,41,40,49,48,57,56};//TOFSense-Mϵ��64��λ���飬���ڼ����Ч�����ҵľ���ֵ
uint8_t TOF_M_middle_point_group[TOF_M_POINT_GROUP_NUM]={4,3,12,11,20,19,28,27,36,35,44,43,52,51,60,59};//TOFSense-Mϵ��64��λ���飬���ڼ����Ч�����ҵľ���ֵ
uint8_t TOF_M_right_point_group[TOF_M_POINT_GROUP_NUM]={7,6,15,14,23,22,31,30,39,38,47,46,55,54,63,62};//TOFSense-Mϵ��64��λ���飬���ڼ����Ч�����ҵľ���ֵ

uint8_t avoid_danger_switch=1;           //����ģʽ���أ�0��1��
uint8_t avoid_danger_status=0;           //����ģʽ״̬��1��ʾ���ڱ����㷨������
float danger_distance=0.37;              //���Ͼ�����ֵ,��λ����
float slow_down_distance=1.0;            //���ϼ��ٻ�����������ֵ,��λ����
uint8_t avoid_danger_slow_down_speed=35; //���ϼ��ٻ����������ٶȣ���Χ[0,127]
uint8_t avoid_danger_shift_speed=100;    //���Ͽ������ƺ����ٶȣ���Χ[0,127]
uint16_t avoid_danger_shift_time=0;      //���Ͽ������ƺ���ʱ�䣬��λ��ms
uint8_t avoid_danger_turn_flag=0;        //ǰ��3TOF���붼С�ڱ�����ֵ����¿�ʼתȦ��־λ
uint16_t avoid_danger_turn_count=0;      //ǰ��3TOF���붼С�ڱ�����ֵ����¿�ʼתȦ��������

/************************************************
�������� �� TOF_Application
�������� �� TOFSenseӦ�ú���
��    �� �� ��
�� �� ֵ �� ��
*************************************************/
void TOF_Application(void)
{
	uint16_t count_i=0;                                                           //ѭ����������
	float TOF_M_left_dis_temp=999999.0;                                           //TOFSense-M��������ֵ��ʱֵ
	float TOF_M_middle_dis_temp=999999.0;                                         //TOFSense-M���м����ֵ��ʱֵ
	float TOF_M_right_dis_temp=999999.0;                                          //TOFSense-M���Ҳ����ֵ��ʱֵ
	uint8_t TOF_M_left_dis_status_temp=0;                                         //TOFSense-M��������״ָ̬ʾ��ʱֵ
	uint8_t TOF_M_middle_dis_status_temp=0;                                       //TOFSense-M���м����״ָ̬ʾ��ʱֵ
	uint8_t TOF_M_right_dis_status_temp=0;                                        //TOFSense-M���Ҳ����״ָ̬ʾ��ʱֵ
	uint8_t TOF_M_left_dis_signal_strength_temp=0;                                //TOFSense-M���������ź�ǿ����ʱֵ
	uint8_t TOF_M_middle_dis_signal_strength_temp=0;                              //TOFSense-M���м�����ź�ǿ����ʱֵ
	uint8_t TOF_M_right_dis_signal_strength_temp=0;                               //TOFSense-M���Ҳ�����ź�ǿ����ʱֵ


	if((u_rx_buf_3[0] == 0x57)&&(u_rx_buf_3[1] == 0x00))                          //������յ�����TOFSense��Э��֡
	{
		TOF_unpack_correct_flag=g_nts_frame0.UnpackData(u_rx_buf_3,TOF_data_length);//���ú���ָ�����н��뺯�������ؽ�����ȷ/ʧ��ֵ
		if(TOF_unpack_correct_flag == 1)                                            //���������ȷ
		{
			TOF_series_switch=0;                                                      //��ǰ���ӵ���TOFSenseϵ�м����ഫ����

			if(g_nts_frame0.result.id == 0)                                           //�����֡�Ǳ��0��TOFSenseģ������
			{
				TOF_signal_strength0=g_nts_frame0.result.signal_strength;               //ȡTOFSense�����ź�ǿ��
				TOF_status0=g_nts_frame0.result.dis_status;                             //ȡTOFSense����״ֵ̬
				TOF_FR_dis0=g_nts_frame0.result.dis;                                    //ȡTOFSense��������ֵ
			}
			else if(g_nts_frame0.result.id == 1)                                      //�����֡�Ǳ��1��TOFSenseģ������
			{
				TOF_signal_strength1=g_nts_frame0.result.signal_strength;               //ȡTOFSense�����ź�ǿ��
				TOF_status1=g_nts_frame0.result.dis_status;                             //ȡTOFSense����״ֵ̬
				TOF_FM_dis1=g_nts_frame0.result.dis;                                  	//ȡTOFSense��������ֵ
			}
			else if(g_nts_frame0.result.id == 2)                                      //�����֡�Ǳ��2��TOFSenseģ������
			{
				TOF_signal_strength2=g_nts_frame0.result.signal_strength;               //ȡTOFSense�����ź�ǿ��
				TOF_status2=g_nts_frame0.result.dis_status;                             //ȡTOFSense����״ֵ̬
				TOF_FL_dis2=g_nts_frame0.result.dis;                                   	//ȡTOFSense��������ֵ
			}
			else if(g_nts_frame0.result.id == 3)                                      //�����֡�Ǳ��3��TOFSenseģ������
			{
				TOF_signal_strength3=g_nts_frame0.result.signal_strength;               //ȡTOFSense�����ź�ǿ��
				TOF_status3=g_nts_frame0.result.dis_status;                             //ȡTOFSense����״ֵ̬
				TOF_BL_dis3=g_nts_frame0.result.dis;                                 		//ȡTOFSense��������ֵ
			}
			else if(g_nts_frame0.result.id == 4)                                      //�����֡�Ǳ��4��TOFSenseģ������
			{
				TOF_signal_strength4=g_nts_frame0.result.signal_strength;               //ȡTOFSense�����ź�ǿ��
				TOF_status4=g_nts_frame0.result.dis_status;                             //ȡTOFSense����״ֵ̬
				TOF_BM_dis4=g_nts_frame0.result.dis;                                 		//ȡTOFSense��������ֵ
			}
			else if(g_nts_frame0.result.id == 5)                                      //�����֡�Ǳ��5��TOFSenseģ������
			{
				TOF_signal_strength5=g_nts_frame0.result.signal_strength;               //ȡTOFSense�����ź�ǿ��
				TOF_status5=g_nts_frame0.result.dis_status;                             //ȡTOFSense����״ֵ̬
				TOF_BR_dis5=g_nts_frame0.result.dis;                                    //ȡTOFSense��������ֵ
			}
		}
	}
	else if((u_rx_buf_3[0] == 0x57)&&(u_rx_buf_3[1] == 0x01))                     //������յ�����TOFSense-M��Э��֡
	{
		TOF_unpack_correct_flag=g_ntsm_frame0.UnpackData(u_rx_buf_3,TOF_data_length);//���ú���ָ�����н��뺯�������ؽ�����ȷ/ʧ��ֵ
		if(TOF_unpack_correct_flag == 1)                                            //���������ȷ
		{
			TOF_series_switch=2;                                                      //��ǰ���ӵ���TOFSense-Mϵ�м����ഫ����

			if(g_ntsm_frame0.id == 0)                                                 //�����֡�Ǳ��0��TOFSense-Mģ������
			{
      //�Ҳ����
				for(count_i=0;count_i<TOF_M_POINT_GROUP_NUM;count_i++)//ɸѡTOFSense-M����������С������Ч�ľ�����Ϊ���ľ���ֵ
				{
					if(g_ntsm_frame0.pixels[TOF_M_right_point_group[count_i]].dis_status == 0)//�����ж������ÿ����ľ���״ָ̬ʾɸ�����ϸ������
					{
						if(g_ntsm_frame0.pixels[TOF_M_right_point_group[count_i]].dis<TOF_M_right_dis_temp)//�����ǰ�����ĵ�ľ���С�ھ�����ʱ������ѵ�ǰ��ľ��븳ֵ����ʱ����������ȡ��Сֵ
						{
							TOF_M_right_dis_temp=g_ntsm_frame0.pixels[TOF_M_right_point_group[count_i]].dis;
							TOF_M_right_dis_status_temp=g_ntsm_frame0.pixels[TOF_M_right_point_group[count_i]].dis_status;
							TOF_M_right_dis_signal_strength_temp=g_ntsm_frame0.pixels[TOF_M_right_point_group[count_i]].signal_strength;
						}

					}
				}
				TOF_signal_strength0=TOF_M_right_dis_signal_strength_temp;              //ȡTOFSense�����ź�ǿ��
				TOF_status0=TOF_M_right_dis_status_temp;                                //ȡTOFSense����״ֵ̬
				TOF_FR_dis0=TOF_M_right_dis_temp;                                       //ȡTOFSense��������ֵ
      //�м����
				for(count_i=0;count_i<TOF_M_POINT_GROUP_NUM;count_i++)//ɸѡTOFSense-M�м��������С������Ч�ľ�����Ϊ�м�ľ���ֵ
				{
					if(g_ntsm_frame0.pixels[TOF_M_middle_point_group[count_i]].dis_status == 0)//�����ж������ÿ����ľ���״ָ̬ʾɸ�����ϸ������
					{
						if(g_ntsm_frame0.pixels[TOF_M_middle_point_group[count_i]].dis<TOF_M_middle_dis_temp)//�����ǰ�����ĵ�ľ���С�ھ�����ʱ������ѵ�ǰ��ľ��븳ֵ����ʱ����������ȡ��Сֵ
						{
							TOF_M_middle_dis_temp=g_ntsm_frame0.pixels[TOF_M_middle_point_group[count_i]].dis;
							TOF_M_middle_dis_status_temp=g_ntsm_frame0.pixels[TOF_M_middle_point_group[count_i]].dis_status;
							TOF_M_middle_dis_signal_strength_temp=g_ntsm_frame0.pixels[TOF_M_middle_point_group[count_i]].signal_strength;
						}

					}
				}
				TOF_signal_strength1=TOF_M_middle_dis_signal_strength_temp;             //ȡTOFSense�����ź�ǿ��
				TOF_status1=TOF_M_middle_dis_status_temp;                               //ȡTOFSense����״ֵ̬
				TOF_FM_dis1=TOF_M_middle_dis_temp;                                  	  //ȡTOFSense��������ֵ
      //������
				for(count_i=0;count_i<TOF_M_POINT_GROUP_NUM;count_i++)//ɸѡTOFSense-M����������С������Ч�ľ�����Ϊ���ľ���ֵ
				{
					if(g_ntsm_frame0.pixels[TOF_M_left_point_group[count_i]].dis_status == 0)//�����ж������ÿ����ľ���״ָ̬ʾɸ�����ϸ������
					{
						if(g_ntsm_frame0.pixels[TOF_M_left_point_group[count_i]].dis<TOF_M_left_dis_temp)//�����ǰ�����ĵ�ľ���С�ھ�����ʱ������ѵ�ǰ��ľ��븳ֵ����ʱ����������ȡ��Сֵ
						{
							TOF_M_left_dis_temp=g_ntsm_frame0.pixels[TOF_M_left_point_group[count_i]].dis;
							TOF_M_left_dis_status_temp=g_ntsm_frame0.pixels[TOF_M_left_point_group[count_i]].dis_status;
							TOF_M_left_dis_signal_strength_temp=g_ntsm_frame0.pixels[TOF_M_left_point_group[count_i]].signal_strength;
						}

					}
				}
				TOF_signal_strength2=TOF_M_left_dis_signal_strength_temp;               //ȡTOFSense�����ź�ǿ��
				TOF_status2=TOF_M_left_dis_status_temp;                                 //ȡTOFSense����״ֵ̬
				TOF_FL_dis2=TOF_M_left_dis_temp;                                        //ȡTOFSense��������ֵ

/*
��С������С��ǰ������TOFSense-Mϵ�еĵ�λ�ռ�ֲ����£�
��Ҫ��TOFSense-M�ӿڳ��ϰ�װ��TOFSense-M S���߳��ϰ�װ��

        00 01 02 03 04 05 06 07
        08 09 10 11 12 13 14 15
        16 17 18 19 20 21 22 23
        24 25 26 27 28 29 30 31
        32 33 34 35 36 37 38 39
        40 41 42 43 44 45 46 47
        48 49 50 51 52 53 54 55
        56 57 58 59 60 61 62 63

               �������ȥ��״̬��ЧֵȻ��ȡ��Сֵ��Ϊ���TOF�ı��ϵ�Ч����
        00 01
        08 09
        16 17
        24 25
        32 33
        40 41
        48 49
        56 57
               �м�����ȥ��״̬��ЧֵȻ��ȡ��Сֵ��Ϊ�м�TOF�ı��ϵ�Ч����
                 03 04
                 11 12
                 19 20
                 27 28
                 35 36
                 43 44
                 51 52
                 59 60
               �ұ�����ȥ��״̬��ЧֵȻ��ȡ��Сֵ��Ϊ�м�TOF�ı��ϵ�Ч����
                          06 07
                          14 15
                          22 23
                          30 31
                          38 39
                          46 47
                          54 55
                          62 63

 */
			}

		}
	}


}

/************************************************
�������� �� Avoid_Danger
�������� �� TOFSense���Ϻ���
��    �� �� ��
�� �� ֵ �� ��
*************************************************/
void Avoid_Danger(void)
{
	if((TOF_FR_dis0<danger_distance)&&(TOF_FM_dis1<danger_distance)&&(TOF_FL_dis2<danger_distance)
			&&(TOF_signal_strength0>TOF_signal_strength)&&(TOF_signal_strength1>TOF_signal_strength)
			&&(TOF_signal_strength2>TOF_signal_strength)&&(TOF_status0 != 255)&&(TOF_status1 != 255)&&(TOF_status2 != 255))//��V1.0.1���Ӿ���״ָ̬ʾ�жϣ����ǰ�ҡ�ǰ�С�ǰ��TOF����������С�ڱ�����ֵ�Ҷ�Ӧ�ź�ǿ�ȴ�����ֵ
	{
		if(avoid_danger_turn_flag == 0)                                                  //�����һ�γ���3TOF����С����ֵ���
		{
            printf("l");
			avoid_danger_turn_flag=1;                                                      //��־λ��1���ڵδ�ʱ������ʱxms
		}
		else if(avoid_danger_turn_flag == 2)                                             //����δ�ʱ����ʱxms����3��TOF����С����ֵ��˵���������Ҳ��ǽ��
		{
            printf("r");
		}
	}
	else if((TOF_FR_dis0<danger_distance)&&(TOF_FM_dis1<danger_distance)&&
			(TOF_signal_strength0>TOF_signal_strength)&&(TOF_signal_strength1>TOF_signal_strength)&&
			(TOF_status0 != 255)&&(TOF_status1 != 255)) //���ǰ�ҡ�ǰ��TOF����������С�ڱ�����ֵ�Ҷ�Ӧ�ź�ǿ�ȴ�����ֵ
	{
        printf("l");
	}
	else if((TOF_FM_dis1<danger_distance)&&(TOF_FL_dis2<danger_distance)&&
			(TOF_signal_strength1>TOF_signal_strength)&&(TOF_signal_strength2>TOF_signal_strength)&&
			(TOF_status1 != 255)&&(TOF_status2 != 255)) //���ǰ�С�ǰ��TOF����������С�ڱ�����ֵ�Ҷ�Ӧ�ź�ǿ�ȴ�����ֵ
	{
        printf("r");
	}
	else if((TOF_FR_dis0<danger_distance)&&(TOF_FL_dis2<danger_distance)&&
			(TOF_signal_strength0>TOF_signal_strength)&&(TOF_signal_strength2>TOF_signal_strength)&&
			(TOF_status0 != 255)&&(TOF_status2 != 255)) //���ǰ�ҡ�ǰ��TOF����������С�ڱ�����ֵ�Ҷ�Ӧ�ź�ǿ�ȴ�����ֵ
	{
        printf("l");
	}
	else if((TOF_FR_dis0<danger_distance)&&(TOF_signal_strength0>TOF_signal_strength)&&(TOF_status0 != 255)) //���ǰ��TOF����������С�ڱ�����ֵ�Ҷ�Ӧ�ź�ǿ�ȴ�����ֵ
	{
        printf("l");
		avoid_danger_shift_time=AVOID_DANGER_SHIFT_SHORT_TIME;                           //���򿪻�����һ�ν϶�ʱ��
		if(avoid_danger_turn_flag == 2)                                                  //���3TOF���붼С����ֵ����ʼ˳ʱ����ת����һ����������ֵ
		{
			avoid_danger_turn_flag=0;                                                      //����ת���־λ�����պ��Ʊ����߼�����
		}
		else if(avoid_danger_turn_flag == 1)                                             //���3TOF���붼С����ֵ����ʼ��ʱ����ת����һ����������ֵ
		{
			avoid_danger_turn_flag=0;                                                      //����ת���־λ�����պ��Ʊ����߼�����
			avoid_danger_turn_count=0;                                                     //����ת��������������պ��Ʊ����߼�����
		}
	}
	else if((TOF_FM_dis1<danger_distance)&&(TOF_signal_strength1>TOF_signal_strength)&&(TOF_status1 != 255)) //���ǰ��TOF����������С�ڱ�����ֵ�Ҷ�Ӧ�ź�ǿ�ȴ�����ֵ
	{
        printf("r");
	}
	else if((TOF_FL_dis2<danger_distance)&&(TOF_signal_strength2>TOF_signal_strength)&&(TOF_status2 != 255)) //���ǰ��TOF����������С�ڱ�����ֵ�Ҷ�Ӧ�ź�ǿ�ȴ�����ֵ
	{
        printf("r");
		avoid_danger_shift_time=AVOID_DANGER_SHIFT_SHORT_TIME;                           //���򿪻�����һ�ν϶�ʱ��
		if(avoid_danger_turn_flag == 2)                                                  //���3TOF���붼С����ֵ����ʼ˳ʱ����ת����һ����������ֵ
		{
			avoid_danger_turn_flag=0;                                                      //����ת���־λ�����պ��Ʊ����߼�����
		}
		else if(avoid_danger_turn_flag == 1)                                             //���3TOF���붼С����ֵ����ʼ��ʱ����ת����һ����������ֵ
		{
			avoid_danger_turn_flag=0;                                                      //����ת���־λ�����պ��Ʊ����߼�����
			avoid_danger_turn_count=0;                                                     //����ת��������������պ��Ʊ����߼�����
		}
	}										
	
}

>>>>>>> update
