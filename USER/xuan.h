
//���ܲü�
#define Crystal         8  //����Ƶ��
#define CPU_SPEED       72 //CPU�ٶ�

#define MAVLINK         1   //MAVLINKЭ��ʹ��
#define MAVLINK_DEBUG   0   //�Ƿ�ʹ��MavlinkЭ��ش�IDͳ�ƹ���--���ֹر�

#if CPU_SPEED == 72
	#define USB      			1  	//USB��Ϸҡ��ʹ�ܣ�����CPUΪ72Mhz��ʱ��USB������������
#else 
	#define USB      			0  	//USB��Ϸҡ�˽�ֹ������CPUƵ��
#endif


#define aux_version     36  //����汾����

#define logic_count     4   //�߼����� -- �����޸�
#define mixer_count     4   //������� -- �����޸�

#define USE_animation   0   //�Ƿ�ʹ��UI�л�����
#define CRSF_DEBUG      1   //�Ƿ�ʹ��CRSFЭ��ش�IDͳ�ƹ���
#define DMA             0   //������Ļˢ���Ƿ�ʹ��DMA����Ļ����ռ�÷�һ����12800->25600
#define BKG             0   //�Ƿ�ʹ��ͼƬ��Ϊ����





