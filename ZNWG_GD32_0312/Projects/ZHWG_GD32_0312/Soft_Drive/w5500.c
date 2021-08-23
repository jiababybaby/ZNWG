/**********************************************************************************
 * �ļ���  ��W5500.c
 * ����    ��W5500 ����������         
 * ��汾  ��ST_v3.5
 * ����    ��
 * ����    ��
**********************************************************************************/

#include "gd32f30x.h"	
#include "systick.h"
#include "W5500.h"	
#include "string.h"
#include "esp32.h"
/***************----- ��������������� -----***************/
unsigned char Gateway_IP[4];//����IP��ַ 
unsigned char Sub_Mask[4];	//�������� 
unsigned char Phy_Addr[6];	//�����ַ(MAC) 
unsigned char IP_Addr[4];	//����IP��ַ 

uint8_t S0_Port[2],S1_Port[2],S2_Port[2],S3_Port[2],S4_Port[2],S5_Port[2],S6_Port[2],S7_Port[2];	//�˿ڶ˿ں�
uint8_t S0_DIP[4],S1_DIP[4],S2_DIP[4],S3_DIP[4],S4_DIP[4],S5_DIP[4],S6_DIP[4],S7_DIP[4];	//�˿�Ŀ��IP��ַ 
uint8_t S0_DPort[2],S1_DPort[2],S2_DPort[2],S3_DPort[2],S4_DPort[2],S5_DPort[2],S6_DPort[2],S7_DPort[2];	//�˿�Ŀ�Ķ˿ں�


unsigned char UDP_DIPR[4];	//UDP(�㲥)ģʽ,Ŀ������IP��ַ
unsigned char UDP_DPORT[2];	//UDP(�㲥)ģʽ,Ŀ�������˿ں�

/***************----- �˿ڵ�����ģʽ -----***************/
/***************----- �˿ڵ�����ģʽ -----***************/
uint8_t S0_Mode =3,S1_Mode =3,S2_Mode =3,S3_Mode =3,S4_Mode =3,S5_Mode =3,S6_Mode =3,S7_Mode =3;	//�˿�0������ģʽ,0:TCP������ģʽ,1:TCP�ͻ���ģʽ,2:UDP(�㲥)ģʽ
#define TCP_SERVER	0x00	//TCP������ģʽ
#define TCP_CLIENT	0x01	//TCP�ͻ���ģʽ 
#define UDP_MODE	0x02	//UDP(�㲥)ģʽ 

/***************----- �˿ڵ�����״̬ -----***************/

/* Socket 0 */
#define S0_REG		0x08
#define S0_TX_BUF	0x10
#define S0_RX_BUF	0x18

/* Socket 1 */
#define S1_REG		0x28
#define S1_TX_BUF	0x30
#define S1_RX_BUF	0x38

/* Socket 2 */
#define S2_REG		0x48
#define S2_TX_BUF	0x50
#define S2_RX_BUF	0x58

/* Socket 3 */
#define S3_REG		0x68
#define S3_TX_BUF	0x70
#define S3_RX_BUF	0x78

/* Socket 4 */
#define S4_REG		0x88
#define S4_TX_BUF	0x90
#define S4_RX_BUF	0x98

/* Socket 5 */
#define S5_REG		0xa8
#define S5_TX_BUF	0xb0
#define S5_RX_BUF	0xb8

/* Socket 6 */
#define S6_REG		0xc8
#define S6_TX_BUF	0xd0
#define S6_RX_BUF	0xd8

/* Socket 7 */
#define S7_REG		0xe8
#define S7_TX_BUF	0xf0
#define S7_RX_BUF	0xf8


/***************----- �˿ڵ�����״̬ -----***************/
uint8_t S0_State =0,S1_State =0,S2_State =0,S3_State =0,S4_State =0,S5_State =0,S6_State =0,S7_State =0;	//�˿�0״̬��¼,1:�˿���ɳ�ʼ��,2�˿��������(����������������) 
#define S_INIT		0x01	//�˿���ɳ�ʼ�� 
#define S_CONN		0x02	//�˿��������,���������������� 

/***************----- �˿��շ����ݵ�״̬ -----***************/
uint8_t S0_Data,S1_Data,S2_Data,S3_Data,S4_Data,S5_Data,S6_Data,S7_Data;		//�˿�0���պͷ������ݵ�״̬,1:�˿ڽ��յ�����,2:�˿ڷ���������� 
#define S_RECEIVE	 0x01	//�˿ڽ��յ�һ�����ݰ� 
#define S_TRANSMITOK 0x02	//�˿ڷ���һ�����ݰ���� 

/***************----- �˿����ݻ����� -----***************/
unsigned char Rx_Buffer[2048];	//�˿ڽ������ݻ����� 
unsigned char Tx_Buffer[2048];	//�˿ڷ������ݻ����� 

unsigned char W5500_Interrupt;	//W5500�жϱ�־(0:���ж�,1:���ж�)

uint8_t Conmandn10[16]={0xc0,0xa8,0x01,0x76,0xc0,0xa8,0x01,0x01,0x01,0xF6,0xFF,0xFF,0xFF,0x00};//���ر���IP��ַ��0:3�����������ز�����4:7�����ض˿ں� ��8:9��  ��������  ��11:13��
uint8_t Conmandn20[8]	={0x70,0xB3,0xD5,0x49,0x00,0x00}; //MAC��ַ����
uint8_t Conmandn30[8]	={0X01,0X00};//�źŲ����޸���0-20/4-20mA��������
uint8_t Conmandn40[8]; 	  //Lora���ã��豸ID��0����͸����ַ��1����
uint8_t Conmandn50[8];		//485��������,�����ʡ�0��������λ��1����ֹͣλ��2����У��λ��3������������4������ѯʱ�䡾5������Լ��6��
uint8_t Conmandn60[16]; 	//Modbus����1�Ż�2�Ż����������ͣ��ߵ��ֽ�˳�򣬹����룬�豸��ַ���Ĵ�����ʼ��ַH���Ĵ�����ʼ��ַL���Ĵ�������H���Ĵ�������L
uint8_t Conmandn70[16]; 	//Modbus����3�Ż�4�Ż����������ͣ��ߵ��ֽ�˳�򣬹����룬�豸��ַ���Ĵ�����ʼ��ַH���Ĵ�����ʼ��ַL���Ĵ�������H���Ĵ�������L
uint8_t Conmandn80[16]; 	//Modbus����5�Ż�6�Ż����������ͣ��ߵ��ֽ�˳�򣬹����룬�豸��ַ���Ĵ�����ʼ��ַH���Ĵ�����ʼ��ַL���Ĵ�������H���Ĵ�������L
uint8_t Conmandn90[16]; 	//Modbus����7�Ż�8�Ż����������ͣ��ߵ��ֽ�˳�򣬹����룬�豸��ַ���Ĵ�����ʼ��ַH���Ĵ�����ʼ��ַL���Ĵ�������H���Ĵ�������L
uint8_t WebArry[1024] ={0x00,};	 //�����������д���������
uint8_t analogArry[32] = {0x00,};   //���ڴ��ģ��������
/*******************************************************************************
* ������  : W5500_GPIO_Configuration
* ����    : W5500 GPIO��ʼ������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void W5500_GPIO_Configuration(void)
{
	spi_parameter_struct spi_init_struct;

	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_SPI1);

	/* SPI0_SCK(PA5), SPI0_MISO(PA6) and SPI0_MOSI(PA7) GPIO pin configuration */
	gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_15);
	gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_14);
	/* SPI0_CS(PE3) GPIO pin configuration */
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

	/* chip select invalid*/
	SPI_W5500_CS_HIGH();

	/* SPI0 parameter config */
	spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
	spi_init_struct.device_mode          = SPI_MASTER;;
	spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;;
	spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
	spi_init_struct.nss                  = SPI_NSS_SOFT;
	spi_init_struct.prescale             = SPI_PSC_8 ;
	spi_init_struct.endian               = SPI_ENDIAN_MSB;;
	spi_init(SPI1, &spi_init_struct);

	/* set crc polynomial */
	spi_crc_polynomial_set(SPI1,7);
	/* enable SPI1 */
	spi_enable(SPI1);
}

/*******************************************************************************
* ������  : SPI1_Send_Byte
* ����    : SPI1����1���ֽ�����
* ����    : dat:�����͵�����
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
uint8_t SPI1_Send_Byte(unsigned char dat)
{
/* loop while data register in not emplty */
    while (RESET == spi_i2s_flag_get(SPI1,SPI_FLAG_TBE));

    /* send byte through the SPI0 peripheral */
    spi_i2s_data_transmit(SPI1,dat);

    /* wait to receive a byte */
    while(RESET == spi_i2s_flag_get(SPI1,SPI_FLAG_RBNE));

    /* return the byte read from the SPI bus */
    return(spi_i2s_data_receive(SPI1));
}

/*******************************************************************************
* ������  : SPI1_Send_Short
* ����    : SPI1����2���ֽ�����(16λ)
* ����    : dat:�����͵�16λ����
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void SPI1_Send_Short(unsigned short dat)
{
	SPI1_Send_Byte(dat/256);//д���ݸ�λ
	SPI1_Send_Byte(dat);	//д���ݵ�λ
}

/*******************************************************************************
* ������  : Write_W5500_1Byte
* ����    : ͨ��SPI1��ָ����ַ�Ĵ���д1���ֽ�����
* ����    : reg:16λ�Ĵ�����ַ,dat:��д�������
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void Write_W5500_1Byte(unsigned short reg, unsigned char dat)
{
	SPI_W5500_CS_LOW();//��W5500��SCSΪ�͵�ƽ

	SPI1_Send_Short(reg);//ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(FDM1|RWB_WRITE|COMMON_R);//ͨ��SPI1д�����ֽ�,1���ֽ����ݳ���,д����,ѡ��ͨ�üĴ���
	SPI1_Send_Byte(dat);//д1���ֽ�����

	SPI_W5500_CS_HIGH(); //��W5500��SCSΪ�ߵ�ƽ
}

/*******************************************************************************
* ������  : Write_W5500_2Byte
* ����    : ͨ��SPI1��ָ����ַ�Ĵ���д2���ֽ�����
* ����    : reg:16λ�Ĵ�����ַ,dat:16λ��д�������(2���ֽ�)
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void Write_W5500_2Byte(unsigned short reg, unsigned short dat)
{
	SPI_W5500_CS_LOW();//��W5500��SCSΪ�͵�ƽ
		
	SPI1_Send_Short(reg);//ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(FDM2|RWB_WRITE|COMMON_R);//ͨ��SPI1д�����ֽ�,2���ֽ����ݳ���,д����,ѡ��ͨ�üĴ���
	SPI1_Send_Short(dat);//д16λ����

	SPI_W5500_CS_HIGH(); //��W5500��SCSΪ�ߵ�ƽ
}

/*******************************************************************************
* ������  : Write_W5500_nByte
* ����    : ͨ��SPI1��ָ����ַ�Ĵ���дn���ֽ�����
* ����    : reg:16λ�Ĵ�����ַ,*dat_ptr:��д�����ݻ�����ָ��,size:��д������ݳ���
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void Write_W5500_nByte(unsigned short reg, unsigned char *dat_ptr, unsigned short size)
{
	unsigned short i;

	SPI_W5500_CS_LOW();//��W5500��SCSΪ�͵�ƽ	
		
	SPI1_Send_Short(reg);//ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(VDM|RWB_WRITE|COMMON_R);//ͨ��SPI1д�����ֽ�,N���ֽ����ݳ���,д����,ѡ��ͨ�üĴ���

	for(i=0;i<size;i++)//ѭ������������size���ֽ�����д��W5500
	{
		SPI1_Send_Byte(*dat_ptr++);//дһ���ֽ�����
	}

	SPI_W5500_CS_HIGH(); //��W5500��SCSΪ�ߵ�ƽ
}

/*******************************************************************************
* ������  : Write_W5500_SOCK_1Byte
* ����    : ͨ��SPI1��ָ���˿ڼĴ���д1���ֽ�����
* ����    : s:�˿ں�,reg:16λ�Ĵ�����ַ,dat:��д�������
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void Write_W5500_SOCK_1Byte(SOCKET s, unsigned short reg, unsigned char dat)
{
	SPI_W5500_CS_LOW();//��W5500��SCSΪ�͵�ƽ	
		
	SPI1_Send_Short(reg);//ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(FDM1|RWB_WRITE|(s*0x20+0x08));//ͨ��SPI1д�����ֽ�,1���ֽ����ݳ���,д����,ѡ��˿�s�ļĴ���
	SPI1_Send_Byte(dat);//д1���ֽ�����

	SPI_W5500_CS_HIGH(); //��W5500��SCSΪ�ߵ�ƽ
}

/*******************************************************************************
* ������  : Write_W5500_SOCK_2Byte
* ����    : ͨ��SPI1��ָ���˿ڼĴ���д2���ֽ�����
* ����    : s:�˿ں�,reg:16λ�Ĵ�����ַ,dat:16λ��д�������(2���ֽ�)
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void Write_W5500_SOCK_2Byte(SOCKET s, unsigned short reg, unsigned short dat)
{
	SPI_W5500_CS_LOW();//��W5500��SCSΪ�͵�ƽ
			
	SPI1_Send_Short(reg);//ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(FDM2|RWB_WRITE|(s*0x20+0x08));//ͨ��SPI1д�����ֽ�,2���ֽ����ݳ���,д����,ѡ��˿�s�ļĴ���
	SPI1_Send_Short(dat);//д16λ����

	SPI_W5500_CS_HIGH(); //��W5500��SCSΪ�ߵ�ƽ
}

/*******************************************************************************
* ������  : Write_W5500_SOCK_4Byte
* ����    : ͨ��SPI1��ָ���˿ڼĴ���д4���ֽ�����
* ����    : s:�˿ں�,reg:16λ�Ĵ�����ַ,*dat_ptr:��д���4���ֽڻ�����ָ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void Write_W5500_SOCK_4Byte(SOCKET s, unsigned short reg, unsigned char *dat_ptr)
{
	SPI_W5500_CS_LOW();//��W5500��SCSΪ�͵�ƽ
			
	SPI1_Send_Short(reg);//ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(FDM4|RWB_WRITE|(s*0x20+0x08));//ͨ��SPI1д�����ֽ�,4���ֽ����ݳ���,д����,ѡ��˿�s�ļĴ���

	SPI1_Send_Byte(*dat_ptr++);//д��1���ֽ�����
	SPI1_Send_Byte(*dat_ptr++);//д��2���ֽ�����
	SPI1_Send_Byte(*dat_ptr++);//д��3���ֽ�����
	SPI1_Send_Byte(*dat_ptr++);//д��4���ֽ�����

	SPI_W5500_CS_HIGH(); //��W5500��SCSΪ�ߵ�ƽ
}

/*******************************************************************************
* ������  : Read_W5500_1Byte
* ����    : ��W5500ָ����ַ�Ĵ�����1���ֽ�����
* ����    : reg:16λ�Ĵ�����ַ
* ���    : ��
* ����ֵ  : ��ȡ���Ĵ�����1���ֽ�����
* ˵��    : ��
*******************************************************************************/
unsigned char Read_W5500_1Byte(unsigned short reg)
{
	unsigned char i;

	SPI_W5500_CS_LOW();//��W5500��SCSΪ�͵�ƽ
			
	SPI1_Send_Short(reg);//ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(FDM1|RWB_READ|COMMON_R);//ͨ��SPI1д�����ֽ�,1���ֽ����ݳ���,������,ѡ��ͨ�üĴ���

	i=spi_i2s_data_receive(SPI1);
	SPI1_Send_Byte(0x00);//����һ��������
	i=spi_i2s_data_receive(SPI1);//��ȡ1���ֽ�����

	SPI_W5500_CS_HIGH();//��W5500��SCSΪ�ߵ�ƽ
	return i;//���ض�ȡ���ļĴ�������
}

/*******************************************************************************
* ������  : Read_W5500_SOCK_1Byte
* ����    : ��W5500ָ���˿ڼĴ�����1���ֽ�����
* ����    : s:�˿ں�,reg:16λ�Ĵ�����ַ
* ���    : ��
* ����ֵ  : ��ȡ���Ĵ�����1���ֽ�����
* ˵��    : ��
*******************************************************************************/
unsigned char Read_W5500_SOCK_1Byte(SOCKET s, unsigned short reg)
{
	unsigned char i;

	SPI_W5500_CS_LOW();//��W5500��SCSΪ�͵�ƽ
			
	SPI1_Send_Short(reg);//ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(FDM1|RWB_READ|(s*0x20+0x08));//ͨ��SPI1д�����ֽ�,1���ֽ����ݳ���,������,ѡ��˿�s�ļĴ���

	i=spi_i2s_data_receive(SPI1);
	SPI1_Send_Byte(0x00);//����һ��������
	i=spi_i2s_data_receive(SPI1);//��ȡ1���ֽ�����

	SPI_W5500_CS_HIGH();//��W5500��SCSΪ�ߵ�ƽ
	return i;//���ض�ȡ���ļĴ�������
}

/*******************************************************************************
* ������  : Read_W5500_SOCK_2Byte
* ����    : ��W5500ָ���˿ڼĴ�����2���ֽ�����
* ����    : s:�˿ں�,reg:16λ�Ĵ�����ַ
* ���    : ��
* ����ֵ  : ��ȡ���Ĵ�����2���ֽ�����(16λ)
* ˵��    : ��
*******************************************************************************/
unsigned short Read_W5500_SOCK_2Byte(SOCKET s, unsigned short reg)
{
	unsigned short i;

	SPI_W5500_CS_LOW();//��W5500��SCSΪ�͵�ƽ
			
	SPI1_Send_Short(reg);//ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(FDM2|RWB_READ|(s*0x20+0x08));//ͨ��SPI1д�����ֽ�,2���ֽ����ݳ���,������,ѡ��˿�s�ļĴ���

	i=spi_i2s_data_receive(SPI1);
	SPI1_Send_Byte(0x00);//����һ��������
	i=spi_i2s_data_receive(SPI1);//��ȡ��λ����
	SPI1_Send_Byte(0x00);//����һ��������
	i*=256;
	i+=spi_i2s_data_receive(SPI1);//��ȡ��λ����

	SPI_W5500_CS_HIGH();//��W5500��SCSΪ�ߵ�ƽ
	return i;//���ض�ȡ���ļĴ�������
}

/*******************************************************************************
* ������  : Read_SOCK_Data_Buffer
* ����    : ��W5500�������ݻ������ж�ȡ����
* ����    : s:�˿ں�,*dat_ptr:���ݱ��滺����ָ��
* ���    : ��
* ����ֵ  : ��ȡ�������ݳ���,rx_size���ֽ�
* ˵��    : ��
*******************************************************************************/
unsigned short Read_SOCK_Data_Buffer(SOCKET s, unsigned char *dat_ptr)
{
	unsigned short rx_size;
	unsigned short offset, offset1;
	unsigned short i;
	unsigned char j;

	rx_size=Read_W5500_SOCK_2Byte(s,Sn_RX_RSR);
	if(rx_size==0) return 0;//û���յ������򷵻�
	if(rx_size>1460) rx_size=1460;

	offset=Read_W5500_SOCK_2Byte(s,Sn_RX_RD);
	offset1=offset;
	offset&=(S_RX_SIZE-1);//����ʵ�ʵ������ַ

	SPI_W5500_CS_LOW();//��W5500��SCSΪ�͵�ƽ

	SPI1_Send_Short(offset);//д16λ��ַ
	SPI1_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//д�����ֽ�,N���ֽ����ݳ���,������,ѡ��˿�s�ļĴ���
	j=spi_i2s_data_receive(SPI1);
	
	if((offset+rx_size)<S_RX_SIZE)//�������ַδ����W5500���ջ������Ĵ���������ַ
	{
		for(i=0;i<rx_size;i++)//ѭ����ȡrx_size���ֽ�����
		{
			SPI1_Send_Byte(0x00);//����һ��������
			j=spi_i2s_data_receive(SPI1);//��ȡ1���ֽ�����
			*dat_ptr=j;//����ȡ�������ݱ��浽���ݱ��滺����
			dat_ptr++;//���ݱ��滺����ָ���ַ����1
		}
	}
	else//�������ַ����W5500���ջ������Ĵ���������ַ
	{
		offset=S_RX_SIZE-offset;
		for(i=0;i<offset;i++)//ѭ����ȡ��ǰoffset���ֽ�����
		{
			SPI1_Send_Byte(0x00);//����һ��������
			j=spi_i2s_data_receive(SPI1);//��ȡ1���ֽ�����
			*dat_ptr=j;//����ȡ�������ݱ��浽���ݱ��滺����
			dat_ptr++;//���ݱ��滺����ָ���ַ����1
		}
		SPI_W5500_CS_HIGH(); //��W5500��SCSΪ�ߵ�ƽ

		SPI_W5500_CS_LOW();//��W5500��SCSΪ�͵�ƽ

		SPI1_Send_Short(0x00);//д16λ��ַ
		SPI1_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//д�����ֽ�,N���ֽ����ݳ���,������,ѡ��˿�s�ļĴ���
		j=spi_i2s_data_receive(SPI1);

		for(;i<rx_size;i++)//ѭ����ȡ��rx_size-offset���ֽ�����
		{
			SPI1_Send_Byte(0x00);//����һ��������
			j=spi_i2s_data_receive(SPI1);//��ȡ1���ֽ�����
			*dat_ptr=j;//����ȡ�������ݱ��浽���ݱ��滺����
			dat_ptr++;//���ݱ��滺����ָ���ַ����1
		}
	}
	SPI_W5500_CS_HIGH(); //��W5500��SCSΪ�ߵ�ƽ

	offset1+=rx_size;//����ʵ�������ַ,���´ζ�ȡ���յ������ݵ���ʼ��ַ
	Write_W5500_SOCK_2Byte(s, Sn_RX_RD, offset1);
	Write_W5500_SOCK_1Byte(s, Sn_CR, RECV);//����������������
	return rx_size;//���ؽ��յ����ݵĳ���
}

/*******************************************************************************
* ������  : Write_SOCK_Data_Buffer
* ����    : ������д��W5500�����ݷ��ͻ�����
* ����    : s:�˿ں�,*dat_ptr:���ݱ��滺����ָ��,size:��д�����ݵĳ���
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void Write_SOCK_Data_Buffer(SOCKET s, unsigned char *dat_ptr, unsigned short size)
{
	unsigned short offset,offset1;
	unsigned short i;

	//�����UDPģʽ,�����ڴ�����Ŀ��������IP�Ͷ˿ں�
	if((Read_W5500_SOCK_1Byte(s,Sn_MR)&0x0f) != SOCK_UDP)//���Socket��ʧ��
	{		
		Write_W5500_SOCK_4Byte(s, Sn_DIPR, UDP_DIPR);//����Ŀ������IP  		
		Write_W5500_SOCK_2Byte(s, Sn_DPORTR, UDP_DPORT[0]*256+UDP_DPORT[1]);//����Ŀ�������˿ں�				
	}

	offset=Read_W5500_SOCK_2Byte(s,Sn_TX_WR);
	offset1=offset;
	offset&=(S_TX_SIZE-1);//����ʵ�ʵ������ַ

	SPI_W5500_CS_LOW();//��W5500��SCSΪ�͵�ƽ

	SPI1_Send_Short(offset);//д16λ��ַ
	SPI1_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//д�����ֽ�,N���ֽ����ݳ���,д����,ѡ��˿�s�ļĴ���

	if((offset+size)<S_TX_SIZE)//�������ַδ����W5500���ͻ������Ĵ���������ַ
	{
		for(i=0;i<size;i++)//ѭ��д��size���ֽ�����
		{
			SPI1_Send_Byte(*dat_ptr++);//д��һ���ֽڵ�����		
		}
	}
	else//�������ַ����W5500���ͻ������Ĵ���������ַ
	{
		offset=S_TX_SIZE-offset;
		for(i=0;i<offset;i++)//ѭ��д��ǰoffset���ֽ�����
		{
			SPI1_Send_Byte(*dat_ptr++);//д��һ���ֽڵ�����
		}
		SPI_W5500_CS_HIGH(); //��W5500��SCSΪ�ߵ�ƽ

		SPI_W5500_CS_LOW();//��W5500��SCSΪ�͵�ƽ

		SPI1_Send_Short(0x00);//д16λ��ַ
		SPI1_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//д�����ֽ�,N���ֽ����ݳ���,д����,ѡ��˿�s�ļĴ���

		for(;i<size;i++)//ѭ��д��size-offset���ֽ�����
		{
			SPI1_Send_Byte(*dat_ptr++);//д��һ���ֽڵ�����
		}
	}
	SPI_W5500_CS_HIGH(); //��W5500��SCSΪ�ߵ�ƽ

	offset1+=size;//����ʵ�������ַ,���´�д���������ݵ��������ݻ���������ʼ��ַ
	Write_W5500_SOCK_2Byte(s, Sn_TX_WR, offset1);
	Write_W5500_SOCK_1Byte(s, Sn_CR, SEND);//����������������				
}

/*******************************************************************************
* ������  : W5500_Hardware_Reset
* ����    : Ӳ����λW5500
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : W5500�ĸ�λ���ű��ֵ͵�ƽ����500us����,������ΧW5500
*******************************************************************************/
//void W5500_Hardware_Reset(void)
//{
//	GPIO_ResetBits(W5500_RST_PORT, W5500_RST);//��λ��������
//	delay_1ms(50);
//	GPIO_SetBits(W5500_RST_PORT, W5500_RST);//��λ��������
//	delay_1ms(200);
//	while((Read_W5500_1Byte(PHYCFGR)&LINK)==0);//�ȴ���̫���������
//}

/*******************************************************************************
* ������  : W5500_Init
* ����    : ��ʼ��W5500�Ĵ�������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��ʹ��W5500֮ǰ���ȶ�W5500��ʼ��
*******************************************************************************/
void W5500_Init(void)
{
	uint8_t i=0;

	Write_W5500_1Byte(MR, RST);//�����λW5500,��1��Ч,��λ���Զ���0
//	delay_1ms(10);//��ʱ10ms,�Լ�����ú���

	//��������(Gateway)��IP��ַ,Gateway_IPΪ4�ֽ�unsigned char����,�Լ����� 
	//ʹ�����ؿ���ʹͨ��ͻ�������ľ��ޣ�ͨ�����ؿ��Է��ʵ��������������Internet
	Write_W5500_nByte(GAR, Gateway_IP, 4);
			
	//������������(MASK)ֵ,SUB_MASKΪ4�ֽ�unsigned char����,�Լ�����
	//��������������������
	Write_W5500_nByte(SUBR,Sub_Mask,4);		
	
	//���������ַ,PHY_ADDRΪ6�ֽ�unsigned char����,�Լ�����,����Ψһ��ʶ�����豸�������ֵַ
	//�õ�ֵַ��Ҫ��IEEE���룬����OUI�Ĺ涨��ǰ3���ֽ�Ϊ���̴��룬�������ֽ�Ϊ��Ʒ���
	//����Լ����������ַ��ע���һ���ֽڱ���Ϊż��
	Write_W5500_nByte(SHAR,Phy_Addr,6);		

	//���ñ�����IP��ַ,IP_ADDRΪ4�ֽ�unsigned char����,�Լ�����
	//ע�⣬����IP�����뱾��IP����ͬһ�����������򱾻����޷��ҵ�����
	Write_W5500_nByte(SIPR,IP_Addr,4);		
	
	//���÷��ͻ������ͽ��ջ������Ĵ�С���ο�W5500�����ֲ�
	for(i=0;i<8;i++)
	{
		Write_W5500_SOCK_1Byte(i,Sn_RXBUF_SIZE, 0x02);//Socket Rx memory size=2k
		Write_W5500_SOCK_1Byte(i,Sn_TXBUF_SIZE, 0x02);//Socket Tx mempry size=2k
	}

	//��������ʱ�䣬Ĭ��Ϊ2000(200ms) 
	//ÿһ��λ��ֵΪ100΢��,��ʼ��ʱֵ��Ϊ2000(0x07D0),����200����
	Write_W5500_2Byte(RTR, 0x07d0);

	//�������Դ�����Ĭ��Ϊ8�� 
	//����ط��Ĵ��������趨ֵ,�������ʱ�ж�(��صĶ˿��жϼĴ����е�Sn_IR ��ʱλ(TIMEOUT)�á�1��)
	Write_W5500_1Byte(RCR,8);
}

/*******************************************************************************
* ������  : Detect_Gateway
* ����    : ������ط�����
* ����    : ��
* ���    : ��
* ����ֵ  : �ɹ�����TRUE(0xFF),ʧ�ܷ���FALSE(0x00)
* ˵��    : ��
*******************************************************************************/
unsigned char Detect_Gateway(void)
{
	unsigned char ip_adde[4];
	ip_adde[0]=IP_Addr[0]+1;
	ip_adde[1]=IP_Addr[1]+1;
	ip_adde[2]=IP_Addr[2]+1;
	ip_adde[3]=IP_Addr[3]+1;

	//������ؼ���ȡ���ص������ַ
	Write_W5500_SOCK_4Byte(0,Sn_DIPR,ip_adde);//��Ŀ�ĵ�ַ�Ĵ���д���뱾��IP��ͬ��IPֵ
	Write_W5500_SOCK_1Byte(0,Sn_MR,MR_TCP);//����socketΪTCPģʽ
	Write_W5500_SOCK_1Byte(0,Sn_CR,OPEN);//��Socket	
//	delay_1ms(5);//��ʱ5ms 	
	
	if(Read_W5500_SOCK_1Byte(0,Sn_SR) != SOCK_INIT)//���socket��ʧ��
	{
		Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//�򿪲��ɹ�,�ر�Socket
		return FALSE;//����FALSE(0x00)
	}

	Write_W5500_SOCK_1Byte(0,Sn_CR,CONNECT);//����SocketΪConnectģʽ						

	do
	{
		uint8_t j=0;
		j=Read_W5500_SOCK_1Byte(0,Sn_IR);//��ȡSocket0�жϱ�־�Ĵ���
		if(j!=0)
		Write_W5500_SOCK_1Byte(0,Sn_IR,j);
//		delay_1ms(5);//��ʱ5ms 
		if((j&IR_TIMEOUT) == IR_TIMEOUT)
		{
			return FALSE;	
		}
		else if(Read_W5500_SOCK_1Byte(0,Sn_DHAR) != 0xff)
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//�ر�Socket
			return TRUE;							
		}
	}while(1);
}

/*******************************************************************************
* ������  : Socket_Init
* ����    : ָ��Socket(0~7)��ʼ��
* ����    : s:����ʼ���Ķ˿�
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void Socket_Init(SOCKET s)
{
	//���÷�Ƭ���ȣ��ο�W5500�����ֲᣬ��ֵ���Բ��޸�	
	Write_W5500_SOCK_2Byte(0, Sn_MSSR, 1460);//����Ƭ�ֽ���=1460(0x5b4)
	//����ָ���˿�
	switch(s)
	{
		case 0:
			//���ö˿�0�Ķ˿ں�
			Write_W5500_SOCK_2Byte(0, Sn_PORT, S0_Port[0]*256+S0_Port[1]);
			//���ö˿�0Ŀ��(Զ��)�˿ں�
			Write_W5500_SOCK_2Byte(0, Sn_DPORTR, S0_DPort[0]*256+S0_DPort[1]);
			//���ö˿�0Ŀ��(Զ��)IP��ַ
			Write_W5500_SOCK_4Byte(0, Sn_DIPR, S0_DIP);			
			
			break;

		case 1:
			Write_W5500_SOCK_2Byte(1, Sn_PORT, S1_Port[0]*256+S1_Port[1]);		
			break;

		case 2:
			Write_W5500_SOCK_2Byte(2, Sn_PORT, S2_Port[0]*256+S2_Port[1]);		

			break;

		case 3:
			Write_W5500_SOCK_2Byte(3, Sn_PORT, S3_Port[0]*256+S3_Port[1]);		

			break;

		case 4:
			Write_W5500_SOCK_2Byte(4, Sn_PORT, S4_Port[0]*256+S4_Port[1]);		

			break;

		case 5:
			Write_W5500_SOCK_2Byte(5, Sn_PORT, S5_Port[0]*256+S5_Port[1]);		

			break;

		case 6:
			break;

		case 7:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* ������  : Socket_Connect
* ����    : ����ָ��Socket(0~7)Ϊ�ͻ�����Զ�̷���������
* ����    : s:���趨�Ķ˿�
* ���    : ��
* ����ֵ  : �ɹ�����TRUE(0xFF),ʧ�ܷ���FALSE(0x00)
* ˵��    : ������Socket�����ڿͻ���ģʽʱ,���øó���,��Զ�̷�������������
*			����������Ӻ���ֳ�ʱ�жϣ��������������ʧ��,��Ҫ���µ��øó�������
*			�ó���ÿ����һ��,�������������һ������
*******************************************************************************/
unsigned char Socket_Connect(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_TCP);//����socketΪTCPģʽ
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//��Socket
//	delay_1ms(5);//��ʱ5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_INIT)//���socket��ʧ��
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//�򿪲��ɹ�,�ر�Socket
		return FALSE;//����FALSE(0x00)
	}
	Write_W5500_SOCK_1Byte(s,Sn_CR,CONNECT);//����SocketΪConnectģʽ
	return TRUE;//����TRUE,���óɹ�
}

/*******************************************************************************
* ������  : Socket_Listen
* ����    : ����ָ��Socket(0~7)��Ϊ�������ȴ�Զ������������
* ����    : s:���趨�Ķ˿�
* ���    : ��
* ����ֵ  : �ɹ�����TRUE(0xFF),ʧ�ܷ���FALSE(0x00)
* ˵��    : ������Socket�����ڷ�����ģʽʱ,���øó���,�ȵ�Զ������������
*			�ó���ֻ����һ��,��ʹW5500����Ϊ������ģʽ
*******************************************************************************/

unsigned char Socket_Listen(SOCKET s)
{

	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_TCP);//����socketΪTCPģʽ 
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//��Socket	
//	delay_1ms(5);//��ʱ5ms
	if((Read_W5500_SOCK_1Byte(s,Sn_SR))!=SOCK_INIT)//���socket��ʧ��
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//�򿪲��ɹ�,�ر�Socket
		return FALSE;//����FALSE(0x00)
	}	
	Write_W5500_SOCK_1Byte(s,Sn_CR,LISTEN);//����SocketΪ����ģʽ	
//	delay_1ms(5);//��ʱ5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_LISTEN)//���socket����ʧ��
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//���ò��ɹ�,�ر�Socket
		return FALSE;//����FALSE(0x00)
	}

	return TRUE;

	//���������Socket�Ĵ򿪺�������������,����Զ�̿ͻ����Ƿ�������������,����Ҫ�ȴ�Socket�жϣ�
	//���ж�Socket�������Ƿ�ɹ����ο�W5500�����ֲ��Socket�ж�״̬
	//�ڷ���������ģʽ����Ҫ����Ŀ��IP��Ŀ�Ķ˿ں�
}

/*******************************************************************************
* ������  : Socket_UDP
* ����    : ����ָ��Socket(0~7)ΪUDPģʽ
* ����    : s:���趨�Ķ˿�
* ���    : ��
* ����ֵ  : �ɹ�����TRUE(0xFF),ʧ�ܷ���FALSE(0x00)
* ˵��    : ���Socket������UDPģʽ,���øó���,��UDPģʽ��,Socketͨ�Ų���Ҫ��������
*			�ó���ֻ����һ�Σ���ʹW5500����ΪUDPģʽ
*******************************************************************************/
unsigned char Socket_UDP(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_UDP);//����SocketΪUDPģʽ*/
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//��Socket*/
//	delay_1ms(5);//��ʱ5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_UDP)//���Socket��ʧ��
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//�򿪲��ɹ�,�ر�Socket
		return FALSE;//����FALSE(0x00)
	}
	else
		return TRUE;

	//���������Socket�Ĵ򿪺�UDPģʽ����,������ģʽ��������Ҫ��Զ��������������
	//��ΪSocket����Ҫ��������,�����ڷ�������ǰ����������Ŀ������IP��Ŀ��Socket�Ķ˿ں�
	//���Ŀ������IP��Ŀ��Socket�Ķ˿ں��ǹ̶���,�����й�����û�иı�,��ôҲ��������������
}

/*******************************************************************************
* ������  : W5500_Interrupt_Process
* ����    : W5500�жϴ��������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void W5500_Interrupt_Process(void)
{
	unsigned char i,j;

	IntDispose:

	i=Read_W5500_1Byte(SIR);//��ȡ�˿��жϱ�־�Ĵ���	
	if((i & S0_INT) == S0_INT)//Socket0�¼����� 
	{
		j=Read_W5500_SOCK_1Byte(0,Sn_IR);//��ȡSocket0�жϱ�־�Ĵ���
		Write_W5500_SOCK_1Byte(0,Sn_IR,j);
		if(j&IR_CON)//��TCPģʽ��,Socket0�ɹ����� 
		{
			S0_State|=S_CONN;//��������״̬0x02,�˿�������ӣ�����������������
		}
		if(j&IR_DISCON)//��TCPģʽ��Socket�Ͽ����Ӵ���
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//�رն˿�,�ȴ����´����� 
			Socket_Init(0);		//ָ��Socket(0~7)��ʼ��,��ʼ���˿�0
			S0_State=0;//��������״̬0x00,�˿�����ʧ��
		}
		if(j&IR_SEND_OK)//Socket0���ݷ������,�����ٴ�����S_tx_process()������������ 
		{
			S0_Data|=S_TRANSMITOK;//�˿ڷ���һ�����ݰ���� 
		}
		if(j&IR_RECV)//Socket���յ�����,��������S_rx_process()���� 
		{
			S0_Data|=S_RECEIVE;//�˿ڽ��յ�һ�����ݰ�
		}
		if(j&IR_TIMEOUT)//Socket���ӻ����ݴ��䳬ʱ���� 
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);// �رն˿�,�ȴ����´����� 			
			S0_State=0;//��������״̬0x00,�˿�����ʧ��
		}
	}

	if(Read_W5500_1Byte(SIR) != 0) 
		goto IntDispose;
}

unsigned int Timer2_Counter=0; //Timer2��ʱ����������(ms)
unsigned int W5500_Send_Delay_Counter=0; //W5500������ʱ��������(ms)

/*******************************************************************************
* ������  : W5500_Initialization
* ����    : W5500��ʼ������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void W5500_Initialization(void)
{
	W5500_Init();		//��ʼ��W5500�Ĵ�������
	Detect_Gateway();	//������ط����� 
	Socket_Init(0);		//ָ��Socket(0~7)��ʼ��,��ʼ���˿�0
	Socket_Init(1);		//ָ��Socket(0~7)��ʼ��,��ʼ���˿�0
	Socket_Init(2);		//ָ��Socket(0~7)��ʼ��,��ʼ���˿�0
	Socket_Init(3);		//ָ��Socket(0~7)��ʼ��,��ʼ���˿�0
	Socket_Init(4);		//ָ��Socket(0~7)��ʼ��,��ʼ���˿�0
	Socket_Init(5);		//ָ��Socket(0~7)��ʼ��,��ʼ���˿�0
}
uint16_t 	ip_localAddr[4]={192,168,1,200};
extern uint32_t mcuID[3];
/*******************************************************************************
* ������  : Load_Net_Parameters
* ����    : װ���������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ���ء����롢�����ַ������IP��ַ���˿ںš�Ŀ��IP��ַ��Ŀ�Ķ˿ںš��˿ڹ���ģʽ
*******************************************************************************/
void Load_Net_Parameters(void)
{
	Gateway_IP[0] = 192;//�������ز���
	Gateway_IP[1] = 168;
	Gateway_IP[2] = 1;
	Gateway_IP[3] = 1;

	Sub_Mask[0]=255;//������������
	Sub_Mask[1]=255;
	Sub_Mask[2]=255;
	Sub_Mask[3]=0;

	Phy_Addr[0]=0x0c;//���������ַ
	Phy_Addr[1]=0x29;
	Phy_Addr[2]=0xab;
	Phy_Addr[3]=mcuID[2]&0xff;
	Phy_Addr[4]=mcuID[1]&0xff;
	Phy_Addr[5]=mcuID[0]&0xff;

	IP_Addr[0]=ip_localAddr[0];//���ر���IP��ַ
	IP_Addr[1]=ip_localAddr[1];
	IP_Addr[2]=ip_localAddr[2];
	IP_Addr[3]=ip_localAddr[3];

//���ظ��˿�IP*****************************************************************/
	S0_DIP[0] =192;//���ض˿�0��Ŀ��IP��ַ
	S0_DIP[1] =168;
	S0_DIP[2] = 1;
	S0_DIP[3] = 128;
	
	S1_DIP[0] =192;//���ض˿�0��Ŀ��IP��ַ
	S1_DIP[1] =168;
	S1_DIP[2] = 1;
	S1_DIP[3] = 190;
	
	S2_DIP[0] =192;//���ض˿�0��Ŀ��IP��ַ
	S2_DIP[1] =168;
	S2_DIP[2] = 1;
	S2_DIP[3] = 190;
	
	S3_DIP[0] =192;//���ض˿�0��Ŀ��IP��ַ
	S3_DIP[1] =168;
	S3_DIP[2] = 1;
	S3_DIP[3] = 190;
	
	S4_DIP[0] =192;//���ض˿�0��Ŀ��IP��ַ
	S4_DIP[1] =168;
	S4_DIP[2] = 1;
	S4_DIP[3] = 190;
	
	S5_DIP[0] =192;//���ض˿�0��Ŀ��IP��ַ
	S5_DIP[1] =168;
	S5_DIP[2] = 1;
	S5_DIP[3] = 190;
//���ظ��˿ں�*****************************************************************/
	S0_Port[0] =0x01;//���ض˿�0�Ķ˿ں�
	S0_Port[1] =0xf6;
	S1_Port[0] = 0x13;//���ض˿�1�Ķ˿ں�
	S1_Port[1] = 0x89;
	S2_Port[0] = 0x13;//���ض˿�2�Ķ˿ں�
	S2_Port[1] = 0x8a;
	S3_Port[0] = 0x13;//���ض˿�3�Ķ˿ں�
	S3_Port[1] = 0x8b;
	S4_Port[0] = 0x13;//���ض˿�4�Ķ˿ں�
	S4_Port[1] = 0x8c;
	S5_Port[0] = 0x13;//���ض˿�5�Ķ˿ں�
	S5_Port[1] = 0x8d;
	
//����Ŀ�Ķ˿ں�****************************************************************/	
	S0_DPort[0] = 0x22;//���ض˿�0��Ŀ�Ķ˿ں�6000
	S0_DPort[1] = 0xc3;
	S1_DPort[0] = 0x17;//���ض˿�1��Ŀ�Ķ˿ں�6001
	S1_DPort[1] = 0x71;
	S2_DPort[0] = 0x17;//���ض˿�2��Ŀ�Ķ˿ں�6002
	S2_DPort[1] = 0x72;
	S3_DPort[0] = 0x17;//���ض˿�3��Ŀ�Ķ˿ں�6003
	S3_DPort[1] = 0x73;
	S4_DPort[0] = 0x17;//���ض˿�4��Ŀ�Ķ˿ں�6004
	S4_DPort[1] = 0x74;
	S5_DPort[0] = 0x17;//���ض˿�5��Ŀ�Ķ˿ں�6005
	S5_DPort[1] = 0x75;

//���ض˿ڹ���ģ***************************************************************/	
	S0_Mode=TCP_CLIENT;//���ض˿�0�Ĺ���ģʽ,TCP�����ģʽ
	S1_Mode=TCP_SERVER;
	S2_Mode=TCP_SERVER;
	S3_Mode=TCP_SERVER;
	S4_Mode=TCP_SERVER;
	S5_Mode=TCP_SERVER;
	
}

/*******************************************************************************
* ������  : W5500_Socket_Set
* ����    : W5500�˿ڳ�ʼ������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : �ֱ�����4���˿�,���ݶ˿ڹ���ģʽ,���˿�����TCP��������TCP�ͻ��˻�UDPģʽ.
*			�Ӷ˿�״̬�ֽ�Socket_State�����ж϶˿ڵĹ������
*******************************************************************************/
void W5500_Socket_Set(void)
{
	if(S0_State==0)//�˿�0��ʼ������
	{
		if(S0_Mode==TCP_SERVER)//TCP������ģʽ 
		{
			if(Socket_Listen(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else if(S0_Mode==TCP_CLIENT)//TCP�ͻ���ģʽ 
		{
			if(Socket_Connect(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else//UDPģʽ 
		{
			if(Socket_UDP(0)==TRUE)
				S0_State=S_INIT|S_CONN;
			else
				S0_State=0;
		}
	}
	
	if(S1_State==0)//�˿�1��ʼ������
	{
		if(S1_Mode==TCP_SERVER)//TCP������ģʽ 
		{
			if(Socket_Listen(1)==TRUE)
				S1_State=S_INIT;
			else
				S1_State=0;
		}
		else if(S1_Mode==TCP_CLIENT)//TCP�ͻ���ģʽ 
		{
			if(Socket_Connect(1)==TRUE)
				S1_State=S_INIT;
			else
				S1_State=0;
		}
		else//UDPģʽ 
		{
			if(Socket_UDP(1)==TRUE)
				S1_State=S_INIT|S_CONN;
			else
				S1_State=0;
		}
	}
	
	if(S2_State==0)//�˿�2��ʼ������
	{
		if(S2_Mode==TCP_SERVER)//TCP������ģʽ 
		{
			if(Socket_Listen(2)==TRUE)
				S2_State=S_INIT;
			else
				S2_State=0;
		}
		else if(S2_Mode==TCP_CLIENT)//TCP�ͻ���ģʽ 
		{
			if(Socket_Connect(2)==TRUE)
				S2_State=S_INIT;
			else
				S2_State=0;
		}
		else//UDPģʽ 
		{
			if(Socket_UDP(2)==TRUE)
				S2_State=S_INIT|S_CONN;
			else
				S2_State=0;
		}
	}
	
	if(S3_State==0)//�˿�3��ʼ������
	{
		if(S3_Mode==TCP_SERVER)//TCP������ģʽ 
		{
			if(Socket_Listen(3)==TRUE)
				S3_State=S_INIT;
			else
				S3_State=0;
		}
		else if(S3_Mode==TCP_CLIENT)//TCP�ͻ���ģʽ 
		{
			if(Socket_Connect(3)==TRUE)
				S3_State=S_INIT;
			else
				S3_State=0;
		}
		else//UDPģʽ 
		{
			if(Socket_UDP(3)==TRUE)
				S3_State=S_INIT|S_CONN;
			else
				S3_State=0;
		}
	}
	
  if(S4_State==0)//�˿�4��ʼ������
	{
		if(S4_Mode==TCP_SERVER)//TCP������ģʽ 
		{
			if(Socket_Listen(4)==TRUE)
				S4_State=S_INIT;
			else
				S4_State=0;
		}
		else if(S4_Mode==TCP_CLIENT)//TCP�ͻ���ģʽ 
		{
			if(Socket_Connect(4)==TRUE)
				S4_State=S_INIT;
			else
				S4_State=0;
		}
		else//UDPģʽ 
		{
			if(Socket_UDP(4)==TRUE)
				S4_State=S_INIT|S_CONN;
			else
				S4_State=0;
		}
	}
	
	if(S5_State==0)//�˿�4��ʼ������
	{
		if(S5_Mode==TCP_SERVER)//TCP������ģʽ 
		{
			if(Socket_Listen(5)==TRUE)
				S5_State=S_INIT;
			else
				S5_State=0;
		}
		else if(S5_Mode==TCP_CLIENT)//TCP�ͻ���ģʽ 
		{
			if(Socket_Connect(5)==TRUE)
				S5_State=S_INIT;
			else
				S5_State=0;
		}
		else//UDPģʽ 
		{
			if(Socket_UDP(5)==TRUE)
				S5_State=S_INIT|S_CONN;
			else
				S5_State=0;
		}
	}
}

/*******************************************************************************
* ������  : Process_Socket_Data
* ����    : W5500���ղ����ͽ��յ�������
* ����    : s:�˿ں�
* ���    : ��
* ����ֵ  : ��
* ˵��    : �������ȵ���S_rx_process()��W5500�Ķ˿ڽ������ݻ�������ȡ����,
*			Ȼ�󽫶�ȡ�����ݴ�Rx_Buffer������Temp_Buffer���������д���
*			������ϣ������ݴ�Temp_Buffer������Tx_Buffer������������S_tx_process()
*			�������ݡ�
*******************************************************************************/
void Process_Socket_Data(SOCKET s)
{
	unsigned short size;
	size=Read_SOCK_Data_Buffer(s, Rx_Buffer);
	usart2_sendstr((char *)Rx_Buffer);
}
/*******************************************************************************
* ������  : WEB()
* ����    : ����ͨѶ����
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void WEB()
{
	uint16_t AnsLen=0;
	W5500_Socket_Set();//W5500�˿ڳ�ʼ������
	 
	W5500_Interrupt_Process();//W5500�жϴ��������
	
	if((S0_Data & S_RECEIVE) == S_RECEIVE)//���Socket0���յ�����
	{   
		 
			S0_Data&=~S_RECEIVE;
			Process_Socket_Data(0);//W5500����
			if(S0_State == (S_INIT|S_CONN))//����
			{
				
				S0_Data&=~S_TRANSMITOK;
			  if((Rx_Buffer[9]==0x13)&&(Rx_Buffer[10]==0xf8))
				{
					//WriteConfigToAT24Cxx();   //д����
					//Write_SOCK_Data_Buffer(1, succeed,sizeof(succeed));//ָ��Socket(0~7)�������ݴ���,�˿�1����20�ֽ�����
				}
				else
				{
					
//					AnsLen = SpliceModbus();    //ƴ��ModbusЭ��
//					Write_SOCK_Data_Buffer(0, Tx_Buffer,AnsLen+9);//ָ��Socket(0~7)�������ݴ���,�˿�0����20�ֽ�����
					
				}
			}
		}
	
		if((S1_Data & S_RECEIVE) == S_RECEIVE)//���Socket1���յ�����
		{
			S1_Data&=~S_RECEIVE;
			Process_Socket_Data(1);//W5500����
			if(S1_State == (S_INIT|S_CONN))//����
			{
				S1_Data&=~S_TRANSMITOK;
				
				if((Rx_Buffer[9]==0x13)&&(Rx_Buffer[10]==0xf8))
				{
					//WriteConfigToAT24Cxx();   //д����
					//Write_SOCK_Data_Buffer(1, succeed,sizeof(succeed));//ָ��Socket(0~7)�������ݴ���,�˿�1����20�ֽ�����
				}
				else
				{
					//AnsLen = SpliceModbus();    //ƴ��ModbusЭ��
					Write_SOCK_Data_Buffer(1, Tx_Buffer,AnsLen+9);//ָ��Socket(0~7)�������ݴ���,�˿�1����20�ֽ�����
				}
			}
		}
		
		if((S2_Data & S_RECEIVE) == S_RECEIVE)//���Socket2���յ�����
		{
			S2_Data&=~S_RECEIVE;
			Process_Socket_Data(2);//W5500����
			if(S2_State == (S_INIT|S_CONN))//����
			{
				S2_Data&=~S_TRANSMITOK;
							
				if((Rx_Buffer[9]==0x13)&&(Rx_Buffer[10]==0xf8))
				{
					//WriteConfigToAT24Cxx();   //д����
					//Write_SOCK_Data_Buffer(2, succeed,sizeof(succeed));//ָ��Socket(0~7)�������ݴ���,�˿�2����20�ֽ�����
				}
				else
				{
					//AnsLen = SpliceModbus();    //ƴ��ModbusЭ��
					Write_SOCK_Data_Buffer(2, Tx_Buffer,AnsLen+9);//ָ��Socket(0~7)�������ݴ���,�˿�2����20�ֽ�����
				}
			}
		}	
		
		if((S3_Data & S_RECEIVE) == S_RECEIVE)//���Socket3���յ�����
		{
			S3_Data&=~S_RECEIVE;
			Process_Socket_Data(3);//W5500����
			if(S3_State == (S_INIT|S_CONN))//����
			{
				S3_Data&=~S_TRANSMITOK;
							
				if((Rx_Buffer[9]==0x13)&&(Rx_Buffer[10]==0xf8))
				{
					//WriteConfigToAT24Cxx();   //д����
					//Write_SOCK_Data_Buffer(3, succeed,sizeof(succeed));//ָ��Socket(0~7)�������ݴ���,�˿�3����20�ֽ�����
				}
				else
				{
					//AnsLen = SpliceModbus();    //ƴ��ModbusЭ��
					Write_SOCK_Data_Buffer(3, Tx_Buffer,AnsLen+9);//ָ��Socket(0~7)�������ݴ���,�˿�3����20�ֽ�����
				}
			}
		}	
		
	if((S4_Data & S_RECEIVE) == S_RECEIVE)//���Socket4���յ�����
	{
			S4_Data&=~S_RECEIVE;
			Process_Socket_Data(4);//W5500����
			if(S4_State == (S_INIT|S_CONN))//����
			{
				S4_Data&=~S_TRANSMITOK;
				
				if((Rx_Buffer[9]==0x13)&&(Rx_Buffer[10]==0xf8))
				{
					//WriteConfigToAT24Cxx();   //д����
					//Write_SOCK_Data_Buffer(4, succeed,sizeof(succeed));//ָ��Socket(0~7)�������ݴ���,�˿�4����20�ֽ�����
				}
				else
				{
					//AnsLen = SpliceModbus();    //ƴ��ModbusЭ��
					Write_SOCK_Data_Buffer(4, Tx_Buffer,AnsLen+9);//ָ��Socket(0~7)�������ݴ���,�˿�4����20�ֽ�����
				}
			}
	}
	
	if((S5_Data & S_RECEIVE) == S_RECEIVE)//���Socket5���յ�����
	{
		  
			S5_Data&=~S_RECEIVE;
			Process_Socket_Data(5);//W5500����
			if(S5_State == (S_INIT|S_CONN))//����
			{
				S5_Data&=~S_TRANSMITOK;
				if((Rx_Buffer[9]==0x13)&&(Rx_Buffer[10]==0xf8))
				{
					//WriteConfigToAT24Cxx();   //д����
					//Write_SOCK_Data_Buffer(5, succeed,sizeof(succeed));//ָ��Socket(0~7)�������ݴ���,�˿�5����20�ֽ�����
				}
				else
				{
					//AnsLen = SpliceModbus();    //ƴ��ModbusЭ��
					Write_SOCK_Data_Buffer(5, Tx_Buffer,AnsLen+9);//ָ��Socket(0~7)�������ݴ���,�˿�5����20�ֽ�����
				}
			}
	}		
}
