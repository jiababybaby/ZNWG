/**********************************************************************************
 * 文件名  ：W5500.c
 * 描述    ：W5500 驱动函数库         
 * 库版本  ：ST_v3.5
 * 作者    ：
 * 博客    ：
**********************************************************************************/

#include "gd32f30x.h"	
#include "systick.h"
#include "W5500.h"	
#include "string.h"
#include "esp32.h"
/***************----- 网络参数变量定义 -----***************/
unsigned char Gateway_IP[4];//网关IP地址 
unsigned char Sub_Mask[4];	//子网掩码 
unsigned char Phy_Addr[6];	//物理地址(MAC) 
unsigned char IP_Addr[4];	//本机IP地址 

uint8_t S0_Port[2],S1_Port[2],S2_Port[2],S3_Port[2],S4_Port[2],S5_Port[2],S6_Port[2],S7_Port[2];	//端口端口号
uint8_t S0_DIP[4],S1_DIP[4],S2_DIP[4],S3_DIP[4],S4_DIP[4],S5_DIP[4],S6_DIP[4],S7_DIP[4];	//端口目的IP地址 
uint8_t S0_DPort[2],S1_DPort[2],S2_DPort[2],S3_DPort[2],S4_DPort[2],S5_DPort[2],S6_DPort[2],S7_DPort[2];	//端口目的端口号


unsigned char UDP_DIPR[4];	//UDP(广播)模式,目的主机IP地址
unsigned char UDP_DPORT[2];	//UDP(广播)模式,目的主机端口号

/***************----- 端口的运行模式 -----***************/
/***************----- 端口的运行模式 -----***************/
uint8_t S0_Mode =3,S1_Mode =3,S2_Mode =3,S3_Mode =3,S4_Mode =3,S5_Mode =3,S6_Mode =3,S7_Mode =3;	//端口0的运行模式,0:TCP服务器模式,1:TCP客户端模式,2:UDP(广播)模式
#define TCP_SERVER	0x00	//TCP服务器模式
#define TCP_CLIENT	0x01	//TCP客户端模式 
#define UDP_MODE	0x02	//UDP(广播)模式 

/***************----- 端口的运行状态 -----***************/

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


/***************----- 端口的运行状态 -----***************/
uint8_t S0_State =0,S1_State =0,S2_State =0,S3_State =0,S4_State =0,S5_State =0,S6_State =0,S7_State =0;	//端口0状态记录,1:端口完成初始化,2端口完成连接(可以正常传输数据) 
#define S_INIT		0x01	//端口完成初始化 
#define S_CONN		0x02	//端口完成连接,可以正常传输数据 

/***************----- 端口收发数据的状态 -----***************/
uint8_t S0_Data,S1_Data,S2_Data,S3_Data,S4_Data,S5_Data,S6_Data,S7_Data;		//端口0接收和发送数据的状态,1:端口接收到数据,2:端口发送数据完成 
#define S_RECEIVE	 0x01	//端口接收到一个数据包 
#define S_TRANSMITOK 0x02	//端口发送一个数据包完成 

/***************----- 端口数据缓冲区 -----***************/
unsigned char Rx_Buffer[2048];	//端口接收数据缓冲区 
unsigned char Tx_Buffer[2048];	//端口发送数据缓冲区 

unsigned char W5500_Interrupt;	//W5500中断标志(0:无中断,1:有中断)

uint8_t Conmandn10[16]={0xc0,0xa8,0x01,0x76,0xc0,0xa8,0x01,0x01,0x01,0xF6,0xFF,0xFF,0xFF,0x00};//加载本机IP地址【0:3】，加载网关参数【4:7】加载端口号 【8:9】  子网掩码  【11:13】
uint8_t Conmandn20[8]	={0x70,0xB3,0xD5,0x49,0x00,0x00}; //MAC地址设置
uint8_t Conmandn30[8]	={0X01,0X00};//信号参数修改码0-20/4-20mA参数设置
uint8_t Conmandn40[8]; 	  //Lora配置，设备ID【0】，透传地址【1】；
uint8_t Conmandn50[8];		//485参数设置,波特率【0】，数据位【1】，停止位【2】，校验位【3】，控制流【4】，轮询时间【5】，规约【6】
uint8_t Conmandn60[16]; 	//Modbus配置1号机2号机，数据类型，高低字节顺序，功能码，设备地址，寄存器起始地址H，寄存器起始地址L，寄存器数量H，寄存器数量L
uint8_t Conmandn70[16]; 	//Modbus配置3号机4号机，数据类型，高低字节顺序，功能码，设备地址，寄存器起始地址H，寄存器起始地址L，寄存器数量H，寄存器数量L
uint8_t Conmandn80[16]; 	//Modbus配置5号机6号机，数据类型，高低字节顺序，功能码，设备地址，寄存器起始地址H，寄存器起始地址L，寄存器数量H，寄存器数量L
uint8_t Conmandn90[16]; 	//Modbus配置7号机8号机，数据类型，高低字节顺序，功能码，设备地址，寄存器起始地址H，寄存器起始地址L，寄存器数量H，寄存器数量L
uint8_t WebArry[1024] ={0x00,};	 //用于载入所有待发送数据
uint8_t analogArry[32] = {0x00,};   //用于存放模拟量数据
/*******************************************************************************
* 函数名  : W5500_GPIO_Configuration
* 描述    : W5500 GPIO初始化配置
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 无
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
* 函数名  : SPI1_Send_Byte
* 描述    : SPI1发送1个字节数据
* 输入    : dat:待发送的数据
* 输出    : 无
* 返回值  : 无
* 说明    : 无
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
* 函数名  : SPI1_Send_Short
* 描述    : SPI1发送2个字节数据(16位)
* 输入    : dat:待发送的16位数据
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void SPI1_Send_Short(unsigned short dat)
{
	SPI1_Send_Byte(dat/256);//写数据高位
	SPI1_Send_Byte(dat);	//写数据低位
}

/*******************************************************************************
* 函数名  : Write_W5500_1Byte
* 描述    : 通过SPI1向指定地址寄存器写1个字节数据
* 输入    : reg:16位寄存器地址,dat:待写入的数据
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Write_W5500_1Byte(unsigned short reg, unsigned char dat)
{
	SPI_W5500_CS_LOW();//置W5500的SCS为低电平

	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM1|RWB_WRITE|COMMON_R);//通过SPI1写控制字节,1个字节数据长度,写数据,选择通用寄存器
	SPI1_Send_Byte(dat);//写1个字节数据

	SPI_W5500_CS_HIGH(); //置W5500的SCS为高电平
}

/*******************************************************************************
* 函数名  : Write_W5500_2Byte
* 描述    : 通过SPI1向指定地址寄存器写2个字节数据
* 输入    : reg:16位寄存器地址,dat:16位待写入的数据(2个字节)
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Write_W5500_2Byte(unsigned short reg, unsigned short dat)
{
	SPI_W5500_CS_LOW();//置W5500的SCS为低电平
		
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM2|RWB_WRITE|COMMON_R);//通过SPI1写控制字节,2个字节数据长度,写数据,选择通用寄存器
	SPI1_Send_Short(dat);//写16位数据

	SPI_W5500_CS_HIGH(); //置W5500的SCS为高电平
}

/*******************************************************************************
* 函数名  : Write_W5500_nByte
* 描述    : 通过SPI1向指定地址寄存器写n个字节数据
* 输入    : reg:16位寄存器地址,*dat_ptr:待写入数据缓冲区指针,size:待写入的数据长度
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Write_W5500_nByte(unsigned short reg, unsigned char *dat_ptr, unsigned short size)
{
	unsigned short i;

	SPI_W5500_CS_LOW();//置W5500的SCS为低电平	
		
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(VDM|RWB_WRITE|COMMON_R);//通过SPI1写控制字节,N个字节数据长度,写数据,选择通用寄存器

	for(i=0;i<size;i++)//循环将缓冲区的size个字节数据写入W5500
	{
		SPI1_Send_Byte(*dat_ptr++);//写一个字节数据
	}

	SPI_W5500_CS_HIGH(); //置W5500的SCS为高电平
}

/*******************************************************************************
* 函数名  : Write_W5500_SOCK_1Byte
* 描述    : 通过SPI1向指定端口寄存器写1个字节数据
* 输入    : s:端口号,reg:16位寄存器地址,dat:待写入的数据
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Write_W5500_SOCK_1Byte(SOCKET s, unsigned short reg, unsigned char dat)
{
	SPI_W5500_CS_LOW();//置W5500的SCS为低电平	
		
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM1|RWB_WRITE|(s*0x20+0x08));//通过SPI1写控制字节,1个字节数据长度,写数据,选择端口s的寄存器
	SPI1_Send_Byte(dat);//写1个字节数据

	SPI_W5500_CS_HIGH(); //置W5500的SCS为高电平
}

/*******************************************************************************
* 函数名  : Write_W5500_SOCK_2Byte
* 描述    : 通过SPI1向指定端口寄存器写2个字节数据
* 输入    : s:端口号,reg:16位寄存器地址,dat:16位待写入的数据(2个字节)
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Write_W5500_SOCK_2Byte(SOCKET s, unsigned short reg, unsigned short dat)
{
	SPI_W5500_CS_LOW();//置W5500的SCS为低电平
			
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM2|RWB_WRITE|(s*0x20+0x08));//通过SPI1写控制字节,2个字节数据长度,写数据,选择端口s的寄存器
	SPI1_Send_Short(dat);//写16位数据

	SPI_W5500_CS_HIGH(); //置W5500的SCS为高电平
}

/*******************************************************************************
* 函数名  : Write_W5500_SOCK_4Byte
* 描述    : 通过SPI1向指定端口寄存器写4个字节数据
* 输入    : s:端口号,reg:16位寄存器地址,*dat_ptr:待写入的4个字节缓冲区指针
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Write_W5500_SOCK_4Byte(SOCKET s, unsigned short reg, unsigned char *dat_ptr)
{
	SPI_W5500_CS_LOW();//置W5500的SCS为低电平
			
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM4|RWB_WRITE|(s*0x20+0x08));//通过SPI1写控制字节,4个字节数据长度,写数据,选择端口s的寄存器

	SPI1_Send_Byte(*dat_ptr++);//写第1个字节数据
	SPI1_Send_Byte(*dat_ptr++);//写第2个字节数据
	SPI1_Send_Byte(*dat_ptr++);//写第3个字节数据
	SPI1_Send_Byte(*dat_ptr++);//写第4个字节数据

	SPI_W5500_CS_HIGH(); //置W5500的SCS为高电平
}

/*******************************************************************************
* 函数名  : Read_W5500_1Byte
* 描述    : 读W5500指定地址寄存器的1个字节数据
* 输入    : reg:16位寄存器地址
* 输出    : 无
* 返回值  : 读取到寄存器的1个字节数据
* 说明    : 无
*******************************************************************************/
unsigned char Read_W5500_1Byte(unsigned short reg)
{
	unsigned char i;

	SPI_W5500_CS_LOW();//置W5500的SCS为低电平
			
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM1|RWB_READ|COMMON_R);//通过SPI1写控制字节,1个字节数据长度,读数据,选择通用寄存器

	i=spi_i2s_data_receive(SPI1);
	SPI1_Send_Byte(0x00);//发送一个哑数据
	i=spi_i2s_data_receive(SPI1);//读取1个字节数据

	SPI_W5500_CS_HIGH();//置W5500的SCS为高电平
	return i;//返回读取到的寄存器数据
}

/*******************************************************************************
* 函数名  : Read_W5500_SOCK_1Byte
* 描述    : 读W5500指定端口寄存器的1个字节数据
* 输入    : s:端口号,reg:16位寄存器地址
* 输出    : 无
* 返回值  : 读取到寄存器的1个字节数据
* 说明    : 无
*******************************************************************************/
unsigned char Read_W5500_SOCK_1Byte(SOCKET s, unsigned short reg)
{
	unsigned char i;

	SPI_W5500_CS_LOW();//置W5500的SCS为低电平
			
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM1|RWB_READ|(s*0x20+0x08));//通过SPI1写控制字节,1个字节数据长度,读数据,选择端口s的寄存器

	i=spi_i2s_data_receive(SPI1);
	SPI1_Send_Byte(0x00);//发送一个哑数据
	i=spi_i2s_data_receive(SPI1);//读取1个字节数据

	SPI_W5500_CS_HIGH();//置W5500的SCS为高电平
	return i;//返回读取到的寄存器数据
}

/*******************************************************************************
* 函数名  : Read_W5500_SOCK_2Byte
* 描述    : 读W5500指定端口寄存器的2个字节数据
* 输入    : s:端口号,reg:16位寄存器地址
* 输出    : 无
* 返回值  : 读取到寄存器的2个字节数据(16位)
* 说明    : 无
*******************************************************************************/
unsigned short Read_W5500_SOCK_2Byte(SOCKET s, unsigned short reg)
{
	unsigned short i;

	SPI_W5500_CS_LOW();//置W5500的SCS为低电平
			
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM2|RWB_READ|(s*0x20+0x08));//通过SPI1写控制字节,2个字节数据长度,读数据,选择端口s的寄存器

	i=spi_i2s_data_receive(SPI1);
	SPI1_Send_Byte(0x00);//发送一个哑数据
	i=spi_i2s_data_receive(SPI1);//读取高位数据
	SPI1_Send_Byte(0x00);//发送一个哑数据
	i*=256;
	i+=spi_i2s_data_receive(SPI1);//读取低位数据

	SPI_W5500_CS_HIGH();//置W5500的SCS为高电平
	return i;//返回读取到的寄存器数据
}

/*******************************************************************************
* 函数名  : Read_SOCK_Data_Buffer
* 描述    : 从W5500接收数据缓冲区中读取数据
* 输入    : s:端口号,*dat_ptr:数据保存缓冲区指针
* 输出    : 无
* 返回值  : 读取到的数据长度,rx_size个字节
* 说明    : 无
*******************************************************************************/
unsigned short Read_SOCK_Data_Buffer(SOCKET s, unsigned char *dat_ptr)
{
	unsigned short rx_size;
	unsigned short offset, offset1;
	unsigned short i;
	unsigned char j;

	rx_size=Read_W5500_SOCK_2Byte(s,Sn_RX_RSR);
	if(rx_size==0) return 0;//没接收到数据则返回
	if(rx_size>1460) rx_size=1460;

	offset=Read_W5500_SOCK_2Byte(s,Sn_RX_RD);
	offset1=offset;
	offset&=(S_RX_SIZE-1);//计算实际的物理地址

	SPI_W5500_CS_LOW();//置W5500的SCS为低电平

	SPI1_Send_Short(offset);//写16位地址
	SPI1_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//写控制字节,N个字节数据长度,读数据,选择端口s的寄存器
	j=spi_i2s_data_receive(SPI1);
	
	if((offset+rx_size)<S_RX_SIZE)//如果最大地址未超过W5500接收缓冲区寄存器的最大地址
	{
		for(i=0;i<rx_size;i++)//循环读取rx_size个字节数据
		{
			SPI1_Send_Byte(0x00);//发送一个哑数据
			j=spi_i2s_data_receive(SPI1);//读取1个字节数据
			*dat_ptr=j;//将读取到的数据保存到数据保存缓冲区
			dat_ptr++;//数据保存缓冲区指针地址自增1
		}
	}
	else//如果最大地址超过W5500接收缓冲区寄存器的最大地址
	{
		offset=S_RX_SIZE-offset;
		for(i=0;i<offset;i++)//循环读取出前offset个字节数据
		{
			SPI1_Send_Byte(0x00);//发送一个哑数据
			j=spi_i2s_data_receive(SPI1);//读取1个字节数据
			*dat_ptr=j;//将读取到的数据保存到数据保存缓冲区
			dat_ptr++;//数据保存缓冲区指针地址自增1
		}
		SPI_W5500_CS_HIGH(); //置W5500的SCS为高电平

		SPI_W5500_CS_LOW();//置W5500的SCS为低电平

		SPI1_Send_Short(0x00);//写16位地址
		SPI1_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//写控制字节,N个字节数据长度,读数据,选择端口s的寄存器
		j=spi_i2s_data_receive(SPI1);

		for(;i<rx_size;i++)//循环读取后rx_size-offset个字节数据
		{
			SPI1_Send_Byte(0x00);//发送一个哑数据
			j=spi_i2s_data_receive(SPI1);//读取1个字节数据
			*dat_ptr=j;//将读取到的数据保存到数据保存缓冲区
			dat_ptr++;//数据保存缓冲区指针地址自增1
		}
	}
	SPI_W5500_CS_HIGH(); //置W5500的SCS为高电平

	offset1+=rx_size;//更新实际物理地址,即下次读取接收到的数据的起始地址
	Write_W5500_SOCK_2Byte(s, Sn_RX_RD, offset1);
	Write_W5500_SOCK_1Byte(s, Sn_CR, RECV);//发送启动接收命令
	return rx_size;//返回接收到数据的长度
}

/*******************************************************************************
* 函数名  : Write_SOCK_Data_Buffer
* 描述    : 将数据写入W5500的数据发送缓冲区
* 输入    : s:端口号,*dat_ptr:数据保存缓冲区指针,size:待写入数据的长度
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Write_SOCK_Data_Buffer(SOCKET s, unsigned char *dat_ptr, unsigned short size)
{
	unsigned short offset,offset1;
	unsigned short i;

	//如果是UDP模式,可以在此设置目的主机的IP和端口号
	if((Read_W5500_SOCK_1Byte(s,Sn_MR)&0x0f) != SOCK_UDP)//如果Socket打开失败
	{		
		Write_W5500_SOCK_4Byte(s, Sn_DIPR, UDP_DIPR);//设置目的主机IP  		
		Write_W5500_SOCK_2Byte(s, Sn_DPORTR, UDP_DPORT[0]*256+UDP_DPORT[1]);//设置目的主机端口号				
	}

	offset=Read_W5500_SOCK_2Byte(s,Sn_TX_WR);
	offset1=offset;
	offset&=(S_TX_SIZE-1);//计算实际的物理地址

	SPI_W5500_CS_LOW();//置W5500的SCS为低电平

	SPI1_Send_Short(offset);//写16位地址
	SPI1_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//写控制字节,N个字节数据长度,写数据,选择端口s的寄存器

	if((offset+size)<S_TX_SIZE)//如果最大地址未超过W5500发送缓冲区寄存器的最大地址
	{
		for(i=0;i<size;i++)//循环写入size个字节数据
		{
			SPI1_Send_Byte(*dat_ptr++);//写入一个字节的数据		
		}
	}
	else//如果最大地址超过W5500发送缓冲区寄存器的最大地址
	{
		offset=S_TX_SIZE-offset;
		for(i=0;i<offset;i++)//循环写入前offset个字节数据
		{
			SPI1_Send_Byte(*dat_ptr++);//写入一个字节的数据
		}
		SPI_W5500_CS_HIGH(); //置W5500的SCS为高电平

		SPI_W5500_CS_LOW();//置W5500的SCS为低电平

		SPI1_Send_Short(0x00);//写16位地址
		SPI1_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//写控制字节,N个字节数据长度,写数据,选择端口s的寄存器

		for(;i<size;i++)//循环写入size-offset个字节数据
		{
			SPI1_Send_Byte(*dat_ptr++);//写入一个字节的数据
		}
	}
	SPI_W5500_CS_HIGH(); //置W5500的SCS为高电平

	offset1+=size;//更新实际物理地址,即下次写待发送数据到发送数据缓冲区的起始地址
	Write_W5500_SOCK_2Byte(s, Sn_TX_WR, offset1);
	Write_W5500_SOCK_1Byte(s, Sn_CR, SEND);//发送启动发送命令				
}

/*******************************************************************************
* 函数名  : W5500_Hardware_Reset
* 描述    : 硬件复位W5500
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : W5500的复位引脚保持低电平至少500us以上,才能重围W5500
*******************************************************************************/
//void W5500_Hardware_Reset(void)
//{
//	GPIO_ResetBits(W5500_RST_PORT, W5500_RST);//复位引脚拉低
//	delay_1ms(50);
//	GPIO_SetBits(W5500_RST_PORT, W5500_RST);//复位引脚拉高
//	delay_1ms(200);
//	while((Read_W5500_1Byte(PHYCFGR)&LINK)==0);//等待以太网连接完成
//}

/*******************************************************************************
* 函数名  : W5500_Init
* 描述    : 初始化W5500寄存器函数
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 在使用W5500之前，先对W5500初始化
*******************************************************************************/
void W5500_Init(void)
{
	uint8_t i=0;

	Write_W5500_1Byte(MR, RST);//软件复位W5500,置1有效,复位后自动清0
//	delay_1ms(10);//延时10ms,自己定义该函数

	//设置网关(Gateway)的IP地址,Gateway_IP为4字节unsigned char数组,自己定义 
	//使用网关可以使通信突破子网的局限，通过网关可以访问到其它子网或进入Internet
	Write_W5500_nByte(GAR, Gateway_IP, 4);
			
	//设置子网掩码(MASK)值,SUB_MASK为4字节unsigned char数组,自己定义
	//子网掩码用于子网运算
	Write_W5500_nByte(SUBR,Sub_Mask,4);		
	
	//设置物理地址,PHY_ADDR为6字节unsigned char数组,自己定义,用于唯一标识网络设备的物理地址值
	//该地址值需要到IEEE申请，按照OUI的规定，前3个字节为厂商代码，后三个字节为产品序号
	//如果自己定义物理地址，注意第一个字节必须为偶数
	Write_W5500_nByte(SHAR,Phy_Addr,6);		

	//设置本机的IP地址,IP_ADDR为4字节unsigned char数组,自己定义
	//注意，网关IP必须与本机IP属于同一个子网，否则本机将无法找到网关
	Write_W5500_nByte(SIPR,IP_Addr,4);		
	
	//设置发送缓冲区和接收缓冲区的大小，参考W5500数据手册
	for(i=0;i<8;i++)
	{
		Write_W5500_SOCK_1Byte(i,Sn_RXBUF_SIZE, 0x02);//Socket Rx memory size=2k
		Write_W5500_SOCK_1Byte(i,Sn_TXBUF_SIZE, 0x02);//Socket Tx mempry size=2k
	}

	//设置重试时间，默认为2000(200ms) 
	//每一单位数值为100微秒,初始化时值设为2000(0x07D0),等于200毫秒
	Write_W5500_2Byte(RTR, 0x07d0);

	//设置重试次数，默认为8次 
	//如果重发的次数超过设定值,则产生超时中断(相关的端口中断寄存器中的Sn_IR 超时位(TIMEOUT)置“1”)
	Write_W5500_1Byte(RCR,8);
}

/*******************************************************************************
* 函数名  : Detect_Gateway
* 描述    : 检查网关服务器
* 输入    : 无
* 输出    : 无
* 返回值  : 成功返回TRUE(0xFF),失败返回FALSE(0x00)
* 说明    : 无
*******************************************************************************/
unsigned char Detect_Gateway(void)
{
	unsigned char ip_adde[4];
	ip_adde[0]=IP_Addr[0]+1;
	ip_adde[1]=IP_Addr[1]+1;
	ip_adde[2]=IP_Addr[2]+1;
	ip_adde[3]=IP_Addr[3]+1;

	//检查网关及获取网关的物理地址
	Write_W5500_SOCK_4Byte(0,Sn_DIPR,ip_adde);//向目的地址寄存器写入与本机IP不同的IP值
	Write_W5500_SOCK_1Byte(0,Sn_MR,MR_TCP);//设置socket为TCP模式
	Write_W5500_SOCK_1Byte(0,Sn_CR,OPEN);//打开Socket	
//	delay_1ms(5);//延时5ms 	
	
	if(Read_W5500_SOCK_1Byte(0,Sn_SR) != SOCK_INIT)//如果socket打开失败
	{
		Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//打开不成功,关闭Socket
		return FALSE;//返回FALSE(0x00)
	}

	Write_W5500_SOCK_1Byte(0,Sn_CR,CONNECT);//设置Socket为Connect模式						

	do
	{
		uint8_t j=0;
		j=Read_W5500_SOCK_1Byte(0,Sn_IR);//读取Socket0中断标志寄存器
		if(j!=0)
		Write_W5500_SOCK_1Byte(0,Sn_IR,j);
//		delay_1ms(5);//延时5ms 
		if((j&IR_TIMEOUT) == IR_TIMEOUT)
		{
			return FALSE;	
		}
		else if(Read_W5500_SOCK_1Byte(0,Sn_DHAR) != 0xff)
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//关闭Socket
			return TRUE;							
		}
	}while(1);
}

/*******************************************************************************
* 函数名  : Socket_Init
* 描述    : 指定Socket(0~7)初始化
* 输入    : s:待初始化的端口
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Socket_Init(SOCKET s)
{
	//设置分片长度，参考W5500数据手册，该值可以不修改	
	Write_W5500_SOCK_2Byte(0, Sn_MSSR, 1460);//最大分片字节数=1460(0x5b4)
	//设置指定端口
	switch(s)
	{
		case 0:
			//设置端口0的端口号
			Write_W5500_SOCK_2Byte(0, Sn_PORT, S0_Port[0]*256+S0_Port[1]);
			//设置端口0目的(远程)端口号
			Write_W5500_SOCK_2Byte(0, Sn_DPORTR, S0_DPort[0]*256+S0_DPort[1]);
			//设置端口0目的(远程)IP地址
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
* 函数名  : Socket_Connect
* 描述    : 设置指定Socket(0~7)为客户端与远程服务器连接
* 输入    : s:待设定的端口
* 输出    : 无
* 返回值  : 成功返回TRUE(0xFF),失败返回FALSE(0x00)
* 说明    : 当本机Socket工作在客户端模式时,引用该程序,与远程服务器建立连接
*			如果启动连接后出现超时中断，则与服务器连接失败,需要重新调用该程序连接
*			该程序每调用一次,就与服务器产生一次连接
*******************************************************************************/
unsigned char Socket_Connect(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_TCP);//设置socket为TCP模式
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//打开Socket
//	delay_1ms(5);//延时5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_INIT)//如果socket打开失败
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//打开不成功,关闭Socket
		return FALSE;//返回FALSE(0x00)
	}
	Write_W5500_SOCK_1Byte(s,Sn_CR,CONNECT);//设置Socket为Connect模式
	return TRUE;//返回TRUE,设置成功
}

/*******************************************************************************
* 函数名  : Socket_Listen
* 描述    : 设置指定Socket(0~7)作为服务器等待远程主机的连接
* 输入    : s:待设定的端口
* 输出    : 无
* 返回值  : 成功返回TRUE(0xFF),失败返回FALSE(0x00)
* 说明    : 当本机Socket工作在服务器模式时,引用该程序,等等远程主机的连接
*			该程序只调用一次,就使W5500设置为服务器模式
*******************************************************************************/

unsigned char Socket_Listen(SOCKET s)
{

	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_TCP);//设置socket为TCP模式 
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//打开Socket	
//	delay_1ms(5);//延时5ms
	if((Read_W5500_SOCK_1Byte(s,Sn_SR))!=SOCK_INIT)//如果socket打开失败
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//打开不成功,关闭Socket
		return FALSE;//返回FALSE(0x00)
	}	
	Write_W5500_SOCK_1Byte(s,Sn_CR,LISTEN);//设置Socket为侦听模式	
//	delay_1ms(5);//延时5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_LISTEN)//如果socket设置失败
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//设置不成功,关闭Socket
		return FALSE;//返回FALSE(0x00)
	}

	return TRUE;

	//至此完成了Socket的打开和设置侦听工作,至于远程客户端是否与它建立连接,则需要等待Socket中断，
	//以判断Socket的连接是否成功。参考W5500数据手册的Socket中断状态
	//在服务器侦听模式不需要设置目的IP和目的端口号
}

/*******************************************************************************
* 函数名  : Socket_UDP
* 描述    : 设置指定Socket(0~7)为UDP模式
* 输入    : s:待设定的端口
* 输出    : 无
* 返回值  : 成功返回TRUE(0xFF),失败返回FALSE(0x00)
* 说明    : 如果Socket工作在UDP模式,引用该程序,在UDP模式下,Socket通信不需要建立连接
*			该程序只调用一次，就使W5500设置为UDP模式
*******************************************************************************/
unsigned char Socket_UDP(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_UDP);//设置Socket为UDP模式*/
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//打开Socket*/
//	delay_1ms(5);//延时5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_UDP)//如果Socket打开失败
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//打开不成功,关闭Socket
		return FALSE;//返回FALSE(0x00)
	}
	else
		return TRUE;

	//至此完成了Socket的打开和UDP模式设置,在这种模式下它不需要与远程主机建立连接
	//因为Socket不需要建立连接,所以在发送数据前都可以设置目的主机IP和目的Socket的端口号
	//如果目的主机IP和目的Socket的端口号是固定的,在运行过程中没有改变,那么也可以在这里设置
}

/*******************************************************************************
* 函数名  : W5500_Interrupt_Process
* 描述    : W5500中断处理程序框架
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void W5500_Interrupt_Process(void)
{
	unsigned char i,j;

	IntDispose:

	i=Read_W5500_1Byte(SIR);//读取端口中断标志寄存器	
	if((i & S0_INT) == S0_INT)//Socket0事件处理 
	{
		j=Read_W5500_SOCK_1Byte(0,Sn_IR);//读取Socket0中断标志寄存器
		Write_W5500_SOCK_1Byte(0,Sn_IR,j);
		if(j&IR_CON)//在TCP模式下,Socket0成功连接 
		{
			S0_State|=S_CONN;//网络连接状态0x02,端口完成连接，可以正常传输数据
		}
		if(j&IR_DISCON)//在TCP模式下Socket断开连接处理
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//关闭端口,等待重新打开连接 
			Socket_Init(0);		//指定Socket(0~7)初始化,初始化端口0
			S0_State=0;//网络连接状态0x00,端口连接失败
		}
		if(j&IR_SEND_OK)//Socket0数据发送完成,可以再次启动S_tx_process()函数发送数据 
		{
			S0_Data|=S_TRANSMITOK;//端口发送一个数据包完成 
		}
		if(j&IR_RECV)//Socket接收到数据,可以启动S_rx_process()函数 
		{
			S0_Data|=S_RECEIVE;//端口接收到一个数据包
		}
		if(j&IR_TIMEOUT)//Socket连接或数据传输超时处理 
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);// 关闭端口,等待重新打开连接 			
			S0_State=0;//网络连接状态0x00,端口连接失败
		}
	}

	if(Read_W5500_1Byte(SIR) != 0) 
		goto IntDispose;
}

unsigned int Timer2_Counter=0; //Timer2定时器计数变量(ms)
unsigned int W5500_Send_Delay_Counter=0; //W5500发送延时计数变量(ms)

/*******************************************************************************
* 函数名  : W5500_Initialization
* 描述    : W5500初始货配置
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void W5500_Initialization(void)
{
	W5500_Init();		//初始化W5500寄存器函数
	Detect_Gateway();	//检查网关服务器 
	Socket_Init(0);		//指定Socket(0~7)初始化,初始化端口0
	Socket_Init(1);		//指定Socket(0~7)初始化,初始化端口0
	Socket_Init(2);		//指定Socket(0~7)初始化,初始化端口0
	Socket_Init(3);		//指定Socket(0~7)初始化,初始化端口0
	Socket_Init(4);		//指定Socket(0~7)初始化,初始化端口0
	Socket_Init(5);		//指定Socket(0~7)初始化,初始化端口0
}
uint16_t 	ip_localAddr[4]={192,168,1,200};
extern uint32_t mcuID[3];
/*******************************************************************************
* 函数名  : Load_Net_Parameters
* 描述    : 装载网络参数
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 网关、掩码、物理地址、本机IP地址、端口号、目的IP地址、目的端口号、端口工作模式
*******************************************************************************/
void Load_Net_Parameters(void)
{
	Gateway_IP[0] = 192;//加载网关参数
	Gateway_IP[1] = 168;
	Gateway_IP[2] = 1;
	Gateway_IP[3] = 1;

	Sub_Mask[0]=255;//加载子网掩码
	Sub_Mask[1]=255;
	Sub_Mask[2]=255;
	Sub_Mask[3]=0;

	Phy_Addr[0]=0x0c;//加载物理地址
	Phy_Addr[1]=0x29;
	Phy_Addr[2]=0xab;
	Phy_Addr[3]=mcuID[2]&0xff;
	Phy_Addr[4]=mcuID[1]&0xff;
	Phy_Addr[5]=mcuID[0]&0xff;

	IP_Addr[0]=ip_localAddr[0];//加载本机IP地址
	IP_Addr[1]=ip_localAddr[1];
	IP_Addr[2]=ip_localAddr[2];
	IP_Addr[3]=ip_localAddr[3];

//加载各端口IP*****************************************************************/
	S0_DIP[0] =192;//加载端口0的目的IP地址
	S0_DIP[1] =168;
	S0_DIP[2] = 1;
	S0_DIP[3] = 128;
	
	S1_DIP[0] =192;//加载端口0的目的IP地址
	S1_DIP[1] =168;
	S1_DIP[2] = 1;
	S1_DIP[3] = 190;
	
	S2_DIP[0] =192;//加载端口0的目的IP地址
	S2_DIP[1] =168;
	S2_DIP[2] = 1;
	S2_DIP[3] = 190;
	
	S3_DIP[0] =192;//加载端口0的目的IP地址
	S3_DIP[1] =168;
	S3_DIP[2] = 1;
	S3_DIP[3] = 190;
	
	S4_DIP[0] =192;//加载端口0的目的IP地址
	S4_DIP[1] =168;
	S4_DIP[2] = 1;
	S4_DIP[3] = 190;
	
	S5_DIP[0] =192;//加载端口0的目的IP地址
	S5_DIP[1] =168;
	S5_DIP[2] = 1;
	S5_DIP[3] = 190;
//加载各端口号*****************************************************************/
	S0_Port[0] =0x01;//加载端口0的端口号
	S0_Port[1] =0xf6;
	S1_Port[0] = 0x13;//加载端口1的端口号
	S1_Port[1] = 0x89;
	S2_Port[0] = 0x13;//加载端口2的端口号
	S2_Port[1] = 0x8a;
	S3_Port[0] = 0x13;//加载端口3的端口号
	S3_Port[1] = 0x8b;
	S4_Port[0] = 0x13;//加载端口4的端口号
	S4_Port[1] = 0x8c;
	S5_Port[0] = 0x13;//加载端口5的端口号
	S5_Port[1] = 0x8d;
	
//加载目的端口号****************************************************************/	
	S0_DPort[0] = 0x22;//加载端口0的目的端口号6000
	S0_DPort[1] = 0xc3;
	S1_DPort[0] = 0x17;//加载端口1的目的端口号6001
	S1_DPort[1] = 0x71;
	S2_DPort[0] = 0x17;//加载端口2的目的端口号6002
	S2_DPort[1] = 0x72;
	S3_DPort[0] = 0x17;//加载端口3的目的端口号6003
	S3_DPort[1] = 0x73;
	S4_DPort[0] = 0x17;//加载端口4的目的端口号6004
	S4_DPort[1] = 0x74;
	S5_DPort[0] = 0x17;//加载端口5的目的端口号6005
	S5_DPort[1] = 0x75;

//加载端口工作模***************************************************************/	
	S0_Mode=TCP_CLIENT;//加载端口0的工作模式,TCP服务端模式
	S1_Mode=TCP_SERVER;
	S2_Mode=TCP_SERVER;
	S3_Mode=TCP_SERVER;
	S4_Mode=TCP_SERVER;
	S5_Mode=TCP_SERVER;
	
}

/*******************************************************************************
* 函数名  : W5500_Socket_Set
* 描述    : W5500端口初始化配置
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 分别设置4个端口,根据端口工作模式,将端口置于TCP服务器、TCP客户端或UDP模式.
*			从端口状态字节Socket_State可以判断端口的工作情况
*******************************************************************************/
void W5500_Socket_Set(void)
{
	if(S0_State==0)//端口0初始化配置
	{
		if(S0_Mode==TCP_SERVER)//TCP服务器模式 
		{
			if(Socket_Listen(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else if(S0_Mode==TCP_CLIENT)//TCP客户端模式 
		{
			if(Socket_Connect(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else//UDP模式 
		{
			if(Socket_UDP(0)==TRUE)
				S0_State=S_INIT|S_CONN;
			else
				S0_State=0;
		}
	}
	
	if(S1_State==0)//端口1初始化配置
	{
		if(S1_Mode==TCP_SERVER)//TCP服务器模式 
		{
			if(Socket_Listen(1)==TRUE)
				S1_State=S_INIT;
			else
				S1_State=0;
		}
		else if(S1_Mode==TCP_CLIENT)//TCP客户端模式 
		{
			if(Socket_Connect(1)==TRUE)
				S1_State=S_INIT;
			else
				S1_State=0;
		}
		else//UDP模式 
		{
			if(Socket_UDP(1)==TRUE)
				S1_State=S_INIT|S_CONN;
			else
				S1_State=0;
		}
	}
	
	if(S2_State==0)//端口2初始化配置
	{
		if(S2_Mode==TCP_SERVER)//TCP服务器模式 
		{
			if(Socket_Listen(2)==TRUE)
				S2_State=S_INIT;
			else
				S2_State=0;
		}
		else if(S2_Mode==TCP_CLIENT)//TCP客户端模式 
		{
			if(Socket_Connect(2)==TRUE)
				S2_State=S_INIT;
			else
				S2_State=0;
		}
		else//UDP模式 
		{
			if(Socket_UDP(2)==TRUE)
				S2_State=S_INIT|S_CONN;
			else
				S2_State=0;
		}
	}
	
	if(S3_State==0)//端口3初始化配置
	{
		if(S3_Mode==TCP_SERVER)//TCP服务器模式 
		{
			if(Socket_Listen(3)==TRUE)
				S3_State=S_INIT;
			else
				S3_State=0;
		}
		else if(S3_Mode==TCP_CLIENT)//TCP客户端模式 
		{
			if(Socket_Connect(3)==TRUE)
				S3_State=S_INIT;
			else
				S3_State=0;
		}
		else//UDP模式 
		{
			if(Socket_UDP(3)==TRUE)
				S3_State=S_INIT|S_CONN;
			else
				S3_State=0;
		}
	}
	
  if(S4_State==0)//端口4初始化配置
	{
		if(S4_Mode==TCP_SERVER)//TCP服务器模式 
		{
			if(Socket_Listen(4)==TRUE)
				S4_State=S_INIT;
			else
				S4_State=0;
		}
		else if(S4_Mode==TCP_CLIENT)//TCP客户端模式 
		{
			if(Socket_Connect(4)==TRUE)
				S4_State=S_INIT;
			else
				S4_State=0;
		}
		else//UDP模式 
		{
			if(Socket_UDP(4)==TRUE)
				S4_State=S_INIT|S_CONN;
			else
				S4_State=0;
		}
	}
	
	if(S5_State==0)//端口4初始化配置
	{
		if(S5_Mode==TCP_SERVER)//TCP服务器模式 
		{
			if(Socket_Listen(5)==TRUE)
				S5_State=S_INIT;
			else
				S5_State=0;
		}
		else if(S5_Mode==TCP_CLIENT)//TCP客户端模式 
		{
			if(Socket_Connect(5)==TRUE)
				S5_State=S_INIT;
			else
				S5_State=0;
		}
		else//UDP模式 
		{
			if(Socket_UDP(5)==TRUE)
				S5_State=S_INIT|S_CONN;
			else
				S5_State=0;
		}
	}
}

/*******************************************************************************
* 函数名  : Process_Socket_Data
* 描述    : W5500接收并发送接收到的数据
* 输入    : s:端口号
* 输出    : 无
* 返回值  : 无
* 说明    : 本过程先调用S_rx_process()从W5500的端口接收数据缓冲区读取数据,
*			然后将读取的数据从Rx_Buffer拷贝到Temp_Buffer缓冲区进行处理。
*			处理完毕，将数据从Temp_Buffer拷贝到Tx_Buffer缓冲区。调用S_tx_process()
*			发送数据。
*******************************************************************************/
void Process_Socket_Data(SOCKET s)
{
	unsigned short size;
	size=Read_SOCK_Data_Buffer(s, Rx_Buffer);
	usart2_sendstr((char *)Rx_Buffer);
}
/*******************************************************************************
* 函数名  : WEB()
* 描述    : 网络通讯函数
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void WEB()
{
	uint16_t AnsLen=0;
	W5500_Socket_Set();//W5500端口初始化配置
	 
	W5500_Interrupt_Process();//W5500中断处理程序框架
	
	if((S0_Data & S_RECEIVE) == S_RECEIVE)//如果Socket0接收到数据
	{   
		 
			S0_Data&=~S_RECEIVE;
			Process_Socket_Data(0);//W5500接收
			if(S0_State == (S_INIT|S_CONN))//发送
			{
				
				S0_Data&=~S_TRANSMITOK;
			  if((Rx_Buffer[9]==0x13)&&(Rx_Buffer[10]==0xf8))
				{
					//WriteConfigToAT24Cxx();   //写配置
					//Write_SOCK_Data_Buffer(1, succeed,sizeof(succeed));//指定Socket(0~7)发送数据处理,端口1发送20字节数据
				}
				else
				{
					
//					AnsLen = SpliceModbus();    //拼接Modbus协议
//					Write_SOCK_Data_Buffer(0, Tx_Buffer,AnsLen+9);//指定Socket(0~7)发送数据处理,端口0发送20字节数据
					
				}
			}
		}
	
		if((S1_Data & S_RECEIVE) == S_RECEIVE)//如果Socket1接收到数据
		{
			S1_Data&=~S_RECEIVE;
			Process_Socket_Data(1);//W5500接收
			if(S1_State == (S_INIT|S_CONN))//发送
			{
				S1_Data&=~S_TRANSMITOK;
				
				if((Rx_Buffer[9]==0x13)&&(Rx_Buffer[10]==0xf8))
				{
					//WriteConfigToAT24Cxx();   //写配置
					//Write_SOCK_Data_Buffer(1, succeed,sizeof(succeed));//指定Socket(0~7)发送数据处理,端口1发送20字节数据
				}
				else
				{
					//AnsLen = SpliceModbus();    //拼接Modbus协议
					Write_SOCK_Data_Buffer(1, Tx_Buffer,AnsLen+9);//指定Socket(0~7)发送数据处理,端口1发送20字节数据
				}
			}
		}
		
		if((S2_Data & S_RECEIVE) == S_RECEIVE)//如果Socket2接收到数据
		{
			S2_Data&=~S_RECEIVE;
			Process_Socket_Data(2);//W5500接收
			if(S2_State == (S_INIT|S_CONN))//发送
			{
				S2_Data&=~S_TRANSMITOK;
							
				if((Rx_Buffer[9]==0x13)&&(Rx_Buffer[10]==0xf8))
				{
					//WriteConfigToAT24Cxx();   //写配置
					//Write_SOCK_Data_Buffer(2, succeed,sizeof(succeed));//指定Socket(0~7)发送数据处理,端口2发送20字节数据
				}
				else
				{
					//AnsLen = SpliceModbus();    //拼接Modbus协议
					Write_SOCK_Data_Buffer(2, Tx_Buffer,AnsLen+9);//指定Socket(0~7)发送数据处理,端口2发送20字节数据
				}
			}
		}	
		
		if((S3_Data & S_RECEIVE) == S_RECEIVE)//如果Socket3接收到数据
		{
			S3_Data&=~S_RECEIVE;
			Process_Socket_Data(3);//W5500接收
			if(S3_State == (S_INIT|S_CONN))//发送
			{
				S3_Data&=~S_TRANSMITOK;
							
				if((Rx_Buffer[9]==0x13)&&(Rx_Buffer[10]==0xf8))
				{
					//WriteConfigToAT24Cxx();   //写配置
					//Write_SOCK_Data_Buffer(3, succeed,sizeof(succeed));//指定Socket(0~7)发送数据处理,端口3发送20字节数据
				}
				else
				{
					//AnsLen = SpliceModbus();    //拼接Modbus协议
					Write_SOCK_Data_Buffer(3, Tx_Buffer,AnsLen+9);//指定Socket(0~7)发送数据处理,端口3发送20字节数据
				}
			}
		}	
		
	if((S4_Data & S_RECEIVE) == S_RECEIVE)//如果Socket4接收到数据
	{
			S4_Data&=~S_RECEIVE;
			Process_Socket_Data(4);//W5500接收
			if(S4_State == (S_INIT|S_CONN))//发送
			{
				S4_Data&=~S_TRANSMITOK;
				
				if((Rx_Buffer[9]==0x13)&&(Rx_Buffer[10]==0xf8))
				{
					//WriteConfigToAT24Cxx();   //写配置
					//Write_SOCK_Data_Buffer(4, succeed,sizeof(succeed));//指定Socket(0~7)发送数据处理,端口4发送20字节数据
				}
				else
				{
					//AnsLen = SpliceModbus();    //拼接Modbus协议
					Write_SOCK_Data_Buffer(4, Tx_Buffer,AnsLen+9);//指定Socket(0~7)发送数据处理,端口4发送20字节数据
				}
			}
	}
	
	if((S5_Data & S_RECEIVE) == S_RECEIVE)//如果Socket5接收到数据
	{
		  
			S5_Data&=~S_RECEIVE;
			Process_Socket_Data(5);//W5500接收
			if(S5_State == (S_INIT|S_CONN))//发送
			{
				S5_Data&=~S_TRANSMITOK;
				if((Rx_Buffer[9]==0x13)&&(Rx_Buffer[10]==0xf8))
				{
					//WriteConfigToAT24Cxx();   //写配置
					//Write_SOCK_Data_Buffer(5, succeed,sizeof(succeed));//指定Socket(0~7)发送数据处理,端口5发送20字节数据
				}
				else
				{
					//AnsLen = SpliceModbus();    //拼接Modbus协议
					Write_SOCK_Data_Buffer(5, Tx_Buffer,AnsLen+9);//指定Socket(0~7)发送数据处理,端口5发送20字节数据
				}
			}
	}		
}
