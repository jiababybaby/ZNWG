#include "gd32f30x_fmc.h"
#include "flash.h"
#include "cJSON.h"
#include "string.h"
#define StartServerManageFlashAddress    ((uint32_t)0x0807F800)//��д��ʼ��ַ���ڲ�flash�����洢���ַ��0x08036000��ʼ��
#define FLASH_SIZE 512 //��ѡMCU��FLASH������С(��λΪK)
 

#define SECTOR_SIZE 2048  //�ֽ�

extern uint8_t mode;
extern uint8_t mesh_id[10];
uint8_t canshu_get_from_Json(char *data){
		uint8_t ret_val=0;
		cJSON *json_root  = NULL;
		cJSON *json_mode  = NULL;
		cJSON *json_mesh_id  = NULL;
		json_root = cJSON_Parse((char *)data);
    
		json_mode=cJSON_GetObjectItem(json_root, "mode");
		if(json_mode){
			ret_val++;
				mode=(uint8_t)json_mode->valueint;
				if(mode==0){
					
						//("wifi mode ");
				}
				if(mode==1){
						//("eth mode ");
				}
				if(mode==2){
						//("4G mode ");
				}
		}
//		json_mesh_id=cJSON_GetObjectItem(json_root, "mesh_id");
//		if(json_mesh_id){
//			ret_val++;
//			strcpy((char *)mesh_id,json_mesh_id->valuestring);
//		}
		if(!cJSON_IsInvalid(json_root)){
			cJSON_Delete(json_root);
		}
//		if(!cJSON_IsInvalid(json_mode)){
//			cJSON_Delete(json_mode);
//		}
//		if(!cJSON_IsInvalid(json_mesh_id)){
//			cJSON_Delete(json_mesh_id);
//		}
		
		return ret_val;
}

void canshu_write_to_Jsonstr(char *data){
		char *send_temp=NULL;
		cJSON *json_write  = NULL;
		json_write = cJSON_CreateObject();
    cJSON_AddItemToObject(json_write, "mode", cJSON_CreateNumber(mode));
		cJSON_AddItemToObject(json_write, "mesh_id", cJSON_CreateString((char *)mesh_id));
		send_temp=cJSON_Print(json_write);
		//strcpy(send_temp,(const char *)cJSON_Print(json_root));
		memcpy(data,send_temp,strlen(send_temp));
		if(!cJSON_IsInvalid(json_write)){
			cJSON_Delete(json_write);
		}
		free(send_temp);
}
//��ָ����ַ��ʼд��������
void FLASH_WriteMoreData(uint32_t startAddress,uint16_t *writeData,uint16_t countToWrite)
{    
	uint32_t offsetAddress=startAddress - FLASH_BASE;               //����ȥ��0X08000000���ʵ��ƫ�Ƶ�ַ
	uint32_t sectorPosition=offsetAddress/SECTOR_SIZE;            //����������ַ������GD32F305ZET6Ϊ0~255
	uint32_t sectorStartAddress=sectorPosition*SECTOR_SIZE+FLASH_BASE;    //��Ӧ�������׵�ַ
	uint16_t dataIndex;
	if(startAddress<FLASH_BASE||((startAddress+countToWrite*2)>=(FLASH_BASE + SECTOR_SIZE * FLASH_SIZE)))
	{
		return;//�Ƿ���ַ
	}
	fmc_bank0_unlock();         //����д����

	fmc_page_erase(sectorStartAddress);//�����������

	for(dataIndex=0;dataIndex<countToWrite;dataIndex++)
	{
		fmc_halfword_program(startAddress+dataIndex*2,writeData[dataIndex]);
	}

	fmc_bank0_lock();//����д����
}
 
//��ȡָ����ַ�İ���(16λ����)
uint16_t FLASH_ReadHalfWord(uint32_t address)
{
	return *(__IO uint16_t*)address; 
}
 
//��ָ����ַ��ʼ��ȡ�������
void FLASH_ReadMoreData(uint32_t startAddress,uint16_t *readData,uint16_t countToRead)
{
	uint16_t dataIndex;
	for(dataIndex=0;dataIndex<countToRead;dataIndex++)
	{
		readData[dataIndex]=FLASH_ReadHalfWord(startAddress+dataIndex*2);
	}
}
void write_to_flash(uint8_t *ptr)
{
	//uint8_t temp[5]="1234";
	//uint16_t count_len = sizeof(ptr) / 2;
	FLASH_WriteMoreData(StartServerManageFlashAddress,(uint16_t *)ptr,strlen((const char *)ptr));
}
void read_from_flash(uint8_t *ptr)
{
	//uint16_t count_len = sizeof(ptr)  / 2;
	FLASH_ReadMoreData(StartServerManageFlashAddress,(uint16_t *)ptr,canshu_max_size);
}
