#include "gd32f30x_fmc.h"
#include "flash.h"
#include "cJSON.h"
#include "string.h"
#define StartServerManageFlashAddress    ((uint32_t)0x0807F800)//读写起始地址（内部flash的主存储块地址从0x08036000开始）
#define FLASH_SIZE 512 //所选MCU的FLASH容量大小(单位为K)
 

#define SECTOR_SIZE 2048  //字节

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
//从指定地址开始写入多个数据
void FLASH_WriteMoreData(uint32_t startAddress,uint16_t *writeData,uint16_t countToWrite)
{    
	uint32_t offsetAddress=startAddress - FLASH_BASE;               //计算去掉0X08000000后的实际偏移地址
	uint32_t sectorPosition=offsetAddress/SECTOR_SIZE;            //计算扇区地址，对于GD32F305ZET6为0~255
	uint32_t sectorStartAddress=sectorPosition*SECTOR_SIZE+FLASH_BASE;    //对应扇区的首地址
	uint16_t dataIndex;
	if(startAddress<FLASH_BASE||((startAddress+countToWrite*2)>=(FLASH_BASE + SECTOR_SIZE * FLASH_SIZE)))
	{
		return;//非法地址
	}
	fmc_bank0_unlock();         //解锁写保护

	fmc_page_erase(sectorStartAddress);//擦除这个扇区

	for(dataIndex=0;dataIndex<countToWrite;dataIndex++)
	{
		fmc_halfword_program(startAddress+dataIndex*2,writeData[dataIndex]);
	}

	fmc_bank0_lock();//上锁写保护
}
 
//读取指定地址的半字(16位数据)
uint16_t FLASH_ReadHalfWord(uint32_t address)
{
	return *(__IO uint16_t*)address; 
}
 
//从指定地址开始读取多个数据
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
