#include "MODBUS_RTU.h"
#include "main.h"
#include "sc16is7xx.h"
#include "modbus.h"
#include "MODBUS_func4.h"
#include "MODBUS_func3.h"
#include "LPC17xx.H"
#include "25lc640.h"
#include "eeprom_map.h"
#include "gran.h"

unsigned char NULL_0=0;
unsigned char mb_rtu_func;
unsigned long mb_rtu_start_adr;
unsigned char mb_rtu_num, mb_rtu_num_send;
unsigned short mb_data_1, mb_data_2, crc_f;
//-----------------------------------------------
// вычисление CRC побайтно, результат в unsigned short crc_f
//перед вычислением сделать crc_f=0xFFFF;
void crc_calc_f( unsigned short data)
{
  short i;
  crc_f ^= data;
  for ( i = 8; i != 0; i--){ 
      	if ((crc_f & 0x0001) != 0) { crc_f >>= 1; crc_f ^= 0xA001;}
      	else  crc_f >>= 1;
  }
  
}
//-----------------------------------------------

void modbus_puts (void) {
	mb_data_1=((unsigned short)modbus_rx_buffer[2]<<8)+modbus_rx_buffer[3];
	mb_data_2=((unsigned short)modbus_rx_buffer[4]<<8)+modbus_rx_buffer[5];
	mb_rtu_func=modbus_rx_buffer[1];
/*	if(mb_rtu_func==4){
		if(mb_data_2<=127 && mb_data_2>0){
			wr_reg_func4();
			mb_rtu_start_adr=mb_data_1;
			mb_rtu_start_adr=mb_rtu_start_adr*2;  //адрес в таблице			
			mb_rtu_num=(unsigned char)(mb_data_2<<1);
			if((mb_rtu_start_adr+mb_rtu_num) <=0x20000UL){
				mb_rtu_num_send=0;
			}
			else mb_rtu_num=0;
		} 
	}  */
	if(mb_rtu_func==6){
		sc16is700RecieveDisableFlag=1; //запретить прием во время отправки
	 	sc16is700_wr_buff_ptr(CS16IS7xx_THR, modbus_rx_buffer, 8);//отправить подтверждение получения
	 	analiz_func6(mb_data_1, mb_data_2);				
	}
	else if(mb_rtu_func==3 || mb_rtu_func==4){
		if(mb_data_2<=127 && mb_data_2>0){
			mb_rtu_start_adr=mb_data_1;
			mb_rtu_start_adr=mb_rtu_start_adr*2;  //адрес в таблице			
			mb_rtu_num=(unsigned char)(mb_data_2<<1);
			if((mb_rtu_start_adr+mb_rtu_num) <=0x20000UL){
				mb_rtu_num_send=0;
		   		if(mb_rtu_func==3) wr_reg_func3(mb_rtu_start_adr, mb_rtu_start_adr+mb_rtu_num);
				if(mb_rtu_func==4) wr_reg_func4();
			}
			else mb_rtu_num=0;
		} 
	}
}
//-----------

//-----------

void analiz_func6(unsigned short mbadr, unsigned short mbdat){
			if(mbadr==11)		//Установка времени 
				{
				gran(&mbdat,0,99);
				LPC_RTC->YEAR=(uint16_t)mbdat;
				}
			else if(mbadr==12)		//Установка времени 
				{
				gran(&mbdat,1,12);
				LPC_RTC->MONTH=(uint16_t)mbdat;
				}
			else if(mbadr==13)		//Установка времени 
				{
				gran(&mbdat,1,31);
				LPC_RTC->DOM=(uint16_t)mbdat;
				}
			else if(mbadr==14)		//Установка времени 
				{
				gran(&mbdat,0,23);
				LPC_RTC->HOUR=(uint16_t)mbdat;
				}
			else if(mbadr==15)		//Установка времени 
				{
				gran(&mbdat,0,60);
				LPC_RTC->MIN=(uint16_t)mbdat;
				}
			else if(mbadr==16)		//Установка времени 
				{
				gran(&mbdat,0,60);
				LPC_RTC->SEC=(uint16_t)mbdat;
				}
			else if(mbadr==20)		//ток стабилизации для режима стабилизации тока
				{
			//	if(mbdat<=18)
			//	lc640_write_int(EE_NUMIST,mbdat);  
				}
			else if(mbadr==21)		//Рег21  Звуковая аварийная сигнализация вкл./выкл.
				{
				gran(&mbdat,0,1);
				lc640_write_int(EE_ZV_ON,mbdat);  
				}
			else if(mbadr==22)		//Рег22  Отключение аварийного сигнала 0-ручн., 1-автом.
				{
				gran(&mbdat,0,1);
				lc640_write_int(EE_AV_OFF_AVT,mbdat);  
				}
			else if(mbadr==23)		//Рег23 Уставка выходного напряжения,  1В
				{
				gran(&mbdat,220,230);
				lc640_write_int(EE_U_OUT_SET,mbdat);			
				}
			else if(mbadr==24)		//Рег24 Уставка максимального (аварийного) выходного напряжения,  1В
				{
				gran(&mbdat,240,270);
				lc640_write_int(EE_U_OUT_MAX,mbdat);			
				}
			else if(mbadr==25)		//Рег25 Уставка минимального (аварийного) выходного напряжения,  1В
				{
				gran(&mbdat,0,200);
				lc640_write_int(EE_U_OUT_MIN,mbdat);			
				}
			else if(mbadr==26)		//Рег26 Уставка устранения фиксации заниженного напряжения по входу сеть или входу инв или выходу байпаса (активно только совместно с уставкой UacTurn...),  1В
				{
				gran(&mbdat,0,300);
				lc640_write_int(EE_U_NET_ON_MIN,mbdat);			
				}
			else if(mbadr==27)		//Рег27 Уставка фиксации заниженного напряжения по входу сеть или входу инв или выходу байпаса (активно только совместно с уставкой UacTurn...),  1В
				{
				gran(&mbdat,0,300);
				lc640_write_int(EE_U_NET_OFF_MIN,mbdat);			
				}
			else if(mbadr==28)		//Рег28 Уставка устранения фиксации завышенного напряжения по входу сеть или входу инв или выходу байпаса (активно только совместно с уставкой UacTurn... и только при скорости КАН 125кбит/с),  1В
				{
				gran(&mbdat,0,300);
				lc640_write_int(EE_U_NET_ON_MAX,mbdat);			
				}
			else if(mbadr==29)		//Рег29 Уставка фиксации завышенного напряжения по входу сеть или входу инв или выходу байпаса (активно только совместно с уставкой UacTurn... и только при скорости КАН 125кбит/с),  1В
				{
				gran(&mbdat,0,300);
				lc640_write_int(EE_U_NET_OFF_MAX,mbdat);			
				}
			else if(mbadr==30)		//Рег30	 Напряжение батареи включения,  1В
				{
				short temp_min=0,temp_max=300,temp_d=1;
				if(AUSW_MAIN==24)
				 	{
					temp_min=22,temp_max=26,temp_d=1;
					}
				else if(AUSW_MAIN==4860)
				 	{
					temp_min=42,temp_max=52,temp_d=3;
					}
				else if(AUSW_MAIN==110)
				 	{
					temp_min=83,temp_max=113,temp_d=5;
					}
				else if(AUSW_MAIN==220)
				 	{
					temp_min=175,temp_max=300,temp_d=1;
					}
				gran(&mbdat,temp_min,temp_max);
				//gran(&snmp_u_bat_on,snmp_u_bat_off+5,temp_max);
				lc640_write_int(EE_U_BAT_MAX,mbdat);			
				}
			else if(mbadr==31)		//Рег31	 Напряжение батареи выключения,  1В
				{
				short temp_min=0,temp_max=300,temp_d=1;
				if(AUSW_MAIN==24)
				 	{
					temp_min=20,temp_max=24,temp_d=1;
					}
				else if(AUSW_MAIN==4860)
				 	{
					temp_min=40,temp_max=50,temp_d=3;
					}
				else if(AUSW_MAIN==110)
				 	{
					temp_min=80,temp_max=110,temp_d=5;
					}
				else if(AUSW_MAIN==220)
				 	{
					temp_min=170,temp_max=300,temp_d=1;
					}
				
				gran(&mbdat,temp_min,temp_max);
				//gran(&snmp_u_bat_off,10,snmp_u_bat_on-5);
				lc640_write_int(EE_U_BAT_MIN,mbdat);		
				}
			else if(mbadr==32)		//Рег32	 Напряжение выхода максимальное (аварийное),  1В
				{
				gran(&mbdat,20,300);
				lc640_write_int(EE_U_OUT_AC_MAX_AV,mbdat);
				}
			else if(mbadr==33)		//Рег33	 Напряжение выхода минимальное (аварийное),  1В
				{
				gran(&mbdat,20,300);
				lc640_write_int(EE_U_OUT_AC_MIN_AV,mbdat);
				}
			else if(mbadr==34)		//Рег34	 Напряжение входа AC максимальное (аварийное),  1В
				{
				gran(&mbdat,20,300);
				lc640_write_int(EE_U_IN_AC_MAX_AV,mbdat);
				}
			else if(mbadr==35)		//Рег35	 Напряжение входа AC минимальное (аварийное),  1В
				{
				gran(&mbdat,20,300);
				lc640_write_int(EE_U_IN_AC_MIN_AV,mbdat);
				}
			else if(mbadr==36)		//Рег36	 Напряжение входа DC максимальное (аварийное),  1В
				{
				gran(&mbdat,20,300);
				lc640_write_int(EE_U_IN_DC_MAX_AV,mbdat);
				}
			else if(mbadr==37)		//Рег37	 Напряжение входа DC минимальное (аварийное),  1В
				{
				gran(&mbdat,20,300);
				lc640_write_int(EE_U_IN_DC_MIN_AV,mbdat);
				}
			else if(mbadr==38)		//Рег38	 Битовая маска срабатывания реле 1 по событиям
				{
			//	gran(&mbdat,20,300);
				lc640_write_int(EE_RELE_SET_MASK0,mbdat);
				}
			else if(mbadr==39)		//Рег39	 Битовая маска срабатывания реле 1 по событиям
				{
			//	gran(&mbdat,20,300);
				lc640_write_int(EE_RELE_SET_MASK1,mbdat);
				}
			else if(mbadr==48)		
				{
				modbus_log_ptr = mbdat;
				}

}

//----------------------------------------------- 

//Обработчик sc16is700
void sc16is700_hndl(void)
{

sc16is700ByteAvailable=sc16is700_rd_byte(CS16IS7xx_RXLVL); //Читаем состояние ФИФО приема микросхемы

if(sc16is700ByteAvailable) //Если в приемном ФИФО	микросхемы есть данные
	{
	char i;
	for(i=0;(i<sc16is700ByteAvailable)&&(i<5);i++) //Читаем их пачками не больше 5 в программный буфер модбас
		{
		if(!sc16is700RecieveDisableFlag) //если не идет передача данных
			{
				char zi;
				for(zi=1;zi<8;zi++) modbus_rx_buffer[zi-1]=modbus_rx_buffer[zi];
				modbus_rx_buffer[7]=sc16is700_rd_byte(CS16IS7xx_RHR);
				if(modbus_rx_buffer_ptr==8) modbus_rx_buffer_ptr=0; //если после запроса идут данные, то сброс и не высылать ответ
				if(modbus_rx_buffer[0]==MODBUS_ADRESS && (modbus_rx_buffer[1]==3 || modbus_rx_buffer[1]==4 || modbus_rx_buffer[1]==6) &&
					CRC16_2((char*)modbus_rx_buffer,6)==(((unsigned short)modbus_rx_buffer[7])<<8) + modbus_rx_buffer[6] ){    //*((short*)&modbus_rx_buffer[6]) ){
					modbus_timeout_cnt=0;   //Запускаем таймер задержки отправки посылки 
					modbus_rx_buffer_ptr=8;
				}		
			

		}
		else sc16is700_rd_byte(CS16IS7xx_RHR); //считываем данные, которые передали, попавшие в буфер приема.
		}
	}

if(mb_rtu_num!=0){
	sc16is700TxFifoLevel=sc16is700_rd_byte(CS16IS7xx_TXLVL);//Читаем сколько свободно в ФИФО передачи 
 	if(sc16is700TxFifoLevel>0){ //если есть свободное место в ФИФО передачи
	 unsigned char z=0;
	 if(mb_rtu_num_send==0){ //если начало передачи
	 	if(sc16is700TxFifoLevel==64){ //начинать, если буфер пустой
			sc16is700RecieveDisableFlag=1;
			crc_f=0xFFFF;
			sc16is700_spi_init();
			delay_us(2);
			sc16is700_CS_ON
			spi1((CS16IS7xx_THR&0x0f)<<3);
			spi1(MODBUS_ADRESS);
			crc_calc_f(MODBUS_ADRESS);
			spi1(mb_rtu_func);
			crc_calc_f(mb_rtu_func);
			spi1(mb_rtu_num);
			crc_calc_f(mb_rtu_num);
			z=0;
			while (mb_rtu_num_send<mb_rtu_num && z<61){
				unsigned char data_reg=0;
				unsigned long adrr_reg;
				adrr_reg=mb_rtu_start_adr+mb_rtu_num_send;
				if(mb_rtu_func==4){
					if(adrr_reg<MODBUS_FUNC_4_LENGTH) data_reg=*reg_func4[adrr_reg];
					else data_reg=0; //если запрос регистров за пределами таблицы регистров, то 0
				}

				if(mb_rtu_func==3){
					if(adrr_reg<MODBUS_FUNC_3_LENGTH) data_reg=*reg_func3[adrr_reg];
					else data_reg=0; //если запрос регистров за пределами таблицы регистров, то 0
				}
				
				spi1(data_reg);
				crc_calc_f(data_reg);
				++mb_rtu_num_send;
				++z;
			}	
			if(mb_rtu_num_send==mb_rtu_num && z<61){
				spi1((unsigned char)crc_f);
				++mb_rtu_num_send;
				++z;
			}
			if(mb_rtu_num_send>mb_rtu_num && z<61) {
					spi1((unsigned char)(crc_f>>8));					
					mb_rtu_num=0;//закончить передачу
			}	
 			sc16is700_CS_OFF 
			
		}
	 }
	 else{ //отправка остальных регистров
	 	sc16is700RecieveDisableFlag=1;
	 	sc16is700_spi_init();
		delay_us(2);
		sc16is700_CS_ON
		spi1((CS16IS7xx_THR&0x0f)<<3);
		if(mb_rtu_num_send<mb_rtu_num){//если отправлены не все регистры
			z=0;
			while (mb_rtu_num_send<mb_rtu_num && z<sc16is700TxFifoLevel){
				unsigned char data_reg=0;
				unsigned long adrr_reg;
				adrr_reg=mb_rtu_start_adr+mb_rtu_num_send;
				if(mb_rtu_func==4){
					if(adrr_reg<MODBUS_FUNC_4_LENGTH) data_reg=*reg_func4[adrr_reg];	
					else data_reg=0; //если запрос регистров за пределами таблицы регистров, то 0
				}
				else if(mb_rtu_func==3){
					if(adrr_reg<MODBUS_FUNC_3_LENGTH)	data_reg=*reg_func3[adrr_reg];
					else data_reg=0; //если запрос регистров за пределами таблицы регистров, то 0				
				}
				spi1(data_reg);
				crc_calc_f(data_reg);
				++mb_rtu_num_send;
				++z;
			}
		}

		if(mb_rtu_num_send==mb_rtu_num && z<sc16is700TxFifoLevel){
				spi1((unsigned char)crc_f);
				++mb_rtu_num_send;
				++z;
		}
		if(mb_rtu_num_send>mb_rtu_num && z<sc16is700TxFifoLevel) {
					spi1((unsigned char)(crc_f>>8));					
					mb_rtu_num=0;//закончить передачу
		}
 		sc16is700_CS_OFF

	 } 
	}
}
//Ожидаем, когда освободятся буферы передачи и приема:
if((sc16is700_rd_byte(CS16IS7xx_LSR))&0x40)	sc16is700RecieveDisableFlag=0;


}


