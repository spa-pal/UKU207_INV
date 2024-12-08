#include "MODBUS_func3.h"
#include "eeprom_map.h"
#include "main.h"
#include "MODBUS_RTU.h"
#include <LPC17xx.H>
#include "snmp_data_file.h"
#include "modbus.h"
#include "control.h"

signed short UB20_minus_DU;
char sntp_ip1_mb, sntp_ip2_mb, sntp_ip3_mb, sntp_ip4_mb;
char numbat_mb;
unsigned char sk_sign_0_mb, sk_zvuk_en_0_mb, sk_lcd_en_0_mb;
unsigned char sk_sign_1_mb, sk_zvuk_en_1_mb, sk_lcd_en_1_mb;
unsigned char sk_sign_2_mb, sk_zvuk_en_2_mb, sk_lcd_en_2_mb;
unsigned char sk_sign_3_mb, sk_zvuk_en_3_mb, sk_lcd_en_3_mb;
unsigned short tventmax_mb; 
unsigned short modbus_log_deep;
unsigned short modbus_log_ptr;
unsigned char modbus_log_data_byte[32];

void wr_reg_func3(unsigned long start_adr, unsigned long end_adr){ //Заполнение регистров для функции 3 перед их отправкой

		{
		if(modbus_log_ptr>=modbus_log_deep) modbus_log_ptr=0xffff;
		}
	//if(start_adr<=249 && end_adr>=249)
		{
		modbus_log_deep=lc640_read_int(CNT_EVENT_LOG);
		}
	//if(start_adr<=265 && end_adr>=250)
		{
		unsigned short tempUI;
	
		tempUI=lc640_read_int(PTR_EVENT_LOG);
		tempUI=ptr_carry(tempUI,64,-1*((signed)modbus_log_ptr));
		tempUI*=32;
		tempUI+=EVENT_LOG;
     
     	lc640_read_long_ptr(tempUI,&modbus_log_data_byte[0]);
		lc640_read_long_ptr(tempUI+4,&modbus_log_data_byte[4]);
		lc640_read_long_ptr(tempUI+8,&modbus_log_data_byte[8]);
		lc640_read_long_ptr(tempUI+12,&modbus_log_data_byte[12]);
		lc640_read_long_ptr(tempUI+16,&modbus_log_data_byte[16]);
		lc640_read_long_ptr(tempUI+20,&modbus_log_data_byte[20]);
		lc640_read_long_ptr(tempUI+24,&modbus_log_data_byte[24]);
		lc640_read_long_ptr(tempUI+28,&modbus_log_data_byte[28]);

		/*modbus_log_data_byte[0]=3;
		modbus_log_data_byte[1]=0;
		modbus_log_data_byte[2]=0;
		modbus_log_data_byte[3]=3; */

		}					 
}
//----------- Таблица адресов функц 3
//адреса распологаются по порядку, если нет регистра, то &NULL_0, &NULL_0,
unsigned char *const reg_func3 []={
&NULL_0,  //0
&NULL_0,  //0
&NULL_0,  //1
&NULL_0,  //1
&NULL_0,  //2
&NULL_0,  //2
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0, //10
&NULL_0, //10
(unsigned char*)&LPC_RTC->YEAR+1,	//Рег11  Время, год
(unsigned char*)&LPC_RTC->YEAR,  	//Рег11  Время, год
(unsigned char*)&LPC_RTC->MONTH+1,	//Рег12  Время, месяц
(unsigned char*)&LPC_RTC->MONTH,  	//Рег12  Время, месяц
(unsigned char*)&LPC_RTC->DOM+1,	//Рег13  Время, день месяца
(unsigned char*)&LPC_RTC->DOM,  	//Рег13  Время, день месяца
(unsigned char*)&LPC_RTC->HOUR+1,	//Рег14  Время, час
(unsigned char*)&LPC_RTC->HOUR,  	//Рег14  Время, час
(unsigned char*)&LPC_RTC->MIN+1,	//Рег15  Время, минуты
(unsigned char*)&LPC_RTC->MIN,  	//Рег15  Время, минутыы	
(unsigned char*)&LPC_RTC->SEC+1,	//Рег16  Время, секунды
(unsigned char*)&LPC_RTC->SEC,  	//Рег16  Время, секунды
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0, //20
&NULL_0, //20
(unsigned char*)&ZV_ON+1,			//Рег21  Звуковая аварийная сигнализация вкл./выкл.
(unsigned char*)&ZV_ON,				//Рег21  Звуковая аварийная сигнализация вкл./выкл.
&NULL_0,					 		//Рег22  Отключение аварийного сигнала 0-ручн., 1-автом.
(unsigned char*)&AV_OFF_AVT, 		//Рег22  Отключение аварийного сигнала 0-ручн., 1-автом.
(unsigned char*)&U_OUT_SET+1,		//Рег23	 Уставка выходного напряжения,  1В
(unsigned char*)&U_OUT_SET,			//Рег23	 Уставка выходного напряжения,  1В
(unsigned char*)&U_OUT_MAX+1,		//Рег24	 Уставка максимального (аварийного) выходного напряжения,  1В
(unsigned char*)&U_OUT_MAX,			//Рег24	 Уставка максимального (аварийного) выходного напряжения,  1В
(unsigned char*)&U_OUT_MIN+1,		//Рег25	 Уставка минимального (аварийного) выходного напряжения,  1В
(unsigned char*)&U_OUT_MIN,			//Рег25	 Уставка минимального (аварийного) выходного напряжения,  1В
(unsigned char*)&U_NET_ON_MIN+1,	//Рег26	 Уставка устранения фиксации заниженного напряжения по входу сеть или входу инв или выходу байпаса (активно только совместно с уставкой UacTurn...),  1В
(unsigned char*)&U_NET_ON_MIN,		//Рег26	 Уставка устранения фиксации заниженного напряжения по входу сеть или входу инв или выходу байпаса (активно только совместно с уставкой UacTurn...),  1В
(unsigned char*)&U_NET_OFF_MIN+1,	//Рег27	 Уставка фиксации заниженного напряжения по входу сеть или входу инв или выходу байпаса (активно только совместно с уставкой UacTurn...),  1В
(unsigned char*)&U_NET_OFF_MIN,		//Рег27	 Уставка фиксации заниженного напряжения по входу сеть или входу инв или выходу байпаса (активно только совместно с уставкой UacTurn...),  1В
(unsigned char*)&U_NET_ON_MAX+1,	//Рег28	 Уставка устранения фиксации завышенного напряжения по входу сеть или входу инв или выходу байпаса (активно только совместно с уставкой UacTurn... и только при скорости КАН 125кбит/с),  1В
(unsigned char*)&U_NET_ON_MAX,		//Рег28	 Уставка устранения фиксации завышенного напряжения по входу сеть или входу инв или выходу байпаса (активно только совместно с уставкой UacTurn... и только при скорости КАН 125кбит/с),  1В
(unsigned char*)&U_NET_OFF_MAX+1,	//Рег29	 Уставка фиксации завышенного напряжения по входу сеть или входу инв или выходу байпаса (активно только совместно с уставкой UacTurn... и только при скорости КАН 125кбит/с),  1В
(unsigned char*)&U_NET_OFF_MAX,		//Рег29	 Уставка фиксации завышенного напряжения по входу сеть или входу инв или выходу байпаса (активно только совместно с уставкой UacTurn... и только при скорости КАН 125кбит/с),  1В
(unsigned char*)&U_BAT_MAX+1,		//Рег30	 Напряжение батареи включения,  1В
(unsigned char*)&U_BAT_MAX,			//Рег30	 Напряжение батареи включения,  1В
(unsigned char*)&U_BAT_MIN+1,		//Рег31	 Напряжение батареи выключения,  1В
(unsigned char*)&U_BAT_MIN,			//Рег31	 Напряжение батареи выключения,  1В
(unsigned char*)&U_OUT_AC_MAX_AV+1,	//Рег32	 Напряжение выхода максимальное (аварийное),  1В
(unsigned char*)&U_OUT_AC_MAX_AV,	//Рег32	 Напряжение выхода максимальное (аварийное),  1В
(unsigned char*)&U_OUT_AC_MIN_AV+1,	//Рег33	 Напряжение выхода минимальное (аварийное),  1В
(unsigned char*)&U_OUT_AC_MIN_AV,	//Рег33	 Напряжение выхода минимальное (аварийное),  1В
(unsigned char*)&U_IN_AC_MAX_AV+1,	//Рег34	 Напряжение входа AC максимальное (аварийное),  1В
(unsigned char*)&U_IN_AC_MAX_AV,	//Рег34	 Напряжение входа AC максимальное (аварийное),  1В
(unsigned char*)&U_IN_AC_MIN_AV+1,	//Рег35	 Напряжение входа AC минимальное (аварийное),  1В
(unsigned char*)&U_IN_AC_MIN_AV,	//Рег35	 Напряжение входа AC минимальное (аварийное),  1В
(unsigned char*)&U_IN_DC_MAX_AV+1,	//Рег36	 Напряжение входа DC максимальное (аварийное),  1В
(unsigned char*)&U_IN_DC_MAX_AV,	//Рег36	 Напряжение входа DC максимальное (аварийное),  1В
(unsigned char*)&U_IN_DC_MIN_AV+1,	//Рег37	 Напряжение входа DC минимальное (аварийное),  1В
(unsigned char*)&U_IN_DC_MIN_AV,	//Рег37	 Напряжение входа DC минимальное (аварийное),  1В
(unsigned char*)&RELE_SET_MASK[0]+1,//Рег38	 Битовая маска срабатывания реле 1 по событиям
(unsigned char*)&RELE_SET_MASK[0],	//Рег38	 Битовая маска срабатывания реле 1 по событиям
(unsigned char*)&RELE_SET_MASK[1]+1,//Рег39	 Битовая маска срабатывания реле 2 по событиям
(unsigned char*)&RELE_SET_MASK[1],	//Рег39	 Битовая маска срабатывания реле 2 по событиям
&NULL_0,		 					//Рег40
&NULL_0,						   	//Рег40
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
(unsigned char*)&modbus_log_ptr+1,
(unsigned char*)&modbus_log_ptr,
(unsigned char*)&modbus_log_deep+1,
(unsigned char*)&modbus_log_deep,
(unsigned char*)&modbus_log_data_byte[0], 	//Рег50
(unsigned char*)&modbus_log_data_byte[1],	//Рег50
(unsigned char*)&modbus_log_data_byte[2],
(unsigned char*)&modbus_log_data_byte[3],
(unsigned char*)&modbus_log_data_byte[4],
(unsigned char*)&modbus_log_data_byte[5],
(unsigned char*)&modbus_log_data_byte[6],
(unsigned char*)&modbus_log_data_byte[7],
(unsigned char*)&modbus_log_data_byte[8],
(unsigned char*)&modbus_log_data_byte[9],
(unsigned char*)&modbus_log_data_byte[10],
(unsigned char*)&modbus_log_data_byte[11],
(unsigned char*)&modbus_log_data_byte[12],
(unsigned char*)&modbus_log_data_byte[13],
(unsigned char*)&modbus_log_data_byte[14],
(unsigned char*)&modbus_log_data_byte[15],
(unsigned char*)&modbus_log_data_byte[16],
(unsigned char*)&modbus_log_data_byte[17],
(unsigned char*)&modbus_log_data_byte[18],
(unsigned char*)&modbus_log_data_byte[19],
(unsigned char*)&modbus_log_data_byte[20], 	//Рег60
(unsigned char*)&modbus_log_data_byte[21],	//Рег60
(unsigned char*)&modbus_log_data_byte[22],
(unsigned char*)&modbus_log_data_byte[23],
(unsigned char*)&modbus_log_data_byte[24],
(unsigned char*)&modbus_log_data_byte[25],
(unsigned char*)&modbus_log_data_byte[26],
(unsigned char*)&modbus_log_data_byte[27],
(unsigned char*)&modbus_log_data_byte[28],
(unsigned char*)&modbus_log_data_byte[29],
(unsigned char*)&modbus_log_data_byte[30],
(unsigned char*)&modbus_log_data_byte[31],
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
(unsigned char*)&NUMINV+1,		//Рег70	 Количество инверторов в структуре
(unsigned char*)&NUMINV,		//Рег70	 Количество инверторов в структуре
(unsigned char*)&NUMBYPASS+1,	//Рег71	 Количество байпасов в структуре (10 - встроенный)
(unsigned char*)&NUMBYPASS,		//Рег71	 Количество байпасов в структуре (10 - встроенный)
(unsigned char*)&NUMPHASE+1,	//Рег72	 Количество фаз выходной сети в структуре (1 или 3)
(unsigned char*)&NUMPHASE,		//Рег72	 Количество фаз выходной сети в структуре (1 или 3)
(unsigned char*)&NUMINAC+1,		//Рег73	 (DC-AC)/AC 1 - Да  0 - Нет
(unsigned char*)&NUMINAC,		//Рег73	 (DC-AC)/AC 1 - Да  0 - Нет
(unsigned char*)&NUMSK+1,		//Рег74	 Количество сухих контактов в структуре
(unsigned char*)&NUMSK,			//Рег74	 Количество сухих контактов в структуре
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0

};




