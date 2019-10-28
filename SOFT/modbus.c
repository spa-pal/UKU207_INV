#include "stdint.h"
#include "modbus.h"

#include "LPC17xx.H"
#include "main.h"
#include "stdio.h"
#include "eeprom_map.h"
#include "25lc640.h"
#include "gran.h"
#include "uart0.h"
#include "sc16is7xx.h"
#include "control.h"
	
#include <string.h>
#include "rtl.h"
#include "modbus_tcp.h"
unsigned char modbus_buf[20];
short modbus_crc16;
char modbus_timeout_cnt;
char bMODBUS_TIMEOUT;
unsigned char modbus_rx_buffer[30];	//Буфер, куда складывает принимаемые даннные обработчик прерывания по приему УАРТа 
unsigned char modbus_an_buffer[30];    	//Буфер, куда они потом копируются для анализа
unsigned char modbus_rx_buffer_ptr;	//Указатель на текущую позицию принимающего буфера
unsigned char modbus_rx_counter;		//Количество принятых байт, используется при анализе целостности посылки и при расшифровке
signed short modbusTimeoutInMills;
short modbus_plazma;				//Отладка
short modbus_plazma1;				//Отладка
short modbus_plazma2;				//Отладка
short modbus_plazma3;				//Отладка
char modbus_cmnd_cnt,modbus_cmnd,modbus_self_cmnd_cnt=33;

short modbus_crc_plazma[2];
short modbus_rtu_plazma[10];

#define MODBUS_RTU_PROT	0

//-----------------------------------------------
unsigned short CRC16_2(char* buf, short len)
{
unsigned short crc = 0xFFFF;
short pos;
short i;

for (pos = 0; pos < len; pos++)
  	{
    	crc ^= (unsigned short)buf[pos];          // XOR byte into least sig. byte of crc

    	for ( i = 8; i != 0; i--) 
		{    // Loop over each bit
      	if ((crc & 0x0001) != 0) 
			{      // If the LSB is set
        		crc >>= 1;                    // Shift right and XOR 0xA001
        		crc ^= 0xA001;
      		}
      	else  crc >>= 1;                    // Just shift right
    		}
  	}
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
return crc;
}

//-----------------------------------------------
void modbus_in(void)
{
short crc16_calculated;		//вычисляемая из принятых данных CRC
short crc16_incapsulated;	//встроеннная в посылку CRC
unsigned short modbus_rx_arg0;		//встроенный в посылку первый аргумент
unsigned short modbus_rx_arg1;		//встроенный в посылку второй аргумент
//unsigned short modbus_rx_arg2;		//встроенный в посылку третий аргумент
//unsigned short modbus_rx_arg3;		//встроенный в посылку четвертый аргумент
unsigned char modbus_func;			//встроенный в посылку код функции



memcpy(modbus_an_buffer,modbus_rx_buffer,modbus_rx_buffer_ptr);
modbus_rx_counter=modbus_rx_buffer_ptr;
modbus_rx_buffer_ptr=0;
bMODBUS_TIMEOUT=0;

	
crc16_calculated  = CRC16_2((char*)modbus_an_buffer, modbus_rx_counter-2);
crc16_incapsulated = *((short*)&modbus_an_buffer[modbus_rx_counter-2]);

//modbus_plazma=modbus_rx_counter;
//modbus_plazma2=crc16_calculated;
//modbus_plazma3=crc16_incapsulated;

modbus_plazma++;


modbus_func=modbus_an_buffer[1];
modbus_rx_arg0=(((unsigned short)modbus_an_buffer[2])*((unsigned short)256))+((unsigned short)modbus_an_buffer[3]);
modbus_rx_arg1=(((unsigned short)modbus_an_buffer[4])*((unsigned short)256))+((unsigned short)modbus_an_buffer[5]);
//modbus_rx_arg2=(((unsigned short)modbus_an_buffer[6])*((unsigned short)256))+((unsigned short)modbus_an_buffer[7]);
//modbus_rx_arg3=(((unsigned short)modbus_an_buffer[8])*((unsigned short)256))+((unsigned short)modbus_an_buffer[9]);
modbus_cmnd=(modbus_rx_arg1>>8)&0x7f;
modbus_cmnd_cnt=(modbus_rx_arg1)&0x0f;

modbus_plazma3=modbus_cmnd;
modbus_plazma2=modbus_cmnd_cnt;
modbus_plazma1=modbus_self_cmnd_cnt;

modbus_crc_plazma[0]=crc16_calculated;
modbus_crc_plazma[1]=crc16_incapsulated;

if(crc16_calculated==crc16_incapsulated)
	{
	if(modbus_self_cmnd_cnt==33)modbus_self_cmnd_cnt=modbus_cmnd_cnt;
	
	//modbus_plazma=modbus_an_buffer[0];

	if(modbus_an_buffer[0]==MODBUS_ADRESS)
		{
		if(modbus_func==3)		//чтение произвольного кол-ва регистров
			{
			modbus_hold_registers_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0,modbus_rx_arg1,MODBUS_RTU_PROT);
			}
		else if(modbus_func==4)		//чтение произвольного кол-ва регистров	входов
			{
			modbus_input_registers_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0,modbus_rx_arg1,MODBUS_RTU_PROT);
			}

		else if(modbus_func==6) 	//запись регистра
			{/*
			if(modbus_rx_arg0==11)		//Установка времени 
				{
				LPC_RTC->YEAR=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==12)		//Установка времени 
				{
				LPC_RTC->MONTH=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==13)		//Установка времени 
				{
				LPC_RTC->DOM=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==14)		//Установка времени 
				{
				LPC_RTC->HOUR=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==15)		//Установка времени 
				{
				LPC_RTC->MIN=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==16)		//Установка времени 
				{
				LPC_RTC->SEC=(uint16_t)modbus_rx_arg1;
				} */
			

			//modbus_hold_register_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0);
			

			modbus_hold_register_write(MODBUS_ADRESS,modbus_func,modbus_rx_arg0,modbus_rx_arg1, MODBUS_RTU_PROT);



			//modbus_hold_registers_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0,2);
			}
		} 
	//;
	}
// modbus_plazma++;

}

//-----------------------------------------------
void modbus_input_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot)
{
char modbus_registers[1000];
char modbus_tx_buff[120];
unsigned short crc_temp;
char i;



if(NUMBYPASS)
	{
	modbus_registers[0]=(char)(byps[0]._Iout/256);			//Рег1
	modbus_registers[1]=(char)(byps[0]._Iout%256);
	modbus_registers[2]=(char)(byps[0]._Uout/256);			//Рег2
	modbus_registers[3]=(char)(byps[0]._Uout%256);
	modbus_registers[4]=(char)(byps[0]._Pout/256);			//Рег3
	modbus_registers[5]=(char)(byps[0]._Pout%256);
	modbus_registers[6]=(char)(byps[0]._T/256);			//Рег4
	modbus_registers[7]=(char)(byps[0]._T%256);
	}
else 
	{
	modbus_registers[0]=(char)(load_I/256);				//Рег1
	modbus_registers[1]=(char)(load_I%256);
	modbus_registers[2]=(char)(load_U/256);				//Рег2
	modbus_registers[3]=(char)(load_U%256);
	modbus_registers[4]=(char)(load_P/256);				//Рег3
	modbus_registers[5]=(char)(load_P%256);
	modbus_registers[6]=0;							//Рег4
	modbus_registers[7]=0;
	}

modbus_registers[8]=(char)(t_ext[0]/256);								//Рег5
modbus_registers[9]=(char)(t_ext[0]%256);

	
modbus_registers[10]=(char)(NUMBYPASS/256);				//Рег6
modbus_registers[11]=(char)(NUMBYPASS%256);
modbus_registers[12]=(char)(NUMINV/256);				//Рег7
modbus_registers[13]=(char)(NUMINV%256);
modbus_registers[14]=(char)(num_of_wrks_inv/256);			//Рег8
modbus_registers[15]=(char)(num_of_wrks_inv%256);

modbus_registers[16]=(char)(dcin_U/256);					//Рег9
modbus_registers[17]=(char)(dcin_U%256);
modbus_registers[18]=(char)(byps[0]._UinACprim/256);			//Рег10
modbus_registers[19]=(char)(byps[0]._UinACprim%256);

inv[0]._Uout=1234;

//Инвертор №1
modbus_registers[20]=(char)(inv[0]._Uout/256);			//Рег11
modbus_registers[21]=(char)(inv[0]._Uout%256);
modbus_registers[22]=(char)(inv[0]._Iout/256);			 	//Рег12
modbus_registers[23]=(char)(inv[0]._Iout%256);
modbus_registers[24]=(char)(inv[0]._T/256);				//Рег13
modbus_registers[25]=(char)(inv[0]._T%256);
modbus_registers[26]=(char)(inv[0]._Pout/256);			//Рег14
modbus_registers[27]=(char)(inv[0]._Pout%256);
modbus_registers[28]=(char)(inv[0]._Uacin/256);			//Рег15
modbus_registers[29]=(char)(inv[0]._Uacin%256);
modbus_registers[30]=(char)(inv[0]._Uload/256);			//Рег16
modbus_registers[31]=(char)(inv[0]._Uload%256);
i=inv[0]._flags_tm;
if(inv[0]._cnt>10)i|=0x80;
modbus_registers[32]=0;								//Рег17
modbus_registers[33]=i;
modbus_registers[34]=0;								//Рег18
modbus_registers[35]=0;
modbus_registers[36]=0;								//Рег19
modbus_registers[37]=0;
modbus_registers[38]=0;								//Рег20
modbus_registers[39]=0;

//Инвертор №2
modbus_registers[40]=(char)(inv[1]._Uout/256);			//Рег21
modbus_registers[41]=(char)(inv[1]._Uout%256);
modbus_registers[42]=(char)(inv[1]._Iout/256);			 	//Рег22
modbus_registers[43]=(char)(inv[1]._Iout%256);
modbus_registers[44]=(char)(inv[1]._T/256);				//Рег23
modbus_registers[45]=(char)(inv[1]._T%256);
modbus_registers[46]=(char)(inv[1]._Pout/256);			//Рег24
modbus_registers[47]=(char)(inv[1]._Pout%256);
modbus_registers[48]=(char)(inv[1]._Uacin/256);			//Рег25
modbus_registers[49]=(char)(inv[1]._Uacin%256);
modbus_registers[50]=(char)(inv[1]._Uload/256);			//Рег26
modbus_registers[51]=(char)(inv[1]._Uload%256);
i=inv[1]._flags_tm;
if(inv[1]._cnt>10)i|=0x80;
modbus_registers[52]=0;								//Рег27
modbus_registers[53]=i;
modbus_registers[54]=0;								//Рег28
modbus_registers[55]=0;
modbus_registers[56]=0;								//Рег29
modbus_registers[57]=0;
modbus_registers[58]=0;								//Рег30
modbus_registers[59]=0;

//Инвертор №3
modbus_registers[60]=(char)(inv[2]._Uout/256);			//Рег31
modbus_registers[61]=(char)(inv[2]._Uout%256);
modbus_registers[62]=(char)(inv[2]._Iout/256);			 	//Рег32
modbus_registers[63]=(char)(inv[2]._Iout%256);
modbus_registers[64]=(char)(inv[2]._T/256);				//Рег33
modbus_registers[65]=(char)(inv[2]._T%256);
modbus_registers[66]=(char)(inv[2]._Pout/256);			//Рег34
modbus_registers[67]=(char)(inv[2]._Pout%256);
modbus_registers[68]=(char)(inv[2]._Uacin/256);			//Рег35
modbus_registers[69]=(char)(inv[2]._Uacin%256);
modbus_registers[70]=(char)(inv[2]._Uload/256);			//Рег36
modbus_registers[71]=(char)(inv[2]._Uload%256);
i=inv[2]._flags_tm;
if(inv[2]._cnt>10)i|=0x80;
modbus_registers[72]=0;								//Рег37
modbus_registers[73]=i;
modbus_registers[74]=0;								//Рег38
modbus_registers[75]=0;
modbus_registers[76]=0;								//Рег39
modbus_registers[77]=0;
modbus_registers[78]=0;								//Рег40
modbus_registers[79]=0;

//Инвертор №4
modbus_registers[80]=(char)(inv[3]._Uout/256);			//Рег41
modbus_registers[81]=(char)(inv[3]._Uout%256);
modbus_registers[82]=(char)(inv[3]._Iout/256);			 	//Рег42
modbus_registers[83]=(char)(inv[3]._Iout%256);
modbus_registers[84]=(char)(inv[3]._T/256);				//Рег43
modbus_registers[85]=(char)(inv[3]._T%256);
modbus_registers[86]=(char)(inv[3]._Pout/256);			//Рег44
modbus_registers[87]=(char)(inv[3]._Pout%256);
modbus_registers[88]=(char)(inv[3]._Uacin/256);			//Рег45
modbus_registers[89]=(char)(inv[3]._Uacin%256);
modbus_registers[90]=(char)(inv[3]._Uload/256);			//Рег46
modbus_registers[91]=(char)(inv[3]._Uload%256);
i=inv[3]._flags_tm;
if(inv[3]._cnt>10)i|=0x80;
modbus_registers[92]=0;								//Рег47
modbus_registers[93]=i;
modbus_registers[94]=0;								//Рег48
modbus_registers[95]=0;
modbus_registers[96]=0;								//Рег49
modbus_registers[97]=0;
modbus_registers[98]=0;								//Рег50
modbus_registers[99]=0;

//Инвертор №5
modbus_registers[100]=(char)(inv[4]._Uout/256);			//Рег51
modbus_registers[101]=(char)(inv[4]._Uout%256);
modbus_registers[102]=(char)(inv[4]._Iout/256);			//Рег52
modbus_registers[103]=(char)(inv[4]._Iout%256);
modbus_registers[104]=(char)(inv[4]._T/256);			//Рег53
modbus_registers[105]=(char)(inv[4]._T%256);
modbus_registers[106]=(char)(inv[4]._Pout/256);			//Рег54
modbus_registers[107]=(char)(inv[4]._Pout%256);
modbus_registers[108]=(char)(inv[4]._Uacin/256);			//Рег55
modbus_registers[109]=(char)(inv[4]._Uacin%256);
modbus_registers[110]=(char)(inv[4]._Uload/256);			//Рег56
modbus_registers[111]=(char)(inv[4]._Uload%256);
i=inv[4]._flags_tm;
if(inv[4]._cnt>10)i|=0x80;
modbus_registers[112]=0;								//Рег57
modbus_registers[113]=i;
modbus_registers[114]=0;								//Рег58
modbus_registers[115]=0;
modbus_registers[116]=0;								//Рег59
modbus_registers[117]=0;
modbus_registers[118]=0;								//Рег60
modbus_registers[119]=0;

//Инвертор №6
modbus_registers[120]=(char)(inv[5]._Uout/256);			//Рег61
modbus_registers[121]=(char)(inv[5]._Uout%256);
modbus_registers[122]=(char)(inv[5]._Iout/256);			//Рег62
modbus_registers[123]=(char)(inv[5]._Iout%256);
modbus_registers[124]=(char)(inv[5]._T/256);			//Рег63
modbus_registers[125]=(char)(inv[5]._T%256);
modbus_registers[126]=(char)(inv[5]._Pout/256);			//Рег64
modbus_registers[127]=(char)(inv[5]._Pout%256);
modbus_registers[128]=(char)(inv[5]._Uacin/256);			//Рег65
modbus_registers[129]=(char)(inv[5]._Uacin%256);
modbus_registers[130]=(char)(inv[5]._Uload/256);			//Рег66
modbus_registers[131]=(char)(inv[5]._Uload%256);
i=inv[5]._flags_tm;
if(inv[5]._cnt>10)i|=0x80;
modbus_registers[132]=0;								//Рег67
modbus_registers[133]=i;
modbus_registers[134]=0;								//Рег68
modbus_registers[135]=0;
modbus_registers[136]=0;								//Рег69
modbus_registers[137]=0;
modbus_registers[138]=0;								//Рег70
modbus_registers[139]=0;

//Инвертор №7
modbus_registers[140]=(char)(inv[6]._Uout/256);			//Рег71
modbus_registers[141]=(char)(inv[6]._Uout%256);
modbus_registers[142]=(char)(inv[6]._Iout/256);			//Рег72
modbus_registers[143]=(char)(inv[6]._Iout%256);
modbus_registers[144]=(char)(inv[6]._T/256);			//Рег73
modbus_registers[145]=(char)(inv[6]._T%256);
modbus_registers[146]=(char)(inv[6]._Pout/256);			//Рег74
modbus_registers[147]=(char)(inv[6]._Pout%256);
modbus_registers[148]=(char)(inv[6]._Uacin/256);			//Рег75
modbus_registers[149]=(char)(inv[6]._Uacin%256);
modbus_registers[150]=(char)(inv[6]._Uload/256);			//Рег76
modbus_registers[151]=(char)(inv[6]._Uload%256);
i=inv[6]._flags_tm;
if(inv[6]._cnt>10)i|=0x80;
modbus_registers[152]=0;								//Рег77
modbus_registers[153]=i;
modbus_registers[154]=0;								//Рег78
modbus_registers[155]=0;
modbus_registers[156]=0;								//Рег79
modbus_registers[157]=0;
modbus_registers[118]=0;								//Рег80
modbus_registers[159]=0;

//Инвертор №8
modbus_registers[160]=(char)(inv[7]._Uout/256);			//Рег81
modbus_registers[161]=(char)(inv[7]._Uout%256);
modbus_registers[162]=(char)(inv[7]._Iout/256);			//Рег82
modbus_registers[163]=(char)(inv[7]._Iout%256);
modbus_registers[164]=(char)(inv[7]._T/256);			//Рег83
modbus_registers[165]=(char)(inv[7]._T%256);
modbus_registers[166]=(char)(inv[7]._Pout/256);			//Рег84
modbus_registers[167]=(char)(inv[7]._Pout%256);
modbus_registers[168]=(char)(inv[7]._Uacin/256);			//Рег85
modbus_registers[169]=(char)(inv[7]._Uacin%256);
modbus_registers[170]=(char)(inv[7]._Uload/256);			//Рег86
modbus_registers[171]=(char)(inv[7]._Uload%256);
i=inv[7]._flags_tm;
if(inv[7]._cnt>10)i|=0x80;
modbus_registers[172]=0;								//Рег87
modbus_registers[173]=i;
modbus_registers[174]=0;								//Рег88
modbus_registers[175]=0;
modbus_registers[176]=0;								//Рег89
modbus_registers[177]=0;
modbus_registers[178]=0;								//Рег90
modbus_registers[179]=0;

//Инвертор №9
modbus_registers[180]=(char)(inv[8]._Uout/256);			//Рег91
modbus_registers[181]=(char)(inv[8]._Uout%256);
modbus_registers[182]=(char)(inv[8]._Iout/256);			//Рег92
modbus_registers[183]=(char)(inv[8]._Iout%256);
modbus_registers[184]=(char)(inv[8]._T/256);			//Рег93
modbus_registers[185]=(char)(inv[8]._T%256);
modbus_registers[186]=(char)(inv[8]._Pout/256);			//Рег94
modbus_registers[187]=(char)(inv[8]._Pout%256);
modbus_registers[188]=(char)(inv[8]._Uacin/256);			//Рег95
modbus_registers[189]=(char)(inv[8]._Uacin%256);
modbus_registers[190]=(char)(inv[8]._Uload/256);			//Рег96
modbus_registers[191]=(char)(inv[8]._Uload%256);
i=inv[8]._flags_tm;
if(inv[8]._cnt>10)i|=0x80;
modbus_registers[192]=0;								//Рег97
modbus_registers[193]=i;
modbus_registers[194]=0;								//Рег98
modbus_registers[195]=0;
modbus_registers[196]=0;								//Рег99
modbus_registers[197]=0;
modbus_registers[198]=0;								//Рег100
modbus_registers[199]=0;

//Инвертор №10
modbus_registers[200]=(char)(inv[9]._Uout/256);			//Рег101
modbus_registers[201]=(char)(inv[9]._Uout%256);
modbus_registers[202]=(char)(inv[9]._Iout/256);			//Рег102
modbus_registers[203]=(char)(inv[9]._Iout%256);
modbus_registers[204]=(char)(inv[9]._T/256);			//Рег103
modbus_registers[205]=(char)(inv[9]._T%256);
modbus_registers[206]=(char)(inv[9]._Pout/256);			//Рег104
modbus_registers[207]=(char)(inv[9]._Pout%256);
modbus_registers[208]=(char)(inv[9]._Uacin/256);			//Рег105
modbus_registers[209]=(char)(inv[9]._Uacin%256);
modbus_registers[210]=(char)(inv[9]._Uload/256);			//Рег106
modbus_registers[211]=(char)(inv[9]._Uload%256);
i=inv[9]._flags_tm;
if(inv[9]._cnt>10)i|=0x80;
modbus_registers[212]=0;								//Рег107
modbus_registers[213]=i;
modbus_registers[214]=0;								//Рег108
modbus_registers[215]=0;
modbus_registers[216]=0;								//Рег109
modbus_registers[217]=0;
modbus_registers[218]=0;								//Рег110
modbus_registers[219]=0;

//Инвертор №11
modbus_registers[220]=(char)(inv[10]._Uout/256);			//Рег111
modbus_registers[221]=(char)(inv[10]._Uout%256);
modbus_registers[222]=(char)(inv[10]._Iout/256);			//Рег112
modbus_registers[223]=(char)(inv[10]._Iout%256);
modbus_registers[224]=(char)(inv[10]._T/256);			//Рег113
modbus_registers[225]=(char)(inv[10]._T%256);
modbus_registers[226]=(char)(inv[10]._Pout/256);			//Рег114
modbus_registers[227]=(char)(inv[10]._Pout%256);
modbus_registers[228]=(char)(inv[10]._Uacin/256);			//Рег115
modbus_registers[229]=(char)(inv[10]._Uacin%256);
modbus_registers[230]=(char)(inv[10]._Uload/256);			//Рег116
modbus_registers[231]=(char)(inv[10]._Uload%256);
i=inv[10]._flags_tm;
if(inv[10]._cnt>10)i|=0x80;
modbus_registers[232]=0;								//Рег117
modbus_registers[233]=i;
modbus_registers[234]=0;								//Рег118
modbus_registers[235]=0;
modbus_registers[236]=0;								//Рег119
modbus_registers[237]=0;
modbus_registers[238]=0;								//Рег120
modbus_registers[239]=0;

//Инвертор №12
modbus_registers[240]=(char)(inv[11]._Uout/256);			//Рег121
modbus_registers[241]=(char)(inv[11]._Uout%256);
modbus_registers[242]=(char)(inv[11]._Iout/256);			//Рег122
modbus_registers[243]=(char)(inv[11]._Iout%256);
modbus_registers[244]=(char)(inv[11]._T/256);			//Рег123
modbus_registers[245]=(char)(inv[11]._T%256);
modbus_registers[246]=(char)(inv[11]._Pout/256);			//Рег124
modbus_registers[247]=(char)(inv[11]._Pout%256);
modbus_registers[248]=(char)(inv[11]._Uacin/256);			//Рег125
modbus_registers[249]=(char)(inv[11]._Uacin%256);
modbus_registers[250]=(char)(inv[11]._Uload/256);			//Рег126
modbus_registers[251]=(char)(inv[11]._Uload%256);
i=inv[11]._flags_tm;
if(inv[11]._cnt>10)i|=0x80;
modbus_registers[252]=0;								//Рег127
modbus_registers[253]=i;
modbus_registers[254]=0;								//Рег128
modbus_registers[255]=0;
modbus_registers[256]=0;								//Рег129
modbus_registers[257]=0;
modbus_registers[258]=0;								//Рег130
modbus_registers[259]=0;

//Инвертор №13
modbus_registers[260]=(char)(inv[12]._Uout/256);			//Рег131
modbus_registers[261]=(char)(inv[12]._Uout%256);
modbus_registers[262]=(char)(inv[12]._Iout/256);			//Рег132
modbus_registers[263]=(char)(inv[12]._Iout%256);
modbus_registers[264]=(char)(inv[12]._T/256);			//Рег133
modbus_registers[265]=(char)(inv[12]._T%256);
modbus_registers[266]=(char)(inv[12]._Pout/256);			//Рег134
modbus_registers[267]=(char)(inv[12]._Pout%256);
modbus_registers[268]=(char)(inv[12]._Uacin/256);			//Рег135
modbus_registers[269]=(char)(inv[12]._Uacin%256);
modbus_registers[270]=(char)(inv[12]._Uload/256);			//Рег136
modbus_registers[271]=(char)(inv[12]._Uload%256);
i=inv[12]._flags_tm;
if(inv[12]._cnt>10)i|=0x80;
modbus_registers[272]=0;								//Рег137
modbus_registers[273]=i;
modbus_registers[274]=0;								//Рег138
modbus_registers[275]=0;
modbus_registers[276]=0;								//Рег139
modbus_registers[277]=0;
modbus_registers[278]=0;								//Рег140
modbus_registers[279]=0;

//Инвертор №14
modbus_registers[280]=(char)(inv[13]._Uout/256);			//Рег141
modbus_registers[281]=(char)(inv[13]._Uout%256);
modbus_registers[282]=(char)(inv[13]._Iout/256);			//Рег142
modbus_registers[283]=(char)(inv[13]._Iout%256);
modbus_registers[284]=(char)(inv[13]._T/256);			//Рег143
modbus_registers[285]=(char)(inv[13]._T%256);
modbus_registers[286]=(char)(inv[13]._Pout/256);			//Рег144
modbus_registers[287]=(char)(inv[13]._Pout%256);
modbus_registers[288]=(char)(inv[13]._Uacin/256);			//Рег145
modbus_registers[289]=(char)(inv[13]._Uacin%256);
modbus_registers[290]=(char)(inv[13]._Uload/256);			//Рег146
modbus_registers[291]=(char)(inv[13]._Uload%256);
i=inv[13]._flags_tm;
if(inv[13]._cnt>10)i|=0x80;
modbus_registers[292]=0;								//Рег147
modbus_registers[293]=i;
modbus_registers[294]=0;								//Рег148
modbus_registers[295]=0;
modbus_registers[296]=0;								//Рег149
modbus_registers[297]=0;
modbus_registers[298]=0;								//Рег150
modbus_registers[299]=0;

//Инвертор №15
modbus_registers[300]=(char)(inv[14]._Uout/256);			//Рег151
modbus_registers[301]=(char)(inv[14]._Uout%256);
modbus_registers[302]=(char)(inv[14]._Iout/256);			//Рег152
modbus_registers[303]=(char)(inv[14]._Iout%256);
modbus_registers[304]=(char)(inv[14]._T/256);			//Рег153
modbus_registers[305]=(char)(inv[14]._T%256);
modbus_registers[306]=(char)(inv[14]._Pout/256);			//Рег154
modbus_registers[307]=(char)(inv[14]._Pout%256);
modbus_registers[308]=(char)(inv[14]._Uacin/256);			//Рег155
modbus_registers[309]=(char)(inv[14]._Uacin%256);
modbus_registers[310]=(char)(inv[14]._Uload/256);			//Рег156
modbus_registers[311]=(char)(inv[14]._Uload%256);
i=inv[14]._flags_tm;
if(inv[14]._cnt>10)i|=0x80;
modbus_registers[312]=0;								//Рег157
modbus_registers[313]=i;
modbus_registers[314]=0;								//Рег158
modbus_registers[315]=0;
modbus_registers[316]=0;								//Рег159
modbus_registers[317]=0;
modbus_registers[318]=0;								//Рег160
modbus_registers[319]=0;

//Инвертор №16
modbus_registers[320]=(char)(inv[15]._Uout/256);			//Рег161
modbus_registers[321]=(char)(inv[15]._Uout%256);
modbus_registers[322]=(char)(inv[15]._Iout/256);			//Рег162
modbus_registers[323]=(char)(inv[15]._Iout%256);
modbus_registers[324]=(char)(inv[15]._T/256);			//Рег163
modbus_registers[325]=(char)(inv[15]._T%256);
modbus_registers[326]=(char)(inv[15]._Pout/256);			//Рег164
modbus_registers[327]=(char)(inv[15]._Pout%256);
modbus_registers[328]=(char)(inv[15]._Uacin/256);			//Рег165
modbus_registers[329]=(char)(inv[15]._Uacin%256);
modbus_registers[330]=(char)(inv[15]._Uload/256);			//Рег166
modbus_registers[331]=(char)(inv[15]._Uload%256);
i=inv[15]._flags_tm;
if(inv[15]._cnt>10)i|=0x80;
modbus_registers[332]=0;									//Рег167
modbus_registers[333]=i;
modbus_registers[334]=0;									//Рег168
modbus_registers[335]=0;
modbus_registers[336]=0;									//Рег169
modbus_registers[337]=0;
modbus_registers[338]=0;									//Рег170
modbus_registers[339]=0;

//Инвертор №17
modbus_registers[340]=(char)(inv[16]._Uout/256);			//Рег171
modbus_registers[341]=(char)(inv[16]._Uout%256);
modbus_registers[342]=(char)(inv[16]._Iout/256);			//Рег172
modbus_registers[343]=(char)(inv[16]._Iout%256);
modbus_registers[344]=(char)(inv[16]._T/256);				//Рег173
modbus_registers[345]=(char)(inv[16]._T%256);
modbus_registers[346]=(char)(inv[16]._Pout/256);			//Рег174
modbus_registers[347]=(char)(inv[16]._Pout%256);
modbus_registers[348]=(char)(inv[16]._Uacin/256);			//Рег175
modbus_registers[349]=(char)(inv[16]._Uacin%256);
modbus_registers[350]=(char)(inv[16]._Uload/256);			//Рег176
modbus_registers[351]=(char)(inv[16]._Uload%256);
i=inv[16]._flags_tm;
if(inv[16]._cnt>10)i|=0x80;
modbus_registers[352]=0;									//Рег177
modbus_registers[353]=i;
modbus_registers[354]=0;									//Рег178
modbus_registers[355]=0;
modbus_registers[356]=0;									//Рег179
modbus_registers[357]=0;
modbus_registers[358]=0;									//Рег180
modbus_registers[359]=0;

//Инвертор №18
modbus_registers[360]=(char)(inv[17]._Uout/256);			//Рег181
modbus_registers[361]=(char)(inv[17]._Uout%256);
modbus_registers[362]=(char)(inv[17]._Iout/256);			//Рег182
modbus_registers[363]=(char)(inv[17]._Iout%256);
modbus_registers[364]=(char)(inv[17]._T/256);				//Рег183
modbus_registers[365]=(char)(inv[17]._T%256);
modbus_registers[366]=(char)(inv[17]._Pout/256);			//Рег184
modbus_registers[367]=(char)(inv[17]._Pout%256);
modbus_registers[368]=(char)(inv[17]._Uacin/256);			//Рег185
modbus_registers[369]=(char)(inv[17]._Uacin%256);
modbus_registers[370]=(char)(inv[17]._Uload/256);			//Рег186
modbus_registers[371]=(char)(inv[17]._Uload%256);
i=inv[17]._flags_tm;
if(inv[17]._cnt>10)i|=0x80;
modbus_registers[372]=0;									//Рег187
modbus_registers[373]=i;
modbus_registers[374]=0;									//Рег188
modbus_registers[375]=0;
modbus_registers[376]=0;									//Рег189
modbus_registers[377]=0;
modbus_registers[378]=0;									//Рег190
modbus_registers[379]=0;

//Инвертор №19
modbus_registers[380]=(char)(inv[18]._Uout/256);			//Рег201
modbus_registers[381]=(char)(inv[18]._Uout%256);
modbus_registers[382]=(char)(inv[18]._Iout/256);			//Рег202
modbus_registers[383]=(char)(inv[18]._Iout%256);
modbus_registers[384]=(char)(inv[18]._T/256);				//Рег203
modbus_registers[385]=(char)(inv[18]._T%256);
modbus_registers[386]=(char)(inv[18]._Pout/256);			//Рег204
modbus_registers[387]=(char)(inv[18]._Pout%256);
modbus_registers[388]=(char)(inv[18]._Uacin/256);			//Рег205
modbus_registers[389]=(char)(inv[18]._Uacin%256);
modbus_registers[390]=(char)(inv[18]._Uload/256);			//Рег206
modbus_registers[391]=(char)(inv[18]._Uload%256);
i=inv[18]._flags_tm;
if(inv[18]._cnt>10)i|=0x80;
modbus_registers[392]=0;									//Рег207
modbus_registers[393]=i;
modbus_registers[394]=0;									//Рег208
modbus_registers[395]=0;
modbus_registers[396]=0;									//Рег209
modbus_registers[397]=0;
modbus_registers[398]=0;									//Рег210
modbus_registers[399]=0;

//Инвертор №20
modbus_registers[400]=(char)(inv[19]._Uout/256);			//Рег211
modbus_registers[401]=(char)(inv[19]._Uout%256);
modbus_registers[402]=(char)(inv[19]._Iout/256);			//Рег212
modbus_registers[403]=(char)(inv[19]._Iout%256);
modbus_registers[404]=(char)(inv[19]._T/256);				//Рег213
modbus_registers[405]=(char)(inv[19]._T%256);
modbus_registers[406]=(char)(inv[19]._Pout/256);			//Рег214
modbus_registers[407]=(char)(inv[19]._Pout%256);
modbus_registers[408]=(char)(inv[19]._Uacin/256);			//Рег215
modbus_registers[409]=(char)(inv[19]._Uacin%256);
modbus_registers[410]=(char)(inv[19]._Uload/256);			//Рег216
modbus_registers[411]=(char)(inv[19]._Uload%256);
i=inv[19]._flags_tm;
if(inv[19]._cnt>10)i|=0x80;
modbus_registers[412]=0;									//Рег217
modbus_registers[413]=i;
modbus_registers[414]=0;									//Рег218
modbus_registers[415]=0;
modbus_registers[416]=0;									//Рег219
modbus_registers[417]=0;
modbus_registers[418]=0;									//Рег220
modbus_registers[419]=0;

//Инвертор №21
modbus_registers[420]=(char)(inv[20]._Uout/256);			//Рег221
modbus_registers[421]=(char)(inv[20]._Uout%256);
modbus_registers[422]=(char)(inv[20]._Iout/256);			//Рег222
modbus_registers[423]=(char)(inv[20]._Iout%256);
modbus_registers[424]=(char)(inv[20]._T/256);				//Рег223
modbus_registers[425]=(char)(inv[20]._T%256);
modbus_registers[426]=(char)(inv[20]._Pout/256);			//Рег224
modbus_registers[427]=(char)(inv[20]._Pout%256);
modbus_registers[428]=(char)(inv[20]._Uacin/256);			//Рег225
modbus_registers[429]=(char)(inv[20]._Uacin%256);
modbus_registers[430]=(char)(inv[20]._Uload/256);			//Рег226
modbus_registers[431]=(char)(inv[20]._Uload%256);
i=inv[20]._flags_tm;
if(inv[20]._cnt>10)i|=0x80;
modbus_registers[432]=0;									//Рег227
modbus_registers[433]=i;
modbus_registers[434]=0;									//Рег228
modbus_registers[435]=0;
modbus_registers[436]=0;									//Рег229
modbus_registers[437]=0;
modbus_registers[438]=0;									//Рег230
modbus_registers[439]=0;

//Инвертор №22
modbus_registers[440]=(char)(inv[21]._Uout/256);			//Рег231
modbus_registers[441]=(char)(inv[21]._Uout%256);
modbus_registers[442]=(char)(inv[21]._Iout/256);			//Рег232
modbus_registers[443]=(char)(inv[21]._Iout%256);
modbus_registers[444]=(char)(inv[21]._T/256);				//Рег233
modbus_registers[445]=(char)(inv[21]._T%256);
modbus_registers[446]=(char)(inv[21]._Pout/256);			//Рег234
modbus_registers[447]=(char)(inv[21]._Pout%256);
modbus_registers[448]=(char)(inv[21]._Uacin/256);			//Рег235
modbus_registers[449]=(char)(inv[21]._Uacin%256);
modbus_registers[450]=(char)(inv[21]._Uload/256);			//Рег236
modbus_registers[451]=(char)(inv[21]._Uload%256);
i=inv[21]._flags_tm;
if(inv[21]._cnt>10)i|=0x80;
modbus_registers[452]=0;									//Рег237
modbus_registers[453]=i;
modbus_registers[454]=0;									//Рег238
modbus_registers[455]=0;
modbus_registers[456]=0;									//Рег239
modbus_registers[457]=0;
modbus_registers[458]=0;									//Рег240
modbus_registers[459]=0;


//Байпас
modbus_registers[800]=(char)(byps[0]._Uout/256);			//Рег401	  Напряжение выхода байпас Ф.А (0.1В)
modbus_registers[801]=(char)(byps[0]._Uout%256);
modbus_registers[802]=(char)(byps[1]._Uout/256);			//Рег402	  Напряжение выхода байпас Ф.B (0.1В)
modbus_registers[803]=(char)(byps[1]._Uout%256);
modbus_registers[804]=(char)(byps[2]._Uout/256);			//Рег403	  Напряжение выхода байпас Ф.C (0.1В)
modbus_registers[805]=(char)(byps[2]._Uout%256);
modbus_registers[806]=(char)(byps[0]._Iout/256);			//Рег404	  Ток выхода байпас Ф.А (0.1А)
modbus_registers[807]=(char)(byps[0]._Iout%256);
modbus_registers[808]=(char)(byps[1]._Iout/256);			//Рег405	  Ток выхода байпас Ф.B (0.1А)
modbus_registers[809]=(char)(byps[1]._Iout%256);
modbus_registers[810]=(char)(byps[2]._Iout/256);			//Рег406	  Ток выхода байпас Ф.C (0.1А)
modbus_registers[811]=(char)(byps[2]._Iout%256);
modbus_registers[812]=(char)(byps[0]._Pout/256);			//Рег407	  Мощность выхода байпас Ф.А (0.1Вт)
modbus_registers[813]=(char)(byps[0]._Pout%256);
modbus_registers[814]=(char)(byps[1]._Pout/256);			//Рег408	  Мощность выхода байпас Ф.B (0.1Вт)
modbus_registers[815]=(char)(byps[1]._Pout%256);
modbus_registers[816]=(char)(byps[2]._Pout/256);			//Рег409	  Мощность выхода байпас Ф.C (0.1Вт)
modbus_registers[817]=(char)(byps[2]._Pout%256);
modbus_registers[818]=(char)(byps[0]._T/256);				//Рег410	  Температура байпас Ф.А (1г.Ц.)
modbus_registers[819]=(char)(byps[0]._T%256);
modbus_registers[820]=(char)(byps[1]._T/256);				//Рег411	  Температура байпас Ф.B (1г.Ц.)
modbus_registers[821]=(char)(byps[1]._T%256);
modbus_registers[822]=(char)(byps[2]._T/256);				//Рег412	  Температура байпас Ф.C (1г.Ц.)
modbus_registers[823]=(char)(byps[2]._T%256);
modbus_registers[824]=(char)(byps[0]._UinACprim/256);			//Рег413	  Напряжение входа AC байпас Ф.А (0.1В)
modbus_registers[825]=(char)(byps[0]._UinACprim%256);
modbus_registers[826]=(char)(byps[1]._UinACprim/256);			//Рег414	  Напряжение входа AC байпас Ф.B (0.1В)
modbus_registers[827]=(char)(byps[1]._UinACprim%256);
modbus_registers[828]=(char)(byps[2]._UinACprim/256);			//Рег415	  Напряжение входа AC байпас Ф.C (0.1В)
modbus_registers[829]=(char)(byps[2]._UinACprim%256);
modbus_registers[830]=(char)(byps[0]._UinACinvbus/256);				//Рег416	  Напряжение входа AC инверторов байпас Ф.А (0.1В)
modbus_registers[831]=(char)(byps[0]._UinACinvbus%256);
modbus_registers[832]=(char)(byps[1]._UinACinvbus/256);				//Рег417	  Напряжение входа AC инверторов байпас Ф.B (0.1В)
modbus_registers[833]=(char)(byps[1]._UinACinvbus%256);
modbus_registers[834]=(char)(byps[2]._UinACinvbus/256);				//Рег418	  Напряжение входа AC инверторов байпас Ф.C (0.1В)
modbus_registers[835]=(char)(byps[2]._UinACinvbus%256);
modbus_registers[836]=(char)(byps[0]._flags/256);			//Рег419	  Флаги байпас Ф.А (0.1В)
modbus_registers[837]=(char)(byps[0]._flags%256);
modbus_registers[838]=(char)(byps[1]._flags/256);			//Рег420	  Флаги байпас Ф.B (0.1В)
modbus_registers[839]=(char)(byps[1]._flags%256);
modbus_registers[840]=(char)(byps[2]._flags/256);			//Рег421	  Флаги байпас Ф.C (0.1В)
modbus_registers[841]=(char)(byps[2]._flags%256);			//   1000000 работа от инверторов(0 - от сети)
															//   0100000 приоритет инверторы (0 - сеть)
															//	 0000100 температура больше 80 Г.ц.
															//	 0000010 температура больше 70 Г.ц.
/*
modbus_tx_buff[0]=adr;
modbus_tx_buff[1]=func;
modbus_tx_buff[2]=(char)(reg_quantity*2);

memcpy((char*)&modbus_tx_buff[3],(char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);

crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

modbus_tx_buff[3+(reg_quantity*2)]=crc_temp%256;
modbus_tx_buff[4+(reg_quantity*2)]=crc_temp/256;

for (i=0;i<(5+(reg_quantity*2));i++)
	{
	putchar0(modbus_tx_buff[i]);
	}
for (i=0;i<(5+(reg_quantity*2));i++)
	{
	putchar_sc16is700(modbus_tx_buff[i]);
	} */

if(prot==MODBUS_RTU_PROT)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;
	modbus_tx_buff[2]=(char)(reg_quantity*2);

	mem_copy((signed char*)&modbus_tx_buff[3],(signed char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);

	crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

	modbus_tx_buff[3+(reg_quantity*2)]=(char)crc_temp;
	modbus_tx_buff[4+(reg_quantity*2)]=crc_temp>>8;

	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar0(modbus_tx_buff[i]);
		}
	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar_sc16is700(modbus_tx_buff[i]);
		}
	}
else if(prot==MODBUS_TCP_PROT)
	{
	mem_copy((signed char*)modbus_tx_buff,(signed char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);
	modbus_tcp_out_ptr=(signed char*)modbus_tx_buff;
	}

}




//-----------------------------------------------
void modbus_register_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr)
{
char modbus_registers[120];
char modbus_tx_buff[100];
unsigned short crc_temp;
char i;

modbus_registers[0]=(char)(load_U/256);					//Рег1
modbus_registers[1]=(char)(load_U%256);
modbus_registers[2]=(char)(load_I/256);					//Рег2
modbus_registers[3]=(char)(load_I%256);
/*modbus_registers[4]=(char)((time_proc%60)/256);			//Рег3
modbus_registers[5]=(char)((time_proc%60)%256);
modbus_registers[6]=(char)((time_proc/60)/256);			//Рег4
modbus_registers[7]=(char)((time_proc/60)%256);
modbus_registers[8]=(char)((time_proc/3600)/256);			//Рег5
modbus_registers[9]=(char)((time_proc/3600)%256);		 	
modbus_registers[10]=(char)((time_proc_remain%60)/256);	//Рег6
modbus_registers[11]=(char)((time_proc_remain%60)%256);
modbus_registers[12]=(char)((time_proc_remain/60)/256);	//Рег7
modbus_registers[13]=(char)((time_proc_remain/60)%256);
modbus_registers[14]=(char)((time_proc_remain/3600)/256);	//Рег8
modbus_registers[15]=(char)((time_proc_remain/3600)%256);
modbus_registers[16]=(char)(I_ug/256);					//Рег9
modbus_registers[17]=(char)(I_ug%256);
modbus_registers[18]=(char)(U_up/256);					//Рег10
modbus_registers[19]=(char)(U_up%256);
modbus_registers[20]=(char)(U_maxg/256);				//Рег11
modbus_registers[21]=(char)(U_maxg%256);
modbus_registers[22]=(char)(I_maxp/256);				//Рег12
modbus_registers[23]=(char)(I_maxp%256);
modbus_registers[24]=(char)((T_PROC_GS%60)/256);			//Рег13
modbus_registers[25]=(char)((T_PROC_GS%60)%256);
modbus_registers[26]=(char)((T_PROC_GS/60)/256);			//Рег14
modbus_registers[27]=(char)((T_PROC_GS/60)%256);
modbus_registers[28]=(char)((T_PROC_GS/3600)/256);		//Рег15
modbus_registers[29]=(char)((T_PROC_GS/3600)%256);
modbus_registers[30]=(char)((T_PROC_PS%60)/256);			//Рег16
modbus_registers[31]=(char)((T_PROC_PS%60)%256);
modbus_registers[32]=(char)((T_PROC_PS/60)/256);			//Рег17
modbus_registers[33]=(char)((T_PROC_PS/60)%256);
modbus_registers[34]=(char)((T_PROC_PS/3600)/256);		//Рег18
modbus_registers[35]=(char)((T_PROC_PS/3600)%256);
modbus_registers[36]=0;								//Рег19
modbus_registers[37]=0;
if(work_stat==wsPS)modbus_registers[37]=1;
modbus_registers[38]=0;								//Рег20
modbus_registers[39]=0;
if(work_stat==wsGS)modbus_registers[39]=1;
modbus_registers[40]=0;								//Рег21
modbus_registers[41]=0;
if(REV_STAT==rsREW)modbus_registers[41]=1;
modbus_registers[42]=0;								//Рег22
modbus_registers[43]=0;
if(AVT_REV_IS_ON)modbus_registers[42]=1;

modbus_tx_buff[0]=adr;
modbus_tx_buff[1]=func;
modbus_tx_buff[2]=(char)(reg_adr/256);
modbus_tx_buff[3]=(char)(reg_adr%256);
//modbus_tx_buff[4]=(char)(reg_quantity/256);
//modbus_tx_buff[5]=(char)(reg_quantity%256);
/*
modbus_registers[0]=0x10;
modbus_registers[1]=0x11;
modbus_registers[2]=0x12;
modbus_registers[3]=0x13;
modbus_registers[4]=0x14;
modbus_registers[5]=0x15;
*/

memcpy((char*)&modbus_tx_buff[4],(char*)&modbus_registers[(reg_adr-1)*2],2);

crc_temp=CRC16_2(modbus_tx_buff,6);

modbus_tx_buff[6]=crc_temp%256;
modbus_tx_buff[7]=crc_temp/256;

for (i=0;i<8;i++)
	{
	putchar0(modbus_tx_buff[i]);
	}
for (i=0;i<8;i++)
	{
	putchar_sc16is700(modbus_tx_buff[i]);
	}
}


//-----------------------------------------------
void modbus_hold_register_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr, char prot)
{
char modbus_registers[120];
char modbus_tx_buff[100];
unsigned short crc_temp;
char i;

/*modbus_registers[0]=(char)(I_ug/256);					//Рег50
modbus_registers[1]=(char)(I_ug%256);
modbus_registers[2]=(char)(U_up/256);					//Рег51
modbus_registers[3]=(char)(U_up%256);
modbus_registers[4]=(char)(U_maxg/256);					//Рег52
modbus_registers[5]=(char)(U_maxg%256);
modbus_registers[6]=(char)(I_maxp/256);					//Рег53
modbus_registers[7]=(char)(I_maxp%256);
modbus_registers[8]=(char)((T_PROC_GS%60)/256);			//Рег54
modbus_registers[9]=(char)((T_PROC_GS%60)%256);
modbus_registers[10]=(char)(((T_PROC_GS/60)%60)/256);		//Рег55
modbus_registers[11]=(char)(((T_PROC_GS/60)%60)%256);
modbus_registers[12]=(char)((T_PROC_GS/3600)/256);		//Рег56
modbus_registers[13]=(char)((T_PROC_GS/3600)%256);
modbus_registers[14]=(char)((T_PROC_PS%60)/256);			//Рег57
modbus_registers[15]=(char)((T_PROC_PS%60)%256);
modbus_registers[16]=(char)(((T_PROC_PS/60)%60)/256);		//Рег58
modbus_registers[17]=(char)(((T_PROC_PS/60)%60)%256);
modbus_registers[18]=(char)((T_PROC_PS/3600)/256);		//Рег59
modbus_registers[19]=(char)((T_PROC_PS/3600)%256);
modbus_registers[20]=0;								//Рег60
modbus_registers[21]=0;
if(work_stat==wsPS)modbus_registers[21]=1;
modbus_registers[22]=0;								//Рег61
modbus_registers[23]=0;
if(work_stat==wsGS)modbus_registers[23]=1;
modbus_registers[24]=0;								//Рег62
modbus_registers[25]=0;
if(REV_STAT==rsREW)modbus_registers[25]=1;
modbus_registers[26]=0;								//Рег63
modbus_registers[27]=0;
if(AVT_REV_IS_ON)modbus_registers[27]=1;
modbus_registers[28]=(char)((AVT_REV_TIME_FF)/256);		//Рег64
modbus_registers[29]=(char)((AVT_REV_TIME_FF)%256);
modbus_registers[30]=(char)((AVT_REV_TIME_REW)/256);		//Рег65
modbus_registers[31]=(char)((AVT_REV_TIME_REW)%256);
modbus_registers[32]=(char)((AVT_REV_TIME_PAUSE)/256);		//Рег66
modbus_registers[33]=(char)((AVT_REV_TIME_PAUSE)%256);
modbus_registers[34]=(char)((AVT_REV_I_NOM_FF)/256);		//Рег67
modbus_registers[35]=(char)((AVT_REV_I_NOM_FF)%256);
modbus_registers[36]=(char)((AVT_REV_I_NOM_REW)/256);		//Рег68
modbus_registers[37]=(char)((AVT_REV_I_NOM_REW)%256);
modbus_registers[38]=(char)((AVT_REV_U_NOM_FF)/256);		//Рег69
modbus_registers[39]=(char)((AVT_REV_U_NOM_FF)%256);
modbus_registers[40]=(char)((AVT_REV_U_NOM_REW)/256);		//Рег70
modbus_registers[41]=(char)((AVT_REV_U_NOM_REW)%256);
modbus_registers[42]=(char)((CAP_ZAR_TIME)/256);			//Рег71
modbus_registers[43]=(char)((CAP_ZAR_TIME)%256);
modbus_registers[44]=(char)((CAP_PAUSE1_TIME)/256);		//Рег72
modbus_registers[45]=(char)((CAP_PAUSE1_TIME)%256);
modbus_registers[46]=(char)((CAP_RAZR_TIME)/256);			//Рег73
modbus_registers[47]=(char)((CAP_RAZR_TIME)%256);
modbus_registers[48]=(char)((CAP_PAUSE2_TIME)/256);		//Рег74
modbus_registers[49]=(char)((CAP_PAUSE2_TIME)%256);
modbus_registers[50]=(char)((CAP_MAX_VOLT)/256);			//Рег75
modbus_registers[51]=(char)((CAP_MAX_VOLT)%256);
modbus_registers[52]=(char)((CAP_WRK_CURR)/256);			//Рег76
modbus_registers[53]=(char)((CAP_WRK_CURR)%256);	*/

modbus_tx_buff[0]=adr;
modbus_tx_buff[1]=func;
modbus_tx_buff[2]=(char)(reg_adr/256);
modbus_tx_buff[3]=(char)(reg_adr%256);
//modbus_tx_buff[4]=(char)(reg_quantity/256);
//modbus_tx_buff[5]=(char)(reg_quantity%256);

/*
memcpy((char*)&modbus_tx_buff[4],(char*)&modbus_registers[(reg_adr-1)*2],2);

crc_temp=CRC16_2(modbus_tx_buff,6);

modbus_tx_buff[6]=crc_temp%256;
modbus_tx_buff[7]=crc_temp/256;

for (i=0;i<8;i++)
	{
	putchar0(modbus_tx_buff[i]);
	}
for (i=0;i<8;i++)
	{
	putchar_sc16is700(modbus_tx_buff[i]);
	}*/

if(prot==MODBUS_RTU_PROT)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;
	modbus_tx_buff[2]=(char)(2);

	mem_copy((signed char*)&modbus_tx_buff[3],(signed char*)&modbus_registers[(reg_adr-1)*2],2);

	crc_temp=CRC16_2(modbus_tx_buff,(2)+3);

	modbus_tx_buff[3+(2)]=(char)crc_temp;
	modbus_tx_buff[4+(2)]=crc_temp>>8;

	for (i=0;i<(5+(2));i++)
		{
		putchar0(modbus_tx_buff[i]);
		}
	for (i=0;i<(5+(2));i++)
		{
		putchar_sc16is700(modbus_tx_buff[i]);
		}
	}
else if(prot==MODBUS_TCP_PROT)
	{
	mem_copy((signed char*)modbus_tx_buff,(signed char*)&modbus_registers[(reg_adr-1)*2],2);
	modbus_tcp_out_ptr=(signed char*)modbus_tx_buff;
	}
}	


//-----------------------------------------------
void modbus_hold_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot)
{
char modbus_registers[110];
char modbus_tx_buff[120];
unsigned short crc_temp;
char i;



modbus_registers[20]=(char)((LPC_RTC->YEAR)>>8);		//Рег11  Время, год
modbus_registers[21]=(char)((LPC_RTC->YEAR));
modbus_registers[22]=(char)((LPC_RTC->MONTH)>>8);		//Рег12  Время, месяц
modbus_registers[23]=(char)((LPC_RTC->MONTH));
modbus_registers[24]=(char)((LPC_RTC->DOM)>>8);			//Рег13  Время, день месяца
modbus_registers[25]=(char)((LPC_RTC->DOM));
modbus_registers[26]=(char)((LPC_RTC->HOUR)>>8);		//Рег14  Время, час
modbus_registers[27]=(char)((LPC_RTC->HOUR));
modbus_registers[28]=(char)((LPC_RTC->MIN)>>8);			//Рег15  Время, минуты
modbus_registers[29]=(char)((LPC_RTC->MIN));
modbus_registers[30]=(char)((LPC_RTC->SEC)>>8);			//Рег16  Время, секунды
modbus_registers[31]=(char)((LPC_RTC->SEC));




modbus_registers[40]=(char)(U_OUT_SET/256);					//Рег11	Уставка выходного напряжения
modbus_registers[41]=(char)(U_OUT_SET%256);
/*modbus_registers[2]=(char)(U_up/256);					//Рег51
modbus_registers[3]=(char)(U_up%256);
modbus_registers[4]=(char)(U_maxg/256);					//Рег52
modbus_registers[5]=(char)(U_maxg%256);
modbus_registers[6]=(char)(I_maxp/256);					//Рег53
modbus_registers[7]=(char)(I_maxp%256);
modbus_registers[8]=(char)((T_PROC_GS%60)/256);			//Рег54
modbus_registers[9]=(char)((T_PROC_GS%60)%256);
modbus_registers[10]=(char)(((T_PROC_GS/60)%60)/256);		//Рег55
modbus_registers[11]=(char)(((T_PROC_GS/60)%60)%256);
modbus_registers[12]=(char)((T_PROC_GS/3600)/256);		//Рег56
modbus_registers[13]=(char)((T_PROC_GS/3600)%256);
modbus_registers[14]=(char)((T_PROC_PS%60)/256);			//Рег57
modbus_registers[15]=(char)((T_PROC_PS%60)%256);
modbus_registers[16]=(char)(((T_PROC_PS/60)%60)/256);		//Рег58
modbus_registers[17]=(char)(((T_PROC_PS/60)%60)%256);
modbus_registers[18]=(char)((T_PROC_PS/3600)/256);		//Рег59
modbus_registers[19]=(char)((T_PROC_PS/3600)%256);
modbus_registers[20]=0;								//Рег60
modbus_registers[21]=0;
if(work_stat==wsPS)modbus_registers[21]=1;
modbus_registers[22]=0;								//Рег61
modbus_registers[23]=0;
if(work_stat==wsGS)modbus_registers[23]=1;
modbus_registers[24]=0;								//Рег62
modbus_registers[25]=0;
if(REV_STAT==rsREW)modbus_registers[25]=1;
modbus_registers[26]=0;								//Рег63
modbus_registers[27]=0;
if(AVT_REV_IS_ON)modbus_registers[27]=1;
modbus_registers[28]=(char)((AVT_REV_TIME_FF)/256);		//Рег64
modbus_registers[29]=(char)((AVT_REV_TIME_FF)%256);
modbus_registers[30]=(char)((AVT_REV_TIME_REW)/256);		//Рег65
modbus_registers[31]=(char)((AVT_REV_TIME_REW)%256);
modbus_registers[32]=(char)((AVT_REV_TIME_PAUSE)/256);		//Рег66
modbus_registers[33]=(char)((AVT_REV_TIME_PAUSE)%256);
modbus_registers[34]=(char)((AVT_REV_I_NOM_FF)/256);		//Рег67
modbus_registers[35]=(char)((AVT_REV_I_NOM_FF)%256);
modbus_registers[36]=(char)((AVT_REV_I_NOM_REW)/256);		//Рег68
modbus_registers[37]=(char)((AVT_REV_I_NOM_REW)%256);
modbus_registers[38]=(char)((AVT_REV_U_NOM_FF)/256);		//Рег69
modbus_registers[39]=(char)((AVT_REV_U_NOM_FF)%256);
modbus_registers[40]=(char)((AVT_REV_U_NOM_REW)/256);		//Рег70
modbus_registers[41]=(char)((AVT_REV_U_NOM_REW)%256);
modbus_registers[42]=(char)((CAP_ZAR_TIME)/256);			//Рег71
modbus_registers[43]=(char)((CAP_ZAR_TIME)%256);
modbus_registers[44]=(char)((CAP_PAUSE1_TIME)/256);		//Рег72
modbus_registers[45]=(char)((CAP_PAUSE1_TIME)%256);
modbus_registers[46]=(char)((CAP_RAZR_TIME)/256);			//Рег73
modbus_registers[47]=(char)((CAP_RAZR_TIME)%256);
modbus_registers[48]=(char)((CAP_PAUSE2_TIME)/256);		//Рег74
modbus_registers[49]=(char)((CAP_PAUSE2_TIME)%256);
modbus_registers[50]=(char)((CAP_MAX_VOLT)/256);			//Рег75
modbus_registers[51]=(char)((CAP_MAX_VOLT)%256);
modbus_registers[52]=(char)((CAP_WRK_CURR)/256);			//Рег76
modbus_registers[53]=(char)((CAP_WRK_CURR)%256);	   */

/*
modbus_tx_buff[0]=adr;
modbus_tx_buff[1]=func;
modbus_tx_buff[2]=(char)(reg_quantity*2);

memcpy((char*)&modbus_tx_buff[3],(char*)&modbus_registers[(reg_adr-50)*2],reg_quantity*2);
						   
crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

modbus_tx_buff[3+(reg_quantity*2)]=crc_temp%256;
modbus_tx_buff[4+(reg_quantity*2)]=crc_temp/256;

for (i=0;i<(5+(reg_quantity*2));i++)
	{
	putchar0(modbus_tx_buff[i]);
	}
for (i=0;i<(5+(reg_quantity*2));i++)
	{
	putchar_sc16is700(modbus_tx_buff[i]);
	}*/
if(prot==MODBUS_RTU_PROT)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;
	modbus_tx_buff[2]=(char)(reg_quantity*2);

	mem_copy((signed char*)&modbus_tx_buff[3],(signed char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);

	crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

	modbus_tx_buff[3+(reg_quantity*2)]=(char)crc_temp;
	modbus_tx_buff[4+(reg_quantity*2)]=crc_temp>>8;

	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar0(modbus_tx_buff[i]);
		}
	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar_sc16is700(modbus_tx_buff[i]);
		}
	}
else if(prot==MODBUS_TCP_PROT)
	{
	mem_copy((signed char*)modbus_tx_buff,(signed char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);
	modbus_tcp_out_ptr=(signed char*)modbus_tx_buff;
	}

			modbus_rtu_plazma[0]=modbus_tx_buff[0];
			modbus_rtu_plazma[1]=modbus_tx_buff[1];
			modbus_rtu_plazma[2]=modbus_tx_buff[2];
			modbus_rtu_plazma[3]=modbus_tx_buff[3];
			modbus_rtu_plazma[4]=modbus_tx_buff[4];
			modbus_rtu_plazma[5]=modbus_tx_buff[5];
			modbus_rtu_plazma[6]=modbus_tx_buff[6];
			modbus_rtu_plazma[7]=modbus_tx_buff[7];
			modbus_rtu_plazma[8]=modbus_tx_buff[8];
			modbus_rtu_plazma[9]=modbus_tx_buff[9];
}

//-----------------------------------------------
void modbus_hold_register_write(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_value, char prot)
{
char modbus_registers[110];
char modbus_tx_buff[120];
unsigned short crc_temp;
short tempS;
/*
if(reg_adr==11)
	{
	U_OUT_SET=reg_value;
	modbus_tcp_plazma[1]=reg_value;
	//gran(&U_OUT_SET,220,230);
	lc640_write_int(EE_U_OUT_SET,U_OUT_SET);
	tempS=U_OUT_SET;
	modbus_tcp_plazma[0]++;
	}

modbus_registers[0]=(char)(tempS);
modbus_registers[1]=(char)(tempS>>8);

if(prot==MODBUS_RTU_PROT)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;
	modbus_tx_buff[2]=2;
	mem_copy((signed char*)&modbus_tx_buff[3],(signed char*)&modbus_registers[0],2);

	crc_temp=CRC16_2(modbus_tx_buff,5);

	modbus_tx_buff[5]=(char)crc_temp;
	modbus_tx_buff[6]=(char)crc_temp>>8;

	for (i=0;i<7;i++)
		{
		putchar0(modbus_tx_buff[i]);
		}
	for (i=0;i<7;i++)
		{
		putchar_sc16is700(modbus_tx_buff[i]);
		}
	}
else if(prot==MODBUS_TCP_PROT)
	{
	mem_copy((signed char*)modbus_tx_buff,(signed char*)&modbus_registers[0],2);
	modbus_tcp_out_ptr=(signed char*)modbus_tx_buff;
	}*/

if(reg_adr==11)		//Установка времени 
	{
	tempS=(uint16_t)reg_value;
	gran(&tempS,10,30);
	LPC_RTC->YEAR=tempS;
	}
if(reg_adr==12)		//Установка времени 
	{
	LPC_RTC->MONTH=(uint16_t)reg_value;
	}
if(reg_adr==13)		//Установка времени 
	{
	LPC_RTC->DOM=(uint16_t)reg_value;
	}
if(reg_adr==14)		//Установка времени 
	{
	LPC_RTC->HOUR=(uint16_t)reg_value;
	}
if(reg_adr==15)		//Установка времени 
	{
	LPC_RTC->MIN=(uint16_t)reg_value;
	}
if(reg_adr==16)		//Установка времени 
	{
	LPC_RTC->SEC=(uint16_t)reg_value;
	}

//modbus_hold_register_transmit(MODBUS_ADRESS,func,reg_adr,prot);
//modbus_hold_registers_transmit(MODBUS_ADRESS,func,reg_adr,1,prot);

if(prot==MODBUS_RTU_PROT)
	{

	mem_copy(modbus_tx_buff,modbus_rx_buffer,8);
	//mem_copy(&modbus_tx_buff[4],&tempS,2);
	modbus_tx_buff[4]=(char)(tempS/256);
	modbus_tx_buff[5]=(char)(tempS%256);

	tempS=CRC16_2(modbus_tx_buff,6);

	modbus_tx_buff[6]=(char)tempS;
	modbus_tx_buff[7]=tempS>>8;

	for (i=0;i<(8);i++)
		{
		putchar0(modbus_tx_buff[i]);
		}

	for (i=0;i<(8);i++)
		{
		putchar_sc16is700(modbus_tx_buff[i]);
		}
	}
else if(prot==MODBUS_TCP_PROT)
	{
/*	U8 *sendbuf;
				sendbuf = tcp_get_buf(11);
				//sendbuf[0]=ptr[0];
				//sendbuf[1]=ptr[1];
				//sendbuf[2]=ptr[2];
				//sendbuf[3]=ptr[3];
				sendbuf[4]=0;
				sendbuf[5]=6;
				sendbuf[6]=modbus_tcp_unit;
				sendbuf[7]=modbus_tcp_func;
				sendbuf[8]=modbus_tcp_rx_arg0/256;
				sendbuf[9]=modbus_tcp_rx_arg0%256;
				mem_copy((char*)&sendbuf[10],modbus_tcp_out_ptr,2);
				//sendbuf[9]=3;
				//sendbuf[10]=4;
	          	tcp_send (socket_tcp, sendbuf, 12);	*/

	modbus_tcp_out_ptr=(char*)&modbus_registers[(reg_adr-1)*2];
	}

}
