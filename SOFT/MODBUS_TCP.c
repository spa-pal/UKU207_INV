
#include <lpc17xx.h>
#include <rtl.h>
#include "main.h"
#include "modbus_tcp.h"
#include "eeprom_map.h"
#include "25lc640.h"
#include "MODBUS_func4.h" //o_1

char plazma_modbus_tcp[20];
char modbus_tcp_plazma[20];

char modbus_tcp_func;
char modbus_tcp_unit;
short modbus_tcp_rx_arg0;
short modbus_tcp_rx_arg1;
unsigned long start_adr_tcp_mb;//o_1
unsigned char start_num_tcp_mb;//o_1
//#define MODBUS_TCP_PROT	1

char* modbus_tcp_out_ptr;

/*--------------------------- TCP socket ------------------------------------*/

U16 tcp_callback (U8 soc, U8 evt, U8 *ptr, U16 par) 
{
/* This function is called by the TCP module on TCP event */
/* Check the 'Net_Config.h' for possible events.          */
par = par;

if (soc != socket_tcp) {
	return (0);
}

modbus_tcp_plazma[0]++;

switch (evt) 
	{
    case TCP_EVT_DATA:
    /* TCP data frame has arrived, data is located at *par1, */
    /* data length is par2. Allocate buffer to send reply.   */
    //procrec(ptr);
	//modbus_tcp_plazma[1]++;	

	plazma_modbus_tcp[1]=ptr[0];
	plazma_modbus_tcp[2]=ptr[1];
	plazma_modbus_tcp[3]=ptr[2];
	plazma_modbus_tcp[4]=ptr[3];
	plazma_modbus_tcp[5]=ptr[4];
	plazma_modbus_tcp[6]=ptr[5];
	plazma_modbus_tcp[7]=ptr[6];
	plazma_modbus_tcp[8]=ptr[7];
	plazma_modbus_tcp[9]=ptr[8];
	plazma_modbus_tcp[10]=ptr[9];
	plazma_modbus_tcp[11]=ptr[10];
	  //plazma_modbus_tcp[4]=ptr[3];
	  //plazma_modbus_tcp[5]=par;
	  //plazma_modbus_tcp[6]=ptr[5];

   	
	modbus_tcp_func=ptr[7];
	modbus_tcp_unit=ptr[6];

	modbus_tcp_rx_arg0=(((unsigned short)ptr[8])*((unsigned short)256))+((unsigned short)ptr[9]);
	//modbus_tcp_rx_arg0++;	//o_1
	modbus_tcp_rx_arg1=(((unsigned short)ptr[10])*((unsigned short)256))+((unsigned short)ptr[11]);

	if(modbus_tcp_unit==MODBUS_ADRESS)
		{
		//char modbus_tcp_tx_buff[200];	 //o_1
		start_adr_tcp_mb=modbus_tcp_rx_arg0*2; //o_1
		start_num_tcp_mb=modbus_tcp_rx_arg1*2; //o_1

		if(modbus_tcp_func==3)		//чтение произвольного кол-ва регистров хранения
			{
			
			/* //o_1
			modbus_hold_registers_transmit(MODBUS_ADRESS,modbus_tcp_func,modbus_tcp_rx_arg0,modbus_tcp_rx_arg1,MODBUS_TCP_PROT);
			sendbuf = tcp_get_buf((modbus_tcp_rx_arg1*2)+9);
			sendbuf[0]=ptr[0];
			sendbuf[1]=ptr[1];
			sendbuf[2]=ptr[2];
			sendbuf[3]=ptr[3];
			sendbuf[4]=((modbus_tcp_rx_arg1*2)+3)/256;
			sendbuf[5]=((modbus_tcp_rx_arg1*2)+3)%256;;
			sendbuf[6]=modbus_tcp_unit;
			sendbuf[7]=modbus_tcp_func;
			sendbuf[8]=(U8)(modbus_tcp_rx_arg1*2);
			mem_copy((char*)&sendbuf[9],modbus_tcp_out_ptr,(modbus_tcp_rx_arg1*2));
			//sendbuf[9]=3;
			//sendbuf[10]=4;
          	tcp_send (socket_tcp, sendbuf, ((modbus_tcp_rx_arg1*2)+9));
			//
			*/ //o_1

			//o_1_s
			// если появятся регистры записи, то ниже разремировать и создать таблицу адресов регистров reg_func3
			/*unsigned char io;
			U8 *sendbuf;
			sendbuf = tcp_get_buf(start_num_tcp_mb+9);
			sendbuf[0]=ptr[0];
			sendbuf[1]=ptr[1];
			sendbuf[2]=ptr[2];
			sendbuf[3]=ptr[3];
			sendbuf[4]=(start_num_tcp_mb+3)/256;
			sendbuf[5]=(start_num_tcp_mb+3)%256;;
			sendbuf[6]=modbus_tcp_unit;
			sendbuf[7]=modbus_tcp_func;
			sendbuf[8]=(U8)start_num_tcp_mb;
			wr_reg_func3(start_adr_tcp_mb, start_adr_tcp_mb+start_num_tcp_mb);
			for(io=0;io<sendbuf[8];++io) sendbuf[9+io]=*reg_func3[start_adr_tcp_mb+io];	 
	 		tcp_send (socket_tcp, sendbuf, (start_num_tcp_mb+9));
			*/
			//o_1_e

			//modbus_tcp_tx_buff[4]=0;
			//modbus_tcp_tx_buff[5]=5;
			//modbus_tcp_tx_buff[6]=1;//modbus_tcp_unit;
			//modbus_tcp_tx_buff[7]=3;//modbus_tcp_func;
			//modbus_tcp_tx_buff[8]=2;
			//mem_copy((char*)&modbus_tcp_tx_buff[9],modbus_tcp_out_ptr,2);
			//modbus_tcp_tx_buff[9]=2;
			//modbus_tcp_tx_buff[10]=3;
			//tcp_send (socket_tcp, modbus_tcp_tx_buff, 11);


			}

		if(modbus_tcp_func==4)		//чтение произвольного кол-ва регистров	входов
			{
			U8 *sendbuf;

			//modbus_tcp_plazma[2]++;
			/*  //o_1
			modbus_input_registers_transmit(MODBUS_ADRESS,modbus_tcp_func,modbus_tcp_rx_arg0-1,modbus_tcp_rx_arg1,MODBUS_TCP_PROT);

			sendbuf = tcp_get_buf((modbus_tcp_rx_arg1*2)+9);
			sendbuf[0]=ptr[0];
			sendbuf[1]=ptr[1];
			sendbuf[2]=ptr[2];
			sendbuf[3]=ptr[3];
			sendbuf[4]=((modbus_tcp_rx_arg1*2)+3)/256;
			sendbuf[5]=((modbus_tcp_rx_arg1*2)+3)%256;;
			sendbuf[6]=modbus_tcp_unit;
			sendbuf[7]=modbus_tcp_func;
			sendbuf[8]=(U8)(modbus_tcp_rx_arg1*2);
			mem_copy((char*)&sendbuf[9],modbus_tcp_out_ptr,(modbus_tcp_rx_arg1*2));
			//sendbuf[9]=3;
			//sendbuf[10]=4;
          	tcp_send (socket_tcp, sendbuf, ((modbus_tcp_rx_arg1*2)+9));
			//
			*/ //o_1

			//o_1_s
			unsigned char io;
			sendbuf = tcp_get_buf(start_num_tcp_mb+9);
			sendbuf[0]=ptr[0];
			sendbuf[1]=ptr[1];
			sendbuf[2]=ptr[2];
			sendbuf[3]=ptr[3];
			sendbuf[4]=(start_num_tcp_mb+3)/256;
			sendbuf[5]=(start_num_tcp_mb+3)%256;;
			sendbuf[6]=modbus_tcp_unit;
			sendbuf[7]=modbus_tcp_func;
			sendbuf[8]=(U8)start_num_tcp_mb;
			wr_reg_func4();		 
			for(io=0;io<sendbuf[8];++io) sendbuf[9+io]=*reg_func4[start_adr_tcp_mb+io];	 
          	tcp_send (socket_tcp, sendbuf, (start_num_tcp_mb+9));
			//o_1_e

			//modbus_tcp_tx_buff[4]=0;
			//modbus_tcp_tx_buff[5]=5;
			//modbus_tcp_tx_buff[6]=1;//modbus_tcp_unit;
			//modbus_tcp_tx_buff[7]=3;//modbus_tcp_func;
			//modbus_tcp_tx_buff[8]=2;
			//mem_copy((char*)&modbus_tcp_tx_buff[9],modbus_tcp_out_ptr,2);
			//modbus_tcp_tx_buff[9]=2;
			//modbus_tcp_tx_buff[10]=3;
			//tcp_send (socket_tcp, modbus_tcp_tx_buff, 11);
			}

		else if(modbus_tcp_func==6) 	//запись регистров хранения
			{ //o_1_s   обработку 6 функции заменить полностью
				U8 *sendbuf;
				//если будут команды записи-разремировать строку ниже и сделать анализ в функции analiz_func6 для TCP и UDP одинаково
				//analiz_func6(modbus_tcp_rx_arg0, modbus_tcp_rx_arg1);
				sendbuf = tcp_get_buf(12);
				sendbuf[0]=ptr[0];
				sendbuf[1]=ptr[1];
				sendbuf[2]=ptr[2];
				sendbuf[3]=ptr[3];
				sendbuf[4]=0;
				sendbuf[5]=6;
				sendbuf[6]=modbus_tcp_unit;
				sendbuf[7]=modbus_tcp_func;
				sendbuf[8]=modbus_tcp_rx_arg0/256;
				sendbuf[9]=modbus_tcp_rx_arg0%256;
				sendbuf[10]=modbus_tcp_rx_arg1/256;
				sendbuf[11]=modbus_tcp_rx_arg1%256;
	          	tcp_send (socket_tcp, sendbuf, 12);
				}


			 //o_1_e
			} 
      	break;

    	case TCP_EVT_CONREQ:
      		/* Remote peer requested connect, accept it */
			//ica_plazma[5]++;
      		return (1);

    	case TCP_EVT_CONNECT:
      		/* The TCP socket is connected */
			tcp_connect_stat=1;
			//ica_plazma[6]++;
      		return (1);

    	case TCP_EVT_CLOSE: 
      		/* The TCP socket is connected */
			tcp_connect_stat=0;
			//ica_plazma[7]++;
      		return (1);
  		}
  	return (0);
}
