#include "http_data.h"
#include "control.h"
#include "eeprom_map.h"
#include "25lc640.h"
#include "common_func.h"
#include "main.h"
#include "stdio.h"
#include "avar_hndl.h"

//Телеметрия сети
char http_power_num_of_phases;
short http_power_voltage_of_phase[3];
short http_power_frequncy;
char http_power_status;
char http_output_buff[70];
const char hex_alfa[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
char log_item_cnt=0;
char pal_cyr_coder_output[200];

//-----------------------------------------------
char* pal_cyr_decoder(char* input) 
{
char* output;
short i=0,ii=0;

output = pal_cyr_coder_output;
	
while (input[i])
	{
	if(input[i]=='^')
		{
		i++;
		if(input[i]=='X')
			{
			i++;
			if(input[i]=='A') output[ii++]='Ш';
			else if(input[i]=='E') output[ii++]='Ё';
			else if(input[i]=='C') output[ii++]='Ж';
			else if(input[i]=='D') output[ii++]='Щ';
			else if(input[i]=='B') output[ii++]='Ъ';
			else if(input[i]=='F') output[ii++]='Ы';
			else if(input[i]=='G') output[ii++]='Ь';
			else if(input[i]=='H') output[ii++]='Э';
			else if(input[i]=='a') output[ii++]='ш';
			else if(input[i]=='e') output[ii++]='ё';
			else if(input[i]=='c') output[ii++]='ж';
			else if(input[i]=='d') output[ii++]='щ';
			else if(input[i]=='b') output[ii++]='ъ';
			else if(input[i]=='f') output[ii++]='ы';
			else if(input[i]=='g') output[ii++]='ь';
			else if(input[i]=='y') output[ii++]='э';
			else if(input[i]=='i') output[ii++]='°';
			else if(input[i]=='j') output[ii++]='#';
			}
		else if(input[i]=='A') output[ii++]='А';
		else if(input[i]=='B') output[ii++]='Б';
		else if(input[i]=='C') output[ii++]='Ц';
		else if(input[i]=='D') output[ii++]='Д';
		else if(input[i]=='E') output[ii++]='Е';
		else if(input[i]=='F') output[ii++]='Ф';
		else if(input[i]=='G') output[ii++]='Г';
		else if(input[i]=='H') output[ii++]='Х';
		else if(input[i]=='I') output[ii++]='И';
		else if(input[i]=='J') output[ii++]='Й';
		else if(input[i]=='K') output[ii++]='К';
		else if(input[i]=='L') output[ii++]='Л';
		else if(input[i]=='M') output[ii++]='М';
		else if(input[i]=='N') output[ii++]='Н';
		else if(input[i]=='O') output[ii++]='О';
		else if(input[i]=='P') output[ii++]='П';
		else if(input[i]=='Q') output[ii++]='Я';
		else if(input[i]=='R') output[ii++]='Р';
		else if(input[i]=='S') output[ii++]='С';
		else if(input[i]=='T') output[ii++]='Т';
		else if(input[i]=='U') output[ii++]='У';
		else if(input[i]=='V') output[ii++]='Ю';
		else if(input[i]=='W') output[ii++]='В';
		else if(input[i]=='Y') output[ii++]='Ч';
		else if(input[i]=='Z') output[ii++]='З';
		else if(input[i]=='a') output[ii++]='а';
		else if(input[i]=='b') output[ii++]='б';
		else if(input[i]=='c') output[ii++]='ц';
		else if(input[i]=='d') output[ii++]='д';
		else if(input[i]=='e') output[ii++]='е';
		else if(input[i]=='f') output[ii++]='ф';
		else if(input[i]=='g') output[ii++]='г';
		else if(input[i]=='h') output[ii++]='х';
		else if(input[i]=='i') output[ii++]='и';
		else if(input[i]=='j') output[ii++]='й';
		else if(input[i]=='k') output[ii++]='к';
		else if(input[i]=='l') output[ii++]='л';
		else if(input[i]=='m') output[ii++]='м';
		else if(input[i]=='n') output[ii++]='н';
		else if(input[i]=='o') output[ii++]='о';
		else if(input[i]=='p') output[ii++]='п';
		else if(input[i]=='q') output[ii++]='я';
		else if(input[i]=='r') output[ii++]='р';
		else if(input[i]=='s') output[ii++]='с';
		else if(input[i]=='t') output[ii++]='т';
		else if(input[i]=='u') output[ii++]='у';
		else if(input[i]=='v') output[ii++]='ю';
		else if(input[i]=='w') output[ii++]='в';
		else if(input[i]=='y') output[ii++]='ч';
		else if(input[i]=='z') output[ii++]='з';
		i++;
		}
	else 
		{
		output[ii++]=input[i++];
		}
	}
output[ii]=0;
return output;
}

//-----------------------------------------------
char* pal_cyr_coder(char* in)
{
char* output;
short i=0,ii=0;
output = pal_cyr_coder_output;

while(in[i])
	{
	if(in[i]=='А')
		{
		output[ii++]='^';
		output[ii++]='A';
		i++;
		}
	else if(in[i]=='Б')
		{
		output[ii++]='^';
		output[ii++]='B';
		i++;
		}
	else if(in[i]=='В')
		{
		output[ii++]='^';
		output[ii++]='W';
		i++;
		}
	else if(in[i]=='Г')
		{
		output[ii++]='^';
		output[ii++]='G';
		i++;
		}
	else if(in[i]=='Д')
		{
		output[ii++]='^';
		output[ii++]='D';
		i++;
		}
	else if(in[i]=='Е')
		{
		output[ii++]='^';
		output[ii++]='E';
		i++;
		}
	else if(in[i]=='Ё')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='E';
		i++;
		}
	else if(in[i]=='Ж')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='C';
		i++;
		}
	else if(in[i]=='З')
		{
		output[ii++]='^';
		output[ii++]='Z';
		i++;
		}
	else if(in[i]=='И')
		{
		output[ii++]='^';
		output[ii++]='I';
		i++;
		}
	else if(in[i]=='Й')
		{
		output[ii++]='^';
		output[ii++]='J';
		i++;
		}
	else if(in[i]=='К')
		{
		output[ii++]='^';
		output[ii++]='K';
		i++;
		}
	else if(in[i]=='Л')
		{
		output[ii++]='^';
		output[ii++]='L';
		i++;
		}
	else if(in[i]=='М')
		{
		output[ii++]='^';
		output[ii++]='M';
		i++;
		}
	else if(in[i]=='Н')
		{
		output[ii++]='^';
		output[ii++]='N';
		i++;
		}
	else if(in[i]=='О')
		{
		output[ii++]='^';
		output[ii++]='O';
		i++;
		}
	else if(in[i]=='П')
		{
		output[ii++]='^';
		output[ii++]='P';
		i++;
		}
	else if(in[i]=='Р')
		{
		output[ii++]='^';
		output[ii++]='R';
		i++;
		}
	else if(in[i]=='С')
		{
		output[ii++]='^';
		output[ii++]='S';
		i++;
		}
	else if(in[i]=='Т')
		{
		output[ii++]='^';
		output[ii++]='T';
		i++;
		}
	else if(in[i]=='У')
		{
		output[ii++]='^';
		output[ii++]='U';
		i++;
		}
	else if(in[i]=='Ф')
		{
		output[ii++]='^';
		output[ii++]='F';
		i++;
		}
	else if(in[i]=='Х')
		{
		output[ii++]='^';
		output[ii++]='H';
		i++;
		}
	else if(in[i]=='Ц')
		{
		output[ii++]='^';
		output[ii++]='C';
		i++;
		}
	else if(in[i]=='Ч')
		{
		output[ii++]='^';
		output[ii++]='Y';
		i++;
		}
	else if(in[i]=='Ш')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='A';
		i++;
		}
	else if(in[i]=='Щ')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='D';
		i++;
		}
	else if(in[i]=='Ъ')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='B';
		i++;
		}
	else if(in[i]=='Ы')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='F';
		i++;
		}
	else if(in[i]=='Ь')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='G';
		i++;
		}
	else if(in[i]=='Э')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='H';
		i++;
		}
	else if(in[i]=='Ю')
		{
		output[ii++]='^';
		output[ii++]='V';
		i++;
		}
	else if(in[i]=='Я')
		{
		output[ii++]='^';
		output[ii++]='Q';
		i++;
		}
	else if(in[i]=='а')
		{
		output[ii++]='^';
		output[ii++]='a';
		i++;
		}
	else if(in[i]=='б')
		{
		output[ii++]='^';
		output[ii++]='b';
		i++;
		}
	else if(in[i]=='в')
		{
		output[ii++]='^';
		output[ii++]='w';
		i++;
		}
	else if(in[i]=='г')
		{
		output[ii++]='^';
		output[ii++]='g';
		i++;
		}
	else if(in[i]=='д')
		{
		output[ii++]='^';
		output[ii++]='d';
		i++;
		}
	else if(in[i]=='е')
		{
		output[ii++]='^';
		output[ii++]='e';
		i++;
		}
	else if(in[i]=='ё')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='e';
		i++;
		}
	else if(in[i]=='ж')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='c';
		i++;
		}
	else if(in[i]=='з')
		{
		output[ii++]='^';
		output[ii++]='z';
		i++;
		}
	else if(in[i]=='и')
		{
		output[ii++]='^';
		output[ii++]='i';
		i++;
		}
	else if(in[i]=='й')
		{
		output[ii++]='^';
		output[ii++]='j';
		i++;
		}
	else if(in[i]=='к')
		{
		output[ii++]='^';
		output[ii++]='k';
		i++;
		}
	else if(in[i]=='л')
		{
		output[ii++]='^';
		output[ii++]='l';
		i++;
		}
	else if(in[i]=='м')
		{
		output[ii++]='^';
		output[ii++]='m';
		i++;
		}
	else if(in[i]=='н')
		{
		output[ii++]='^';
		output[ii++]='n';
		i++;
		}
	else if(in[i]=='о')
		{
		output[ii++]='^';
		output[ii++]='o';
		i++;
		}
	else if(in[i]=='п')
		{
		output[ii++]='^';
		output[ii++]='p';
		i++;
		}
	else if(in[i]=='р')
		{
		output[ii++]='^';
		output[ii++]='r';
		i++;
		}
	else if(in[i]=='с')
		{
		output[ii++]='^';
		output[ii++]='s';
		i++;
		}
	else if(in[i]=='т')
		{
		output[ii++]='^';
		output[ii++]='t';
		i++;
		}
	else if(in[i]=='у')
		{
		output[ii++]='^';
		output[ii++]='u';
		i++;
		}
	else if(in[i]=='ф')
		{
		output[ii++]='^';
		output[ii++]='f';
		i++;
		}
	else if(in[i]=='х')
		{
		output[ii++]='^';
		output[ii++]='h';
		i++;
		}
	else if(in[i]=='ц')
		{
		output[ii++]='^';
		output[ii++]='c';
		i++;
		}
	else if(in[i]=='ч')
		{
		output[ii++]='^';
		output[ii++]='y';
		i++;
		}
	else if(in[i]=='ш')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='a';
		i++;
		}
	else if(in[i]=='щ')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='d';
		i++;
		}
	else if(in[i]=='ъ')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='b';
		i++;
		}
	else if(in[i]=='ы')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='f';
		i++;
		}
	else if(in[i]=='ь')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='g';
		i++;
		}
	else if(in[i]=='э')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='h';
		i++;
		}
	else if(in[i]=='ю')
		{
		output[ii++]='^';
		output[ii++]='v';
		i++;
		}
	else if(in[i]=='я')
		{
		output[ii++]='^';
		output[ii++]='q';
		i++;
		}
	else if(in[i]=='°')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='i';
		i++;
		}
	else if(in[i]=='№')
		{
		output[ii++]='^';
		output[ii++]='X';
		output[ii++]='j';
		i++;
		}
	else
		{
		output[ii++]=in[i++];
		}
	}

/*while(in[i])
	{
	output[ii++]=in[i++];
	}*/

output[ii++]=0;	
/*
for(i=0;i<4;i++)
	{
	output[ii++]=in[i++];
	}  

output[0]='0';
output[1]='1';
output[2]='2';
output[3]='3';
output[4]='4';
output[5]='5';
output[6]='6';
output[7]='7';
output[8]='8';
output[9]='9';
output[10]='a';
output[11]='b';
output[12]='c';
output[13]='d';
output[14]='e';
output[15]='f';
output[16]='g';
output[17]='h';
output[18]='i';
output[19]='j';
output[20]=0;  */

return output;
}

//-----------------------------------------------
void http_data(void)
{
http_power_num_of_phases=NUMPHASE;
if((http_power_num_of_phases!=1)&&(http_power_num_of_phases!=3)) http_power_num_of_phases=0;
if(http_power_num_of_phases==1)
	{
	http_power_voltage_of_phase[0]=net_U;
	}
else http_power_voltage_of_phase[0]=net_U;
http_power_voltage_of_phase[1]=net_Ub;
http_power_voltage_of_phase[2]=net_Uc;
http_power_frequncy = net_F;
http_power_status=0;
//if(avar_stat&0x0001)http_power_status=1;
http_power_status=net_av;
};

//-----------------------------------------------
short http_get_log_deep(void)
{
return lc640_read_int(CNT_EVENT_LOG);
};

//-----------------------------------------------
char* http_get_log_rec(char num)
{
char i;
unsigned int tempii;
char buff[40];

for (i=0;i<40;i++) buff[i]=0;

tempii=lc640_read_int(PTR_EVENT_LOG);
tempii=ptr_carry(tempii,64,-1*((signed)num));
tempii*=32;
tempii+=EVENT_LOG;

lc640_read_long_ptr(tempii,buff);
lc640_read_long_ptr(tempii+4,buff+4);
lc640_read_long_ptr(tempii+8,buff+8);
lc640_read_long_ptr(tempii+12,buff+12);
lc640_read_long_ptr(tempii+16,buff+16);
lc640_read_long_ptr(tempii+20,buff+20);
lc640_read_long_ptr(tempii+24,buff+24);
lc640_read_long_ptr(tempii+28,buff+28);

for (i=0;i<32;i++)
	{
	http_output_buff[i*2]=hex_alfa[buff[i]/16];
	http_output_buff[(i*2)+1]=hex_alfa[buff[i]%16];
	}
//http_output_buff[6]='0';
//http_output_buff[7]='1';
http_output_buff[64]=0;

return http_output_buff;
}

//-----------------------------------------------
char* http_tm_dt_output(char numOfDt)
{
char buffer[100];

sprintf(buffer,"%d %d", t_ext[numOfDt], ND_EXT[numOfDt]);

return buffer;
}

//-----------------------------------------------
char* http_tm_sk_output(char numOfSk)
{
char buffer[100];
char temp1=0;
char temp2=0;

if(sk_stat[numOfSk]==ssON)temp1=1;
if(sk_av_stat[numOfSk]==sasON)temp2=1;

sprintf(buffer,"%d %d", temp1, temp2);

return buffer;
}

//-----------------------------------------------
char http_bps_status2number(char number)
{
/*//return number+spirit_wrk_cnt;
if((bps[number]._state==bsWRK)&&(!bps[number]._flags_tm)) 		return 1;
if((bps[number]._state==bsRDY)) 								return 2;
if((bps[number]._state==bsWRK)&&(bps[number]._flags_tm&0x08)) 	return 3;
if((bps[number]._state==bsBL)) 									return 4;
if((bps[number]._state==bsAPV)) 								return 5;
if((bps[number]._av&(1<<0))) 									return 6;
if((bps[number]._av&(1<<2))) 									return 7;
if((bps[number]._av&(1<<1))) 									return 8;
if((bps[number]._av&(1<<3))) 									return 9;
if((bps[number]._state==bsOFF_AV_NET)) 							return 10; */
}

//-----------------------------------------------
char* http_tm_inv_output(char numOfInv)
{
char buffer[300];
char* inv_stat=pal_cyr_coder("                        ");

/*
if((inv[numOfInv]._flags_tm==0)&&(inv[numOfInv]._cnt==0))
	{
	inv_stat=pal_cyr_coder("в работе");
	}
else if((inv[numOfInv]._flags_tm==0x04)&&(inv[numOfInv]._cnt==0))
	{
	inv_stat=pal_cyr_coder("СИЛЬНЫЙ НАГРЕВ!!!");	      
	}
else if((inv[numOfInv]._flags_tm==0x24)&&(inv[numOfInv]._cnt==0))
	{
	inv_stat=pal_cyr_coder("отключился,перегрев");	      
	}
else if((inv[numOfInv]._flags_tm&0x20)&&(inv[numOfInv]._cnt==0))
	{
	inv_stat=pal_cyr_coder("отсутствует Uвых");
	}
else if(inv[numOfInv]._cnt!=0)
 	{
	inv_stat=pal_cyr_coder("не подключен");	
	}
*/

	if(inv[numOfInv]._cnt>5)
	 	{
		inv_stat=pal_cyr_coder("не подключен");	
		}
	else if(inv[numOfInv]._inv_int_err_cnt>149)
		{
		inv_stat=pal_cyr_coder("ОТКЛ. внутр. неиспр.");
		}
	else if((inv[numOfInv]._flags_tm&0x01)==0x01)
		{
		inv_stat=pal_cyr_coder("ПЕРЕГРУЖЕН!!!");	      
		}
	else if((inv[numOfInv]._flags_tm_dop&0x01)==0x01)
		{
		inv_stat=pal_cyr_coder("ОТКЛ. Udc не в норме");	      
		}

	else if(((inv[numOfInv]._flags_tm&0x22)==0x02) || ((inv[numOfInv]._flags_tm&0x24)==0x04))
		{
		inv_stat=pal_cyr_coder("ПЕРЕГРЕВ!!ВЫКЛЮЧЕН!!");	      
		}
	else if((inv[numOfInv]._flags_tm&0x28)==0x28)
		{
		inv_stat=pal_cyr_coder("В РАБОТЕ.ЗАВЫШ Uвых!");	      
		}
	else if((inv[numOfInv]._flags_tm&0x30)==0x30)
		{
		inv_stat=pal_cyr_coder("В РАБОТЕ.ЗАНИЖ Uвых!");	      
		}
	else if((inv[numOfInv]._flags_tm&0x24)==0x24)
		{
		inv_stat=pal_cyr_coder("  СИЛЬНЫЙ НАГРЕВ!!! ");	      
		}
	else if((inv[numOfInv]._flags_tm&0xa0)==0x20)
		{
		inv_stat=pal_cyr_coder("В РАБОТЕ.ОТ БАТАРЕИ ");	      
		}
	else if((inv[numOfInv]._flags_tm&0xa0)==0xa0)
		{
		inv_stat=pal_cyr_coder("В РАБОТЕ.ОТ СЕТИ    ");	      
		}


//inv_stat=pal_cyr_coder("не подключен");

/*inv[numOfInv]._Uout=(numOfInv+1)*10 +1;
inv[numOfInv]._Iout=(numOfInv+1)*10 +2;  
inv[numOfInv]._T=(numOfInv+1)*10 +3; */
//inv[numOfInv]._Pout=(numOfInv+1)*100 +4;
/*inv[numOfInv]._Uacin=(numOfInv+1)*10 +5;
inv[numOfInv]._Uload=(numOfInv+1)*10 +6;
inv[numOfInv]._Udcin=(numOfInv+1)*10 +7;*/

sprintf(buffer,"%d, %d, %d, %d, %d, %d, %d, %d, %s, 0x%02x", numOfInv, 
									inv[numOfInv]._Uout,
									inv[numOfInv]._Iout,
									inv[numOfInv]._Pout,
									inv[numOfInv]._T,
									inv[numOfInv]._Uload,
									inv[numOfInv]._Uacin,
									inv[numOfInv]._Udcin,
									inv_stat,
									inv[numOfInv]._flags_tm);

return buffer;
}

//-----------------------------------------------
char* http_tm_bypass_output(char numOfByps)
{
char buffer[300];
char* byps_stat=pal_cyr_coder("");

if(byps[numOfByps]._flags&0x40)
	{
	if(byps[numOfByps]._flags&0x80) byps_stat =	pal_cyr_coder("Приоритет инверторы, работа от инверторов");
	else  							byps_stat =	pal_cyr_coder("Приоритет инверторы, работа от сети");
	}
else
	{
	if(byps[numOfByps]._flags&0x80) byps_stat =	pal_cyr_coder("Приоритет сеть, работа от инверторов");
	else  							byps_stat =	pal_cyr_coder("Приоритет сеть, работа от сети");
	}
/*	if(iByps_ind_cnt<=20)
		{
		if(byps[0]._flags&0x40)ptr[0]=				"Приоритет инверторы ";
		else ptr[0]=								"Приоритет сеть      ";
		}

	if(iByps_ind_cnt>20)
		{
		if(byps[0]._flags&0x80)ptr[0]=				"Работа от инверторов";
		else ptr[0]=								"Работа от сети      ";
		} */

/*	if((byps[0]._flags&0x04)&&(byps[0]._cnt<5))
		{
		ptr[0]=		"  СИЛЬНЫЙ НАГРЕВ!!! ";	      
		}
	else if((byps[0]._flags&0x02)&&(byps[0]._cnt<5))
		{
		ptr[0]=		"отключился,перегрев ";	      
		}*/
	if((byps[numOfByps]._flags&0x04)&&(byps[numOfByps]._cnt<5) && (byps[numOfByps]._Uout<10) && bFL)
		{
		byps_stat =	pal_cyr_coder(" ПЕРЕГРЕВ!!!ВЫКЛ!!! ");	      
		}
	else if((byps[numOfByps]._flags&0x04)&&(byps[numOfByps]._cnt<5) && (byps[numOfByps]._Uout>10) && bFL)
		{
		byps_stat =	pal_cyr_coder("  СИЛЬНЫЙ НАГРЕВ!!! ");	      
		}
 	if((byps[numOfByps]._cnt>10))
	 	{
		byps_stat =	pal_cyr_coder("не подключен");	
		}
/*
byps[numOfByps]._Uout=(numOfByps+1)*10 +1;
byps[numOfByps]._Iout=(numOfByps+1)*10 +2;  
byps[numOfByps]._Pout=(numOfByps+1)*10 +3; 
byps[numOfByps]._T=(numOfByps+1)*10 +4;
byps[numOfByps]._UinACprim=(numOfByps+1)*10 +5;
byps[numOfByps]._UinACinvbus=(numOfByps+1)*10 +6;
//inv[numOfInv]._Udcin=(numOfByps+1)*10 +7;	*/

sprintf(buffer,"%d: %d: %d: %d: %d: %d: %d: %s", numOfByps, 
									byps[numOfByps]._Uout,
									byps[numOfByps]._Iout,
									byps[numOfByps]._Pout,
									byps[numOfByps]._T,
									byps[numOfByps]._UinACprim,
									byps[numOfByps]._UinACinvbus,
									byps_stat);

return buffer;
}

//-----------------------------------------------
char* http_tm_load_output(char numOfLoad)
{
char buffer[300];
char* byps_stat=pal_cyr_coder("                        ");

signed short temp_load_U,temp_load_I,temp_load_P,temp_load_F;

/*
if((inv[numOfInv]._flags_tm==0)&&(inv[numOfInv]._cnt==0))
	{
	inv_stat=pal_cyr_coder("в работе");
	}
else if((inv[numOfInv]._flags_tm==0x04)&&(inv[numOfInv]._cnt==0))
	{
	inv_stat=pal_cyr_coder("СИЛЬНЫЙ НАГРЕВ!!!");	      
	}
else if((inv[numOfInv]._flags_tm==0x24)&&(inv[numOfInv]._cnt==0))
	{
	inv_stat=pal_cyr_coder("отключился,перегрев");	      
	}
else if((inv[numOfInv]._flags_tm&0x20)&&(inv[numOfInv]._cnt==0))
	{
	inv_stat=pal_cyr_coder("отсутствует Uвых");
	}
else if(inv[numOfInv]._cnt!=0)
 	{
	inv_stat=pal_cyr_coder("не подключен");	
	}
*/
/*
	if(inv[numOfInv]._cnt>5)
	 	{
		inv_stat=pal_cyr_coder("не подключен");	
		}
	else if(inv[numOfInv]._inv_int_err_cnt>149)
		{
		inv_stat=pal_cyr_coder("ОТКЛ. внутр. неиспр.");
		}
	else if((inv[numOfInv]._flags_tm&0x01)==0x01)
		{
		inv_stat=pal_cyr_coder("ПЕРЕГРУЖЕН!!!");	      
		}
	else if((inv[numOfInv]._flags_tm_dop&0x01)==0x01)
		{
		inv_stat=pal_cyr_coder("ОТКЛ. Udc не в норме");	      
		}

	else if(((inv[numOfInv]._flags_tm&0x22)==0x02) || ((inv[numOfInv]._flags_tm&0x24)==0x04))
		{
		inv_stat=pal_cyr_coder("ПЕРЕГРЕВ!!ВЫКЛЮЧЕН!!");	      
		}
	else if((inv[numOfInv]._flags_tm&0x28)==0x28)
		{
		inv_stat=pal_cyr_coder("В РАБОТЕ.ЗАВЫШ Uвых!");	      
		}
	else if((inv[numOfInv]._flags_tm&0x30)==0x30)
		{
		inv_stat=pal_cyr_coder("В РАБОТЕ.ЗАНИЖ Uвых!");	      
		}
	else if((inv[numOfInv]._flags_tm&0x24)==0x24)
		{
		inv_stat=pal_cyr_coder("  СИЛЬНЫЙ НАГРЕВ!!! ");	      
		}
	else if((inv[numOfInv]._flags_tm&0xa0)==0x20)
		{
		inv_stat=pal_cyr_coder("В РАБОТЕ.ОТ БАТАРЕИ ");	      
		}
	else if((inv[numOfInv]._flags_tm&0xa0)==0xa0)
		{
		inv_stat=pal_cyr_coder("В РАБОТЕ.ОТ СЕТИ    ");	      
		}	*/


if(NUMBYPASS==10)
	{
/*	if((inv[0]._flags_tm&0x80)==0x80)
		{
		int2lcd(inv[0]._Uacin/10,'[',0);	      
		}
	else 
		{
		int2lcd(inv[0]._Uload/10,'[',0);	      
		}			



		//if(byps[0]._Iout>99)int2lcd(byps[0]._Iout/10,'}',0);
	    //else int2lcd(byps[0]._Iout,'}',1); 
		//if(byps[1]._Iout>99)int2lcd(byps[1]._Iout/10,'}',0);
	    //else int2lcd(byps[1]._Iout,'}',1); 
		//if(byps[2]._Iout>99)int2lcd(byps[2]._Iout/10,'}',0);
	   // else int2lcd(byps[2]._Iout,'}',1); 
	int2lcd(inv[0]._Iout,']',1);  
	//int2lcd_mmm(inv[sub_ind1]._T,'[',0); 
	//int2lcd_mmm(inv[sub_ind1]._Pout,']',0);

	int2lcd_mmm(inv[0]._Pout,'@',0);*/

	}
else if((NUMPHASE==3)||(NUMPHASE==2))
	{
	if(NUMBYPASS)
		{
		temp_load_U = byps[numOfLoad]._Uout;
		temp_load_I = byps[numOfLoad]._Iout;
		}
	else 
		{
		temp_load_U = load_U_inv_3F[numOfLoad];
		temp_load_I = load_I_inv_3F[numOfLoad];
		}

	if(byps[0]._Pout>65000)byps[0]._Pout=0; 
	if(byps[1]._Pout>65000)byps[1]._Pout=0;
	if(byps[2]._Pout>65000)byps[2]._Pout=0;

	if(NUMBYPASS)
		{
		temp_load_P = (short)(byps[numOfLoad]._Pout);
		}
	else 
		{
		temp_load_P = (short)(load_P_inv_3F[numOfLoad]);
		}
	}
else 
	{
	temp_load_U=load_U;
	temp_load_I=load_I;
	temp_load_P=load_P;
	}
/*temp_load_U=(numOfLoad+1)*10 +1;
temp_load_I=(numOfLoad+1)*10 +2;  
temp_load_P=(numOfLoad+1)*10 +3; */
temp_load_F=f_out; 

sprintf(buffer,"%d, %d, %d, %d, %d", numOfLoad, 
									temp_load_U,
									temp_load_I,
									temp_load_P,
									temp_load_F);

return buffer;
}



 //-----------------------------------------------
char* http_tm_status_output(void)
{
char buffer[300];
char bbb[50];
//const char* ptr;
//const char* ptrs[80];
const char* sub_ptrs[40];
static char sub_cnt,sub_cnt1;
char i,sub_cnt_max;
char ii_;				  
static char ii_cnt,cnt_ind_bat;
	 

sprintf(bbb,"В работе  %d инверторов ",	num_of_wrks_inv); 
sub_ptrs[i++]=bbb;  
sub_cnt_max=1;
i=1;

if(byps[0]._valid)
	{
	if(language)
		{
		if(byps[0]._flags&0x40)sub_ptrs[i++]=			"Приоритет инверторы ";
		else sub_ptrs[i++]=								"Приоритет сеть      ";
		}
	else 
		{
		if(byps[0]._flags&0x40)sub_ptrs[i++]=			"Priority: invertors ";
		else sub_ptrs[i++]=								"Priority: mains     ";
		}
	sub_cnt_max++;	
	if(language)
		{
		if(byps[0]._flags&0x80)sub_ptrs[i++]=			"Работа от инверторов";
		else sub_ptrs[i++]=								"Работа от сети      ";
		}
	else 
		{
		if(byps[0]._flags&0x80)sub_ptrs[i++]=			"In operate: invertor";
		else sub_ptrs[i++]=								"In operate: mains   ";
		}
	sub_cnt_max++;
	if(language)
		{
		if(B4_4)
			{sub_ptrs[i++]=							"Ручн. упр-ние: сеть ";
		//else sub_ptrs[i++]=								"Работа от сети      ";	
			sub_cnt_max++;
			}
		}
	else 
		{
		if(B4_4)
			{
			sub_ptrs[i++]=							"Hand control: mains";
		//else sub_ptrs[i++]=								"In operate: mains   ";
			sub_cnt_max++;
			}
		}

	if(language)
		{
		if(B5_)
			{sub_ptrs[i++]=							"Ручн. упр-ние: инв. ";
		//else sub_ptrs[i++]=								"Работа от сети      ";	
			sub_cnt_max++;
			}
		}
	else 
		{
		if(B5_)
			{
			sub_ptrs[i++]=							" Hand control: inv  ";
		//else sub_ptrs[i++]=								"In operate: mains   ";
			sub_cnt_max++;
			}
		}	
	}


/*
	if(iByps_ind_cnt<=50)
		{
		if(byps[sub_ind1]._flags&0x40)ptr[0]=		"Приоритет инверторы ";
		else ptr[0]=								"Приоритет сеть      ";
		}

	if(iByps_ind_cnt>50)
		{
		if(byps[sub_ind1]._flags&0x80)ptr[0]=		"Работа от инверторов";
		else ptr[0]=								"Работа от сети      ";
		}

	if(iByps_ind_cnt<=50)
		{
		if(byps[sub_ind1]._flags&0x40)ptr[0]=		"Priority: invertors ";
		else ptr[0]=								"Priority: mains     ";
		}

	if(iByps_ind_cnt>50)
		{
		if(byps[sub_ind1]._flags&0x80)ptr[0]=		"In operate: invertor";
		else ptr[0]=								"In operate: mains   ";
		} */
	      
	
if(avar_stat&0x0001)
	{
	if(language)sub_ptrs[i++]=		"   Авария сети!!!   ";//o_2
	else 		sub_ptrs[i++]=		"   Alarm mains!!!   ";//o_2
	sub_cnt_max++;	
	}



if(ips_bat_av_stat)
	{
	if(language)sub_ptrs[i++]=	" Авария батареи №1  ";//o_2
	else		sub_ptrs[i++]=	"  Alarm battery №1  ";//o_2
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+0)))
	{
	sub_ptrs[i++]=	"   Авария БПС №1    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+1)))
	{
	sub_ptrs[i++]=	"   Авария БПС №2    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+2)))
	{
	sub_ptrs[i++]=	"   Авария БПС №3    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+3)))
	{
	sub_ptrs[i++]=	"   Авария БПС №4    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+4)))
	{
	sub_ptrs[i++]=	"   Авария БПС №5    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+5)))
	{
	sub_ptrs[i++]=	"   Авария БПС №6    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+6)))
	{
	sub_ptrs[i++]=	"   Авария БПС №7    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+7)))
	{
	sub_ptrs[i++]=	"   Авария БПС №8    ";
	sub_cnt_max++;	
	}
/*
if((avar_stat&(1<<(25)))&&(SK_LCD_EN[0]))
	{
	sub_ptrs[i++]=	" Открыта дверь!!    ";
	sub_cnt_max++;	
	}

if((avar_stat&(1<<(26)))&&(SK_LCD_EN[1]))
	{
	sub_ptrs[i++]=	"Сработал датч. дыма ";
	sub_cnt_max++;	
	}

if((avar_stat&(1<<(27)))&&(SK_LCD_EN[2]))
	{
	sub_ptrs[i++]=	"Сработал датч. удара";
	sub_cnt_max++;	
	}
*/
//#ifdef UKU_GLONASS
if((sk_av_stat[0]==sasON)&&(NUMSK)&&(!SK_LCD_EN[0]))
	{
	if(language) sub_ptrs[i++]=	"   Сработал СК№1    ";//o_2
	else 		sub_ptrs[i++]=	" Active state DC №1 ";//o_2
	sub_cnt_max++;	
	}
if((sk_av_stat[1]==sasON)&&(NUMSK>1)&&(!SK_LCD_EN[1]))
	{
	if(language) sub_ptrs[i++]=	"   Сработал СК№2    ";//o_2
	else 		sub_ptrs[i++]=	" Active state DC №2 ";//o_2
	sub_cnt_max++;	
	}
if((sk_av_stat[2]==sasON)&&(NUMSK>2)&&(!SK_LCD_EN[2]))
	{
	if(language) sub_ptrs[i++]=	"   Сработал СК№3    ";//o_2
	else 		sub_ptrs[i++]=	" Active state DC №3 ";//o_2
	sub_cnt_max++;	
	}
if((sk_av_stat[3]==sasON)&&(NUMSK>3)&&(!SK_LCD_EN[3]))
	{
	if(language) sub_ptrs[i++]=	"   Сработал СК№4    ";//o_2
	else 		sub_ptrs[i++]=	" Active state DC №4 ";//o_2
	sub_cnt_max++;	
	}
//#endif
/*if((avar_stat&(1<<(28)))&&(SK_LCD_EN[3]))
	{
	sub_ptrs[i++]=	"   Авария СК №4     ";
	sub_cnt_max++;	
	} */

if(byps[0]._uin_av_stat || byps[1]._uin_av_stat || byps[2]._uin_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Авария по Uвх(инв)!!"; //o_2
	else 			sub_ptrs[i++]=	"  Alarm Uin(inv)!!  "; //o_2
	sub_cnt_max++;	
	}

if(uOutAvIsOn)
	{
	if(language) 	sub_ptrs[i++]=	"  Авария по Uвых!!! "; //o_2
	else 			sub_ptrs[i++]=	"    Alarm Uout!!!   "; //o_2
	sub_cnt_max++;	
	}
if(uNetAvIsOn)
	{
	if(language) 	sub_ptrs[i++]=	"Авария по Uвх(AC)!!!"; //o_2
	else 			sub_ptrs[i++]=	"  Alarm Uin(AC)!!!  "; //o_2
	sub_cnt_max++;	
	}
if(dcAvIsOn)
	{
	if(language) 	sub_ptrs[i++]=	"Авария по Uвх(DC)!!!"; //o_2
	else 			sub_ptrs[i++]=	"  Alarm Uin(DC)!!!  "; //o_2
	sub_cnt_max++;	
	}

if(fBypsInAvIsOn)
	{
	if(language) 	sub_ptrs[i++]=	"Авария по Фвх(AC)!!!"; //o_2
	else 			sub_ptrs[i++]=	"  Alarm Фin(AC)!!!  "; //o_2
	sub_cnt_max++;	
	}

if(fBypsInvAvIsOn)
	{
	if(language) 	sub_ptrs[i++]=	"Авария по Фвх(инв)!!"; //o_2
	else 			sub_ptrs[i++]=	"  Alarm Фin(inv)!!! "; //o_2
	sub_cnt_max++;	
	}

if((!byps[0]._valid) && (NUMBYPASS) && (NUMBYPASS!=10))
	{
	if(language) 	sub_ptrs[i++]=	"Байпас разрыв связи "; //o_2
	else 			sub_ptrs[i++]=	"Bypass not connected"; //o_2
	sub_cnt_max++;	
	}

if((!byps[1]._valid) && (NUMBYPASS>1) && (NUMBYPASS!=10))
	{
	if(language) 	sub_ptrs[i++]=	"Байпас разрыв связи "; //o_2
	else 			sub_ptrs[i++]=	"Bypass not connected"; //o_2
	sub_cnt_max++;	
	}

if((!byps[2]._valid) && (NUMBYPASS>2) && (NUMBYPASS!=10))
	{
	if(language) 	sub_ptrs[i++]=	"Байпас разрыв связи "; //o_2
	else 			sub_ptrs[i++]=	"Bypass not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[0]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№1 разрыв связи "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№1 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[0]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№1 внутр.неиспр."; //o_2
	else 			sub_ptrs[i++]=	"Inv.№1 Defective.   "; //o_2
	sub_cnt_max++;
	}

if((inv[0]._flags_tm&0x01)&&(inv[0]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№1 перегрузка   "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№1 Overload.    "; //o_2
	sub_cnt_max++;	
	}

if((inv[0]._flags_tm&0x04)&&(inv[0]._valid)&&(!(inv[0]._flags_tm&0x02))&&(inv[0]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№1 сильн.нагрев "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№1  high heat   "; //o_2
	sub_cnt_max++;	
	}

if((inv[0]._valid)&&((inv[0]._flags_tm&0x02)||((inv[0]._flags_tm&0x04)&&(!(inv[0]._flags_tm&(1<<5)))&&(!(inv[0]._flags_tm&0x01))&&(!(inv[0]._flags_tm_dop&0x01)))))//if((inv[0]._flags_tm&0x06)&&/*&&(!(inv[0]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№1 перегрев,выкл"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№1  temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[0]._flags_tm&0x08)&&(inv[0]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№1 завыш. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№1  high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[0]._flags_tm&0x10)&&(inv[0]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№1 заниж. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№1  low Uout!   "; //o_2
	sub_cnt_max++;	
	}

if((inv[0]._flags_tm_dop&0x01)&&(inv[0]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№1 заниж. Udc  "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№1  low Udc    "; //o_2	
	sub_cnt_max++;	
	}

if(inv[1]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№2 разрыв связи "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№2 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[1]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№2 внутр.неиспр."; //o_2
	else 			sub_ptrs[i++]=	"Inv.№2 Defective.   "; //o_2
	sub_cnt_max++;
	}

if((inv[1]._flags_tm&0x01)&&(inv[1]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№2 перегрузка   "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№2 Overload.    "; //o_2
	sub_cnt_max++;	
	}

if((inv[1]._flags_tm&0x04)&&(inv[1]._valid)&&(!(inv[1]._flags_tm&0x02))&&(inv[1]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№2 сильн.нагрев "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№2  high heat   "; //o_2
	sub_cnt_max++;	
	}

if((inv[1]._valid)&&((inv[1]._flags_tm&0x02)||((inv[1]._flags_tm&0x04)&&(!(inv[1]._flags_tm&(1<<5)))&&(!(inv[1]._flags_tm&0x01))&&(!(inv[1]._flags_tm_dop&0x01)))))//if((inv[1]._flags_tm&0x06)&&/*&&(!(inv[1]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№2 перегрев,выкл"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№2  temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[1]._flags_tm&0x08)&&(inv[1]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№2 завыш. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№2  high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[1]._flags_tm&0x10)&&(inv[1]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№2 заниж. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№2  low Uout!   "; //o_2
	sub_cnt_max++;	
	}

if((inv[1]._flags_tm_dop&0x01)&&(inv[1]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№2 заниж. Udc  "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№2  low Udc    "; //o_2
	sub_cnt_max++;	
	}


if(inv[2]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№3 разрыв связи "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№3 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[2]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№3 внутр.неиспр."; //o_2
	else 			sub_ptrs[i++]=	"Inv.№3 Defective.   "; //o_2
	sub_cnt_max++;
	}

if((inv[2]._flags_tm&0x01)&&(inv[2]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№3 перегрузка   "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№3 Overload.    "; //o_2
	sub_cnt_max++;	
	}

if((inv[2]._flags_tm&0x04)&&(inv[2]._valid)&&(!(inv[2]._flags_tm&0x02))&&(inv[2]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№3 сильн.нагрев "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№3  high heat   "; //o_2
	sub_cnt_max++;	
	}

if((inv[2]._valid)&&((inv[2]._flags_tm&0x02)||((inv[2]._flags_tm&0x04)&&(!(inv[2]._flags_tm&(1<<5)))&&(!(inv[2]._flags_tm&0x01))&&(!(inv[2]._flags_tm_dop&0x01)))))//if((inv[2]._flags_tm&0x06)&&/*&&(!(inv[2]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№3 перегрев,выкл"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№3  temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[2]._flags_tm&0x08)&&(inv[2]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№3 завыш. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№3  high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[2]._flags_tm&0x10)&&(inv[2]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№3 заниж. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№3  low Uout!   "; //o_2
	sub_cnt_max++;	
	}

if((inv[2]._flags_tm_dop&0x01)&&(inv[2]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№3 заниж. Udc  "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№3  low Udc    "; //o_2	
	sub_cnt_max++;	
	}


if(inv[3]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№4 разрыв связи "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№4 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[3]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№4 внутр.неиспр."; //o_2
	else 			sub_ptrs[i++]=	"Inv.№4 Defective.   "; //o_2
	sub_cnt_max++;
	}

if((inv[3]._flags_tm&0x01)&&(inv[3]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№4 перегрузка   "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№4 Overload.    "; //o_2
	sub_cnt_max++;	
	}

if((inv[3]._flags_tm&0x04)&&(inv[3]._valid)&&(!(inv[3]._flags_tm&0x02))&&(inv[3]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№4 сильн.нагрев "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№4  high heat   "; //o_2
	sub_cnt_max++;	
	}

if((inv[3]._valid)&&((inv[3]._flags_tm&0x02)||((inv[3]._flags_tm&0x04)&&(!(inv[3]._flags_tm&(1<<5)))&&(!(inv[3]._flags_tm&0x01))&&(!(inv[3]._flags_tm_dop&0x01)))))//if((inv[3]._flags_tm&0x06)&&/*&&(!(inv[3]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№4 перегрев,выкл"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№4  temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[3]._flags_tm&0x08)&&(inv[3]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№4 завыш. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№4  high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[3]._flags_tm&0x10)&&(inv[3]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№4 заниж. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№4  low Uout!   "; //o_2
	sub_cnt_max++;	
	}

if((inv[3]._flags_tm_dop&0x01)&&(inv[3]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№4 заниж. Udc  "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№4  low Udc    "; //o_2	
	sub_cnt_max++;	
	}


if(inv[4]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№5 разрыв связи "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№5 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[4]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№5 внутр.неиспр."; //o_2
	else 			sub_ptrs[i++]=	"Inv.№5 Defective.   "; //o_2
	sub_cnt_max++;
	}

if((inv[4]._flags_tm&0x01)&&(inv[4]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№5 перегрузка   "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№5 Overload.    "; //o_2
	sub_cnt_max++;	
	}

if((inv[4]._flags_tm&0x04)&&(inv[4]._valid)&&(!(inv[4]._flags_tm&0x02))&&(inv[4]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№5 сильн.нагрев "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№5  high heat   "; //o_2
	sub_cnt_max++;	
	}

if((inv[4]._valid)&&((inv[4]._flags_tm&0x02)||((inv[4]._flags_tm&0x04)&&(!(inv[4]._flags_tm&(1<<5)))&&(!(inv[4]._flags_tm&0x01))&&(!(inv[4]._flags_tm_dop&0x01)))))//if((inv[4]._flags_tm&0x06)&&/*&&(!(inv[4]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№5 перегрев,выкл"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№5  temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[4]._flags_tm&0x08)&&(inv[4]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№5 завыш. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№5  high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[4]._flags_tm&0x10)&&(inv[4]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№5 заниж. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№5  low Uout!   "; //o_2
	sub_cnt_max++;	
	}

if((inv[4]._flags_tm_dop&0x01)&&(inv[4]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№5 заниж. Udc  "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№5  low Udc    "; //o_2	
	sub_cnt_max++;	
	}


if(inv[5]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№6 разрыв связи "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№6 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[5]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№6 внутр.неиспр."; //o_2
	else 			sub_ptrs[i++]=	"Inv.№6 Defective.   "; //o_2
	sub_cnt_max++;
	}

if((inv[5]._flags_tm&0x01)&&(inv[5]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№6 перегрузка   "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№6 Overload.    "; //o_2
	sub_cnt_max++;	
	}

if((inv[5]._flags_tm&0x04)&&(inv[5]._valid)&&(!(inv[5]._flags_tm&0x02))&&(inv[5]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№6 сильн.нагрев "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№6  high heat   "; //o_2
	sub_cnt_max++;	
	}

if((inv[5]._valid)&&((inv[5]._flags_tm&0x02)||((inv[5]._flags_tm&0x04)&&(!(inv[5]._flags_tm&(1<<5)))&&(!(inv[5]._flags_tm&0x01))&&(!(inv[5]._flags_tm_dop&0x01)))))//if((inv[5]._flags_tm&0x06)&&/*&&(!(inv[5]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№6 перегрев,выкл"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№6  temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[5]._flags_tm&0x08)&&(inv[5]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№6 завыш. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№6  high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[5]._flags_tm&0x10)&&(inv[5]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№6 заниж. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№6  low Uout!   "; //o_2
	sub_cnt_max++;	
	}

if((inv[5]._flags_tm_dop&0x01)&&(inv[5]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№6 заниж. Udc  "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№6  low Udc    "; //o_2	
	sub_cnt_max++;	
	}


if(inv[6]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№7 разрыв связи "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№7 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[6]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№7 внутр.неиспр."; //o_2
	else 			sub_ptrs[i++]=	"Inv.№7 Defective.   "; //o_2
	sub_cnt_max++;
	}

if((inv[6]._flags_tm&0x01)&&(inv[6]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№7 перегрузка   "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№7 Overload.    "; //o_2
	sub_cnt_max++;	
	}

if((inv[6]._flags_tm&0x04)&&(inv[6]._valid)&&(!(inv[6]._flags_tm&0x02))&&(inv[6]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№7 сильн.нагрев "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№7  high heat   "; //o_2
	sub_cnt_max++;	
	}

if((inv[6]._valid)&&((inv[6]._flags_tm&0x02)||((inv[6]._flags_tm&0x04)&&(!(inv[6]._flags_tm&(1<<5)))&&(!(inv[6]._flags_tm&0x01))&&(!(inv[6]._flags_tm_dop&0x01)))))//if((inv[6]._flags_tm&0x06)&&/*&&(!(inv[6]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№7 перегрев,выкл"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№7  temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[6]._flags_tm&0x08)&&(inv[6]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№7 завыш. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№7  high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[6]._flags_tm&0x10)&&(inv[6]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№7 заниж. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№7  low Uout!   "; //o_2
	sub_cnt_max++;	
	}

if((inv[6]._flags_tm_dop&0x01)&&(inv[6]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№7 заниж. Udc  "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№7  low Udc    "; //o_2	
	sub_cnt_max++;	
	}



if(inv[7]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№8 разрыв связи "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№8 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[7]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№8 внутр.неиспр."; //o_2
	else 			sub_ptrs[i++]=	"Inv.№8 Defective.   "; //o_2
	sub_cnt_max++;
	}

if((inv[7]._flags_tm&0x01)&&(inv[7]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№8 перегрузка   "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№8 Overload.    "; //o_2
	sub_cnt_max++;	
	}

if((inv[7]._flags_tm&0x04)&&(inv[7]._valid)&&(!(inv[7]._flags_tm&0x02))&&(inv[7]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№8 сильн.нагрев "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№8  high heat   "; //o_2
	sub_cnt_max++;	
	}

if((inv[7]._valid)&&((inv[7]._flags_tm&0x02)||((inv[7]._flags_tm&0x04)&&(!(inv[7]._flags_tm&(1<<5)))&&(!(inv[7]._flags_tm&0x01))&&(!(inv[7]._flags_tm_dop&0x01)))))//if((inv[7]._flags_tm&0x06)&&/*&&(!(inv[7]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№8 перегрев,выкл"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№8  temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[7]._flags_tm&0x08)&&(inv[7]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№8 завыш. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№8  high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[7]._flags_tm&0x10)&&(inv[7]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№8 заниж. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№8  low Uout!   "; //o_2
	sub_cnt_max++;	
	}

if((inv[7]._flags_tm_dop&0x01)&&(inv[7]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№8 заниж. Udc  "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№8  low Udc    "; //o_2	
	sub_cnt_max++;	
	}



if(inv[8]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№9 разрыв связи "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№9 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[8]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№9 внутр.неиспр."; //o_2
	else 			sub_ptrs[i++]=	"Inv.№9 Defective.   "; //o_2
	sub_cnt_max++;
	}

if((inv[8]._flags_tm&0x01)&&(inv[8]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№9 перегрузка   "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№9 Overload.    "; //o_2
	sub_cnt_max++;	
	}

if((inv[8]._flags_tm&0x04)&&(inv[8]._valid)&&(!(inv[8]._flags_tm&0x02))&&(inv[8]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№9 сильн.нагрев "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№9  high heat   "; //o_2
	sub_cnt_max++;	
	}

if((inv[8]._valid)&&((inv[8]._flags_tm&0x02)||((inv[8]._flags_tm&0x04)&&(!(inv[8]._flags_tm&(1<<5)))&&(!(inv[8]._flags_tm&0x01))&&(!(inv[8]._flags_tm_dop&0x01)))))//if((inv[8]._flags_tm&0x06)&&/*&&(!(inv[8]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№9 перегрев,выкл"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№9  temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[8]._flags_tm&0x08)&&(inv[8]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№9 завыш. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№9  high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[8]._flags_tm&0x10)&&(inv[8]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№9 заниж. Uвых! "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№9  low Uout!   "; //o_2
	sub_cnt_max++;	
	}

if((inv[8]._flags_tm_dop&0x01)&&(inv[8]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№9 заниж. Udc  "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№9  low Udc    "; //o_2	
	sub_cnt_max++;	
	}


if(inv[9]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№10 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№10 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[9]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№10 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№10 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[9]._flags_tm&0x01)&&(inv[9]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№10 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№10 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[9]._flags_tm&0x04)&&(inv[9]._valid)&&(!(inv[9]._flags_tm&0x02))&&(inv[9]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№10 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№10  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[9]._valid)&&((inv[9]._flags_tm&0x02)||((inv[9]._flags_tm&0x04)&&(!(inv[9]._flags_tm&(1<<5)))&&(!(inv[9]._flags_tm&0x01))&&(!(inv[9]._flags_tm_dop&0x01)))))//if((inv[9]._flags_tm&0x06)&&/*&&(!(inv[9]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№10 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№10 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[9]._flags_tm&0x08)&&(inv[9]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№10 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№10 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[9]._flags_tm&0x10)&&(inv[9]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№10 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№10  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[9]._flags_tm_dop&0x01)&&(inv[9]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№10 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№10  low Udc   "; //o_2	
	sub_cnt_max++;	
	}



if(inv[10]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№11 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№11 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[10]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№11 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№11 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[10]._flags_tm&0x01)&&(inv[10]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№11 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№11 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[10]._flags_tm&0x04)&&(inv[10]._valid)&&(!(inv[10]._flags_tm&0x02))&&(inv[10]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№11 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№11  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[10]._valid)&&((inv[10]._flags_tm&0x02)||((inv[10]._flags_tm&0x04)&&(!(inv[10]._flags_tm&(1<<5)))&&(!(inv[10]._flags_tm&0x01))&&(!(inv[10]._flags_tm_dop&0x01)))))//if((inv[10]._flags_tm&0x06)&&/*&&(!(inv[10]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№11 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№11 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[10]._flags_tm&0x08)&&(inv[10]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№11 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№11 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[10]._flags_tm&0x10)&&(inv[10]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№11 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№11  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[10]._flags_tm_dop&0x01)&&(inv[10]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№11 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№11  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[11]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№12 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№12 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[11]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№12 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№12 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[11]._flags_tm&0x01)&&(inv[11]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№12 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№12 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[11]._flags_tm&0x04)&&(inv[11]._valid)&&(!(inv[11]._flags_tm&0x02))&&(inv[11]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№12 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№12  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[11]._valid)&&((inv[11]._flags_tm&0x02)||((inv[11]._flags_tm&0x04)&&(!(inv[11]._flags_tm&(1<<5)))&&(!(inv[11]._flags_tm&0x01))&&(!(inv[11]._flags_tm_dop&0x01)))))//if((inv[11]._flags_tm&0x06)&&/*&&(!(inv[11]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№12 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№12 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[11]._flags_tm&0x08)&&(inv[11]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№12 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№12 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[11]._flags_tm&0x10)&&(inv[11]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№12 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№12  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[11]._flags_tm_dop&0x01)&&(inv[11]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№12 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№12  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[12]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№13 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№13 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[12]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№13 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№13 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[12]._flags_tm&0x01)&&(inv[12]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№13 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№13 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[12]._flags_tm&0x04)&&(inv[12]._valid)&&(!(inv[12]._flags_tm&0x02))&&(inv[12]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№13 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№13  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[12]._valid)&&((inv[12]._flags_tm&0x02)||((inv[12]._flags_tm&0x04)&&(!(inv[12]._flags_tm&(1<<5)))&&(!(inv[12]._flags_tm&0x01))&&(!(inv[12]._flags_tm_dop&0x01)))))//if((inv[12]._flags_tm&0x06)&&/*&&(!(inv[12]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№13 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№13 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[12]._flags_tm&0x08)&&(inv[12]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№13 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№13 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[12]._flags_tm&0x10)&&(inv[12]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№13 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№13  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[12]._flags_tm_dop&0x01)&&(inv[12]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№13 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№13  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[13]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№14 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№14 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[13]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№14 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№14 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[13]._flags_tm&0x01)&&(inv[13]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№14 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№14 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[13]._flags_tm&0x04)&&(inv[13]._valid)&&(!(inv[13]._flags_tm&0x02))&&(inv[13]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№14 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№14  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[13]._valid)&&((inv[13]._flags_tm&0x02)||((inv[13]._flags_tm&0x04)&&(!(inv[13]._flags_tm&(1<<5)))&&(!(inv[13]._flags_tm&0x01))&&(!(inv[13]._flags_tm_dop&0x01)))))//if((inv[13]._flags_tm&0x06)&&/*&&(!(inv[13]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№14 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№14 temp,disabl."; //o_2;
	sub_cnt_max++;	
	}

if((inv[13]._flags_tm&0x08)&&(inv[13]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№14 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№14 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[13]._flags_tm&0x10)&&(inv[13]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№14 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№14  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[13]._flags_tm_dop&0x01)&&(inv[13]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№14 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№14  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[14]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№15 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№15 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[14]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№15 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№15 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[14]._flags_tm&0x01)&&(inv[14]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№15 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№15 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[14]._flags_tm&0x04)&&(inv[14]._valid)&&(!(inv[14]._flags_tm&0x02))&&(inv[14]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№15 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№15  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[14]._valid)&&((inv[14]._flags_tm&0x02)||((inv[14]._flags_tm&0x04)&&(!(inv[14]._flags_tm&(1<<5)))&&(!(inv[14]._flags_tm&0x01))&&(!(inv[14]._flags_tm_dop&0x01)))))//if((inv[14]._flags_tm&0x06)&&/*&&(!(inv[14]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№15 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№15 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[14]._flags_tm&0x08)&&(inv[14]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№15 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№15 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[14]._flags_tm&0x10)&&(inv[14]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№15 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№15  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[14]._flags_tm_dop&0x01)&&(inv[14]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№15 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№15  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[15]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№16 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№16 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[15]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№16 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№16 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[15]._flags_tm&0x01)&&(inv[15]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№16 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№16 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[15]._flags_tm&0x04)&&(inv[15]._valid)&&(!(inv[15]._flags_tm&0x02))&&(inv[15]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№16 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№16  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[15]._valid)&&((inv[15]._flags_tm&0x02)||((inv[15]._flags_tm&0x04)&&(!(inv[15]._flags_tm&(1<<5)))&&(!(inv[15]._flags_tm&0x01))&&(!(inv[15]._flags_tm_dop&0x01)))))//if((inv[15]._flags_tm&0x06)&&/*&&(!(inv[15]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№16 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№16 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[15]._flags_tm&0x08)&&(inv[15]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№16 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№16 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[15]._flags_tm&0x10)&&(inv[15]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№16 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№16  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[15]._flags_tm_dop&0x01)&&(inv[15]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№16 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№16  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[16]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№17 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№17 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[16]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№17 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№17 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[16]._flags_tm&0x01)&&(inv[16]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№17 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№17 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[16]._flags_tm&0x04)&&(inv[16]._valid)&&(!(inv[16]._flags_tm&0x02))&&(inv[16]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№17 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№17  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[16]._valid)&&((inv[16]._flags_tm&0x02)||((inv[16]._flags_tm&0x04)&&(!(inv[16]._flags_tm&(1<<5)))&&(!(inv[16]._flags_tm&0x01))&&(!(inv[16]._flags_tm_dop&0x01)))))//if((inv[16]._flags_tm&0x06)&&/*&&(!(inv[16]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№17 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№17 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[16]._flags_tm&0x08)&&(inv[16]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№17 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№17 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[16]._flags_tm&0x10)&&(inv[16]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№17 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№17  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[16]._flags_tm_dop&0x01)&&(inv[16]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№17 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№17  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[17]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№18 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№18 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[17]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№18 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№18 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[17]._flags_tm&0x01)&&(inv[17]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№18 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№18 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[17]._flags_tm&0x04)&&(inv[17]._valid)&&(!(inv[17]._flags_tm&0x02))&&(inv[17]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№18 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№18  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[17]._valid)&&((inv[17]._flags_tm&0x02)||((inv[17]._flags_tm&0x04)&&(!(inv[17]._flags_tm&(1<<5)))&&(!(inv[17]._flags_tm&0x01))&&(!(inv[17]._flags_tm_dop&0x01)))))//if((inv[17]._flags_tm&0x06)&&/*&&(!(inv[17]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№18 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№18 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[17]._flags_tm&0x08)&&(inv[17]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№18 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№18 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[17]._flags_tm&0x10)&&(inv[17]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№18 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№18  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[17]._flags_tm_dop&0x01)&&(inv[17]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№18 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№18  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[18]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№19 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№19 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[18]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№19 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№19 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[18]._flags_tm&0x01)&&(inv[18]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№19 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№19 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[18]._flags_tm&0x04)&&(inv[18]._valid)&&(!(inv[18]._flags_tm&0x02))&&(inv[18]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№19 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№19  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[18]._valid)&&((inv[18]._flags_tm&0x02)||((inv[18]._flags_tm&0x04)&&(!(inv[18]._flags_tm&(1<<5)))&&(!(inv[18]._flags_tm&0x01))&&(!(inv[18]._flags_tm_dop&0x01)))))//if((inv[18]._flags_tm&0x06)&&/*&&(!(inv[18]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№19 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№19 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[18]._flags_tm&0x08)&&(inv[18]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№19 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№19 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[18]._flags_tm&0x10)&&(inv[18]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№19 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№19  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[18]._flags_tm_dop&0x01)&&(inv[18]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№19 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№19  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[19]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№20 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№20 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[19]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№20 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№20 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[19]._flags_tm&0x01)&&(inv[19]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№20 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№20 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[19]._flags_tm&0x04)&&(inv[19]._valid)&&(!(inv[19]._flags_tm&0x02))&&(inv[19]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№20 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№20  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[19]._valid)&&((inv[19]._flags_tm&0x02)||((inv[19]._flags_tm&0x04)&&(!(inv[19]._flags_tm&(1<<5)))&&(!(inv[19]._flags_tm&0x01))&&(!(inv[19]._flags_tm_dop&0x01)))))//if((inv[19]._flags_tm&0x06)&&/*&&(!(inv[19]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№20 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№20 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[19]._flags_tm&0x08)&&(inv[19]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№20 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№20 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[19]._flags_tm&0x10)&&(inv[19]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№20 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№20  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[19]._flags_tm_dop&0x01)&&(inv[19]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№20 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№20  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[20]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№21 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№21 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[20]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№21 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№21 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[20]._flags_tm&0x01)&&(inv[20]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№21 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№21 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[20]._flags_tm&0x04)&&(inv[20]._valid)&&(!(inv[20]._flags_tm&0x02))&&(inv[20]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№21 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№21  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[20]._valid)&&((inv[20]._flags_tm&0x02)||((inv[20]._flags_tm&0x04)&&(!(inv[20]._flags_tm&(1<<5)))&&(!(inv[20]._flags_tm&0x01))&&(!(inv[20]._flags_tm_dop&0x01)))))//if((inv[20]._flags_tm&0x06)&&/*&&(!(inv[20]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№21 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№21 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[20]._flags_tm&0x08)&&(inv[20]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№21 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№21 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[20]._flags_tm&0x10)&&(inv[20]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№21 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№21  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[20]._flags_tm_dop&0x01)&&(inv[20]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№21 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№21  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[21]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№22 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№22 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[21]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№22 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№22 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[21]._flags_tm&0x01)&&(inv[21]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№22 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№22 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[21]._flags_tm&0x04)&&(inv[21]._valid)&&(!(inv[21]._flags_tm&0x02))&&(inv[21]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№22 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№22  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[21]._valid)&&((inv[21]._flags_tm&0x02)||((inv[21]._flags_tm&0x04)&&(!(inv[21]._flags_tm&(1<<5)))&&(!(inv[21]._flags_tm&0x01))&&(!(inv[21]._flags_tm_dop&0x01)))))//if((inv[21]._flags_tm&0x06)&&/*&&(!(inv[21]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№22 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№22 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[21]._flags_tm&0x08)&&(inv[21]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№22 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№22 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[21]._flags_tm&0x10)&&(inv[21]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№22 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№22  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[21]._flags_tm_dop&0x01)&&(inv[21]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№22 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№22  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[22]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№23 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№23 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[22]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№23 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№23 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[22]._flags_tm&0x01)&&(inv[22]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№23 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№23 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[22]._flags_tm&0x04)&&(inv[22]._valid)&&(!(inv[22]._flags_tm&0x02))&&(inv[22]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№23 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№23  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[22]._valid)&&((inv[22]._flags_tm&0x02)||((inv[22]._flags_tm&0x04)&&(!(inv[22]._flags_tm&(1<<5)))&&(!(inv[22]._flags_tm&0x01))&&(!(inv[22]._flags_tm_dop&0x01)))))//if((inv[22]._flags_tm&0x06)&&/*&&(!(inv[22]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№23 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№23 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[22]._flags_tm&0x08)&&(inv[22]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№23 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№23 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[22]._flags_tm&0x10)&&(inv[22]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№23 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№23  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[22]._flags_tm_dop&0x01)&&(inv[22]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№23 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№23  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[23]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№24 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№24 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[23]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№24 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№24 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[23]._flags_tm&0x01)&&(inv[23]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№24 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№24 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[23]._flags_tm&0x04)&&(inv[23]._valid)&&(!(inv[23]._flags_tm&0x02))&&(inv[23]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№24 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№24  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[23]._valid)&&((inv[23]._flags_tm&0x02)||((inv[23]._flags_tm&0x04)&&(!(inv[23]._flags_tm&(1<<5)))&&(!(inv[23]._flags_tm&0x01))&&(!(inv[23]._flags_tm_dop&0x01)))))//if((inv[23]._flags_tm&0x06)&&/*&&(!(inv[23]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№24 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№24 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[23]._flags_tm&0x08)&&(inv[23]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№24 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№24 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[23]._flags_tm&0x10)&&(inv[23]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№24 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№24  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[23]._flags_tm_dop&0x01)&&(inv[23]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№24 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№24  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[24]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№25 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№25 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[24]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№25 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№25 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[24]._flags_tm&0x01)&&(inv[24]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№25 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№25 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[24]._flags_tm&0x04)&&(inv[24]._valid)&&(!(inv[24]._flags_tm&0x02))&&(inv[24]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№25 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№25  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[24]._valid)&&((inv[24]._flags_tm&0x02)||((inv[24]._flags_tm&0x04)&&(!(inv[24]._flags_tm&(1<<5)))&&(!(inv[24]._flags_tm&0x01))&&(!(inv[24]._flags_tm_dop&0x01)))))//if((inv[24]._flags_tm&0x06)&&/*&&(!(inv[24]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№25 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№25 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[24]._flags_tm&0x08)&&(inv[24]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№25 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№25 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[24]._flags_tm&0x10)&&(inv[24]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№25 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№25  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[24]._flags_tm_dop&0x01)&&(inv[24]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№25 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№25  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[25]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№26 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№26 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[25]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№26 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№26 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[25]._flags_tm&0x01)&&(inv[25]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№26 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№26 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[25]._flags_tm&0x04)&&(inv[25]._valid)&&(!(inv[25]._flags_tm&0x02))&&(inv[25]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№26 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№26  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[25]._valid)&&((inv[25]._flags_tm&0x02)||((inv[25]._flags_tm&0x04)&&(!(inv[25]._flags_tm&(1<<5)))&&(!(inv[25]._flags_tm&0x01))&&(!(inv[25]._flags_tm_dop&0x01)))))//if((inv[25]._flags_tm&0x06)&&/*&&(!(inv[25]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№26 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№26 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[25]._flags_tm&0x08)&&(inv[25]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№26 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№26 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[25]._flags_tm&0x10)&&(inv[25]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№26 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№26  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[25]._flags_tm_dop&0x01)&&(inv[25]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№26 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№26  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[26]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№27 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№27 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[26]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№27 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№27 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[26]._flags_tm&0x01)&&(inv[26]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№27 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№27 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[26]._flags_tm&0x04)&&(inv[26]._valid)&&(!(inv[26]._flags_tm&0x02))&&(inv[26]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№27 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№27  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[26]._valid)&&((inv[26]._flags_tm&0x02)||((inv[26]._flags_tm&0x04)&&(!(inv[26]._flags_tm&(1<<5)))&&(!(inv[26]._flags_tm&0x01))&&(!(inv[26]._flags_tm_dop&0x01)))))//if((inv[26]._flags_tm&0x06)&&/*&&(!(inv[26]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№27 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№27 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[26]._flags_tm&0x08)&&(inv[26]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№27 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№27 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[26]._flags_tm&0x10)&&(inv[26]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№27 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№27  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[26]._flags_tm_dop&0x01)&&(inv[26]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№27 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№27  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[27]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№28 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№28 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[27]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№28 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№28 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[27]._flags_tm&0x01)&&(inv[27]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№28 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№28 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[27]._flags_tm&0x04)&&(inv[27]._valid)&&(!(inv[27]._flags_tm&0x02))&&(inv[27]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№28 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№28  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[27]._valid)&&((inv[27]._flags_tm&0x02)||((inv[27]._flags_tm&0x04)&&(!(inv[27]._flags_tm&(1<<5)))&&(!(inv[27]._flags_tm&0x01))&&(!(inv[27]._flags_tm_dop&0x01)))))//if((inv[27]._flags_tm&0x06)&&/*&&(!(inv[27]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№28 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№28 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[27]._flags_tm&0x08)&&(inv[27]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№28 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№28 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[27]._flags_tm&0x10)&&(inv[27]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№28 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№28  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[27]._flags_tm_dop&0x01)&&(inv[27]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№28 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№28  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[28]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№29 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№29 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[28]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№29 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№29 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[28]._flags_tm&0x01)&&(inv[28]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№29 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№29 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[28]._flags_tm&0x04)&&(inv[28]._valid)&&(!(inv[28]._flags_tm&0x02))&&(inv[28]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№29 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№29  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[28]._valid)&&((inv[28]._flags_tm&0x02)||((inv[28]._flags_tm&0x04)&&(!(inv[28]._flags_tm&(1<<5)))&&(!(inv[28]._flags_tm&0x01))&&(!(inv[28]._flags_tm_dop&0x01)))))//if((inv[28]._flags_tm&0x06)&&/*&&(!(inv[28]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№29 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№29 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[28]._flags_tm&0x08)&&(inv[28]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№29 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№29 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[28]._flags_tm&0x10)&&(inv[28]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№29 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№29  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[28]._flags_tm_dop&0x01)&&(inv[28]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№29 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№29  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[29]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№30 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№30 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[29]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№30 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№30 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[29]._flags_tm&0x01)&&(inv[29]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№30 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№30 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[29]._flags_tm&0x04)&&(inv[29]._valid)&&(!(inv[29]._flags_tm&0x02))&&(inv[29]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№30 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№30  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[29]._valid)&&((inv[29]._flags_tm&0x02)||((inv[29]._flags_tm&0x04)&&(!(inv[29]._flags_tm&(1<<5)))&&(!(inv[29]._flags_tm&0x01))&&(!(inv[29]._flags_tm_dop&0x01)))))//if((inv[29]._flags_tm&0x06)&&/*&&(!(inv[29]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№30 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№30 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[29]._flags_tm&0x08)&&(inv[29]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№30 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№30 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[29]._flags_tm&0x10)&&(inv[29]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№30 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№30  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[29]._flags_tm_dop&0x01)&&(inv[29]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№30 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№30  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[30]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№31 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№31 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[30]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№31 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№31 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[30]._flags_tm&0x01)&&(inv[30]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№31 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№31 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[30]._flags_tm&0x04)&&(inv[30]._valid)&&(!(inv[30]._flags_tm&0x02))&&(inv[30]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№31 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№31  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[30]._valid)&&((inv[30]._flags_tm&0x02)||((inv[30]._flags_tm&0x04)&&(!(inv[30]._flags_tm&(1<<5)))&&(!(inv[30]._flags_tm&0x01))&&(!(inv[30]._flags_tm_dop&0x01)))))//if((inv[30]._flags_tm&0x06)&&/*&&(!(inv[30]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№31 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№31 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[30]._flags_tm&0x08)&&(inv[30]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№31 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№31 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[30]._flags_tm&0x10)&&(inv[30]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№31 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№31  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[30]._flags_tm_dop&0x01)&&(inv[30]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№31 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№31  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[31]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№32 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№32 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[31]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№32 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№32 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[31]._flags_tm&0x01)&&(inv[31]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№32 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№32 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[31]._flags_tm&0x04)&&(inv[31]._valid)&&(!(inv[31]._flags_tm&0x02))&&(inv[31]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№32 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№32  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[31]._valid)&&((inv[31]._flags_tm&0x02)||((inv[31]._flags_tm&0x04)&&(!(inv[31]._flags_tm&(1<<5)))&&(!(inv[31]._flags_tm&0x01))&&(!(inv[31]._flags_tm_dop&0x01)))))//if((inv[31]._flags_tm&0x06)&&/*&&(!(inv[31]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№32 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№32 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[31]._flags_tm&0x08)&&(inv[31]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№32 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№32 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[31]._flags_tm&0x10)&&(inv[31]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№32 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№32  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[31]._flags_tm_dop&0x01)&&(inv[31]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№32 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№32  low Udc   "; //o_2	
	sub_cnt_max++;	
	}


if(inv[32]._conn_av_stat)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№33 разрыв связи"; //o_2
	else 			sub_ptrs[i++]=	"Inv№33 not connected"; //o_2
	sub_cnt_max++;	
	}

if(inv[32]._inv_int_err_cnt>149)
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№33 внутр.неиспр"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№33 Defective.  "; //o_2
	sub_cnt_max++;
	}

if((inv[32]._flags_tm&0x01)&&(inv[32]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№33 перегрузка  "; //o_2
	else 			sub_ptrs[i++]=	"Inv.№33 Overload.   "; //o_2
	sub_cnt_max++;	
	}

if((inv[32]._flags_tm&0x04)&&(inv[32]._valid)&&(!(inv[32]._flags_tm&0x02))&&(inv[32]._flags_tm&(1<<5)))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№33 сильн.нагрев"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№33  high heat  "; //o_2
	sub_cnt_max++;	
	}

if((inv[32]._valid)&&((inv[32]._flags_tm&0x02)||((inv[32]._flags_tm&0x04)&&(!(inv[32]._flags_tm&(1<<5)))&&(!(inv[32]._flags_tm&0x01))&&(!(inv[32]._flags_tm_dop&0x01)))))//if((inv[32]._flags_tm&0x06)&&/*&&(!(inv[32]._flags_tm&0x04))*/))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№33 перегрев,вык"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№33 temp,disabl."; //o_2
	sub_cnt_max++;	
	}

if((inv[32]._flags_tm&0x08)&&(inv[32]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№33 завыш. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№33 high Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[32]._flags_tm&0x10)&&(inv[32]._valid))
	{
	if(language) 	sub_ptrs[i++]=	"Инв.№33 заниж. Uвых!"; //o_2
	else 			sub_ptrs[i++]=	"Inv.№33  low Uout!  "; //o_2
	sub_cnt_max++;	
	}

if((inv[32]._flags_tm_dop&0x01)&&(inv[32]._valid))
	{
	if(language) 	sub_ptrs[i++]=	" Инв.№33 заниж. Udc "; //o_2
	else 			sub_ptrs[i++]=	" Inv.№33  low Udc   "; //o_2	
	sub_cnt_max++;	
	}
cnt_of_slave=/*0502NUMIST+0502*/NUMINV;


//cnt_of_wrks=0;
//for(i=0;i<NUMIST;i++)
 //    {
//     if(bps[i]._state==bsWRK)cnt_of_wrks++;
  //   }


if((sk_av_stat[0]==sasON)&&(NUMSK)&&(!SK_LCD_EN[0]))
	{
	sub_ptrs[i++]=	"   Сработал СК№1    ";
	sub_cnt_max++;	
	}
/*
sub_ptrs[0]=	"Сработал СК №1";	
sub_ptrs[1]=	"Инвертор №33 занижено Uвых!";
sub_ptrs[2]=	"Приоритет инверторы";
sub_ptrs[3]=	"Приоритет сеть";
sub_ptrs[4]=	"Работа от инверторов";

sub_cnt_max=5;*/

sub_cnt1++;	
if(sub_cnt1>=6)
	{
	sub_cnt1=0;
	sub_cnt++;
	if(sub_cnt>=sub_cnt_max)
		{
		sub_cnt=0;
		}
	}

sprintf(buffer,"%s", pal_cyr_coder((char*)sub_ptrs[sub_cnt]));
//sprintf(buffer,"%s", pal_cyr_coder("Мама мыла раму"));

return buffer;
}





//-----------------------------------------------
char* http_ip_output(char ip1, char ip2, char ip3, char ip4)
{
char buffer[100];

sprintf(buffer,"%d.%d.%d.%d", ip1, ip2, ip3, ip4);

return buffer;
}

//-----------------------------------------------
char* http_tm_bat_output(char numOfBat)
{
char buffer[300];
char* batstat="abcdef";

short batison=0;
short batcreal=-1;
short batubm=-1;
if(BAT_IS_ON[numOfBat]==bisON)batison=1;
if(BAT_C_REAL[numOfBat]!=0x5555)batcreal=BAT_C_REAL[numOfBat];
else batcreal=-BAT_C_NOM[numOfBat]*10;
if(UBM_AV)batubm=bat[numOfBat]._Ubm;

if(bat[numOfBat]._Ib>0)	batstat=pal_cyr_coder("заряжается");
else batstat=pal_cyr_coder("разряжается");
if(bat[numOfBat]._av&1)batstat=pal_cyr_coder("Авария цепи батареи!!!");
if(bat[numOfBat]._av&2)batstat=pal_cyr_coder("Авария средней точки батареи!!!");
//batstat=pal_cyr_coder("Авария средней точки батареи!!!");

sprintf(buffer," %d, %d, %d, %d, %d, %d, %d, %d, %d, %s", batison, bat[numOfBat]._Ub, bat[numOfBat]._Ib, bat[numOfBat]._Tb,
 		bat[numOfBat]._nd, batcreal, bat[numOfBat]._zar,BAT_RESURS[numOfBat],batubm, batstat);

return buffer;
}

/*

 char iii;
char dt[4],dt_[4],dt__[4];
    
		


     
lc640_read_long_ptr(tempii,dt);
lc640_read_long_ptr(tempii+8,dt_);
lc640_read_long_ptr(tempii+12,dt__);
*/

void demo_avar_vrite(void)
{

}

