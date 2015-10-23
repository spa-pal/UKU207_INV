#include "snmp_data_file.h" 
#include "eeprom_map.h"
#include "main.h"
#include "net_config.h"
//#include "main.h"
#include "control.h"
#include "LPC17xx.H"
#include <string.h>
#include "25lc640.h"
#include "common_func.h"

char snmp_community[10];

//Информация об устройстве
signed short snmp_device_code;
signed 	   snmp_sernum;
signed short snmp_sernum_lsb;
signed short snmp_sernum_msb;
char 	   snmp_location[100];
signed short snmp_numofbat;
signed short snmp_numofbps;
signed short snmp_numofinv;
signed short snmp_numofavt;
signed short snmp_numofdt;
signed short snmp_numofsk;
signed short snmp_numofevents;


//Состояние первичной сети
signed short snmp_mains_power_voltage;
signed short snmp_mains_power_frequency;
signed short snmp_mains_power_status;
signed short snmp_mains_power_alarm;
signed short snmp_mains_power_voltage_phaseA;
signed short snmp_mains_power_voltage_phaseB;
signed short snmp_mains_power_voltage_phaseC;

//Состояние нагрузки
signed short snmp_load_voltage;
signed short snmp_load_current;

//Состояние БПСов
signed short snmp_bps_number[8];
signed short snmp_bps_voltage[8];
signed short snmp_bps_current[8];
signed short snmp_bps_temperature[8];
signed short snmp_bps_stat[8];

//Состояние инверторов
signed short snmp_inv_number[20];
signed short snmp_inv_voltage[20];
signed short snmp_inv_current[20];
signed short snmp_inv_temperature[20];
signed short snmp_inv_stat[20];

//Состояние Батарей
signed short snmp_bat_number[2];
signed short snmp_bat_voltage[2];
signed short snmp_bat_current[2];
signed short snmp_bat_temperature[2];
signed short snmp_bat_capacity[2];
signed short snmp_bat_charge[2];
signed short snmp_bat_status[2]; 

//Мониторы состояния Батарей
signed short snmp_makb_number[4];
signed short snmp_makb_connect_status[4];
signed short snmp_makb_voltage0[4];
signed short snmp_makb_voltage1[4];
signed short snmp_makb_voltage2[4];
signed short snmp_makb_voltage3[4];
signed short snmp_makb_voltage4[4];
signed short snmp_makb_temper0[4];
signed short snmp_makb_temper1[4];
signed short snmp_makb_temper2[4];
signed short snmp_makb_temper3[4];
signed short snmp_makb_temper4[4];
signed short snmp_makb_temper0_stat[4];
signed short snmp_makb_temper1_stat[4];
signed short snmp_makb_temper2_stat[4];
signed short snmp_makb_temper3_stat[4];
signed short snmp_makb_temper4_stat[4];
signed short snmp_bat_voltage[2];
signed short snmp_bat_part_voltage[2];
signed short snmp_bat_current[2];
signed short snmp_bat_temperature[2];
signed short snmp_bat_capacity[2];
signed short snmp_bat_charge[2];
signed short snmp_bat_status[2]; 

//Спецфункции
signed short snmp_spc_stat;
char snmp_spc_trap_message[100];
signed short snmp_spc_trap_value_0,snmp_spc_trap_value_1,snmp_spc_trap_value_2;

//Состояние автоматов
signed char snmp_avt_number[12];
signed char snmp_avt_stat[12];

//Состояние силовых вводов
signed short snmp_energy_vvod_phase_a;
signed short snmp_energy_vvod_phase_b;
signed short snmp_energy_vvod_phase_c;
signed short snmp_energy_pes_phase_a;
signed short snmp_energy_pes_phase_b;
signed short snmp_energy_pes_phase_c;

//Показания счетчика
signed long snmp_energy_total_energy;
signed short snmp_energy_current_energy;

//Состояние сухих контактов
signed char snmp_sk_number[4];
signed char snmp_sk_aktiv[4];
signed char snmp_sk_alarm_aktiv[4];
signed char snmp_sk_alarm[4];
char snmp_sk_name[4][20];

//Состояние датчиков температур
signed char snmp_dt_number[3];
signed short snmp_dt_temper[3];
signed char snmp_dt_error[3];

//Команды
signed short snmp_command;
signed short snmp_command_parametr;

//Журнал аварий
char snmp_log[64][128]=
				{
				"01@abcd@efgh@ijkl@01@        ",
				"02@abcd@efgh@ijkl@02@        ",
				"03@abcd@efgh@ijkl@03@        ",
				"04@abcd@efgh@ijkl@04@        ",
				"05@abcd@efgh@ijkl@05@        ",
				"06@abcd@efgh@ijkl@06@        ",
				"07@abcd@efgh@ijkl@07@        ",
				"08@abcd@efgh@ijkl@08@        ",
				"09@abcd@efgh@ijkl@09@        ",
				"10@abcd@efgh@ijkl@10@        ",
				"11@abcd@efgh@ijkl@11@        ",
				"12@abcd@efgh@ijkl@12@        ",
				"13@abcd@efgh@ijkl@13@        ",
				"14@abcd@efgh@ijkl@14@        ",
				"15@abcd@efgh@ijkl@15@        ",
				"16@abcd@efgh@ijkl@16@        ",
				"17@abcd@efgh@ijkl@17@        ",
				"18@abcd@efgh@ijkl@18@        ",
				"19@abcd@efgh@ijkl@19@        ",
				"20@abcd@efgh@ijkl@20@        ",
				"21@abcd@efgh@ijkl@21@        ",
				"22@abcd@efgh@ijkl@22@        ",
				"23@abcd@efgh@ijkl@23@        ",
				"24@abcd@efgh@ijkl@24@        ",
				"25@abcd@efgh@ijkl@25@        ",
				"26@abcd@efgh@ijkl@26@        ",
				"27@abcd@efgh@ijkl@27@        ",
				"28@abcd@efgh@ijkl@28@        ",
				"29@abcd@efgh@ijkl@29@        ",
				"30@abcd@efgh@ijkl@30@        "
				};

//Установочные параметры
signed short snmp_main_bps;
signed short snmp_zv_en;
signed short snmp_alarm_auto_disable;
signed short snmp_bat_test_time;
signed short snmp_u_max;
signed short snmp_u_min;
signed short snmp_u_0_grad;
signed short snmp_u_20_grad;
signed short snmp_u_sign;
signed short snmp_u_min_power;
signed short snmp_u_withouth_bat;
signed short snmp_control_current;
signed short snmp_max_charge_current;
signed short snmp_max_current;
signed short snmp_min_current;
signed short snmp_uvz;
signed short snmp_max_current_koef;
signed short snmp_max_current_koef;
signed short snmp_up_charge_koef;
signed short snmp_powerup_psu_timeout;
signed short snmp_max_temperature;
signed short snmp_tsign_bat; 
signed short snmp_tmax_bat;
signed short snmp_tsign_bps;
signed short snmp_tmax_bps;
signed short snmp_bat_part_alarm;
signed short snmp_power_cnt_adress;

//Климат-контроль
signed short snmp_klimat_box_temper;
signed short snmp_klimat_settings_box_alarm;
signed short snmp_klimat_settings_vent_on;
signed short snmp_klimat_settings_vent_off;
signed short snmp_klimat_settings_warm_on;
signed short snmp_klimat_settings_warm_off;
signed short snmp_klimat_settings_load_on;
signed short snmp_klimat_settings_load_off;
signed short snmp_klimat_settings_batt_on;
signed short snmp_klimat_settings_batt_off;

//Внешние датчики температур
signed short snmp_dt_ext;
signed short snmp_dt_msan;
signed short snmp_dt_epu;


U16 obj[10];
U8 temp_ip[4];
char snmp_trap_send_i,snmp_trap_send_ii;

//-----------------------------------------------
void snmp_data (void) 
{
char i;

snmp_mains_power_voltage=net_U;
#ifdef UKU_220_IPS_TERMOKOMPENSAT
snmp_mains_power_frequency=net_F3;
#else
snmp_mains_power_frequency=net_F;
#endif
snmp_mains_power_voltage_phaseA=net_Ua;
snmp_mains_power_voltage_phaseB=net_Ub;
snmp_mains_power_voltage_phaseC=net_Uc;

/*
snmp_mains_power_status=0; 
#if(UKU_VERSION==900)
snmp_mains_power_status=2;
#endif
if(St&0x01)snmp_mains_power_status|=0x01;
if(St&0x01)snmp_mains_power_alarm=1;
*/





//for(i=0;i<snmp_numofevents;i++)event2snmp(i);

snmp_log[0][0]='5';
snmp_log[1][0]='6';


//snmp_bpsnumber[0]=1;
//snmp_bpsnumber[1]=2;
/*
snmp_sernum=AUSW_MAIN_NUMBER;
snmp_sernum_lsb=0x1122;
snmp_sernum_msb=0x3344;
snmp_device_code=AUSW_MAIN;

//memcpy(snmp_location,"lkhg;la",);


snmp_numofbat=1;

*/

snmp_load_voltage=lc640_read_int(CNT_EVENT_LOG);//load_U;
snmp_load_current=load_I;
snmp_numofbat=NUMBAT;
snmp_numofbps=NUMIST;
snmp_numofinv=NUMINV;
snmp_numofavt=NUMAVT;
snmp_numofdt=NUMDT;
snmp_numofsk=NUMSK;
snmp_numofevents=lc640_read_int(CNT_EVENT_LOG);

snmp_energy_vvod_phase_a=Uvv_eb2[0];
snmp_energy_vvod_phase_b=Uvv_eb2[1];
snmp_energy_vvod_phase_c=Uvv_eb2[2];
snmp_energy_pes_phase_a=Upes_eb2[0];
snmp_energy_pes_phase_b=Upes_eb2[1];
snmp_energy_pes_phase_c=Upes_eb2[2];

#ifdef UKU_KONTUR
snmp_energy_vvod_phase_a=Uvv0;
snmp_energy_pes_phase_a=Uvv[1];
#endif

snmp_energy_total_energy=power_summary;
snmp_energy_current_energy=power_current;


snmp_bat_number[0]=1;
snmp_bat_voltage[0]=bat[0]._Ub;
snmp_bat_part_voltage[0]=bat[0]._Ubm;
snmp_bat_current[0]=bat[0]._Ib;

#ifdef UKU_220_IPS_TERMOKOMPENSAT
if(((AUSW_MAIN==22063)||(AUSW_MAIN==22023))&&(bps[8]._device==dIBAT_METR))
	{
	snmp_bat_current[0]=Ib_ips_termokompensat;
	}
#endif

snmp_bat_temperature[0]=bat[0]._Tb;
if(BAT_C_REAL[0]==0x5555)snmp_bat_capacity[0]=BAT_C_NOM[0];
else snmp_bat_capacity[0]=BAT_C_REAL[0];
snmp_bat_charge[0]=bat[0]._zar;
snmp_bat_status[0]=bat[0]._av;


snmp_bat_number[1]=2;
snmp_bat_voltage[1]=bat[1]._Ub;
snmp_bat_part_voltage[1]=bat[1]._Ubm;
snmp_bat_current[1]=bat[1]._Ib;
snmp_bat_temperature[1]=bat[1]._Tb;
if(BAT_C_REAL[1]==0x5555)snmp_bat_capacity[1]=BAT_C_NOM[1];
else snmp_bat_capacity[1]=BAT_C_REAL[1];
snmp_bat_charge[1]=bat[1]._zar;
snmp_bat_status[1]=bat[1]._av;


snmp_bps_number[0]=1;
snmp_bps_voltage[0]=bps[0]._Uii;
snmp_bps_current[0]=bps[0]._Ii;
snmp_bps_temperature[0]=bps[0]._Ti;
snmp_bps_stat[0]=bps[0]._av;												//Байт состояния БПСа.

/*if(St_[0]&(1<<2))snmp_bps_stat[0]=(1<<3); 							//авария по Umin
else if(St_[0]&(1<<3))snmp_bps_stat[0]=(1<<2); 						//авария по Umax
else if(bps[0]._av&(1<<0))snmp_bps_stat[0]=(1<<1); 						//авария по Tmax
else if(St_[0]&(1<<5))snmp_bps_stat[0]=(1<<5); 						//заблокирован
else if((!(St_[0]&0x3c))&&(!St&0x01)&&(!OFFBP1))snmp_bps_stat[0]=1; 		//Работает
*/

snmp_bps_number[1]=2;
snmp_bps_voltage[1]=bps[1]._Uii;
snmp_bps_current[1]=bps[1]._Ii;
snmp_bps_temperature[1]=bps[1]._Ti;
snmp_bps_stat[1]=bps[1]._av;
												//Байт состояния БПСа.
/*if(St_[1]&(1<<2))snmp_bps_stat[1]=(1<<3); 							//авария по Umin
else if(St_[1]&(1<<3))snmp_bps_stat[1]=(1<<2); 						//авария по Umax
else if(St_[1]&(1<<4))snmp_bps_stat[1]=(1<<1); 						//авария по Tmax
else if(St_[1]&(1<<5))snmp_bps_stat[1]=(1<<5); 						//заблокирован
else if((!(St_[1]&0x3c))&&(!St&0x01)&&(!OFFBP2))snmp_bps_stat[1]=1; 		//Работает
*/

snmp_bps_number[2]=3;
snmp_bps_voltage[2]=bps[2]._Uii;
snmp_bps_current[2]=bps[2]._Ii;
snmp_bps_temperature[2]=bps[2]._Ti;
snmp_bps_stat[2]=bps[2]._av;

snmp_bps_number[3]=4;
snmp_bps_voltage[3]=bps[3]._Uii;
snmp_bps_current[3]=bps[3]._Ii;
snmp_bps_temperature[3]=bps[3]._Ti;
snmp_bps_stat[3]=bps[3]._av;

snmp_bps_number[4]=5;
snmp_bps_voltage[4]=bps[4]._Uii;
snmp_bps_current[4]=bps[4]._Ii;
snmp_bps_temperature[4]=bps[4]._Ti;
snmp_bps_stat[4]=bps[4]._av;

snmp_bps_number[5]=6;
snmp_bps_voltage[5]=bps[5]._Uii;
snmp_bps_current[5]=bps[5]._Ii;
snmp_bps_temperature[5]=bps[5]._Ti;
snmp_bps_stat[5]=bps[5]._av;

snmp_bps_number[6]=7;
snmp_bps_voltage[6]=bps[6]._Uii;
snmp_bps_current[6]=bps[6]._Ii;
snmp_bps_temperature[6]=bps[6]._Ti;
snmp_bps_stat[6]=bps[6]._av;

snmp_bps_number[7]=8;
snmp_bps_voltage[7]=bps[7]._Uii;
snmp_bps_current[7]=bps[7]._Ii;
snmp_bps_temperature[7]=bps[7]._Ti;
snmp_bps_stat[7]=bps[7]._av;



snmp_inv_number[0]=1;
snmp_inv_voltage[0]=inv[0]._Uio;
snmp_inv_current[0]=inv[0]._Ii;
snmp_inv_temperature[0]=inv[0]._Ti;
snmp_inv_stat[0]=inv[0]._flags_tm;

snmp_inv_number[1]=2;
snmp_inv_voltage[1]=inv[1]._Uio;
snmp_inv_current[1]=inv[1]._Ii;
snmp_inv_temperature[1]=inv[1]._Ti;
snmp_inv_stat[1]=inv[1]._flags_tm;

snmp_inv_number[2]=3;
snmp_inv_voltage[2]=inv[2]._Uio;
snmp_inv_current[2]=inv[2]._Ii;
snmp_inv_temperature[2]=inv[2]._Ti;
snmp_inv_stat[2]=inv[2]._flags_tm;

snmp_inv_number[3]=4;
snmp_inv_voltage[3]=inv[3]._Uio;
snmp_inv_current[3]=inv[3]._Ii;
snmp_inv_temperature[3]=inv[3]._Ti;
snmp_inv_stat[3]=inv[3]._flags_tm;

snmp_inv_number[4]=5;
snmp_inv_voltage[4]=inv[4]._Uio;
snmp_inv_current[4]=inv[4]._Ii;
snmp_inv_temperature[4]=inv[4]._Ti;
snmp_inv_stat[4]=inv[4]._flags_tm;

snmp_inv_number[5]=6;
snmp_inv_voltage[5]=inv[5]._Uio;
snmp_inv_current[5]=inv[5]._Ii;
snmp_inv_temperature[5]=inv[5]._Ti;
snmp_inv_stat[5]=inv[5]._flags_tm;

snmp_inv_number[6]=7;
snmp_inv_voltage[6]=inv[6]._Uio;
snmp_inv_current[6]=inv[6]._Ii;
snmp_inv_temperature[6]=inv[6]._Ti;
snmp_inv_stat[6]=inv[6]._flags_tm;

snmp_inv_number[7]=8;
snmp_inv_voltage[7]=inv[7]._Uio;
snmp_inv_current[7]=inv[7]._Ii;
snmp_inv_temperature[7]=inv[7]._Ti;
snmp_inv_stat[7]=inv[7]._flags_tm;

snmp_inv_number[8]=9;
snmp_inv_voltage[8]=inv[8]._Uio;
snmp_inv_current[8]=inv[8]._Ii;
snmp_inv_temperature[8]=inv[8]._Ti;
snmp_inv_stat[8]=inv[8]._flags_tm;

snmp_inv_number[9]=10;
snmp_inv_voltage[9]=inv[9]._Uio;
snmp_inv_current[9]=inv[9]._Ii;
snmp_inv_temperature[9]=inv[9]._Ti;
snmp_inv_stat[9]=inv[9]._flags_tm;

snmp_inv_number[10]=11;
snmp_inv_voltage[10]=inv[10]._Uio;
snmp_inv_current[10]=inv[10]._Ii;
snmp_inv_temperature[10]=inv[10]._Ti;
snmp_inv_stat[10]=inv[10]._flags_tm;

snmp_inv_number[11]=12;
snmp_inv_voltage[11]=inv[11]._Uio;
snmp_inv_current[11]=inv[11]._Ii;
snmp_inv_temperature[11]=inv[11]._Ti;
snmp_inv_stat[11]=inv[11]._flags_tm;

snmp_inv_number[12]=13;
snmp_inv_voltage[12]=inv[12]._Uio;
snmp_inv_current[12]=inv[12]._Ii;
snmp_inv_temperature[12]=inv[12]._Ti;
snmp_inv_stat[12]=inv[12]._flags_tm;

snmp_inv_number[13]=14;
snmp_inv_voltage[13]=inv[13]._Uio;
snmp_inv_current[13]=inv[13]._Ii;
snmp_inv_temperature[13]=inv[13]._Ti;
snmp_inv_stat[13]=inv[13]._flags_tm;

snmp_inv_number[14]=15;
snmp_inv_voltage[14]=inv[14]._Uio;
snmp_inv_current[14]=inv[14]._Ii;
snmp_inv_temperature[14]=inv[14]._Ti;
snmp_inv_stat[14]=inv[14]._flags_tm;

snmp_inv_number[15]=16;
snmp_inv_voltage[15]=inv[15]._Uio;
snmp_inv_current[15]=inv[15]._Ii;
snmp_inv_temperature[15]=inv[15]._Ti;
snmp_inv_stat[15]=inv[15]._flags_tm;

snmp_inv_number[16]=17;
snmp_inv_voltage[16]=inv[16]._Uio;
snmp_inv_current[16]=inv[16]._Ii;
snmp_inv_temperature[16]=inv[16]._Ti;
snmp_inv_stat[16]=inv[16]._flags_tm;

snmp_inv_number[17]=18;
snmp_inv_voltage[17]=inv[17]._Uio;
snmp_inv_current[17]=inv[17]._Ii;
snmp_inv_temperature[17]=inv[17]._Ti;
snmp_inv_stat[17]=inv[17]._flags_tm;

snmp_inv_number[18]=19;
snmp_inv_voltage[18]=inv[18]._Uio;
snmp_inv_current[18]=inv[18]._Ii;
snmp_inv_temperature[18]=inv[18]._Ti;
snmp_inv_stat[18]=inv[18]._flags_tm;

snmp_inv_number[19]=20;
snmp_inv_voltage[19]=inv[19]._Uio;
snmp_inv_current[19]=inv[19]._Ii;
snmp_inv_temperature[19]=inv[19]._Ti;
snmp_inv_stat[19]=inv[19]._flags_tm;


snmp_sk_number[0]=1;
memcpy(&snmp_sk_name[0][0],"Shock",10);
if(sk_stat[0]==ssON) snmp_sk_aktiv[0]=1;
else snmp_sk_aktiv[0]=0;
if(!SK_SIGN[0])snmp_sk_alarm_aktiv[0]=1;
else snmp_sk_alarm_aktiv[0]=0;
if(sk_av_stat[0]==sasON)	snmp_sk_alarm[0]=1;
else                     snmp_sk_alarm[0]=0;


snmp_sk_number[1]=2;
memcpy(&snmp_sk_name[1][0],"Smoke",10);
if(sk_stat[1]==ssON) snmp_sk_aktiv[1]=1;
else snmp_sk_aktiv[1]=0;
if(!SK_SIGN[1])snmp_sk_alarm_aktiv[1]=1;
else snmp_sk_alarm_aktiv[1]=0;
if(sk_av_stat[1]==sasON)	snmp_sk_alarm[1]=1;
else                     snmp_sk_alarm[1]=0;

snmp_sk_number[2]=3;
memcpy(&snmp_sk_name[2][0],"Door",10);
if(sk_stat[2]==ssON) snmp_sk_aktiv[2]=1;
else snmp_sk_aktiv[2]=0;
if(!SK_SIGN[2])snmp_sk_alarm_aktiv[2]=1;
else snmp_sk_alarm_aktiv[2]=0;
if(sk_av_stat[2]==sasON)	snmp_sk_alarm[2]=1;
else                     snmp_sk_alarm[2]=0;

snmp_sk_number[3]=4;
memcpy(&snmp_sk_name[3][0],"     ",10);
if(sk_stat[3]==ssON) snmp_sk_aktiv[3]=1;
else snmp_sk_aktiv[3]=0;
if(!SK_SIGN[3])snmp_sk_alarm_aktiv[3]=1;
else snmp_sk_alarm_aktiv[3]=0;
if(sk_av_stat[3]==sasON)	snmp_sk_alarm[3]=1;
else                     snmp_sk_alarm[3]=0;


if(makb[0]._cnt>8) snmp_makb_connect_status[0]=1;
else if(makb[0]._cnt<2) snmp_makb_connect_status[0]=0;
if(makb[1]._cnt>8) snmp_makb_connect_status[1]=1;
else if(makb[1]._cnt<2) snmp_makb_connect_status[1]=0;
if(makb[2]._cnt>8) snmp_makb_connect_status[2]=1;
else if(makb[2]._cnt<2) snmp_makb_connect_status[2]=0;
if(makb[3]._cnt>8) snmp_makb_connect_status[3]=1;
else if(makb[3]._cnt<2) snmp_makb_connect_status[3]=0;

snmp_makb_voltage0[0]=makb[0]._Ub[0];
snmp_makb_voltage1[0]=makb[0]._Ub[1];
snmp_makb_voltage2[0]=makb[0]._Ub[2];
snmp_makb_voltage3[0]=makb[0]._Ub[3];
snmp_makb_voltage4[0]=makb[0]._Ub[4];
snmp_makb_voltage0[1]=makb[1]._Ub[0];
snmp_makb_voltage1[1]=makb[1]._Ub[1];
snmp_makb_voltage2[1]=makb[1]._Ub[2];
snmp_makb_voltage3[1]=makb[1]._Ub[3];
snmp_makb_voltage4[1]=makb[1]._Ub[4];
snmp_makb_voltage0[2]=makb[2]._Ub[0];
snmp_makb_voltage1[2]=makb[2]._Ub[1];
snmp_makb_voltage2[2]=makb[2]._Ub[2];
snmp_makb_voltage3[2]=makb[2]._Ub[3];
snmp_makb_voltage4[2]=makb[2]._Ub[4];
snmp_makb_voltage0[3]=makb[3]._Ub[0];
snmp_makb_voltage1[3]=makb[3]._Ub[1];
snmp_makb_voltage2[3]=makb[3]._Ub[2];
snmp_makb_voltage3[3]=makb[3]._Ub[3];
snmp_makb_voltage4[3]=makb[3]._Ub[4];

snmp_makb_temper0[0]=makb[0]._T[0];
snmp_makb_temper1[0]=makb[0]._T[1];
snmp_makb_temper2[0]=makb[0]._T[2];
snmp_makb_temper3[0]=makb[0]._T[3];
snmp_makb_temper4[0]=makb[0]._T[4];
snmp_makb_temper0[1]=makb[1]._T[0];
snmp_makb_temper1[1]=makb[1]._T[1];
snmp_makb_temper2[1]=makb[1]._T[2];
snmp_makb_temper3[1]=makb[1]._T[3];
snmp_makb_temper4[1]=makb[1]._T[4];
snmp_makb_temper0[2]=makb[2]._T[0];
snmp_makb_temper1[2]=makb[2]._T[1];
snmp_makb_temper2[2]=makb[2]._T[2];
snmp_makb_temper3[2]=makb[2]._T[3];
snmp_makb_temper4[2]=makb[2]._T[4];
snmp_makb_temper0[3]=makb[3]._T[0];
snmp_makb_temper1[3]=makb[3]._T[1];
snmp_makb_temper2[3]=makb[3]._T[2];
snmp_makb_temper3[3]=makb[3]._T[3];
snmp_makb_temper4[3]=makb[3]._T[4];

snmp_makb_temper0_stat[0]=makb[0]._T_nd[0];
snmp_makb_temper1_stat[0]=makb[0]._T_nd[1];
snmp_makb_temper2_stat[0]=makb[0]._T_nd[2];
snmp_makb_temper3_stat[0]=makb[0]._T_nd[3];
snmp_makb_temper4_stat[0]=makb[0]._T_nd[4];
snmp_makb_temper0_stat[1]=makb[1]._T_nd[0];
snmp_makb_temper1_stat[1]=makb[1]._T_nd[1];
snmp_makb_temper2_stat[1]=makb[1]._T_nd[2];
snmp_makb_temper3_stat[1]=makb[1]._T_nd[3];
snmp_makb_temper4_stat[1]=makb[1]._T_nd[4];
snmp_makb_temper0_stat[2]=makb[2]._T_nd[0];
snmp_makb_temper1_stat[2]=makb[2]._T_nd[1];
snmp_makb_temper2_stat[2]=makb[2]._T_nd[2];
snmp_makb_temper3_stat[2]=makb[2]._T_nd[3];
snmp_makb_temper4_stat[2]=makb[2]._T_nd[4];
snmp_makb_temper0_stat[3]=makb[3]._T_nd[0];
snmp_makb_temper1_stat[3]=makb[3]._T_nd[1];
snmp_makb_temper2_stat[3]=makb[3]._T_nd[2];
snmp_makb_temper3_stat[3]=makb[3]._T_nd[3];
snmp_makb_temper4_stat[3]=makb[3]._T_nd[4];


snmp_klimat_box_temper=t_box;
snmp_klimat_settings_box_alarm=TBOXMAX;
snmp_klimat_settings_vent_on=TBOXVENTON;
snmp_klimat_settings_vent_off=TBOXVENTOFF;
snmp_klimat_settings_warm_on=TBOXWARMON;
snmp_klimat_settings_warm_off=TBOXWARMOFF;
snmp_klimat_settings_load_on=TLOADENABLE;
snmp_klimat_settings_load_off=TLOADDISABLE;
snmp_klimat_settings_batt_on=TBATENABLE;
snmp_klimat_settings_batt_off=TBATDISABLE;

snmp_dt_ext=t_ext[0];
snmp_dt_msan=t_ext[1];
snmp_dt_epu=t_ext[2];

/*
snmp_bat_voltage=Ubat;
snmp_bat_current=Ibat;
snmp_bat_temperature=t_b;
if(BAT_C_REAL==0x5555)
	{
	snmp_bat_capacity=BAT_C_NOM*10;												    11
	}
else
	{
	snmp_bat_capacity=BAT_C_REAL;
	}
snmp_bat_charge=zar_percent;
snmp_bat_status=0;
if(St&0x02)snmp_bat_status|=0x01;
if(Ibat>0)snmp_bat_status|=0x02;


if(spc_stat==spc_OFF) snmp_spc_stat=0;
else if(spc_stat==spc_KE) snmp_spc_stat=1;
else if(spc_stat==spc_VZ) snmp_spc_stat=10;


snmp_main_bps=MAIN_IST+1;
*/
snmp_zv_en=ZV_ON;
snmp_alarm_auto_disable=AV_OFF_AVT;
snmp_bat_test_time=TBAT;
snmp_u_max=UMAX;
snmp_u_min=UB20-DU;
snmp_u_0_grad=UB0;
snmp_u_20_grad=UB20;
snmp_u_sign=USIGN;
snmp_u_min_power=UMN;
snmp_u_withouth_bat=U0B;
snmp_control_current=IKB;
snmp_max_charge_current=IZMAX;
snmp_max_current=IMAX;
snmp_min_current=IMIN;
//snmp_max_current_koef=KIMAX;
snmp_uvz=UVZ;
snmp_powerup_psu_timeout=TZAS;
snmp_max_temperature=TMAX;
snmp_tsign_bat=TBATSIGN; 
snmp_tmax_bat=TBATMAX;
snmp_tsign_bps=TSIGN;
snmp_tmax_bps=TMAX;
snmp_bat_part_alarm=UBM_AV; 
snmp_power_cnt_adress=POWER_CNT_ADRESS;

for(i=0;i<12;i++)
	{
	snmp_avt_number[i]=i+1;
	if(avt_stat[i]==avtOFF)snmp_avt_stat[i]=0;
	else snmp_avt_stat[i]=1;
	}

snmp_dt_temper[0]=t_ext[0];
snmp_dt_temper[1]=t_ext[1];
snmp_dt_temper[2]=t_ext[2];
}

//-----------------------------------------------
void snmp_sernum_write (int mode) 
{
if(mode==MIB_WRITE)
	{
	lc640_write_long(EE_AUSW_MAIN_NUMBER,snmp_sernum);
	lc640_write_long(EE_AUSW_UKU_NUMBER,snmp_sernum);
	}
}

//-----------------------------------------------
void snmp_location_write (int mode) 
{
char i;
if(mode==MIB_WRITE)
	{
	for(i=0;i<64;i++)
		{
		lc640_write(EE_LOCATION+i,snmp_location[i]);
		}
	}
}

//-----------------------------------------------
void snmp_alarm_aktiv_write1(void)
{
if(!snmp_sk_alarm_aktiv[0])   lc640_write_int(ADR_SK_SIGN[0],0xffff);
else lc640_write_int(ADR_SK_SIGN[0],0);
}

//-----------------------------------------------
void snmp_alarm_aktiv_write2(void)
{
if(!snmp_sk_alarm_aktiv[1])   lc640_write_int(ADR_SK_SIGN[1],0xffff);
else lc640_write_int(ADR_SK_SIGN[1],0);
}
//-----------------------------------------------
void snmp_alarm_aktiv_write3(void)
{
if(!snmp_sk_alarm_aktiv[2])   lc640_write_int(ADR_SK_SIGN[2],0xffff);
else lc640_write_int(ADR_SK_SIGN[2],0);
}
//-----------------------------------------------
void snmp_alarm_aktiv_write4(void)
{
if(!snmp_sk_alarm_aktiv[3])   lc640_write_int(ADR_SK_SIGN[3],0xffff);
else lc640_write_int(ADR_SK_SIGN[3],0);
}
//-----------------------------------------------
void snmp_main_bps_write (int mode)
{
if(mode==MIB_WRITE)
	{
//	lc640_write_int(EE_MAIN_BPS,snmp_main_bps-1);
	}
}

//-----------------------------------------------
void snmp_zv_on_write (int mode)
{
if(mode==MIB_WRITE)
	{
	lc640_write_int(EE_ZV_ON,snmp_zv_en);
	}
}

//-----------------------------------------------
void snmp_alarm_auto_disable_write (int mode)
{
if(mode==MIB_WRITE)
	{
     lc640_write_int(EE_AV_OFF_AVT,snmp_alarm_auto_disable);
	}
}

//-----------------------------------------------
void snmp_bat_test_time_write (int mode)
{
if(mode==MIB_WRITE)
	{
	lc640_write_int(EE_TBAT,snmp_bat_test_time);
	}
}

//-----------------------------------------------
void snmp_u_max_write (int mode)
{
if(mode==MIB_WRITE)
	{
     lc640_write_int(EE_UMAX,snmp_u_max);
	}
}


//-----------------------------------------------
void snmp_u_min_write (int mode)
{
if(mode==MIB_WRITE)
	{
     lc640_write_int(EE_DU,UB20-snmp_u_min);
	}
}


//-----------------------------------------------
void snmp_u_0_grad_write (int mode)
{
if(mode==MIB_WRITE)
	{
	lc640_write_int(EE_UB0,snmp_u_0_grad);
	}
}
//-----------------------------------------------
void snmp_u_20_grad_write (int mode)
{
if(mode==MIB_WRITE)
	{
	lc640_write_int(EE_UB20,snmp_u_20_grad);
	}
}

//-----------------------------------------------
void snmp_u_sign_write (int mode)
{
if(mode==MIB_WRITE)
	{
 //    lc640_write_int(EE_USIGN,snmp_u_sign);
	}
}
//-----------------------------------------------
void snmp_u_min_power_write (int mode)
{
if(mode==MIB_WRITE)
	{
//     lc640_write_int(EE_UMN,snmp_u_min_power);
	}
}
//-----------------------------------------------
void snmp_u_withouth_bat_write (int mode)
{
if(mode==MIB_WRITE)
	{
	lc640_write_int(EE_U0B,snmp_u_withouth_bat);
	}
}

//-----------------------------------------------
void snmp_control_current_write (int mode)
{
if(mode==MIB_WRITE)
	{
	lc640_write_int(EE_IKB,snmp_control_current);
	}
}

//-----------------------------------------------
void snmp_max_charge_current_write (int mode)
{
if(mode==MIB_WRITE)
	{
	lc640_write_int(EE_IZMAX,snmp_max_charge_current);
	}
}

//-----------------------------------------------
void snmp_max_current_write (int mode)
{
if(mode==MIB_WRITE)
	{
     lc640_write_int(EE_IMAX,snmp_max_current);
	}
}

//-----------------------------------------------
void snmp_min_current_write (int mode)
{
if(mode==MIB_WRITE)
	{
     lc640_write_int(EE_IMIN,snmp_min_current);
	}
}

//-----------------------------------------------
void snmp_max_current_koef_write (int mode)
{
if(mode==MIB_WRITE)
	{
 //    lc640_write_int(EE_KIMAX,snmp_max_current_koef);
	}
}

//-----------------------------------------------
void snmp_up_charge_koef_write (int mode)
{
if(mode==MIB_WRITE)
	{
 //    lc640_write_int(EE_KVZ,snmp_up_charge_koef);
	}
}

//-----------------------------------------------
void snmp_powerup_psu_timeout_write (int mode)
{
if(mode==MIB_WRITE)
	{
     lc640_write_int(EE_TZAS,snmp_powerup_psu_timeout);
	}
}

//-----------------------------------------------
void snmp_max_temperature_write (int mode)
{
if(mode==MIB_WRITE)
	{
 //    lc640_write_int(EE_TMAX,snmp_max_temperature);
	}
}

//-----------------------------------------------
void snmp_tsign_bat_write(int mode)
{
if(mode==MIB_WRITE)
	{
 	lc640_write_int(EE_TBATSIGN,snmp_tsign_bat);
	}
}
//-----------------------------------------------
void snmp_tmax_bat_write(int mode)
{
if(mode==MIB_WRITE)
	{
 	lc640_write_int(EE_TBATMAX,snmp_tmax_bat);
	}
}
//-----------------------------------------------
void snmp_tsign_bps_write(int mode)
{
if(mode==MIB_WRITE)
	{
	lc640_write_int(EE_TSIGN,snmp_tsign_bps);
	}
}
//-----------------------------------------------
void snmp_tmax_bps_write(int mode)
{
if(mode==MIB_WRITE)
	{
	lc640_write_int(EE_TMAX,snmp_tmax_bps);
	}
}

//-----------------------------------------------
void snmp_uvz_write(int mode)
{
if(mode==MIB_WRITE)
	{
	lc640_write_int(EE_UVZ,snmp_uvz);
	}
}
//-----------------------------------------------
void snmp_bat_part_alarm_write(int mode)
{
if(mode==MIB_WRITE)
	{
	lc640_write_int(EE_UBM_AV,snmp_bat_part_alarm);
	}
}
//-----------------------------------------------
void snmp_power_cnt_adress_write(int mode)
{
if(mode==MIB_WRITE)
	{
	lc640_write_int(EE_POWER_CNT_ADRESS,snmp_power_cnt_adress);
	}
}

//-----------------------------------------------
void snmp_command_execute (int mode)
{
if(mode==MIB_WRITE)
	{
	
	//snmp_command=0x5555;

	switch (snmp_command)
		{
		case SNMP_BPS_DISABLE:
			{
			snmp_command=COMMAND_OK;

		/*	switch (snmp_command_parametr)
				{
			
				case 1: 
				{
				St_[0]|=0x20;
				St_[1]&=0xdf;
				St&=0xfb;
				cnt_src[1]=10;
				snmp_plazma++;
				snmp_plazma++;
				break;
				}
			
				case 2:
				{
				St_[1]|=0x20;
				St_[0]&=0xdf;
				St&=0xfb;
				cnt_src[0]=10;	
				snmp_plazma++;
				break;
				}*/	
			
				//break;
			//	}
			if(snmp_command_parametr==1) 
				{
			//	St_[0]|=0x20;
			//	St_[1]&=0xdf;
			//	St&=0xfb;
		//		cnt_src[1]=10;
		//		snmp_plazma++;
		//		snmp_plazma++;
				}
			
			else if(snmp_command_parametr==2)
				{
			//	St_[1]|=0x20;
			//	St_[0]&=0xdf;
			//	St&=0xfb;
		//		cnt_src[0]=10;	
		//		snmp_plazma++;
				}	
			
			break;
			}

		case SNMP_BPS_UNDISABLE:
			{
			snmp_command=COMMAND_OK;
		//	St_[0]&=0xdf;
		//	St_[1]&=0xdf;
			break;
			}

		case SNMP_SPEC_VZ:
			{
			if((snmp_command_parametr>=1)&&(snmp_command_parametr<=24))
				{
			//	if(!(St&0x03)&&(NUMBAT))
					{
					snmp_command=COMMAND_OK;
		//			spc_stat=spc_VZ;
		//			cnt_vz_sec_=3600UL*snmp_command_parametr;
					}
			//	else
 					{
					snmp_command=COMAND_FAIL;	
 					}
				}
			else 
				{
				snmp_command=WRONG_PARAMETER;
				}
			break;
			}

		case SNMP_SPEC_KE:
			{
	  	//	if(!(St&0x02)&&(NUMBAT))
				{
				//spc_stat=spc_KE;
			//zar_cnt_ee_ke=0;
			//	zar_cnt=0L;
				snmp_command=COMMAND_OK;
				}
		//	else
				{
				snmp_command=COMAND_FAIL;	
				}
			break;
			}

		case SNMP_SPEC_DISABLE:
			{
		//	spc_stat=spc_OFF;
			snmp_command=COMMAND_OK;
			break;
			}


		default:
			{
			snmp_command=COMMAND_INVALID;
			break;
			}
		}
/*		else if((UIB2[1]==0x52)&&(UIB2[4]==5)&&(UIB2[5]==5)&&(UIB2[6])&&(UIB2[6]<=NUMIST)&&(UIB2[6]==UIB2[7])) 	//Выключение источника 
		{
	
		if((UIB2[6]==1)&&(UIB2[7]==1)) 
			{
			St_[0]|=0x20;
			St_[1]&=0xdf;
			St&=0xfb;
			cnt_src[1]=10;
			}
			
		else if((UIB2[6]==2)&&(UIB2[7]==2))
			{
			St_[1]|=0x20;
			St_[0]&=0xdf;
			St&=0xfb;
			cnt_src[0]=10;
			}	
		
     	memo_out2[0]=0x33;
     	memo_out2[1]=0x62;
     	memo_out2[2]=4;
     	memo_out2[3]=0x03;
     	
     	memo_out2[4]=5;
     	memo_out2[5]=5;
     	memo_out2[6]=UIB2[6];
     	memo_out2[7]=UIB2[6];
         	memo_out2[8]=crc_87(memo_out2,8);
		memo_out2[9]=crc_95(memo_out2,8);
     	uart_out_adr2(memo_out2,10); 		
		} */



	}
}

//-----------------------------------------------
char* datatime2str(char day,char month,char year, char hour, char min, char sec)
{
char temp_str[20];
memcpy(temp_str,"00/янв/00  00:00:00       ",20);

temp_str[1]=(day%10)+0x30;
temp_str[0]=(day/10)+0x30;

memcpy(&temp_str[3],sm_mont[month],3);

temp_str[8]=(year%10)+0x30;
temp_str[7]=(year/10)+0x30;

temp_str[12]=(hour%10)+0x30;
temp_str[11]=(hour/10)+0x30;

temp_str[15]=(min%10)+0x30;
temp_str[14]=(min/10)+0x30;

temp_str[18]=(sec%10)+0x30;
temp_str[17]=(sec/10)+0x30;
return temp_str;
}

//-----------------------------------------------
void event2snmp(char num)
{
char /*iii,*/index;
char dt[4],dt_[4],dt__[4],dt___[4],dt____[4],dt4[4];
unsigned int tempii;    

memcpy(&snmp_log[num][0],"                                                                                ",78);
//memcpy(&snmp_log[num][0],"BKL",10);


		
tempii=lc640_read_int(PTR_EVENT_LOG);
tempii=ptr_carry(tempii,64,-1*((signed)num));
tempii*=32;
tempii+=EVENT_LOG;
     
lc640_read_long_ptr(tempii,dt);
lc640_read_long_ptr(tempii+4,dt4);
lc640_read_long_ptr(tempii+8,dt_);
lc640_read_long_ptr(tempii+12,dt__);
lc640_read_long_ptr(tempii+16,dt___);
lc640_read_long_ptr(tempii+20,dt____);
//iii=find(simbol);
     
if(dt[0]=='U')	 		//Включение ИБЭПа
    	{ 
    	if(dt[2]=='R')
    		{
		memcpy(&snmp_log[num][0],"Включение ИБЭПа@                                      ",50);
		memcpy(&snmp_log[num][17],datatime2str(dt_[2],dt_[1],dt_[0],dt__[0],dt__[1],dt__[2]),20);
		memcpy(&snmp_log[num][40],"@                   ",20);
    		}
     }   

     
else if(dt[0]=='P')		//Авария питающей сети
	{
	index=0;
     memcpy(&snmp_log[num][index],"Авария питающей сети @  ",23);	
	index+=23;
	memcpy(&snmp_log[num][index],datatime2str(dt_[2],dt_[1],dt_[0],dt__[0],dt__[1],dt__[2]),19);
	index+=19;
	snmp_log[num][index]='@';
	index++;

	if((dt___[0]=='A')&&(dt___[1]=='A'))
		{
		memcpy(&snmp_log[num][index]," не устранена  ",13);
		index+=13;
		}
	else 
		{
		memcpy(&snmp_log[num][index]," устранена   ",11);
		index+=11;
			
		memcpy(&snmp_log[num][index],datatime2str(dt___[2],dt___[1],dt___[0],dt____[0],dt____[1],dt____[2]),19);
		}

     }  

else if(dt[0]=='B')		//События батареи
    	{
	index=0;
    	if(dt[2]=='C')
    		{
		memcpy(&snmp_log[num][index],"Батарея.  Авария!!! @  ",21);
		index+=21;
		memcpy(&snmp_log[num][index],datatime2str(dt_[2],dt_[1],dt_[0],dt__[0],dt__[1],dt__[2]),19);
		index+=19;
		
		snmp_log[num][index]='@';
		index++;

		memcpy(&snmp_log[num][index],"Батарея не обнаружена, ",22);
		index+=22;

		if((dt___[0]=='A')&&(dt___[1]=='A'))
			{
			memcpy(&snmp_log[num][index]," не устранена  ",13);
			index+=13;
			}
		else 
			{
			memcpy(&snmp_log[num][index]," устранена   ",11);
			index+=11;
			memcpy(&snmp_log[num][index],datatime2str(dt___[2],dt___[1],dt___[0],dt____[0],dt____[1],dt____[2]),19);
			}

    		}
    	if(dt[2]=='Z')
    		{

		memcpy(&snmp_log[num][index],"Батарея.  Выравнивающий заряд @  ",32);
		index+=32;
		memcpy(&snmp_log[num][index],datatime2str(dt_[2],dt_[1],dt_[0],dt__[0],dt__[1],dt__[2]),19);
		index+=19;
		
		snmp_log[num][index]='@';
		index++;

		if((dt___[0]=='A')&&(dt___[1]=='A'))
			{
			memcpy(&snmp_log[num][index]," не завершен  ",13);
			index+=13;
			}
		else 
			{
			memcpy(&snmp_log[num][index]," завершен   ",11);
			index+=11;
			memcpy(&snmp_log[num][index],datatime2str(dt___[2],dt___[1],dt___[0],dt____[0],dt____[1],dt____[2]),19);
			}


/*
    		lcd_buffer[iii++]='В';
    		lcd_buffer[iii++]='З';
    		lcd_buffer[iii++]=' ';    		
    		lcd_buffer[iii++]=' ';
    		lcd_buffer[iii++]=' ';  
    		lcd_buffer[iii++]=' ';
    		lcd_buffer[iii++]=' ';
    		
    		if((dt_[0]==LPC_RTC->YEAR)&&(dt_[1]==LPC_RTC->MONTH)&&(dt_[2]==LPC_RTC->DOM))
    			{
    	    		lcd_buffer[iii++]='0';
    			lcd_buffer[iii++]='!';
    	    		lcd_buffer[iii++]=':'; 
    			lcd_buffer[iii++]='0';
    			lcd_buffer[iii++]='@';
    			lcd_buffer[iii++]=':';
    			lcd_buffer[iii++]='0';
    			lcd_buffer[iii++]='#';
    	    		int2lcd(dt__[0],'!',0);
    			int2lcd(dt__[1],'@',0);
    			int2lcd(dt__[2],'#',0);    		     		
    			}	                   
 		else      	
    			{
    	 		lcd_buffer[iii++]=' ';
    			lcd_buffer[iii++]='0';
    			lcd_buffer[iii++]='!';
    			lcd_buffer[iii++]='@'; 
    			lcd_buffer[iii++]=' ';
    			lcd_buffer[iii++]=' ';
    			lcd_buffer[iii++]='0';
    			lcd_buffer[iii++]='#';
    			int2lcd(dt_[2],'!',0);
    			
    			int2lcd(dt_[0],'#',0);   
    			if(!((dt_[1]>=1)&&(dt_[1]<=12)))dt_[1]=1;
			sub_bgnd(sm_mont[dt_[1]],'@',0);  
    			} */ 		
    		}    		
/*
    	if(dt[2]=='W')
    		{
    		lcd_buffer[iii++]='Б';
    		lcd_buffer[iii++]='а';
    		lcd_buffer[iii++]='т';
		if(dt[1]<9)
    			{
    			lcd_buffer[iii++]=0x31+dt[1];
    			lcd_buffer[iii++]='р';
    			lcd_buffer[iii++]=' ';
    			}
    		else if((dt[1]>=9)&&(dt[1]<99))
    			{
    			lcd_buffer[iii++]=0x30+((dt[1]+1)/10);
    			lcd_buffer[iii++]=0x30+((dt[1]+1)%10);
    			lcd_buffer[iii++]='р';
    			}
    		else 
    			{
    			lcd_buffer[iii++]=' ';
    			lcd_buffer[iii++]=' '; 
    			lcd_buffer[iii++]=' ';    		
    			} 
    		lcd_buffer[iii++]=' ';    		
    		
    		
		
    		if((dt_[0]==LPC_RTC->YEAR)&&(dt_[1]==LPC_RTC->MONTH)&&(dt_[2]==LPC_RTC->DOM))
    			{
    	    		lcd_buffer[iii++]='0';
    			lcd_buffer[iii++]='!';
    	    		lcd_buffer[iii++]=':'; 
    			lcd_buffer[iii++]='0';
    			lcd_buffer[iii++]='@';
    			lcd_buffer[iii++]=':';
    			lcd_buffer[iii++]='0';
    			lcd_buffer[iii++]='#';
    	    		int2lcd(dt__[0],'!',0);
    			int2lcd(dt__[1],'@',0);
    			int2lcd(dt__[2],'#',0);    		     		
    			}	                   
 		else      	
    			{
    	 		lcd_buffer[iii++]=' ';
    			lcd_buffer[iii++]='0';
    			lcd_buffer[iii++]='!';
    			lcd_buffer[iii++]='@'; 
    			lcd_buffer[iii++]=' ';
    			lcd_buffer[iii++]=' ';
    			lcd_buffer[iii++]='0';
    			lcd_buffer[iii++]='#';
    			int2lcd(dt_[2],'!',0);
    			
    			int2lcd(dt_[0],'#',0);   
    			if(!((dt_[1]>=1)&&(dt_[1]<=12)))dt_[1]=1;
			sub_bgnd(sm_mont[dt_[1]],'@',0);  
    			}  		
    		} */   		
 
 	if(dt[2]=='K')
    		{

		memcpy(&snmp_log[num][index],"Батарея.  Контроль емкости @  ",29);
		index+=29;
		memcpy(&snmp_log[num][index],datatime2str(dt_[2],dt_[1],dt_[0],dt__[0],dt__[1],dt__[2]),19);
		index+=19;
		
		snmp_log[num][index]='@';
		index++;

		if((dt___[0]=='A')&&(dt___[1]=='A'))
			{
			memcpy(&snmp_log[num][index]," не завершен  ",13);
			index+=13;
			}
		else 
			{
			short temp_US;
			memcpy(&snmp_log[num][index]," завершен   ",11);
			index+=11;
			memcpy(&snmp_log[num][index],datatime2str(dt___[2],dt___[1],dt___[0],dt____[0],dt____[1],dt____[2]),19);
			memcpy(&snmp_log[num][index],", Uнач   , В, Uкон   , В, Cбат    , А*ч ", 39);
			
			temp_US=dt_[3]+(dt__[3]*256);

			snmp_log[num][index+10]=(temp_US%10)+0x30;
			temp_US/=10;
			snmp_log[num][index+8]=(temp_US%10)+0x30;
			temp_US/=10;
			if(temp_US)snmp_log[num][index+7]=(temp_US%10)+0x30;
			else snmp_log[num][index+7]=0x20;


			temp_US=dt4[2]+(dt4[3]*256);

			snmp_log[num][index+22]=(temp_US%10)+0x30;
			temp_US/=10;
			snmp_log[num][index+20]=(temp_US%10)+0x30;
			temp_US/=10;
			if(temp_US)snmp_log[num][index+19]=(temp_US%10)+0x30;
			else snmp_log[num][index+19]=0x20;


			temp_US=dt4[0]+(dt4[1]*256);

			snmp_log[num][index+35]=(temp_US%10)+0x30;
			temp_US/=10;
			snmp_log[num][index+33]=(temp_US%10)+0x30;
			temp_US/=10;
			if(temp_US)snmp_log[num][index+32]=(temp_US%10)+0x30;
			else snmp_log[num][index+32]=0x20;
			}



		/*
    		lcd_buffer[iii++]='Б';
    		lcd_buffer[iii++]='а';
    		lcd_buffer[iii++]='т';
		if(dt[1]<9)
    			{
    			lcd_buffer[iii++]=0x31+dt[1];
    			lcd_buffer[iii++]='к';
    			lcd_buffer[iii++]='е';
    			}
    		else if((dt[1]>=9)&&(dt[1]<99))
    			{
    			lcd_buffer[iii++]=0x30+((dt[1]+1)/10);
    			lcd_buffer[iii++]=0x30+((dt[1]+1)%10);
    			lcd_buffer[iii++]='к';
    			}
    		else 
    			{
    			lcd_buffer[iii++]=' ';
    			lcd_buffer[iii++]=' '; 
    			lcd_buffer[iii++]=' ';    		
    			} 
    		lcd_buffer[iii++]=' ';    		
    		
    		if((dt_[0]==LPC_RTC->YEAR)&&(dt_[1]==LPC_RTC->MONTH)&&(dt_[2]==LPC_RTC->DOM))
    			{
    	    		lcd_buffer[iii++]='0';
    			lcd_buffer[iii++]='!';
    	    		lcd_buffer[iii++]=':'; 
    			lcd_buffer[iii++]='0';
    			lcd_buffer[iii++]='@';
    			lcd_buffer[iii++]=':';
    			lcd_buffer[iii++]='0';
    			lcd_buffer[iii++]='#';
    	    		int2lcd(dt__[0],'!',0);
    			int2lcd(dt__[1],'@',0);
    			int2lcd(dt__[2],'#',0);    		     		
    			}	                   
 		else      	
    			{
    	 		lcd_buffer[iii++]=' ';
    			lcd_buffer[iii++]='0';
    			lcd_buffer[iii++]='!';
    			lcd_buffer[iii++]='@'; 
    			lcd_buffer[iii++]=' ';
    			lcd_buffer[iii++]=' ';
    			lcd_buffer[iii++]='0';
    			lcd_buffer[iii++]='#';
    			int2lcd(dt_[2],'!',0);
    			
    			int2lcd(dt_[0],'#',0);   
    			if(!((dt_[1]>=1)&&(dt_[1]<=12)))dt_[1]=1;
			sub_bgnd(sm_mont[dt_[1]],'@',0);  
    			} */ 		
    		}    		

    		     	     	
    	}     	    
     	
else if(dt[0]=='S')
    	{
	index=0;
	memcpy(&snmp_log[num][0],"БПС №      ",6);
	index=6;
	snmp_log[num][5]=0x31+dt[1];
	index=7;
	memcpy(&snmp_log[num][index],"   Авария!!!@  ",14);
	index+=14;
		//memcpy(&snmp_log[num][0/*+num*/],"00/янв/11 00:00:00 @",20);
	memcpy(&snmp_log[num][index],datatime2str(dt_[2],dt_[1],dt_[0],dt__[0],dt__[1],dt__[2]),19);
	index+=20;

	if(dt[2]=='L')
		{
		memcpy(&snmp_log[num][40],"@отключился             ",20);
		index=65;
		}
	else if(dt[2]=='T')
		{
		memcpy(&snmp_log[num][index],"@ перегрев   ",10);
		index+=10;
		}		
	else if(dt[2]=='U')
		{
		memcpy(&snmp_log[num][index],"@ завышено Uвых.  ",16);
		index+=16;
		}		
	else if(dt[2]=='u')
		{
		memcpy(&snmp_log[num][index],"@ занижено Uвых.  ",16);
		index+=16;
		}
	else 		
		{
		memcpy(&snmp_log[num][index],"@ фигня  ",7);
		index+=7;
		}


	if((dt___[0]=='A')&&(dt___[1]=='A'))
		{
		memcpy(&snmp_log[num][index],", не устранена  ",15);
		index+=15;
		}
	else 
		{
		memcpy(&snmp_log[num][index],",  устранена   ",13);
		index+=13;
			
		memcpy(&snmp_log[num][index],datatime2str(dt___[2],dt___[1],dt___[0],dt____[0],dt____[1],dt____[2]),19);


		    /*
			ptrs[4]="  0[]  0< 0>:0=:0)  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);   */





 /*
		ptrs[0]="   Авария БПС N+    ";
		
		if(av_head[2]=='L')
			{
			ptrs[1]="     отключился     ";
			}
		else if(av_head[2]=='T')
			{
			ptrs[1]="      перегрев      ";
			}		
		else if(av_head[2]=='U')
			{
			ptrs[1]="   завышено Uвых.   ";
			}		
		else if(av_head[2]=='u')
			{
			ptrs[1]="   занижено Uвых.   ";
			}								
		
		
		ptrs[2]="  0%(  0^ 0@:0#:0$  ";
			
						
			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		*/
	    }



/*    	lcd_buffer[iii++]='Б';
    	lcd_buffer[iii++]='П';
    	lcd_buffer[iii++]='С';
 	if(dt[1]<9)
    		{
    		lcd_buffer[iii++]=0x31+dt[1];
    		lcd_buffer[iii++]=' ';
    		}
    	else if((dt[1]>=9)&&(dt[1]<99))
    		{
    		lcd_buffer[iii++]=0x30+((dt[1]+1)/10);
    		lcd_buffer[iii++]=0x30+((dt[1]+1)%10);
    		}
    	else 
    		{
    		lcd_buffer[iii++]=' ';
    		lcd_buffer[iii++]=' ';     		
    		} 
    	lcd_buffer[iii++]=' '; 
    	lcd_buffer[iii++]=' ';
    	
    	if((dt_[0]==LPC_RTC->YEAR)&&(dt_[1]==LPC_RTC->MONTH)&&(dt_[2]==LPC_RTC->DOM))
    		{
    	    	lcd_buffer[iii++]='0';
    		lcd_buffer[iii++]='!';
    	    	lcd_buffer[iii++]=':'; 
    		lcd_buffer[iii++]='0';
    		lcd_buffer[iii++]='@';
    		lcd_buffer[iii++]=':';
    		lcd_buffer[iii++]='0';
    		lcd_buffer[iii++]='#';
    	    	int2lcd(dt__[0],'!',0);
    		int2lcd(dt__[1],'@',0);
    		int2lcd(dt__[2],'#',0);    		     		
    		}	                   
 	else      	
    		{
    	 	lcd_buffer[iii++]=' ';
    		lcd_buffer[iii++]='0';
    		lcd_buffer[iii++]='!';
    		lcd_buffer[iii++]='@'; 
    		lcd_buffer[iii++]=' ';
    		lcd_buffer[iii++]=' ';
    		lcd_buffer[iii++]='0';
    		lcd_buffer[iii++]='#';
    		int2lcd(dt_[2],'!',0);
    		int2lcd(dt_[0],'#',0);   
    		if(!((dt_[1]>=1)&&(dt_[1]<=12)))dt_[1]=1;
		sub_bgnd(sm_mont[dt_[1]],'@',0);  
		} */   	
    	}
  /*   	
else if(dt[0]=='B')
    	{
    	lcd_buffer[iii++]='Б';
    	lcd_buffer[iii++]='а';
    	lcd_buffer[iii++]='т';
 	if(dt[1]<9)
    		{
    		lcd_buffer[iii++]=0x31+dt[1];
    		lcd_buffer[iii++]=' ';
    		}
    	else if((dt[1]>=9)&&(dt[1]<99))
    		{
    		lcd_buffer[iii++]=0x30+((dt[1]+1)/10);
    		lcd_buffer[iii++]=0x30+((dt[1]+1)%10);
    		}
    	else 
    		{
    		lcd_buffer[iii++]=' ';
    		lcd_buffer[iii++]=' ';     		
    		} 
    	lcd_buffer[iii++]=' ';
    	}     	    
else if(dt[0]=='I')
    	{
    	lcd_buffer[iii++]='И';
    	lcd_buffer[iii++]='н';
    	lcd_buffer[iii++]='в';
	if(dt[1]<9)
    		{
    		lcd_buffer[iii++]=0x31+dt[1];
    		lcd_buffer[iii++]=' ';
    		}
    	else if((dt[1]>=9)&&(dt[1]<99))
    		{
    		lcd_buffer[iii++]=0x30+((dt[1]+1)/10);
    		lcd_buffer[iii++]=0x30+((dt[1]+1)%10);
    		}
    	else 
    		{
    		lcd_buffer[iii++]=' ';
    		lcd_buffer[iii++]=' ';     		
    		} 
    	lcd_buffer[iii++]=' ';
    	} */   
}

//-----------------------------------------------
void snmp_trap_send(char* str, signed short in0, signed short in1, signed short in2)
{
for(snmp_trap_send_i=0;snmp_trap_send_i<100;snmp_trap_send_i++)
	{
	snmp_spc_trap_message[snmp_trap_send_i]=0;
	}


obj[0] = 0;
snmp_trap_send_ii=1;
if(str!=0)
	{
	obj[0]++;
	for(snmp_trap_send_i=0;snmp_trap_send_i<100&&(str[snmp_trap_send_i]);snmp_trap_send_i++)
		{
		snmp_spc_trap_message[snmp_trap_send_i]=str[snmp_trap_send_i];
		}
	obj[snmp_trap_send_ii] = 7;
	snmp_trap_send_ii++;
	}
if(in0!=0xffff)
	{
	obj[0]++;
	snmp_spc_trap_value_0=in0;
	obj[snmp_trap_send_ii] = 8;
	snmp_trap_send_ii++;
	}
if(in1!=0xffff)
	{
	obj[0]++;
	snmp_spc_trap_value_1=in1;
	obj[snmp_trap_send_ii] = 9;
	snmp_trap_send_ii++;
	}
if(in2!=0xffff)
	{
	obj[0]++;
	snmp_spc_trap_value_2=in2;
	obj[snmp_trap_send_ii] = 10;
	snmp_trap_send_ii++;
	}


if((ETH_TRAP1_IP_1!=255)&&(ETH_TRAP1_IP_2!=255)&&(ETH_TRAP1_IP_3!=255)&&(ETH_TRAP1_IP_4!=255))
	{
	temp_ip[0]= ETH_TRAP1_IP_1;
	temp_ip[1]= ETH_TRAP1_IP_2;
	temp_ip[2]= ETH_TRAP1_IP_3;
	temp_ip[3]= ETH_TRAP1_IP_4;
	snmp_trap (temp_ip, 6, 3, obj);
	}
if((ETH_TRAP2_IP_1!=255)&&(ETH_TRAP2_IP_2!=255)&&(ETH_TRAP2_IP_3!=255)&&(ETH_TRAP2_IP_4!=255))
	{
	temp_ip[0]= ETH_TRAP2_IP_1;
	temp_ip[1]= ETH_TRAP2_IP_2;
	temp_ip[2]= ETH_TRAP2_IP_3;
	temp_ip[3]= ETH_TRAP2_IP_4;
	snmp_trap (temp_ip, 6, 3, obj);
	}

if((ETH_TRAP3_IP_1!=255)&&(ETH_TRAP3_IP_2!=255)&&(ETH_TRAP3_IP_3!=255)&&(ETH_TRAP3_IP_4!=255))
	{
	temp_ip[0]= ETH_TRAP3_IP_1;
	temp_ip[1]= ETH_TRAP3_IP_2;
	temp_ip[2]= ETH_TRAP3_IP_3;
	temp_ip[3]= ETH_TRAP3_IP_4;
	snmp_trap (temp_ip, 6, 3, obj);
	}

if((ETH_TRAP4_IP_1!=255)&&(ETH_TRAP4_IP_2!=255)&&(ETH_TRAP4_IP_3!=255)&&(ETH_TRAP4_IP_4!=255))
	{
	temp_ip[0]= ETH_TRAP4_IP_1;
	temp_ip[1]= ETH_TRAP4_IP_2;
	temp_ip[2]= ETH_TRAP4_IP_3;
	temp_ip[3]= ETH_TRAP4_IP_4;
	snmp_trap (temp_ip, 6, 3, obj);
	}

if((ETH_TRAP5_IP_1!=255)&&(ETH_TRAP5_IP_2!=255)&&(ETH_TRAP5_IP_3!=255)&&(ETH_TRAP5_IP_4!=255))
	{
	temp_ip[0]= ETH_TRAP5_IP_1;
	temp_ip[1]= ETH_TRAP5_IP_2;
	temp_ip[2]= ETH_TRAP5_IP_3;
	temp_ip[3]= ETH_TRAP5_IP_4;
	snmp_trap (temp_ip, 6, 3, obj);
	}			
}

//-----------------------------------------------
void snmp_klimat_settings_box_alarm_write(int mode)
{
if(mode==MIB_WRITE)
	{
     lc640_write_int(EE_TBOXMAX,snmp_klimat_settings_box_alarm);
	}
}
//-----------------------------------------------
void snmp_klimat_settings_vent_on_write(int mode)
{
if(mode==MIB_WRITE)
	{
     lc640_write_int(EE_TBOXVENTON,snmp_klimat_settings_vent_on);
	}
}
//-----------------------------------------------
void snmp_klimat_settings_vent_off_write(int mode)
{
if(mode==MIB_WRITE)
	{
     lc640_write_int(EE_TBOXVENTOFF,snmp_klimat_settings_vent_off);
	}
}
//-----------------------------------------------
void snmp_klimat_settings_warm_on_write(int mode)
{
if(mode==MIB_WRITE)
	{
     lc640_write_int(EE_TBOXWARMON,snmp_klimat_settings_warm_on);
	}
}
//-----------------------------------------------
void snmp_klimat_settings_warm_off_write(int mode)
{
if(mode==MIB_WRITE)
	{
     lc640_write_int(EE_TBOXWARMOFF,snmp_klimat_settings_warm_off);
	}
}
//-----------------------------------------------
void snmp_klimat_settings_load_on_write(int mode)
{
if(mode==MIB_WRITE)
	{
     lc640_write_int(EE_TLOADENABLE,snmp_klimat_settings_load_on);
	}
}
//-----------------------------------------------
void snmp_klimat_settings_load_off_write(int mode)
{
if(mode==MIB_WRITE)
	{
     lc640_write_int(EE_TLOADDISABLE,snmp_klimat_settings_load_off);
	}
}
//-----------------------------------------------
void snmp_klimat_settings_batt_on_write(int mode)
{
if(mode==MIB_WRITE)
	{
     lc640_write_int(EE_TBATENABLE,snmp_klimat_settings_batt_on);
	}
}
//-----------------------------------------------
void snmp_klimat_settings_batt_off_write(int mode)
{
if(mode==MIB_WRITE)
	{
     lc640_write_int(EE_TBATDISABLE,snmp_klimat_settings_batt_off);
	}
}


 
