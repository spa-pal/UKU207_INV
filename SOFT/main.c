//Ветка для MCP2515
//
//

#include "lcd_AGM1232_uku207_3.h"
#include "rtl.h"
#include "type.h"
#include "main.h"
#include "simbol.h"
#include "25lc640.h"
#include "Timer.h"
#include "gran.h"
#include "uart0.h"
#include "uart1.h"
#include "cmd.h"
#include "ret.h"
#include "eeprom_map.h"
#include "common_func.h"
#include "control.h"
#include "mess.h"
#include "full_can.h"
#include "watchdog.h"
#include "ad7705.h"
#include "beep.h"
#include "avar_hndl.h"
#include "memo.h"
#include "simbols.h"
#include "graphic.h"
#include "snmp_data_file.h" 
#include "net_config.h"
#include "uart0.h"
#include <rtl.h>
#include "mcp2515.h"
#include "sc16is7xx.h"
#include "modbus.h"
#include "curr_version.h"
#include "sntp.h"
#include "modbus_tcp.h"

extern U8 own_hw_adr[];
extern U8  snmp_Community[];
BOOL tick;
extern LOCALM localm[];
#define MY_IP localm[NETIF_ETH].IpAdr
#define DHCP_TOUT   50

//***********************************************
//Таймер
char b10000Hz,b1000Hz,b2000Hz,b100Hz,b50Hz,b10Hz,b5Hz,b2Hz,b1Hz,b1min;
short t0cnt,t0cnt0,t0cnt1,t0cnt2,t0cnt3,t0cnt4,t0cnt5,t0cnt6,t0_cnt7,t0_cnt_min;
char bFL5,bFL2,bFL,bFL_,bTPS;
signed short main_10Hz_cnt=0;
signed short main_1Hz_cnt=0;
signed short kan_aktivity_cnt;

 
//***********************************************
//Структура ИБЭПа
char cnt_of_slave=3;
//char cnt_of_wrks;   //колличество работающих источников , для индикации



//**********************************************
//Коэффициенты, отображаемые из EEPROM
signed short Ktsrc[2];
signed short Kusrc[2];
signed short Kisrc[2];
signed short Ki0src[2];
signed short Kubat[2];
signed short Kubatm[2];
unsigned short Kibat0[2];
signed short Kibat1[2];
signed short Ktbat[2];
signed short Kunet;
signed short Kunet_ext[3];
signed short Ktext[3];
signed short Kuload;
signed short KunetA;
signed short KunetB;
signed short KunetC;
signed short Kudcin;

signed short MAIN_IST;
signed short UMAX;
signed short UB0;
signed short UB20;
signed short TMAX;
signed short TSIGN;
signed short AV_OFF_AVT;
signed short USIGN;
signed short UMN;
signed short ZV_ON;
signed short IKB;
//signed short KVZ;
signed short UVZ;
signed short IMAX;
signed short IMIN;
signed short APV_ON;
signed short IZMAX;
signed short U0B;
signed short TZAS;
signed short VZ_HR;
signed short TBAT;
signed short U_AVT;
signed short DU;
signed short PAR;
signed short TBATMAX;
signed short TBATSIGN;
signed short UBM_AV;
signed short TBOXMAX;
signed short TBOXREG;
signed short TBOXVENTMAX;
signed short TLOADDISABLE;
signed short TLOADENABLE;
signed short TBATDISABLE;
signed short TBATENABLE;
signed short TVENTON;
signed short TVENTOFF;
enum_releventsign RELEVENTSIGN;
signed short TZNPN;
signed short UONPN;
signed short UVNPN;
enum_npn_out NPN_OUT;
enum_npn_sign NPN_SIGN;
signed short TERMOKOMPENS;
signed short TBOXVENTON; 
signed short TBOXVENTOFF;
signed short TBOXWARMON; 
signed short TBOXWARMOFF;

signed short U_OUT_SET;
signed short U_NET_MAX;
signed short U_OUT_MAX;
signed short U_OUT_MIN;
signed short U_NET_MIN;
signed short U_BAT_MAX;
signed short U_BAT_MIN;


signed short U_OUT_AC_MAX_AV;
signed short U_OUT_AC_MIN_AV;
signed short U_IN_AC_MAX_AV;
signed short U_IN_AC_MIN_AV;
signed short U_IN_DC_MAX_AV;
signed short U_IN_DC_MIN_AV;

signed short NUMBAT;
signed short NUMIST;
signed short NUMINV;
signed short NUMDT;
signed short NUMSK;
signed short NUMEXT;
signed short NUMAVT;
signed short NUMMAKB;
signed short NUMBYPASS;
signed short NUMPHASE;
signed short NUMINAC;
	

enum_apv_on APV_ON1,APV_ON2;
signed short APV_ON2_TIME;

enum_bat_is_on BAT_IS_ON[2];
signed short BAT_DAY_OF_ON[2];
signed short BAT_MONTH_OF_ON[2];
signed short BAT_YEAR_OF_ON[2];
signed short BAT_C_NOM[2];
signed short BAT_RESURS[2];
signed short BAT_C_REAL[2];

unsigned short AUSW_MAIN;
unsigned long 	AUSW_MAIN_NUMBER;
unsigned short AUSW_DAY;
unsigned short AUSW_MONTH;
unsigned short AUSW_YEAR;
unsigned short AUSW_UKU;
unsigned short AUSW_UKU_SUB;
unsigned long AUSW_UKU_NUMBER;
unsigned long	AUSW_BPS1_NUMBER;
unsigned long  AUSW_BPS2_NUMBER;
unsigned short AUSW_RS232;
unsigned short AUSW_PDH;
unsigned short AUSW_SDH;
unsigned short AUSW_ETH;

signed short TMAX_EXT_EN[3];
signed short TMAX_EXT[3];
signed short TMIN_EXT_EN[3];
signed short TMIN_EXT[3];
signed short T_EXT_REL_EN[3];
signed short T_EXT_ZVUK_EN[3];
signed short T_EXT_LCD_EN[3];
signed short T_EXT_RS_EN[3];

signed short SK_SIGN[4];
signed short SK_REL_EN[4];
signed short SK_ZVUK_EN[4];
signed short SK_LCD_EN[4];
signed short SK_RS_EN[4];

enum_avz AVZ;

unsigned short HOUR_AVZ;
unsigned short MIN_AVZ;
unsigned short SEC_AVZ;
unsigned short DATE_AVZ;
unsigned short MONTH_AVZ;
unsigned short YEAR_AVZ;
unsigned short AVZ_TIME;

enum_mnemo_on MNEMO_ON;
unsigned short MNEMO_TIME;

signed short POWER_CNT_ADRESS;

signed short ETH_IS_ON;
signed short ETH_DHCP_ON;
signed short ETH_IP_1;
signed short ETH_IP_2;
signed short ETH_IP_3;
signed short ETH_IP_4;
signed short ETH_MASK_1;
signed short ETH_MASK_2;
signed short ETH_MASK_3;
signed short ETH_MASK_4;
signed short ETH_TRAP1_IP_1;
signed short ETH_TRAP1_IP_2;
signed short ETH_TRAP1_IP_3;
signed short ETH_TRAP1_IP_4;
signed short ETH_TRAP2_IP_1;
signed short ETH_TRAP2_IP_2;
signed short ETH_TRAP2_IP_3;
signed short ETH_TRAP2_IP_4;
signed short ETH_TRAP3_IP_1;
signed short ETH_TRAP3_IP_2;
signed short ETH_TRAP3_IP_3;
signed short ETH_TRAP3_IP_4;
signed short ETH_TRAP4_IP_1;
signed short ETH_TRAP4_IP_2;
signed short ETH_TRAP4_IP_3;
signed short ETH_TRAP4_IP_4;
signed short ETH_TRAP5_IP_1;
signed short ETH_TRAP5_IP_2;
signed short ETH_TRAP5_IP_3;
signed short ETH_TRAP5_IP_4;

signed short ETH_SNMP_PORT_READ;
signed short ETH_SNMP_PORT_WRITE;

signed short ETH_GW_1;
signed short ETH_GW_2;
signed short ETH_GW_3;
signed short ETH_GW_4;

signed short RELE_VENT_LOGIC;
signed short MODBUS_ADRESS;
signed short MODBUS_BAUDRATE;
signed short RS485_QWARZ_DIGIT;

signed short SNTP_ENABLE;
signed short SNTP_GMT;


//***********************************************
//Состояние батарей
BAT_STAT bat[2],bat_ips;
signed short		bat_u_old_cnt;
signed short 		Ib_ips_termokompensat;

/*
//***********************************************
//Мониторы АКБ
MAKB_STAT makb[4];
*/

//***********************************************
//Телеметрия по внутренней шине
//char can_slot[12][16];

//***********************************************
//Состояние источников
BPS_STAT bps[64];

//***********************************************
//Состояние инверторов
INV_STAT inv[64];
char first_inv_slot=MINIM_INV_ADRESS;


//***********************************************
//Состояние байпаса
BYPS_STAT byps[3];

//***********************************************
//Состояние нагрузки
signed short load_U;
signed short load_I;
signed long load_P;
signed short load_U_inv;
signed short load_I_inv;
signed long load_P_inv;

signed short dcin_U;

//***********************************************
//Индикация

char lcd_buffer[LCD_SIZE+100]={"Hello World"};
signed char parol[3];
char phase;
char lcd_bitmap[1024];
char dig[5];
char dumm_ind[20];
stuct_ind a_ind,b_ind[10],c_ind;
char dumm_ind_[20];
char zero_on;
char mnemo_cnt=50;
char simax;
short av_j_si_max;
const char ABCDEF[]={"0123456789ABCDEF"};
const char sm_mont[13][4]={"   ","янв","фев","мар","апр","май","июн","июл","авг","сен","окт","ноя","дек"}; //
signed short ptr_ind=0;

signed short ind_pointer=0;

//***********************************************
//Состояние первичной сети
signed short net_U,net_Ustore,net_Ua,net_Ub,net_Uc;
char bFF,bFF_;
signed short net_F,hz_out,hz_out_cnt,net_F3;
signed char unet_drv_cnt;
char net_av;

//***********************************************
//Состояние внешних датчиков
//signed short tout[4];
char tout_max_cnt[4],tout_min_cnt[4];
enum_tout_stat tout_stat[4];
signed short t_ext[3];
BOOL ND_EXT[3];
signed char sk_cnt_dumm[4],sk_cnt[4],sk_av_cnt[4];
enum_sk_stat sk_stat[4]={ssOFF,ssOFF,ssOFF,ssOFF};
enum_sk_av_stat sk_av_stat[4]={sasOFF,sasOFF,sasOFF,sasOFF},sk_av_stat_old[4];
short sk_spec_reg;
signed short t_box;

//***********************************************
//Звуки
extern char beep_cnt;
BOOL bSILENT;








signed short u_necc,u_necc_,u_necc_up,u_necc_dn;
signed short main_cnt_5Hz;
signed short num_necc;
signed short num_necc_Imax;
signed short num_necc_Imin;
signed short cnt_num_necc;
//char bSAME_IST_ON;
signed mat_temper;

//***********************************************
//АПВ
unsigned main_apv_cnt,hour_apv_cnt[2],reset_apv_cnt[2];
unsigned short apv_cnt_sec[2],apv_cnt[2];

//***********************************************
//Текстовые константы
const char sm_[]	={"                    "};
const char sm_exit[]={" Выход              "};
const char sm_time[]={" 0%:0^:0& 0</>  /0{ "};





//**********************************************
//Работа с кнопками 
char but;                            
unsigned long but_n,but_s;
char but0_cnt;
char but1_cnt;
char but_onL_temp;

//***********************************************
//Межблоковая связь
char cnt_net_drv;

//***********************************************
//КАН 
extern short ptr_can1_tx_wr,ptr_can1_tx_rd;
extern char ptr_can2_tx_wr,ptr_can2_tx_rd;
extern unsigned short rotor_can[6];
extern char RXBUFF[40],TXBUFF[40];





//***********************************************
//Работа с кнопками
char speed,l_but,n_but;

//***********************************************
//Неразобранное
enum {wrkON=0x55,wrkOFF=0xAA}wrk;
char cnt_wrk;
signed short ibat_integr;
unsigned short av_beep,av_rele,av_stat;
char default_temp;
char ND_out[3];

//***********************************************
//Тест
enum_tst_state tst_state[15];

//***********************************************
//АЦП
//extern short adc_buff[16][16],adc_buff_[16];
extern char adc_cnt,adc_cnt1,adc_ch;

//***********************************************

char flag=0;


extern signed short bat_ver_cnt;
signed short Isumm;
signed short Isumm_;

#include <LPC17xx.H>                        /* LPC21xx definitions */



/*
extern void lcd_init(void);
extern void lcd_on(void);
extern void lcd_clear(void);
*/

extern short plazma_adc_cnt;
extern char net_buff_cnt;
extern unsigned short net_buff[32],net_buff_;
extern char rele_stat/*,rele_stat_*/;
extern char bRXIN0;


char cntrl_plazma;
extern char bOUT_FREE2;
extern char /*av_net,*//*av_bat[2],*/av_bps[12],av_inv[6],av_dt[4],av_sk[4];

char content[63];

//const short ptr_bat_zar_cnt[2]={EE_ZAR1_CNT,EE_ZAR2_CNT};


//unsigned short YEAR_AVZ,MONTH_AVZ,DATE_AVZ,HOUR_AVZ,MIN_AVZ,SEC_AVZ;


//**********************************************
//Самокалиброввка
extern signed short samokalibr_cnt;

//**********************************************
//Сообщения
extern char mess[MESS_DEEP],mess_old[MESS_DEEP],mess_cnt[MESS_DEEP];
extern short mess_par0[MESS_DEEP],mess_par1[MESS_DEEP],mess_data[2];


//**********************************************
//Контроль наличия батарей
extern signed short 	main_kb_cnt;
extern signed short 	kb_cnt_1lev;
extern signed short 	kb_cnt_2lev;
extern char 			kb_full_ver;
extern char 			kb_start[2],kb_start_ips;

//***********************************************
//Управление ШИМом
extern signed short cntrl_stat;
extern signed short cntrl_stat_old;
extern signed short cntrl_stat_new;
extern signed short Ibmax;


//-----------------------------------------------
//Контроль заряда
char sign_U[2],sign_I[2];
char superviser_cnt;


char plazma_plazma_plazma;

char bRESET=0;
char bRESET_EXT=0;
char ext_can_cnt;
//-----------------------------------------------
//Состояние вводов
signed short vvod_pos;

//-----------------------------------------------
//Плата расширения
unsigned short adc_buff_ext_[3];
unsigned short Uvv[3];
unsigned short Uvv0;
short pos_vent;
short t_ext_can;
char t_ext_can_nd;


//-----------------------------------------------
//Плата расширения 2
char eb2_data[30];
short eb2_data_short[10];
short Uvv_eb2[3],Upes_eb2[3];
short Kvv_eb2[3],Kpes_eb2[3];
//-----------------------------------------------
//Работа со щетчиком
signed long power_summary;
signed short power_current;

//-----------------------------------------------
//Климатконтроль и вентиляторы
signed short main_vent_pos;
signed char t_box_cnt=0;
enum_mixer_vent_stat mixer_vent_stat=mvsOFF;
INT_BOX_TEMPER ibt;
enum_tbatdisable_stat tbatdisable_stat=tbdsON;
enum_tloaddisable_stat tloaddisable_stat=tldsON;
enum_av_tbox_stat av_tbox_stat=atsOFF;
signed short av_tbox_cnt;
char tbatdisable_cmnd=20,tloaddisable_cmnd=22;
short tbatdisable_cnt,tloaddisable_cnt;

//-----------------------------------------------
//Состояние контролируемых автоматов нагрузки 
enum_avt_stat avt_stat[12],avt_stat_old[12];

//short sys_plazma,sys_plazma1;

char snmp_plazma;
char plazma_bypas;

short plazma_but_an;

char bCAN_OFF;


char max_net_slot;

//-----------------------------------------------
//Показания АЦП на плате измерения тока батареи
signed long ibat_metr_buff_[2];
short bIBAT_SMKLBR;

//-----------------------------------------------
//Управление низкоприоритетной нагрузкой
signed short npn_tz_cnt;
enum_npn_stat npn_stat=npnsON;


char ips_bat_av_vzvod=0;
char ips_bat_av_stat=0;

char rel_warm_plazma;
char can_byps_plazma0,can_byps_plazma1;

short reset_plazma;
char plazma_rx;

//-----------------------------------------------
//Управление реле
signed short RELE_SET_MASK[2]={1,2};
char avar_stat_temp[2];

//-----------------------------------------------
//Флаги состояния системы
char someInvAvIsOn;		//авария какого-либо инвертора
char dcAvIsOn; 			//авария по входному напряжению DC
char uOutAvIsOn;		//авария по выходному напряжению
char uNetAvIsOn;		//авария по входному напряжению AC
char wrkFromNet1Inv0; 	//работа от сети(1)/инверторов(0)


U8 socket_tcp;
U8 tcp_soc_avg;
U8 tcp_connect_stat;


signed short f_out;

//-----------------------------------------------
void rtc_init (void) 
{
LPC_RTC->CCR=0x11;
}

//-----------------------------------------------
static void timer_poll () 
{
if (SysTick->CTRL & 0x10000) 
     {
     timer_tick ();
     tick = __TRUE;
     }
}

//-----------------------------------------------
void inv_search(void)
{
char i;

first_inv_slot=8;
for(i=0;i<12;i++)
	{
	if(bps[i]._device==dINV)
		{
		first_inv_slot=i;
		break;
		}
	}
}

//-----------------------------------------------
signed short abs_pal(signed short in)
{
if(in<0)return -in;
else return in;
}

//-----------------------------------------------
void init_ETH(void)
{
localm[NETIF_ETH].IpAdr[0]=lc640_read_int(EE_ETH_IP_1);
localm[NETIF_ETH].IpAdr[1]=lc640_read_int(EE_ETH_IP_2);
localm[NETIF_ETH].IpAdr[2]=lc640_read_int(EE_ETH_IP_3);
localm[NETIF_ETH].IpAdr[3]=lc640_read_int(EE_ETH_IP_4);

localm[NETIF_ETH].NetMask[0]=lc640_read_int(EE_ETH_MASK_1);
localm[NETIF_ETH].NetMask[1]=lc640_read_int(EE_ETH_MASK_2);
localm[NETIF_ETH].NetMask[2]=lc640_read_int(EE_ETH_MASK_3);
localm[NETIF_ETH].NetMask[3]=lc640_read_int(EE_ETH_MASK_4);

localm[NETIF_ETH].DefGW[0]=lc640_read_int(EE_ETH_GW_1);
localm[NETIF_ETH].DefGW[1]=lc640_read_int(EE_ETH_GW_2);
localm[NETIF_ETH].DefGW[2]=lc640_read_int(EE_ETH_GW_3);
localm[NETIF_ETH].DefGW[3]=lc640_read_int(EE_ETH_GW_4);

}


//-----------------------------------------------
void ADC_IRQHandler(void) {
LPC_ADC->ADCR &=  ~(7<<24);



adc_self_ch_buff[adc_self_ch_cnt]=(LPC_ADC->ADGDR>>4) & 0xFFF;/* Read Conversion Result             */
adc_self_ch_cnt++;
if(adc_self_ch_cnt<3)
	{
	LPC_ADC->ADCR |=  (1<<24);
	}
else
	{

 
	//SET_REG(LPC_ADC->ADCR,1,24,3);
	}

/*			adc_buff_[0]=AD_last;
			if(AD_last<adc_buff_min[adc_ch])adc_buff_min[adc_ch]=AD_last;
			if(AD_last>adc_buff_max[adc_ch])adc_buff_max[adc_ch]=AD_last;*/
}

//-----------------------------------------------
void def_set(int umax__,int ub0__,int ub20__,int usign__,int imax__,int uob__,int numi,int _uvz)
{
;
lc640_write_int(EE_NUMIST,numi);
lc640_write_int(EE_NUMINV,0);
//lc640_write_int(EE_NUMDT,0);
//lc640_write_int(EE_NUMSK,0);
lc640_write_int(EE_MAIN_IST,0);
lc640_write_int(EE_PAR,1);
lc640_write_int(EE_TBAT,60);
lc640_write_int(EE_UMAX,umax__);
lc640_write_int(EE_DU,ub20__/2);
lc640_write_int(EE_UB0,ub0__);
lc640_write_int(EE_UB20,ub20__);
lc640_write_int(EE_TSIGN,70);
lc640_write_int(EE_TMAX,80);
//lc640_write_int(EE_C_BAT,180);
lc640_write_int(EE_USIGN,usign__);
lc640_write_int(EE_UMN,187);
lc640_write_int(EE_ZV_ON,0);
lc640_write_int(EE_IKB,10);
//lc640_write_int(EE_KVZ,1030);
lc640_write_int(EE_UVZ,_uvz);
lc640_write_int(EE_IMAX,imax__);
lc640_write_int(EE_IMIN,(imax__*8)/10);
//lc640_write_int(EE_APV_ON,apvON);
lc640_write_int(EE_APV_ON1,apvON);
lc640_write_int(EE_APV_ON2,apvON);
lc640_write_int(EE_APV_ON2_TIME,1);
lc640_write_int(EE_IZMAX,160);
lc640_write_int(EE_U0B,uob__);
lc640_write_int(EE_TZAS,3);
lc640_write_int(EE_TBATMAX,50);  
lc640_write_int(EE_TBATSIGN,40);
lc640_write_int(EE_MNEMO_ON,mnON);
lc640_write_int(EE_MNEMO_TIME,30);	
lc640_write_int(EE_AV_OFF_AVT,1);
//lc640_write_int(EE_APV_ON1,apvOFF);



lc640_write_int(EE_TBOXMAX,70);
lc640_write_int(EE_TBOXVENTMAX,60);
lc640_write_int(EE_TBOXREG,25);
lc640_write_int(EE_TLOADDISABLE,80);
lc640_write_int(EE_TLOADENABLE,70);
lc640_write_int(EE_TBATDISABLE,91);
lc640_write_int(EE_TBATENABLE,80);

lc640_write_int(ADR_SK_SIGN[0],0);
lc640_write_int(ADR_SK_REL_EN[0],0);
lc640_write_int(ADR_SK_LCD_EN[0],0xffff);

lc640_write_int(ADR_SK_SIGN[1],0);
lc640_write_int(ADR_SK_REL_EN[1],0);
lc640_write_int(ADR_SK_LCD_EN[1],0xffff);

lc640_write_int(ADR_SK_SIGN[2],0);
lc640_write_int(ADR_SK_REL_EN[2],0);
lc640_write_int(ADR_SK_LCD_EN[2],0xffff);

lc640_write_int(ADR_SK_SIGN[3],0);
lc640_write_int(ADR_SK_REL_EN[3],0);
lc640_write_int(ADR_SK_LCD_EN[3],0xffff);

lc640_write_int(EE_UBM_AV,10);

lc640_write_int(EE_POS_VENT,11);
}


//-----------------------------------------------
void def_ips_set(short voltage)
{
if(voltage==24)
	{
	def_set(300,voltage,voltage,22,150,240,7,0);
	}
if(voltage==48)
	{
	def_set(600,voltage,voltage,44,100,480,7,0);
	}
if(voltage==60)
	{
	def_set(750,voltage,voltage,55,100,600,7,0);
	}

if(voltage==220)
	{
	def_set(2450,2366,2315,187,100,2200,2,2346);
	lc640_write_int(EE_DU,2315-1870);
	lc640_write_int(EE_U_AVT,2200);
	lc640_write_int(EE_IZMAX,20);
	lc640_write_int(EE_AUSW_MAIN,22033);
	lc640_write_int(EE_PAR,1);
	lc640_write_int(EE_MNEMO_ON,mnOFF);
	}

lc640_write_int(ADR_EE_BAT_IS_ON[0],bisOFF);
lc640_write_int(ADR_EE_BAT_DAY_OF_ON[0],LPC_RTC->DOM);
lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[0],LPC_RTC->MONTH);
lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[0],LPC_RTC->YEAR);
lc640_write_int(ADR_EE_BAT_C_NOM[0],0);
lc640_write_int(ADR_EE_BAT_RESURS[0],0);

lc640_write_int(ADR_EE_BAT_IS_ON[1],bisOFF);
lc640_write_int(ADR_EE_BAT_DAY_OF_ON[1],LPC_RTC->DOM);
lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[1],LPC_RTC->MONTH);
lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[1],LPC_RTC->YEAR);
lc640_write_int(ADR_EE_BAT_C_NOM[1],0);
lc640_write_int(ADR_EE_BAT_RESURS[1],0);
}

//-----------------------------------------------
void can_reset_hndl(void)
{
if((lc640_read_int(EE_CAN_RESET_CNT)<0)||(lc640_read_int(EE_CAN_RESET_CNT)>2))	lc640_write_int(EE_CAN_RESET_CNT,0);

can_reset_cnt++;

if((can_reset_cnt>=10)&&(!(avar_stat&0x0001))&&(!bRESET))
	{
	if(lc640_read_int(EE_CAN_RESET_CNT)<2)
		{
		lc640_write_int(EE_CAN_RESET_CNT,lc640_read_int(EE_CAN_RESET_CNT)+1);
		bRESET=1;
		}
	}

if((main_1Hz_cnt>=3600UL)&&(lc640_read_int(EE_CAN_RESET_CNT)!=0))
	{
	lc640_write_int(EE_CAN_RESET_CNT,0);
	}

if(((LPC_CAN1->GSR)>>24)==127)bRESET=1;
if((((LPC_CAN1->GSR)>>16)&0x00ff)==127)bRESET=1;

}

//-----------------------------------------------
void net_drv(void)
{ 
//char temp_;    



max_net_slot=MINIM_INV_ADRESS+NUMINV+8;
//if(NUMINV) max_net_slot=MINIM_INV_ADRESS+NUMINV;
//gran_char(&max_net_slot,0,MAX_NET_ADRESS);

if(++cnt_net_drv>max_net_slot) 
	{
	cnt_net_drv=MINIM_INV_ADRESS;
	//LPC_GPIO2->FIODIR|=(1UL<<7);
	//LPC_GPIO2->FIOPIN^=(1UL<<7);
	} 


	
	
	
if((cnt_net_drv>=MINIM_INV_ADRESS)&&(cnt_net_drv<MINIM_INV_ADRESS+NUMINV))
	{
	//if((!bCAN_OFF)&&(cnt_net_drv!=4))can1_out(cnt_net_drv,cnt_net_drv,GETTM,bps[cnt_net_drv]._flags_tu,*((char*)(&bps[cnt_net_drv]._vol_u)),*((char*)((&bps[cnt_net_drv]._vol_u))+1),*((char*)(&bps[cnt_net_drv]._vol_i)),*((char*)((&bps[cnt_net_drv]._vol_i))+1));
     
	//if(cnt_net_drv<=11)
	     {
	     if(bps[cnt_net_drv]._cnt<CNT_SRC_MAX)
   	 		{    
   	 		bps[cnt_net_drv]._cnt++;
   	 	/*	if( (bps[cnt_net_drv]._cnt>=CNT_SRC_MAX) && (!net_av) && (!(bps[cnt_net_drv]._av&0x08)) && (cnt_net_drv<NUMIST) ) 
   	 			{
   	 			avar_bps_hndl(cnt_net_drv,3,1);
   	 			}*/
   	 		}
		else bps[cnt_net_drv]._cnt=CNT_SRC_MAX;
						
		if((bps[cnt_net_drv]._cnt>=3)&&(bps[cnt_net_drv]._cnt_old<3))bps[cnt_net_drv]._cnt_more2++;
		bps[cnt_net_drv]._cnt_old=bps[cnt_net_drv]._cnt;
	     }
	}


/*

else if((cnt_net_drv==MINIM_INV_ADRESS+2)&&(NUMINV))
	{
    if(!bCAN_OFF) can1_out(cnt_net_drv,cnt_net_drv,0,0,0,0,0,0);
	}*/


}


//-----------------------------------------------
void net_drv_mcp2515(void)
{ 
//char temp_;    



max_net_slot=MINIM_INV_ADRESS+NUMINV+8;
//if(NUMINV) max_net_slot=MINIM_INV_ADRESS+NUMINV;
//gran_char(&max_net_slot,0,MAX_NET_ADRESS);

if((++cnt_net_drv>50)&&(kan_aktivity_cnt))
	{
	cnt_net_drv=0;
	mcp2515_transmit(0xf1,(char)U_OUT_SET,(char)U_OUT_MIN,(char)(U_OUT_MAX-50),(char)U_NET_MIN,(char)U_NET_MAX,(char)U_BAT_MIN,(char)U_BAT_MAX);
	} 
	
if((cnt_net_drv>=MINIM_INV_ADRESS)&&(cnt_net_drv<MINIM_INV_ADRESS+NUMINV))
	{
	if((!bCAN_OFF)&&(kan_aktivity_cnt))mcp2515_transmit(cnt_net_drv,cnt_net_drv,GETTM,bps[cnt_net_drv]._flags_tu,*((char*)(&bps[cnt_net_drv]._vol_u)),*((char*)((&bps[cnt_net_drv]._vol_u))+1),*((char*)(&bps[cnt_net_drv]._vol_i)),*((char*)((&bps[cnt_net_drv]._vol_i))+1));
     
	//if(cnt_net_drv<=11)
	     {
	     if(bps[cnt_net_drv]._cnt<CNT_SRC_MAX)
   	 		{    
   	 		bps[cnt_net_drv]._cnt++;
   	 	/*	if( (bps[cnt_net_drv]._cnt>=CNT_SRC_MAX) && (!net_av) && (!(bps[cnt_net_drv]._av&0x08)) && (cnt_net_drv<NUMIST) ) 
   	 			{
   	 			avar_bps_hndl(cnt_net_drv,3,1);
   	 			}*/
   	 		}
		else bps[cnt_net_drv]._cnt=CNT_SRC_MAX;
						
		if((bps[cnt_net_drv]._cnt>=3)&&(bps[cnt_net_drv]._cnt_old<3))bps[cnt_net_drv]._cnt_more2++;
		bps[cnt_net_drv]._cnt_old=bps[cnt_net_drv]._cnt;
	     }
	}


	if(NUMBYPASS)
		{
		if(cnt_net_drv==MINIM_INV_ADRESS)
			{
			if(byps[0]._cnt<CNT_SRC_MAX)byps[0]._cnt++;
			if(byps[1]._cnt<CNT_SRC_MAX)byps[1]._cnt++;
			if(byps[2]._cnt<CNT_SRC_MAX)byps[2]._cnt++;
			}
		}
/*	if(NUMBYPASS>=2)
		{
		if(cnt_net_drv==MINIM_INV_ADRESS)
			{
			if(byps[1]._cnt<CNT_SRC_MAX)byps[1]._cnt++;
			}
		}
	if(NUMBYPASS>=3)
		{
		if(cnt_net_drv==MINIM_INV_ADRESS)
			{
			if(byps[2]._cnt<CNT_SRC_MAX)byps[2]._cnt++;
			}
		}  */

/*

else if((cnt_net_drv==MINIM_INV_ADRESS+2)&&(NUMINV))
	{
    if(!bCAN_OFF) can1_out(cnt_net_drv,cnt_net_drv,0,0,0,0,0,0);
	}*/




}



//-----------------------------------------------
void parol_init(void)
{
parol[0]=0;
parol[1]=0;
parol[2]=0;
sub_ind=0;
}

//-----------------------------------------------
void bitmap_hndl(void)
{
short x,ii,i;
unsigned int ptr_bitmap;
static char ptr_cnt,ptr_cnt1,ptr_cnt2,ptr_cnt3,ptr_cnt4;

for(ii=0;ii<488;ii++)
	{
	lcd_bitmap[ii]=0x00;
	}

if((!mnemo_cnt)&&((NUMBAT==0)||((NUMBAT==1)&&(BAT_IS_ON[0]==bisON))))
	{
	if(avar_stat&0x0001)
		{
		if(bFL2)
			{
			graphic_print(3,3,50,24,50,3,sAVNET,0);
			graphic_print(3,3,50,24,50,3,sAVNET1,0);
			}
		}
	else
		{

		if(NUMIST>=1)
			{
/*

	if(bps[sub_ind1]._state==bsWRK)
		{
		ptr[0]=		"      в работе      ";
		if((bps[sub_ind1]._flags_tm&0x08)&&(bFL2))
		ptr[0]=		"  СИЛЬНЫЙ НАГРЕВ!!! ";	      
		}
 	 else if(bps[sub_ind1]._state==bsRDY)
	 	{
		ptr[0]=		"      в резерве     ";	
		}

 	 else if(bps[sub_ind1]._state==bsBL)
	 	{
		ptr[0]=		" заблокирован извне ";	
		}

	 else if(bps[sub_ind1]._state==bsAPV)
	 	{
		ptr[0]=		"    Работает АПВ    ";
		}
	 
	 else if(bps[sub_ind1]._state==bsAV)
	 	{
		if(bps[sub_ind1]._av&(1<<0))
		ptr[0]=		" Авария - перегрев! ";
		else if(bps[sub_ind1]._av&(1<<1))
		ptr[0]=		"Авария - завыш.Uвых!";
		else if(bps[sub_ind1]._av&(1<<2))	 
		ptr[0]=		"Авария - заниж.Uвых!";
		else if(bps[sub_ind1]._av&(1<<3))
			{
			ptr[0]=	"  Авария - потеряна ";
			ptr[1]=	"      связь!!!      ";
			ptr[2]=	"                    ";
			simax=0;
			}
		}

	 else if(bps[sub_ind1]._state==bsOFF_AV_NET)
	 	{
		ptr[0]=		"      ВЫКЛЮЧЕН      ";
		ptr[1]=		"     Отсутствует    ";
		ptr[2]=		" первичное питание! ";
		simax=0;
		}

	bgnd_par(			"       БПС N&       ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;

	if(sub_ind>=4)	pointer_set(1);*/


			draw_rectangle(0,0,20,20,0,0);
			draw_rectangle(1,1,18,18,0,0);
			if(bps[0]._state!=bsAV)
				{
				graphic_print(3,2,15,15,15,2,sBPS1,0);
				}
			else if(bps[0]._av&(1<<0))
				{
				if(bFL2)graphic_print(3,2,15,15,15,2,sAVT,0);
				}
			else if( (bps[0]._av&(1<<1)) || (bps[0]._av&(1<<2)))
				{
				if(bFL2)graphic_print(2,2,15,15,15,2,sAVU,0);
				}	
			
			if(bps[0]._state==bsWRK)
				{
				draw(9,20,0,11,0);
				draw(9,31,91,0,0);
				draw_ptr(9,19+ptr_cnt1,0,4);
				}				
			}
		if(NUMIST>=2)
			{
			draw_rectangle(23,0,20,20,0,0);
			draw_rectangle(24,1,18,18,0,0);
			if(bps[1]._state!=bsAV)
				{
				graphic_print(25,2,15,15,15,2,sBPS2,0);
				}
			else if(bps[1]._av&(1<<0))
				{
				if(bFL2)graphic_print(25,2,15,15,15,2,sAVT,0);
				}
			else if( (bps[1]._av&(1<<1)) || (bps[1]._av&(1<<2)))
				{
				if(bFL2)graphic_print(25,2,15,15,15,2,sAVU,0);
				}	
			
			if(bps[1]._state==bsWRK)
				{
				draw(32,20,0,11,0);
				draw(32,31,68,0,0);
				draw_ptr(32,19+ptr_cnt1,0,4);
				}				
			}			
		}
	if(NUMBAT)
		{
		draw_rectangle(50,0,35,20,0,0);
		draw_rectangle(53,20,3,2,0,0);
		draw_rectangle(79,20,3,2,0,0);
		if(bat[0]._av&0x01)
			{
			if(bFL2)graphic_print(43,0,50,24,50,3,sAVNET1,0);
			}
		else 
			{
			draw(66,20,0,11,0);
			draw(66,31,34,0,0);
			if(bat[0]._Ib<0)draw_ptr(66,19+ptr_cnt1,0,4);
			else if(bat[0]._Ib>=0)draw_ptr(66,34-ptr_cnt1,2,4);
			
			if(ptr_cnt4<15)
				{
				if(BAT_C_REAL[0]!=0x5555)
					{
					signed short u;
					u=(((signed short)bat[0]._zar/5));
					gran(&u,0,20);
					draw_rectangle(51,0,32,u,1,0);
					//zar_percent=100;
					if(bat[0]._zar<10)
						{
						draw_rectangle(61,5,12,9,1,2);
						graphic_print_text(61,5," %",2,bat[0]._zar,0,1,1);
						}
					else if(bat[0]._zar<100)
						{
						draw_rectangle(58,5,18,9,1,2);
						graphic_print_text(58,5,"  %",3,bat[0]._zar,0,2,1);
						}		
					else 
						{
						draw_rectangle(55,5,24,9,1,2);
						graphic_print_text(55,5,"   %",4,bat[0]._zar,0,3,1);
						}									
					//draw_rectangle(59,3,18,9,1,2);
					//graphic_print_text(53,3,"   %",4,zar_percent,0,3,1);
					}

				}				
			else if(ptr_cnt4<30)
				{
				graphic_print_text(58,5,"   A",4,bat[0]._Ib/10,1,3,1);
				}
			else
				{
				graphic_print_text_text(53,5,"ACCИМ",5,bat[0]._Ib/10,1,3,1);
				}
			//graphic_print_text_text(53,5,"ACCИМ",5,bat[0]._Ib/10,1,3,1);
					
			}

		}	
		

	draw_rectangle(92,4,27,14,0,0);
	draw(92,10,-4,0,0);
	draw(118,10,4,0,0);
	draw(67,31,39,0,0);
	draw(105,31,0,-14,0);	
	draw_ptr(105,34-ptr_cnt3,2,4);
	
	graphic_print_text(70,22,"    B",5,/*ind_reset_cnt*/load_U/10,0,4,1);
	if(load_I<100)graphic_print_text(93,7,"   A",4,load_I,1,3,1);
	else graphic_print_text(90,7,"   A",4,load_I/10,0,3,1);
			
	ptr_cnt++;
	if(ptr_cnt>=3)
		{
		ptr_cnt=0;
		ptr_cnt1++;
		if(ptr_cnt1>=13)
			{
			ptr_cnt1=0;
			}
	
		ptr_cnt2++;
		if(ptr_cnt2>=32)
			{
			ptr_cnt2=0;
			}
				
		ptr_cnt3++;
		if(ptr_cnt3>=15)
			{
			ptr_cnt3=0;
			}

		ptr_cnt4++;
		if(bat[0]._av&0x02)
			{
			if(ptr_cnt4>=45)
				{
				ptr_cnt4=0;
				}
			}
		else
			{
			if(ptr_cnt4>=30)
				{
				ptr_cnt4=0;
				}					
			}
		}			
	}

else
	{
	for(i=0;i<4;i++)
		{
		ptr_bitmap=122*(unsigned)i;
		for(x=(20*i);x<((20*i)+20);x++)
	 		{
			lcd_bitmap[ptr_bitmap++]=caracter[(unsigned)lcd_buffer[x]*6];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+1];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+2];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+3];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+4];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+5];
			} 
		}
	}	
}

//-----------------------------------------------
void ind_hndl(void)
{
//const char* ptr;
const char* ptrs[80];
const char* sub_ptrs[40];
static char sub_cnt,sub_cnt1;
char i,sub_cnt_max;
char ii_;
static char ii_cnt,cnt_ind_bat;


	   
sub_cnt_max=5;
i=0;
	      
	
if(avar_stat&0x0001)
	{
	sub_ptrs[i++]=		"   Авария сети!!!   ";
	sub_cnt_max++;	
	}



if(ips_bat_av_stat)
	{
	sub_ptrs[i++]=	"  Авария батареи    ";
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
	sub_ptrs[i++]=	"   Сработал СК№1    ";
	sub_cnt_max++;	
	}
if((sk_av_stat[1]==sasON)&&(NUMSK>1)&&(!SK_LCD_EN[1]))
	{
	sub_ptrs[i++]=	"   Сработал СК№2    ";
	sub_cnt_max++;	
	}
if((sk_av_stat[2]==sasON)&&(NUMSK>2)&&(!SK_LCD_EN[2]))
	{
	sub_ptrs[i++]=	"   Сработал СК№3    ";
	sub_cnt_max++;	
	}
if((sk_av_stat[3]==sasON)&&(NUMSK>3)&&(!SK_LCD_EN[3]))
	{
	sub_ptrs[i++]=	"   Сработал СК№4    ";
	sub_cnt_max++;	
	}
//#endif
/*if((avar_stat&(1<<(28)))&&(SK_LCD_EN[3]))
	{
	sub_ptrs[i++]=	"   Авария СК №4     ";
	sub_cnt_max++;	
	} */

if(uOutAvIsOn)
	{
	sub_ptrs[i++]=	"  Авария по Uвых!!! ";
	sub_cnt_max++;	
	}
if(uNetAvIsOn)
	{
	sub_ptrs[i++]=	"Авария по Uвх(AC)!!!";
	sub_cnt_max++;	
	}
if(dcAvIsOn)
	{
	sub_ptrs[i++]=	"Авария по Uвх(DC)!!!";
	sub_cnt_max++;	
	}

if(inv[0]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№1 разрыв связи ";
	sub_cnt_max++;	
	}

if((inv[0]._flags_tm&0x01)&&(inv[0]._valid))
	{
	sub_ptrs[i++]=	"Инв.№1 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[0]._flags_tm&0x04)&&(inv[0]._valid))
	{
	sub_ptrs[i++]=	"Инв.№1 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[0]._flags_tm&0x02)&&(inv[0]._valid))
	{
	sub_ptrs[i++]=	"Инв.№1 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[0]._flags_tm&0x08)&&(inv[0]._valid))
	{
	sub_ptrs[i++]=	"Инв.№1 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[0]._flags_tm&0x10)&&(inv[0]._valid))
	{
	sub_ptrs[i++]=	"Инв.№1 заниж. Uвых! ";
	sub_cnt_max++;	
	}

if(inv[1]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№2 разрыв связи ";
	sub_cnt_max++;	
	}

if((inv[1]._flags_tm&0x01)&&(inv[1]._valid))
	{
	sub_ptrs[i++]=	"Инв.№2 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[1]._flags_tm&0x04)&&(inv[1]._valid))
	{
	sub_ptrs[i++]=	"Инв.№2 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[1]._flags_tm&0x02)&&(inv[1]._valid))
	{
	sub_ptrs[i++]=	"Инв.№2 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[1]._flags_tm&0x08)&&(inv[1]._valid))
	{
	sub_ptrs[i++]=	"Инв.№2 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[1]._flags_tm&0x10)&&(inv[1]._valid))
	{
	sub_ptrs[i++]=	"Инв.№2 заниж. Uвых! ";
	sub_cnt_max++;	
	}

if(inv[2]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№3 разрыв связи ";
	sub_cnt_max++;	
	}

if((inv[2]._flags_tm&0x01)&&(inv[2]._valid))
	{
	sub_ptrs[i++]=	"Инв.№3 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[2]._flags_tm&0x04)&&(inv[2]._valid))
	{
	sub_ptrs[i++]=	"Инв.№3 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[2]._flags_tm&0x02)&&(inv[2]._valid))
	{
	sub_ptrs[i++]=	"Инв.№3 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[2]._flags_tm&0x08)&&(inv[2]._valid))
	{
	sub_ptrs[i++]=	"Инв.№3 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[2]._flags_tm&0x10)&&(inv[2]._valid))
	{
	sub_ptrs[i++]=	"Инв.№3 заниж. Uвых! ";
	sub_cnt_max++;	
	}

if(inv[3]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№4 разрыв связи ";
	sub_cnt_max++;	
	}

if((inv[3]._flags_tm&0x01)&&(inv[3]._valid))
	{
	sub_ptrs[i++]=	"Инв.№4 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[3]._flags_tm&0x04)&&(inv[3]._valid))
	{
	sub_ptrs[i++]=	"Инв.№4 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[3]._flags_tm&0x02)&&(inv[3]._valid))
	{
	sub_ptrs[i++]=	"Инв.№4 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[3]._flags_tm&0x08)&&(inv[3]._valid))
	{
	sub_ptrs[i++]=	"Инв.№4 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[3]._flags_tm&0x10)&&(inv[3]._valid))
	{
	sub_ptrs[i++]=	"Инв.№4 заниж. Uвых! ";
	sub_cnt_max++;	
	}

if(inv[4]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№5 разрыв связи ";
	sub_cnt_max++;	
	}

if((inv[4]._flags_tm&0x01)&&(inv[4]._valid))
	{
	sub_ptrs[i++]=	"Инв.№5 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[4]._flags_tm&0x04)&&(inv[4]._valid))
	{
	sub_ptrs[i++]=	"Инв.№5 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[4]._flags_tm&0x02)&&(inv[4]._valid))
	{
	sub_ptrs[i++]=	"Инв.№5 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[4]._flags_tm&0x08)&&(inv[4]._valid))
	{
	sub_ptrs[i++]=	"Инв.№5 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[4]._flags_tm&0x10)&&(inv[4]._valid))
	{
	sub_ptrs[i++]=	"Инв.№5 заниж. Uвых! ";
	sub_cnt_max++;	
	}

if(inv[5]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№6 разрыв связи ";
	sub_cnt_max++;	
	}

if((inv[5]._flags_tm&0x01)&&(inv[5]._valid))
	{
	sub_ptrs[i++]=	"Инв.№6 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[5]._flags_tm&0x04)&&(inv[5]._valid))
	{
	sub_ptrs[i++]=	"Инв.№6 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[5]._flags_tm&0x02)&&(inv[5]._valid))
	{
	sub_ptrs[i++]=	"Инв.№6 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[5]._flags_tm&0x08)&&(inv[5]._valid))
	{
	sub_ptrs[i++]=	"Инв.№6 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[5]._flags_tm&0x10)&&(inv[5]._valid))
	{
	sub_ptrs[i++]=	"Инв.№6 заниж. Uвых! ";
	sub_cnt_max++;	
	}

if(inv[6]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№7 разрыв связи ";
	sub_cnt_max++;	
	}

if((inv[6]._flags_tm&0x01)&&(inv[6]._valid))
	{
	sub_ptrs[i++]=	"Инв.№7 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[6]._flags_tm&0x04)&&(inv[6]._valid))
	{
	sub_ptrs[i++]=	"Инв.№7 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[6]._flags_tm&0x02)&&(inv[6]._valid))
	{
	sub_ptrs[i++]=	"Инв.№7 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[6]._flags_tm&0x08)&&(inv[6]._valid))
	{
	sub_ptrs[i++]=	"Инв.№7 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[6]._flags_tm&0x10)&&(inv[6]._valid))
	{
	sub_ptrs[i++]=	"Инв.№7 заниж. Uвых! ";
	sub_cnt_max++;	
	}

if(inv[7]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№8 разрыв связи ";
	sub_cnt_max++;	
	}

if((inv[7]._flags_tm&0x01)&&(inv[7]._valid))
	{
	sub_ptrs[i++]=	"Инв.№8 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[7]._flags_tm&0x04)&&(inv[7]._valid))
	{
	sub_ptrs[i++]=	"Инв.№8 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[7]._flags_tm&0x02)&&(inv[7]._valid))
	{
	sub_ptrs[i++]=	"Инв.№8 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[7]._flags_tm&0x08)&&(inv[7]._valid))
	{
	sub_ptrs[i++]=	"Инв.№8 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[7]._flags_tm&0x10)&&(inv[7]._valid))
	{
	sub_ptrs[i++]=	"Инв.№8 заниж. Uвых! ";
	sub_cnt_max++;	
	}

if(inv[8]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№9 разрыв связи ";
	sub_cnt_max++;	
	}

if((inv[8]._flags_tm&0x01)&&(inv[8]._valid))
	{
	sub_ptrs[i++]=	"Инв.№9 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[8]._flags_tm&0x04)&&(inv[8]._valid))
	{
	sub_ptrs[i++]=	"Инв.№9 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[8]._flags_tm&0x02)&&(inv[8]._valid))
	{
	sub_ptrs[i++]=	"Инв.№9 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[8]._flags_tm&0x08)&&(inv[8]._valid))
	{
	sub_ptrs[i++]=	"Инв.№9 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[8]._flags_tm&0x10)&&(inv[8]._valid))
	{
	sub_ptrs[i++]=	"Инв.№9 заниж. Uвых! ";
	sub_cnt_max++;	
	}

if(inv[9]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№10 разрыв связи";
	sub_cnt_max++;	
	}

if((inv[9]._flags_tm&0x01)&&(inv[9]._valid))
	{
	sub_ptrs[i++]=	"Инв.№10 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[9]._flags_tm&0x04)&&(inv[9]._valid))
	{
	sub_ptrs[i++]=	"Инв.№10 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[9]._flags_tm&0x02)&&(inv[9]._valid))
	{
	sub_ptrs[i++]=	"Инв.№10 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[9]._flags_tm&0x08)&&(inv[9]._valid))
	{
	sub_ptrs[i++]=	"Инв.№10 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[9]._flags_tm&0x10)&&(inv[9]._valid))
	{
	sub_ptrs[i++]=	"Инв.№10 заниж. Uвых! ";
	sub_cnt_max++;	
	}

if(inv[10]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№11 разрыв связи";
	sub_cnt_max++;	
	}

if((inv[10]._flags_tm&0x01)&&(inv[10]._valid))
	{
	sub_ptrs[i++]=	"Инв.№11 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[10]._flags_tm&0x04)&&(inv[10]._valid))
	{
	sub_ptrs[i++]=	"Инв.№11 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[10]._flags_tm&0x02)&&(inv[10]._valid))
	{
	sub_ptrs[i++]=	"Инв.№11 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[10]._flags_tm&0x08)&&(inv[10]._valid))
	{
	sub_ptrs[i++]=	"Инв.№11 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[10]._flags_tm&0x10)&&(inv[10]._valid))
	{
	sub_ptrs[i++]=	"Инв.№11 заниж. Uвых! ";
	sub_cnt_max++;	
	}

if(inv[11]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№12 разрыв связи";
	sub_cnt_max++;	
	}

if((inv[11]._flags_tm&0x01)&&(inv[11]._valid))
	{
	sub_ptrs[i++]=	"Инв.№12 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[11]._flags_tm&0x04)&&(inv[11]._valid))
	{
	sub_ptrs[i++]=	"Инв.№12 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[11]._flags_tm&0x02)&&(inv[11]._valid))
	{
	sub_ptrs[i++]=	"Инв.№12 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[11]._flags_tm&0x08)&&(inv[11]._valid))
	{
	sub_ptrs[i++]=	"Инв.№12 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[11]._flags_tm&0x10)&&(inv[11]._valid))
	{
	sub_ptrs[i++]=	"Инв.№12 заниж. Uвых! ";
	sub_cnt_max++;	
	}

if(inv[12]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№13 разрыв связи";
	sub_cnt_max++;	
	}

if((inv[12]._flags_tm&0x01)&&(inv[12]._valid))
	{
	sub_ptrs[i++]=	"Инв.№13 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[12]._flags_tm&0x04)&&(inv[12]._valid))
	{
	sub_ptrs[i++]=	"Инв.№13 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[12]._flags_tm&0x02)&&(inv[12]._valid))
	{
	sub_ptrs[i++]=	"Инв.№13 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[12]._flags_tm&0x08)&&(inv[12]._valid))
	{
	sub_ptrs[i++]=	"Инв.№13 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[12]._flags_tm&0x10)&&(inv[12]._valid))
	{
	sub_ptrs[i++]=	"Инв.№13 заниж. Uвых! ";
	sub_cnt_max++;	
	}

if(inv[12]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№14 разрыв связи";
	sub_cnt_max++;	
	}

if((inv[13]._flags_tm&0x01)&&(inv[13]._valid))
	{
	sub_ptrs[i++]=	"Инв.№14 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[13]._flags_tm&0x04)&&(inv[13]._valid))
	{
	sub_ptrs[i++]=	"Инв.№14 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[13]._flags_tm&0x02)&&(inv[13]._valid))
	{
	sub_ptrs[i++]=	"Инв.№14 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[13]._flags_tm&0x08)&&(inv[13]._valid))
	{
	sub_ptrs[i++]=	"Инв.№14 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[13]._flags_tm&0x10)&&(inv[13]._valid))
	{
	sub_ptrs[i++]=	"Инв.№14 заниж. Uвых! ";
	sub_cnt_max++;	
	}

if(inv[14]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№15 разрыв связи";
	sub_cnt_max++;	
	}

if((inv[14]._flags_tm&0x01)&&(inv[14]._valid))
	{
	sub_ptrs[i++]=	"Инв.№15 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[14]._flags_tm&0x04)&&(inv[14]._valid))
	{
	sub_ptrs[i++]=	"Инв.№15 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[14]._flags_tm&0x02)&&(inv[14]._valid))
	{
	sub_ptrs[i++]=	"Инв.№15 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[14]._flags_tm&0x08)&&(inv[14]._valid))
	{
	sub_ptrs[i++]=	"Инв.№15 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[14]._flags_tm&0x10)&&(inv[14]._valid))
	{
	sub_ptrs[i++]=	"Инв.№15 заниж. Uвых! ";
	sub_cnt_max++;	
	}

if(inv[15]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№16 разрыв связи";
	sub_cnt_max++;	
	}

if((inv[15]._flags_tm&0x01)&&(inv[15]._valid))
	{
	sub_ptrs[i++]=	"Инв.№16 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[15]._flags_tm&0x04)&&(inv[15]._valid))
	{
	sub_ptrs[i++]=	"Инв.№16 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[15]._flags_tm&0x02)&&(inv[15]._valid))
	{
	sub_ptrs[i++]=	"Инв.№16 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[15]._flags_tm&0x08)&&(inv[15]._valid))
	{
	sub_ptrs[i++]=	"Инв.№16 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[15]._flags_tm&0x10)&&(inv[15]._valid))
	{
	sub_ptrs[i++]=	"Инв.№16 заниж. Uвых! ";
	sub_cnt_max++;	
	}

if(inv[16]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№17 разрыв связи";
	sub_cnt_max++;	
	}

if((inv[16]._flags_tm&0x01)&&(inv[16]._valid))
	{
	sub_ptrs[i++]=	"Инв.№17 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[16]._flags_tm&0x04)&&(inv[16]._valid))
	{
	sub_ptrs[i++]=	"Инв.№17 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[16]._flags_tm&0x02)&&(inv[16]._valid))
	{
	sub_ptrs[i++]=	"Инв.№17 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[16]._flags_tm&0x08)&&(inv[16]._valid))
	{
	sub_ptrs[i++]=	"Инв.№17 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[16]._flags_tm&0x10)&&(inv[16]._valid))
	{
	sub_ptrs[i++]=	"Инв.№17 заниж. Uвых! ";
	sub_cnt_max++;	
	}

if(inv[17]._conn_av_stat)
	{
	sub_ptrs[i++]=	"Инв.№18 разрыв связи ";
	sub_cnt_max++;	
	}

if((inv[17]._flags_tm&0x01)&&(inv[17]._valid))
	{
	sub_ptrs[i++]=	"Инв.№18 перегрузка   ";
	sub_cnt_max++;	
	}

if((inv[17]._flags_tm&0x04)&&(inv[17]._valid))
	{
	sub_ptrs[i++]=	"Инв.№18 сильн.нагрев ";
	sub_cnt_max++;	
	}

if((inv[17]._flags_tm&0x02)&&(inv[17]._valid))
	{
	sub_ptrs[i++]=	"Инв.№18 перегрев,выкл";
	sub_cnt_max++;	
	}

if((inv[17]._flags_tm&0x08)&&(inv[17]._valid))
	{
	sub_ptrs[i++]=	"Инв.№18 завыш. Uвых! ";
	sub_cnt_max++;	
	}

if((inv[17]._flags_tm&0x10)&&(inv[17]._valid))
	{
	sub_ptrs[i++]=	"Инв.№18 заниж. Uвых! ";
	sub_cnt_max++;	
	}
cnt_of_slave=NUMIST+NUMINV;


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


sub_cnt1++;	
if(sub_cnt1>=20)
	{
	sub_cnt1=0;
	sub_cnt++;
	if(sub_cnt>=sub_cnt_max)
		{
		sub_cnt=0;
		}
	}




	  
if(ind==iMn_INV)
	{
	char flfl=0;
	if((NUMBYPASS>3)||(NUMBYPASS<0))
		{
		//NUMBYPASS=1;
		//NUMPHASE=1;
		flfl=1;
		}

	ptrs[0]	=	"  неопределенность  ";

	if((NUMBYPASS!=1))	
		{
		ptrs[0]	=	"  В работе    kинв. ";

 		ptrs[1]="Uвых=  [В Iвых=   ]А";
     	ptrs[2]="    Pвых=     @Вт   ";
     	ptrs[3]=" 0%:0^:0& 0</>  /0{ ";
		ptrs[4]="    Udc.вх.   #В    ";
		ptrs[5]="    Fвых.     zГц   ";

		 ptrs[6]=  								" Байпасс            ";     
		 if(NUMBYPASS>1) ptrs[6]=  				" Байпасс  N1        ";
		 ptrs[7]=  								" Байпасс  N2        ";
		 ptrs[8]=  								" Байпасс  N3        ";
 		ptrs[9]=  								" Байпасс  N4        ";
 		ptrs[10]=  								" Байпасс  N5        ";
	     ptrs[6+NUMBYPASS]=  					" Инвертор N1        ";
	     ptrs[7+NUMBYPASS]=  					" Инвертор N2        ";
	     ptrs[8+NUMBYPASS]=  					" Инвертор N3        ";
	     ptrs[9+NUMBYPASS]=  					" Инвертор N4        ";
	     ptrs[10+NUMBYPASS]=  					" Инвертор N5        ";
	     ptrs[11+NUMBYPASS]=  					" Инвертор N6        ";
	     ptrs[12+NUMBYPASS]=  					" Инвертор N7        ";
	     ptrs[13+NUMBYPASS]=  					" Инвертор N8        ";
	     ptrs[14+NUMBYPASS]=  					" Инвертор N9        ";
	     ptrs[15+NUMBYPASS]=  					" Инвертор N10       ";
	     ptrs[16+NUMBYPASS]=  					" Инвертор N11       ";
	     ptrs[17+NUMBYPASS]=  					" Инвертор N12       ";
	     ptrs[18+NUMBYPASS]=  					" Инвертор N13       ";
	     ptrs[19+NUMBYPASS]=  					" Инвертор N14       ";
	     ptrs[20+NUMBYPASS]=  					" Инвертор N15       ";
	     ptrs[21+NUMBYPASS]=  					" Инвертор N16       ";
	     ptrs[22+NUMBYPASS]=  					" Инвертор N17       ";
	     ptrs[23+NUMBYPASS]=  					" Инвертор N18       ";
	     ptrs[24+NUMBYPASS]=  					" Инвертор N19       ";
	     ptrs[25+NUMBYPASS]=  					" Инвертор N20       ";
	     ptrs[26+NUMBYPASS]=  					" Инвертор N21       ";
	     ptrs[27+NUMBYPASS]=  					" Инвертор N22       ";
	     ptrs[28+NUMBYPASS]=  					" Инвертор N23       ";
	     ptrs[29+NUMBYPASS]=  					" Инвертор N24       ";
	     ptrs[30+NUMBYPASS]=  					" Инвертор N25       ";
	     ptrs[31+NUMBYPASS]=  					" Инвертор N26       ";
	     ptrs[32+NUMBYPASS]=  					" Инвертор N27       ";
	     ptrs[33+NUMBYPASS]=  					" Инвертор N28       ";
	     ptrs[34+NUMBYPASS]=  					" Инвертор N29       ";
	     ptrs[35+NUMBYPASS]=  					" Инвертор N30       ";
	     ptrs[36+NUMBYPASS]=  					" Инвертор N31       ";
	     ptrs[37+NUMBYPASS]=  					" Инвертор N32       ";
		 ptrs[6+NUMBYPASS+NUMINV]= 				" Таблица инверторов ";
		 ptrs[7+NUMBYPASS+NUMINV]= 				" Внешние датчики    ";
	     ptrs[8+NUMBYPASS+NUMINV]= 				" Установки          "; 
	     ptrs[9+NUMBYPASS+NUMINV]=  			" Журнал событий     "; 
	     ptrs[10+NUMBYPASS+NUMINV]=  			" Выход              "; 
		 ptrs[11+NUMBYPASS+NUMINV]=  			" Версия ПО          ";
		ptrs[12+NUMBYPASS+NUMINV]=  			" tшкаф.        s°С  ";;

		}
	else if((NUMBYPASS==1)&&(NUMPHASE==1))
		{

		ptrs[0]	=	"  В работе    kинв. ";

 		ptrs[1]="Uвых=  [В Iвых=   ]А";
     	ptrs[2]="    Pвых=     @Вт   ";
     	ptrs[3]=" 0%:0^:0& 0</>  /0{ ";
		ptrs[4]="    Udc.вх.   #В    ";
	   	ptrs[5]="    Fвых.     zГц   ";

		 ptrs[6]=  								" Байпасс            ";     
		 if(NUMBYPASS>1) ptrs[6]=  				" Байпасс  N1        ";
		 ptrs[7]=  								" Байпасс  N2        ";
		 ptrs[8]=  								" Байпасс  N3        ";
	     ptrs[6+NUMBYPASS]=  					" Инвертор N1        ";
	     ptrs[7+NUMBYPASS]=  					" Инвертор N2        ";
	     ptrs[8+NUMBYPASS]=  					" Инвертор N3        ";
	     ptrs[9+NUMBYPASS]=  					" Инвертор N4        ";
	     ptrs[10+NUMBYPASS]=  					" Инвертор N5        ";
	     ptrs[11+NUMBYPASS]=  					" Инвертор N6        ";
	     ptrs[12+NUMBYPASS]=  					" Инвертор N7        ";
	     ptrs[13+NUMBYPASS]=  					" Инвертор N8        ";
	     ptrs[14+NUMBYPASS]=  					" Инвертор N9        ";
	     ptrs[15+NUMBYPASS]=  					" Инвертор N10       ";
	     ptrs[16+NUMBYPASS]=  					" Инвертор N11       ";
	     ptrs[17+NUMBYPASS]=  					" Инвертор N12       ";
	     ptrs[18+NUMBYPASS]=  					" Инвертор N13       ";
	     ptrs[19+NUMBYPASS]=  					" Инвертор N14       ";
	     ptrs[20+NUMBYPASS]=  					" Инвертор N15       ";
	     ptrs[21+NUMBYPASS]=  					" Инвертор N16       ";
	     ptrs[22+NUMBYPASS]=  					" Инвертор N17       ";
	     ptrs[23+NUMBYPASS]=  					" Инвертор N18       ";
	     ptrs[24+NUMBYPASS]=  					" Инвертор N19       ";
	     ptrs[25+NUMBYPASS]=  					" Инвертор N20       ";
	     ptrs[26+NUMBYPASS]=  					" Инвертор N21       ";
	     ptrs[27+NUMBYPASS]=  					" Инвертор N22       ";
	     ptrs[28+NUMBYPASS]=  					" Инвертор N23       ";
	     ptrs[29+NUMBYPASS]=  					" Инвертор N24       ";
	     ptrs[30+NUMBYPASS]=  					" Инвертор N25       ";
	     ptrs[31+NUMBYPASS]=  					" Инвертор N26       ";
	     ptrs[32+NUMBYPASS]=  					" Инвертор N27       ";
	     ptrs[33+NUMBYPASS]=  					" Инвертор N28       ";
	     ptrs[34+NUMBYPASS]=  					" Инвертор N29       ";
	     ptrs[35+NUMBYPASS]=  					" Инвертор N30       ";
	     ptrs[36+NUMBYPASS]=  					" Инвертор N31       ";
	     ptrs[37+NUMBYPASS]=  					" Инвертор N32       ";
		 ptrs[6+NUMBYPASS+NUMINV]= 				" Таблица инверторов ";
		 ptrs[7+NUMBYPASS+NUMINV]= 				" Внешние датчики    ";
	     ptrs[8+NUMBYPASS+NUMINV]= 				" Установки          "; 
	     ptrs[9+NUMBYPASS+NUMINV]=  			" Журнал событий     "; 
	     ptrs[10+NUMBYPASS+NUMINV]=  			" Выход              "; 
		ptrs[11+NUMBYPASS+NUMINV]=  			" Версия ПО          ";
		ptrs[12+NUMBYPASS+NUMINV]=  			" tшкаф.        s°С  ";;
		}
	 
 	else if((NUMBYPASS==1)&&(NUMPHASE==3))
		{

		ptrs[0]	=	"  В работе    kинв. ";

		ptrs[1]="    Pвых=     @Вт   ";
 		ptrs[2]="Uвых=  [В/  zВ/  ZВ";
     //	ptrs[2]="    Pвых=     @Вт   ";
     	ptrs[3]=" 0%:0^:0& 0</>  /0{ ";
		ptrs[4]="    Udc.вх.   #В    ";
		ptrs[5]="    Fвых.     zГц   ";

		 ptrs[6]=  								" Байпасс            ";     
		 if(NUMBYPASS>1) ptrs[6]=  				" Байпасс  N1        ";
		 ptrs[7]=  								" Байпасс  N2        ";
		 ptrs[8]=  								" Байпасс  N3        ";
	     ptrs[6+NUMBYPASS]=  					" Инвертор N1        ";
	     ptrs[7+NUMBYPASS]=  					" Инвертор N2        ";
	     ptrs[8+NUMBYPASS]=  					" Инвертор N3        ";
	     ptrs[9+NUMBYPASS]=  					" Инвертор N4        ";
	     ptrs[10+NUMBYPASS]=  					" Инвертор N5        ";
	     ptrs[11+NUMBYPASS]=  					" Инвертор N6        ";
	     ptrs[12+NUMBYPASS]=  					" Инвертор N7        ";
	     ptrs[13+NUMBYPASS]=  					" Инвертор N8        ";
	     ptrs[14+NUMBYPASS]=  					" Инвертор N9        ";
	     ptrs[15+NUMBYPASS]=  					" Инвертор N10       ";
	     ptrs[16+NUMBYPASS]=  					" Инвертор N11       ";
	     ptrs[17+NUMBYPASS]=  					" Инвертор N12       ";
	     ptrs[18+NUMBYPASS]=  					" Инвертор N13       ";
	     ptrs[19+NUMBYPASS]=  					" Инвертор N14       ";
	     ptrs[20+NUMBYPASS]=  					" Инвертор N15       ";
	     ptrs[21+NUMBYPASS]=  					" Инвертор N16       ";
	     ptrs[22+NUMBYPASS]=  					" Инвертор N17       ";
	     ptrs[23+NUMBYPASS]=  					" Инвертор N18       ";
	     ptrs[24+NUMBYPASS]=  					" Инвертор N19       ";
	     ptrs[25+NUMBYPASS]=  					" Инвертор N20       ";
	     ptrs[26+NUMBYPASS]=  					" Инвертор N21       ";
	     ptrs[27+NUMBYPASS]=  					" Инвертор N22       ";
	     ptrs[28+NUMBYPASS]=  					" Инвертор N23       ";
	     ptrs[29+NUMBYPASS]=  					" Инвертор N24       ";
	     ptrs[30+NUMBYPASS]=  					" Инвертор N25       ";
	     ptrs[31+NUMBYPASS]=  					" Инвертор N26       ";
	     ptrs[32+NUMBYPASS]=  					" Инвертор N27       ";
	     ptrs[33+NUMBYPASS]=  					" Инвертор N28       ";
	     ptrs[34+NUMBYPASS]=  					" Инвертор N29       ";
	     ptrs[35+NUMBYPASS]=  					" Инвертор N30       ";
	     ptrs[36+NUMBYPASS]=  					" Инвертор N31       ";
	     ptrs[37+NUMBYPASS]=  					" Инвертор N32       ";
		 ptrs[6+NUMBYPASS+NUMINV]= 				" Таблица инверторов ";
		 ptrs[7+NUMBYPASS+NUMINV]= 				" Внешние датчики    ";
	     ptrs[8+NUMBYPASS+NUMINV]= 				" Установки          "; 
	     ptrs[9+NUMBYPASS+NUMINV]=  			" Журнал событий     "; 
	     ptrs[10+NUMBYPASS+NUMINV]=  			" Выход              "; 
		 ptrs[11+NUMBYPASS+NUMINV]=  			" Версия ПО          ";
		ptrs[12+NUMBYPASS+NUMINV]=  			" tшкаф.        s°С  ";;

		}

	if(flfl) ptrs[0]	=	"Проверьте структуру ";

     if(sub_ind==0)index_set=0;
	else if((index_set-sub_ind)>2)index_set=sub_ind+2;
	else if(sub_ind>index_set)index_set=sub_ind;
	
	if(sub_cnt<5)bgnd_par(ptrs[0],ptrs[index_set+1],ptrs[index_set+2],ptrs[index_set+3]);
	else bgnd_par(sub_ptrs[sub_cnt-5],ptrs[index_set+1],ptrs[index_set+2],ptrs[index_set+3]);



	int2lcd(num_of_wrks_inv,'k',0);

	if(NUMBYPASS)
		{
		int2lcd(byps[0]._Uout/10,'[',0);
		int2lcd(byps[1]._Uout/10,'z',0);
		int2lcd(byps[2]._Uout/10,'Z',0);

		if(byps[0]._Iout>999)int2lcd(byps[0]._Iout/10,']',0);
     	else int2lcd(byps[0]._Iout,']',1);  
   		//int2lcd_mmm(byps._T,'[',0);

		if(byps[0]._Pout>65000)byps[0]._Pout=0; 
		if(byps[1]._Pout>65000)byps[1]._Pout=0;
		if(byps[2]._Pout>65000)byps[2]._Pout=0;

		if(NUMPHASE==3)long2lcd_mmm((unsigned short)(byps[0]._Pout+byps[1]._Pout+byps[2]._Pout),'@',0);
		else long2lcd_mmm((unsigned short)byps[0]._Pout,'@',0);
		}
	else 
		{
		int2lcd(load_U/10,'[',0);
 
  	//int2lcd(load_U,'#',1);
		if(load_I>999)int2lcd(load_I/10,']',0);
 		else int2lcd(load_I,']',1);

		long2lcd_mmm/*int2lcd*/(/*(unsigned short)*/load_P,'@',0);
		}
	int2lcd(dcin_U/10,'#',0);
 	
	int2lcd(LPC_RTC->HOUR,'%',0);
	int2lcd(LPC_RTC->MIN,'^',0);
	int2lcd(LPC_RTC->SEC,'&',0);
	int2lcd(LPC_RTC->DOM,'<',0);
	int2lcd(LPC_RTC->YEAR,'{',0); 
	sub_bgnd(sm_mont[LPC_RTC->MONTH],'>',0);

 	if(bFL2)
 	     {
 	     lcd_buffer[find(':')]=' ';
 	     lcd_buffer[find(':')]=' ';
 	     lcd_buffer[find(':')]=' ';
 	     }

	if((index_set)&&(sub_ind>1))
	     {
	     if(index_set==sub_ind)lcd_buffer[60]=1;
	     else if((index_set-sub_ind)==1)lcd_buffer[40]=1;
	     else if((index_set-sub_ind)==2)lcd_buffer[20]=1;
	     }	
		
	cnt_ind_bat++;
	if(cnt_ind_bat>=(NUMBAT*20)) cnt_ind_bat=0;
	
	if(ND_EXT[0])sub_bgnd("неиспр.",'s',-3);
	else int2lcd_mmm(t_ext[0],'s',0);

	int2lcd(f_out,'z',1);
	//int2lcdyx(bps[20]._cnt,0,2,0); 
	//int2lcdyx(byps._cnt,0,6,0);
	//int2lcdyx(plazma_can1,1,3,0);
	//int2lcdyx(plazma_can2,2,3,0);
/*	int2lcdyx(makb[2]._cnt,0,10,0);*/
	//int2lcdyx(dcAvIsOn,0,1,0);
	//int2lcdyx(dcin_av_stat,0,1,0);
	//int2lcdyx(dcin_av_cnt,0,5,0);
	//int2lcdyx(inv[0]._conn_av_stat,0,10,0);
	//int2lcdyx(inv[0]._flags_tm,0,3,0);	
 	//int2lcdyx(NUMPHASE,0,10,0);
/*	int2lcdyx(modbus_plazma,0,3,0);
	int2lcdyx(U_OUT_SET,0,8,0);

	int2lcdyx(modbus_tcp_plazma[1],0,15,0);
	int2lcdyx(modbus_tcp_plazma[2],0,11,0);
	int2lcdhyx(modbus_crc_plazma[0],1,5);
	int2lcdhyx(modbus_crc_plazma[1],2,5);*/
	//int2lcdyx(inv[0]._flags_tm_dop,0,19,0);
	}

 else if(ind==iBps)
	{
	const char* ptr[8];
 
	simax=5;

	ptr[1]=			" Uист =        (В   ";
	ptr[2]=			" Iист =        [A   ";
	ptr[3]=			" tист =        ]°С  ";
	ptr[4]=			" Сброс аварий       ";
	ptr[5]=			sm_exit;

	if(bps[sub_ind1]._state==bsWRK)
		{
		ptr[0]=		"      в работе      ";
		if((bps[sub_ind1]._flags_tm&0x08)&&(bFL2))
		ptr[0]=		"  СИЛЬНЫЙ НАГРЕВ!!! ";	      
		}
 	 else if(bps[sub_ind1]._state==bsRDY)
	 	{
		ptr[0]=		"      в резерве     ";	
		}

 	 else if(bps[sub_ind1]._state==bsBL)
	 	{
		ptr[0]=		" заблокирован извне ";	
		}

	 else if(bps[sub_ind1]._state==bsAPV)
	 	{
		ptr[0]=		"    Работает АПВ    ";
		}
	 
	 else if(bps[sub_ind1]._state==bsAV)
	 	{
		if(bps[sub_ind1]._av&(1<<0))
		ptr[0]=		" Авария - перегрев! ";
		else if(bps[sub_ind1]._av&(1<<1))
		ptr[0]=		"Авария - завыш.Uвых!";
		else if(bps[sub_ind1]._av&(1<<2))	 
		ptr[0]=		"Авария - заниж.Uвых!";
		else if(bps[sub_ind1]._av&(1<<3))
			{
			ptr[0]=	"  Авария - потеряна ";
			ptr[1]=	"      связь!!!      ";
			ptr[2]=	"                    ";
			simax=0;
			}
		}

	 else if(bps[sub_ind1]._state==bsOFF_AV_NET)
	 	{
		ptr[0]=		"      ВЫКЛЮЧЕН      ";
		ptr[1]=		"     Отсутствует    ";
		ptr[2]=		" первичное питание! ";
		simax=0;
		}

	bgnd_par(			"       БПС N&       ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;

	if(sub_ind>=4)	pointer_set(1);


		

	int2lcd(sub_ind1+1,'&',0);
	int2lcd(bps[sub_ind1]._Uii,'(',1);
     int2lcd(bps[sub_ind1]._Ii,'[',1);  
   	int2lcd_mmm(bps[sub_ind1]._Ti,']',0); 
   			 
    // char2lcdhxy(bps[sub_ind1]._state,0x32);
    
    //	int2lcdyx(sub_ind,0,2,0);
//	int2lcdyx(index_set,0,4,0);	
     }  
else if(ind==iInv)
	{
	const char* ptr[8];
 
	simax=5;

	ptr[1]=			" Uинв =        (В   ";
	ptr[2]=			" Iинв =        [A   ";
	ptr[3]=			" tинв =        ]°С  ";
	ptr[4]=			" Сброс аварий       ";
	ptr[5]=			sm_exit;

	if((inv[sub_ind1]._flags_tm==0)&&(inv[sub_ind1]._cnt==0))
		{
		ptr[0]=		"      в работе      ";
		}
	else if((inv[sub_ind1]._flags_tm==0x04)&&(inv[sub_ind1]._cnt==0))
		{
		ptr[0]=		"  СИЛЬНЫЙ НАГРЕВ!!! ";	      
		}
	else if((inv[sub_ind1]._flags_tm==0x24)&&(inv[sub_ind1]._cnt==0))
		{
		ptr[0]=		"отключился,перегрев ";	      
		}
	else if((inv[sub_ind1]._flags_tm&0x20)&&(inv[sub_ind1]._cnt==0))
		{
		ptr[0]=		"  отсутствует Uвых  ";
		}
	else if(inv[sub_ind1]._cnt!=0)
	 	{
		ptr[0]=		"    не подключен    ";	
		}


	bgnd_par(			"     ИНВЕРТОР N&    ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;

	if(sub_ind>=4)	pointer_set(1);


		

	int2lcd(sub_ind1+1,'&',0);
	int2lcd(inv[sub_ind1]._Uout,'(',1);
     int2lcd(inv[sub_ind1]._Iout,'[',1);  
   	int2lcd_mmm(inv[sub_ind1]._T,']',0); 
   	//int2lcd(inv[sub_ind1]._flags_tm,'[',1);		 
    // char2lcdhxy(bps[sub_ind1]._state,0x32);
    
//int2lcdyx(bps[sub_ind1]._flags_tm&0xFF,0,18,0);

//int2lcdyx(ava,0,4,0);	
//int2lcdyx(plazma_inv[0],0,2,0);
//int2lcdyx(plazma_inv[1],0,5,0);
//int2lcdyx(plazma_inv[2],0,8,0);
//int2lcdyx(plazma_inv[3],0,11,0);
	
     } 
	 
else if(ind==iInv_v2)
	{
	const char* ptr[8];
 
	simax=7;

	ptr[1]=			" Uвых =        (В   ";
	ptr[2]=			" Iвых =        )A   ";
	ptr[3]=			" tинв =        [°С  ";
	ptr[4]=			" Pвых =        ]Вт  ";
	ptr[5]=			" Uсети =       <В   ";
	ptr[6]=			" Uшины =       >В   ";
	ptr[7]=			sm_exit;

	//if((inv[sub_ind1]._flags_tm==0)&&(inv[sub_ind1]._cnt==0))
	//	{
		ptr[0]=		"      в работе      ";
	//	}
	/*else */if((inv[sub_ind1]._flags_tm==0x04)&&(inv[sub_ind1]._cnt==0))
		{
		ptr[0]=		"  СИЛЬНЫЙ НАГРЕВ!!! ";	      
		}
	else if((inv[sub_ind1]._flags_tm==0x24)&&(inv[sub_ind1]._cnt==0))
		{
		ptr[0]=		"отключился,перегрев ";	      
		}
	else if((!(inv[sub_ind1]._flags_tm&0x20))&&(inv[sub_ind1]._cnt==0))
		{
		ptr[0]=		"  отсутствует Uвых  ";
		}
	else if(inv[sub_ind1]._cnt!=0)
	 	{
		ptr[0]=		"    не подключен    ";	
		}


	bgnd_par(			"     ИНВЕРТОР N&    ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;

	if(sub_ind>=simax)	pointer_set(1);


		

	int2lcd(sub_ind1+1,'&',0);
	int2lcd(inv[sub_ind1]._Uout,'(',1);
     int2lcd(inv[sub_ind1]._Iout,')',1);  
   	int2lcd_mmm(inv[sub_ind1]._T,'[',0); 
	int2lcd_mmm(inv[sub_ind1]._Pout,']',0);
	int2lcd(inv[sub_ind1]._Uacin,'<',1);
	int2lcd(inv[sub_ind1]._Uload,'>',1);
   	//int2lcd(inv[sub_ind1]._flags_tm,'[',1);		 
     //char2lcdhyx(inv[sub_ind1]._flags_tm,0,3);
    
	//int2lcdyx(inv[sub_ind1]._cnt,0,19,0); 
	//int2lcdyx(bps[sub_ind1+20]._cnt,0,16,0); 
//int2lcdyx(ava,0,4,0);	
//int2lcdyx(plazma_inv[0],0,2,0);
//int2lcdyx(plazma_inv[1],0,5,0);
//int2lcdyx(plazma_inv[2],0,8,0);
//int2lcdyx(plazma_inv[3],0,11,0);
	//int2lcdyx(sub_ind,0,2,0);
	//int2lcdyx(index_set,0,4,0);
    }

else if(ind==iInv_v3)
	{
	const char* ptr[10];
 
	simax=8;

	ptr[1]=			" Uвых =        (В   ";
	ptr[2]=			" Iвых =        )A   ";
	ptr[3]=			" tинв =        [°С  ";
	ptr[4]=			" Pвых =        ]Вт  ";
	ptr[5]=			" Uсети =       <В   ";
	ptr[6]=			" Uшины =       >В   ";
	ptr[7]=			" Uвход =       ^В   ";
	ptr[8]=			sm_exit;


	ptr[0]=		"                    ";

	if(inv[sub_ind1]._cnt>5)
	 	{
		ptr[0]=		"    не подключен    ";	
		}
	else if((inv[sub_ind1]._flags_tm&0x01)==0x01)
		{
		ptr[0]=		"     ПЕРЕГРУЖЕН!!!  ";	      
		}
	else if((inv[sub_ind1]._flags_tm&0x24)==0x24)
		{
		ptr[0]=		"  СИЛЬНЫЙ НАГРЕВ!!! ";	      
		}
	else if((inv[sub_ind1]._flags_tm&0x02)==0x02)
		{
		ptr[0]=		"ПЕРЕГРЕВ!!ВЫКЛЮЧЕН!!";	      
		}
	else if((inv[sub_ind1]._flags_tm&0x28)==0x28)
		{
		ptr[0]=		"В РАБОТЕ.ЗАВЫШ Uвых!";	      
		}
	else if((inv[sub_ind1]._flags_tm&0x30)==0x30)
		{
		ptr[0]=		"В РАБОТЕ.ЗАНИЖ Uвых!";	      
		}
	else if((inv[sub_ind1]._flags_tm&0xa0)==0x20)
		{
		ptr[0]=		"В РАБОТЕ.ОТ БАТАРЕИ ";	      
		}
	else if((inv[sub_ind1]._flags_tm&0xa0)==0xa0)
		{
		ptr[0]=		"В РАБОТЕ.ОТ СЕТИ    ";	      
		}
	else if((inv[sub_ind1]._flags_tm_dop&0x01)==0x01)
		{
		ptr[0]=		"ОТКЛ. Udc не в норме";	      
		}


	bgnd_par(			"     ИНВЕРТОР N&    ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;

	if(sub_ind>=simax)	pointer_set(1);


		

	int2lcd(sub_ind1+1,'&',0);
	int2lcd(inv[sub_ind1]._Uout,'(',1);
     int2lcd(inv[sub_ind1]._Iout,')',1);  
   	int2lcd_mmm(inv[sub_ind1]._T,'[',0); 
	int2lcd_mmm(inv[sub_ind1]._Pout,']',0);
	int2lcd(inv[sub_ind1]._Uacin,'<',1);
	int2lcd(inv[sub_ind1]._Uload,'>',1);
	int2lcd(inv[sub_ind1]._Udcin,'^',1);
   	//int2lcd(inv[sub_ind1]._flags_tm,'[',1);		 
     //char2lcdhyx(inv[sub_ind1]._flags_tm,0,3);
    
	//int2lcdyx(inv[sub_ind1]._cnt,0,19,0); 
	//int2lcdyx(bps[sub_ind1+20]._cnt,0,16,0); 
//int2lcdyx(ava,0,4,0);	
//int2lcdyx(plazma_inv[0],0,2,0);
//int2lcdyx(plazma_inv[1],0,5,0);
//int2lcdyx(plazma_inv[2],0,8,0);
//int2lcdyx(plazma_inv[3],0,11,0);
	//int2lcdyx(sub_ind,0,2,0);
	//int2lcdyx(index_set,0,4,0);
	//int2lcdyx(inv[sub_ind1]._flags_tm,0,8,0);
	//int2lcdyx(inv[sub_ind1]._flags_tm_dop,0,4,0);
    }

else if(ind==iByps)
	{
	const char* ptr[8];

	static char iByps_ind_cnt;
	
	if(++iByps_ind_cnt>=100)iByps_ind_cnt=0;



	simax=7;

	ptr[1]=			" Uвых =        {В   ";
	ptr[2]=			" Iвых =        }A   ";
	ptr[3]=			" Pвых =        ]Вт  ";
	ptr[4]=			" tбп  =        [°С  ";
	ptr[5]=			" Uсети =       <В   ";
	ptr[6]=			" Uинв  =       >В   ";
	ptr[7]=			sm_exit;

	ptr[0]=		"      в работе      ";
	
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

	if((byps[sub_ind1]._flags&0x04)&&(byps[sub_ind1]._cnt<5))
		{
		ptr[0]=		"  СИЛЬНЫЙ НАГРЕВ!!! ";	      
		}
	else if((byps[sub_ind1]._flags&0x02)&&(byps[sub_ind1]._cnt<5))
		{
		ptr[0]=		"отключился,перегрев ";	      
		}
	else if(byps[sub_ind1]._cnt>10)
	 	{
		ptr[0]=		"    не подключен    ";	
		}
	   //"  БАЙПАСС (адр.= !) ",

	if(NUMBYPASS>=2)
		{
		bgnd_par(	"     БАЙПАС N@     ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);
		}
	 else 
	 	{
		bgnd_par(		"   БАЙПАС (адр.= !) ",
						ptr[index_set],
						ptr[index_set+1],
						ptr[index_set+2]);
		}

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;

	if(sub_ind>=simax)	pointer_set(1);


	int2lcd(byps[sub_ind1]._Uout,'{',1);
	if(byps[sub_ind1]._Iout>999)int2lcd(byps[sub_ind1]._Iout/10,'}',0);
     else int2lcd(byps[sub_ind1]._Iout,'}',1);  
   	int2lcd_mmm(byps[sub_ind1]._T,'[',0); 
	if(byps[sub_ind1]._Pout>65000)byps[sub_ind1]._Pout=0; 
	long2lcd_mmm((unsigned short)byps[sub_ind1]._Pout,']',0);
	//int2lcd_mmm(byps._Pout,']',0);
	int2lcd(byps[sub_ind1]._UinACprim,'<',1);
	int2lcd(byps[sub_ind1]._UinACinvbus,'>',1);
	//int2lcdyx(iByps_ind_cnt,0,2,0);
	int2lcd(byps[sub_ind1]._adress-19,'!',0);
	int2lcd(sub_ind1+1,'@',0);
    }
else if(ind==iByps3f)
	{
	const char* ptr[8];

	static char iByps_ind_cnt;
	
	if(++iByps_ind_cnt>=40)iByps_ind_cnt=0;



	simax=7;

	ptr[1]=			" Uвых=  {В/  {В/  {В";
	ptr[2]=			" Iвых=  }A/  }A/  }A";
	ptr[3]=			" Pвых=  ]/  ]/  ]кВт";
	ptr[4]=			" tбп=   [/  [/  [°С ";
	ptr[5]=			" Uсети=   </  </  <В";
	ptr[6]=			" Uшины=   >/  >/  >В";
	ptr[7]=			sm_exit;

	ptr[0]=		"      в работе      ";
	
	if(iByps_ind_cnt<=20)
		{
		if(byps[0]._flags&0x40)ptr[0]=				"Приоритет инверторы ";
		else ptr[0]=								"Приоритет сеть      ";
		}

	if(iByps_ind_cnt>20)
		{
		if(byps[0]._flags&0x80)ptr[0]=				"Работа от инверторов";
		else ptr[0]=								"Работа от сети      ";
		}

	if((byps[0]._flags&0x04)&&(byps[0]._cnt<5))
		{
		ptr[0]=		"  СИЛЬНЫЙ НАГРЕВ!!! ";	      
		}
	else if((byps[0]._flags&0x02)&&(byps[0]._cnt<5))
		{
		ptr[0]=		"отключился,перегрев ";	      
		}
	else if(byps[0]._cnt>10)
	 	{
		ptr[0]=		"    не подключен    ";	
		}
	   //"  БАЙПАСС (адр.= !) ",

	bgnd_par(	"     БАЙПАС 3Ф     ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;

	if(sub_ind>=simax)	pointer_set(1);


	int2lcd(byps[0]._Uout/10,'{',0);
	int2lcd(byps[1]._Uout/10,'{',0);
	int2lcd(byps[2]._Uout/10,'{',0);
	if(byps[0]._Iout>99)int2lcd(byps[0]._Iout/10,'}',0);
    else int2lcd(byps[0]._Iout,'}',1); 
	if(byps[1]._Iout>99)int2lcd(byps[1]._Iout/10,'}',0);
    else int2lcd(byps[1]._Iout,'}',1); 
	if(byps[2]._Iout>99)int2lcd(byps[2]._Iout/10,'}',0);
    else int2lcd(byps[2]._Iout,'}',1); 
			 
   	int2lcd_mmm(byps[0]._T,'[',0);
	int2lcd_mmm(byps[1]._T,'[',0);
	int2lcd_mmm(byps[2]._T,'[',0);
	 
	if(byps[0]._Pout>65000)byps[0]._Pout=0;
	if(byps[1]._Pout>65000)byps[1]._Pout=0;
	if(byps[2]._Pout>65000)byps[2]._Pout=0;
	 
	long2lcd_mmm((unsigned short)(byps[0]._Pout/100),']',1);
	long2lcd_mmm((unsigned short)(byps[1]._Pout/100),']',1);
	long2lcd_mmm((unsigned short)(byps[2]._Pout/100),']',1);

	//int2lcd_mmm(byps._Pout,']',0);
	int2lcd(byps[0]._UinACprim/10,'<',0);
	int2lcd(byps[1]._UinACprim/10,'<',0);
	int2lcd(byps[2]._UinACprim/10,'<',0);

	int2lcd(byps[0]._UinACinvbus/10,'>',0);
	int2lcd(byps[1]._UinACinvbus/10,'>',0);
	int2lcd(byps[2]._UinACinvbus/10,'>',0);
	//int2lcdyx(iByps_ind_cnt,0,2,0);
	//int2lcd(byps[sub_ind1]._adress-19,'!',0);
	//int2lcd(sub_ind1+1,'@',0);
    }
#ifndef _DEBUG_	 	  
else if(ind==iNet)
	{
	bgnd_par(		"        СЕТЬ        ",
				" U   =     [В       ",
				" f   =     ]Гц      ",
				sm_exit);
	lcd_buffer[60]=1;
	     	
     int2lcd(net_U,'[',0);
     int2lcd(net_F,']',1);

 //temp_SL=(signed long)net_buff_;
//temp_SL*=Kunet;    
//int2lcdyx(net_buff_,0,4,0);
//int2lcdyx(Kunet,0,9,0);
                  	      	   	    		
     }

else if(ind==iNet3)
	{


	ptrs[0]=  		" UфA           !В   ";
    ptrs[1]=  		" UфB           @В   ";
    ptrs[2]=  	    " UфC           #В   ";
	ptrs[3]=  	    " f   =     ]Гц      ";           
	ptrs[4]=  		" Выход              ";


	bgnd_par(		"        СЕТЬ        ",
					ptrs[index_set],
					ptrs[index_set+1],
					ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

    int2lcd(net_Ua,'!',0);
	int2lcd(net_Ub,'@',0);
	int2lcd(net_Uc,'#',0);
	#ifdef UKU_220_IPS_TERMOKOMPENSAT
	int2lcd(net_F3,']',1);
	#else
    int2lcd(net_F,']',1);
    #endif

 //temp_SL=(signed long)net_buff_;
//temp_SL*=Kunet;    
//int2lcdyx(net_buff_,0,4,0);
//int2lcdyx(Kunet,0,9,0);
                  	      	   	    		
     }

else if(ind==iLoad)
	{
	bgnd_par(		"      НАГРУЗКА      ",
				" Uнагр =     [В     ",
				" Iнагр =     ]А     ",
				sm_exit);
	lcd_buffer[60]=1;
	     	
     int2lcd(load_U,'[',1);
     int2lcd(load_I,']',1);

     
                   	      	   	    		
     }

else if(ind==iExtern)
	{
	signed char temp;

	ptrs[0]=  		" СК1              $ ";
	ptrs[1]= 		" СК2              % ";
	ptrs[2]= 		" СК3              ^ ";
	ptrs[3]= 		" СК4              & ";		
	ptrs[NUMSK]=	" Выход              ";
	ptrs[NUMSK+1]=	"                    ";
	ptrs[NUMSK+2]=	"                    ";

	bgnd_par(		"  ВНЕШНИЕ ДАТЧИКИ   ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

	temp=-11;
	if(sk_stat[0]==ssON)		temp=-9;
	if(sk_av_stat[0]==sasON) 	sub_bgnd("АВАРИЯ",'$',temp-2);
	else                     	sub_bgnd("НОРМА",'$',temp-1);
	if(sk_stat[0]==ssON)		sub_bgnd("ЗАМКН.",'$',-4);
	if(sk_stat[0]==ssOFF)		sub_bgnd("РАЗОМКН.",'$',-6);

	temp=-11;
	if(sk_stat[1]==ssON)		temp=-9;
	if(sk_av_stat[1]==sasON) 	sub_bgnd("АВАРИЯ",'%',temp-2);
	else                     	sub_bgnd("НОРМА",'%',temp-1);
	if(sk_stat[1]==ssON)		sub_bgnd("ЗАМКН.",'%',-4);
	if(sk_stat[1]==ssOFF)		sub_bgnd("РАЗОМКН.",'%',-6);

	temp=-11;
	if(sk_stat[2]==ssON)		temp=-9;
	if(sk_av_stat[2]==sasON) 	sub_bgnd("АВАРИЯ",'^',temp-2);
	else                     	sub_bgnd("НОРМА",'^',temp-1);
	if(sk_stat[2]==ssON)		sub_bgnd("ЗАМКН.",'^',-4);
	if(sk_stat[2]==ssOFF)		sub_bgnd("РАЗОМКН.",'^',-6);

	temp=-11;
	if(sk_stat[3]==ssON)		temp=-9;
	if(sk_av_stat[3]==sasON) 	sub_bgnd("АВАРИЯ",'&',temp-2);
	else                     	sub_bgnd("НОРМА",'&',temp-1);
 	if(sk_stat[3]==ssON)		sub_bgnd("ЗАМКН.",'&',-4);
	if(sk_stat[3]==ssOFF)		sub_bgnd("РАЗОМКН.",'&',-6);

     }

else if(ind==iExtern_3U)
	{

	ptrs[0]=  		" tвнеш.возд.    !°С ";
     ptrs[1]=  		" tотсек ЭПУ     @°С ";
     ptrs[5]=  	     " Сух.конт.№1      $ ";            
     ptrs[6]=  	     " Сух.конт.№2      % ";
	ptrs[7]=  	     " Сух.конт.№3      ^ ";
	ptrs[8]=  	     " Сух.конт.№4      & ";
	ptrs[5+NUMSK]=  	" Выход              ";
	ptrs[6+NUMSK]=  	"                    ";
	ptrs[7+NUMSK]=  	"                    ";

	bgnd_par(		"  ВНЕШНИЕ ДАТЧИКИ   ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);


     if(ND_EXT[0])sub_bgnd("неиспр.",'!',-3);
     else int2lcd_mmm(t_ext[0],'!',0);

	if(ND_EXT[1])sub_bgnd("неиспр.",'@',-3);
     else int2lcd_mmm(t_ext[1],'@',0);


	if(sk_av_stat[0]==sasON)	sub_bgnd("АВАРИЯ",'$',0);
	else                     sub_bgnd("НОРМА",'$',0);
	

	if(sk_av_stat[1]==sasON) sub_bgnd("АВАРИЯ",'%',0);
	else                     sub_bgnd("НОРМА",'%',0);

	if(sk_av_stat[2]==sasON) sub_bgnd("АВАРИЯ",'^',0);
	else                     sub_bgnd("НОРМА",'^',0);

	if(sk_av_stat[3]==sasON) sub_bgnd("АВАРИЯ",'&',0);
	else                     sub_bgnd("НОРМА",'&',0);

     }


else if(ind==iExtern_6U)
	{

	ptrs[0]=  			" t1             !°С ";
	ptrs[1]=  			" t2             @°С ";
	ptrs[2]=  			" t3             #°С ";
	ptrs[NUMDT]=  		" СК1        $       ";
	ptrs[NUMDT+1]= 		" СК2        %       ";
	ptrs[NUMDT+2]= 		" СК3        ^       ";
	ptrs[NUMDT+3]= 		" СК4        &       ";		
	ptrs[NUMDT+NUMSK]=	" Выход              ";
	ptrs[NUMDT+NUMSK+1]="                    ";
	ptrs[NUMDT+NUMSK+2]="                    ";

	bgnd_par(		"  ВНЕШНИЕ ДАТЧИКИ   ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

    if(ND_EXT[0])sub_bgnd("неиспр.",'!',-3);
    else if(index_set==0)int2lcd_mmm(t_ext[0],'!',0);

    if(ND_EXT[1])sub_bgnd("неиспр.",'@',-3);
    else int2lcd_mmm(t_ext[1],'@',0);

    if(ND_EXT[2])sub_bgnd("неиспр.",'#',-3);
    else int2lcd_mmm(t_ext[2],'#',0);

	if(sk_av_stat[0]==sasON) sub_bgnd("АВАРИЯ",'$',0);
	else                     sub_bgnd("НОРМА",'$',0);

	if(sk_av_stat[1]==sasON) sub_bgnd("АВАРИЯ",'%',0);
	else                     sub_bgnd("НОРМА",'%',0);

	if(sk_av_stat[2]==sasON) sub_bgnd("АВАРИЯ",'^',0);
	else                     sub_bgnd("НОРМА",'^',0);

	if(sk_av_stat[3]==sasON) sub_bgnd("АВАРИЯ",'&',0);
	else                     sub_bgnd("НОРМА",'&',0);

     }

else if(ind==iExtern_220)
	{

	ptrs[0]=  			" t1             !°С ";
	ptrs[1]=  			" t2             @°С ";
	ptrs[2]=  			" t3             #°С ";
	ptrs[NUMDT]=  		" СК1        $       ";
	ptrs[NUMDT+1]= 		" СК2        %       ";
	ptrs[NUMDT+2]= 		" СК3        ^       ";
	ptrs[NUMDT+3]= 		" СК4        &       ";		
	ptrs[NUMDT+NUMSK]=	" Выход              ";
	ptrs[NUMDT+NUMSK+1]="                    ";
	ptrs[NUMDT+NUMSK+2]="                    ";

	bgnd_par(		"  ВНЕШНИЕ ДАТЧИКИ   ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

    if(ND_EXT[0])sub_bgnd("неиспр.",'!',-3);
    else int2lcd_mmm(t_ext[0],'!',0);

    if(ND_EXT[1])sub_bgnd("неиспр.",'@',-3);
    else int2lcd_mmm(t_ext[1],'@',0);

    if(ND_EXT[2])sub_bgnd("неиспр.",'#',-3);
    else int2lcd_mmm(t_ext[2],'#',0);

	if(sk_av_stat[0]==sasON) sub_bgnd("АВАРИЯ",'$',0);
	else                     sub_bgnd("НОРМА",'$',0);

	if(sk_av_stat[1]==sasON) sub_bgnd("АВАРИЯ",'%',0);
	else                     sub_bgnd("НОРМА",'%',0);

	if(sk_av_stat[2]==sasON) sub_bgnd("АВАРИЯ",'^',0);
	else                     sub_bgnd("НОРМА",'^',0);

	if(sk_av_stat[3]==sasON) sub_bgnd("АВАРИЯ",'&',0);
	else                     sub_bgnd("НОРМА",'&',0);

     }


else if(ind==iVent)
	{

	ptrs[0]=  		" Fвент.текущ.     !%";
     ptrs[1]=  		" Fвент.max. (  @%) #";
	ptrs[2]=  	     " Выход              ";

	bgnd_par(			"     ВЕНТИЛЯТОР     ",
					ptrs[index_set],
					ptrs[index_set+1],
					ptrs[index_set+2]);

	pointer_set(1);

     int2lcd(main_vent_pos*5,'!',0);
	int2lcd(pos_vent,'#',0);
	int2lcd(pos_vent*5+45,'@',0);     
	}

else if(ind==iAvt)
	{
     ptrs[0]=  		"  АВТОМАТЫ НАГРУЗОК ";
	ptrs[1]=  		" Автомат №1       ! ";
	ptrs[2]=  		" Автомат №2       @ ";
	ptrs[3]=  		" Автомат №3       # ";
	ptrs[4]=  		" Автомат №4       $ ";
	ptrs[5]=  		" Автомат №5       % ";
	ptrs[6]=  		" Автомат №6       ^ ";
	ptrs[7]=  		" Автомат №7       & ";
	ptrs[8]=  		" Автомат №8       * ";
	ptrs[9]=  		" Автомат №9       ( ";
	ptrs[10]=  		" Автомат №10      ) ";
	ptrs[11]=  		" Автомат №11      + ";
	ptrs[12]=  		" Автомат №12      = ";

	ptrs[1+NUMAVT]=  	" Выход              ";
	ptrs[2+NUMAVT]=  	"                    ";
	ptrs[3+NUMAVT]=  	"                    ";

	bgnd_par(		ptrs[0],
				ptrs[index_set+1],
				ptrs[index_set+2],
				ptrs[index_set+3]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

	//int2lcdyx(eb2_data_short[6],0,6,0);

	if(avt_stat[0]==avtON)	sub_bgnd("ВКЛ.",'!',-3);
	else 				sub_bgnd("ВЫКЛ.",'!',-4);
	if(avt_stat[1]==avtON)	sub_bgnd("ВКЛ.",'@',-3);
	else 				sub_bgnd("ВЫКЛ.",'@',-4);
	if(avt_stat[2]==avtON)	sub_bgnd("ВКЛ.",'#',-3);
	else 				sub_bgnd("ВЫКЛ.",'#',-4);
	if(avt_stat[3]==avtON)	sub_bgnd("ВКЛ.",'$',-3);
	else 				sub_bgnd("ВЫКЛ.",'$',-4);
	if(avt_stat[4]==avtON)	sub_bgnd("ВКЛ.",'%',-3);
	else 				sub_bgnd("ВЫКЛ.",'%',-4);
	if(avt_stat[5]==avtON)	sub_bgnd("ВКЛ.",'^',-3);
	else 				sub_bgnd("ВЫКЛ.",'^',-4);
	if(avt_stat[6]==avtON)	sub_bgnd("ВКЛ.",'&',-3);
	else 				sub_bgnd("ВЫКЛ.",'&',-4);
	if(avt_stat[7]==avtON)	sub_bgnd("ВКЛ.",'*',-3);
	else 				sub_bgnd("ВЫКЛ.",'*',-4);
	if(avt_stat[8]==avtON)	sub_bgnd("ВКЛ.",'(',-3);
	else 				sub_bgnd("ВЫКЛ.",'(',-4);
	if(avt_stat[9]==avtON)	sub_bgnd("ВКЛ.",')',-3);
	else 				sub_bgnd("ВЫКЛ.",')',-4);
	if(avt_stat[10]==avtON)	sub_bgnd("ВКЛ.",'+',-3);
	else 				sub_bgnd("ВЫКЛ.",'+',-4); 
	if(avt_stat[11]==avtON)	sub_bgnd("ВКЛ.",'=',-3);
	else 				sub_bgnd("ВЫКЛ.",'=',-4);
     //int2lcd(Uvv[1],'$',0);
     //int2lcd(Uvv[2],'$',0);

     //long2lcd_mmm(power_summary,'%',2);
     //int2lcd(power_current,'^',0);

     //int2lcdyx(adc_buff_ext_[0],0,4,0);
     //int2lcdyx(adc_buff_ext_[1],0,10,0);
     //int2lcdyx(adc_buff_ext_[2],0,16,0);
     }

else if(ind==iEnerg)
	{
     ptrs[0]=  		"  ЭЛЕКТРОСНАБЖЕНИЕ  ";

     ptrs[1]=  		" Ввод       #В      ";
     ptrs[2]=  	     " ПЭС        $В      ";            
     ptrs[3]=  	     " Pсумм.       %кВт*ч";
	ptrs[4]=  	     " Pтекущ.      ^Вт   ";
	ptrs[5]=  	     " Выход              ";
	ptrs[6]=  	     "                    ";
	ptrs[7]=  	     "                    ";

	bgnd_par(		ptrs[0],
				ptrs[index_set+1],
				ptrs[index_set+2],
				ptrs[index_set+3]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

	int2lcd(Uvv0,'#',0);
     int2lcd(Uvv[1],'$',0);
     //int2lcd(Uvv[2],'$',0);

     long2lcd_mmm(power_summary,'%',2);
     int2lcd(power_current,'^',0);

     //int2lcdyx(adc_buff_ext_[0],0,4,0);
     //int2lcdyx(adc_buff_ext_[1],0,10,0);
     //int2lcdyx(adc_buff_ext_[2],0,16,0);
     }

else if(ind==iEnerg3)
	{
     ptrs[0]=  		"  ЭЛЕКТРОСНАБЖЕНИЕ  ";

     ptrs[1]=  		" Ввод ф.A    !В     ";
	ptrs[2]=  		" Ввод ф.B    @В     ";
	ptrs[3]=  		" Ввод ф.C    #В     ";
     ptrs[4]=  	     " ПЭС  ф.A    &В     ";
     ptrs[5]=  	     " ПЭС  ф.B    )В     ";
     ptrs[6]=  	     " ПЭС  ф.C    (В     ";		            
     ptrs[7]=  	     " Pсумм.       %кВт*ч";
	ptrs[8]=  	     " Pтекущ.      ^Вт   ";
	ptrs[9]=  	     " Выход              ";
	ptrs[10]=  	     "                    ";
	ptrs[11]=  	     "                    ";

	bgnd_par(		ptrs[0],
				ptrs[index_set+1],
				ptrs[index_set+2],
				ptrs[index_set+3]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

	int2lcd(Uvv_eb2[0],'!',0);
	int2lcd(Uvv_eb2[1],'@',0);
	int2lcd(Uvv_eb2[2],'#',0);
	int2lcd(Upes_eb2[0],'&',0);
	int2lcd(Upes_eb2[1],')',0);
	int2lcd(Upes_eb2[2],'(',0);
     long2lcd_mmm(power_summary,'%',3);
     int2lcd(power_current,'^',0);

     }

else if(ind==iSpc)
	{

 	ptrs[0]=	" Выр.заряд          ";
 	ptrs[1]=	" Авт.выр.заряд      ";
 	ptrs[2]=	" К.Е. батареи N1    ";
 	ptrs[3]=	" К.Е. батареи N2    ";
 //	ptrs[4]=	" А.К.Е.  бат. N1    ";
 //	ptrs[5]=	" А.К.Е.  бат. N2    ";
 	ptrs[4]=	" Выход              ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
    	bgnd_par( "     СПЕЦФУНКЦИИ    ",
    	          ptrs[index_set],
    	          ptrs[index_set+1],
    	          ptrs[index_set+2]);
	pointer_set(1);
	}    		


 else if(ind==iVz)
	{          
	if(sub_ind==22) bgnd_par(	"ВЫРАВНИВАЮЩИЙ ЗАРЯД ",
							"    невозможен,     ",
							"  включен контроль  ",
							"   емкости бат.N1   "); 
	else if(sub_ind==33) bgnd_par("ВЫРАВНИВАЮЩИЙ ЗАРЯД ",
							"    невозможен,     ",
							"  включен контроль  ",
							"   емкости бат.N2   ");
	else if(spc_stat==spcVZ)
		{
		bgnd_par(				"ВЫРАВНИВАЮЩИЙ ЗАРЯД ",
							" Длит.-сть     (ч.  ",
							" Включен            ",
							sm_exit);
		}
	else 
		{
		bgnd_par(				"ВЫРАВНИВАЮЩИЙ ЗАРЯД ",
							" Длит.-сть     (ч.  ",
							" Выключен           ",
							sm_exit);
		}	

	pointer_set(1);	

	int2lcd(VZ_HR,'(',0);
	} 
	
	
else if(ind==iKe)
	{    
	if((spc_stat==spcKE)&&(spc_bat==sub_ind1))
		{
		bgnd_par(	"  КОНТРОЛЬ ЕМКОСТИ  ",
				"     БАТАРЕИ N{     ",
				" Включен            ",
				sm_exit);
		}
	else
		{
		bgnd_par(	"  КОНТРОЛЬ ЕМКОСТИ  ",
				"     БАТАРЕИ N{     ",
				" Выключен           ",
				sm_exit);
		}
	
				
	//if(sub_ind==0) lcd_buffer[41]=1; 
	//else if(sub_ind==1) lcd_buffer[51]=1;
	pointer_set(2);
	int2lcd(sub_ind1+1,'{',0);
	}	  

#endif

else if(ind==iLog)
	{
	//char dt[4],dt_[4],dt__[4];
//	char iii;

	av_j_si_max=lc640_read_int(CNT_EVENT_LOG);
	if(av_j_si_max>64)av_j_si_max=0;

	if(av_j_si_max==0)
		{
		bgnd_par("   ЖУРНАЛ СОБЫТИЙ   "," Журнал пуст        ",sm_exit,sm_);
		//lcd_buffer[33]=1;
		sub_ind=1;
		index_set=0;
		}       
		
	else if(av_j_si_max==1)
		{
		bgnd_par("   ЖУРНАЛ СОБЫТИЙ   "," (                  ",sm_exit," Очистить журнал    ");
		//if(sub_ind==0)lcd_buffer[16]=1;
		//else if(sub_ind==1)lcd_buffer[33]=1;
		//else if(sub_ind==2)lcd_buffer[50]=1;		
		index_set=0;
		}

	else if(av_j_si_max==2)
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>2) index_set=sub_ind-2;		
		if(index_set==0) bgnd_par("   ЖУРНАЛ СОБЫТИЙ   "," (                  "," [                  ",sm_exit);
		else if(index_set==1) bgnd_par("   ЖУРНАЛ СОБЫТИЙ   "," (                  ",sm_exit," Очистить журнал    ");
		
		//if((sub_ind-index_set)==0) lcd_buffer[16]=1; 
		//else if((sub_ind-index_set)==1) lcd_buffer[33]=1;
		//else if((sub_ind-index_set)==2) lcd_buffer[50]=1;
		}
		
	else if(av_j_si_max>2)
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>2) index_set=sub_ind-2;  
		if(index_set==(av_j_si_max-1)) bgnd_par("   ЖУРНАЛ СОБЫТИЙ   "," (                  ",sm_exit," Очистить журнал    ");
		else if(index_set==(av_j_si_max-2)) bgnd_par("   ЖУРНАЛ СОБЫТИЙ   "," (                  "," [                  ",sm_exit);
		else bgnd_par("   ЖУРНАЛ СОБЫТИЙ   "," (                  "," [                  "," {                  ");
		
		//if((sub_ind-index_set)==0) lcd_buffer[16]=1; 
		//else if((sub_ind-index_set)==1) lcd_buffer[33]=1;
		//else if((sub_ind-index_set)==2) lcd_buffer[50]=1;

		}
	pointer_set(1);
	event2ind(index_set,'(');
	event2ind(index_set+1,'[');	
	event2ind(index_set+2,'{');	  
     
	}



else if(ind==iLog_)
	{	
	unsigned short tempUI/*,tempUI_*/;
//	unsigned long tempUL;
	char av_head[4],av_data_on[8],av_data_off[8],av_data[4];
	short av_head_int[2];
	
	bgnd_par(sm_,sm_,sm_,sm_);
	tempUI=lc640_read_int(PTR_EVENT_LOG);
	tempUI=ptr_carry(tempUI,64,-1*((signed)sub_ind1));
	tempUI*=32;
	tempUI+=EVENT_LOG;
     
     lc640_read_long_ptr(tempUI,av_head);
     lc640_read_long_ptr(tempUI+4,(char*)av_head_int);
     lc640_read_long_ptr(tempUI+8,av_data_on);
     lc640_read_long_ptr(tempUI+12,&(av_data_on[4])); 
     lc640_read_long_ptr(tempUI+16,av_data_off);
     lc640_read_long_ptr(tempUI+20,&(av_data_off[4]));      
	lc640_read_long_ptr(tempUI+24,av_data);
	
	//av_head_int[0]=123;  
//av_head_int[1]=456;	

	if((av_head[0]=='U')&&(av_head[2]=='R'))
		{
		bgnd_par(	
				"    Перезагрузка    ",
				"   или включение    ",
				" инверторной системы",
				"  0%(  0^ 0@:0#:0$  ");
				
				  	
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		//int2lcd(av_data_on[1],'(',0);
		//int2lcdyx(av_data_on[1],2,1,0);
		av_j_si_max=0;
		
		}

	else if((av_head[0]=='P')&&(av_head[2]=='A'))
		{  
		ptrs[0]="   Авария сети!!!   ";
		ptrs[1]="  0%(  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[2]="    не устранена    ";
			ptrs[3]="     Uсети=  +В     ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			int2lcd(net_U,'+',0);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[2]="      устранена     ";
			ptrs[3]="  0[]  0< 0>:0=:0)  ";
			ptrs[4]="     Uмин=  +В      ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);
			
			int2lcd(av_data[0]+(av_data[1]*256),'+',0);			
			}	
		
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		av_j_si_max=1;
		}
	else if((av_head[0]=='D')&&(av_head[2]=='U'))
		{  
		ptrs[0]=" Авария Udc   (  !В)";
		ptrs[1]="                    ";
		ptrs[2]="  0%&  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    не устранена    ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="     устранена      ";
			ptrs[4]="  0[]  0< 0>:0=:0z  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],'z',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);

			}	
		//int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'&',0);

		int2lcd(av_head_int[0],'!',0);

		if(av_head[1]>=9)		sub_bgnd(" Z",'Z',0);
		int2lcd(av_head[1]+1,'Z',0);
		
		av_j_si_max=1;
		}

	else if((av_head[0]=='I')&&(av_head[2]=='O'))
		{  
		ptrs[0]=" Авария Uвых  (  !В)";
		ptrs[1]="    инвертор NZ     ";
		ptrs[2]="  0%&  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    не устранена    ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="     устранена      ";
			ptrs[4]="  0[]  0< 0>:0=:0z  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],'z',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);

			}	
		//int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'&',0);

		int2lcd(av_head_int[0],'!',0);

		if(av_head[1]>=9)		sub_bgnd(" Z",'Z',0);
		int2lcd(av_head[1]+1,'Z',0);
		
		av_j_si_max=1;
		}
	else if((av_head[0]=='I')&&(av_head[2]=='T'))
		{
		ptrs[0]=" Авария инвертор NZ ";
		ptrs[1]=" перегрев (   !°С)  ";  
		ptrs[2]="  0%&  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    не устранена    ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="     устранена      ";
			ptrs[4]="  0[]  0< 0>:0=:0z  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],'z',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);

			}	
		//int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'&',0);

		int2lcd(av_head_int[0],'!',0);

		if(av_head[1]>=9)		sub_bgnd(" Z",'Z',0);
		int2lcd(av_head[1]+1,'Z',0);
		
		av_j_si_max=1;
		}

	else if((av_head[0]=='I')&&(av_head[2]=='C'))
		{
		ptrs[0]=" Авария инвертор NZ ";
		ptrs[1]=" разрыв связи       ";  
		ptrs[2]="  0%&  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    не устранена    ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="     устранена      ";
			ptrs[4]="  0[]  0< 0>:0=:0z  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],'z',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);

			}	
		//int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'&',0);

		int2lcd(av_head_int[0],'!',0);

		if(av_head[1]>=9)		sub_bgnd(" Z",'Z',0);
		int2lcd(av_head[1]+1,'Z',0);
		
		av_j_si_max=1;
		}

	else if((av_head[0]=='B')&&(av_head[2]=='N'))
		{  
		ptrs[0]=" Авария Uсети (  !В)";
		ptrs[1]="     байпас ф.Z     ";
		ptrs[2]="  0%&  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    не устранена    ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="     устранена      ";
			ptrs[4]="  0[]  0< 0>:0=:0z  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],'z',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);

			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'&',0);

		int2lcd(av_head_int[0],'!',0);

		if(av_head[1]==1)		sub_bgnd("A",'Z',0);
		else if(av_head[1]==2)	sub_bgnd("B",'Z',0);
		else if(av_head[1]==3)	sub_bgnd("C",'Z',0);
		else  					sub_bgnd(" байпас    ",'Z',-9);
		
		av_j_si_max=1;
		}

	else if((av_head[0]=='B')&&(av_head[2]=='O'))
		{  
		ptrs[0]=" Авария Uвых  (  !В)";
		ptrs[1]="     байпас ф.Z     ";
		ptrs[2]="  0%&  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    не устранена    ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="     устранена      ";
			ptrs[4]="  0[]  0< 0>:0=:0z  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],'z',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);

			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'&',0);

		int2lcd(av_head_int[0],'!',0);

		if(av_head[1]==1)		sub_bgnd("A",'Z',0);
		else if(av_head[1]==2)	sub_bgnd("B",'Z',0);
		else if(av_head[1]==3)	sub_bgnd("C",'Z',0);
		else  					sub_bgnd(" байпас    ",'Z',-9);
		
		av_j_si_max=1;
		}

	else if((av_head[0]=='B')&&(av_head[2]=='I'))
		{  
		ptrs[0]=" Авария Uинв  (  !В)";
		ptrs[1]="     байпас ф.Z     ";
		ptrs[2]="  0%&  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    не устранена    ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="     устранена      ";
			ptrs[4]="  0[]  0< 0>:0=:0z  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],'z',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);

			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'&',0);

		int2lcd(av_head_int[0],'!',0);

		if(av_head[1]==1)		sub_bgnd("A",'Z',0);
		else if(av_head[1]==2)	sub_bgnd("B",'Z',0);
		else if(av_head[1]==3)	sub_bgnd("C",'Z',0);
		else  					sub_bgnd(" байпас    ",'Z',-9);
		
		av_j_si_max=1;
		}

	else if((av_head[0]=='B')&&(av_head[2]=='T'))
		{  
		ptrs[0]=" Авария байпас ф.Z  ";
		ptrs[1]=" перегрев (   !°С)  ";
		ptrs[2]="  0%&  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    не устранена    ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="     устранена      ";
			ptrs[4]="  0[]  0< 0>:0=:0z  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],'z',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);

			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'&',0);

		int2lcd(av_head_int[0],'!',0);

		if(av_head[1]==1)		sub_bgnd("A",'Z',0);
		else if(av_head[1]==2)	sub_bgnd("B",'Z',0);
		else if(av_head[1]==3)	sub_bgnd("C",'Z',0);
		else  					sub_bgnd(" байпас    ",'Z',-9);
		
		av_j_si_max=1;
		}


	else if((av_head[0]=='B')&&(av_head[2]=='C'))
		{  
		ptrs[0]="       Авария       ";
		ptrs[1]="     батареи N+     ";
		ptrs[2]="  0%(  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    не устранена    ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="     устранена      ";
			ptrs[4]="  0[]  0< 0>:0=:0)  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);
			
						
			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		av_j_si_max=1;
		}

	else if((av_head[0]=='B')&&(av_head[2]=='S'))
		{  
		ptrs[0]="       Авария       ";
		ptrs[1]="    несимметрии     ";
		ptrs[2]="     батареи N+     ";
		ptrs[3]="  0%(  0^ 0@:0#:0$  ";
		bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		av_j_si_max=0;
		}

	else if((av_head[0]=='B')&&(av_head[2]=='Z'))
		{  
		ptrs[0]="   Выравнивающий    ";
		ptrs[1]="       заряд        ";
		ptrs[2]="  0%(  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    не завершен     ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="      завершен      ";
			ptrs[4]="  0[]  0< 0>:0=:0)  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);
			
						
			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		av_j_si_max=1;
		}



	else if((av_head[0]=='B')&&(av_head[2]=='W'))
		{  
		ptrs[0]="       Разряд       ";
		ptrs[1]="     батареи N!     ";
		ptrs[2]="   Начало           ";
		ptrs[3]="  0%(  0^ 0@:0#:0$  ";
		ptrs[4]="       Uбат=  <В";
		ptrs[5]="   Конец            ";
		ptrs[6]="  0qw  0r 0i:0l:0s  ";
		ptrs[7]="         Uбат=  >В  ";
		ptrs[8]="   Отдано    /а*ч.  ";
		
		bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
		
		int2lcd(av_head[1]+1,'!',0);

		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
				
		int2lcd(av_data_off[4],'i',0);
		int2lcd(av_data_off[5],'l',0);
		int2lcd(av_data_off[6],'s',0);
		int2lcd(av_data_off[2],'q',0);
		int2lcd(av_data_off[0],'r',0); 
		if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
		sub_bgnd(sm_mont[av_data_off[1]],'w',0);
		
		
		int2lcd(av_head_int[0]/10,'/',1);
		int2lcd(av_data_on[3]+(av_data_on[7]*256),'<',1);
		int2lcd(av_head_int[1],'>',1);	
		av_j_si_max=5;				

		
		}

	else if((av_head[0]=='B')&&(av_head[2]=='K'))
		{  
		ptrs[0]="  Контроль емкости  ";
		ptrs[1]="       батареи      ";
		ptrs[2]="   Начало           ";
		ptrs[3]="  0%(  0^ 0@:0#:0$  ";
		ptrs[4]="         Uбат=  <В  ";
		ptrs[5]="   Конец            ";
		ptrs[6]="  0qw  0r 0i:0l:0s  ";
		ptrs[7]="         Uбат=  >В  ";
		ptrs[8]="   Ёмкость   /а*ч.  ";
		
		bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
		
		int2lcd(av_head[1]+1,'!',0);

		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
				
		int2lcd(av_data_off[4],'i',0);
		int2lcd(av_data_off[5],'l',0);
		int2lcd(av_data_off[6],'s',0);
		int2lcd(av_data_off[2],'q',0);
		int2lcd(av_data_off[0],'r',0); 
		if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
		sub_bgnd(sm_mont[av_data_off[1]],'w',0);
		
		
		int2lcd(av_head_int[0],'/',1);
		int2lcd(av_data_on[3]+(av_data_on[7]*256),'<',1);
		int2lcd(av_head_int[1],'>',1);	
		av_j_si_max=5;				

		
		}



	else if((av_head[0]=='S')||(av_head[0]=='I'))
		{  
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
		else if(av_head[2]=='O')
			{
			ptrs[1]="    завышено Iвых   ";
			}		
		
		ptrs[2]="  0%(  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    не устранена    ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="      устранена     ";
			ptrs[4]="  0[]  0< 0>:0=:0)  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);
			
						
			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		}

	
	}
#ifndef _DEBUG_		 
else if(ind==iBatLog)
	{
	if(BAT_IS_ON[sub_ind1]==bisON)ptrs[0]=" Введена  0!/@  /0# ";
	else ptrs[0]=" Выведена 0!/@  /0# ";
     ptrs[1]=" Номин.емк.     $A*ч";
     ptrs[2]=" Наработка      %ч. ";
     ptrs[3]=" Контроль емкости   ";
     ptrs[4]=" Выравнивающий заряд";
     ptrs[5]=" Разряды            ";
     ptrs[6]=sm_exit;	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(	" БАТАРЕЙНЫЙ ЖУРНАЛ  ",
			"     БАТАРЕЯ N^     ",
			ptrs[index_set],
			ptrs[index_set+1]);
	pointer_set(2);	

	int2lcd(sub_ind1+1,'^',0); 
	int2lcd(BAT_DAY_OF_ON[sub_ind1],'!',0);
	sub_bgnd(sm_mont[BAT_MONTH_OF_ON[sub_ind1]],'@',0);
	int2lcd(BAT_YEAR_OF_ON[sub_ind1],'#',0); 
	int2lcd(BAT_C_NOM[sub_ind1],'$',0);
	int2lcd(BAT_RESURS[sub_ind1],'%',0);
	}

else if(ind==iBatLogKe)
	{             
	if(av_j_si_max==0)
		{
		bgnd_par(	"  КОНТРОЛИ ЕМКОСТИ  ",
				"     БАТАРЕИ N!     ",
				" Журнал пуст        ",
				sm_exit);
		pointer_set(3);
		sub_ind=0;
		index_set=0;
		} 
	else if(av_j_si_max==1)
		{
		bgnd_par(	"  КОНТРОЛИ ЕМКОСТИ  ",
				"     БАТАРЕИ N!     ",
				" (                  ",
				sm_exit);
		index_set=0;
		pointer_set(2);
		}	
	else
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>1) index_set=sub_ind-1;
		if(index_set==(av_j_si_max-1)) 
			{
			bgnd_par( "  КОНТРОЛИ ЕМКОСТИ  ",
					"     БАТАРЕИ N!     ",
					" (                  ",
					sm_exit);
			}
		else
			{
			bgnd_par(	"  КОНТРОЛИ ЕМКОСТИ  ",
					"     БАТАРЕИ N!     ",
					" (                  ",
					" [                  ");
			}
		pointer_set(2);			 
		}
		
   	int2lcd(sub_ind1+1,'!',0);
 	event_data2ind(content[index_set],'(');
 	event_data2ind(content[index_set+1],'[');
	}

else if(ind==iBatLogVz)
	{
	if(av_j_si_max==0)
		{
		bgnd_par(	"ВЫРАВНИВАЮЩИЕ ЗАРЯДЫ",
				"     БАТАРЕИ N!     ",
				" Журнал пуст        ",
				sm_exit);
		sub_ind=0;
		index_set=0;
		pointer_set(3);
		} 
	else if(av_j_si_max==1)
		{
		bgnd_par(	"ВЫРАВНИВАЮЩИЕ ЗАРЯДЫ",
				"     БАТАРЕИ N!     ",
				" (                  ",
				sm_exit);
		index_set=0;
		pointer_set(2);
		}	
	else
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>1) index_set=sub_ind-1;
		if(index_set==(av_j_si_max-1)) 
			{
			bgnd_par(	"ВЫРАВНИВАЮЩИЕ ЗАРЯДЫ",
					"     БАТАРЕИ N!     ",
					" (                  ",
					sm_exit);
			}

		else bgnd_par(	"ВЫРАВНИВАЮЩИЕ ЗАРЯДЫ",
					"     БАТАРЕИ N!     ",
					" (                  ",
					" [                  "); 
		pointer_set(2);			        
		}
   	int2lcd(sub_ind1+1,'!',0);
 	event_data2ind(content[index_set],'(');
 	event_data2ind(content[index_set+1],'[');
	
	}
   
else if(ind==iBatLogWrk)
	{
	if(av_j_si_max==0)
		{
		bgnd_par(	"      РАЗРЯДЫ       ",
				"     БАТАРЕИ N!     ",
				" Журнал пуст        ",
				sm_exit);
		sub_ind=0;
		index_set=0;
		pointer_set(3);
		} 
	else if(av_j_si_max==1)
		{
		bgnd_par(	"      РАЗРЯДЫ       ",
				"     БАТАРЕИ N!     ",
				" (                  ",
				sm_exit);
		index_set=0;
		pointer_set(2);
		}	

	else
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>1) index_set=sub_ind-1;
		if(index_set==(av_j_si_max-1))
			{
			bgnd_par(	"      РАЗРЯДЫ       ",
					"     БАТАРЕИ N!     ",
					" (                  ",
					sm_exit);
			}
		else bgnd_par(	"      РАЗРЯДЫ       ",
					"     БАТАРЕИ N!     ",
					" (                  ",
					" [                  ");

		pointer_set(2);
		}

   	int2lcd(sub_ind1+1,'!',0);
 	event_data2ind(content[index_set],'(');
 	event_data2ind(content[index_set+1],'[');

	

	} 
	
else if((ind==iSet_prl)||(ind==iK_prl)||(ind==iSpc_prl_vz)
	||(ind==iSpc_prl_ke)||(ind==iAusw_prl)||(ind==iPrltst))
	{
	bgnd_par("  Введите  пароль   ",sm_,sm_,sm_);
	int2lcdyx(parol[0],1,8,0);
     int2lcdyx(parol[1],1,9,0);
     int2lcdyx(parol[2],1,10,0);
     lcd_buffer[48+sub_ind]='¤';
	}	
		
else if(ind==iPrl_bat_in_out)
	{
	if(BAT_IS_ON[sub_ind1]==bisON)ptrs[0]="Для выведения бат.-и";
	else  ptrs[0]="Для введения батареи";
	bgnd_par(ptrs[0],"  наберите пароль   ",sm_,sm_);
	
     int2lcdyx(parol[0],2,8,0);
     int2lcdyx(parol[1],2,9,0);
     int2lcdyx(parol[2],2,10,0);
     lcd_buffer[68+sub_ind]='¤';	
	}

else if(ind==iLog_reset_prl)
	{
	ptrs[0]="Для очистки журнала ";
	bgnd_par(ptrs[0],"  введите  пароль   ",sm_,sm_);
	
     int2lcdyx(parol[0],2,8,0);
     int2lcdyx(parol[1],2,9,0);
     int2lcdyx(parol[2],2,10,0);
     lcd_buffer[68+sub_ind]='¤';	
	}	

else if(ind==iSet_INV)
	{
	ptrs[0]=		" Стандартные        ";
	ptrs[1]=		" Время и дата       ";
	ptrs[2]=		" Синхронизация      ";
	ptrs[3]=		"   времени и даты   ";
    ptrs[4]=		" Структура          ";
	ptrs[5]=		" Зв.сигн.   (       ";
	ptrs[6]=		" Отключение сигнала ";
	ptrs[7]=		"  аварии    )       ";
	ptrs[8]=		" Выходное напряжение";
	ptrs[9]=		" инвертора       !В ";
	ptrs[10]=		" Напряжение выхода  ";
	ptrs[11]=		" максимальное    @В ";
	ptrs[12]=		" Напряжение выхода  ";
	ptrs[13]=		" минимальное     #В ";
	ptrs[14]=		" Напряжение сети    ";
	ptrs[15]=		" включения       [В ";
	ptrs[16]=		" Напряжение сети    ";
	ptrs[17]=		" отключения      ]В ";
	ptrs[18]=		" Напряжение батареи ";
	ptrs[19]=		" включения       {В ";
	ptrs[20]=		" Напряжение батареи ";
	ptrs[21]=		" отключения      }В ";
	ptrs[22]=		" Ethernet           ";
	ptrs[23]=		" MODBUS ADRESS     <";
	ptrs[24]=		" MODBUS BAUDRATE    ";
	ptrs[25]=		"                  >0";
	ptrs[26]=      	" Реле               ";
	ptrs[27]=      	" Аварийные пороги   ";
	ptrs[28]=      	" байпас             ";
	ptrs[29]=		" Внешние датчики    ";
	ptrs[30]=      	" Серийный N        w";
    ptrs[31]=		" Выход              ";
    ptrs[32]=		" Калибровки         "; 
    ptrs[33]=		"                    ";        
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	
	if((index_set==9)||(index_set==13)||(index_set==17))
		{
		bgnd_par(	ptrs[index_set-1],
					ptrs[index_set],
					ptrs[index_set+1],
					ptrs[index_set+2]);	
		//pointer_set(0);
		if(sub_ind==index_set-1)lcd_buffer[0]=1;
		else lcd_buffer[40]=1;
		}
	else
		{
		bgnd_par(	"     УСТАНОВКИ      ",
					ptrs[index_set],
					ptrs[index_set+1],
					ptrs[index_set+2]);
		pointer_set(1);
		}
		
	
	if(ZV_ON)sub_bgnd("ВКЛ.",'(',0);
	else sub_bgnd("ВЫК.",'(',0);
	if(AV_OFF_AVT)sub_bgnd("автом.",')',0);
	else sub_bgnd("ручн.",')',0);
	int2lcd(MODBUS_ADRESS,'<',0);
	int2lcd(MODBUS_BAUDRATE,'>',0);
	int2lcd(U_OUT_SET,'!',0);
	int2lcd(U_OUT_MAX,'@',0);
	int2lcd(U_OUT_MIN,'#',0);
	int2lcd(U_NET_MAX,'[',0);
	int2lcd(U_NET_MIN,']',0);
	int2lcd(U_BAT_MAX,'{',0);
	int2lcd(U_BAT_MIN,'}',0);
	/*
	int2lcdyx(sub_ind,0,2,0);	
	int2lcdyx(index_set,0,4,0);
	int2lcdyx(sub_ind1,0,6,0);*/

	//long2lcd_mmm(AUSW_MAIN_NUMBER,'w',0);
	serial2lcd(AUSW_MAIN_NUMBER,'w',sub_ind1);	
	}



else if (ind==iDef)

	{ 
	ptrs[0]=" Инвертор 24В       ";
	ptrs[1]=" Инвертор 48(60)В   ";
	ptrs[2]=" Инвертор 110В      ";
	ptrs[3]=" Инвертор 220В      ";
	ptrs[4]=sm_exit;
	if(bFL5)ptrs[default_temp]=sm_;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

    bgnd_par("СТАНДАРТНЫЕ УСТ.-КИ ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1); 
	} 


else if(ind==iSet_T)
	{
	static char phase_cnt;
	if(++phase_cnt>=15)
	     {
	     phase_cnt=0;
	     if(++phase>=3)phase=0;
	     }
	ptrs[0]=sm_time;
	ptrs[1]=sm_;
	if(phase==0)ptrs[2]="     <> - выбор     ";
     if(phase==1)ptrs[2]="   ^v - установка   ";
     if(phase==2)ptrs[2]="     ¤  - выход     ";
	
	bgnd_par(" УСТАНОВКА  ВРЕМЕНИ ",ptrs[0],ptrs[1],ptrs[2]);
     if(sub_ind==0)lcd_buffer[42]='^';
     else if(sub_ind==1)lcd_buffer[45]='^';
     else if(sub_ind==2)lcd_buffer[48]='^';
     else if(sub_ind==3)lcd_buffer[51]='^';
     else if(sub_ind==4)lcd_buffer[54]='^';
     else if(sub_ind==5)lcd_buffer[58]='^';
  
 	int2lcd(LPC_RTC->SEC,'&',0);
 	int2lcd(LPC_RTC->MIN,'^',0);
 	int2lcd(LPC_RTC->HOUR,'%',0);
 	
 	int2lcd(LPC_RTC->DOM,'<',0);
 	sub_bgnd(sm_mont[LPC_RTC->MONTH],'>',0);
 	int2lcd(LPC_RTC->YEAR,'{',0);
 	if(bFL2)
 	     {
 	     lcd_buffer[find(':')]=' ';
 	     lcd_buffer[find(':')]=' ';
 	     }  
	}  
/*
else if(ind==iSet_T_avt)
	{
	if(SNTP_ENABLE==0)		ptrs[0]=	" Выключено          ";
	else if(SNTP_ENABLE==1)	ptrs[0]=	" Период        1 час";
	else if(SNTP_ENABLE==2)	ptrs[0]=	" Период      1 сутки";
	else if(SNTP_ENABLE==3)	ptrs[0]=	" Период     1 неделя";

	if(SNTP_ENABLE==0)
		{
							ptrs[1]=	sm_exit;
							ptrs[2]=	sm_;
							ptrs[3]=	sm_;
		}
	else 
		{
		if((SNTP_GMT>=0)&&(SNTP_GMT<=9))			ptrs[1]=	" Часовой пояс GMT+! ";
		else if((SNTP_GMT>=10)&&(SNTP_GMT<=13))		ptrs[1]=	" Часовой пояс GMT+ !";
		else if((SNTP_GMT<0)&&(SNTP_GMT>-10))		ptrs[1]=	" Часовой пояс GMT-! ";
		else if((SNTP_GMT<=-10)&&(SNTP_GMT>=-12))	ptrs[1]=	" Часовой пояс GMT- !";
													ptrs[2]=	sm_exit;
													ptrs[3]=	sm_;
		}
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>1) index_set=sub_ind-1;
	
	bgnd_par(	"    СИНХРОНИЗАЦИЯ   ",
				"    ВРЕМЕНИ (SNTP)  ",
				ptrs[index_set],
				ptrs[index_set+1]);
  
 	int2lcd(abs(SNTP_GMT),'!',0);

	pointer_set(2);
	}  
*/

else if(ind==iSet_T_avt)
	{
	if(SNTP_ENABLE==0)		ptrs[0]=	" Выключено          ";
	else if(SNTP_ENABLE==1)	ptrs[0]=	" Период        1 час";
	else if(SNTP_ENABLE==2)	ptrs[0]=	" Период      1 сутки";
	else if(SNTP_ENABLE==3)	ptrs[0]=	" Период     1 неделя";

	if(SNTP_ENABLE==0)
		{
							ptrs[1]=	sm_exit;
							ptrs[2]=	sm_;
							ptrs[3]=	sm_;
		}
	else 
		{
		if((SNTP_GMT>=0)&&(SNTP_GMT<=9))			ptrs[1]=	" Часовой пояс GMT+! ";
		else if((SNTP_GMT>=10)&&(SNTP_GMT<=13))		ptrs[1]=	" Часовой пояс GMT+ !";
		else if((SNTP_GMT<0)&&(SNTP_GMT>-10))		ptrs[1]=	" Часовой пояс GMT-! ";
		else if((SNTP_GMT<=-10)&&(SNTP_GMT>=-12))	ptrs[1]=	" Часовой пояс GMT- !";
		}
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>1) index_set=sub_ind-1;

	if(lc640_read_int(EE_SNTP_WEB_ENABLE)==1)
		{
		ptrs[2]=	" Синхронизация через";
		ptrs[3]=	"     ИНТЕРНЕТ       ";
		}
	else
		{
		ptrs[2]=	" Синхронизация через";
		ptrs[3]=	" IP 000.000.000.00# ";
		}
	ptrs[4]=	" Синхронизировать   ";
	ptrs[5]=	sm_exit;
	ptrs[6]=	sm_;
	
	bgnd_par(	"    СИНХРОНИЗАЦИЯ   ",
				"    ВРЕМЕНИ (SNTP)  ",
				ptrs[index_set],
				ptrs[index_set+1]);
  
 	int2lcd(abs(SNTP_GMT),'!',0);
	if(sub_ind==2)		ip2lcd(lc640_read_int(EE_SNTP_IP1),lc640_read_int(EE_SNTP_IP2),lc640_read_int(EE_SNTP_IP3),lc640_read_int(EE_SNTP_IP4),'#',(sub_ind1+1));
	else 				ip2lcd(lc640_read_int(EE_SNTP_IP1),lc640_read_int(EE_SNTP_IP2),lc640_read_int(EE_SNTP_IP3),lc640_read_int(EE_SNTP_IP4),'#',0);

	pointer_set(2);
	//int2lcdyx(udp_callback_cnt,0,3,0);
	
	}  

else if(ind==iStr_INV)
	{
	ptrs[0]=" Инверторов        ^";	
	ptrs[1]=" Байпасов          [";
	if(NUMBYPASS==1) ptrs[1]=" Байпас            [";
	if(NUMBYPASS!=0)
		{
		ptrs[2]=" Сухих контактов   $";
		ptrs[3]=" Выход              ";
		}
	else
		{
		ptrs[2]=" Выходных фаз      <";
		ptrs[3]=" Вход AC         >  ";
		ptrs[4]=" Сухих контактов   $";
		ptrs[5]=" Выход              ";
		}

	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
	bgnd_par("      СТРУКТУРА     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	 
	int2lcd(NUMINV,'^',0);
	if(NUMBYPASS==0) int2lcd(NUMBYPASS,'[',0);
	else if(NUMPHASE==1) sub_bgnd("1ф.",'[',-2);
	else if(NUMPHASE==3) sub_bgnd("3ф.",'[',-2); 
	int2lcd(NUMPHASE,'<',0);
	if(NUMINAC==1)	sub_bgnd("есть",'>',-1);
	else 			sub_bgnd("нет",'>',-1);
	int2lcd(NUMSK,'$',0);
	

	}

else if (ind==iExt_set)
	{ 
	ptrs[0]=		" Сухой контакт N1   ";
	ptrs[1]=		" Сухой контакт N2   ";
	ptrs[2]=		" Сухой контакт N3   ";
	ptrs[3]=		" Сухой контакт N4   ";
	ptrs[NUMSK]=sm_exit;
	ptrs[1+NUMSK]=	"                    ";
	ptrs[2+NUMSK]=	"                    ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("  ВНЕШНИЕ ДАТЧИКИ   ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	}

else if (ind==iLan_set)
	{
	char sss[10]="abcdef";
	char i/*,i_flag*/;
	 
	ptrs[0]=	" Ethernet         ! ";
	ptrs[1]=	" DHCPклиент       @ ";
	ptrs[2]=	" IPадрес            ";
	ptrs[3]=	"  000.000.000.00#   ";
	ptrs[4]=	" Маска подсети      ";
	ptrs[5]=	"  000.000.000.00$   ";
	ptrs[6]=	" Шлюз               ";
	ptrs[7]=	"  000.000.000.00)   ";
	ptrs[8]=	" Порт.чтения       [";
	ptrs[9]=	" Порт.записи       ]";
	ptrs[10]=	" Community <        ";
	ptrs[11]=	" Адресат для TRAP N1";
	ptrs[12]=	"  000.000.000.00%   ";
	ptrs[13]=	" Адресат для TRAP N2";
	ptrs[14]=	"  000.000.000.00^   ";
	ptrs[15]=	" Адресат для TRAP N3";
	ptrs[16]=	"  000.000.000.00&   ";
	ptrs[17]=	" Адресат для TRAP N4";
	ptrs[18]=	"  000.000.000.00*   ";
	ptrs[19]=	" Адресат для TRAP N5";
	ptrs[20]=	"  000.000.000.00(   ";
	ptrs[21]=	" Выход              ";

	
	if(!ETH_IS_ON)
		{
		ptrs[1]=" Выход              ";
		ptrs[2]="                    ";
		ptrs[3]="                    ";
		}

	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

     bgnd_par(	" УСТАНОВКИ Ethernet ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);
	
	pointer_set(1);
     if(ETH_IS_ON)
     	{
     	sub_bgnd("ВКЛ.",'!',-3);   
     	}
     else 
     	{
     	sub_bgnd("ВЫКЛ.",'!',-4);   
     	}

     if(ETH_DHCP_ON)
     	{
     	sub_bgnd("ВКЛ.",'@',-3);   
     	}
     else 
     	{
     	sub_bgnd("ВЫКЛ.",'@',-4);   
     	}
		  
	if(sub_ind==2)	ip2lcd(ETH_IP_1,ETH_IP_2,ETH_IP_3,ETH_IP_4,'#',(sub_ind1+1));
	else ip2lcd(ETH_IP_1,ETH_IP_2,ETH_IP_3,ETH_IP_4,'#',0);
	if(sub_ind==4)	ip2lcd(ETH_MASK_1,ETH_MASK_2,ETH_MASK_3,ETH_MASK_4,'$',(sub_ind1+1));
	else ip2lcd(ETH_MASK_1,ETH_MASK_2,ETH_MASK_3,ETH_MASK_4,'$',0);
	if(sub_ind==6)	ip2lcd(ETH_GW_1,ETH_GW_2,ETH_GW_3,ETH_GW_4,')',(sub_ind1+1));
	else ip2lcd(ETH_GW_1,ETH_GW_2,ETH_GW_3,ETH_GW_4,')',0);

	int2lcd(ETH_SNMP_PORT_READ,'[',0);
	int2lcd(ETH_SNMP_PORT_WRITE,']',0);

	if( (ETH_TRAP1_IP_1==255) && (ETH_TRAP1_IP_2==255) && (ETH_TRAP1_IP_3==255) && (ETH_TRAP1_IP_4==255) ) sub_bgnd("    неактивен    ",'%',-14);
	else
		{
		if(sub_ind==11)	ip2lcd(ETH_TRAP1_IP_1,ETH_TRAP1_IP_2,ETH_TRAP1_IP_3,ETH_TRAP1_IP_4,'%',(sub_ind1+1));
		else ip2lcd(ETH_TRAP1_IP_1,ETH_TRAP1_IP_2,ETH_TRAP1_IP_3,ETH_TRAP1_IP_4,'%',0);
		}

	if( (ETH_TRAP2_IP_1==255) && (ETH_TRAP2_IP_2==255) && (ETH_TRAP2_IP_3==255) && (ETH_TRAP2_IP_4==255) ) sub_bgnd("    неактивен    ",'^',-14);
	else
		{
		if(sub_ind==13)	ip2lcd(ETH_TRAP2_IP_1,ETH_TRAP2_IP_2,ETH_TRAP2_IP_3,ETH_TRAP2_IP_4,'^',(sub_ind1+1));
		else ip2lcd(ETH_TRAP2_IP_1,ETH_TRAP2_IP_2,ETH_TRAP2_IP_3,ETH_TRAP2_IP_4,'^',0);
		}

	if( (ETH_TRAP3_IP_1==255) && (ETH_TRAP3_IP_2==255) && (ETH_TRAP3_IP_3==255) && (ETH_TRAP3_IP_4==255) ) sub_bgnd("    неактивен    ",'&',-14);
	else
		{
		if(sub_ind==15)	ip2lcd(ETH_TRAP3_IP_1,ETH_TRAP3_IP_2,ETH_TRAP3_IP_3,ETH_TRAP3_IP_4,'&',(sub_ind1+1));
		else ip2lcd(ETH_TRAP3_IP_1,ETH_TRAP3_IP_2,ETH_TRAP3_IP_3,ETH_TRAP3_IP_4,'&',0);
		}

	if( (ETH_TRAP4_IP_1==255) && (ETH_TRAP4_IP_2==255) && (ETH_TRAP4_IP_3==255) && (ETH_TRAP4_IP_4==255) ) sub_bgnd("    неактивен    ",'*',-14);
	else
		{
		if(sub_ind==17)	ip2lcd(ETH_TRAP4_IP_1,ETH_TRAP4_IP_2,ETH_TRAP4_IP_3,ETH_TRAP4_IP_4,'*',(sub_ind1+1));
		else ip2lcd(ETH_TRAP4_IP_1,ETH_TRAP4_IP_2,ETH_TRAP4_IP_3,ETH_TRAP4_IP_4,'*',0);
		}

	if( (ETH_TRAP5_IP_1==255) && (ETH_TRAP5_IP_2==255) && (ETH_TRAP5_IP_3==255) && (ETH_TRAP5_IP_4==255) ) sub_bgnd("    неактивен    ",'(',-14);
	else
		{
		if(sub_ind==19)	ip2lcd(ETH_TRAP5_IP_1,ETH_TRAP5_IP_2,ETH_TRAP5_IP_3,ETH_TRAP5_IP_4,'(',(sub_ind1+1));
		else ip2lcd(ETH_TRAP5_IP_1,ETH_TRAP5_IP_2,ETH_TRAP5_IP_3,ETH_TRAP5_IP_4,'(',0);
		}

/*	if((sub_ind==2)&&(sub_ind1==0)&&(bFL2))
		{
		sub_bgnd("   ",'#',-2);
		}
	else int2lcd(ETH_IP_1,'#',0);

	if((sub_ind==2)&&(sub_ind1==1)&&(bFL2))
		{
		sub_bgnd("   ",'$',-2);
		}
	else int2lcd(ETH_IP_2,'$',0);

	if((sub_ind==2)&&(sub_ind1==2)&&(bFL2))
		{
		sub_bgnd("   ",'%',-2);
		}
	else int2lcd(ETH_IP_3,'%',0);

	if((sub_ind==2)&&(sub_ind1==3)&&(bFL2))
		{
		sub_bgnd("   ",'^',-2);
		}
	else int2lcd(ETH_IP_4,'^',0);*/


	//int2lcdyx(sub_ind,0,1,0);	
	//int2lcdyx(index_set,0,3,0);
	//int2lcdyx(sub_ind1,0,5,0);
	//for(i=0;(i<9)&&(snmp_community[i]))

	for(i=0;i<9;i++)
		{
		sss[i]=snmp_community[i];
		}
	sss[9]=0;		

	if(sub_ind==10)community2lcd(sss,'<',sub_ind1,1);
	else community2lcd(sss,'<',sub_ind1,0);
	
	//int2lcdyx(snmp_community[0],0,4,0);
	//int2lcdyx(snmp_community[11],0,9,0);
	//int2lcdyx(snmp_community[2],0,14,0);
	//int2lcdyx(snmp_community[sub_ind1],0,19,0);	
	}

else if (ind==iApv)
	{ 
	ptrs[0]=			" АПВ 1й уровень !   ";
	if(APV_ON1!=apvON)
	     {
	     ptrs[1]=		" Выход              ";
	     ptrs[2]=sm_;
	     ptrs[3]=sm_;
	     ptrs[4]=sm_;
	     simax=1;
	     }
	else
	     {
	     if(APV_ON2!=apvON)
	          {
	          ptrs[1]=" АПВ 2й уровень @   ";
	          ptrs[2]=" Выход              ";
	          ptrs[3]=sm_;
	          ptrs[4]=sm_;
	          simax=2;
	          }
	     else 
	          {
               ptrs[1]=" АПВ 2й уровень @   ";
	          ptrs[2]=" Период АПВ2     #ч.";
	          ptrs[3]=" Выход              ";
	          ptrs[4]=sm_;
	          simax=3;	          
	          }     
	     }     
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;	
     bgnd_par("   АПВ ИСТОЧНИКОВ   ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	if(APV_ON1==apvON)sub_bgnd("ВКЛ.",'!',0);
	else sub_bgnd("ВЫКЛ.",'!',-1);
	
	if(APV_ON2==apvON)
	     {
	     sub_bgnd("ВКЛ.",'@',0);
	     int2lcd(APV_ON2_TIME,'#',0);
	     }
	else sub_bgnd("ВЫКЛ.",'@',-1);	
     
 	} 
/*
     ptrs[0+NUMDT]=  	" СК1        $       ";            
     ptrs[1+NUMDT]=  	" СК2        %       ";
	ptrs[2+NUMDT]=  	" СК3        ^       ";
	ptrs[3+NUMDT]=  	" СК4        &       ";
	ptrs[0+NUMEXT]=  	" Выход              ";
	ptrs[1+NUMEXT]=  	"                    ";
	ptrs[2+NUMEXT]=  	"                    ";

	bgnd_par(		"  ВНЕШНИЕ ДАТЧИКИ   ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

*/

else if (ind==iExt_set)
	{ 
//	ptrs[0]=			" Датчик темпер. N1  ";
//	ptrs[1]=			" Датчик темпер. N2  ";
//	ptrs[2]=			" Датчик темпер. N3  ";
	ptrs[0]=		" Датчик двери       ";
	ptrs[1]=		" Датчик дыма        ";
	ptrs[2]=		" Датчик удара       ";
//	ptrs[3]=		" Датчик переворачив.";
	ptrs[3]=  	" Выход              ";
	ptrs[4]=  	"                    ";
	ptrs[5]=  	"                    ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("     УСТАНОВКИ      ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	}

else if (ind==iExt_set_3U)
	{ 
	ptrs[0]=		" Сухой контакт №1   ";
	ptrs[1]=		" Сухой контакт №2   ";
	ptrs[2]=		" Сухой контакт №3   ";
	ptrs[3]=		" Сухой контакт №4   ";
	ptrs[NUMSK]=  	" Выход              ";
	ptrs[NUMSK+1]= "                    ";
	ptrs[NUMSK+2]=	"                    ";
	ptrs[NUMSK+3]=	"                    ";
		
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("     УСТАНОВКИ      ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	}


	
else if (ind==iExt_dt)
	{ 
	ptrs[0]=" температура     @°C";
	ptrs[1]=" tmax            #°C";
	ptrs[2]=" tmin            $°C";
	ptrs[3]=" Реле            [  ";
	ptrs[4]=" Звук            ]  ";
	ptrs[5]=" Дисплей         (  ";
	ptrs[6]=" RS232           )  ";
	ptrs[7]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>1) index_set=sub_ind-1;
     bgnd_par("  ВНЕШНИЙ ДАТЧИК    ","   ТЕМПЕРАТУРЫ N!   ",ptrs[index_set],ptrs[index_set+1]);
	
	pointer_set(2);
	int2lcd(sub_ind1+1,'!',0);
	int2lcd_mmm(t_ext[sub_ind1],'@',0);
	if(!TMAX_EXT_EN[sub_ind1])int2lcd_mmm(TMAX_EXT[sub_ind1],'#',0);
	else sub_bgnd("выкл.",'#',-2);
	if(!TMIN_EXT_EN[sub_ind1])int2lcd_mmm(TMIN_EXT[sub_ind1],'$',0);
	else sub_bgnd("выкл.",'$',-2);
	if(!T_EXT_REL_EN[sub_ind1])sub_bgnd("вкл.",'[',-2);
	else sub_bgnd("выкл.",'[',-2);
	if(!T_EXT_ZVUK_EN[sub_ind1])sub_bgnd("вкл.",']',-2);
	else sub_bgnd("выкл.",']',-2);
	if(!T_EXT_LCD_EN[sub_ind1])sub_bgnd("вкл.",'(',-2);
	else sub_bgnd("выкл.",'(',-2);
	if(!T_EXT_RS_EN[sub_ind1])sub_bgnd("вкл.",')',-2);
	else sub_bgnd("выкл.",')',-2);	
	
	//int2lcdyx(sub_ind,0,1,0);	
	//int2lcdyx(index_set,0,3,0);
	}	
else if (ind==iExt_sk)
	{ 
	ptrs[0]=" состояние - @      ";
	ptrs[1]=" аварийное          ";
	ptrs[2]=" состояние - #      ";
/*	ptrs[3]=" Реле            [  ";
	ptrs[4]=" Звук            ]  ";
	ptrs[5]=" Дисплей         (  ";
	ptrs[6]=" RS232           )  ";*/
	ptrs[3]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("  СУХОЙ КОНТАКТ N!  ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	int2lcd(sub_ind1+1,'!',0);
	if(sk_stat[sub_ind1]==ssON)sub_bgnd("замкнут",'@',0);
	else sub_bgnd("разомк.",'@',0);
	if(!SK_SIGN[sub_ind1])sub_bgnd("замкнут",'#',0);
	else sub_bgnd("незамк.",'#',0);
/*	if(!TMIN_EXT_EN[sub_ind1])int2lcd_mmm(TMIN_EXT[sub_ind1],'$',0);
	else sub_bgnd("выкл.",'$',-6);
	if(!SK_REL_EN[sub_ind1])sub_bgnd("вкл.",'[',-2);
	else sub_bgnd("выкл.",'[',-2);
	if(!SK_ZVUK_EN[sub_ind1])sub_bgnd("вкл.",']',-2);
	else sub_bgnd("выкл.",']',-2);
	if(!SK_LCD_EN[sub_ind1])sub_bgnd("вкл.",'(',-2);
	else sub_bgnd("выкл.",'(',-2);
	if(!SK_RS_EN[sub_ind1])sub_bgnd("вкл.",')',-2);
	else sub_bgnd("выкл.",')',-2);				   */
	
	//int2lcdyx(sub_ind,0,1,0);	
	//int2lcdyx(index_set,0,3,0);
	}		

else if (ind==iExt_sk_3U)
	{ 
	ptrs[0]=" состояние - @      ";
	ptrs[1]=" аварийное          ";
	ptrs[2]=" состояние - #      ";
	ptrs[3]=" Звук            ]  ";
	ptrs[4]=" Дисплей         (  ";
	ptrs[5]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("  СУХОЙ КОНТАКТ N!  ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	int2lcd(sub_ind1+1,'!',0);
	if(sk_stat[sub_ind1]==ssON)sub_bgnd("замкнут",'@',0);
	else sub_bgnd("разомк.",'@',0);
	if(!SK_SIGN[sub_ind1])sub_bgnd("замкнут",'#',0);
	else sub_bgnd("незамк.",'#',0);
	if(!SK_ZVUK_EN[sub_ind1])sub_bgnd("вкл.",']',-2);
	else sub_bgnd("выкл.",']',-2);
	if(!SK_LCD_EN[sub_ind1])sub_bgnd("вкл.",'(',-2);
	else sub_bgnd("выкл.",'(',-2);
	}		

		

else if (ind==iExt_ddv)
	{ 
	ptrs[0]=" состояние - @      ";
	ptrs[1]=" открытое состояние ";
	ptrs[2]=" двери     - #      ";
	ptrs[3]=" Реле            [  ";
	ptrs[4]=" Дисплей         (  ";
	ptrs[5]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("   ДАТЧИК ДВЕРИ     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	
	if(sk_stat[0]==ssON)sub_bgnd("замкнут",'@',0);
	else sub_bgnd("разомк.",'@',0);
	if(!SK_SIGN[0])sub_bgnd("замкнут",'#',0);
	else sub_bgnd("незамк.",'#',0);
	if(SK_REL_EN[0])sub_bgnd("вкл.",'[',-2);
	else sub_bgnd("выкл.",'[',-2);
     if(SK_LCD_EN[0])sub_bgnd("вкл.",'(',-2);
	else sub_bgnd("выкл.",'(',-2);
	
	}	

else if (ind==iExt_ddi)
	{ 
	ptrs[0]=" состояние - @      ";
	ptrs[1]=" аварийное          ";
	ptrs[2]=" состояние - #      ";
	ptrs[3]=" Реле            [  ";
	ptrs[4]=" Дисплей         (  ";
	ptrs[5]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("   ДАТЧИК ДЫМА      ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	
	if(sk_stat[1]==ssON)sub_bgnd("замкнут",'@',0);
	else sub_bgnd("разомк.",'@',0);
	if(!SK_SIGN[1])sub_bgnd("замкнут",'#',0);
	else sub_bgnd("незамк.",'#',0);
	if(SK_REL_EN[1])sub_bgnd("вкл.",'[',-2);
	else sub_bgnd("выкл.",'[',-2);
	if(SK_LCD_EN[1])sub_bgnd("вкл.",'(',-2);
	else sub_bgnd("выкл.",'(',-2);
	}	

else if (ind==iExt_dud)
	{ 
	ptrs[0]=" состояние - @      ";
	ptrs[1]=" аварийное          ";
	ptrs[2]=" состояние - #      ";
	ptrs[3]=" Реле            [  ";
	ptrs[4]=" Дисплей         (  ";
	ptrs[5]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("   ДАТЧИК УДАРА     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	
	if(sk_stat[2]==ssON)sub_bgnd("замкнут",'@',0);
	else sub_bgnd("разомк.",'@',0);
	if(!SK_SIGN[2])sub_bgnd("замкнут",'#',0);
	else sub_bgnd("незамк.",'#',0);
	if(SK_REL_EN[2])sub_bgnd("вкл.",'[',-2);
	else sub_bgnd("выкл.",'[',-2);
	if(SK_LCD_EN[2])sub_bgnd("вкл.",'(',-2);
	else sub_bgnd("выкл.",'(',-2);
	}


else if (ind==iExt_dp)
	{ 
	ptrs[0]=" состояние - @      ";
	ptrs[1]=" аварийное          ";
	ptrs[2]=" состояние - #      ";
	ptrs[3]=" Реле            [  ";
     ptrs[4]=" Дисплей         (  ";
	ptrs[5]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("ДАТЧИК ПЕРЕВОРАЧИВ. ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);

	if(sk_stat[3]==ssON)sub_bgnd("замкнут",'@',0);
	else sub_bgnd("разомк.",'@',0);
	if(!SK_SIGN[3])sub_bgnd("замкнут",'#',0);
	else sub_bgnd("незамк.",'#',0);
	if(!SK_REL_EN[3])sub_bgnd("вкл.",'[',-2);
	else sub_bgnd("выкл.",'[',-2);
	if(!SK_LCD_EN[3])sub_bgnd("вкл.",'(',-2);
	else sub_bgnd("выкл.",'(',-2);


    /* int2lcdyx(sk_stat[0],0,2,0);
     int2lcdyx(sk_stat[1],0,5,0);
     int2lcdyx(sk_stat[2],0,8,0);
     int2lcdyx(sk_stat[3],0,11,0);*/
	}

else if(ind==iRele_set_sel)
	{
	ptrs[0]=				" Реле N1            ";
    ptrs[1]=				" Реле N2            ";
	ptrs[2]=				" Выход              ";
	ptrs[3]=				"                    ";
	ptrs[4]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(	"   НАСТРОЙКА РЕЛЕ   ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);
	pointer_set(1);
	
     }

else if(ind==iRele_set)
	{
	
	ptrs[0]=	" Авария инвертора  @";
    ptrs[1]=	" Авария DC         #";
	ptrs[2]=	" Авария Uвых       $"; 
    ptrs[3]=	" Авария Uвх        %";
    ptrs[4]=	" Состояние входного ";
	ptrs[5]=	" селектора (AC/DC) ^";
	ptrs[6]=	" Активное состояние ";
	ptrs[7]=	"  реле             &";
    ptrs[8]=	sm_exit;
    ptrs[9]=	sm_;
    ptrs[10]=	sm_;     	     	    
	

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	
	bgnd_par(	" Реле N! срабатыв. ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

	pointer_set(1);	

	int2lcd(sub_ind1+1,'!',0); 
 	checkboxing('@',RELE_SET_MASK[sub_ind1]&(1<<0));
	checkboxing('#',RELE_SET_MASK[sub_ind1]&(1<<1));
 	checkboxing('$',RELE_SET_MASK[sub_ind1]&(1<<2));
	checkboxing('%',RELE_SET_MASK[sub_ind1]&(1<<3));
 	checkboxing('^',RELE_SET_MASK[sub_ind1]&(1<<4));
//	checkboxing('[',RELE_SET_MASK[sub_ind1]&(1<<5));
//	checkboxing('{',RELE_SET_MASK[sub_ind1]&(1<<6));
//	checkboxing('}',RELE_SET_MASK[sub_ind1]&(1<<7));
	if(RELE_SET_MASK[sub_ind1]&(1<<15))		sub_bgnd("ВКЛ.",'&',-3);
	else 									sub_bgnd("ВЫКЛ.",'&',-4);
	
	//int2lcdyx(lc640_read_int(ADR_EE_RELE_SET_MASK[sub_ind1]),0,19,0);
	//int2lcdyx(RELE_SET_MASK[sub_ind1],0,12,0);
	//int2lcdyx(sub_ind1,0,2,0);  
	}

else if(ind==iByps_av_set)
	{
	ptrs[0]=		" Uвых.AC.max.    !В ";
	ptrs[1]=		" Uвых.AC.min.    @В ";
	ptrs[2]=		" Uвх.AC.max.     #В ";
	ptrs[3]=		" Uвх.AC.min.     $В ";
	ptrs[4]=		" Uвх.DC.max.     %В ";
	ptrs[5]=		" Uвх.DC.min.     ^В ";
    ptrs[6]=		" Выход              ";
    ptrs[7]=		"                    "; 
    ptrs[8]=		"                    ";        
	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	
	bgnd_par(	"   ПОРОГИ АВАРИЙ    ",
				"      БАЙПАСС       ",
				ptrs[index_set],
				ptrs[index_set+1]);
	pointer_set(2);

	int2lcd(U_OUT_AC_MAX_AV,'!',0);
	int2lcd(U_OUT_AC_MIN_AV,'@',0);
	int2lcd(U_IN_AC_MAX_AV,'#',0);
	int2lcd(U_IN_AC_MIN_AV,'$',0);
	int2lcd(U_IN_DC_MAX_AV,'%',0);
	int2lcd(U_IN_DC_MIN_AV,'^',0);
	
	}

else if(ind==iK)
	{
	char i;
	i=0;
	
	ptrs[i++]=" Сеть               ";
	if(NUMBAT)
     ptrs[i++]=" Батареи            ";
	if(NUMIST)
	ptrs[i++]=" БПС                ";
	if(NUMINV)
     ptrs[i++]=" Инверторы          ";
	ptrs[i++]=" Нагрузка           ";
     ptrs[i++]=" Внешние датчики    ";
     ptrs[i++]=" Силовые вводы      ";
     ptrs[i++]=" Выход              ";
     ptrs[i++]="                    ";
     ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}    	

else if(ind==iK_RSTKM)
	{
	char i;
	i=0;
	
	ptrs[i++]=" Сеть               ";
	if(NUMBAT)
     ptrs[i++]=" Батареи            ";
	if(NUMIST)
	ptrs[i++]=" БПС                ";
	if(NUMINV)
     ptrs[i++]=" Инверторы          ";
	ptrs[i++]=" Нагрузка           ";
     ptrs[i++]=" Внешние датчики    ";
     ptrs[i++]=" Силовые вводы      ";
     ptrs[i++]=" Выход              ";
     ptrs[i++]="                    ";
     ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}    

else if(ind==iK_3U)
	{
	char i;
	i=0;
	ptrs[i++]=" Сеть               ";
	if(NUMBAT)
     ptrs[i++]=" Батареи            ";
	if(NUMIST)
	ptrs[i++]=" БПС                ";
	if(NUMINV)
     ptrs[i++]=" Инверторы          ";
	ptrs[i++]=" Нагрузка           ";
     ptrs[i++]=" Внешн.датч.темпер. ";
     ptrs[i++]=" Выход              ";
     ptrs[i++]="                    ";
     ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}    	

  

else if(ind==iK_INV)
	{
	char i;
	i=0;

	if(NUMINV)
    ptrs[i++]=" Инверторы          ";
	if(NUMBYPASS>1)
    ptrs[i++]=" Байпасы            ";
	else if(NUMBYPASS)
    ptrs[i++]=" Байпас             ";

	ptrs[i++]=" Udc.вх.       #В   ";
	ptrs[i++]=  " tшкаф.        >°С  ";
    ptrs[i++]=" Выход              ";
	ptrs[i++]=" Кварц RS485   !МГЦ ";
	ptrs[i++]="                    ";
	ptrs[i++]="                    ";
	ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	int2lcd(RS485_QWARZ_DIGIT,'!',0);
	int2lcd(dcin_U,'#',1);
     if(ND_EXT[0])sub_bgnd("неиспр.",'>',-3);
     else int2lcd_mmm(t_ext[0],'>',0);

	pointer_set(1);

	//int2lcdyx(adc_buff_[2],0,4,0);
	//int2lcdyx(Kudcin,0,10,0);
	}    	


else if(ind==iK_6U)
	{
	char i;
	i=0;
	
	ptrs[i++]=" Сеть               ";
	if(NUMBAT)
     ptrs[i++]=" Батареи            ";
	if(NUMIST)
	ptrs[i++]=" БПС                ";
	if(NUMINV)
     ptrs[i++]=" Инверторы          ";
	if(NUMBYPASS)
     ptrs[i++]=" Байпасс            ";
	ptrs[i++]=" Нагрузка           ";
     if(NUMDT)
     ptrs[i++]=" Внешние датчики    ";
     if(NUMMAKB)
     ptrs[i++]=" Мониторы АКБ       ";
     ptrs[i++]=" Выход              ";
     ptrs[i++]="                    ";
     ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}    	

else if(ind==iK_220)
	{
	char i;
	i=0;
	
	ptrs[i++]=" Сеть               ";
	if(NUMBAT)
     ptrs[i++]=" Батареи            ";
	if(NUMIST)
	ptrs[i++]=" БПС                ";
	if(NUMINV)
     ptrs[i++]=" Инверторы          ";
	ptrs[i++]=" Нагрузка           ";
     if(NUMDT)
     ptrs[i++]=" Внешние датчики    ";
     ptrs[i++]=" Выход              ";
     ptrs[i++]="                    ";
     ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}    	

else if(ind==iK_220_380)
	{
	char i;
	i=0;
	
	ptrs[i++]=" Сеть               ";
	if(NUMBAT)
    ptrs[i++]=" Батареи            ";
	if(NUMIST)
	ptrs[i++]=" БПС                ";
	if(NUMINV)
    ptrs[i++]=" Инверторы          ";
	ptrs[i++]=" Нагрузка           ";
    if(NUMDT)
    ptrs[i++]=" Внешние датчики    ";
	ptrs[i++]=" Логика реле       !";
    ptrs[i++]=" Выход              ";
    ptrs[i++]="                    ";
    ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);
	
	if(RELE_VENT_LOGIC==0)sub_bgnd("Обычн.",'!',-5);
	else if(RELE_VENT_LOGIC==1)sub_bgnd("Вент.",'!',-4);
	else sub_bgnd("Ав.Б2",'!',-4);	 
	}    	

else if(ind==iK_net)
	{
	ptrs[0]=" U =     @В         ";
     ptrs[1]=" Выход              ";
	ptrs[2]="                    ";
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("   КАЛИБРОВКА СЕТИ  ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	int2lcd(net_U,'@',0);
	//int2lcdyx(net_buff_,3,10,0);
	
	//int2lcdyx(Kunet,3,16,0);
     }


else if(ind==iK_net3)
	{

	ptrs[0]=  		" UфA           !В   ";
    ptrs[1]=  		" UфB           @В   ";
    ptrs[2]=  	    " UфC           #В   ";
	ptrs[3]=  	    " Выход              ";


	bgnd_par(		"   КАЛИБРОВКА СЕТИ  ",
					ptrs[index_set],
					ptrs[index_set+1],
					ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

    int2lcd(net_Ua,'!',0);
	int2lcd(net_Ub,'@',0);
	int2lcd(net_Uc,'#',0);

	/*int2lcdyx(KunetC,0,19,0);
	int2lcdyx(adc_buff_[10],0,13,0);
	int2lcdyx(KunetB,0,8,0);
	int2lcdyx(adc_buff_[3],0,4,0);*/

    }


else if(ind==iK_load)
	{
	ptrs[0]=" U =     @В         ";
     ptrs[1]=" Выход              ";
	ptrs[2]="                    ";
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(		" КАЛИБРОВКА НАГРУЗКИ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

	pointer_set(1);
	if((load_U)>1000)int2lcd(load_U/10,'@',0);	
	else int2lcd(load_U,'@',1);
     }


else if(ind==iK_t_ext)
	{
	ptrs[0]=  	" tвнеш.возд.    !°С ";
     ptrs[1]=  	" tотсек ЭПУ     @°С ";
     ptrs[2]=  	" tотсек MSAN    #°С ";
     ptrs[3]=	     " Выход              ";
	ptrs[4]=	     "                    ";
	ptrs[5]=	     "                    ";
	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(		" КАЛИБРОВКА ВНЕШНИХ ",
				" ДАТЧИКОВ ТЕМПЕРАТУР",
				ptrs[index_set],
				ptrs[index_set+1]);

	pointer_set(2);	
	if(ND_EXT[0])sub_bgnd("неиспр.",'!',-3);
     else int2lcd_mmm(t_ext[0],'!',0);

	if(ND_EXT[1])sub_bgnd("неиспр.",'@',-3);
     else int2lcd_mmm(t_ext[1],'@',0);

	if(ND_EXT[2])sub_bgnd("неиспр.",'#',-3);
     else int2lcd_mmm(t_ext[2],'#',0);
     }

else if(ind==iK_t_ext_6U)
	{
	ptrs[0]=  		" t1             !°С ";
    ptrs[1]=  		" t2             @°С ";
    ptrs[2]=  		" t3             #°С ";
    ptrs[NUMDT]=	" Выход              ";
	ptrs[NUMDT+1]=  "                    ";
	ptrs[NUMDT+2]=  "                    ";
	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(		" КАЛИБРОВКА ВНЕШНИХ ",
				" ДАТЧИКОВ ТЕМПЕРАТУР",
				ptrs[index_set],
				ptrs[index_set+1]);

	pointer_set(2);	
	if(ND_EXT[0])sub_bgnd("неиспр.",'!',-3);
     else int2lcd_mmm(t_ext[0],'!',0);

	if(ND_EXT[1])sub_bgnd("неиспр.",'@',-3);
     else int2lcd_mmm(t_ext[1],'@',0);

	if(ND_EXT[2])sub_bgnd("неиспр.",'#',-3);
     else int2lcd_mmm(t_ext[2],'#',0);
	//int2lcdyx(u_necc,3,18,0);
     }
     
else if(ind==iK_bat_sel)
	{
	ptrs[0]=						" Батарея N1         ";
     ptrs[1]=						" Батарея N2         ";
     if(BAT_IS_ON[0]!=bisON)ptrs[0]=	" Батарея N2         ";
	ptrs[0+NUMBAT]=				" Выход              ";
	ptrs[1+NUMBAT]=				"                    ";
	ptrs[2+NUMBAT]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(" КАЛИБРОВКА БАТАРЕЙ ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
     }     

else if(ind==iK_bat)
	{
	ptrs[0]=		" Uбат =     @В      ";
	ptrs[1]=		" откалибруйте Uбат  ";
	ptrs[2]=		"  нажатием љ или њ  ";
     ptrs[3]=		" Iбат =     #А      ";
     if(phase==0)
          {
          ptrs[4]=	"   нажмите ¤ для    ";
          ptrs[5]=	"калибровки нуля Iбат";
          }
     else          
          {
          ptrs[4]=	" откалибруйте Iбат  ";
          ptrs[5]=	"  нажатием љ или њ  ";
          }
     if(bat[sub_ind1]._nd)
     	{
     	ptrs[6]=		" Датчик температуры ";
     	ptrs[7]=		"     неисправен     ";
     	ptrs[8]=		"  или неподключен.  ";
     	}
     else
     	{	     
     	ptrs[6]=		" tбат =    $°C      ";
     	ptrs[7]=		" откалибруйте tбат  ";
     	ptrs[8]=		"  нажатием љ или њ  ";
     	}

	ptrs[9]=		" Uбат.с.т. =     ^В ";
	ptrs[10]=		"калибруйте Uбат.с.т.";
	ptrs[11]=		"  нажатием љ или њ  ";

     ptrs[12]=		" Выход              ";
     ptrs[13]=		"                    ";
     ptrs[14]=		"                    ";

	bgnd_par(		" КАЛИБРОВКА БАТ. N! ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);
     
     if(sub_ind==0)
     	{
     	mess_send(MESS2BPS_HNDL,PARAM_BPS_ALL_OFF_AFTER_2SEC,0xffff,10);
     	mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<(1-sub_ind1)),10);
     	//mess_send(MESS2BAT_HNDL1,PARAM_BAT_ON,(1<<sub_ind1),10);
     	}
     
     if(sub_ind==3)
     	{
     	if(phase==0)
     		{
			mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,0xffff,10);
     		mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,10);
			mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<sub_ind1),10);
     		//mess_send(MESS2BAT_HNDL1,PARAM_BAT_ON,(1<<(1-sub_ind1)),10);
     		}
     	else if(phase==1)
     		{
			mess_send(MESS2BPS_HNDL,PARAM_BPS_ALL_OFF_AFTER_2SEC,0xffff,10);
			mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<(1-sub_ind1)),10);
     		//mess_send(MESS2BAT_HNDL1,PARAM_BAT_ON,(1<<sub_ind1),10);
   			}
     		
     	}

     if(sub_ind==6)
     	{
   		//mess_send(_MESS_BAT_MASK_ON,_MESS_BAT_MASK_ON,(0xffff),10);
    		//mess_send(MESS_SRC_ON_OFF,_MESS_SRC_MASK_UNBLOK,0xffff,10);
     		
     	}

     if(sub_ind==9)
     	{
     	mess_send(MESS2BPS_HNDL,PARAM_BPS_ALL_OFF_AFTER_2SEC,0xffff,10);
     	mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<(1-sub_ind1)),10);
     	//mess_send(MESS2BAT_HNDL1,PARAM_BAT_ON,(1<<sub_ind1),10);
     	}
	
	if((sub_ind==0)||(sub_ind==1)||(sub_ind==2))index_set=0;
	else if((sub_ind==3)||(sub_ind==4)||(sub_ind==5))index_set=3;
	else if((sub_ind==6)||(sub_ind==7)||(sub_ind==8))index_set=6;
     else if((sub_ind==9)||(sub_ind==10)||(sub_ind==11))index_set=9;
	else index_set=12;
	


	pointer_set(1);	
	int2lcd(sub_ind1+1,'!',0);
	if((bat[sub_ind1]._Ub)>1000)int2lcd(bat[sub_ind1]._Ub/10,'@',0);
	else int2lcd(bat[sub_ind1]._Ub,'@',1);
	int2lcd_mmm(bat[sub_ind1]._Ib,'#',2);
	int2lcd_mmm(bat[sub_ind1]._Tb,'$',0);
     int2lcd(bat[sub_ind1]._Ubm,'^',1);
	//int2lcd(Ibat,'a',1);
	
	   /*  int2lcdyx(ad7705_res1,0,10,0);
         int2lcdyx(ad7705_res2,0,16,0); */
		    //int2lcdyx(ad7705_buff_[sub_ind1],0,10,0);
         //int2lcdyx(Kibat1[sub_ind1],0,16,0); 
	//int2lcdyx(Kubat[0],0,4,0);
	//int2lcdyx(adc_buff_[4],0,8,0);

	//int2lcdyx(rele_stat,2,15,0);
	
	
	/*	int2lcdyx(bat_cnt_to_block[0],3,3,0);
	int2lcdyx(bat_rel_stat[0],3,7,0); */

	//int2lcdyx(Ktbat[sub_ind1],0,16,0);
	//char2lcdhyx(rele_stat,0,19);
	}  	


	

else if(ind==iK_bat_simple)
	{
	ptrs[0]=		" Uбат =     @В      ";
	ptrs[1]=		" откалибруйте Uбат  ";
	ptrs[2]=		"  нажатием љ или њ  ";
     ptrs[3]=		" Iбат =     #А      ";
     if(phase==0)
          {
          ptrs[4]=	"   нажмите ¤ для    ";
          ptrs[5]=	"калибровки нуля Iбат";
          }
     else          
          {
          ptrs[4]=	" откалибруйте Iбат  ";
          ptrs[5]=	"  нажатием љ или њ  ";
          }
     if(bat[sub_ind1]._nd)
     	{
     	ptrs[6]=		" Датчик температуры ";
     	ptrs[7]=		"     неисправен     ";
     	ptrs[8]=		"  или неподключен.  ";
     	}
     else
     	{	     
     	ptrs[6]=		" tбат =    $°C      ";
     	ptrs[7]=		" откалибруйте tбат  ";
     	ptrs[8]=		"  нажатием љ или њ  ";
     	}

     ptrs[9]=		" Выход              ";
     ptrs[10]=		"                    ";
     ptrs[11]=		"                    ";

	bgnd_par(		" КАЛИБРОВКА БАТ. N! ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);
     
     if(sub_ind==0)
     	{
     	mess_send(MESS2BPS_HNDL,PARAM_BPS_ALL_OFF_AFTER_2SEC,0xffff,10);
     	mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<(1-sub_ind1)),10);
     	//mess_send(MESS2BAT_HNDL1,PARAM_BAT_ON,(1<<sub_ind1),10);
     	}
     
     if(sub_ind==3)
     	{
     	if(phase==0)
     		{
			mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,0xffff,10);
     		mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,10);
			mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<sub_ind1),10);
     		//mess_send(MESS2BAT_HNDL1,PARAM_BAT_ON,(1<<(1-sub_ind1)),10);
     		}
     	else if(phase==1)
     		{
			mess_send(MESS2BPS_HNDL,PARAM_BPS_ALL_OFF_AFTER_2SEC,0xffff,10);
			mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<(1-sub_ind1)),10);
     		//mess_send(MESS2BAT_HNDL1,PARAM_BAT_ON,(1<<sub_ind1),10);
   			}
     		
     	}

     if(sub_ind==6)
     	{
   		//mess_send(_MESS_BAT_MASK_ON,_MESS_BAT_MASK_ON,(0xffff),10);
    		//mess_send(MESS_SRC_ON_OFF,_MESS_SRC_MASK_UNBLOK,0xffff,10);
     		
     	}

     if(sub_ind==9)
     	{
     	mess_send(MESS2BPS_HNDL,PARAM_BPS_ALL_OFF_AFTER_2SEC,0xffff,10);
     	mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<(1-sub_ind1)),10);
     	//mess_send(MESS2BAT_HNDL1,PARAM_BAT_ON,(1<<sub_ind1),10);
     	}
	
	if((sub_ind==0)||(sub_ind==1)||(sub_ind==2))index_set=0;
	else if((sub_ind==3)||(sub_ind==4)||(sub_ind==5))index_set=3;
	else if((sub_ind==6)||(sub_ind==7)||(sub_ind==8))index_set=6;
	else index_set=9;
	


	pointer_set(1);	
	int2lcd(sub_ind1+1,'!',0);
	if((bat[sub_ind1]._Ub)>1000)int2lcd(bat[sub_ind1]._Ub/10,'@',0);
	else int2lcd(bat[sub_ind1]._Ub,'@',1);
	int2lcd_mmm(bat[sub_ind1]._Ib,'#',2);
	int2lcd_mmm(bat[sub_ind1]._Tb,'$',0);

	//int2lcdyx(ad7705_buff_[0],0,6,0);
	//int2lcdyx(adc_buff_[12],0,15,0);
	
	}  	

else if(ind==iK_inv_sel)
	{
	ptrs[0]=						" ИНВЕРТОР N1        ";
     ptrs[1]=						" ИНВЕРТОР N2        ";
     ptrs[2]=						" ИНВЕРТОР N3        ";
	ptrs[3]=						" ИНВЕРТОР N4        ";
     ptrs[4]=						" ИНВЕРТОР N5        ";
     ptrs[5]=						" ИНВЕРТОР N6        ";
	ptrs[6]=						" ИНВЕРТОР N7        ";
     ptrs[7]=						" ИНВЕРТОР N8        ";
     ptrs[8]=						" ИНВЕРТОР N9        ";
	ptrs[9]=						" ИНВЕРТОР N10       ";
     ptrs[10]=						" ИНВЕРТОР N11       ";
     ptrs[11]=						" ИНВЕРТОР N12       "; 
	ptrs[12]=						" ИНВЕРТОР N13       ";
     ptrs[13]=						" ИНВЕРТОР N14       ";
     ptrs[14]=						" ИНВЕРТОР N15       ";	
	 ptrs[15]=						" ИНВЕРТОР N16       ";
	ptrs[16]=						" ИНВЕРТОР N17       ";
     ptrs[17]=						" ИНВЕРТОР N18       ";
     ptrs[18]=						" ИНВЕРТОР N19       ";
	ptrs[19]=						" ИНВЕРТОР N20       ";
     ptrs[20]=						" ИНВЕРТОР N21       ";
     ptrs[21]=						" ИНВЕРТОР N22       "; 
	ptrs[22]=						" ИНВЕРТОР N23       ";
     ptrs[23]=						" ИНВЕРТОР N24       ";
     ptrs[24]=						" ИНВЕРТОР N25       ";	
	 ptrs[25]=						" ИНВЕРТОР N26       ";
	ptrs[26]=						" ИНВЕРТОР N27       ";
     ptrs[27]=						" ИНВЕРТОР N28       ";
     ptrs[28]=						" ИНВЕРТОР N29       ";
	ptrs[29]=						" ИНВЕРТОР N30       ";
     ptrs[30]=						" ИНВЕРТОР N31       ";
     ptrs[31]=						" ИНВЕРТОР N32       "; 
	ptrs[32]=						" ИНВЕРТОР N33       ";
     ptrs[33]=						" ИНВЕРТОР N34       ";
     ptrs[34]=						" ИНВЕРТОР N35       ";	
	 ptrs[35]=						" ИНВЕРТОР N36       ";
	ptrs[36]=						" ИНВЕРТОР N37       ";
     ptrs[37]=						" ИНВЕРТОР N38       ";
     ptrs[38]=						" ИНВЕРТОР N39       ";
	ptrs[39]=						" ИНВЕРТОР N40       ";
     ptrs[40]=						" ИНВЕРТОР N41       ";
     ptrs[41]=						" ИНВЕРТОР N42       "; 
	ptrs[42]=						" ИНВЕРТОР N43       ";
     ptrs[43]=						" ИНВЕРТОР N44       ";
     ptrs[44]=						" ИНВЕРТОР N45       ";	              
	ptrs[NUMINV]=					" Выход              ";
	ptrs[1+NUMINV]=				"                    ";
	ptrs[2+NUMINV]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("КАЛИБРОВАТЬ ИНВЕРТОР",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
     }     
else if(ind==iK_makb_sel)
	{
	ptrs[0]=						" Монитор АКБ N1     ";
     ptrs[1]=						" Монитор АКБ N2     ";
     ptrs[2]=						" Монитор АКБ N3     ";
	ptrs[3]=						" Монитор АКБ N4     ";
	ptrs[NUMMAKB]=					" Выход              ";
	ptrs[1+NUMMAKB]=				"                    ";
	ptrs[2+NUMMAKB]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("КАЛИБРОВАТЬ МОНИТОРЫ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
     }     
/*
else if(ind==iK_makb)
	{
	ptrs[0]=						" U1  =    !В        ";
	ptrs[1]=						" U2  =    @В        ";
	ptrs[2]=						" U3  =    #В        ";
	ptrs[3]=						" U4  =    $В        ";
	ptrs[4]=						" U5  =    %В        ";
	ptrs[5]=						" t1  =    ^°C       ";
	ptrs[6]=						" t2  =    &°C       ";
	ptrs[7]=						" t3  =    *°C       ";
	ptrs[8]=						" t4  =    (°C       ";
	ptrs[9]=						" t5  =    )°C       ";
	ptrs[10]=						" Выход              ";
	ptrs[11]=						"                    ";

	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ","   МОНИТОР АКБ N<   ",ptrs[index_set],ptrs[index_set+1]);
	pointer_set(2);
	simax=10;

	int2lcd(sub_ind1+1,'<',0);
	int2lcd(makb[sub_ind1]._U[0],'!',1);
	int2lcd(makb[sub_ind1]._U[1],'@',1);
	int2lcd(makb[sub_ind1]._U[2],'#',1);
	int2lcd(makb[sub_ind1]._U[3],'$',1);
	int2lcd(makb[sub_ind1]._U[4],'%',1);

	if(makb[sub_ind1]._T_nd[0])sub_bgnd("НЕПОДКЛЮЧЕН",'^',-5);
	else int2lcd_mmm(makb[sub_ind1]._T[0],'^',0); 
	if(makb[sub_ind1]._T_nd[1])sub_bgnd("НЕПОДКЛЮЧЕН",'&',-5);
	else int2lcd_mmm(makb[sub_ind1]._T[1],'&',0); 
	if(makb[sub_ind1]._T_nd[2])sub_bgnd("НЕПОДКЛЮЧЕН",'*',-5);
	else int2lcd_mmm(makb[sub_ind1]._T[2],'*',0); 
	if(makb[sub_ind1]._T_nd[3])sub_bgnd("НЕПОДКЛЮЧЕН",'(',-5);
	else int2lcd_mmm(makb[sub_ind1]._T[3],'(',0); 
	if(makb[sub_ind1]._T_nd[4])sub_bgnd("НЕПОДКЛЮЧЕН",')',-5);
	else int2lcd_mmm(makb[sub_ind1]._T[4],')',0); 



	
     }   
*/
else if(ind==iK_inv)
	{
	
	ptrs[0]=	" Uвых =    @В       ";
	ptrs[1]=	"откалибруйте Uвыхинв";
	ptrs[2]=	"  нажатием љ или њ  "; 
	ptrs[3]=	" Iвых =     %А      ";
	if(phase==0)
          {
          ptrs[4]=	"   нажмите ¤ для    ";
          ptrs[5]=	"калибровки нуля Iвых";
          }
     else
     	{
          ptrs[4]=" откалибруйте Iвых  ";
          ptrs[5]="  нажатием љ или њ  ";     	
     	} 
     	
	ptrs[6]=	" tинв =   ^°C       ";    
	ptrs[7]=	" откалибруйте tинв  ";
	ptrs[8]=	"  нажатием љ или њ  ";
	ptrs[9]=	" Uшины =    &В      ";
	ptrs[10]=	"откалибруйте Uшины  ";
	ptrs[11]=	"  нажатием љ или њ  "; 
	ptrs[12]=	" Uсети =    *В      ";
	ptrs[13]=	"откалибруйте Uсети  ";
	ptrs[14]=	"  нажатием љ или њ  "; 
	ptrs[15]=	" Pвых  =    (Вт     ";
	ptrs[16]=	"откалибруйте Pвых   ";
	ptrs[17]=	"  нажатием љ или њ  ";
	ptrs[18]=	" Uвход =    [В      ";
	ptrs[19]=	"откалибруйте Uвход  ";
	ptrs[20]=	"  нажатием љ или њ  ";	 

	ptrs[21]=	sm_exit;
	ptrs[22]=	sm_;
	ptrs[23]=	sm_;     	     	    
	

    	 if((sub_ind==0)||(sub_ind==1)||(sub_ind==2))index_set=0;
	else if((sub_ind==3)||(sub_ind==4)||(sub_ind==5))index_set=3;
	else if((sub_ind==6)||(sub_ind==7)||(sub_ind==8))index_set=6;
	else if((sub_ind==9)||(sub_ind==10)||(sub_ind==11))index_set=9;
	else if((sub_ind==12)||(sub_ind==13)||(sub_ind==14))index_set=12;
	else if((sub_ind==15)||(sub_ind==16)||(sub_ind==17))index_set=15;
	else if((sub_ind==18)||(sub_ind==19)||(sub_ind==20))index_set=18;
	else index_set=21;
	
	bgnd_par("КАЛИБРОВКА ИНВЕРТ N!",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	int2lcd(sub_ind1+1,'!',0);
	int2lcd(inv[sub_ind1]._Uout,'@',1);
	int2lcd(inv[sub_ind1]._Iout,'%',1);
	int2lcd(inv[sub_ind1]._T,'^',0); 
	int2lcd(inv[sub_ind1]._Uload,'&',1);
	int2lcd(inv[sub_ind1]._Uacin,'*',1);
	int2lcd_mmm(inv[sub_ind1]._Pout,'(',0);
	int2lcd(inv[sub_ind1]._Udcin,'[',1); 

     if((sub_ind==0))
		{
//		mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
//		mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
	    mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
     if(sub_ind==3)
		{
		if(phase==0)
			{
//          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,~(1<<sub_ind1),10);
          	}
      	else if(phase==1)
			{
//          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
//			mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
          	}
          mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
	

	
  
	     
	//MSG_IND2PWM_SRC1=900;
	//MSG_IND2PWM_SRC2=900;         
/*int2lcdyx(sub_ind1,0,0,0);
int2lcdyx(sub_ind,0,1,0);
int2lcdyx(phase,0,2,0);
int2lcdyx(MSG_IND2OUT_DIS_SRC1,0,3,0);
int2lcdyx(MSG_IND2OUT_DIS_SRC2,0,4,0);  
int2lcdyx(MSG_IND2OUT_EN_SRC1,0,5,0);
int2lcdyx(MSG_IND2OUT_EN_SRC2,0,6,0); */

//int2lcdyx(cntrl_stat1,0,19,0); 
//int2lcdyx(load_U,0,5,0); 
//int2lcdyx(cntrl_stat,0,10,0); 
//int2lcdyx(bps[sub_ind1]._rotor,0,19,0); 
//int2lcdyx(u_necc,0,19,0);  
	 }

else if(ind==iK_byps)
	{
	
	ptrs[0]=	" Uвых =    @В       ";
	ptrs[1]=	"откалибруйте Uвых   ";
	ptrs[2]=	"  нажатием љ или њ  "; 
	ptrs[3]=	" Iвых =     %А      ";
	if(phase==0)
          {
          ptrs[4]=	"   нажмите ¤ для    ";
          ptrs[5]=	"калибровки нуля Iвых";
          }
     else
     	{
          ptrs[4]=" откалибруйте Iвых  ";
          ptrs[5]="  нажатием љ или њ  ";     	
     	} 
     	
	ptrs[6]=	" t    =   ^°C       ";    
	ptrs[7]=	" откалибруйте t     ";
	ptrs[8]=	"  нажатием љ или њ  ";
	ptrs[9]=	" Uшины =    &В      ";
	ptrs[10]=	"откалибруйте Uшины  ";
	ptrs[11]=	"  нажатием љ или њ  "; 
	ptrs[12]=	" Uсети =    *В      ";
	ptrs[13]=	"откалибруйте Uсети  ";
	ptrs[14]=	"  нажатием љ или њ  "; 
	ptrs[15]=	" Pвых  =    (Вт     ";
	ptrs[16]=	"откалибруйте Pвых   ";
	ptrs[17]=	"  нажатием љ или њ  "; 

	ptrs[18]=	sm_exit;
	ptrs[19]=	sm_;
	ptrs[20]=	sm_;     	     	    
	

    	 if((sub_ind==0)||(sub_ind==1)||(sub_ind==2))index_set=0;
	else if((sub_ind==3)||(sub_ind==4)||(sub_ind==5))index_set=3;
	else if((sub_ind==6)||(sub_ind==7)||(sub_ind==8))index_set=6;
	else if((sub_ind==9)||(sub_ind==10)||(sub_ind==11))index_set=9;
	else if((sub_ind==12)||(sub_ind==13)||(sub_ind==14))index_set=12;
	else if((sub_ind==15)||(sub_ind==16)||(sub_ind==17))index_set=15;

	else index_set=18;
	
	if(NUMBYPASS>1)	bgnd_par("КАЛИБРОВКА БАЙПАССN!",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);	
	else bgnd_par("КАЛИБРОВКА БАЙПАСС  ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	int2lcd(byps[sub_ind1]._Uout,'@',1);
	int2lcd(byps[sub_ind1]._Iout,'%',1);
	int2lcd(byps[sub_ind1]._T,'^',0); 
	int2lcd(byps[sub_ind1]._UinACinvbus,'&',1);
	int2lcd(byps[sub_ind1]._UinACprim,'*',1);
	int2lcd(sub_ind1+1,'!',0);
	//int2lcd_mmm(byps._Pout,'(',0); 
	if(byps[sub_ind1]._Pout>65000)byps[sub_ind1]._Pout=0; 
	long2lcd_mmm((unsigned short)byps[sub_ind1]._Pout,'(',0);

     if((sub_ind==0))
		{
//		mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
//		mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
	    mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
     if(sub_ind==3)
		{
		if(phase==0)
			{
//          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,~(1<<sub_ind1),10);
          	}
      	else if(phase==1)
			{
//          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
//			mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
          	}
          mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
	

	
  
	     
	//MSG_IND2PWM_SRC1=900;
	//MSG_IND2PWM_SRC2=900;         
/*int2lcdyx(sub_ind1,0,0,0);
int2lcdyx(sub_ind,0,1,0);
int2lcdyx(phase,0,2,0);
int2lcdyx(MSG_IND2OUT_DIS_SRC1,0,3,0);
int2lcdyx(MSG_IND2OUT_DIS_SRC2,0,4,0);  
int2lcdyx(MSG_IND2OUT_EN_SRC1,0,5,0);
int2lcdyx(MSG_IND2OUT_EN_SRC2,0,6,0); */

//int2lcdyx(cntrl_stat1,0,19,0); 
//int2lcdyx(load_U,0,5,0); 
//int2lcdyx(cntrl_stat,0,10,0); 
//int2lcdyx(bps[sub_ind1]._rotor,0,19,0); 
//int2lcdyx(u_necc,0,19,0);  
	 }

else if(ind==iK_byps_3f)
	{
	
	ptrs[0]=	" UвыхA =    !В      ";
	ptrs[1]=	"откалибруйте Uвых   ";
	ptrs[2]=	"  нажатием љ или њ  ";
	ptrs[3]=	" UвыхB =    @В      ";
	ptrs[4]=	"откалибруйте Uвых   ";
	ptrs[5]=	"  нажатием љ или њ  ";
	ptrs[6]=	" UвыхC =    #В      ";
	ptrs[7]=	"откалибруйте Uвых   ";
	ptrs[8]=	"  нажатием љ или њ  ";	
		 
	ptrs[9]=	" IвыхA =     $А     ";
	if(phase==0)
		{
		ptrs[10]=	"   нажмите ¤ для    ";
		ptrs[11]=	"калибровки нуля Iвых";
		}
     else
		{
		ptrs[10]=" откалибруйте Iвых  ";
		ptrs[11]="  нажатием љ или њ  ";     	
     	} 
	ptrs[12]=	" IвыхB =     %А     ";
	if(phase==0)
		{
		ptrs[13]=	"   нажмите ¤ для    ";
		ptrs[14]=	"калибровки нуля Iвых";
		}
     else
		{
		ptrs[13]=" откалибруйте Iвых  ";
		ptrs[14]="  нажатием љ или њ  ";     	
     	}
	ptrs[15]=	" IвыхC =     ^А     ";
	if(phase==0)
		{
		ptrs[16]=	"   нажмите ¤ для    ";
		ptrs[17]=	"калибровки нуля Iвых";
		}
     else
		{
		ptrs[16]=" откалибруйте Iвых  ";
		ptrs[17]="  нажатием љ или њ  ";     	
     	}
				     	
	ptrs[18]=	" tA    =   &°C      ";    
	ptrs[19]=	" откалибруйте t     ";
	ptrs[20]=	"  нажатием љ или њ  ";
	ptrs[21]=	" tB    =   *°C      ";    
	ptrs[22]=	" откалибруйте t     ";
	ptrs[23]=	"  нажатием љ или њ  ";
	ptrs[24]=	" tC    =   (°C      ";    
	ptrs[25]=	" откалибруйте t     ";
	ptrs[26]=	"  нажатием љ или њ  ";

	ptrs[27]=	" UшиныA =    )В     ";
	ptrs[28]=	"откалибруйте Uшины  ";
	ptrs[29]=	"  нажатием љ или њ  ";
	ptrs[30]=	" UшиныB =    -В     ";
	ptrs[31]=	"откалибруйте Uшины  ";
	ptrs[32]=	"  нажатием љ или њ  ";
	ptrs[33]=	" UшиныC =    +В     ";
	ptrs[34]=	"откалибруйте Uшины  ";
	ptrs[35]=	"  нажатием љ или њ  ";
			 
	ptrs[36]=	" UсетиA =    {В     ";
	ptrs[37]=	"откалибруйте Uсети  ";
	ptrs[38]=	"  нажатием љ или њ  "; 
	ptrs[39]=	" UсетиB =    }В     ";
	ptrs[40]=	"откалибруйте Uсети  ";
	ptrs[41]=	"  нажатием љ или њ  "; 
	ptrs[42]=	" UсетиC =    [В     ";
	ptrs[43]=	"откалибруйте Uсети  ";
	ptrs[44]=	"  нажатием љ или њ  ";
	 
	ptrs[45]=	" PвыхA  =    ]Вт    ";
	ptrs[46]=	"откалибруйте Pвых   ";
	ptrs[47]=	"  нажатием љ или њ  "; 
	ptrs[48]=	" PвыхB  =    <Вт    ";
	ptrs[49]=	"откалибруйте Pвых   ";
	ptrs[50]=	"  нажатием љ или њ  ";
	ptrs[51]=	" PвыхC  =    >Вт    ";
	ptrs[52]=	"откалибруйте Pвых   ";
	ptrs[53]=	"  нажатием љ или њ  ";

	ptrs[54]=	sm_exit;
	ptrs[55]=	sm_;
	ptrs[56]=	sm_;     	     	    
	

   	if((sub_ind==0)||(sub_ind==1)||(sub_ind==2))index_set=0;
	else if((sub_ind==3)||(sub_ind==4)||(sub_ind==5))index_set=3;
	else if((sub_ind==6)||(sub_ind==7)||(sub_ind==8))index_set=6;
	else if((sub_ind==9)||(sub_ind==10)||(sub_ind==11))index_set=9;
	else if((sub_ind==12)||(sub_ind==13)||(sub_ind==14))index_set=12;
	else if((sub_ind==15)||(sub_ind==16)||(sub_ind==17))index_set=15;
	else if((sub_ind==18)||(sub_ind==19)||(sub_ind==20))index_set=18;
	else if((sub_ind==21)||(sub_ind==22)||(sub_ind==23))index_set=21;
	else if((sub_ind==24)||(sub_ind==25)||(sub_ind==26))index_set=24;
	else if((sub_ind==27)||(sub_ind==28)||(sub_ind==29))index_set=27;
	else if((sub_ind==30)||(sub_ind==31)||(sub_ind==32))index_set=30;
	else if((sub_ind==33)||(sub_ind==34)||(sub_ind==35))index_set=33;
	else if((sub_ind==36)||(sub_ind==37)||(sub_ind==38))index_set=36;
	else if((sub_ind==39)||(sub_ind==40)||(sub_ind==41))index_set=39;
	else if((sub_ind==42)||(sub_ind==43)||(sub_ind==44))index_set=42;
	else if((sub_ind==45)||(sub_ind==46)||(sub_ind==47))index_set=45;
	else if((sub_ind==48)||(sub_ind==49)||(sub_ind==50))index_set=48;
	else if((sub_ind==51)||(sub_ind==52)||(sub_ind==53))index_set=51;


	else index_set=54;
	
 	bgnd_par(" КАЛИБРОВКА БАЙПАСА ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	int2lcd(byps[0]._Uout,'!',1);
	int2lcd(byps[1]._Uout,'@',1);
	int2lcd(byps[2]._Uout,'#',1);
	int2lcd(byps[0]._Iout,'$',1);
	int2lcd(byps[1]._Iout,'%',1);
	int2lcd(byps[2]._Iout,'^',1);
	int2lcd(byps[0]._T,'&',0);
	int2lcd(byps[1]._T,'*',0);
	int2lcd(byps[2]._T,'(',0);		 
	int2lcd(byps[0]._UinACinvbus,')',1);
	int2lcd(byps[1]._UinACinvbus,'-',1);
	int2lcd(byps[2]._UinACinvbus,'+',1);
	int2lcd(byps[0]._UinACprim,'{',1);
	int2lcd(byps[1]._UinACprim,'}',1);
	int2lcd(byps[2]._UinACprim,'[',1);
	//int2lcd(sub_ind1+1,'!',0);
	//int2lcd_mmm(byps._Pout,'(',0); 
	if(byps[0]._Pout>65000)byps[0]._Pout=0;
	if(byps[1]._Pout>65000)byps[1]._Pout=0;
	if(byps[2]._Pout>65000)byps[2]._Pout=0; 
	long2lcd_mmm((unsigned short)byps[0]._Pout,']',0);
	long2lcd_mmm((unsigned short)byps[1]._Pout,'<',0);
	long2lcd_mmm((unsigned short)byps[2]._Pout,'>',0);

	}
else if(ind==iK_byps_sel)
	{
	ptrs[0]=						" Байпасс N1         ";
	ptrs[1]=						" Байпасс N2         ";
	ptrs[2]=						" Байпасс N3         ";
              
	ptrs[NUMBYPASS]=				" Выход              ";
	ptrs[1+NUMBYPASS]=				"                    ";
	ptrs[2+NUMBYPASS]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("КАЛИБРОВКА БАЙПАССОВ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
	}     

else if(ind==iK_bps_sel)
	{
	ptrs[0]=						" БПС N1             ";
     ptrs[1]=						" БПС N2             ";
     ptrs[2]=						" БПС N3             ";
	ptrs[3]=						" БПС N4             ";
     ptrs[4]=						" БПС N5             ";
     ptrs[5]=						" БПС N6             ";
	ptrs[6]=						" БПС N7             ";
     ptrs[7]=						" БПС N8             ";
     ptrs[8]=						" БПС N9             ";
	ptrs[9]=						" БПС N10            ";
     ptrs[10]=						" БПС N11            ";
     ptrs[11]=						" БПС N12            ";               
	ptrs[NUMIST]=					" Выход              ";
	ptrs[1+NUMIST]=				"                    ";
	ptrs[2+NUMIST]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("  КАЛИБРОВКА БПСов  ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
     }     

else if(ind==iK_bps)
	{
/*	
	ptrs[0]=" Uист =    @В       ";
     ptrs[1]=" откалибруйте Uист  ";
     ptrs[2]="  нажатием љ или њ  "; 
	ptrs[3]=" Uнагр =   #В       ";
     ptrs[4]=" откалибруйте Uнагр ";
     ptrs[5]="  нажатием љ или њ  ";
	ptrs[6]=" Uавтон =   $В      ";
	if(bFL_)
		{
		ptrs[7]=" установите Uавтон  ";
     	ptrs[8]="  нажатием љ или њ  ";
     	}
     else 
     	{
		ptrs[7]=" удерживайте ¤ для  ";
     	ptrs[8]="    запоминания     ";     	
     	}	
	ptrs[9]=" Iист =     %А      ";
	if(phase==0)
          {
          ptrs[10]=	"   нажмите ¤ для    ";
          ptrs[11]=	"калибровки нуля Iист";
          }
     else
     	{
          ptrs[10]=" откалибруйте Iист  ";
          ptrs[11]="  нажатием љ или њ  ";     	
     	} 
     	
     ptrs[12]=" tист =   ^°C       ";    
	ptrs[13]=" откалибруйте tист  ";
     ptrs[14]="  нажатием љ или њ  ";
     ptrs[15]=sm_exit;
     ptrs[16]=sm_;
     ptrs[17]=sm_;     	     	    
	

     if((sub_ind==0)||(sub_ind==1)||(sub_ind==2))index_set=0;
	else if((sub_ind==3)||(sub_ind==4)||(sub_ind==5))index_set=3;
	else if((sub_ind==6)||(sub_ind==7)||(sub_ind==8))index_set=6;
	else if((sub_ind==9)||(sub_ind==10)||(sub_ind==11))index_set=9;
	else if((sub_ind==12)||(sub_ind==13)||(sub_ind==14))index_set=12;	
	else index_set=15;
	
	bgnd_par(" КАЛИБРОВКА БПС N! ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	int2lcd(sub_ind1+1,'!',0);
	int2lcd(bps[sub_ind1]._Uii,'@',1);
	int2lcd(bps[sub_ind1]._Uin,'#',1);
	int2lcd(U_AVT,'$',1);
	int2lcd(bps[sub_ind1]._Ii,'%',1);
	int2lcd(bps[sub_ind1]._Ti,'^',0); 
	 
	
     if((sub_ind==0)||(sub_ind==3))
		{
		mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
		mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
	    	mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
     if(sub_ind==6)
		{
          mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
          mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,40);
          mess_send(MESS2UNECC_HNDL,PARAM_UNECC_SET,U_AVT,10);
	    	mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_FAST_REG,0,10);

          }

     if(sub_ind==9)
		{
		if(phase==0)
			{
          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,~(1<<sub_ind1),10);
          	}
      	else if(phase==1)
			{
          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
			mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
          	}
          mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
	
    	if(sub_ind==12)
		{
          }	
          
          
	if(mess_find( (MESS2IND_HNDL)) && (mess_data[0]==PARAM_U_AVT_GOOD) )
		{
		show_mess("     Установка      ",
	          	"    напряжения      ",
	          	" автономной работы  ",
	          	"    произведена     ",3000);
		}	     
	     
	//MSG_IND2PWM_SRC1=900;
	//MSG_IND2PWM_SRC2=900;         
/*int2lcdyx(sub_ind1,0,0,0);
int2lcdyx(sub_ind,0,1,0);
int2lcdyx(phase,0,2,0);
int2lcdyx(MSG_IND2OUT_DIS_SRC1,0,3,0);
int2lcdyx(MSG_IND2OUT_DIS_SRC2,0,4,0);  
int2lcdyx(MSG_IND2OUT_EN_SRC1,0,5,0);
int2lcdyx(MSG_IND2OUT_EN_SRC2,0,6,0); */

//int2lcdyx(cntrl_stat1,0,19,0); 
//int2lcdyx(load_U,0,5,0); 
//int2lcdyx(cntrl_stat,0,10,0); 
//int2lcdyx(bps[sub_ind1]._rotor,0,19,0); 
//int2lcdyx(u_necc,0,19,0); */ 
	 }

else if(ind==iK_power_net)
	{
	ptrs[0]=" Uввод=    @В       ";
	ptrs[1]=" Uпэс =    #В       ";
     ptrs[2]=" Выход              ";
	ptrs[3]="                    ";
	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par( "      КАЛИБРОВКА    ",
               "   СИЛОВЫХ ВВОДОВ   ",
               ptrs[index_set],
               ptrs[index_set+1]);

	pointer_set(2);	
	int2lcd(Uvv[0],'@',0);
     int2lcd(Uvv[1],'#',0);
	//int2lcdyx(net_buff_,3,10,0);
	
	//int2lcdyx(Kunet,3,16,0);
     }


else if(ind==iK_power_net3)
	{
     ptrs[0]=  		" Ввод ф.A    !В     ";
	ptrs[1]=  		" Ввод ф.B    @В     ";
	ptrs[2]=  		" Ввод ф.C    #В     ";
     ptrs[3]=  	     " ПЭС  ф.A    &В     ";
     ptrs[4]=  	     " ПЭС  ф.B    *В     ";
     ptrs[5]=  	     " ПЭС  ф.C    (В     ";		            
     ptrs[6]=" Выход              ";
	ptrs[7]="                    ";
	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par( "      КАЛИБРОВКА    ",
               "   СИЛОВЫХ ВВОДОВ   ",
               ptrs[index_set],
               ptrs[index_set+1]);

	pointer_set(2);	
	int2lcd(Uvv_eb2[0],'!',0);
	int2lcd(Uvv_eb2[1],'@',0);
	int2lcd(Uvv_eb2[2],'#',0);
	int2lcd(Upes_eb2[0],'&',0);
	int2lcd(Upes_eb2[1],'*',0);
	int2lcd(Upes_eb2[2],'(',0);
	//int2lcdyx(net_buff_,3,10,0);
	
	//int2lcdyx(Kunet,3,16,0);
     }


#endif	 
			
if(ind==iDeb)
     {
     if(sub_ind==0)
     	{


         	bgnd_par("*0000*000000*       ",
     	         "                    ",
     	         "                    ",
     	         "      ********      ");

	int2lcdyx(SOFT_NUM,0,4,0);
	long2lcdyx_mmm(SOFT_DATE,0,11,0);

/*	int2lcdyx(a_ind.i,0,2,0);
	int2lcdyx(a_ind.s_i,0,6,0);
	int2lcdyx(a_ind.s_i1,0,10,0);
	int2lcdyx(a_ind.s_i2,0,14,0);
	int2lcdyx(a_ind.i_s,0,16,0);




	int2lcdyx(c_ind.i,1,2,0);
	int2lcdyx(c_ind.s_i,1,6,0);
	int2lcdyx(c_ind.s_i1,1,10,0);
	int2lcdyx(c_ind.s_i2,1,14,0);
	int2lcdyx(c_ind.i_s,1,16,0);*/


	int2lcdyx(t_box,2,4,0);
	int2lcdyx(t_ext_can,3,5,0);
	//int2lcdyx(t_ext_can_nd,3,6,0);
	int2lcdyx(t_ext[1],2,10,0);
	int2lcdyx(t_ext[2],3,10,0);

	int2lcdyx(cnt_net_drv/*cntrl_stat*/,0,19,0);

	//int2lcdyx(adc_buff_[14],3,14,0);
	int2lcdyx(net_metr_buff_[0],1,5,0);
	int2lcdyx(net_metr_buff_[1],1,11,0);
	int2lcdyx(net_metr_buff_[2],1,17,0);

	/*int2lcdyx(adc_buff_[12],2,14,0);
	
	int2lcdyx(adc_buff_[5],2,19,0);
	int2lcdyx(adc_buff_[6],3,19,0);*/

	//int2lcdyx(tloaddisable_cmnd,2,14,0);
	//int2lcdyx(tloaddisable_stat,3,14,0);

/*		int2lcdyx(load_U,0,15,0);
		int2lcdyx(u_necc,0,19,0);
		
		

		int2lcdyx(sub_ind1+1,1,0,0);
		int2lcdyx(sub_ind1+2,2,0,0);
		int2lcdyx(sub_ind1+3,3,0,0);
		
		
		int2lcdyx(bps[sub_ind1  ]._cnt,1,2,0);
		int2lcdyx(bps[sub_ind1+1]._cnt,2,2,0);
		int2lcdyx(bps[sub_ind1+2]._cnt,3,2,0);*/		
		
	/*	int2lcdyx(bps[sub_ind1  ]._ist_blok_cnt,1,5,0);
		int2lcdyx(bps[sub_ind1+1]._ist_blok_cnt,2,5,0);
		int2lcdyx(bps[sub_ind1+2]._ist_blok_cnt,3,5,0);*/			
		
	/*	char2lcdhyx(bps[sub_ind1  ]._flags_tu,1,8);
		char2lcdhyx(bps[sub_ind1+1]._flags_tu,2,8);
		char2lcdhyx(bps[sub_ind1+2]._flags_tu,3,8);

		int2lcdyx(bps[sub_ind1  ]._vol_u,1,12,0);
		int2lcdyx(bps[sub_ind1+1]._vol_u,2,12,0);
		int2lcdyx(bps[sub_ind1+2]._vol_u,3,12,0);		


		char2lcdhyx(bps[sub_ind1]._flags_tm,1,15);
		char2lcdhyx(bps[sub_ind1+1]._flags_tm,2,15);
		char2lcdhyx(bps[sub_ind1+2]._flags_tm,3,15);	

		char2lcdhyx(bps[sub_ind1]._Ii,1,19);
		char2lcdhyx(bps[sub_ind1+1]._Ii,2,19);
		char2lcdhyx(bps[sub_ind1+2]._Ii,3,19);*/
	/*
		char2lcdhyx(bps[sub_ind1]._rotor>>8,1,15);
		char2lcdhyx(bps[sub_ind1+1]._rotor>>8,2,15);
		char2lcdhyx(bps[sub_ind1+2]._rotor>>8,3,15);		
		
		char2lcdhyx((char)bps[sub_ind1]._rotor,1,17);
		char2lcdhyx((char)bps[sub_ind1+1]._rotor,2,17);
		char2lcdhyx((char)bps[sub_ind1+2]._rotor,3,17);*/



     	
 /*    	bgnd_par("                    ",
     	         "                    ",
     	         "                    ",
     	         "%                   ");


		int2lcdyx(main_kb_cnt,0,3,0);
		int2lcdyx(cntrl_stat,1,3,0);
   		
		
		int2lcdyx(u_necc_up,0,7,0);
		int2lcdyx(u_necc,1,7,0);
		int2lcdyx(u_necc_dn,2,7,0);
		int2lcdyx(bat[0]._Ub,3,7,0);

		int2lcdyx(sign_U,0,10,0);
		int2lcdyx(sign_I,1,10,0);
		int2lcdyx(superviser_cnt,2,10,0);	


		int2lcdyx(bat[0]._zar,0,19,0);
		int2lcdyx(BAT_C_REAL[0],1,19,0);
		int2lcdyx(BAT_C_NOM[0],2,19,0);
		int2lcdyx(lc640_read_int(EE_BAT1_ZAR_CNT),3,19,0);  */

		  //bat_ver_cnt



	/*	int2lcdyx(tlv_buff[1][1],0,9,0);
		int2lcdyx(tlv_buff[1][2],0,14,0);
		int2lcdyx(tlv_buff[1][3],0,19,0);

   		int2lcdyx(tlv_buff[1][4],1,4,0);
		int2lcdyx(tlv_buff[1][5],1,9,0);
		int2lcdyx(tlv_buff[1][6],1,14,0);
		int2lcdyx(tlv_buff[1][7],1,19,0);

   		int2lcdyx(tlv_buff[1][8],2,4,0);
		int2lcdyx(tlv_buff[1][9],2,9,0);
		int2lcdyx(tlv_buff[1][10],2,14,0);
		int2lcdyx(tlv_buff[1][11],2,19,0);

   		int2lcdyx(tlv_buff[1][12],3,4,0);
		int2lcdyx(tlv_buff[1][13],3,9,0);
		int2lcdyx(tlv_buff[1][14],3,14,0);
		int2lcdyx(tlv_buff[1][15],3,19,0);	*/
      

     	}     

    	else if(sub_ind==1) 
     	{
     	bgnd_par("Ошибки передачи    !",
     	         "Ошибки приема      @",
     	         "                    ",
     	         "                    ");

		int2lcd(mcp2515_tec,'!',0);
		int2lcd(mcp2515_rec,'@',0);

 		}

 

    else if(sub_ind==2)
     	{
     	bgnd_par(	"CAN                 ",
     		    	"                    ",
     		    	"                    ",
     		    	"                    ");



		int2lcdyx(plazma_can1,0,19,0);
		int2lcdyx(plazma_can2,1,19,0);
		int2lcdyx(plazma_can3,2,19,0);
		int2lcdyx(plazma_can4,3,19,0);  


		int2lcdyx(can_rotor[0],0,14,0);
		int2lcdyx(can_rotor[1],1,14,0);
		int2lcdyx(can_rotor[2],2,14,0);
		int2lcdyx(can_rotor[3],3,14,0);

		/*	int2lcdyx(makb[2]._cnt,0,10,0);*/
		int2lcdyx(LPC_SC->RSID,1,3,0);	
 		int2lcdyx(reset_plazma,2,3,0);
		int2lcdyx(snmp_numofevents,3,3,0);
			
		int2lcdyx(ptr_can1_tx_wr,0,5,0);
		int2lcdyx(ptr_can1_tx_rd,1,5,0);
		int2lcdyx(ptr_can1_tx_wr-ptr_can1_tx_rd,2,5,0);
		int2lcdyx(((LPC_CAN1->GSR)>>16)&0x00ff,3,5,0);
		int2lcdyx(((LPC_CAN1->GSR)>>24)&0x00ff,3,10,0);
/*
		int2lcdyx(SK_REL_EN[0]&0x000f,0,16,0);
		int2lcdyx(SK_REL_EN[1]&0x000f,1,16,0);
		int2lcdyx(SK_REL_EN[2]&0x000f,2,16,0);

		int2lcdyx(SK_SIGN[0]&0x000f,0,10,0);
		int2lcdyx(SK_SIGN[1]&0x000f,1,10,0);
		int2lcdyx(SK_SIGN[2]&0x000f,2,10,0);

		int2lcdyx(SK_LCD_EN[0]&0x000f,0,13,0);
		int2lcdyx(SK_LCD_EN[1]&0x000f,1,13,0);
		int2lcdyx(SK_LCD_EN[2]&0x000f,2,13,0);
		//int2lcdyx(adc_buff_ext_[1],1,19,0);
		//int2lcdyx(plazma_suz[2],2,4,0);
		//int2lcdyx(plazma_suz[3],3,4,0); */
		}  

	else if(sub_ind==3)
     	{
     	bgnd_par("КЕ                  ",
     	         "                    ",
     	         "                   ^",
     	         "                   &");

	int2lcdyx(spc_stat,0,5,0);
	int2lcdyx(__ee_spc_stat,0,9,0);
	int2lcdyx(lc640_read_int(EE_SPC_STAT),0,13,0);

	int2lcdyx(spc_bat,1,5,0);
	int2lcdyx(__ee_spc_bat,1,9,0);
	int2lcdyx(lc640_read_int(EE_SPC_BAT),1,13,0);

	int2lcdyx(bat_u_old_cnt,0,19,0);
	
	
	int2lcdyx(bat[0]._zar_cnt_ke,2,5,0);
	int2lcdyx(lc640_read_int(ADR_EE_BAT_ZAR_CNT_KE[0]),2,10,0);	
	int2lcdyx(bat[0]._u_old[0],2,14,0);
	int2lcd_mmm(bat[0]._Ib,'^',2);

	int2lcdyx(bat[1]._zar_cnt_ke,3,5,0);
	int2lcdyx(lc640_read_int(ADR_EE_BAT_ZAR_CNT_KE[1]),3,10,0);
	int2lcdyx(bat[1]._Ub,3,14,0);
	int2lcd_mmm(bat[1]._Ib,'&',2);	

	int2lcdyx(spc_phase,1,15,0);
	int2lcdyx(__ee_spc_phase,1,17,0);
	int2lcdyx(lc640_read_int(EE_SPC_PHASE),1,19,0);
	
		

/*	    		int2lcdyx(adc_net_buff_cnt,0,4,0);

		    	int2lcdyx((short)(main_power_buffer[0]>>12),0,19,0);
			int2lcdyx((short)(main_power_buffer[1]>>12),1,19,0);
			int2lcdyx((short)(main_power_buffer[2]>>12),2,19,0);
			int2lcdyx((short)(main_power_buffer[3]>>12),3,19,0);

		    	int2lcdyx((net_buff_),2,5,0); */


		   
		    


/*		int2lcdyx(load_U,0,4,0);
		int2lcdyx(load_I,1,4,0);
		lcd_buffer[44]='a';
		int2lcd_mmm((bat[0]._Ib)/10,'a',1);
		lcd_buffer[64]='a';
		int2lcd_mmm((bat[1]._Ib)/10,'a',1);

 		int2lcdyx(u_necc,0,8,0);

		
		
		lcd_buffer[14]='.';
		lcd_buffer[34]='.';
		int2lcdyx(Isumm,0,15,1);		
		int2lcdyx(Isumm_,1,15,1);


		int2lcdyx(cntrl_stat,0,19,0);
		int2lcdyx(num_necc,1,19,0);
		
		
		  
//		int2lcdyx(cntrl_stat,0,15,0);
		 
		//int2lcdyx(cntrl_plazma,1,3,0);
		//lcd_buffer[30]='a';
		int2lcd_mmm(Ibmax,'a',0);
		int2lcdyx(IZMAX,1,14,0);

		lcd_buffer[65]='a';
		int2lcd_mmm(bat[0]._Ib,'a',0);

		lcd_buffer[70]='a';
		int2lcd_mmm(bat[1]._Ib,'a',0); 

		lcd_buffer[75]='a';
		int2lcd_mmm(Ibmax,'a',0); 

	//	int2lcdyx(IMAX,2,3,0);
		
		

	//	int2lcdyx(IZMAX,3,19,0);

		//int2lcdyx(num_necc_Imax,3,6,0);
		//int2lcdyx(num_necc_Imin,3,12,0);


 //    	lcd_buffer[4]='a';            
 //    	int2lcd_mmm(Ibat,'a',1);   int2lcdyx(cntrl_stat,0,9,0);          int2lcdyx(hour_apv_cnt,0,13,0);                             char2lcdhyx(St_[0],0,19);  
 //    	int2lcdyx(Ubat,1,4,0);     int2lcdyx(main_apv_cnt,1,9,0);        int2lcdyx(lc640_read_int(bps1_AVAR_PTR),1,13,0);            char2lcdhyx(St_[1],1,19);
 //    	int2lcdyx(Us[0],2,4,0);  int2lcdyx(apv_cnt_1,2,9,0);           int2lcdyx(lc640_read_int(SRC1_AVAR_CNT),2,13,0);                                     int2lcdhyx(av_stat,2,19);
 //    	int2lcdyx(Us[1],3,4,0);  int2lcdyx(reset_apv_cnt,3,9,0);                                            int2lcdyx(plazma,3,19,0);
     	//int2lcd(plazma,'(',0);

     	//int2lcd(Us[0],'#',1);
     	//int2lcd(Us[1],'$',1);
     	//int2lcd(Is[0],'%',1);
     	//int2lcd(Is[1],'^',1);
    // 	int2lcd(bat[0]._Ub,'<',1);
    // 	int2lcd_mmm(bat[0]._Ib,'>',2);
 //    	char2lcdhyx(St_[0],3,13);
 //    	char2lcdhyx(St_[1],3,19);
 //    	char2lcdhyx(St,3,5);  */
		}

	else if(sub_ind==4)
     	{
     	bgnd_par(" АВАРИИ             ",
     	         "                    ",
     	         "                    ",
     	         "                    ");

		int2lcdyx(main_10Hz_cnt,0,7,0);
		int2lcdyx(bat[0]._av,0,10,0);
		int2lcdyx(bat[1]._av,0,12,0);
		char2lcdhyx(rele_stat,0,19);

 		long2lcdhyx(avar_stat,1,7);
		long2lcdhyx(avar_stat_old,2,7);
		long2lcdhyx(avar_ind_stat,3,7);

		long2lcdhyx(avar_stat_new,2,19);
		long2lcdhyx(avar_stat_offed,3,19);



	//	int2lcdyx(bat[0]._Ub,1,15,0);
	//	int2lcdyx(bat[1]._rel_stat,2,15,0);


/*		int2lcdyx(mat_temper,0,7,0);
		int2lcdyx(load_U,0,11,0);  
		int2lcdyx(cntrl_stat,0,15,0);
		int2lcdyx(cntrl_stat_old,0,19,0); 
		int2lcdyx(cntrl_plazma,1,3,0);
		lcd_buffer[30]='a';
		int2lcd_mmm(Ibmax,'a',0);
		int2lcdyx(IZMAX,1,14,0);*/

		}
   else if(sub_ind==5)
     	{
     	bgnd_par("**                  ",
     		    "                    ",
     		    "                    ",
     		    "                    ");
     		    


     	
     	int2lcdyx(vz_cnt_s_,0,6,0);
		int2lcdyx(vz_cnt_s,1,6,0);
		int2lcdyx(vz_cnt_h,2,6,0);
		int2lcdyx(vz_cnt_h_,3,6,0);

     	int2lcdyx(__ee_vz_cnt,3,12,0);
		int2lcdyx(lc640_read_int(EE_VZ_CNT),1,10,0);
	/*		int2lcdyx(eb2_data[10],2,10,0);
/*		int2lcdyx(eb2_data[11],3,10,0);

     	int2lcdyx(eb2_data[12],0,14,0);
		int2lcdyx(eb2_data[13],1,14,0);
		int2lcdyx(eb2_data[14],2,14,0);
		int2lcdyx(eb2_data[15],3,14,0);
     	
     	int2lcdyx(eb2_data[16],0,18,0);
		int2lcdyx(eb2_data[17],1,18,0);
		int2lcdyx(eb2_data[18],2,18,0);
		int2lcdyx(eb2_data[19],3,18,0);*/

	/*	int2lcdyx(eb2_data_short[0],0,13,0);
		int2lcdyx(eb2_data_short[1],1,13,0);
		int2lcdyx(eb2_data_short[2],2,13,0);

		int2lcdyx(eb2_data_short[3],0,19,0);
		int2lcdyx(eb2_data_short[4],1,19,0);
		int2lcdyx(eb2_data_short[5],2,19,0);  */

     	/*int2lcdyx(eb2_data[20],0,10,0);
		int2lcdyx(eb2_data[21],1,10,0);
		int2lcdyx(eb2_data[22],2,10,0);
		int2lcdyx(eb2_data[23],3,10,0);*/

    		}
    else if(sub_ind==6)
     	{
     	bgnd_par("                    ",
     		    "                    ",
     		    "                    ",
     		    "                    ");
     		    
     	/*int2lcdyx(ad7705_buff[0][0],0,4,0);
     	int2lcdyx(ad7705_buff[0][1],0,9,0);
     	int2lcdyx(ad7705_buff[0][2],0,14,0);
     	int2lcdyx(ad7705_buff[0][3],0,19,0);
     	int2lcdyx(ad7705_buff[0][4],1,4,0);
     	int2lcdyx(ad7705_buff[0][5],1,9,0);
     	int2lcdyx(ad7705_buff[0][6],1,14,0);
     	int2lcdyx(ad7705_buff[0][7],1,19,0);
     	int2lcdyx(ad7705_buff[0][8],2,4,0);
     	int2lcdyx(ad7705_buff[0][9],2,9,0);
     	int2lcdyx(ad7705_buff[0][10],2,14,0);
     	int2lcdyx(ad7705_buff[0][11],2,19,0);
     	int2lcdyx(ad7705_buff[0][12],3,4,0);
     	int2lcdyx(ad7705_buff[0][13],3,9,0);
     	int2lcdyx(ad7705_buff[0][14],3,14,0);
     	int2lcdyx(ad7705_buff[0][15],3,19,0);*/

		int2lcdyx(adc_buff_[0],0,4,0);
    		int2lcdyx(adc_buff_[1],0,9,0);
     	int2lcdyx(adc_buff_[2],0,14,0);
     	int2lcdyx(adc_buff_[3],0,19,0); 
     	int2lcdyx(adc_buff_[4],1,4,0);	
     	int2lcdyx(adc_buff_[5],1,9,0);
     	int2lcdyx(adc_buff_[6],1,14,0);
     	int2lcdyx(adc_buff_[7],1,19,0); 
     	int2lcdyx(adc_buff_[8],2,4,0);
     	int2lcdyx(adc_buff_[9],2,9,0);
     	int2lcdyx(adc_buff_[10],2,14,0);
     	int2lcdyx(adc_buff_[11],2,19,0);
     	int2lcdyx(adc_buff_[12],3,4,0);
     	int2lcdyx(adc_buff_[13],3,9,0);
     	int2lcdyx(adc_buff_[14],3,14,0);
     	int2lcdyx(adc_buff_[15],3,19,0);
    	}  		  		

   else if(sub_ind==7)
     	{
     	bgnd_par("7                   ",
     		    "                    ",
     		    "                    ",
     		    "                    ");
     		    
		int2lcdyx(main_vent_pos,0,19,0);
		int2lcdyx(TBOXMAX,1,2,0);
		int2lcdyx(TBOXREG,2,2,0);
		int2lcdyx(t_box,3,2,0);

		int2lcdyx(adc_buff_ext_[0],1,10,0);
		int2lcdyx(adc_buff_ext_[1],2,10,0);
		int2lcdyx(adc_buff_ext_[2],3,10,0);
    		}
    else if(sub_ind==8)
     	{
     	bgnd_par("                    ",
     		    "                    ",
     		    "                    ",
     		    "                    ");
     	int2lcdyx(ibt._T[0],0,2,0);
		int2lcdyx(ibt._T[1],1,2,0);
     	int2lcdyx(ibt._T[2],2,2,0);
		int2lcdyx(ibt._T[3],3,2,0);
		
     	int2lcdyx(ibt._nd[0],0,4,0);
		int2lcdyx(ibt._nd[1],1,4,0);
     	int2lcdyx(ibt._nd[2],2,4,0);
		int2lcdyx(ibt._nd[3],3,4,0);	    

     	int2lcdyx(ibt._T_dispers[0],0,7,0);
		int2lcdyx(ibt._T_dispers[1],1,7,0);
     	int2lcdyx(ibt._T_dispers[2],2,7,0);
		int2lcdyx(ibt._T_dispers[3],3,7,0);
			    
		int2lcdyx(ibt._avg1,0,19,0);
		int2lcdyx(ibt._max_dispers_num,1,19,0);
		int2lcdyx(t_box,3,19,0);
     	}		     	

    else if(sub_ind==9)
     	{
     	bgnd_par(	"SNTP                ",
     		    	"                    ",
     		    	"                    ",
     		    	"                    ");
     	
		int2lcdyx(socket_udp,0,10,0);
		int2lcdyx(udp_callback_cnt,0,15,0);
		int2lcdyx(udp_callback_cnt1,0,19,0);
		//int2lcdyx(udp_callback_plazma[0],1,3,0);
		int2lcdyx((U16)full_days_since_2000_01_01,1,4,0);
		int2lcdyx(this_year,1,7,0);
		int2lcdyx(this_month,1,10,0);
		int2lcdyx(day_of_month,1,13,0);
		int2lcdyx(hour_in_this_day,1,16,0);
		int2lcdyx(min_in_this_hour,1,19,0);
		int2lcdyx(udp_callback_plazma[4],3,5,0);
		int2lcdyx(udp_callback_plazma[5],2,3,0);
		int2lcdyx(udp_callback_plazma[6],2,7,0);
		int2lcdyx(udp_callback_plazma[7],2,11,0);
		int2lcdyx(udp_callback_plazma[8],2,15,0);
		int2lcdyx(udp_callback_plazma[9],3,13,0);
		int2lcdyx(udp_callback_plazma[10],3,19,0);
		int2lcdyx(sec_in_this_min,3,10,0);
     	}		    
    else if(sub_ind==10)
     	{
     	bgnd_par(	"MR                  ",
     		    	"                    ",
     		    	"                    ",
     		    	"                    ");
     	
		/*char2lcdhyx(modbus_an_buffer[0],0,2);
		char2lcdhyx(modbus_an_buffer[1],1,2);
		char2lcdhyx(modbus_an_buffer[2],2,2);
		char2lcdhyx(modbus_an_buffer[3],3,2);
		char2lcdhyx(modbus_an_buffer[4],0,5);
		char2lcdhyx(modbus_an_buffer[5],1,5);
		char2lcdhyx(modbus_an_buffer[6],2,5);
		char2lcdhyx(modbus_an_buffer[7],3,5);
		char2lcdhyx(modbus_an_buffer[8],0,8);
		char2lcdhyx(modbus_an_buffer[9],1,8);
		char2lcdhyx(modbus_an_buffer[10],2,8);
		char2lcdhyx(modbus_an_buffer[11],3,8);*/


		int2lcdyx(modbus_rtu_plazma[0],1,4,0);
		int2lcdyx(modbus_rtu_plazma[1],2,4,0);
		int2lcdyx(modbus_rtu_plazma[2],3,4,0);
		int2lcdyx(modbus_rtu_plazma[3],0,8,0);
		int2lcdyx(modbus_rtu_plazma[4],1,8,0);
		int2lcdyx(modbus_rtu_plazma[5],2,8,0);
		int2lcdyx(modbus_rtu_plazma[6],3,8,0);
		int2lcdyx(modbus_rtu_plazma[7],0,12,0);
		int2lcdyx(modbus_rtu_plazma[8],1,12,0);
		int2lcdyx(modbus_rtu_plazma[9],2,12,0);
		int2lcdyx(modbus_rtu_plazma[10],3,12,0);


		int2lcdyx(modbus_plazma,0,17,0);
		//int2lcdyx(U_OUT_SET,0,8,0);
		//int2lcdyx(modbus_tcp_plazma[0],0,19,0);
		//int2lcdyx(modbus_tcp_plazma[1],0,15,0);
		int2lcdyx(modbus_rx_buffer_ptr,3,17,0);
		int2lcdhyx(modbus_crc_plazma[0],1,17);
		int2lcdhyx(modbus_crc_plazma[1],2,17);
     	}
    else if(sub_ind==11)
     	{
     	bgnd_par(	"MT                  ",
     		    	"                    ",
     		    	"                    ",
     		    	"                    ");
     	
		int2lcdyx(socket_udp,0,10,0);
		int2lcdyx(udp_callback_cnt,0,15,0);
		int2lcdyx(udp_callback_cnt1,0,19,0);
		//int2lcdyx(udp_callback_plazma[0],1,3,0);
		int2lcdyx((U16)full_days_since_2000_01_01,1,4,0);
		int2lcdyx(this_year,1,7,0);
		int2lcdyx(this_month,1,10,0);
		int2lcdyx(day_of_month,1,13,0);
		int2lcdyx(hour_in_this_day,1,16,0);
		int2lcdyx(min_in_this_hour,1,19,0);
		int2lcdyx(udp_callback_plazma[4],3,5,0);
		int2lcdyx(udp_callback_plazma[5],2,3,0);
		int2lcdyx(udp_callback_plazma[6],2,7,0);
		int2lcdyx(udp_callback_plazma[7],2,11,0);
		int2lcdyx(udp_callback_plazma[8],2,15,0);
		int2lcdyx(udp_callback_plazma[9],3,13,0);
		int2lcdyx(udp_callback_plazma[10],3,19,0);
		int2lcdyx(sec_in_this_min,3,10,0);
     	}					 		  			
     }

else if((ind==iAv_view)||(ind==iAv_view_avt))
	{
//	unsigned short tempUI,tempUI_;
//    	unsigned long tempUL;
	
	bgnd_par(sm_,sm_,sm_,sm_);
	if(sub_ind==0)
		{	
		if(avar_stat&0x00000001)
			{
			bgnd_par(	"    Авария  сети    ",
				    	"    не устранена    ",
				    	sm_,sm_); 
			int2lcd(net_U,']',0);
			}
    		else 
			{
	    		bgnd_par(	"    Авария  сети    ",
	    				"     устранена      ",
					sm_,sm_); 
			}
		}
	else if((sub_ind==1)||(sub_ind==2))
		{
		if(avar_stat&(1<<sub_ind))
			{
			bgnd_par(	"   Авария бат. N!   ",
				    	"    не устранена    ",
				    	sm_,sm_); 
			}
    		else 
			{
	    		bgnd_par(	"   Авария бат. N!   ",
	    				"     устранена      ",
					sm_,sm_); 
		
		     }
		int2lcd(sub_ind,'!',0);
		} 
     
	else if((sub_ind>=3)&&(sub_ind<=14))
		{
		if((sub_ind-2)<=9)					ptrs[0]=	"   Авария БПС N+    ";
		else 							ptrs[0]=	"   Авария БПС N +   ";
		if(bps[sub_ind-3]._last_avar==0)		ptrs[1]=	"     перегрев!!!    ";
		else if(bps[sub_ind-3]._last_avar==1)	ptrs[1]=	"  завышено Uвых!!!  ";	
		else if(bps[sub_ind-3]._last_avar==2)	ptrs[1]=	"  занижено Uвых!!!  ";	
		else if(bps[sub_ind-3]._last_avar==3)	ptrs[1]=	"    отключился!!!   ";
		if(avar_stat&(1<<sub_ind)) 			ptrs[2]=	"    не устранена    ";
		else								ptrs[2]=	"     устранена      ";	
										ptrs[3]=	"                    ";

		bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
		int2lcd((sub_ind-2),'+',0);
          
		//int2lcdxy(sub_ind,0x20,0);

		} 
		
	else if(sub_ind==25)
		{ 

		if(sk_av_stat[0]==sasON)
			{
			bgnd_par(	"   Авария СК. N1    ",
				    	"    не устранена    ",
				    	sm_,sm_); 
			}
    		else 
			{
	    		bgnd_par(	"   Авария СК. N1    ",
	    				"     устранена      ",
					sm_,sm_); 
		
		     }

		}

	else if(sub_ind==26)
		{ 

		if(sk_av_stat[1]==sasON)
			{
			bgnd_par(	"   Авария СК. N2    ",
				    	"    не устранена    ",
				    	sm_,sm_); 
			}
    		else 
			{
	    		bgnd_par(	"   Авария СК. N2    ",
	    				"     устранена      ",
					sm_,sm_); 
		
		     }

		}

	else if(sub_ind==27)
		{ 

		if(sk_av_stat[2]==sasON)
			{
			bgnd_par(	"   Авария СК. N3    ",
				    	"    не устранена    ",
				    	sm_,sm_); 
			}
    		else 
			{
	    		bgnd_par(	"   Авария СК. N3    ",
	    				"     устранена      ",
					sm_,sm_); 
		
		     }

		}

	else if(sub_ind==28)
		{ 

		if(sk_av_stat[3]==sasON)
			{
			bgnd_par(	"   Авария СК. N4    ",
				    	"    не устранена    ",
				    	sm_,sm_); 
			}
    		else 
			{
	    		bgnd_par(	"   Авария СК. N4    ",
	    				"     устранена      ",
					sm_,sm_); 
		
		     }

		}


	else if(sub_ind==5)
		{

		}

	else if(sub_ind==6)
		{

		}

	else if(sub_ind==7)
		{

		} 
		
	else if(sub_ind==8)
		{

		}

	else if(sub_ind==9)
		{

		}

	else if(sub_ind==10)
		{

		}
	    		     
	else if(sub_ind==11)
		{

		} 
		
	else if(sub_ind==12)
		{

		}

	else if(sub_ind==13)
		{

		}

	else if(sub_ind==14)
		{

		}

	else if(sub_ind==15)
		{

		} 
					
	} 
#ifndef _DEBUG_	
else if(ind==iAvz)
	{
	
 	if(AVZ==AVZ_1) 		ptrs[0]=	" раз в месяц        ";
	else if(AVZ==AVZ_2) 	ptrs[0]=	" раз в 2 месяца     ";
	else if(AVZ==AVZ_3) 	ptrs[0]=	" раз в 3 месяца     "; 
	else if(AVZ==AVZ_6) 	ptrs[0]=	" раз в полгода      ";
	else if(AVZ==AVZ_12) 	ptrs[0]=	" раз в год          ";
	else 				ptrs[0]=	" выключен           "; 
	
	ptrs[1]=						" Длительность    (ч.";
	if(AVZ!=AVZ_OFF)
		{
		ptrs[2]=					" очередное включение";
		ptrs[3]=					"  0%  &0^  0@:0#:0$ ";
		ptrs[4]=					sm_exit;
		}
	else ptrs[2]=						sm_exit;

	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>1) index_set=sub_ind-1;
	if((sub_ind==2)&&(AVZ!=AVZ_OFF)) index_set=2;
	
	bgnd_par(	"   АВТОМАТИЧЕСКИЙ   ",
			"ВЫРАВНИВАЮЩИЙ ЗАРЯД ",
			ptrs[index_set],
			ptrs[index_set+1]);

	pointer_set(2);
		
	int2lcd(HOUR_AVZ,'@',0);
	int2lcd(MIN_AVZ,'#',0);
	int2lcd(SEC_AVZ,'$',0);
	int2lcd(DATE_AVZ,'%',0);
	int2lcd(YEAR_AVZ,'^',0);

	sub_bgnd(sm_mont[MONTH_AVZ],'&',-2);

	int2lcd(AVZ_TIME,'(',0);
	
	}


else if(ind==iTst_RSTKM)
	{
	ptrs[0]=						" Реле аварии        ";
     ptrs[1]=						" сети              !";
     ptrs[2]=						" Реле               ";
     ptrs[3]=						" освещения         @";
     ptrs[4]=						" Реле отключения    ";
     ptrs[5]=						" нагрузки          &";
     ptrs[6]=						" Перемешивающий     ";
     ptrs[7]=						" вентилятор        #";
     ptrs[8]=						" Реле аварий        "; 
     ptrs[9]=                           " общее             %";
     ptrs[10]=						" Приточный          ";
     ptrs[11]=						" вентилятор        l";

     ptrs[12]=						" Реле               ";
     ptrs[13]=						" самокалибровки    ^";

	if((sub_ind==12)&&(bFL2))
		{
		ptrs[12]=					" Iбат1 =        <A  ";
     	ptrs[13]=					" Iбат2 =        >A  ";
		}
     ptrs[14]=						" Реле бат.N1       (";
     ptrs[15]=						" Реле бат.N2       )";
	ptrs[16]=						" БПС N1             ";
     ptrs[17]=						" БПС N2             ";
     ptrs[18]=						" БПС N3             ";
	ptrs[19]=						" БПС N4             ";
     ptrs[20]=						" БПС N5             ";
     ptrs[21]=						" БПС N6             ";
	ptrs[22]=						" БПС N7             ";
     ptrs[23]=						" БПС N8             ";
     ptrs[24]=						" БПС N9             ";               
	ptrs[25]=						" БПС N10            ";
     ptrs[26]=						" БПС N11            ";
     ptrs[27]=						" БПС N12            ";               
	ptrs[16+NUMIST]=				" Сброс              ";
	ptrs[17+NUMIST]=				" Сброс пл.расш.    {";
	ptrs[18+NUMIST]=				" Выход              ";
	ptrs[19+NUMIST]=				"                    ";
	ptrs[20+NUMIST]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("        ТЕСТ        ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);


	int2lcd_mmm(bat[0]._Ib,'<',2);
	int2lcd_mmm(bat[1]._Ib,'>',2);

	if(tst_state[0]==tst1) sub_bgnd("ВКЛ.",'!',-3);
	if(tst_state[0]==tst2) sub_bgnd("ВЫКЛ.",'!',-4);
	else sub_bgnd("РАБОЧ.",'!',-5);

	if(tst_state[1]==tst1) sub_bgnd("ВКЛ.",'@',-3);
	if(tst_state[1]==tst2) sub_bgnd("ВЫКЛ.",'@',-4);
	else sub_bgnd("РАБОЧ.",'@',-5);

	if(tst_state[9]==tst1) sub_bgnd("ВКЛ.",'&',-3);
	if(tst_state[9]==tst2) sub_bgnd("ВЫКЛ.",'&',-4);
	else sub_bgnd("РАБОЧ.",'&',-5);

	if(tst_state[2]==tst1) sub_bgnd("ВКЛ.",'#',-3);
	if(tst_state[2]==tst2) sub_bgnd("ВЫКЛ.",'#',-4);
	else sub_bgnd("РАБОЧ.",'#',-5);

	if(tst_state[3]==tst1) sub_bgnd("ВКЛ.",'%',-3);
	if(tst_state[3]==tst2) sub_bgnd("ВЫКЛ.",'%',-4);
	else sub_bgnd("РАБОЧ.",'%',-5);

	if(tst_state[4]==tst1) sub_bgnd("ВКЛ.",'^',-3);
	if(tst_state[4]==tst2) sub_bgnd("ВЫКЛ.",'^',-4);
	else sub_bgnd("РАБОЧ.",'^',-5);

	if(tst_state[7]==tst1) sub_bgnd("ВЫКЛ.",'(',-4);
	else sub_bgnd("РАБОЧ.",'(',-5);

	if(tst_state[8]==tst1) sub_bgnd("ВЫКЛ.",')',-4);
	else sub_bgnd("РАБОЧ.",')',-5);

	if((tst_state[10]>=1)&&(tst_state[10]<=20)) int2lcd(tst_state[10],'l',0);
	else sub_bgnd("РАБОЧ.",'l',-5);

	
	if(sub_ind==0)
		{
		if(tst_state[0]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,1,5);
		else if(tst_state[0]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,0,5);
		}
	else if(sub_ind==2)
		{
		if(tst_state[1]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_LIGHT,1,5);
		else if(tst_state[1]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_LIGHT,0,5);
		}
	else if(sub_ind==4)
		{
		if(tst_state[9]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_LOAD_OFF,1,5);
		else if(tst_state[9]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_LOAD_OFF,0,5);
		}
	else if(sub_ind==6)
		{
		if(tst_state[2]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,1,5);
		else if(tst_state[2]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,0,5);
		}
	else if(sub_ind==8)
		{
		if(tst_state[3]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_COMM,1,5);
		else if(tst_state[3]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_COMM,0,5);
		}
	else if(sub_ind==10)
		{
		if((tst_state[10]>=1)&&(tst_state[10]<=20))mess_send(MESS2VENT_HNDL,PARAM_VENT_CB,tst_state[10],5);
		}
	else if(sub_ind==12)
		{
		if(tst_state[4]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,5);
		else if(tst_state[4]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,0,5);
		}



	else if(sub_ind==14)
		{
		if(tst_state[7]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<0),10);

		}
	else if(sub_ind==15)
		{
		if(tst_state[8]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<1),10);

		}

	//int2lcdyx(cv_pos,0,2,0);
	//char2lcdbyx(rele_stat,0,7);
	int2lcd(ext_can_cnt,'{',0);
 	}



else if(ind==iTst_3U)
	{
	ptrs[0]=						" Реле аварии        ";
     ptrs[1]=						" сети              !";
     ptrs[2]=						" Реле аварии        ";
     ptrs[3]=						" батареи №1        @";
     ptrs[4]=						" Реле аварии        ";
     ptrs[5]=						" батареи №2        &";
     ptrs[6]=						" Реле аварии        ";
     ptrs[7]=						" источников        #";
     ptrs[8]=						" Реле               ";
     ptrs[9]=						" самокалибровки    ^";

	if((sub_ind==8)&&(bFL2))
		{
		ptrs[8]=					" Iбат1 =        <A  ";
     	ptrs[9]=					" Iбат2 =        >A  ";
		}
     ptrs[10]=						" Реле бат.N1       (";
     ptrs[11]=						" Реле бат.N2       )";
	ptrs[12]=						" БПС N1             ";
     ptrs[13]=						" БПС N2             ";
     ptrs[14]=						" БПС N3             ";
	ptrs[15]=						" БПС N4             ";
     ptrs[16]=						" БПС N5             ";
     ptrs[17]=						" БПС N6             ";
	ptrs[18]=						" БПС N7             ";
     ptrs[19]=						" БПС N8             ";
     ptrs[20]=						" БПС N9             ";               
	ptrs[21]=						" БПС N10            ";
     ptrs[22]=						" БПС N11            ";
     ptrs[23]=						" БПС N12            ";               
	ptrs[12+NUMIST]=				" Сброс              ";
	ptrs[13+NUMIST]=				" Выход              ";
	ptrs[14+NUMIST]=				"                    ";
	ptrs[15+NUMIST]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("        ТЕСТ        ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);


	int2lcd_mmm(bat[0]._Ib,'<',2);
	int2lcd_mmm(bat[1]._Ib,'>',2);

	if(tst_state[0]==tst1) sub_bgnd("ВКЛ.",'!',-3);
	if(tst_state[0]==tst2) sub_bgnd("ВЫКЛ.",'!',-4);
	else sub_bgnd("РАБОЧ.",'!',-5);

	if(tst_state[1]==tst1) sub_bgnd("ВКЛ.",'@',-3);
	if(tst_state[1]==tst2) sub_bgnd("ВЫКЛ.",'@',-4);
	else sub_bgnd("РАБОЧ.",'@',-5);

	if(tst_state[2]==tst1) sub_bgnd("ВКЛ.",'&',-3);
	if(tst_state[2]==tst2) sub_bgnd("ВЫКЛ.",'&',-4);
	else sub_bgnd("РАБОЧ.",'&',-5);

	if(tst_state[3]==tst1) sub_bgnd("ВКЛ.",'#',-3);
	if(tst_state[3]==tst2) sub_bgnd("ВЫКЛ.",'#',-4);
	else sub_bgnd("РАБОЧ.",'#',-5);

	if(tst_state[4]==tst1) sub_bgnd("ВКЛ.",'^',-3);
	if(tst_state[4]==tst2) sub_bgnd("ВЫКЛ.",'^',-4);
	else sub_bgnd("РАБОЧ.",'^',-5);


	if(tst_state[5]==tst1) sub_bgnd("ВЫКЛ.",'(',-4);
	else sub_bgnd("РАБОЧ.",'(',-5);

	if(tst_state[6]==tst1) sub_bgnd("ВЫКЛ.",')',-4);
	else sub_bgnd("РАБОЧ.",')',-5);


	
	if(sub_ind==0)
		{
		if(tst_state[0]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,1,5);
		else if(tst_state[0]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,0,5);
		}
	else if(sub_ind==2)
		{
		if(tst_state[1]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT1,1,5);
		else if(tst_state[1]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT1,0,5);
		}
	else if(sub_ind==4)
		{
		if(tst_state[2]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT2,1,5);
		else if(tst_state[2]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT2,0,5);
		}
	else if(sub_ind==6)
		{
		if(tst_state[3]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,1,5);
		else if(tst_state[3]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,0,5);
		}
	else if(sub_ind==8)
		{
		if(tst_state[4]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,5);
		else if(tst_state[4]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,0,5);
		}
	else if(sub_ind==10)
		{
		if(tst_state[5]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<0),10);

		}
	else if(sub_ind==11)
		{
		if(tst_state[6]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<1),10);

		}
	int2lcdyx(sub_ind,0,19,0);
	int2lcdyx(index_set,0,17,0);
	int2lcdyx(bat[0]._cnt_to_block,0,3,0);
	int2lcdyx(bat[1]._cnt_to_block,0,7,0);
	//int2lcdyx(sub_ind1,0,1,0);
 	}



else if(ind==iTst_6U)
	{
	ptrs[0]=						" Реле аварии        ";
     ptrs[1]=						" сети              !";
     ptrs[2]=						" Реле аварии        ";
     ptrs[3]=						" батареи N1        @";
     ptrs[4]=						" Реле аварии        ";
     ptrs[5]=						" батареи N2        &";
     ptrs[6]=						" Реле аварии        ";
     ptrs[7]=						" БПСов             #";
     ptrs[8]=						" Реле вент.        %";
     ptrs[9]=						" Реле               ";
     ptrs[10]=						" самокалибровки    ^";
	if((sub_ind==9)&&(bFL2))
		{
		ptrs[9]=					" Iбат1 =        <A  ";
     	ptrs[10]=					" Iбат2 =        >A  ";
		}
     ptrs[11]=						" Реле бат.N1       (";
     ptrs[12]=						" Реле бат.N2       )";
	ptrs[13]=						" БПС N1             ";
     ptrs[14]=						" БПС N2             ";
     ptrs[15]=						" БПС N3             ";
	ptrs[16]=						" БПС N4             ";
     ptrs[17]=						" БПС N5             ";
     ptrs[18]=						" БПС N6             ";
	ptrs[19]=						" БПС N7             ";
     ptrs[20]=						" БПС N8             ";
     ptrs[21]=						" БПС N9          ";               
	ptrs[22]=						" БПС N10            ";
     ptrs[23]=						" БПС N11            ";
     ptrs[24]=						" БПС N12            ";
	 ptrs[13+NUMIST]=				" Сброс              ";               
	ptrs[14+NUMIST]=				" Выход              ";
	ptrs[15+NUMIST]=				"                    ";
	ptrs[16+NUMIST]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("        ТЕСТ        ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
/*	int2lcdyx(sub_ind,0,19,0);
	int2lcdyx(index_set,0,17,0);
	int2lcdyx(tst_state[0],0,15,0); */

	int2lcd_mmm(bat[0]._Ib,'<',2);
	int2lcd_mmm(bat[1]._Ib,'>',2);

	if(tst_state[0]==tst1) sub_bgnd("ВКЛ.",'!',-3);
	if(tst_state[0]==tst2) sub_bgnd("ВЫКЛ.",'!',-4);
	else sub_bgnd("РАБОЧ.",'!',-5);

	if(tst_state[1]==tst1) sub_bgnd("ВКЛ.",'@',-3);
	if(tst_state[1]==tst2) sub_bgnd("ВЫКЛ.",'@',-4);
	else sub_bgnd("РАБОЧ.",'@',-5);

	if(tst_state[9]==tst1) sub_bgnd("ВКЛ.",'&',-3);
	if(tst_state[9]==tst2) sub_bgnd("ВЫКЛ.",'&',-4);
	else sub_bgnd("РАБОЧ.",'&',-5);

	if(tst_state[2]==tst1) sub_bgnd("ВКЛ.",'#',-3);
	if(tst_state[2]==tst2) sub_bgnd("ВЫКЛ.",'#',-4);
	else sub_bgnd("РАБОЧ.",'#',-5);

	if(tst_state[3]==tst1) sub_bgnd("ВКЛ.",'%',-3);
	if(tst_state[3]==tst2) sub_bgnd("ВЫКЛ.",'%',-4);
	else sub_bgnd("РАБОЧ.",'%',-5);

	if(tst_state[4]==tst1) sub_bgnd("ВКЛ.",'^',-3);
	if(tst_state[4]==tst2) sub_bgnd("ВЫКЛ.",'^',-4);
	else sub_bgnd("РАБОЧ.",'^',-5);

	if(tst_state[7]==tst1) sub_bgnd("ВЫКЛ.",'(',-4);
	else sub_bgnd("РАБОЧ.",'(',-5);

	if(tst_state[8]==tst1) sub_bgnd("ВЫКЛ.",')',-4);
	else sub_bgnd("РАБОЧ.",')',-5);
	
	if(sub_ind==0)
		{
		if(tst_state[0]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,1,5);
		else if(tst_state[0]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,0,5);
		}
	else if(sub_ind==2)
		{
		if(tst_state[1]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT1,1,5);
		else if(tst_state[1]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT1,0,5);
		}
	else if(sub_ind==4)
		{
		if(tst_state[9]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT2,1,5);
		else if(tst_state[9]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT2,0,5);
		}
	else if(sub_ind==6)
		{
		if(tst_state[2]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,1,5);
		else if(tst_state[2]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,0,5);
		}
	else if(sub_ind==8)
		{
		if(tst_state[3]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,1,5);
		else if(tst_state[3]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,0,5);
		}
	else if(sub_ind==9)
		{
		if(tst_state[4]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,5);
		else if(tst_state[4]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,0,5);
		}
	else if(sub_ind==11)
		{
		if(tst_state[7]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<0),10);

		}
	else if(sub_ind==12)
		{
		if(tst_state[8]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<1),10);

		}

		//char2lcdbyx(GET_REG(LPC_GPIO0->FIOPIN,4,8),0,19);


 	}

else if(ind==iTst_220)
	{
	ptrs[0]=						" Реле аварии        ";
     ptrs[1]=						" сети              !";
     ptrs[2]=						" Реле аварии        ";
     ptrs[3]=						" батарей           @";
     ptrs[4]=						" Реле аварии        ";
     ptrs[5]=						" БПСов             #";
     ptrs[6]=						" Реле вент.        %";
     ptrs[7]=						" Реле               ";
     ptrs[8]=						" самокалибровки    ^";
	if((sub_ind==7)&&(bFL2))
		{
		ptrs[7]=					" Iбат1 =        <A  ";
     	ptrs[8]=					" Iбат2 =        >A  ";
		}
     ptrs[9]=						" Реле бат.N1       (";
     ptrs[10]=						" Реле бат.N2       )";
	ptrs[11]=						" БПС N1             ";
     ptrs[12]=						" БПС N2             ";
     ptrs[13]=						" БПС N3             ";
	ptrs[14]=						" БПС N4             ";
     ptrs[15]=						" БПС N5             ";
     ptrs[16]=						" БПС N6             ";
	ptrs[17]=						" БПС N7             ";
     ptrs[18]=						" БПС N8             ";
     ptrs[19]=						" БПС N9            ";               
	ptrs[20]=						" БПС N10            ";
     ptrs[21]=						" БПС N11            ";
     ptrs[22]=						" БПС N12            ";               
	ptrs[11+NUMIST]=				" Выход              ";
	ptrs[12+NUMIST]=				" Сброс              ";
	ptrs[13+NUMIST]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("        ТЕСТ        ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
/*	int2lcdyx(sub_ind,0,19,0);
	int2lcdyx(index_set,0,17,0);
	int2lcdyx(tst_state[0],0,15,0); */

	int2lcd_mmm(bat[0]._Ib,'<',2);
	int2lcd_mmm(bat[1]._Ib,'>',2);

	if(tst_state[0]==tst1) sub_bgnd("ВКЛ.",'!',-3);
	if(tst_state[0]==tst2) sub_bgnd("ВЫКЛ.",'!',-4);
	else sub_bgnd("РАБОЧ.",'!',-5);

	if(tst_state[1]==tst1) sub_bgnd("ВКЛ.",'@',-3);
	if(tst_state[1]==tst2) sub_bgnd("ВЫКЛ.",'@',-4);
	else sub_bgnd("РАБОЧ.",'@',-5);

	if(tst_state[2]==tst1) sub_bgnd("ВКЛ.",'#',-3);
	if(tst_state[2]==tst2) sub_bgnd("ВЫКЛ.",'#',-4);
	else sub_bgnd("РАБОЧ.",'#',-5);

	if(tst_state[3]==tst1) sub_bgnd("ВКЛ.",'%',-3);
	if(tst_state[3]==tst2) sub_bgnd("ВЫКЛ.",'%',-4);
	else sub_bgnd("РАБОЧ.",'%',-5);

	if(tst_state[4]==tst1) sub_bgnd("ВКЛ.",'^',-3);
	if(tst_state[4]==tst2) sub_bgnd("ВЫКЛ.",'^',-4);
	else sub_bgnd("РАБОЧ.",'^',-5);

	if(tst_state[7]==tst1) sub_bgnd("ВЫКЛ.",'(',-4);
	else sub_bgnd("РАБОЧ.",'(',-5);

	if(tst_state[8]==tst1) sub_bgnd("ВЫКЛ.",')',-4);
	else sub_bgnd("РАБОЧ.",')',-5);
	
	if(sub_ind==0)
		{
		if(tst_state[0]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,1,5);
		else if(tst_state[0]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,0,5);
		}
	else if(sub_ind==2)
		{
		if(tst_state[1]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT,1,5);
		else if(tst_state[1]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT,0,5);
		}
	else if(sub_ind==4)
		{
		if(tst_state[2]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,1,5);
		else if(tst_state[2]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,0,5);
		}
	else if(sub_ind==6)
		{
		if(tst_state[3]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,1,5);
		else if(tst_state[3]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,0,5);
		}
	else if(sub_ind==7)
		{
		if(tst_state[4]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,5);
		else if(tst_state[4]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,0,5);
		}
	else if(sub_ind==9)
		{
		if(tst_state[7]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<0),10);

		}
	else if(sub_ind==10)
		{
		if(tst_state[8]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<1),10);

		}
	}	 

else if(ind==iTst_220_380)
	{
	ptrs[0]=						" Реле аварии        ";
     ptrs[1]=						" сети              !";
     ptrs[2]=						" Реле аварии        ";
     ptrs[3]=						" батареи N1        @";
     ptrs[4]=						" Реле аварии        ";
     ptrs[5]=						" БПСов             #";
	 if(RELE_VENT_LOGIC==1)
	 	{
		ptrs[6]=					" Реле вент.        %";
		}
	 else ptrs[6]=					" Реле Ав.БатN2     %";
     ptrs[7]=						" Реле               ";
     ptrs[8]=						" самокалибровки    ^";
	if((sub_ind==7)&&(bFL2))
		{
		ptrs[7]=					" Iбат1 =        <A  ";
     	ptrs[8]=					" Iбат2 =        >A  ";
		}
     ptrs[9]=						" Реле бат.N1       (";
     ptrs[10]=						" Реле бат.N2       )";
	ptrs[11]=						" БПС N1             ";
     ptrs[12]=						" БПС N2             ";
     ptrs[13]=						" БПС N3             ";
	ptrs[14]=						" БПС N4             ";
     ptrs[15]=						" БПС N5             ";
     ptrs[16]=						" БПС N6             ";
	ptrs[17]=						" БПС N7             ";
     ptrs[18]=						" БПС N8             ";
     ptrs[19]=						" БПС N9            ";               
	ptrs[20]=						" БПС N10            ";
     ptrs[21]=						" БПС N11            ";
     ptrs[22]=						" БПС N12            ";               
	ptrs[11+NUMIST]=				" Выход              ";
	ptrs[12+NUMIST]=				" Сброс              ";
	ptrs[13+NUMIST]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("        ТЕСТ        ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
/*	int2lcdyx(sub_ind,0,19,0);
	int2lcdyx(index_set,0,17,0);
	int2lcdyx(tst_state[0],0,15,0); */

	int2lcd_mmm(bat[0]._Ib,'<',2);
	int2lcd_mmm(bat[1]._Ib,'>',2);

	if(tst_state[0]==tst1) sub_bgnd("ВКЛ.",'!',-3);
	if(tst_state[0]==tst2) sub_bgnd("ВЫКЛ.",'!',-4);
	else sub_bgnd("РАБОЧ.",'!',-5);

	if(tst_state[1]==tst1) sub_bgnd("ВКЛ.",'@',-3);
	if(tst_state[1]==tst2) sub_bgnd("ВЫКЛ.",'@',-4);
	else sub_bgnd("РАБОЧ.",'@',-5);

	if(tst_state[2]==tst1) sub_bgnd("ВКЛ.",'#',-3);
	if(tst_state[2]==tst2) sub_bgnd("ВЫКЛ.",'#',-4);
	else sub_bgnd("РАБОЧ.",'#',-5);

	if(tst_state[3]==tst1) sub_bgnd("ВКЛ.",'%',-3);
	if(tst_state[3]==tst2) sub_bgnd("ВЫКЛ.",'%',-4);
	else sub_bgnd("РАБОЧ.",'%',-5);

	if(tst_state[4]==tst1) sub_bgnd("ВКЛ.",'^',-3);
	if(tst_state[4]==tst2) sub_bgnd("ВЫКЛ.",'^',-4);
	else sub_bgnd("РАБОЧ.",'^',-5);

	if(tst_state[7]==tst1) sub_bgnd("ВЫКЛ.",'(',-4);
	else sub_bgnd("РАБОЧ.",'(',-5);

	if(tst_state[8]==tst1) sub_bgnd("ВЫКЛ.",')',-4);
	else sub_bgnd("РАБОЧ.",')',-5);
	
	if(sub_ind==0)
		{
		if(tst_state[0]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,1,5);
		else if(tst_state[0]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,0,5);
		}
	else if(sub_ind==2)
		{
		if(tst_state[1]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT,1,5);
		else if(tst_state[1]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT,0,5);
		}
	else if(sub_ind==4)
		{
		if(tst_state[2]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,1,5);
		else if(tst_state[2]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,0,5);
		}
	else if(sub_ind==6)
		{
		if(tst_state[3]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,1,5);
		else if(tst_state[3]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,0,5);
		}
	else if(sub_ind==7)
		{
		if(tst_state[4]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,5);
		else if(tst_state[4]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,0,5);
		}
	else if(sub_ind==9)
		{
		if(tst_state[7]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<0),10);

		}
	else if(sub_ind==10)
		{
		if(tst_state[8]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<1),10);

		}
	}	 


else if(ind==iTst_bps)
	{
	if(tst_state[5]==tstOFF)ptrs[0]=		" Выключен           ";
	else if(tst_state[5]==tst1)ptrs[0]=		" Включен            ";
	else ptrs[0]=							" Автономно          ";
    ptrs[1]=								" ШИМ              @ ";
    ptrs[2]=								" U =  .$В  I =  .#A ";
	ptrs[3]=								" Выход              ";


	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     ТЕСТ БПС N!    ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);

	if(tst_state[6]==tst1) sub_bgnd("Uтемпер(  .@В)",'@',-13);
	else if(tst_state[6]==tst2) sub_bgnd("Umax",'@',-3);
	else sub_bgnd("Umin",'@',-3);




	/*int2lcdyx(sub_ind,0,19,0);
	int2lcdyx(index_set,0,17,0);
	int2lcdyx(tst_state[5],0,3,0);
	int2lcdyx(tst_state[6],0,5,0); */
	//int2lcdyx(sub_ind1,0,4,0); 

	int2lcd(sub_ind1+1,'!',0);
	if(tst_state[5]==tst2)
		{
		#ifdef UKU_220
		int2lcd(load_U/10,'$',0);
		#else 
		int2lcd(load_U,'$',1);
		#endif
		}
	else
		{
		#ifdef UKU_220
		int2lcd(bps[sub_ind1]._Uii/10,'$',0);
		#else 
		int2lcd(bps[sub_ind1]._Uii,'$',1);
		#endif
		}
	
	int2lcd(bps[sub_ind1]._Ii,'#',1);
	int2lcd(u_necc,'@',1);


	if(tst_state[5]==tstOFF) mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,~(1<<sub_ind1),10);
	else mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
	
	if(tst_state[5]==tst2) mess_send(MESS2NET_DRV,PARAM_BPS_NET_OFF,1,10);
		
	if(tst_state[6]==tstOFF) mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,30,10);
	else if(tst_state[6]==tst1) 
		{
		mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,30,10);
		mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_FAST_REG,0,10);
		}
	else if(tst_state[6]==tst2) mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1020,10);
/*		mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
	    	mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
     if(sub_ind==6)
		{
          mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
          mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,40);
          mess_send(MESS2UNECC_HNDL,PARAM_UNECC_SET,U_AVT,10);
	    	mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_FAST_REG,0,10);

          }

     if(sub_ind==9)
		{
		if(phase==0)
			{
          	
          	}
      	else if(phase==1)
			{
          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
			mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
          	}
          mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
	*/
	}
else if(ind==iInv_tabl)
     {
     if(sub_ind==0)
     	{
     	bgnd_par(" N   U      I     P " ,
     	         " !    ^       @    #",
     	         " !    ^       @    #",
     	         " !    ^       @    #");
      

     	}     

    	else if(sub_ind==1) 
     	{
     	bgnd_par(" N   I     P     t  ",
     	         " !    @     #     $ ",
     	         " !    @     #     $ ",
     	         " !    @     #     $ ");

		}

	int2lcd(sub_ind1+1,'!',0);
	int2lcd(sub_ind1+2,'!',0);
	int2lcd(sub_ind1+3,'!',0);
		
		
	int2lcd(inv[sub_ind1]._Uout/10,'^',0);
	int2lcd(inv[sub_ind1+1]._Uout/10,'^',0);
	int2lcd(inv[sub_ind1+2]._Uout/10,'^',0);

	int2lcd(inv[sub_ind1]._Iout,'@',1); 
	int2lcd(inv[sub_ind1+1]._Iout,'@',1); 
	int2lcd(inv[sub_ind1+2]._Iout,'@',1); 

	int2lcd_mmm(inv[sub_ind1]._Pout,'#',0);
	int2lcd_mmm(inv[sub_ind1+1]._Pout,'#',0);
	int2lcd_mmm(inv[sub_ind1+2]._Pout,'#',0);

	int2lcd_mmm(inv[sub_ind1]._T,'$',0);
	int2lcd_mmm(inv[sub_ind1+1]._T,'$',0); 
   	int2lcd_mmm(inv[sub_ind1+2]._T,'$',0);

	}	 
else if(ind==iKlimat)
	{
	ptrs[0]=				" tшк.max=       !°C ";
	ptrs[1]=				" tвент.max=     @°C ";
	ptrs[2]=				" tшк.рег.=      #°C ";
	ptrs[3]=				" tоткл.нагр.    $°C ";
	ptrs[4]=				" tвкл.нагр.     %°C ";
	ptrs[5]=				" tоткл.бат.     ^°C ";
	ptrs[6]=				" tвкл.бат.      &°C ";
	ptrs[7]=				" Выход              ";



	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(				"   КЛИМАТКОНТРОЛЬ   ",
						ptrs[index_set],
						ptrs[index_set+1],
						ptrs[index_set+2]);
	pointer_set(1);
	
	int2lcd(TBOXMAX,'!',0); 
	if((TBOXVENTMAX>=50)&&(TBOXVENTMAX<=80))int2lcd(TBOXVENTMAX,'@',0); 
	else sub_bgnd("ВЫКЛ.",'@',-2);
	int2lcd(TBOXREG,'#',0);
	if((TLOADDISABLE>=50)&&(TLOADDISABLE<=80))int2lcd(TLOADDISABLE,'$',0);
	else sub_bgnd("ВЫКЛ.",'$',-2);
	if((TLOADDISABLE>=50)&&(TLOADDISABLE<=80)/*&&(TLOADENABLE>=50)&&(TLOADENABLE<=80)*/)int2lcd(TLOADENABLE,'%',0);
	else sub_bgnd("ВЫКЛ.",'%',-2);
	if((TBATDISABLE>=50)&&(TBATDISABLE<=90))int2lcd(TBATDISABLE,'^',0);
	else sub_bgnd("ВЫКЛ.",'^',-2);
	if((TBATDISABLE>=50)&&(TBATDISABLE<=90)/*&&(TBATENABLE>=50)&&(TBATENABLE<=80)*/)int2lcd(TBATENABLE,'&',0);
	else sub_bgnd("ВЫКЛ.",'&',-2);
	}

else if(ind==iNpn_set)
	{
	ptrs[0]=				" Вывод          !   ";
	if(NPN_OUT==npnoOFF)
		{
		ptrs[1]=			" Выход              ";
		ptrs[2]=			"                    ";
		simax=1;
		}
	else 
		{
/*		ptrs[1]=			" Сигнал         @   ";
 		if(NPN_SIGN==npnsAVNET)
			{
			ptrs[2]=		" Tз.н.п.н.       #с.";
			ptrs[3]=		" Выход              ";
			simax=3;
			}
		else if(NPN_SIGN==npnsULOAD)
			{
			ptrs[2]=		" Uоткл.н.п.н.    $В ";
			ptrs[3]=		" Uвкл.н.п.н.     %В ";
			ptrs[4]=		" Tз.н.п.н.       #с.";
			ptrs[5]=		" Выход              ";
			simax=5;
			}*/

			ptrs[1]=		" Uоткл.н.п.н.    $В ";
			ptrs[2]=		" Uвкл.н.п.н.     %В ";
			ptrs[3]=		" Tз.н.п.н.       #с.";
			ptrs[4]=		" Выход              ";
			simax=4;
		}


	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(				" Отключение Н.П.Н.  ",
							ptrs[index_set],
							ptrs[index_set+1],
							ptrs[index_set+2]);
	pointer_set(1);
	
	if(NPN_OUT==npnoRELEVENT) sub_bgnd("Реле вент-ра",'!',-8);
	else if(NPN_OUT==npnoRELEAVBAT2) sub_bgnd("Реле АВ.БАТ2",'!',-8);
	else sub_bgnd("Выкл.",'!',-1);
//	if(NPN_SIGN==npnsAVNET) sub_bgnd("Ав.сети",'@',-3);
//	else sub_bgnd("Uнагр.",'@',-2);
	int2lcd(TZNPN,'#',0);
	int2lcd(UONPN,'$',1);
	int2lcd(UVNPN,'%',1);

	}
else if(ind==iFWabout)
	{
	bgnd_par(	" Версия             ",
				" Сборка  0000.00.00 ",
				"                    ",
				"                    ");
	int2lcdyx(BUILD_YEAR,1,12,0);
	int2lcdyx(BUILD_MONTH,1,15,0);
	int2lcdyx(BUILD_DAY,1,18,0);
	
	sprintf(&lcd_buffer[9],"%d.%d.%d",HARDVARE_VERSION,SOFT_VERSION,BUILD);
	}


#endif
/*
const char sm7[]	={" Источник N2        "}; //
const char sm8[]	={" Нагрузка           "}; //
const char sm9[]	={" Сеть               "}; //
const char sm10[]	={" Спецфункции        "}; // 
const char sm11[]	={" Журнал аварий      "}; //
const char sm12[]	=" Батарейный журнал  "}; //
const cha		=" Паспорт            "}; //
*/


//char2lcdhyx(bat_rel_stat[0],0,10);
//char2lcdhyx(bat_rel_stat[1],0,15);
//int2lcdyx(u_necc,0,19,0);
//int2lcdyx(cntrl_stat,0,5,0); 	   mess_cnt[i]

//char2lcdhyx(bat_rel_stat[0],0,5);
//char2lcdhyx(bat_rel_stat[1],0,10);
//int2lcdyx(mess_cnt[1],0,2,0);
//int2lcdyx(GET_REG(IOPIN1,21,1),0,5,0); 
//int2lcdyx(samokalibr_cnt,0,10,0);
//char2lcdhyx(rele_stat,0,19);
//char2lcdhyx(mess_cnt[1],0,16); 

//int2lcdyx(ad7705_res1,0,8,0);
//int2lcdyx(ad7705_res2,0,16,0); 
//	int2lcdyx(bat[0]._cnt_to_block,0,1,0);
//	int2lcdyx(bat[1]._cnt_to_block,0,3,0);
//	int2lcdyx(bat[0]._rel_stat,0,5,0);
/*	int2lcdyx(ind,0,3,0); 
	int2lcdyx(sub_ind,0,6,0);
	int2lcdyx(index_set,0,9,0);
	int2lcdyx(ptr_ind,0,14,0);
	;*/
/*int2lcdyx(ind,0,19,0);
int2lcdyx(retindsec,0,15,0);
int2lcdyx(retcnt,0,11,0);
int2lcdyx(retcntsec,0,7,0);	*/
//int2lcdyx(bps[0]._vol_i,0,15,0);
//int2lcdyx(AUSW_MAIN,0,19,0); 
}							    


#define BUT0	16
#define BUT1	17
#define BUT2	18
#define BUT3	19
#define BUT4	20   
#define BUT_MASK (1UL<<BUT0)|(1UL<<BUT1)|(1UL<<BUT2)|(1UL<<BUT3)|(1UL<<BUT4)

#define BUT_ON 4
#define BUT_ONL 20 

#define butLUR_  101
#define butU   253
#define butU_  125
#define butD   251
#define butD_  123
#define butL   247
#define butL_  119
#define butR   239
#define butR_  111
#define butE   254
#define butE_  126
#define butEL_  118
#define butUD  249
#define butUD_  121
#define butLR   231
#define butLR_   103
#define butED_  122
#define butDR_  107
#define butUR   237

#define BUT_ON 4
#define BUT_ONL 20 
//-----------------------------------------------
void but_drv(void)
{
char i;
LPC_GPIO1->FIODIR|=(1<<21);
LPC_GPIO1->FIOPIN&=~(1<<21);
LPC_GPIO1->FIODIR&=~((1<<22)|(1<<23)|(1<<24)|(1<<25)|(1<<26));
LPC_PINCON->PINMODE3&=~((1<<12)|(1<<13)|(1<<14)|(1<<15)|(1<<16)|(1<<17)|(1<<18)|(1<<19)|(1<<20)|(1<<21));

LPC_GPIO2->FIODIR|=(1<<8);
LPC_GPIO2->FIOPIN&=~(1<<8);
for(i=0;i<200;i++)
{
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
}

			LPC_GPIO2->FIODIR|=(1<<8);
			LPC_GPIO2->FIOPIN|=(1<<8);

but_n=((LPC_GPIO1->FIOPIN|(~((1<<22)|(1<<23)|(1<<24)|(1<<25)|(1<<26))))>>22)/*&0x0000001f*/;



if((but_n==1023UL)||(but_n!=but_s))
 	{
	speed=0;
 
   	if (((but0_cnt>=BUT_ON)||(but1_cnt!=0))&&(!l_but))
  		{
   	     n_but=1;
          but=but_s;

          }
   	if (but1_cnt>=but_onL_temp)
  		{
   	     n_but=1;
 
          but=but_s&0x7f;
          }
    	l_but=0;
   	but_onL_temp=BUT_ONL;
    	but0_cnt=0;
  	but1_cnt=0;          
     goto but_drv_out;
  	}
else if(but_n==but_s)
 	{
  	but0_cnt++;
  	if(but0_cnt>=BUT_ON)
  		{
   		but0_cnt=0;
   		but1_cnt++;
   		if(but1_cnt>=but_onL_temp)
   			{              
    			but=but_s&0x7f;
    			but1_cnt=0;
    			n_but=1;
    			     
    			l_but=1;
			if(speed)
				{
    				but_onL_temp=but_onL_temp>>1;
        			if(but_onL_temp<=2) but_onL_temp=2;
				}    
   			}
  		}
 	}
but_drv_out: 
but_s=but_n; 
   
}

//-----------------------------------------------
void but_an(void)
{
signed short temp_SS;
signed short deep,i,cap,ptr;
char av_head[4];
if(!n_but)goto but_an_end;
/*else  
	{
	plazma_but_an++;
	goto but_an_end;
	}*/
av_beep=0x0000;
av_rele=0x0000;
mnemo_cnt=MNEMO_TIME;
ips_bat_av_stat=0;
//bat_ips._av&=~1;

if((main_1Hz_cnt<10)&&((but==butU)||(but==butU_)||(but==butD)||(but==butD_)||(but==butL)||(but==butL_)||(but==butR)||(but==butR_)||(but==butE)||(but==butE_)))
	{
	__ee_spc_stat=spcOFF;
	spc_stat=spcOFF;
	}
if(but==butUD)
     {
     if(ind!=iDeb)
          {
		c_ind=a_ind;
		tree_up(iDeb,0,0,0);
		
          }
     else 
          {
		tree_down(0,0);
          }
		
		     
     }
else if(but==butLR)
	{
	bSILENT=1;
	beep_init(0x00000000,'S');
	LPC_SC->RSID=0xFF;
	}
else if(but==butUD_)
     {
	//avar_bat_as_hndl(0,1);
	}

else if(but==butED_)
     {
	if(!bCAN_OFF)bCAN_OFF=1;
	else bCAN_OFF=0;
	speed=0;
	}
/*else if(but==butUR)
	{
	tree_up(iK,0,0,0);
	}*/
else if(ind==iDeb)
	{
	if(but==butR)
		{
		sub_ind++;
		index_set=0;
		gran_ring_char(&sub_ind,0,11);
		}
	else if(but==butL)
		{
		sub_ind--;
		index_set=0;
		gran_ring_char(&sub_ind,0,11);
		}
		
	else if(sub_ind==1)
		{
		if(but==butU)
	     	{
	     	sub_ind1--;
	     	gran_char(&sub_ind1,0,60);
	     	}
		if(but==butD)
	     	{
	     	sub_ind1++;
	     	gran_char(&sub_ind1,0,60);
	     	}
	     
		if(but==butE)
	     	{
	     	/*SET_REG(C2GSR,3,24,8);
			C2MOD=0;
			 bOUT_FREE2=1;*/

			 // CAN interface 1, use IRQVec7, at 125kbit
//can2_init(7,8,CANBitrate250k_60MHz);

// Receive message with ID 102h on CAN 1
//FullCAN_SetFilter(2,0x18e);
			 }
		if(but==butE)
	     	{
			//lc640_write_int(EE_BAT1_ZAR_CNT,10);
			ind_pointer=0;
			ind=(i_enum)0;
			sub_ind=0;
			sub_ind1=0;
			sub_ind2=0;
			index_set=0;
			}
	     
			
		}
		
	 else if(sub_ind==5)
	 	{
		if(but==butE_)
		{
		//can1_init(BITRATE62_5K6_25MHZ);
		//FullCAN_SetFilter(0,0x18e);
		LPC_CAN1->MOD&=~(1<<0);
		}
		}
		
	 else if(sub_ind==9)
	 	{
		if(but==butE)
			{
			sntp_requ ();
			}
		}		
		
			
     else if(but==butU)
	     {
	     index_set--;
	     gran_char(&index_set,0,4);
	     //lc640_write_int(ptr_ki_src[0],lc640_read_int(ptr_ki_src[0])+10);
	     }	
     else if(but==butD)
	     {
	     index_set++;
	     gran_char(&index_set,0,4); 
	     //lc640_write_int(ptr_ki_src[0],lc640_read_int(ptr_ki_src[0])-10);
	     }	
     else if(but==butE)
         	{
          //a=b[--ptr_ind];
          mcp2515_transmit(1,2,3,4,5,6,7,8);
          }   
          
     else if(but==butE_)
         	{
          //a=b[--ptr_ind];
          //mcp2515_transmit_adr(TXBUFF,3);
          }                      				
	}


else if(ind==iMn_INV)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,9+NUMINV+NUMBYPASS);
		}
		
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,9+NUMINV+NUMBYPASS);
		}	
	else if(but==butD_)
		{
		//tree_up(iLog,0,0,0);
		sub_ind=6+NUMBYPASS+NUMINV;
		ret(1000);
		}
	else if(but==butE_)
		{
		//can1_init(BITRATE62_5K25MHZ);
		//FullCAN_SetFilter(0,0x18e);
     	//NUMINV=1;
     	//lc640_write_int(EE_NUMINV,NUMINV);
     	//NUMBYPASS=0;
     	//lc640_write_int(EE_NUMBYPASS,NUMBYPASS);
		}
	else if(but==butDR_)
		{
		//#ifdef GLADKOV
		tree_up(iK_INV,0,0,0);
		//#endif
		}
	else if(but==butU_)
		{
		//ind=iMn;
		sub_ind=0;
		}
	else if(but==butL)
		{
		//avar_inv_hndl(11,'C',1,173);
		}
	else if(but==butR)
		{
		//avar_inv_hndl(11,'C',0,165);
		}	
	else if(but==butL_)
		{
		//avar_inv_hndl(11,'T',1,173);
		}
	else if(but==butR_)
		{
	//	avar_inv_hndl(11,'T',0,165);
		}			
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			if(avar_ind_stat)
				{
				//ind=iAv_view;
				//sub_ind=0;
				tree_up(iAv_view,0,0,0);
				while(!(avar_ind_stat&(1<<sub_ind)))
					{
					sub_ind++;
					if(sub_ind>=32)
						{
						tree_down(0,0);
						avar_ind_stat=0;
						}
					}
				}																							
			}
		else if((sub_ind>2)&&(sub_ind<=(2+NUMBYPASS)))
		    	{
		    	if(NUMPHASE==1)tree_up(iByps,0,0,sub_ind-3);
				if(NUMPHASE==3)tree_up(iByps3f,0,0,sub_ind-3);
		    	}
		else if((sub_ind>(2+NUMBYPASS))&&(sub_ind<=(2+NUMBYPASS+NUMINV)))
		    	{
		    	tree_up(iInv_v3,0,0,sub_ind-NUMBYPASS-3);
		    	}

		else if(sub_ind==(3+NUMBYPASS+NUMINV))
			{
			tree_up(iInv_tabl,0,0,0);
		     ret(0);
		     
			} 			
		else if(sub_ind==(4+NUMBYPASS+NUMINV))
			{
			tree_up(iExtern,0,0,0);
		    ret(1000);
			}

		else if(sub_ind==(5+NUMBYPASS+NUMINV))
			{
			tree_up(iSet_prl,0,0,0);
		     ret(50);
		     parol_init();
			}
		else if(sub_ind==(6+NUMBYPASS+NUMINV))
			{
			tree_up(iLog,0,0,0);
		     ret(1000);
			}
		else if(sub_ind==(7+NUMBYPASS+NUMINV))
			{
			sub_ind=0;
			}
		else if(sub_ind==(8+NUMBYPASS+NUMINV))
			{
			tree_up(iFWabout,0,0,0);
		    ret(1000);
			}
		}
    }

else if(ind==iMn_220_V2)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,9+NUMBAT+NUMIST+NUMINV+(NUMEXT!=0));
		}
		
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,9+NUMBAT+NUMIST+NUMINV+(NUMEXT!=0));
		}	

	else if(but==butL)
		{
		//ind=iMn;
		sub_ind=0;
		}
		
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			if(avar_ind_stat)
				{
				//ind=iAv_view;
				//sub_ind=0;
				tree_up(iAv_view,0,0,0);
				while(!(avar_ind_stat&(1<<sub_ind)))
					{
					sub_ind++;
					if(sub_ind>=32)
						{
						tree_down(0,0);
						avar_ind_stat=0;
						}
					}
				}																							
			}
		else if((sub_ind>0)&&(sub_ind<=NUMBAT))
		    	{
		    	if(BAT_IS_ON[0]!=bisON)tree_up(iBat_simple,0,0,1);
		    	else tree_up(iBat_simple,0,0,sub_ind-1);
		    	}
		else if((sub_ind>NUMBAT)&&(sub_ind<=(NUMBAT+NUMIST)))
		    	{
		    	tree_up(iBps,0,0,sub_ind-(1+NUMBAT));
		    	}
		else if((sub_ind>(NUMBAT+NUMIST))&&(sub_ind<=(NUMBAT+NUMIST+NUMINV)))
		    	{
		    	tree_up(iBps,0,0,sub_ind-(1+NUMBAT+NUMIST));
		    	}
		else if(sub_ind==(1+NUMBAT+NUMIST+NUMINV))
			{
			if((AUSW_MAIN==22035)||(AUSW_MAIN==22033)||(AUSW_MAIN==22063)||(AUSW_MAIN==22023)||(AUSW_MAIN==22043))
				{
				tree_up(iNet3,0,0,0);
		     	ret(1000);
				}
			else
				{
				tree_up(iNet,0,0,0);	
				ret(1000);		
				}
			}
		else if(sub_ind==(2+NUMBAT+NUMIST+NUMINV))
			{
			tree_up(iLoad,0,0,0);
		     ret(1000);
			}
		else if((sub_ind==(3+NUMBAT+NUMIST+NUMINV))&&(NUMEXT))
			{
			tree_up(iExtern_220,0,0,0);
		     ret(1000);
			}

		else if(sub_ind==(3+NUMBAT+NUMIST+NUMINV+(NUMEXT!=0)))
			{
			tree_up(iSpc,0,0,0);
		     ret(1000);
			}

		else if(sub_ind==(4+NUMBAT+NUMIST+NUMINV+(NUMEXT!=0)))
			{
			tree_up(iSet_prl,0,0,0);
		     ret(50);
		     parol_init();
			}
		else if(sub_ind==(5+NUMBAT+NUMIST+NUMINV+(NUMEXT!=0)))
			{
			tree_up(iLog,0,0,0);
		     ret(1000);
			}
		else if(sub_ind==(6+NUMBAT+NUMIST+NUMINV+(NUMEXT!=0)))
			{
			sub_ind=0;
			}
		else if(sub_ind==(7+NUMBAT+NUMIST+NUMINV+(NUMEXT!=0)))
			{
			tree_up(iBatLog,0,0,0);
		     ret(1000);
			}
		else if(sub_ind==(8+NUMBAT+NUMIST+NUMINV+(NUMEXT!=0)))
			{
			tree_up(iBatLog,0,0,1);
		     ret(1000);
			}
		else if(sub_ind==(9+NUMBAT+NUMIST+NUMINV+(NUMEXT!=0)))
			{
			if(but==butE)
		     	{
		     	tree_up(iPrltst,0,0,0);
		     	parol_init();
		     	}
			}
		}
    	}



#ifndef _DEBUG_
else if(ind==iBps)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		if(sub_ind==3)sub_ind=1;
		else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		if(sub_ind<3)sub_ind=3;
		gran_char(&sub_ind,0,simax);
		}
		
	else if((but==butE)&&(sub_ind==4))
		{
		mcp2515_transmit(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		}
				
	else if(((but==butE)&&(sub_ind==5))||(but))
		{
	     tree_down(0,0);
	     ret(0);
		}	
		
	}		
else if(ind==iNet)
	{
	ret(1000);
	if((but==butL)||(but==butE))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	}

else if(ind==iNet3)
	{
	ret(1000);
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,4);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);
		}
	else if((but==butL)||((but==butE)&&(sub_ind==4)))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	}
else if(ind==iInv)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		if(sub_ind==3)sub_ind=1;
		else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		if(sub_ind<3)sub_ind=3;
		gran_char(&sub_ind,0,simax);
		}
		
	else if((but==butE)&&(sub_ind==4))
		{
		mcp2515_transmit(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		}
				
	else if(((but==butE)&&(sub_ind==5))||(but))
		{
	     tree_down(0,0);
	     ret(0);
		}		
	}
#endif
else if(ind==iInv_v2)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		if(sub_ind==3)sub_ind=1;
		else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		if(sub_ind<3)sub_ind=3;
		gran_char(&sub_ind,0,simax);
		}
		
/*	else if((but==butE)&&(sub_ind==4))
		{
		mcp2515_transmit(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		}
		*/		
	else if(((but==butE)&&(sub_ind==simax))||(but==butL))
		{
	    tree_down(0,0);
	    ret(0);
		}		
	}
else if(ind==iInv_v3)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		if(sub_ind==3)sub_ind=1;
		else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		if(sub_ind<3)sub_ind=3;
		gran_char(&sub_ind,0,simax);
		}
		
/*	else if((but==butE)&&(sub_ind==4))
		{
		mcp2515_transmit(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		}
		*/		
	else if(((but==butE)&&(sub_ind==simax))||(but==butL))
		{
	    tree_down(0,0);
	    ret(0);
		}		
	}
else if(ind==iInv_tabl)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind1--;
		gran_char(&sub_ind1,0,NUMINV-3);
		}
		
	else if (but==butD)
		{
		sub_ind1++;
		gran_char(&sub_ind1,0,NUMINV-3);
		}
		
	else if(but==butR)
		{
		sub_ind=1;
		}
				
	else if(but==butL)
		{
		sub_ind=0;
		}
	else if(but==butE)
		{
		tree_down(0,0);
		}				
	}



else if(ind==iByps)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		if(sub_ind==3)sub_ind=1;
		else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		if(sub_ind<3)sub_ind=3;
		gran_char(&sub_ind,0,simax);
		}
		
/*	else if((but==butE)&&(sub_ind==4))
		{
		mcp2515_transmit(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		}
		*/		
	else if(((but==butE)&&(sub_ind==simax))||(but==butL))
		{
	    tree_down(0,0);
	    ret(0);
		}		
	}

else if(ind==iByps3f)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		if(sub_ind==3)sub_ind=1;
		else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		if(sub_ind<3)sub_ind=3;
		gran_char(&sub_ind,0,simax);
		}
		
/*	else if((but==butE)&&(sub_ind==4))
		{
		mcp2515_transmit(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		}
		*/		
	else if(((but==butE)&&(sub_ind==simax))||(but==butL))
		{
	    tree_down(0,0);
	    ret(0);
		}		
	}

#ifndef _DEBUG_
else if(ind==iLoad)
	{
	ret(1000);
	if((but==butL)||(but==butE))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	}	

else if(ind==iExtern)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,NUMDT+NUMSK);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMSK);		
		}

	else if((but==butE)&&(sub_ind==NUMSK))
		{
	     tree_down(0,0);
	     ret(0);
		}
	}
	
else if(ind==iExtern_3U)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,2+NUMSK);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2+NUMSK);		
		}

	else if((but==butE)&&(sub_ind==2+NUMSK))
		{
	     tree_down(0,0);
	     ret(0);
		}	
	}	

else if(ind==iExtern_6U)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,NUMDT+NUMSK);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMDT+NUMSK);		
		}

	else if((but==butE)&&(sub_ind==NUMDT+NUMSK))
		{
	     tree_down(0,0);
	     ret(0);
		}	
	}	

else if(ind==iExtern_220)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,NUMDT+NUMSK);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMDT+NUMSK);		
		}

	else if((but==butE)&&(sub_ind==NUMDT+NUMSK))
		{
	     tree_down(0,0);
	     ret(0);
		}	
	}	



else if(ind==iVent)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,1,2);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,1,2);		
		}

	else if (sub_ind==1)
		{
          if((but==butR)||(but==butR_))
               {
               pos_vent++;
               }
          else if((but==butL)||(but==butL_))
               {
               pos_vent--;
               }

		gran(&pos_vent,1,11);
          lc640_write_int(EE_POS_VENT,pos_vent);		
		}
		
	else if((but==butE)&&(sub_ind==2))
		{
	     tree_down(0,0);
	     ret(0);
		}	
	}
else if(ind==iAvt)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,NUMAVT);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMAVT);		
		}

	else if((but==butE)&&(sub_ind==NUMAVT))
		{
	     tree_down(0,0);
	     ret(0);
		}
	}
else if(ind==iEnerg)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,4);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);		
		}

	else if((but==butE)&&(sub_ind==4))
		{
	     tree_down(0,0);
	     ret(0);
		}
     }

else if(ind==iEnerg3)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,8);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,8);		
		}

	else if((but==butE)&&(sub_ind==8))
		{
	     tree_down(0,0);
	     ret(0);
		}
     }

else if((ind==iPrl_bat_in_out)||(ind==iSet_prl)||(ind==iK_prl)
	||(ind==iSpc_prl_vz)||(ind==iSpc_prl_ke)||(ind==iAusw_prl)
	||(ind==iPrltst)||(ind==iLog_reset_prl))
	{
	ret(50);
	if(but==butR)
		{
		sub_ind++;
		gran_ring_char(&sub_ind,0,2);
		}
	else if(but==butL)
		{
		sub_ind--;
		gran_ring_char(&sub_ind,0,2);
		}	
	else if(but==butU)
		{
		parol[sub_ind]++;
		gran_ring_char(&parol[sub_ind],0,9);
		}	
	else if(but==butD)
		{
		parol[sub_ind]--;
		gran_ring_char(&parol[sub_ind],0,9);
		}	
	else if(but==butE)
		{
		unsigned short tempU;
		tempU=parol[2]+(parol[1]*10U)+(parol[0]*100U);
		
		if(ind==iPrl_bat_in_out)
		     {
		     if(BAT_IS_ON[sub_ind1]!=bisON)
		          {
		          if(tempU==PAROL_BAT_IN)
		               {
		               lc640_write_int(ADR_EE_BAT_IS_ON[sub_ind1],bisON);
		               lc640_write_int(ADR_EE_BAT_DAY_OF_ON[sub_ind1],LPC_RTC->DOM);
		               lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[sub_ind1],LPC_RTC->MONTH);
		               lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[sub_ind1],LPC_RTC->YEAR);
		               lc640_write_int(ADR_EE_BAT_C_REAL[sub_ind1],0x5555);
		               lc640_write_int(ADR_EE_BAT_RESURS[sub_ind1],0);
					lc640_write_int(ADR_EE_BAT_ZAR_CNT[sub_ind1],0);
		               
		               lc640_write(KE_PTR,0);
					lc640_write(VZ_PTR,0);
					lc640_write(WRK_PTR,0);
					lc640_write(KE_CNT,0);
					lc640_write(VZ_CNT,0);
					lc640_write(WRK_CNT,0);
					lc640_write(BAT_AVAR_CNT,0);
					lc640_write(BAT_AVAR_PTR,0);					
		               
                         tree_down(0,0);
                         ret(0);
		               }
		          else
		               {
		               tree_down(0,0);
	    	               show_mess("                    ",
	          				"       Пароль       ",
	          				"     неверный!!!    ",
	          				"                    ",1000);
     	               }
		          }      
               else		          
		          {
		          if(tempU==PAROL_BAT_OUT)
		               {
		               lc640_write_int(ADR_EE_BAT_IS_ON[sub_ind1],bisOFF);
		               lc640_write_int(ADR_EE_BAT_DAY_OF_ON[sub_ind1],LPC_RTC->DOM);
		               lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[sub_ind1],LPC_RTC->MONTH);
		               lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[sub_ind1],LPC_RTC->YEAR);

		               tree_down(0,0);
		               ret(0);
		               
		               }
	               else
		               {
		               tree_down(0,0);
	    	               show_mess("                    ",
	          				"       Пароль       ",
	          				"     неверный!!!    ",
	          				"                    ",1000);
		               }		               
		          }     
               }
		
		else if(ind==iSet_prl)
			{
	     	if(tempU==PAROL_SET) 
				{
				tree_down(0,0);
				#ifdef UKU_INV
				tree_up(iSet_INV,0,0,0);
				#endif

				ret(1000);
				}
			else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       Пароль       ",
	          			"     неверный!!!    ",
	          			"                    ",1000);
				}
			}
		else	if(ind==iK_prl)
			{
	     	if(tempU==PAROL_KALIBR) 
				{
				tree_down(0,0);
				#ifdef UKU_INV
				tree_up(iK_INV,0,0,0);
				#endif
				
				}
			else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       Пароль       ",
	          			"     неверный!!!    ",
	          			"                    ",1000);
				}
			} 
	
		else	if(ind==iAusw_prl)
			{
	     	if(tempU==PAROL_AUSW) 
				{
				tree_down(0,0);
				tree_up(iAusw_set,1,0,0);
				default_temp=10;
				ret(0);
				}
			else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       Пароль       ",
	          			"     неверный!!!    ",
	          			"                    ",1000);
				}
			} 	
			
		else	if(ind==iSet_st_prl)
			{
	     	if(tempU==PAROL_DEFAULT) 
				{
	//			ind=iDefault;
				sub_ind=1;
				index_set=0;
				default_temp=10;
				}
			else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       Пароль       ",
	          			"     неверный!!!    ",
	          			"                    ",1000);
				}
			} 
						
		else if(ind==iPrltst)
			{
			if(tempU==PAROL_TST) 
				{
				tree_down(0,0);
				#ifdef UKU_GLONASS
				tree_up(iTst_GLONASS,0,0,0);
				#endif
				#ifdef UKU_3U
				tree_up(iTst_3U,0,0,0);
				#endif
				#ifdef UKU_RSTKM
				tree_up(iTst_RSTKM,0,0,0);
				#endif
				#ifdef UKU_KONTUR
				tree_up(iTst_KONTUR,0,0,0);
				#endif
				#ifdef UKU
				tree_up(iTst,0,0,0);
				#endif
				#ifdef UKU_6U
				tree_up(iTst_6U,0,0,0);
				#endif
				#ifdef UKU_220
				if(AUSW_MAIN==22035)
					{
					tree_up(iTst_220_380,0,0,0);
					}
				else
				tree_up(iTst_220,0,0,0);
				#endif
				#ifdef UKU_220_V2
				tree_up(iTst_220,0,0,0);
				#endif
				#ifdef UKU_220_IPS_TERMOKOMPENSAT
				tree_up(iTst_220_IPS_TERMOKOMPENSAT,0,0,0);
				#endif
				
				tst_state[0]=tstOFF;
				tst_state[1]=tstOFF;
				tst_state[2]=tstOFF;
				tst_state[3]=tstOFF;
				tst_state[4]=tstOFF;
				tst_state[5]=tstOFF;
				tst_state[6]=tstOFF;
				tst_state[7]=tstOFF;
				tst_state[9]=tstOFF;
				tst_state[10]=(enum_tst_state)0;
				ret(10000);				


				}
	  		else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       Пароль       ",
	          			"     неверный!!!    ",
	          			"                    ",1000);
				}
			}
		else if(ind==iSpc_prl_ke)
			{
			if(tempU==PAROL_KE) 
				{
				char temp;
				temp=sub_ind1;
				tree_down(0,0);
				tree_up(iKe,0,0,temp);
				ret(1000);
				}
	  		else 
				{	
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       Пароль       ",
	          			"     неверный!!!    ",
	          			"                    ",1000);
				}
			}
		else if(ind==iSpc_prl_vz)
			{
			if(tempU==PAROL_VZ) 
				{
				tree_down(0,0);
				tree_up(iVz,0,0,0);
				ret(1000);
				}
	  		else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       Пароль       ",
	          			"     неверный!!!    ",
	          			"                    ",1000);
				}     	          
			}
		else if(ind==iLog_reset_prl)
			{
			if(tempU==PAROL_LOG_RESET) 
				{
				tree_down(0,0);
				lc640_write(CNT_EVENT_LOG,0);
				lc640_write(PTR_EVENT_LOG,0);
				tree_down(0,0);
				avar_ind_stat=0;
				avar_stat=0;
				avar_stat_old=0;
	    	    show_mess("                    ",
	          			"   Журнал событий   ",
	          			"      очищен!!!     ",
	          			"                    ",1000);
				//ret(1000);
				}
	  		else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       Пароль       ",
	          			"     неверный!!!    ",
	          			"                    ",1000);
				}  
			}
		}
	}

else if(ind==iSpc)
	{
	//ret_ind(0,0,0);
	//ret_ind_sec(iMn,60);
	ret(1000);
	if (but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,4);
		}
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);
		}
	else if(but==butE)
		{
		if(sub_ind==0)
			{   
               tree_up(iSpc_prl_vz,0,0,0);
			parol_init();
			}
		#ifndef UKU_220_IPS_TERMOKOMPENSAT
		else if(sub_ind==1)
			{
               tree_up(iAvz,0,0,0);
               parol_init();
			}			
		else if((sub_ind==2)||(sub_ind==3))
			{
               tree_up(iSpc_prl_ke,0,0,sub_ind-2);
              	parol_init();
			} 
	/*	else if(sub_ind==4)
			{
			tree_up(iAKE,0,0,0);
			}	
		else if(sub_ind==5)
			{
			tree_up(iAKE,0,0,1);
			}*/	
		#endif						
		else if(sub_ind==4)
			{
			tree_down(0,0);
			ret(0);
			}	
		}
	else if(but==butL)
		{
		tree_down(0,0);
		ret(0);
		}			
	}



else if(ind==iKe)
	{
	ret_ind(0,0,0);
	//ret_ind_sec(0,0);
	if (but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,1);
		}
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,1);
		}
	else if(sub_ind==0)
		{
		if(but==butE)
			{
			if((spc_stat==spcKE)&&(spc_bat==sub_ind1))
				{
				spc_stat=spcOFF;
				__ee_spc_stat=spcOFF;
				lc640_write_int(EE_SPC_STAT,spcOFF);
				}
			else
				{

				ke_start(sub_ind1);
				if(ke_start_stat==kssNOT)
					{
					show_mess("Включение контроля  ",
	          				"  емкости батареи   ",
	          				"     невозможно.    ",
	          				" Причина неизвестна ",
							3000);
					}
				else if(ke_start_stat==kssNOT_VZ)
					{
					show_mess("Включение контроля  ",
	          				"  емкости батареи   ",
	          				" невозможно. Идет   ",
	          				"выравнивающий заряд ",
							3000);
					}

				else if(ke_start_stat==kssNOT_BAT)
					{
					show_mess("Включение контроля  ",
	          				"  емкости батареи   ",
	          				" невозможно.Батарея ",
	          				"     не введена     ",
							3000);
					}

				else if(ke_start_stat==kssNOT_BAT_AV)
					{
					show_mess("Включение контроля  ",
	          				"  емкости батареи   ",
	          				" невозможно.Батарея ",
	          				"   не подключена    ",
							3000);
					}

				else if(ke_start_stat==kssNOT_BAT_AV_T)
					{
					show_mess("Включение контроля  ",
	          				"  емкости батареи   ",
	          				" невозможно.Батарея ",
	          				"     перегрета      ",
							3000);
					}

				else if(ke_start_stat==kssNOT_BAT_AV_ASS)
					{
					show_mess("Включение контроля  ",
	          				"  емкости батареи   ",
	          				"     невозможно.    ",
	          				"     Асимметрия.    ",
							3000);
					}


				else if(ke_start_stat==kssNOT_BAT_ZAR)
					{
					show_mess("Включение контроля  ",
	          				"  емкости батареи   ",
	          				"     невозможно.    ",
	          				" Батарея заряжается ",
							3000);
					}

				else if(ke_start_stat==kssNOT_BAT_RAZR)
					{
					show_mess("Включение контроля  ",
	          				"  емкости батареи   ",
	          				"     невозможно.    ",
	          				"Батарея разряжается ",
							3000);
					}

 				else if(ke_start_stat==kssNOT_KE2)
					{
					show_mess("Включение контроля  ",
	          				" емкости батареи №1 ",
	          				"     невозможно.    ",
	          				"     Идет КЕБ №2    ",
							3000);
					}

 				else if(ke_start_stat==kssNOT_KE1)
					{
					show_mess("Включение контроля  ",
	          				" емкости батареи №1 ",
	          				"     невозможно.    ",
	          				"     Идет КЕБ №2    ",
							3000);
					}

				else if(ke_start_stat==kssYES)
					{
					if(sub_ind==0)
					show_mess("  КОНТРОЛЬ ЕМКОСТИ  ",
	          				"     БАТАРЕИ N1     ",
	          				"     ВКЛЮЧЕН!!!     ",
	          				"                    ",
							3000);
					else 
					show_mess("  КОНТРОЛЬ ЕМКОСТИ  ",
	          				"     БАТАРЕИ N2     ",
	          				"     ВКЛЮЧЕН!!!     ",
	          				"                    ",
							3000);

					}

 										  								   										
				}
			}
	/*	else if(but==butR)
			{
			if((spc_stat==spcKE)&&(spc_phase==0))
				{
				lc640_write_int(ADR_EE_BAT_C_REAL[spc_bat],lc640_read_int(ADR_EE_BAT_ZAR_CNT_KE[spc_bat]));
				ke_mem_hndl(spc_bat,lc640_read_int(ADR_EE_BAT_ZAR_CNT_KE[spc_bat]));
				lc640_write_int(ADR_EE_BAT_ZAR_CNT[spc_bat],0);
				cntrl_stat=50;
				cntrl_stat_old=50;
				}

			if((BAT_IS_ON[1-spc_bat]) == bisON)
				{
				spc_phase=1;
				__ee_spc_phase=1;
				lc640_write_int(EE_SPC_PHASE,1);
				}			
			else
				{
				spc_stat=spcOFF;
				__ee_spc_stat=spcOFF;
				lc640_write_int(EE_SPC_STAT,spcOFF);
				}
			}*/									
   		}
	
	else if(sub_ind==1)
		{                 
		if(but==butE)
			{
			tree_down(0,0);
			}
     	} 
 	else sub_ind=0;     
	}

#endif
else if(ind==iLog)
	{
	ret_ind_sec(0,0);
	ret_ind(0,0,0);
	if (but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,av_j_si_max+1);
		}
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,av_j_si_max+1);
          
		}  

	else if (but==butD_)
		{
		sub_ind=av_j_si_max;
		} 
		 
	else if (but==butL)
		{
		tree_down(0,0);
		ret(0);
		}  
		
	else if(but==butE)
		{  
		if((sub_ind==av_j_si_max+1)&&(av_j_si_max!=0))
			{
			tree_up(iLog_reset_prl,0,0,0);
			parol_init();
			/*lc640_write(CNT_EVENT_LOG,0);
			lc640_write(PTR_EVENT_LOG,0);
			tree_down(0,0);
			avar_ind_stat=0;
			avar_stat=0;
			avar_stat_old=0;*/				
			}
					
		else if((sub_ind==av_j_si_max)||(av_j_si_max==0))
			{
			tree_down(0,0);
			ret(0);
			}
			
		else 
			{
			/*ind=iLog_;
			sub_ind1=sub_ind;
			index_set=0;
			sub_ind=0;*/
			tree_up(iLog_,0,0,sub_ind);
			}	
			
		} 

	else if(but==butR)
		{
	    //	avar_bat_hndl(0,1);	
		}
	else if(but==butR_)
		{
	    	//avar_bat_hndl(0,0);	
		}		
	else if(but==butL)
		{
	    	//avar_s_hndl(1,0,1);	
		}
				
	else if(but==butL_)
		{           
		/*lc640_write(CNT_EVENT_LOG,0);
		lc640_write(PTR_EVENT_LOG,0);
		ind=iMn;
		sub_ind=cnt_of_slave+10;
		index_set=0;*/				
	
		}	 		
	}

else if(ind==iLog_)
	{          
	if(but==butU)
		{
		index_set--;
		gran_char(&index_set,0,av_j_si_max);
		}
	else if(but==butD)
		{
		index_set++;
		gran_char(&index_set,0,av_j_si_max);
		}
	else 
		{
		/*ind=iLog;
		sub_ind=sub_ind1;*/
		tree_down(0,0/*sub_ind1-sub_ind*/);
		}		
	}	

#ifndef _DEBUG_


else if(ind==iSet_INV)
	{
	ret(1000);
	kan_aktivity_cnt=15;
	if(but==butD)
		{
		sub_ind1=0;
		sub_ind++;
		if(sub_ind==2)
			{
			index_set=1;
			}
		if(sub_ind==3)
			{
			sub_ind=4;
			index_set=3;
			}
		if(sub_ind==6)
			{
			index_set=5;
			}
		if(sub_ind==7)
			{
			sub_ind=8;
			index_set=7;
			}
		if(sub_ind==9)
			{
			sub_ind=10;
			index_set=11;
			}
		if(sub_ind==11)
			{
			sub_ind=12;
			index_set=11;
			}
		if(sub_ind==13)
			{
			sub_ind=14;
			index_set=15;
			}
		if(sub_ind==15)
			{
			sub_ind=16;
			index_set=15;
			}
		if(sub_ind==17)
			{
			sub_ind=18;
			index_set=19;
			}
		if(sub_ind==19)
			{
			sub_ind=20;
			index_set=19;
			}
		if(sub_ind==21)
			{
			sub_ind=22;
			index_set=21;
			}
        if(sub_ind==24)
            {
            index_set=23;
            } 
        if(sub_ind==25)
            {
            sub_ind=26;
		 //index_set=18;
            }
        if(sub_ind==27)
            {
            //sub_ind=26;
		 	index_set=26;
            }
        if(sub_ind==28)
            {
            sub_ind=29;
		 	sub_ind1=0;
            }
		gran_char(&sub_ind,0,32);
		if((sub_ind-index_set)>2)index_set=sub_ind-2;
		}
	else if(but==butU)
		{
		sub_ind1=0;
		sub_ind--;
		if(sub_ind==3)sub_ind=2;
		if(sub_ind==5)sub_ind=4;
		if(sub_ind==11)
			{
			sub_ind=10;
			//index_set=6;
			}
		if(sub_ind==9)
			{
			sub_ind=8;
			//index_set=4;
			}			
		if(sub_ind==11)
			{
			sub_ind=10;
			//index_set=8;
			}
		if(sub_ind==13)
			{
			sub_ind=12;
			//index_set=10;
			}
		if(sub_ind==15)
			{
			sub_ind=14;
			//index_set=8;
			}
		if(sub_ind==17)
			{
			sub_ind=16;
			//index_set=10;
			}
		if(sub_ind==19)
			{
			sub_ind=18;
			//index_set=12;
			}
		if(sub_ind==21)
			{
			sub_ind=20;
			//index_set=14;
			}
		if(sub_ind==25)
			{
			sub_ind=24;
			//index_set=16;
			}
		if(sub_ind==28)
			{
			sub_ind=27;
			//index_set=16;
			}
		if(sub_ind==29)
			{
			sub_ind1=0;
			//index_set=16;
			}
		gran_char(&sub_ind,0,32);
		if(sub_ind<index_set)index_set=sub_ind;
		if((sub_ind==20)||(sub_ind==18))index_set=19;
		if((sub_ind==16)||(sub_ind==14))index_set=15;
		if((sub_ind==122)||(sub_ind==10))index_set=11;
		}
	else if(but==butD_)
		{
		sub_ind1=0;
		sub_ind=31;
		}
	else if(but==butU_)
		{
		sub_ind1=0;
		sub_ind=0;
		}		
	else if(sub_ind==0)
	     {
	     if(but==butE)
	          	{
				default_temp=100;
				tree_up(iDef,0,0,0);
				/*
	          	lc640_write_int(EE_U_OUT_SET,220);
			  	lc640_write_int(EE_U_OUT_MAX,255);
			  	lc640_write_int(EE_U_OUT_MIN,175);
			  	lc640_write_int(EE_U_NET_MAX,187);
				lc640_write_int(EE_U_NET_MIN,182);
				lc640_write_int(EE_U_BAT_MAX,180);
				lc640_write_int(EE_U_BAT_MIN,175);
				*/
	          	}
	     }	
	
     else if(sub_ind==1)
		{
		if(but==butE)
		     {
		     tree_up(iSet_T,0,0,0);
		     ret(1000);
		     phase=0;
		     }
		}	

     else if(sub_ind==2)
		{
		if(but==butE)
		     {
		     tree_up(iSet_T_avt,0,0,0);
		     ret(1000);
		     phase=0;
		     }
		}					 
     else if(sub_ind==4)
		{
		if(but==butE)
		     {
		     tree_up(iStr_INV,0,0,0);
		     ret(1000);
		     index_set=0;
		     }
		}	
	
				     		
	else if(sub_ind==5)
	     {
		if(ZV_ON)ZV_ON=0;
		else ZV_ON=1;
	     lc640_write_int(EE_ZV_ON,ZV_ON);
	     speed=1;
	     }	
	
	else if(sub_ind==6)
	     {
		if(AV_OFF_AVT)AV_OFF_AVT=0;
		else AV_OFF_AVT=1;
	     lc640_write_int(EE_AV_OFF_AVT,AV_OFF_AVT);
	     speed=1;
	     }	

     else if(sub_ind==8)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	U_OUT_SET++;
	     	gran(&U_OUT_SET,220,230);
	     	lc640_write_int(EE_U_OUT_SET,U_OUT_SET);
			U_OUT_MAX=U_OUT_SET+35;
	     	lc640_write_int(EE_U_OUT_MAX,U_OUT_MAX);
			speed=1;
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	U_OUT_SET--;
	     	gran(&U_OUT_SET,220,230);
	     	lc640_write_int(EE_U_OUT_SET,U_OUT_SET);
			U_OUT_MAX=U_OUT_SET+35;
	     	lc640_write_int(EE_U_OUT_MAX,U_OUT_MAX);
			speed=1;
	     	}
          }

     else if(sub_ind==10)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	U_OUT_MAX++;
	     	gran(&U_OUT_MAX,240,270);
	     	lc640_write_int(EE_U_OUT_MAX,U_OUT_MAX);
			speed=1;
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	U_OUT_MAX--;
	     	gran(&U_OUT_MAX,240,270);
			//gran(&U_OUT_MIN,170,U_OUT_MAX-10);
	     	lc640_write_int(EE_U_OUT_MAX,U_OUT_MAX);
			//lc640_write_int(EE_U_OUT_MIN,U_OUT_MIN);
			speed=1;
	     	}
          }

     else if(sub_ind==12)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	U_OUT_MIN++;
	     	gran(&U_OUT_MIN,0,200);
			//gran(&U_OUT_MAX,U_OUT_MIN+10,260);
			lc640_write_int(EE_U_OUT_MIN,U_OUT_MIN);
		//	lc640_write_int(EE_U_OUT_MAX,U_OUT_MAX);
			speed=1;
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	U_OUT_MIN--;
	     	gran(&U_OUT_MIN,0,200);
	     	lc640_write_int(EE_U_OUT_MIN,U_OUT_MIN);
			speed=1;
	     	}
          }

     else if(sub_ind==14)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	U_NET_MAX++;
	     	gran(&U_NET_MAX,180,205);
	     	lc640_write_int(EE_U_NET_MAX,U_NET_MAX);
			speed=1;
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	U_NET_MAX--;
	     	gran(&U_NET_MAX,180,205);
			gran(&U_NET_MIN,175,U_NET_MAX-5);
	     	lc640_write_int(EE_U_NET_MAX,U_NET_MAX);
			lc640_write_int(EE_U_NET_MIN,U_NET_MIN);
			speed=1;
	     	}
          }

     else if(sub_ind==16)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	U_NET_MIN++; 
			gran(&U_NET_MIN,175,200);
			gran(&U_NET_MAX,U_NET_MIN+5,205);
	     	lc640_write_int(EE_U_NET_MIN,U_NET_MIN);
			lc640_write_int(EE_U_NET_MAX,U_NET_MAX);
			speed=1;
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	U_NET_MIN--;
	     	gran(&U_NET_MIN,175,200);
	     	lc640_write_int(EE_U_NET_MIN,U_NET_MIN);
			speed=1;
	     	}
          }

     else if(sub_ind==18)
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
	     if((but==butR)||(but==butR_))
	     	{
	     	U_BAT_MAX++;
	     	gran(&U_BAT_MAX,temp_min,temp_max);
	     	lc640_write_int(EE_U_BAT_MAX,U_BAT_MAX);
			speed=1;
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	U_BAT_MAX--;
	     	gran(&U_BAT_MAX,temp_min,temp_max);
			gran(&U_BAT_MIN,temp_min-temp_d,U_BAT_MAX-temp_d);
	     	lc640_write_int(EE_U_BAT_MAX,U_BAT_MAX);
			lc640_write_int(EE_U_BAT_MIN,U_BAT_MIN);
			speed=1;
	     	}
          }

     else if(sub_ind==20)
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
	     if((but==butR)||(but==butR_))
	     	{
	     	U_BAT_MIN++;
	     	gran(&U_BAT_MIN,temp_min,temp_max);
			gran(&U_BAT_MAX,U_BAT_MIN+temp_d,temp_max+temp_d);
	     	lc640_write_int(EE_U_BAT_MIN,U_BAT_MIN);
			lc640_write_int(EE_U_BAT_MAX,U_BAT_MAX);
			speed=1;
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	U_BAT_MIN--;
	     	gran(&U_BAT_MIN,temp_min,temp_max);
	     	lc640_write_int(EE_U_BAT_MIN,U_BAT_MIN);
			speed=1;
	     	}
          }

	 else if(sub_ind==22)
		{
		if(but==butE)
		     {
		     tree_up(iLan_set,0,0,0);
		     ret(1000);
		     }
		}


     else if(sub_ind==23)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	MODBUS_ADRESS++;
	     	gran(&MODBUS_ADRESS,1,100);
	     	lc640_write_int(EE_MODBUS_ADRESS,MODBUS_ADRESS);
			speed=1;
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	MODBUS_ADRESS--;
	     	gran(&MODBUS_ADRESS,1,100);
	     	lc640_write_int(EE_MODBUS_ADRESS,MODBUS_ADRESS);
			speed=1;
	     	}
          }

     else if(sub_ind==24)
	     {
	     if((but==butR)||(but==butR_))
	     	{
			if(MODBUS_BAUDRATE==120)MODBUS_BAUDRATE=240;
			else if(MODBUS_BAUDRATE==240)MODBUS_BAUDRATE=480;
	     	else if(MODBUS_BAUDRATE==480)MODBUS_BAUDRATE=960;
			else if(MODBUS_BAUDRATE==960)MODBUS_BAUDRATE=1920;
			else if(MODBUS_BAUDRATE==1920)MODBUS_BAUDRATE=3840;
			//else if(MODBUS_BAUDRATE==3840)MODBUS_BAUDRATE=5760;
			else if(MODBUS_BAUDRATE==3840)MODBUS_BAUDRATE=5760;
			else if(MODBUS_BAUDRATE==5760)MODBUS_BAUDRATE=11520;
			else if(MODBUS_BAUDRATE==11520)MODBUS_BAUDRATE=120;
			else MODBUS_BAUDRATE=960;
	     	gran(&MODBUS_BAUDRATE,120,11520);
	     	lc640_write_int(EE_MODBUS_BAUDRATE,MODBUS_BAUDRATE);
			#ifdef SC16IS740_UART
			sc16is700_init((uint32_t)(MODBUS_BAUDRATE*10UL));
			#endif
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
			if(MODBUS_BAUDRATE==120)MODBUS_BAUDRATE=11520;
			else if(MODBUS_BAUDRATE==240)MODBUS_BAUDRATE=120;
	     	else if(MODBUS_BAUDRATE==480)MODBUS_BAUDRATE=240;
			else if(MODBUS_BAUDRATE==960)MODBUS_BAUDRATE=480;
			else if(MODBUS_BAUDRATE==1920)MODBUS_BAUDRATE=960;
			else if(MODBUS_BAUDRATE==3840)MODBUS_BAUDRATE=1920;
			//else if(MODBUS_BAUDRATE==3840)MODBUS_BAUDRATE=3840;
			else if(MODBUS_BAUDRATE==5760)MODBUS_BAUDRATE=3840;
			else if(MODBUS_BAUDRATE==11520)MODBUS_BAUDRATE=5760;
			else MODBUS_BAUDRATE=960;
	     	gran(&MODBUS_BAUDRATE,120,11520);
	     	lc640_write_int(EE_MODBUS_BAUDRATE,MODBUS_BAUDRATE);
			#ifdef SC16IS740_UART
			sc16is700_init((uint32_t)(MODBUS_BAUDRATE*10UL));
			#endif
	     	}
          }
 
	else if(sub_ind==26)
		{
		if(but==butE)
		     {
		     tree_up(iRele_set_sel,0,0,0);
		     ret(1000);
		     }
		} 
	else if(sub_ind==27)
		{
		if(but==butE)
		     {
		     tree_up(iByps_av_set,0,0,0);
		     ret(1000);
		     }
		}
    else if(sub_ind==29)
		{
		if(but==butE)
		     {
		     tree_up(iExt_set,0,0,0);
		     ret(1000);
		     }
		}		 
 	else if(sub_ind==30)
		{
		long t6,t7,t1,t2,t3;
		if(sub_ind1==1)t6=1L;
		if(sub_ind1==2)t6=10L;
		if(sub_ind1==3)t6=100L;
		if(sub_ind1==4)t6=1000L;
		if(sub_ind1==5)t6=10000L;
		if(sub_ind1==6)t6=100000L;
		t7=t6*10L;
		if(but==butE_)
			{
			if(sub_ind1==0)sub_ind1=6;
			else 
				{
				sub_ind1--;
				gran_ring_char(&sub_ind1,1,6);
				}
			speed=0;
			}
		
		if(sub_ind1)
			{
			t1=AUSW_MAIN_NUMBER%t7;
			t1/=t6;
			t2=t1*t6;
	
		    if((but==butR)||(but==butR_))
				{
				t1++;
				speed=1;
				}
		    else if((but==butL)||(but==butL_))
				{
				t1--;
				speed=1;
				}
			gran_ring_long(&t1, 0L, 9L);	    
	
			t3=t1*t6;
	
			AUSW_MAIN_NUMBER-=t2;
			AUSW_MAIN_NUMBER+=t3;
			//else if(but==butEL_)AUSW_MAIN_NUMBER=15000;
			//if(AUSW_MAIN_NUMBER<13000)AUSW_MAIN_NUMBER=200000;
			//if(AUSW_MAIN_NUMBER>200000)AUSW_MAIN_NUMBER=13000;
			if((but==butR)||(but==butR_)||(but==butL)||(but==butL_))gran_ring_long(&AUSW_MAIN_NUMBER, 1L, 999999L);
		    lc640_write_int(EE_AUSW_MAIN_NUMBER,(short)(AUSW_MAIN_NUMBER&0x0000ffffUL));
			lc640_write_int(EE_AUSW_MAIN_NUMBER+2,(short)((AUSW_MAIN_NUMBER&0xffff0000UL)>>16UL));
		    speed=1;
			}
	    }                       		
 
     else if(sub_ind==31)
		{
		if(but==butE)
		     {
		     tree_down(0,0);
		     ret(0);
		     }
		}
				
	else if(sub_ind==32)
		{
		if(but==butE)
		     {		
			tree_up(iK_prl,0,0,0);
			parol_init();
			ret(50);
			}						
		}
     }


else if(ind==iDef)

#define SIMAXIDEF 4
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butD_)
		{
		sub_ind=SIMAXIDEF;
		}
	
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			lc640_write_int(EE_U_OUT_SET,220);
			lc640_write_int(EE_U_OUT_MAX,253);
			lc640_write_int(EE_U_OUT_MIN,187);
			lc640_write_int(EE_U_NET_MAX,187);
			lc640_write_int(EE_U_NET_MIN,182);
			lc640_write_int(EE_U_BAT_MAX,23);
			lc640_write_int(EE_U_BAT_MIN,20);
			lc640_write_int(EE_AUSW_MAIN,24);

	     	lc640_write_int(EE_U_OUT_AC_MAX_AV,253);
	     	lc640_write_int(EE_U_OUT_AC_MIN_AV,187);
	     	lc640_write_int(EE_U_IN_AC_MAX_AV,253);
	     	lc640_write_int(EE_U_IN_AC_MIN_AV,187);
	     	lc640_write_int(EE_U_IN_DC_MAX_AV,30);
	     	lc640_write_int(EE_U_IN_DC_MIN_AV,20);
			}

		else if(sub_ind==1)
			{
			lc640_write_int(EE_U_OUT_SET,220);
			lc640_write_int(EE_U_OUT_MAX,253);
			lc640_write_int(EE_U_OUT_MIN,187);
			lc640_write_int(EE_U_NET_MAX,187);
			lc640_write_int(EE_U_NET_MIN,182);
			lc640_write_int(EE_U_BAT_MAX,45);
			lc640_write_int(EE_U_BAT_MIN,40);
			lc640_write_int(EE_AUSW_MAIN,4860);

	     	lc640_write_int(EE_U_OUT_AC_MAX_AV,253);
	     	lc640_write_int(EE_U_OUT_AC_MIN_AV,187);
	     	lc640_write_int(EE_U_IN_AC_MAX_AV,253);
	     	lc640_write_int(EE_U_IN_AC_MIN_AV,187);
	     	lc640_write_int(EE_U_IN_DC_MAX_AV,70);
	     	lc640_write_int(EE_U_IN_DC_MIN_AV,40);
			}

		else if(sub_ind==2)
			{
			lc640_write_int(EE_U_OUT_SET,220);
			lc640_write_int(EE_U_OUT_MAX,253);
			lc640_write_int(EE_U_OUT_MIN,187);
			lc640_write_int(EE_U_NET_MAX,187);
			lc640_write_int(EE_U_NET_MIN,182);
			lc640_write_int(EE_U_BAT_MAX,100);
			lc640_write_int(EE_U_BAT_MIN,90);
			lc640_write_int(EE_AUSW_MAIN,110);

	     	lc640_write_int(EE_U_OUT_AC_MAX_AV,253);
	     	lc640_write_int(EE_U_OUT_AC_MIN_AV,187);
	     	lc640_write_int(EE_U_IN_AC_MAX_AV,253);
	     	lc640_write_int(EE_U_IN_AC_MIN_AV,187);
	     	lc640_write_int(EE_U_IN_DC_MAX_AV,90);
	     	lc640_write_int(EE_U_IN_DC_MIN_AV,150);
			}
		else if(sub_ind==3)
			{
			lc640_write_int(EE_U_OUT_SET,220);
			lc640_write_int(EE_U_OUT_MAX,253);
			lc640_write_int(EE_U_OUT_MIN,187);
			lc640_write_int(EE_U_NET_MAX,187);
			lc640_write_int(EE_U_NET_MIN,182);
			lc640_write_int(EE_U_BAT_MAX,180);
			lc640_write_int(EE_U_BAT_MIN,170);
			lc640_write_int(EE_AUSW_MAIN,220);

	     	lc640_write_int(EE_U_OUT_AC_MAX_AV,253);
	     	lc640_write_int(EE_U_OUT_AC_MIN_AV,187);
	     	lc640_write_int(EE_U_IN_AC_MAX_AV,253);
	     	lc640_write_int(EE_U_IN_AC_MIN_AV,187);
	     	lc640_write_int(EE_U_IN_DC_MAX_AV,260);
	     	lc640_write_int(EE_U_IN_DC_MIN_AV,170);
			}


		else if(sub_ind==SIMAXIDEF)
			{
			tree_down(0,0);
			}
		default_temp=sub_ind;	
		}
     }




else if(ind==iSet_T)
	{
	signed char temp;
	if(but==butR)
		{
		sub_ind++;
		gran_char(&sub_ind,0,5);
		}
	else if(but==butL)
		{
		sub_ind--;
		gran_char(&sub_ind,0,5);
		}
	else if(but==butE)
		{
		tree_down(0,0);
		}	
	else if(sub_ind==0)
	     {			    
	     temp=LPC_RTC->HOUR;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,0,23);
	          LPC_RTC->HOUR=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,0,23);
	          LPC_RTC->HOUR=temp;
	          }	
	     speed=1;               
	     }
     else if(sub_ind==1)
	     {
	     temp=LPC_RTC->MIN;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,0,59);
	          LPC_RTC->MIN=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,0,59);
	          LPC_RTC->MIN=temp;
	          }	
	     speed=1;               
	     }
     else if(sub_ind==2)
	     {				  
	     temp=LPC_RTC->SEC;
	     if((but==butU)||(but==butU_))
	          {
	          temp=0;
	          gran_ring_char(&temp,0,59);
	          LPC_RTC->SEC=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp=0;
	          gran_ring_char(&temp,0,59);
	          LPC_RTC->SEC=temp;
	          }	
	     speed=1;               
	     }

     else if(sub_ind==3)
	     {
	     temp=LPC_RTC->DOM;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,1,31);
	          LPC_RTC->DOM=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,1,31);
	          LPC_RTC->DOM=temp;
	          }	
	     speed=1;               
	     }
     else if(sub_ind==4)
	     {
	     temp=LPC_RTC->MONTH;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,1,12);
	          LPC_RTC->MONTH=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,1,12);
	          LPC_RTC->MONTH=temp;
	          }	
	     speed=1;               
	     }	  
     else if(sub_ind==5)
	     {
	     temp=LPC_RTC->YEAR;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,0,99);
	          LPC_RTC->YEAR=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,0,99);
	          LPC_RTC->YEAR=temp;
	          }	
	     speed=1;               
	     }		        
	}  

/*
else if(ind==iSet_T_avt)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(SNTP_ENABLE==0)gran_char(&sub_ind,0,1);
		else gran_char(&sub_ind,0,2); 
		}
	else if(but==butU)
		{
		sub_ind--;
		if(SNTP_ENABLE==0)gran_char(&sub_ind,0,1);
		else gran_char(&sub_ind,0,2); 
		}
			
     else if(sub_ind==0)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	SNTP_ENABLE++;
	     	gran(&SNTP_ENABLE,0,3);
	     	lc640_write_int(EE_SNTP_ENABLE,SNTP_ENABLE);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	SNTP_ENABLE--;
	     	gran(&SNTP_ENABLE,0,3);
	     	lc640_write_int(EE_SNTP_ENABLE,SNTP_ENABLE);
	     	}
          }
     else if(sub_ind==1)
		{
		if(SNTP_ENABLE==0)
		 	{
				if(but==butE)
	          	{
				tree_down(0,0);
	          	}
			}
		else 
			{
	     	if((but==butR)||(but==butR_))
	     		{
	     		SNTP_GMT++;
	     		gran(&SNTP_GMT,-12,13);
	     		lc640_write_int(EE_SNTP_GMT,SNTP_GMT);
	     		}
	     
	     	else if((but==butL)||(but==butL_))
	     		{
	     		SNTP_GMT--;
	     		gran(&SNTP_GMT,-12,13);
	     		lc640_write_int(EE_SNTP_GMT,SNTP_GMT);
	     		}
			}
		}	     			  
          
	else if(sub_ind==2)
		{
		if(but==butE)
			{
			tree_down(0,0);
			}
		}	          
	}     
*/
else if(ind==iSet_T_avt)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(SNTP_ENABLE==0)gran_char(&sub_ind,0,1);
		else gran_char(&sub_ind,0,5);
		if(sub_ind==2)
			{
			index_set=2;
			}
		if(sub_ind==3)
			{
			sub_ind=4; 
			index_set=3;
			}
		}
	else if(but==butU)
		{
		sub_ind--;
		if(SNTP_ENABLE==0)gran_char(&sub_ind,0,1);
		else gran_char(&sub_ind,0,5);
		if(sub_ind==3)
			{
			sub_ind=2;
			index_set=2;
			} 
		}
/*	else if(but==butD_)
		{
		sub_ind=4;
		}*/				
     else if(sub_ind==0)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	SNTP_ENABLE++;
	     	gran(&SNTP_ENABLE,0,3);
	     	lc640_write_int(EE_SNTP_ENABLE,SNTP_ENABLE);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	SNTP_ENABLE--;
	     	gran(&SNTP_ENABLE,0,3);
	     	lc640_write_int(EE_SNTP_ENABLE,SNTP_ENABLE);
	     	}
          }
     else if(sub_ind==1)
		{
		if(SNTP_ENABLE==0)
		 	{
			if(but==butE)
	          	{
				tree_down(0,0);
	          	}
			}
		else 
			{
	     	if((but==butR)||(but==butR_))
	     		{
	     		SNTP_GMT++;
	     		gran(&SNTP_GMT,-12,13);
	     		lc640_write_int(EE_SNTP_GMT,SNTP_GMT);
	     		}
	     
	     	else if((but==butL)||(but==butL_))
	     		{
	     		SNTP_GMT--;
	     		gran(&SNTP_GMT,-12,13);
	     		lc640_write_int(EE_SNTP_GMT,SNTP_GMT);
	     		}
			}
		}	     			  
          
/*	else if(sub_ind==2)
		{
		if(but==butE)
			{
			sntp_requ();
			tree_down(0,0);
			}
		}  */
     else if(sub_ind==2)
		{
		if(but==butE)
	     	{
	     	if(lc640_read_int(EE_SNTP_WEB_ENABLE)==1)lc640_write_int(EE_SNTP_WEB_ENABLE,0);
			else lc640_write_int(EE_SNTP_WEB_ENABLE,1);
	     	}
		else if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,3);
	     	}
		else if(sub_ind1==0)
			{
			short _temp;
			if((but==butR)||(but==butR_))
				{
				_temp=lc640_read_int(EE_SNTP_IP1);
				_temp++;
				gran_ring(&_temp,0,255);
				lc640_write_int(EE_SNTP_IP1,_temp);
				}
			else if((but==butL)||(but==butL_))
				{
				_temp=lc640_read_int(EE_SNTP_IP1);
				_temp--;
				gran_ring(&_temp,0,255);
				lc640_write_int(EE_SNTP_IP1,_temp);
				}
			speed=1;
			}
		else if(sub_ind1==1)
			{
			short _temp;
			if((but==butR)||(but==butR_))
				{
				_temp=lc640_read_int(EE_SNTP_IP2);
				_temp++;
				gran_ring(&_temp,0,255);
				lc640_write_int(EE_SNTP_IP2,_temp);
				}
			else if((but==butL)||(but==butL_))
				{
				_temp=lc640_read_int(EE_SNTP_IP2);
				_temp--;
				gran_ring(&_temp,0,255);
				lc640_write_int(EE_SNTP_IP2,_temp);
				}
			speed=1;
			}
		else if(sub_ind1==2)
			{
			short _temp;
			if((but==butR)||(but==butR_))
				{
				_temp=lc640_read_int(EE_SNTP_IP3);
				_temp++;
				gran_ring(&_temp,0,255);
				lc640_write_int(EE_SNTP_IP3,_temp);
				}
			else if((but==butL)||(but==butL_))
				{
				_temp=lc640_read_int(EE_SNTP_IP3);
				_temp--;
				gran_ring(&_temp,0,255);
				lc640_write_int(EE_SNTP_IP3,_temp);
				}
			speed=1;
			}
		else if(sub_ind1==3)
			{
			short _temp;
			if((but==butR)||(but==butR_))
				{
				_temp=lc640_read_int(EE_SNTP_IP4);
				_temp++;
				gran_ring(&_temp,0,255);
				lc640_write_int(EE_SNTP_IP4,_temp);
				}
			else if((but==butL)||(but==butL_))
				{
				_temp=lc640_read_int(EE_SNTP_IP4);
				_temp--;
				gran_ring(&_temp,0,255);
				lc640_write_int(EE_SNTP_IP4,_temp);
				}
			speed=1;
			}
    	}
     else if(sub_ind==4)
		{
		if(but==butE_)
	        {
			if(lc640_read_int(EE_SNTP_WEB_ENABLE)==1)
				{
				Rem_IP[0]=SNTP_IP1;
				Rem_IP[1]=SNTP_IP2;
				Rem_IP[2]=SNTP_IP3;
				Rem_IP[3]=SNTP_IP4;
				}
			else
				{
				Rem_IP[0]=(char)lc640_read_int(EE_SNTP_IP1);
				Rem_IP[1]=(char)lc640_read_int(EE_SNTP_IP2);
				Rem_IP[2]=(char)lc640_read_int(EE_SNTP_IP3);
				Rem_IP[3]=(char)lc640_read_int(EE_SNTP_IP4);
				}
			sntp_requ();
	        }
		}	
     else if(sub_ind==5)
		{

			if(but==butE)
	          	{
				tree_down(0,0);
	          	}
	
		}					          
	} 

else if(ind==iStr_INV)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(NUMBYPASS==0)gran_char(&sub_ind,0,5);
		else gran_char(&sub_ind,0,3);
		}
	else if(but==butU)
		{
		sub_ind--;
		if(NUMBYPASS==0)gran_char(&sub_ind,0,5);
		else gran_char(&sub_ind,0,3);
		}
	else if(but==butD_)
		{
		sub_ind=3;
		}				
     else if(sub_ind==0)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMINV++;
	     	gran(&NUMINV,0,32);
	     	lc640_write_int(EE_NUMINV,NUMINV);
			speed=1;
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMINV--;
	     	gran(&NUMINV,0,32);
	     	lc640_write_int(EE_NUMINV,NUMINV);
			speed=1;
	     	}
          }
     else if(sub_ind==1)
	     {
	     if((but==butR)||(but==butR_))
	     	{
			if(NUMBYPASS==0)
				{
				NUMBYPASS=1;
	     		NUMPHASE=1;
				}
			else if((NUMBYPASS==1)&&(NUMPHASE==1))
				{
				NUMBYPASS=1;
	     		NUMPHASE=3;
				}
			else
				{
				NUMBYPASS=0;
	     		NUMPHASE=0;
				}
			lc640_write_int(EE_NUMBYPASS,NUMBYPASS);
			lc640_write_int(EE_NUMPHASE,NUMPHASE);
			}
	     
	     else if((but==butL)||(but==butL_))
	     	{
			if(NUMBYPASS==0)
				{
				NUMBYPASS=1;
	     		NUMPHASE=3;
				}
			else if((NUMBYPASS==1)&&(NUMPHASE==3))
				{
				NUMBYPASS=1;
	     		NUMPHASE=1;
				}
			else
				{
				NUMBYPASS=0;
	     		NUMPHASE=0;
				}
			lc640_write_int(EE_NUMBYPASS,NUMBYPASS);
			lc640_write_int(EE_NUMPHASE,NUMPHASE);
			}
       	}	     			  
          
    else if(sub_ind==2)
		{
		if(NUMBYPASS==0)
		 	{
			if((but==butR)||(but==butR_))
				{
		     	NUMPHASE=3;
		     	lc640_write_int(EE_NUMPHASE,NUMPHASE);
		     	}
		     
			else if((but==butL)||(but==butL_))
		     	{
		     	NUMPHASE=1;
		     	lc640_write_int(EE_NUMPHASE,NUMPHASE);

		     	}
			}
		else
			{
		     if((but==butR)||(but==butR_))
		     	{
		     	NUMSK++;
		     	gran(&NUMSK,0,4);
		     	lc640_write_int(EE_NUMSK,NUMSK);
		     	}
		     
		     else if((but==butL)||(but==butL_))
		     	{
		     	NUMSK--;
		     	gran(&NUMSK,0,4);
		     	lc640_write_int(EE_NUMSK,NUMSK);
		     	}
			}
		}	 
    else if(sub_ind==3)
		{
		if(NUMBYPASS==0)
		 	{
			if((but==butR)||(but==butR_))
				{
				NUMINAC=1;
				lc640_write_int(EE_NUMINAC,NUMINAC);
				}
			     
			else if((but==butL)||(but==butL_))
				{
				NUMINAC=0;
				lc640_write_int(EE_NUMINAC,NUMINAC);
		     	}
			}
		else
			{
	     	if(but==butE)
	          	{
				tree_down(0,0);
	          	}
			}
		}
     else if(sub_ind==4)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMSK++;
	     	gran(&NUMSK,0,4);
	     	lc640_write_int(EE_NUMSK,NUMSK);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMSK--;
	     	gran(&NUMSK,0,4);
	     	lc640_write_int(EE_NUMSK,NUMSK);
	     	}
          }	      
    else if(sub_ind==5)
		{
     	if(but==butE)
			{
			tree_down(0,0);
			}
		}	 
	}     

else if (ind==iExt_set)
	{
	char si_max;
	si_max=NUMSK;
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,si_max);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,si_max);
		}
	else if(but==butD_)
		{
		sub_ind=si_max;
		}				
/*	else if((sub_ind==0)&&(NUMDT))
	     {
	     if(but==butE)
	     	{
	     	tree_up(iExt_dt,1,0,0);
	     	}
	     }*/
	else if((sub_ind>=/*(NUMDT-1)*/0)&&(sub_ind<(si_max)))
		{
		tree_up(iExt_sk,1,0,sub_ind);
		}		
    else if(sub_ind==si_max)
	     {
	     if(but==butE)
	          {
	          tree_down(0,0);
	          }
          }	          	
	}     

else if (ind==iLan_set)
	{
	char si_max;
	ret(1000);

	si_max=1;
	if(ETH_IS_ON!=0)si_max=21;

	if(but==butD)
		{
		sub_ind++;

		if((sub_ind==2)&&(index_set==0))
			{
			index_set=1;
			sub_ind1=0;
			}
		if(sub_ind==3) 
			{
			sub_ind=4;
			index_set=3;
			sub_ind1=0;
			}
		if(sub_ind==5) 
			{
			sub_ind=6;
			index_set=5;
			sub_ind1=0;
			}
		if(sub_ind==7) 
			{
			sub_ind=8;
			//index_set=3;
			sub_ind1=0;
			}
		if(sub_ind==10) 
			{
			//sub_ind=6;
			//index_set=9;
			sub_ind1=0;
			}
		if(sub_ind==11) 
			{
			//sub_ind=6;
			index_set=10;
			sub_ind1=0;
			}
		if(sub_ind==12) 
			{
			sub_ind++;
			}
		if(sub_ind==13) 
			{
			//sub_ind=6;
			index_set=12;
			sub_ind1=0;
			}
		if(sub_ind==14) 
			{
			sub_ind++;
			}
		if(sub_ind==15) 
			{
			//sub_ind=6;
			index_set=14;
			sub_ind1=0;
			}
		if(sub_ind==16) 
			{
			sub_ind++;
			}
		if(sub_ind==17) 
			{
			//sub_ind=6;
			index_set=16;
			sub_ind1=0;
			}
		if(sub_ind==18) 
			{
			sub_ind++;
			}
		if(sub_ind==19) 
			{
			//sub_ind=6;
			index_set=18;
			sub_ind1=0;
			}
		if(sub_ind==20) 
			{
			sub_ind++;
			}
	/*	if((sub_ind==4)&&(index_set==2))
			{
			index_set=3;
			sub_ind1=0;
			}*/
		
		gran_char(&sub_ind,0,si_max);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,si_max);
		if(sub_ind==20) 
			{
			sub_ind--;
			}		
		if(sub_ind==18) 
			{
			sub_ind--;
			}		
		if(sub_ind==16) 
			{
			sub_ind--;
			}
		if(sub_ind==14) 
			{
			sub_ind--;
			}
		if(sub_ind==12) 
			{
			sub_ind--;
			}
		if(sub_ind==7) 
			{
			sub_ind--;
			}
		if(sub_ind==5) 
			{
			sub_ind--;
			}
		if(sub_ind==3) 
			{
			sub_ind--;
			}
		}
	else if(but==butD_)
		{
		sub_ind=si_max;
		}
	else if(but==butLR_)
		{
		lc640_write_int(EE_ETH_IS_ON,1);
		lc640_write_int(EE_ETH_DHCP_ON,0);
		lc640_write_int(EE_ETH_IP_1,192);
		lc640_write_int(EE_ETH_IP_2,168);
		lc640_write_int(EE_ETH_IP_3,1);
		lc640_write_int(EE_ETH_IP_4,251);
		lc640_write_int(EE_ETH_MASK_1,255);
		lc640_write_int(EE_ETH_MASK_2,255);
		lc640_write_int(EE_ETH_MASK_3,255);
		lc640_write_int(EE_ETH_MASK_4,0);
		lc640_write_int(EE_ETH_GW_1,192);
		lc640_write_int(EE_ETH_GW_2,168);
		lc640_write_int(EE_ETH_GW_3,1);
		lc640_write_int(EE_ETH_GW_4,254);
		lc640_write_int(EE_ETH_SNMP_PORT_READ,161);
		lc640_write_int(EE_ETH_SNMP_PORT_WRITE,162);
		lc640_write_int(EE_COMMUNITY,'1');
		lc640_write_int(EE_COMMUNITY+2,'2');
		lc640_write_int(EE_COMMUNITY+4,'3');
		lc640_write_int(EE_COMMUNITY+6,0);
		lc640_write_int(EE_COMMUNITY+8,0);
		lc640_write_int(EE_ETH_TRAP1_IP_1,255);
		lc640_write_int(EE_ETH_TRAP1_IP_2,255);
		lc640_write_int(EE_ETH_TRAP1_IP_3,255);
		lc640_write_int(EE_ETH_TRAP1_IP_4,255);
		lc640_write_int(EE_ETH_TRAP2_IP_1,255);
		lc640_write_int(EE_ETH_TRAP2_IP_2,255);
		lc640_write_int(EE_ETH_TRAP2_IP_3,255);
		lc640_write_int(EE_ETH_TRAP2_IP_4,255);
		lc640_write_int(EE_ETH_TRAP3_IP_1,255);
		lc640_write_int(EE_ETH_TRAP3_IP_2,255);
		lc640_write_int(EE_ETH_TRAP3_IP_3,255);
		lc640_write_int(EE_ETH_TRAP3_IP_4,255);
		lc640_write_int(EE_ETH_TRAP4_IP_1,255);
		lc640_write_int(EE_ETH_TRAP4_IP_2,255);
		lc640_write_int(EE_ETH_TRAP4_IP_3,255);
		lc640_write_int(EE_ETH_TRAP4_IP_4,255);
		lc640_write_int(EE_ETH_TRAP5_IP_1,255);
		lc640_write_int(EE_ETH_TRAP5_IP_2,255);
		lc640_write_int(EE_ETH_TRAP5_IP_3,255);
		lc640_write_int(EE_ETH_TRAP5_IP_4,255);
		}					
	else if(sub_ind==0)
	     {
	     if((but==butE)||(but==butL)||(but==butR))
	     	{
	     	if(ETH_IS_ON)lc640_write_int(EE_ETH_IS_ON,0);
			else lc640_write_int(EE_ETH_IS_ON,1);
	     	}
	     }	
     else if((sub_ind==1)&&(ETH_IS_ON))
	     {
		if((but==butE)||(but==butL)||(but==butR))
	     	{
	     	if(ETH_DHCP_ON)lc640_write_int(EE_ETH_DHCP_ON,0);
			else lc640_write_int(EE_ETH_DHCP_ON,1);
	     	}
		}	
     else if(sub_ind==2)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,3);
	     	}
		else if(sub_ind1==0)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_IP_1++;
				gran_ring(&ETH_IP_1,0,255);
				lc640_write_int(EE_ETH_IP_1,ETH_IP_1);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_IP_1--;
				gran_ring(&ETH_IP_1,0,255);
				lc640_write_int(EE_ETH_IP_1,ETH_IP_1);
				}
			speed=1;
			}
		else if(sub_ind1==1)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_IP_2++;
				gran_ring(&ETH_IP_2,0,255);
				lc640_write_int(EE_ETH_IP_2,ETH_IP_2);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_IP_2--;
				gran_ring(&ETH_IP_2,0,255);
				lc640_write_int(EE_ETH_IP_2,ETH_IP_2);
				}
			speed=1;
			}
		else if(sub_ind1==2)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_IP_3++;
				gran_ring(&ETH_IP_3,0,255);
				lc640_write_int(EE_ETH_IP_3,ETH_IP_3);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_IP_3--;
				gran_ring(&ETH_IP_3,0,255);
				lc640_write_int(EE_ETH_IP_3,ETH_IP_3);
				}
			speed=1;
			}
		else if(sub_ind1==3)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_IP_4++;
				gran_ring(&ETH_IP_4,0,255);
				lc640_write_int(EE_ETH_IP_4,ETH_IP_4);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_IP_4--;
				gran_ring(&ETH_IP_4,0,255);
				lc640_write_int(EE_ETH_IP_4,ETH_IP_4);
				}
			speed=1;
			}

          }
     else if(sub_ind==4)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,3);
	     	}
		else if(sub_ind1==0)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_MASK_1++;
				gran_ring(&ETH_MASK_1,0,255);
				lc640_write_int(EE_ETH_MASK_1,ETH_MASK_1);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_MASK_1--;
				gran_ring(&ETH_MASK_1,0,255);
				lc640_write_int(EE_ETH_MASK_1,ETH_MASK_1);
				}
			speed=1;
			}
		else if(sub_ind1==1)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_MASK_2++;
				gran_ring(&ETH_MASK_2,0,255);
				lc640_write_int(EE_ETH_MASK_2,ETH_MASK_2);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_MASK_2--;
				gran_ring(&ETH_MASK_2,0,255);
				lc640_write_int(EE_ETH_MASK_2,ETH_MASK_2);
				}
			speed=1;
			}
		else if(sub_ind1==2)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_MASK_3++;
				gran_ring(&ETH_MASK_3,0,255);
				lc640_write_int(EE_ETH_MASK_3,ETH_MASK_3);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_MASK_3--;
				gran_ring(&ETH_MASK_3,0,255);
				lc640_write_int(EE_ETH_MASK_3,ETH_MASK_3);
				}
			speed=1;
			}
		else if(sub_ind1==3)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_MASK_4++;
				gran_ring(&ETH_MASK_4,0,255);
				lc640_write_int(EE_ETH_MASK_4,ETH_MASK_4);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_MASK_4--;
				gran_ring(&ETH_MASK_4,0,255);
				lc640_write_int(EE_ETH_MASK_4,ETH_MASK_4);
				}
			speed=1;
			}
		}
     else if(sub_ind==6)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,3);
	     	}
		else if(sub_ind1==0)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_GW_1++;
				gran_ring(&ETH_GW_1,0,255);
				lc640_write_int(EE_ETH_GW_1,ETH_GW_1);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_GW_1--;
				gran_ring(&ETH_GW_1,0,255);
				lc640_write_int(EE_ETH_GW_1,ETH_GW_1);
				}
			speed=1;
			}
		else if(sub_ind1==1)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_GW_2++;
				gran_ring(&ETH_GW_2,0,255);
				lc640_write_int(EE_ETH_GW_2,ETH_GW_2);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_GW_2--;
				gran_ring(&ETH_GW_2,0,255);
				lc640_write_int(EE_ETH_GW_2,ETH_GW_2);
				}
			speed=1;
			}
		else if(sub_ind1==2)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_GW_3++;
				gran_ring(&ETH_GW_3,0,255);
				lc640_write_int(EE_ETH_GW_3,ETH_GW_3);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_GW_3--;
				gran_ring(&ETH_GW_3,0,255);
				lc640_write_int(EE_ETH_GW_3,ETH_GW_3);
				}
			speed=1;
			}
		else if(sub_ind1==3)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_GW_4++;
				gran_ring(&ETH_GW_4,0,255);
				lc640_write_int(EE_ETH_GW_4,ETH_GW_4);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_GW_4--;
				gran_ring(&ETH_GW_4,0,255);
				lc640_write_int(EE_ETH_GW_4,ETH_GW_4);
				}
			speed=1;
			}
		}
      else if(sub_ind==8)
	     {
		if(but==butR)ETH_SNMP_PORT_READ++;
		else if(but==butR_)ETH_SNMP_PORT_READ+=2;
		else if(but==butL)ETH_SNMP_PORT_READ--;
		else if(but==butL_)ETH_SNMP_PORT_READ-=2;
		speed=1;
		lc640_write_int(EE_ETH_SNMP_PORT_READ,ETH_SNMP_PORT_READ);
		}

     else if(sub_ind==9)
	     {
		if(but==butR)ETH_SNMP_PORT_WRITE++;
		else if(but==butR_)ETH_SNMP_PORT_WRITE+=2;
		else if(but==butL)ETH_SNMP_PORT_WRITE--;
		else if(but==butL_)ETH_SNMP_PORT_WRITE-=2;
		speed=1;
		lc640_write_int(EE_ETH_SNMP_PORT_WRITE,ETH_SNMP_PORT_WRITE);
		}					
     else if(sub_ind==10)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,8);
	     	}
		if((but==butR)||(but==butR_))
			{
			snmp_community[sub_ind1]++;
			if(snmp_community[sub_ind1]<32) snmp_community[sub_ind1]=32;
			else if ((snmp_community[sub_ind1]>32)&&(snmp_community[sub_ind1]<48)) snmp_community[sub_ind1]=48;
			else if ((snmp_community[sub_ind1]>57)&&(snmp_community[sub_ind1]<65)) snmp_community[sub_ind1]=65;
			else if ((snmp_community[sub_ind1]>90)&&(snmp_community[sub_ind1]<97)) snmp_community[sub_ind1]=97;
			else if (snmp_community[sub_ind1]>122) snmp_community[sub_ind1]=32;
				//gran_ring(&ETH_GW_1,0,255);
			lc640_write_int(EE_COMMUNITY+(sub_ind1*2),snmp_community[sub_ind1]);
			speed=1;
			}
		if((but==butL)||(but==butL_))
			{
			snmp_community[sub_ind1]--;
			if(snmp_community[sub_ind1]<32) snmp_community[sub_ind1]=122;
			else if ((snmp_community[sub_ind1]>32)&&(snmp_community[sub_ind1]<48)) snmp_community[sub_ind1]=32;
			else if ((snmp_community[sub_ind1]>57)&&(snmp_community[sub_ind1]<65)) snmp_community[sub_ind1]=57;
			else if ((snmp_community[sub_ind1]>90)&&(snmp_community[sub_ind1]<97)) snmp_community[sub_ind1]=90;
			else if (snmp_community[sub_ind1]>122) snmp_community[sub_ind1]=122;
			//gran_ring(&ETH_GW_1,0,255);
			lc640_write_int(EE_COMMUNITY+(sub_ind1*2),snmp_community[sub_ind1]);
			speed=1;
			}
		}
 
     else if(sub_ind==11)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,3);
	     	}
		else if(sub_ind1==0)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP1_IP_1++;
				gran_ring(&ETH_TRAP1_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP1_IP_1,ETH_TRAP1_IP_1);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP1_IP_1--;
				gran_ring(&ETH_TRAP1_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP1_IP_1,ETH_TRAP1_IP_1);
				}
			speed=1;
			}
		else if(sub_ind1==1)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP1_IP_2++;
				gran_ring(&ETH_TRAP1_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP1_IP_2,ETH_TRAP1_IP_2);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP1_IP_2--;
				gran_ring(&ETH_TRAP1_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP1_IP_2,ETH_TRAP1_IP_2);
				}
			speed=1;
			}
		else if(sub_ind1==2)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP1_IP_3++;
				gran_ring(&ETH_TRAP1_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP1_IP_3,ETH_TRAP1_IP_3);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP1_IP_3--;
				gran_ring(&ETH_TRAP1_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP1_IP_3,ETH_TRAP1_IP_3);
				}
			speed=1;
			}
		else if(sub_ind1==3)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP1_IP_4++;
				gran_ring(&ETH_TRAP1_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP1_IP_4,ETH_TRAP1_IP_4);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP1_IP_4--;
				gran_ring(&ETH_TRAP1_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP1_IP_4,ETH_TRAP1_IP_4);
				}
			speed=1;
			}
		}	
     else if(sub_ind==13)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,3);
	     	}
		else if(sub_ind1==0)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP2_IP_1++;
				gran_ring(&ETH_TRAP2_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP2_IP_1,ETH_TRAP2_IP_1);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP2_IP_1--;
				gran_ring(&ETH_TRAP2_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP2_IP_1,ETH_TRAP2_IP_1);
				}
			speed=1;
			}
		else if(sub_ind1==1)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP2_IP_2++;
				gran_ring(&ETH_TRAP2_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP2_IP_2,ETH_TRAP2_IP_2);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP2_IP_2--;
				gran_ring(&ETH_TRAP2_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP2_IP_2,ETH_TRAP2_IP_2);
				}
			speed=1;
			}
		else if(sub_ind1==2)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP2_IP_3++;
				gran_ring(&ETH_TRAP2_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP2_IP_3,ETH_TRAP2_IP_3);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP2_IP_3--;
				gran_ring(&ETH_TRAP2_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP2_IP_3,ETH_TRAP2_IP_3);
				}
			speed=1;
			}
		else if(sub_ind1==3)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP2_IP_4++;
				gran_ring(&ETH_TRAP2_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP2_IP_4,ETH_TRAP2_IP_4);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP2_IP_4--;
				gran_ring(&ETH_TRAP2_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP2_IP_4,ETH_TRAP2_IP_4);
				}
			speed=1;
			}
		}	
     else if(sub_ind==15)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,3);
	     	}
		else if(sub_ind1==0)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP3_IP_1++;
				gran_ring(&ETH_TRAP3_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP3_IP_1,ETH_TRAP3_IP_1);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP3_IP_1--;
				gran_ring(&ETH_TRAP3_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP3_IP_1,ETH_TRAP3_IP_1);
				}
			speed=1;
			}
		else if(sub_ind1==1)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP3_IP_2++;
				gran_ring(&ETH_TRAP3_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP3_IP_2,ETH_TRAP3_IP_2);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP3_IP_2--;
				gran_ring(&ETH_TRAP3_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP3_IP_2,ETH_TRAP3_IP_2);
				}
			speed=1;
			}
		else if(sub_ind1==2)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP3_IP_3++;
				gran_ring(&ETH_TRAP3_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP3_IP_3,ETH_TRAP3_IP_3);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP3_IP_3--;
				gran_ring(&ETH_TRAP3_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP3_IP_3,ETH_TRAP3_IP_3);
				}
			speed=1;
			}
		else if(sub_ind1==3)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP3_IP_4++;
				gran_ring(&ETH_TRAP3_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP3_IP_4,ETH_TRAP3_IP_4);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP3_IP_4--;
				gran_ring(&ETH_TRAP3_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP3_IP_4,ETH_TRAP3_IP_4);
				}
			speed=1;
			}
		}	
     else if(sub_ind==17)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,3);
	     	}
		else if(sub_ind1==0)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP4_IP_1++;
				gran_ring(&ETH_TRAP4_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP4_IP_1,ETH_TRAP4_IP_1);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP4_IP_1--;
				gran_ring(&ETH_TRAP4_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP4_IP_1,ETH_TRAP4_IP_1);
				}
			speed=1;
			}
		else if(sub_ind1==1)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP4_IP_2++;
				gran_ring(&ETH_TRAP4_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP4_IP_2,ETH_TRAP4_IP_2);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP4_IP_2--;
				gran_ring(&ETH_TRAP4_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP4_IP_2,ETH_TRAP4_IP_2);
				}
			speed=1;
			}
		else if(sub_ind1==2)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP4_IP_3++;
				gran_ring(&ETH_TRAP4_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP4_IP_3,ETH_TRAP4_IP_3);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP4_IP_3--;
				gran_ring(&ETH_TRAP4_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP4_IP_3,ETH_TRAP4_IP_3);
				}
			speed=1;
			}
		else if(sub_ind1==3)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP4_IP_4++;
				gran_ring(&ETH_TRAP4_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP4_IP_4,ETH_TRAP4_IP_4);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP4_IP_4--;
				gran_ring(&ETH_TRAP4_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP4_IP_4,ETH_TRAP4_IP_4);
				}
			speed=1;
			}
		}	
     else if(sub_ind==19)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,3);
	     	}
		else if(sub_ind1==0)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP5_IP_1++;
				gran_ring(&ETH_TRAP5_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP5_IP_1,ETH_TRAP5_IP_1);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP5_IP_1--;
				gran_ring(&ETH_TRAP5_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP5_IP_1,ETH_TRAP5_IP_1);
				}
			speed=1;
			}
		else if(sub_ind1==1)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP5_IP_2++;
				gran_ring(&ETH_TRAP5_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP5_IP_2,ETH_TRAP5_IP_2);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP5_IP_2--;
				gran_ring(&ETH_TRAP5_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP5_IP_2,ETH_TRAP5_IP_2);
				}
			speed=1;
			}
		else if(sub_ind1==2)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP5_IP_3++;
				gran_ring(&ETH_TRAP5_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP5_IP_3,ETH_TRAP5_IP_3);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP5_IP_3--;
				gran_ring(&ETH_TRAP5_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP5_IP_3,ETH_TRAP5_IP_3);
				}
			speed=1;
			}
		else if(sub_ind1==3)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP5_IP_4++;
				gran_ring(&ETH_TRAP5_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP5_IP_4,ETH_TRAP5_IP_4);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP5_IP_4--;
				gran_ring(&ETH_TRAP5_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP5_IP_4,ETH_TRAP5_IP_4);
				}
			speed=1;
			}
		}													          
    else if(sub_ind==si_max)
	     {
	     if(but==butE)
	          {
	          tree_down(0,0);
	          }
          }	          	
	}



else if (ind==iApv)
	{
     ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,simax);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,simax);
		}
	else if(but==butD_)
		{
		sub_ind=simax;
		}			
	else if(but==butE)
	     {
	     if(sub_ind==simax)
	          {
	          //a=b[--ptr_ind];
	          tree_down(0,0);
	          }
	     else if(sub_ind==0)   
	          {
	          if(APV_ON1==apvON)lc640_write_int(EE_APV_ON1,apvOFF);
	          else lc640_write_int(EE_APV_ON1,apvON);
	          }
          else if((sub_ind==1)&&(APV_ON1==apvON))   
	          {
	          if(APV_ON2==apvON)lc640_write_int(EE_APV_ON2,apvOFF);
	          else lc640_write_int(EE_APV_ON2,apvON);
	          }	 
          }
     
     else if((sub_ind==2)&&(APV_ON2==apvON))   
          {
	     if((but==butR)||(but==butR_))
	          {
	          signed short tempSS;
	          tempSS=APV_ON2_TIME;
	          tempSS++;
	          gran(&tempSS,1,24);
	          lc640_write_int(EE_APV_ON2_TIME,tempSS);
	          }
          else if((but==butL)||(but==butL_))
	          {
	          signed short tempSS;
	          tempSS=APV_ON2_TIME;
	          tempSS--;
	          gran(&tempSS,1,24);
	          lc640_write_int(EE_APV_ON2_TIME,tempSS);
	          }	          
	     speed=1;
	     }	 
  	} 

else if (ind==iExt_set)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,3);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);		
		}

	else if((but==butE)&&(sub_ind==0))
		{
	     tree_up(iExt_ddv,0,0,0);
	     ret(0);
		}

	else if((but==butE)&&(sub_ind==1))
		{
	     tree_up(iExt_ddi,0,0,0);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==2))
		{
	     tree_up(iExt_dud,0,0,0);
	     ret(0);
		}
/*	else if((but==butE)&&(sub_ind==3))
		{
	     tree_up(iExt_dp,0,0,0);
	     ret(0);
		} */
		
	else if((but==butE)&&(sub_ind==3))
		{
	     tree_down(0,0);
	     ret(0);
		}        	
	}

else if (ind==iExt_set_3U)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,NUMSK);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMSK);		
		}
 	else if((but==butE)&&(sub_ind==NUMSK))
		{
	     tree_down(0,0);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==0))
		{
	     tree_up(iExt_sk_3U,0,0,0);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==1))
		{
	     tree_up(iExt_sk_3U,0,0,1);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==2))
		{
	     tree_up(iExt_sk_3U,0,0,2);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==3))
		{
	     tree_up(iExt_sk_3U,0,0,3);
	     ret(0);
		} 
	}


	
else if (ind==iExt_dt)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,1,7);
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		gran_char(&sub_ind,1,7);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
		
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			if(!TMAX_EXT_EN[sub_ind1])lc640_write_int(ADR_TMAX_EXT_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_TMAX_EXT_EN[sub_ind1],0);
			}
		else if((but==butR)||(but==butR_))
			{
			TMAX_EXT[sub_ind1]++;
			}	
		else if((but==butL)||(but==butL_))
			{
			TMAX_EXT[sub_ind1]--;
			}	
		gran(&TMAX_EXT[sub_ind1],-50,100);
		if(lc640_read_int(ADR_TMAX_EXT[sub_ind1])!=TMAX_EXT[sub_ind1]) lc640_write_int(ADR_TMAX_EXT[sub_ind1],TMAX_EXT[sub_ind1]);			
		speed=1;
		}
	else if(sub_ind==2) 
		{
		if(but==butE)
			{
			if(!TMIN_EXT_EN[sub_ind1])lc640_write_int(ADR_TMIN_EXT_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_TMIN_EXT_EN[sub_ind1],0);
			}
		else if((but==butR)||(but==butR_))
			{
			TMIN_EXT[sub_ind1]++;
			}	
		else if((but==butL)||(but==butL_))
			{
			TMIN_EXT[sub_ind1]--;
			}	
		gran(&TMIN_EXT[sub_ind1],-50,100);
		if(lc640_read_int(ADR_TMIN_EXT[sub_ind1])!=TMIN_EXT[sub_ind1]) lc640_write_int(ADR_TMIN_EXT[sub_ind1],TMIN_EXT[sub_ind1]);			
		speed=1;
		}		
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			if(!T_EXT_REL_EN[sub_ind1])lc640_write_int(ADR_T_EXT_REL_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_T_EXT_REL_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			if(!T_EXT_ZVUK_EN[sub_ind1])lc640_write_int(ADR_T_EXT_ZVUK_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_T_EXT_ZVUK_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			if(!T_EXT_LCD_EN[sub_ind1])lc640_write_int(ADR_T_EXT_LCD_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_T_EXT_LCD_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==6) 
		{
		if(but==butE)
			{
			if(!T_EXT_RS_EN[sub_ind1])lc640_write_int(ADR_T_EXT_RS_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_T_EXT_RS_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==7) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			//a=b[--ptr_ind];
			}
		}												
	}	

else if (ind==iExt_sk)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,3);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,3);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			if(!SK_SIGN[sub_ind1])lc640_write_int(ADR_SK_SIGN[sub_ind1],0xffff);
			else lc640_write_int(ADR_SK_SIGN[sub_ind1],0);
			}
		}
/*	else if(sub_ind==3) 
		{
	//	if(but==butE)
		//	{
	//		if(!SK_REL_EN[sub_ind1])lc640_write_int(ADR_SK_REL_EN[sub_ind1],0xffff);
	//		else lc640_write_int(ADR_SK_REL_EN[sub_ind1],0);
	//		}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			if(!SK_ZVUK_EN[sub_ind1])lc640_write_int(ADR_SK_ZVUK_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_SK_ZVUK_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			if(!SK_LCD_EN[sub_ind1])lc640_write_int(ADR_SK_LCD_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_SK_LCD_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==6) 
		{
		if(but==butE)
			{
			if(!SK_RS_EN[sub_ind1])lc640_write_int(ADR_SK_RS_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_SK_RS_EN[sub_ind1],0);
			}
		}*/	
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			//a=b[--ptr_ind];
			tree_down(0,0);
			}
		}												
	}	

else if (ind==iExt_sk_3U)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,5);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,5);
		}	
	else if(but==butD_)
		{
		sub_ind=5;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			if(!SK_SIGN[sub_ind1])lc640_write_int(ADR_SK_SIGN[sub_ind1],0xffff);
			else lc640_write_int(ADR_SK_SIGN[sub_ind1],0);
			}
		}
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			if(!SK_ZVUK_EN[sub_ind1])lc640_write_int(ADR_SK_ZVUK_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_SK_ZVUK_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			if(!SK_LCD_EN[sub_ind1])lc640_write_int(ADR_SK_LCD_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_SK_LCD_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			}
		}												
	}	



else if (ind==iExt_ddv)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,5);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,5);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			if(!SK_SIGN[0])lc640_write_int(ADR_SK_SIGN[0],0xffff);
			else lc640_write_int(ADR_SK_SIGN[0],0);
			}
		}
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			if(SK_REL_EN[0])lc640_write_int(ADR_SK_REL_EN[0],0);
			else lc640_write_int(ADR_SK_REL_EN[0],0xffff);
			}
		}	

	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			if(SK_LCD_EN[0])lc640_write_int(ADR_SK_LCD_EN[0],0);
			else lc640_write_int(ADR_SK_LCD_EN[0],0xffff);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			}
		}												
	}	

else if (ind==iExt_ddi)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,5);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,5);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			if(!SK_SIGN[1])lc640_write_int(ADR_SK_SIGN[1],0xffff);
			else lc640_write_int(ADR_SK_SIGN[1],0);
			}
		}
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			if(SK_REL_EN[1])lc640_write_int(ADR_SK_REL_EN[1],0);
			else lc640_write_int(ADR_SK_REL_EN[1],0xffff);
			}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			if(SK_LCD_EN[1])lc640_write_int(ADR_SK_LCD_EN[1],0);
			else lc640_write_int(ADR_SK_LCD_EN[1],0xffff);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			}
		}												
	}
 
 else if (ind==iExt_dud)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,5);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,5);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			if(!SK_SIGN[2])lc640_write_int(ADR_SK_SIGN[2],0xffff);
			else lc640_write_int(ADR_SK_SIGN[2],0);
			}
		}
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			if(SK_REL_EN[2])lc640_write_int(ADR_SK_REL_EN[2],0);
			else lc640_write_int(ADR_SK_REL_EN[2],0xffff);
			}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			if(SK_LCD_EN[2])lc640_write_int(ADR_SK_LCD_EN[2],0);
			else lc640_write_int(ADR_SK_LCD_EN[2],0xffff);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			}
		}												
	}
/*     
else if (ind==iExt_dp)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,5);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,5);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			if(!SK_SIGN[3])lc640_write_int(ADR_SK_SIGN[3],0xffff);
			else lc640_write_int(ADR_SK_SIGN[3],0);
			}
		}
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			if(!SK_REL_EN[3])lc640_write_int(ADR_SK_REL_EN[3],0xffff);
			else lc640_write_int(ADR_SK_REL_EN[3],0);
			}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			if(!SK_LCD_EN[3])lc640_write_int(ADR_SK_LCD_EN[3],0xffff);
			else lc640_write_int(ADR_SK_LCD_EN[3],0);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			}
		}												
	}         	
*/

else if(ind==iRele_set_sel)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2);
		}
	else if(but==butD_)
		{
		sub_ind=4;
		}	
	else if((but==butE)&&(sub_ind>=0)&&(sub_ind<=1))
		{
		tree_up(iRele_set,0,0,sub_ind);	
		ret(1000);
		}	
	else if(sub_ind==2)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}				
	}

else if(ind==iRele_set)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
/*		if(sub_ind==2)
			{
			index_set=1;
			}
		else*/ if(sub_ind==5)
			{
			sub_ind=6;
			index_set=5;
			}
/*		else if(sub_ind==7)
			{
			index_set=6;
			}*/
		else if(sub_ind==4)
			{
			//sub_ind=4;
			index_set=3;
			}
/*		else if(sub_ind==10)
			{
			index_set=9;
			} */
/*		else if(sub_ind==11)
			{
			sub_ind=12;
			} */
		else if(sub_ind==7)
			{
			sub_ind=8;
			//index_set=5;
			}
		gran_char(&sub_ind,0,8);
		}
	else if(but==butU)
		{
		sub_ind--;
		/*if(sub_ind==3)sub_ind=2;
		else*/ if(sub_ind==7)sub_ind=6;
		else if(sub_ind==5)sub_ind=4;
		gran_char(&sub_ind,0,8);		
		}
	else if(but==butD_)
		{
		sub_ind=11;
		}
	else if (sub_ind == 0)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<0);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<0);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<0);
		}	
		
	else if (sub_ind == 1)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<1);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<1);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<1);
		}	
		
	else if (sub_ind == 2)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<2);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<2);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<2);
		}
		
	else if (sub_ind == 3)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<3);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<3);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<3);
		}	
		
	else if (sub_ind == 4)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<4);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<4);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<4);
		}							

	else if (sub_ind == 6)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<15);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<15);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<15);
		}
/*	else if (sub_ind == 8)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<6);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<6);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<6);
		}
	else if (sub_ind == 9)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<7);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<7);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<7);
		}

	else if (sub_ind == 10)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<15);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<15);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<15);
		} */

	else if(sub_ind==8)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	//if(RELE_SET_MASK[sub_ind1]!=lc640_read_int(ADR_EE_RELE_SET_MASK[sub_ind1]))lc640_write_int(ADR_EE_RELE_SET_MASK[sub_ind1],RELE_SET_MASK[sub_ind1]);
	if(RELE_SET_MASK[sub_ind1]!=lc640_read_int(ADR_EE_RELE_SET_MASK[sub_ind1]))lc640_write_int(ADR_EE_RELE_SET_MASK[sub_ind1],RELE_SET_MASK[sub_ind1]);

	} 

else if(ind==iByps_av_set)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,6);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,6);
		}
	else if(but==butD_)
		{
		sub_ind=6;
		}

     else if(sub_ind==0)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	U_OUT_AC_MAX_AV++;
	     	gran(&U_OUT_AC_MAX_AV,20,300);
	     	lc640_write_int(EE_U_OUT_AC_MAX_AV,U_OUT_AC_MAX_AV);
			speed=1;
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	U_OUT_AC_MAX_AV--;
	     	gran(&U_OUT_AC_MAX_AV,20,300);
	     	lc640_write_int(EE_U_OUT_AC_MAX_AV,U_OUT_AC_MAX_AV);
			speed=1;
	     	}
          }

     else if(sub_ind==1)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	U_OUT_AC_MIN_AV++;
	     	gran(&U_OUT_AC_MIN_AV,20,300);
	     	lc640_write_int(EE_U_OUT_AC_MIN_AV,U_OUT_AC_MIN_AV);
			speed=1;
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	U_OUT_AC_MIN_AV--;
	     	gran(&U_OUT_AC_MIN_AV,20,300);
	     	lc640_write_int(EE_U_OUT_AC_MIN_AV,U_OUT_AC_MIN_AV);
			speed=1;
	     	}          
		}

     else if(sub_ind==2)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	U_IN_AC_MAX_AV++;
	     	gran(&U_IN_AC_MAX_AV,20,300);
	     	lc640_write_int(EE_U_IN_AC_MAX_AV,U_IN_AC_MAX_AV);
			speed=1;
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	U_IN_AC_MAX_AV--;
	     	gran(&U_IN_AC_MAX_AV,20,300);
	     	lc640_write_int(EE_U_IN_AC_MAX_AV,U_IN_AC_MAX_AV);
			speed=1;
	     	}
          }

     else if(sub_ind==3)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	U_IN_AC_MIN_AV++;
	     	gran(&U_IN_AC_MIN_AV,20,300);
	     	lc640_write_int(EE_U_IN_AC_MIN_AV,U_IN_AC_MIN_AV);
			speed=1;
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	U_IN_AC_MIN_AV--;
	     	gran(&U_IN_AC_MIN_AV,20,300);
	     	lc640_write_int(EE_U_IN_AC_MIN_AV,U_IN_AC_MIN_AV);
			speed=1;
	     	}          
		}

      else if(sub_ind==4)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	U_IN_DC_MAX_AV++;
	     	gran(&U_IN_DC_MAX_AV,20,300);
	     	lc640_write_int(EE_U_IN_DC_MAX_AV,U_IN_DC_MAX_AV);
			speed=1;
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	U_IN_DC_MAX_AV--;
	     	gran(&U_IN_DC_MAX_AV,20,300);
	     	lc640_write_int(EE_U_IN_DC_MAX_AV,U_IN_DC_MAX_AV);
			speed=1;
	     	}
          }

     else if(sub_ind==5)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	U_IN_DC_MIN_AV++;
	     	gran(&U_IN_DC_MIN_AV,20,300);
	     	lc640_write_int(EE_U_IN_DC_MIN_AV,U_IN_DC_MIN_AV);
			speed=1;
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	U_IN_DC_MIN_AV--;
	     	gran(&U_IN_DC_MIN_AV,20,300);
	     	lc640_write_int(EE_U_IN_DC_MIN_AV,U_IN_DC_MIN_AV);
			speed=1;
	     	}          
		}

     else if(sub_ind==6)
		{
		if(but==butE)
		    {
		    tree_down(0,0);
			ret(0);
			}
		}
	}
			     
else if(ind==iK)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2);
		}
	else if(but==butD_)
		{
		sub_ind=2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2;
		}				
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			tree_up(iK_net,0,0,0);	
			ret(1000);		
			}
		else if((NUMBAT)&&(sub_ind==1))
			{
			tree_up(iK_bat_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==(1+(NUMBAT!=0))))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}
/*		else if((sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
          
         	else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)))
			{
			tree_up(iK_t_ext,0,0,0);	
			ret(1000);			
			}
          else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+1))
			{
			tree_up(iK_power_net3,0,0,0);	
			ret(1000);
               }               				
          else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+2))
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}*/
/**/		else if((NUMINV)&&(sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_inv_sel,0,0,0);	
			ret(1000);
			}
								
			else if((sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0))))		 /**/
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
          
/**/     	else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)))	   /**/
			{
			tree_up(iK_t_ext,0,0,0);	
			ret(1000);			
			}
/**/		else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+1))		/**/
			{
			tree_up(iK_power_net3,0,0,0);	
			ret(1000);
               }               				
/**/	   	else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2))	   /**/
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}					
	}




else if(ind==iK_INV)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3+(NUMBYPASS!=0)+(NUMINV!=0));
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3+(NUMBYPASS!=0)+(NUMINV!=0));
		}
	else if(but==butD_)
		{
		sub_ind=1+(NUMBYPASS!=0)+(NUMINV!=0);
		}
	else if(sub_ind==(3+(NUMBYPASS!=0)+(NUMINV!=0)))
			{
			if((but==butR)||(but==butR_))
				{
				if(RS485_QWARZ_DIGIT==10)RS485_QWARZ_DIGIT=30;
				else if(RS485_QWARZ_DIGIT==30)RS485_QWARZ_DIGIT=40;
				else RS485_QWARZ_DIGIT=10;
				}
			else if((but==butL)||(but==butL_))
				{
				if(RS485_QWARZ_DIGIT==10)RS485_QWARZ_DIGIT=40;
				else if(RS485_QWARZ_DIGIT==40)RS485_QWARZ_DIGIT=30;
				else RS485_QWARZ_DIGIT=10;
				}
			gran(&RS485_QWARZ_DIGIT,10,40);
			lc640_write_int(EE_RS485_QWARZ_DIGIT,RS485_QWARZ_DIGIT);
			speed=0;
			}
	else if(sub_ind==(NUMBYPASS!=0)+(NUMINV!=0))
		{
		temp_SS=lc640_read_int(EE_KUDCIN);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,150,800);
		lc640_write_int(EE_KUDCIN,temp_SS);
		}	
		
	else if(sub_ind==1+(NUMBYPASS!=0)+(NUMINV!=0))
		{
		temp_SS=lc640_read_int(KT_EXT0);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		lc640_write_int(KT_EXT0,temp_SS);					
		speed=1;	
					
		}
										
	else if(but==butE)
		{
		if((sub_ind==0)&&(NUMINV!=0))
			{
			tree_up(iK_inv_sel,0,0,1);	
			ret(1000);
			}		

/*		else if((sub_ind==(NUMINV!=0))&&(NUMBYPASS>1))
			{
			tree_up(iK_byps_sel,0,0,0);	
			ret(1000);
			}*/

		else if((sub_ind==(NUMINV!=0))&&(NUMBYPASS==1)&&(NUMPHASE==1))
			{
			tree_up(iK_byps,0,0,0);	
			ret(1000);
			}

		else if((sub_ind==(NUMINV!=0))&&(NUMBYPASS==1)&&(NUMPHASE==3))
			{
			tree_up(iK_byps_3f,0,0,0);	
			ret(1000);
			}

		              				
		else if(2+(NUMBYPASS!=0)+(NUMINV!=0))
			{
			tree_down(0,0);
			ret(0);
			}	               			
		}										
	}


else if(ind==iK_220)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0));
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0));
		}
	else if(but==butD_)
		{
		sub_ind=2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0);
		}				
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			if(AUSW_MAIN==22035)
				{
				tree_up(iK_net3,0,0,0);
		     	ret(1000);
				}
			else
				{
				tree_up(iK_net,0,0,0);	
				ret(1000);		
				}
			}
		else if((NUMBAT)&&(sub_ind==1))
			{
			tree_up(iK_bat_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==(1+(NUMBAT!=0))))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMINV)&&(sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_inv_sel,0,0,1);	
			ret(1000);
			}		

		else if((sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0))))
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
          
         	else if((NUMDT)&&(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0))))
			{
			tree_up(iK_t_ext_6U,0,0,0);	
			ret(1000);			
			}				
          else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0)))
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}			
	}







else if(ind==iK_220_380)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0));
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0));
		}
	else if(but==butD_)
		{
		sub_ind=3+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0);
		}
   	else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0)))
			{
			if((but==butR)||(but==butR_)||(but==butE)||(but==butE_))
				{
				if(RELE_VENT_LOGIC==0)RELE_VENT_LOGIC=1;
				else if(RELE_VENT_LOGIC==1)RELE_VENT_LOGIC=2;
				else RELE_VENT_LOGIC=0;
				lc640_write_int(EE_RELE_VENT_LOGIC,RELE_VENT_LOGIC);
				}
			else if((but==butL)||(but==butL_))
				{
				if(RELE_VENT_LOGIC==0)RELE_VENT_LOGIC=2;
				else if(RELE_VENT_LOGIC==2)RELE_VENT_LOGIC=1;
				else RELE_VENT_LOGIC=0;
				lc640_write_int(EE_RELE_VENT_LOGIC,RELE_VENT_LOGIC);
				}			
            }
										
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			if(AUSW_MAIN==22035)
				{
				tree_up(iK_net3,0,0,0);
		     	ret(1000);
				}
			else
				{
				tree_up(iK_net,0,0,0);	
				ret(1000);		
				}
			}
		else if((NUMBAT)&&(sub_ind==1))
			{
			tree_up(iK_bat_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==(1+(NUMBAT!=0))))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMINV)&&(sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_inv_sel,0,0,1);	
			ret(1000);
			}		

		else if((sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0))))
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
          
         	else if((NUMDT)&&(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0))))
			{
			tree_up(iK_t_ext_6U,0,0,0);	
			ret(1000);			
			}
 						
          else if(sub_ind==(3+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0)))
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}			
	}

else if(ind==iK_net)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,1);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,1);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=1;
		}				
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(EE_KUNET);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			//temp_SS=lc640_read_int(EE_KUNET);
			temp_SS+=10;
			//lc640_write_int(EE_KUNET,temp_SS);
			}	
		else if(but==butL)
			{
			//temp_SS=lc640_read_int(EE_KUNET);
			temp_SS--;
			//lc640_write_int(EE_KUNET,temp_SS);
			}
		else if(but==butL_)
			{
			//temp_SS=lc640_read_int(EE_KUNET);
			temp_SS-=10;
			//lc640_write_int(EE_KUNET,temp_SS);
			}				
		speed=1;
		gran(&temp_SS,10,800);
		lc640_write_int(EE_KUNET,temp_SS);
					
		}
	else if(sub_ind==1)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_net3)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=3;
		}				
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(EE_KUNETA);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,150,800);
		lc640_write_int(EE_KUNETA,temp_SS);
		}

	else if(sub_ind==1)
		{
		temp_SS=lc640_read_int(EE_KUNETB);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,150,800);
		lc640_write_int(EE_KUNETB,temp_SS);
		}

	else if(sub_ind==2)
		{
		temp_SS=lc640_read_int(EE_KUNETC);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,150,800);
		lc640_write_int(EE_KUNETC,temp_SS);
		}

	else if(sub_ind==3)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_power_net)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=3;
		}				
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(EE_KUNET_EXT0);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		lc640_write_int(EE_KUNET_EXT0,temp_SS);
					
		}
	else if(sub_ind==1)
		{
		temp_SS=lc640_read_int(EE_KUNET_EXT1);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		lc640_write_int(EE_KUNET_EXT1,temp_SS);
					
		}



	else if(sub_ind==2)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_power_net3)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,6);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,6);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=6;
		}
						
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(EE_KVV0_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		lc640_write_int(EE_KVV0_EB2,temp_SS);
		}

	else if(sub_ind==1)
		{
		temp_SS=lc640_read_int(EE_KVV1_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		lc640_write_int(EE_KVV1_EB2,temp_SS);
		}

	else if(sub_ind==2)
		{
		temp_SS=lc640_read_int(EE_KVV2_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		lc640_write_int(EE_KVV2_EB2,temp_SS);
		}

	else if(sub_ind==3)
		{
		temp_SS=lc640_read_int(EE_KPES0_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		lc640_write_int(EE_KPES0_EB2,temp_SS);
		}

	else if(sub_ind==4)
		{
		temp_SS=lc640_read_int(EE_KPES1_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		lc640_write_int(EE_KPES1_EB2,temp_SS);
		}

	else if(sub_ind==5)
		{
		temp_SS=lc640_read_int(EE_KPES2_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		lc640_write_int(EE_KPES2_EB2,temp_SS);
		}




	else if(sub_ind==6)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_bat_sel)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMBAT);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,NUMBAT);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=1+NUMBAT;
		}	
	else if((but==butE)&&(NUMBAT)&&(BAT_IS_ON[0]==bisON)&&(sub_ind==0))
		{
		#ifdef UKU_6U
		tree_up(iK_bat_simple,0,0,0);
		#else

		#ifdef UKU_220_V2
		tree_up(iK_bat_simple,0,0,0);
		#else
		
	/*	#ifdef UKU_220 
		tree_up(iK_bat_simple,0,0,0);
		#else*/
		tree_up(iK_bat,0,0,0);	
		#endif
		#endif
		//mess_send(MESS_SRC_CONTROL,0xFFFF,0,10);
     	//mess_send(MESS_BAT_CONTROL,0xFFFF&(~(1<<sub_ind1)),1<<(sub_ind1),10);

		ret(1000);
		}	
	else if((but==butE)&&(NUMBAT)&&(BAT_IS_ON[1]==bisON)&&(sub_ind==((BAT_IS_ON[0]==bisON))))
		{
		#ifdef UKU_6U
		tree_up(iK_bat_simple,0,0,1);
		#else
		tree_up(iK_bat,0,0,1);	
		#endif
		
		//mess_send(MESS_SRC_CONTROL,0xFFFF,0,10);
     	//mess_send(MESS_BAT_CONTROL,0xFFFF&(~(1<<sub_ind1)),1<<(sub_ind1),10);
     		
		ret(1000);
		}	
	else if(sub_ind==(NUMBAT))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}				
	}

else if(ind==iK_bat)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if((sub_ind==1)||(sub_ind==2))sub_ind=3;
		else if((sub_ind==4)||(sub_ind==5))sub_ind=6;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=9;
          else if((sub_ind==10)||(sub_ind==11))sub_ind=12;
		gran_char(&sub_ind,0,12);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		if((sub_ind==1)||(sub_ind==2)) sub_ind=0;
	     else if((sub_ind==4)||(sub_ind==5))sub_ind=3;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=6;
		else if((sub_ind==10)||(sub_ind==11))sub_ind=9;
          gran_char(&sub_ind,0,12);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=9;
		}			
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(ADR_KUBAT[sub_ind1]);
	     if(but==butR)
	     	{
     	     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}

		gran(&temp_SS,500,650);
		lc640_write_int(ADR_KUBAT[sub_ind1],temp_SS);					
		speed=1;			
		}
					
	else if(sub_ind==3)
		{
		if(but==butE)
		     {
		     if(phase==0)
		          {
		          lc640_write_int(ADR_KI0BAT[sub_ind1],ad7705_buff_[sub_ind1]);
		          phase=1;
		          }
		     }	
		else
			{
			temp_SS=lc640_read_int(ADR_KI1BAT[sub_ind1]);
			if(but==butR)temp_SS++;
			else if(but==butR_)temp_SS+=2;
			else if(but==butL)temp_SS--;
			else if(but==butL_)temp_SS-=2;
						
			gran(&temp_SS,200,4000);
			lc640_write_int(ADR_KI1BAT[sub_ind1],temp_SS);
			phase=1;
			}
				
		speed=1;			
		}					
	else if(sub_ind==6)
		{
		temp_SS=lc640_read_int(ADR_KTBAT[sub_ind1]);
		if(but==butR)temp_SS++;
		else if(but==butR_)temp_SS+=3;
		else if(but==butL)temp_SS--;
		else if(but==butL_)temp_SS-=3;
		gran(&temp_SS,1900,3000);
		lc640_write_int(ADR_KTBAT[sub_ind1],temp_SS);				
		speed=1;			
		}
	else if(sub_ind==9)
		{
		temp_SS=lc640_read_int(ADR_KUBATM[sub_ind1]);
	     if(but==butR)
	     	{
     	     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}

		gran(&temp_SS,500,650);
		lc640_write_int(ADR_KUBATM[sub_ind1],temp_SS);					
		speed=1;			
		}          	
	else if(sub_ind==12)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}		

		


else if(ind==iK_bat_simple)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if((sub_ind==1)||(sub_ind==2))sub_ind=3;
		else if((sub_ind==4)||(sub_ind==5))sub_ind=6;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=9;
		gran_char(&sub_ind,0,9);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		if((sub_ind==1)||(sub_ind==2)) sub_ind=0;
	     else if((sub_ind==4)||(sub_ind==5))sub_ind=3;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=6;
          gran_char(&sub_ind,0,9);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=9;
		}			
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(ADR_KUBAT[sub_ind1]);
	     if(but==butR)
	     	{
     	     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}

		gran(&temp_SS,500,650);
		lc640_write_int(ADR_KUBAT[sub_ind1],temp_SS);					
		speed=1;			
		}
					
	else if(sub_ind==3)
		{
		if(but==butE)
		     {
		     if(phase==0)
		          {
		          lc640_write_int(ADR_KI0BAT[sub_ind1],ad7705_buff_[sub_ind1]);
		          phase=1;
		          }
		     }	
		else
			{
			temp_SS=lc640_read_int(ADR_KI1BAT[sub_ind1]);
			if(but==butR)temp_SS++;
			else if(but==butR_)temp_SS+=2;
			else if(but==butL)temp_SS--;
			else if(but==butL_)temp_SS-=2;
						
			gran(&temp_SS,200,4000);
			lc640_write_int(ADR_KI1BAT[sub_ind1],temp_SS);
			phase=1;
			}
				
		speed=1;			
		}					
	else if(sub_ind==6)
		{
		temp_SS=lc640_read_int(ADR_KTBAT[sub_ind1]);
		if(but==butR)temp_SS++;
		else if(but==butR_)temp_SS+=3;
		else if(but==butL)temp_SS--;
		else if(but==butL_)temp_SS-=3;
		gran(&temp_SS,1900,3000);
		lc640_write_int(ADR_KTBAT[sub_ind1],temp_SS);				
		speed=1;			
		}
 	
	else if(sub_ind==9)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}		

else if(ind==iK_byps_sel)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMBYPASS);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,NUMBYPASS);
		}
	else if(but==butD_)
		{
		sub_ind=1+NUMBYPASS;
		}	
	else if((but==butE)&&(NUMBYPASS)&&(sub_ind<NUMBYPASS))
		{
		tree_up(iK_byps,0,0,sub_ind);	

		ret(1000);
		}	
	else if(sub_ind==NUMBYPASS)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}				
	}

else if(ind==iK_bps_sel)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMIST);
		phase=0;
		mcp2515_transmit(sub_ind,sub_ind,CMND,ALRM_RES,0,0,0,0);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,NUMIST);
		phase=0;
		mcp2515_transmit(sub_ind,sub_ind,CMND,ALRM_RES,0,0,0,0);
		}
	else if(but==butD_)
		{
		sub_ind=1+NUMIST;
		}	
	else if((but==butE)&&(NUMIST)&&(sub_ind<NUMIST))
		{
		tree_up(iK_bps,0,0,sub_ind);	
		
		mcp2515_transmit(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		//mess_send(MESS_SRC_CONTROL,0xFFFF,0,10);
     	//mess_send(MESS_BAT_CONTROL,0xFFFF&(~(1<<sub_ind1)),1<<(sub_ind1),10);

		ret(1000);
		}	
	else if(sub_ind==(NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}				
	}

else if(ind==iK_bps)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if((sub_ind==1)||(sub_ind==2))sub_ind=3;
		else if((sub_ind==4)||(sub_ind==5))sub_ind=6;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=9;
		else if((sub_ind==10)||(sub_ind==11))sub_ind=12;
		else if((sub_ind==13)||(sub_ind==14))sub_ind=15;
		gran_char(&sub_ind,0,15);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		if((sub_ind==1)||(sub_ind==2))sub_ind=0;
		else if((sub_ind==4)||(sub_ind==5))sub_ind=3;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=6;
		else if((sub_ind==10)||(sub_ind==11))sub_ind=9;
		else if((sub_ind==13)||(sub_ind==14))sub_ind=12;		
		gran_char(&sub_ind,0,15);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=15;
		}
	else if (sub_ind == 0)
		{
		if(but==butLR) mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(0*16)+1,(0*16)+1,0,0,0);
	     else if(but==butR) mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(0*16)+2,(0*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(0*16)+3,(0*16)+3,0,0,0);
    		else if(but==butL) mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(0*16)+4,(0*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(0*16)+5,(0*16)+5,0,0,0);
		speed=1;
		}	
		
	else if (sub_ind == 3)
		{
		if(but==butLR) mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(1*16)+1,(1*16)+1,0,0,0);
	     else if(but==butR) mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(1*16)+2,(1*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(1*16)+3,(1*16)+3,0,0,0);
    		else if(but==butL) mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(1*16)+4,(1*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(1*16)+5,(1*16)+5,0,0,0);
		speed=1;
		}		
		
	else if (sub_ind == 6)
		{
		temp_SS=lc640_read_int(EE_U_AVT);
		if(but==butR)temp_SS++;
		else if(but==butR_)temp_SS+=2;
		else if(but==butL)temp_SS--;
		else if(but==butL_)temp_SS-=2;
		else if(but==butE_)mcp2515_transmit(sub_ind1,sub_ind1,CMND,0xee,0xee,0,0,0);   
		
		#ifdef UKU206_220				
		gran(&temp_SS,2000,3000);
		#endif

		#ifdef UKU206_24
		gran(&temp_SS,200,300);
		#endif

		#ifdef UKU320
		gran(&temp_SS,400,800);
		#endif

		#ifdef UKU320_24
		gran(&temp_SS,200,300);
		#endif

		#ifdef UKU320_F
		gran(&temp_SS,400,800);
		#endif		
		lc640_write_int(EE_U_AVT,temp_SS);
		
		speed=1;
		}	
		
	else if (sub_ind == 9)
		{
		if(but==butE)
			{
			mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(2*16)+1,(2*16)+1,0,0,0);
			phase=1;
			}
	     else if(but==butR) mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(2*16)+2,(2*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(2*16)+3,(2*16)+3,0,0,0);
    		else if(but==butL) mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(2*16)+4,(2*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(2*16)+5,(2*16)+5,0,0,0);
		speed=1;
		}		
		
	else if (sub_ind == 12)
		{
		if(but==butR) mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(3*16)+2,(3*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(3*16)+3,(3*16)+3,0,0,0);
    		else if(but==butL) mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(3*16)+4,(3*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(sub_ind1,sub_ind1,KLBR,(3*16)+5,(3*16)+5,0,0,0);
		speed=1;
		}								
			
	else if(sub_ind==0)
		{
		if(phase==0)
		     {
		     if(but==butE)
		          {
		          #if(UKU_VERSION==300)
		          if(sub_ind1==0)temp_SS=adc_buff_[3];
		          if(sub_ind1==1)temp_SS=adc_buff_[2];
                    #else
                    if(sub_ind1==0)temp_SS=adc_buff_[2];
		          if(sub_ind1==1)temp_SS=adc_buff_[3];
		          #endif
		          //lc640_write_int(ptr_ki0_src[sub_ind1],temp_SS);
		     	phase=1;
		          }
		     else phase=1;     
		     }
		else if(phase==2)
		     {
		     if(but==butR)
		     	{
		     	//temp_SS=lc640_read_int(ptr_ki_src[sub_ind1]);
		     	temp_SS++;
		     	//lc640_write_int(ptr_ki_src[sub_ind1],temp_SS);
	     		}
	     	else if(but==butR_)
	     		{
	     		//temp_SS=lc640_read_int(ptr_ki_src[sub_ind1]);
	     		temp_SS+=2;
	     		//lc640_write_int(ptr_ki_src[sub_ind1],temp_SS);
	     		}	
	     	else if(but==butL)
	     		{
	     		//temp_SS=lc640_read_int(ptr_ki_src[sub_ind1]);
	     		temp_SS--;
	     		//lc640_write_int(ptr_ki_src[sub_ind1],temp_SS);
	     		}
	     	else if(but==butL_)
	     		{
	     		//temp_SS=lc640_read_int(ptr_ki_src[sub_ind1]);
	     		temp_SS-=2;
	     		//lc640_write_int(ptr_ki_src[sub_ind1],temp_SS);
	     		}				
	     	speed=1;			
	     	}
	     }	
					
	else if(sub_ind==3)
		{
	     if(but==butR)
			{
			//temp_SS=lc640_read_int(ptr_ku_src[sub_ind1]);
			temp_SS++;
			//lc640_write_int(ptr_ku_src[sub_ind1],temp_SS);
			}
		else if(but==butR_)
			{
			//temp_SS=lc640_read_int(ptr_ku_src[sub_ind1]);
			temp_SS+=2;
			//lc640_write_int(ptr_ku_src[sub_ind1],temp_SS);
			}	
		else if(but==butL)
			{
			//temp_SS=lc640_read_int(ptr_ku_src[sub_ind1]);
			temp_SS--;
			//lc640_write_int(ptr_ku_src[sub_ind1],temp_SS);
			}
		else if(but==butL_)
			{
			//temp_SS=lc640_read_int(ptr_ku_src[sub_ind1]);
			temp_SS-=2;
			//lc640_write_int(ptr_ku_src[sub_ind1],temp_SS);
			}				
		speed=1;			
		}					
	else if(sub_ind==6)
		{
		if(but==butR)
			{
			//temp_SS=lc640_read_int(ptr_kt_src[sub_ind1]);
			temp_SS++;
			//lc640_write_int(ptr_kt_src[sub_ind1],temp_SS);
			}
		else if(but==butR_)
			{
			//temp_SS=lc640_read_int(ptr_kt_src[sub_ind1]);
			temp_SS+=3;
			//lc640_write_int(ptr_kt_src[sub_ind1],temp_SS);
			}	
		else if(but==butL)
			{
			//temp_SS=lc640_read_int(ptr_kt_src[sub_ind1]);
			temp_SS--;
			//lc640_write_int(ptr_kt_src[sub_ind1],temp_SS);
			}
		else if(but==butL_)
			{
			//temp_SS=lc640_read_int(ptr_kt_src[sub_ind1]);
			temp_SS-=3;
			//lc640_write_int(ptr_kt_src[sub_ind1],temp_SS);
			}				
		speed=1;			
		}	
	else if(sub_ind==15)
		{
		if(but==butE)
			{
			//a=b[--ptr_ind];
			//sub_ind++;
			tree_down(0,1);
			ret(0);
			}
		}			
	}		

else if(ind==iK_inv_sel)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMINV);
		phase=0;
		mcp2515_transmit((sub_ind+first_inv_slot),(sub_ind+first_inv_slot),CMND,ALRM_RES,0,0,0,0);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,NUMINV);
		phase=0;
		mcp2515_transmit((sub_ind+first_inv_slot),(sub_ind+first_inv_slot),CMND,ALRM_RES,0,0,0,0);
		}
	else if(but==butD_)
		{
		sub_ind=1+NUMINV;
		}	
	else if((but==butE)&&(NUMINV)&&(sub_ind<NUMINV))
		{
		tree_up(iK_inv,0,0,sub_ind);	
		
		mcp2515_transmit(4,4,CMND,ALRM_RES,0,0,0,0);
		//mess_send(MESS_SRC_CONTROL,0xFFFF,0,10);
     	//mess_send(MESS_BAT_CONTROL,0xFFFF&(~(1<<sub_ind1)),1<<(sub_ind1),10);

		ret(1000);
		}	
	else if(sub_ind==(NUMINV))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}				
	}
/*	
else if(ind==iK_makb_sel)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMMAKB);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,NUMMAKB);
		}
	else if(but==butD_)
		{
		sub_ind=NUMMAKB;
		}	
	else if((but==butE)&&(NUMMAKB)&&(sub_ind<NUMMAKB))
		{
		if(makb[sub_ind]._cnt<5)
			{
			tree_up(iK_makb,0,0,sub_ind);
			ret(1000);
			}
		else show_mess(
					"                    ",
	          		"   НЕ ПОДКЛЮЧЕН!!!  ",
	          		"                    ",
	          		"                    ",1000);	
		}	
	else if(sub_ind==(NUMMAKB))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}				
	}

else if(ind==iK_makb)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		//if(sub_ind>7)sub_ind=7;
		//else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		//if(sub_ind<3)sub_ind=3;
		gran_char(&sub_ind,0,simax);
		}
	else if(but==butD_)
		{
		sub_ind=10;
		}
	else if ((sub_ind >= 0) && (sub_ind <= 9))
		{
		if(but==butLR) mcp2515_transmit(sub_ind1,sub_ind1,KLBR_MAKB,		(sub_ind*16)+1,(sub_ind*16)+1,0,0,0);
	     else if(but==butR) mcp2515_transmit(sub_ind1,sub_ind1,KLBR_MAKB,	(sub_ind*16)+2,(sub_ind*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(sub_ind1,sub_ind1,KLBR_MAKB,	(sub_ind*16)+3,(sub_ind*16)+3,0,0,0);
    		else if(but==butL) mcp2515_transmit(sub_ind1,sub_ind1,KLBR_MAKB,	(sub_ind*16)+4,(sub_ind*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(sub_ind1,sub_ind1,KLBR_MAKB,	(sub_ind*16)+5,(sub_ind*16)+5,0,0,0);
		speed=1;
		}	
		
	else if(sub_ind==10)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}
*/
#ifndef GLADKOV											
else if(ind==iK_inv)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if((sub_ind==1)||(sub_ind==2))sub_ind=3;
		else if((sub_ind==4)||(sub_ind==5))sub_ind=6;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=9;
		else if((sub_ind==10)||(sub_ind==11))sub_ind=12;
		else if((sub_ind==13)||(sub_ind==14))sub_ind=15;
		else if((sub_ind==16)||(sub_ind==17))sub_ind=18;
		else if((sub_ind==19)||(sub_ind==20))sub_ind=21;

		gran_char(&sub_ind,0,21);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		if((sub_ind==1)||(sub_ind==2))sub_ind=0;
		else if((sub_ind==4)||(sub_ind==5))sub_ind=3;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=6;
		else if((sub_ind==10)||(sub_ind==11))sub_ind=9;
		else if((sub_ind==13)||(sub_ind==14))sub_ind=12;
		else if((sub_ind==16)||(sub_ind==17))sub_ind=15;
		else if((sub_ind==19)||(sub_ind==20))sub_ind=18;
		gran_char(&sub_ind,0,21);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=9;
		}
	else if (sub_ind == 0)
		{
		if(but==butLR) 	mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(0*16)+1,(0*16)+1,0,0,0);
	    	else if(but==butR) 	mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(0*16)+2,(0*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(0*16)+3,(0*16)+3,0,0,0);
    		else if(but==butL) 	mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(0*16)+4,(0*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(0*16)+5,(0*16)+5,0,0,0);
		speed=1;
		}	
		
	else if (sub_ind == 3)
		{
		if(but==butE)
			{
			mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(2*16)+1,(2*16)+1,0,0,0);
			phase=1;
			}
	    	else if(but==butR) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(2*16)+2,(2*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(2*16)+3,(2*16)+3,0,0,0);
    		else if(but==butL) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(2*16)+4,(2*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(2*16)+5,(2*16)+5,0,0,0);
		speed=1;
		}		
		
	else if (sub_ind == 6)
		{
		if(but==butR) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(3*16)+2,(3*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(3*16)+3,(3*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(3*16)+4,(3*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(3*16)+5,(3*16)+5,0,0,0);
		speed=1;
		}								
			
	else if (sub_ind == 9)
		{
		if(but==butR) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(4*16)+2,(4*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(4*16)+3,(4*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(4*16)+4,(4*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(4*16)+5,(4*16)+5,0,0,0);
		speed=1;
		}
	else if (sub_ind == 12)
		{
		if(but==butR) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(5*16)+2,(5*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(5*16)+3,(5*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(5*16)+4,(5*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(5*16)+5,(5*16)+5,0,0,0);
		speed=1;
		}
	else if (sub_ind == 15)
		{
		if(but==butR) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(6*16)+2,(6*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(6*16)+3,(6*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(6*16)+4,(6*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(6*16)+5,(6*16)+5,0,0,0);
		speed=1;
		}							

	else if (sub_ind == 18)
		{
		if(but==butR) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(7*16)+2,(7*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(7*16)+3,(7*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(7*16)+4,(7*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,(7*16)+5,(7*16)+5,0,0,0);
		speed=1;
		}	
									
	else if(sub_ind==21)
		{
		if(but==butE)
			{
			//a=b[--ptr_ind];
			//sub_ind++;
			tree_down(0,1);
			ret(0);
			}
		}			
	}
#endif
#ifdef GLADKOV
else if(ind==iK_inv)
	{
	char GLADKOV_ADR;
	GLADKOV_ADR=4;
	if(sub_ind1==1)GLADKOV_ADR=21;
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if((sub_ind==1)||(sub_ind==2))sub_ind=3;
		else if((sub_ind==4)||(sub_ind==5))sub_ind=6;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=9;
		else if((sub_ind==10)||(sub_ind==11))sub_ind=12;
		else if((sub_ind==13)||(sub_ind==14))sub_ind=15;
		else if((sub_ind==16)||(sub_ind==17))sub_ind=18;

		gran_char(&sub_ind,0,18);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		if((sub_ind==1)||(sub_ind==2))sub_ind=0;
		else if((sub_ind==4)||(sub_ind==5))sub_ind=3;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=6;
		else if((sub_ind==10)||(sub_ind==11))sub_ind=9;
		else if((sub_ind==13)||(sub_ind==14))sub_ind=12;
		else if((sub_ind==16)||(sub_ind==17))sub_ind=15;
		gran_char(&sub_ind,0,18);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=9;
		}
	else if (sub_ind == 0)
		{
		if(but==butLR) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(0*16)+1,(0*16)+1,0,0,0);
	    else if(but==butR) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(0*16)+2,(0*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(0*16)+3,(0*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(0*16)+4,(0*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(0*16)+5,(0*16)+5,0,0,0);
		speed=1;
		}	
		
	else if (sub_ind == 3)
		{
		if(but==butE)
			{
			mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(2*16)+1,(2*16)+1,0,0,0);
			phase=1;
			}
	    	else if(but==butR) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(2*16)+2,(2*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(2*16)+3,(2*16)+3,0,0,0);
    		else if(but==butL) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(2*16)+4,(2*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(2*16)+5,(2*16)+5,0,0,0);
		speed=1;
		}		
		
	else if (sub_ind == 6)
		{
		if(but==butR) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(3*16)+2,(3*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(3*16)+3,(3*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(3*16)+4,(3*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(3*16)+5,(3*16)+5,0,0,0);
		speed=1;
		}								
			
	else if (sub_ind == 9)
		{
		if(but==butR) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(4*16)+2,(4*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(4*16)+3,(4*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(4*16)+4,(4*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(4*16)+5,(4*16)+5,0,0,0);
		speed=1;
		}
	else if (sub_ind == 12)
		{
		if(but==butR) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(5*16)+2,(5*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(5*16)+3,(5*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(5*16)+4,(5*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(5*16)+5,(5*16)+5,0,0,0);
		speed=1;
		}
	else if (sub_ind == 15)
		{
		if(but==butR) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(6*16)+2,(6*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(6*16)+3,(6*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(6*16)+4,(6*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(GLADKOV_ADR,GLADKOV_ADR,KLBR,(6*16)+5,(6*16)+5,0,0,0);
		speed=1;
		}							
							
	else if(sub_ind==18)
		{
		if(but==butE)
			{
			//a=b[--ptr_ind];
			//sub_ind++;
			tree_down(0,1);
			ret(0);
			}
		}			
	}
#endif

else if(ind==iK_byps)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if((sub_ind==1)||(sub_ind==2))sub_ind=3;
		else if((sub_ind==4)||(sub_ind==5))sub_ind=6;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=9;
		else if((sub_ind==10)||(sub_ind==11))sub_ind=12;
		else if((sub_ind==13)||(sub_ind==14))sub_ind=15;
		else if((sub_ind==16)||(sub_ind==17))sub_ind=18;

		gran_char(&sub_ind,0,18);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		if((sub_ind==1)||(sub_ind==2))sub_ind=0;
		else if((sub_ind==4)||(sub_ind==5))sub_ind=3;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=6;
		else if((sub_ind==10)||(sub_ind==11))sub_ind=9;
		else if((sub_ind==13)||(sub_ind==14))sub_ind=12;
		else if((sub_ind==16)||(sub_ind==17))sub_ind=15;
		gran_char(&sub_ind,0,18);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=9;
		}
	else if (sub_ind == 0)
		{
		if(but==butLR) 	mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,	(0*16)+1,(0*16)+1,0,0,0);
	    else if(but==butR) 	mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,	(0*16)+2,(0*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,	(0*16)+3,(0*16)+3,0,0,0);
    	else if(but==butL) 	mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,	(0*16)+4,(0*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,	(0*16)+5,(0*16)+5,0,0,0);
		speed=1;
		}	
		
	else if (sub_ind == 3)
		{
		if(but==butE)
			{
			mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,(2*16)+1,(2*16)+1,0,0,0);
			phase=1;
			}
		else if(but==butR) mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,		(2*16)+2,(2*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,	(2*16)+3,(2*16)+3,0,0,0);
		else if(but==butL) mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,		(2*16)+4,(2*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,	(2*16)+5,(2*16)+5,0,0,0);
		speed=1;
		}		
		
	else if (sub_ind == 6)
		{
		if(but==butR) mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,			(3*16)+2,(3*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,	(3*16)+3,(3*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,		(3*16)+4,(3*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,	(3*16)+5,(3*16)+5,0,0,0);
		speed=1;
		}								
			
	else if (sub_ind == 9)
		{
		if(but==butR) mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,			(4*16)+2,(4*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,	(4*16)+3,(4*16)+3,0,0,0);
		else if(but==butL) mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,		(4*16)+4,(4*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,	(4*16)+5,(4*16)+5,0,0,0);
		speed=1;
		}
	else if (sub_ind == 12)
		{
		if(but==butR) mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,			(5*16)+2,(5*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,	(5*16)+3,(5*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,		(5*16)+4,(5*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,	(5*16)+5,(5*16)+5,0,0,0);
		speed=1;
		}
	else if (sub_ind == 15)
		{
		if(but==butR) mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,			(6*16)+2,(6*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,	(6*16)+3,(6*16)+3,0,0,0);
		else if(but==butL) mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,		(6*16)+4,(6*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(byps[sub_ind1]._adress,byps[sub_ind1]._adress,KLBR,	(6*16)+5,(6*16)+5,0,0,0);
		speed=1;
		}							
							
	else if(sub_ind==18)
		{
		if(but==butE)
			{
			//a=b[--ptr_ind];
			//sub_ind++;
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_byps_3f)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if((sub_ind==1)||(sub_ind==2))sub_ind=3;
		else if((sub_ind==4)||(sub_ind==5))sub_ind=6;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=9;
		else if((sub_ind==10)||(sub_ind==11))sub_ind=12;
		else if((sub_ind==13)||(sub_ind==14))sub_ind=15;
		else if((sub_ind==16)||(sub_ind==17))sub_ind=18;
		else if((sub_ind==19)||(sub_ind==20))sub_ind=21;
		else if((sub_ind==22)||(sub_ind==23))sub_ind=24;
		else if((sub_ind==25)||(sub_ind==26))sub_ind=27;
		else if((sub_ind==28)||(sub_ind==29))sub_ind=30;
		else if((sub_ind==31)||(sub_ind==32))sub_ind=33;
		else if((sub_ind==34)||(sub_ind==35))sub_ind=36;
		else if((sub_ind==37)||(sub_ind==38))sub_ind=39;
		else if((sub_ind==40)||(sub_ind==41))sub_ind=42;
		else if((sub_ind==43)||(sub_ind==44))sub_ind=45;
		else if((sub_ind==46)||(sub_ind==47))sub_ind=48;
		else if((sub_ind==49)||(sub_ind==50))sub_ind=51;
		else if((sub_ind==52)||(sub_ind==53))sub_ind=54;

		gran_char(&sub_ind,0,54);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		if((sub_ind==1)||(sub_ind==2))sub_ind=0;
		else if((sub_ind==4)||(sub_ind==5))sub_ind=3;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=6;
		else if((sub_ind==10)||(sub_ind==11))sub_ind=9;
		else if((sub_ind==13)||(sub_ind==14))sub_ind=12;
		else if((sub_ind==16)||(sub_ind==17))sub_ind=15;
		else if((sub_ind==19)||(sub_ind==20))sub_ind=18;
		else if((sub_ind==22)||(sub_ind==23))sub_ind=21;
		else if((sub_ind==25)||(sub_ind==26))sub_ind=24;
		else if((sub_ind==28)||(sub_ind==29))sub_ind=27;
		else if((sub_ind==31)||(sub_ind==32))sub_ind=30;
		else if((sub_ind==34)||(sub_ind==35))sub_ind=33;
		else if((sub_ind==37)||(sub_ind==38))sub_ind=36;
		else if((sub_ind==40)||(sub_ind==41))sub_ind=39;
		else if((sub_ind==43)||(sub_ind==44))sub_ind=42;
		else if((sub_ind==46)||(sub_ind==47))sub_ind=45;
		else if((sub_ind==49)||(sub_ind==50))sub_ind=48;
		else if((sub_ind==52)||(sub_ind==53))sub_ind=51;
		gran_char(&sub_ind,0,54);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=55;
		}
	else if (sub_ind == 0)
		{
		if(but==butLR) 		mcp2515_transmit(61,61,KLBR,	(0*16)+1,(0*16)+1,0,0,0);
	    else if(but==butR) 	mcp2515_transmit(61,61,KLBR,	(0*16)+2,(0*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(61,61,KLBR,	(0*16)+3,(0*16)+3,0,0,0);
    	else if(but==butL) 	mcp2515_transmit(61,61,KLBR,	(0*16)+4,(0*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(61,61,KLBR,	(0*16)+5,(0*16)+5,0,0,0);
		speed=1;
		}	
	else if (sub_ind == 3)
		{
		if(but==butLR) 		mcp2515_transmit(62,62,KLBR,	(0*16)+1,(0*16)+1,0,0,0);
	    else if(but==butR) 	mcp2515_transmit(62,62,KLBR,	(0*16)+2,(0*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(62,62,KLBR,	(0*16)+3,(0*16)+3,0,0,0);
    	else if(but==butL) 	mcp2515_transmit(62,62,KLBR,	(0*16)+4,(0*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(62,62,KLBR,	(0*16)+5,(0*16)+5,0,0,0);
		speed=1;
		}
	else if (sub_ind == 6)
		{
		if(but==butLR) 		mcp2515_transmit(63,63,KLBR,	(0*16)+1,(0*16)+1,0,0,0);
	    else if(but==butR) 	mcp2515_transmit(63,63,KLBR,	(0*16)+2,(0*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(63,63,KLBR,	(0*16)+3,(0*16)+3,0,0,0);
    	else if(but==butL) 	mcp2515_transmit(63,63,KLBR,	(0*16)+4,(0*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(63,63,KLBR,	(0*16)+5,(0*16)+5,0,0,0);
		speed=1;
		}				
	else if (sub_ind == 9)
		{
		if(but==butE)
			{
			mcp2515_transmit(61,61,KLBR,(2*16)+1,(2*16)+1,0,0,0);
			phase=1;
			}
		else if(but==butR) mcp2515_transmit(61,61,KLBR,		(2*16)+2,(2*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(61,61,KLBR,	(2*16)+3,(2*16)+3,0,0,0);
		else if(but==butL) mcp2515_transmit(61,61,KLBR,		(2*16)+4,(2*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(61,61,KLBR,	(2*16)+5,(2*16)+5,0,0,0);
		speed=1;
		}		

	else if (sub_ind == 12)
		{
		if(but==butE)
			{
			mcp2515_transmit(62,62,KLBR,(2*16)+1,(2*16)+1,0,0,0);
			phase=1;
			}
		else if(but==butR) mcp2515_transmit(62,62,KLBR,		(2*16)+2,(2*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(62,62,KLBR,	(2*16)+3,(2*16)+3,0,0,0);
		else if(but==butL) mcp2515_transmit(62,62,KLBR,		(2*16)+4,(2*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(62,62,KLBR,	(2*16)+5,(2*16)+5,0,0,0);
		speed=1;
		}
			
	else if (sub_ind == 15)
		{
		if(but==butE)
			{
			mcp2515_transmit(63,63,KLBR,(2*16)+1,(2*16)+1,0,0,0);
			phase=1;
			}
		else if(but==butR) mcp2515_transmit(63,63,KLBR,		(2*16)+2,(2*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(63,63,KLBR,	(2*16)+3,(2*16)+3,0,0,0);
		else if(but==butL) mcp2515_transmit(63,63,KLBR,		(2*16)+4,(2*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(63,63,KLBR,	(2*16)+5,(2*16)+5,0,0,0);
		speed=1;
		}
							
	else if (sub_ind == 18)
		{
		if(but==butR) mcp2515_transmit(61,61,KLBR,			(3*16)+2,(3*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(61,61,KLBR,	(3*16)+3,(3*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(61,61,KLBR,		(3*16)+4,(3*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(61,61,KLBR,	(3*16)+5,(3*16)+5,0,0,0);
		speed=1;
		}
										
	else if (sub_ind == 21)
		{
		if(but==butR) mcp2515_transmit(62,62,KLBR,			(3*16)+2,(3*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(62,62,KLBR,	(3*16)+3,(3*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(62,62,KLBR,		(3*16)+4,(3*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(62,62,KLBR,	(3*16)+5,(3*16)+5,0,0,0);
		speed=1;
		}

	else if (sub_ind == 24)
		{
		if(but==butR) mcp2515_transmit(63,63,KLBR,			(3*16)+2,(3*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(63,63,KLBR,	(3*16)+3,(3*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(63,63,KLBR,		(3*16)+4,(3*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(63,63,KLBR,	(3*16)+5,(3*16)+5,0,0,0);
		speed=1;
		}
							
	else if (sub_ind == 27)
		{
		if(but==butR) mcp2515_transmit(61,61,KLBR,			(4*16)+2,(4*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(61,61,KLBR,	(4*16)+3,(4*16)+3,0,0,0);
		else if(but==butL) mcp2515_transmit(61,61,KLBR,		(4*16)+4,(4*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(61,61,KLBR,	(4*16)+5,(4*16)+5,0,0,0);
		speed=1;
		}
							
	else if (sub_ind == 30)
		{
		if(but==butR) mcp2515_transmit(62,62,KLBR,			(4*16)+2,(4*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(62,62,KLBR,	(4*16)+3,(4*16)+3,0,0,0);
		else if(but==butL) mcp2515_transmit(62,62,KLBR,		(4*16)+4,(4*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(62,62,KLBR,	(4*16)+5,(4*16)+5,0,0,0);
		speed=1;
		}
							
	else if (sub_ind == 33)
		{
		if(but==butR) mcp2515_transmit(63,63,KLBR,			(4*16)+2,(4*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(63,63,KLBR,	(4*16)+3,(4*16)+3,0,0,0);
		else if(but==butL) mcp2515_transmit(63,63,KLBR,		(4*16)+4,(4*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(63,63,KLBR,	(4*16)+5,(4*16)+5,0,0,0);
		speed=1;
		}

	else if (sub_ind == 36)
		{
		if(but==butR) mcp2515_transmit(61,61,KLBR,			(5*16)+2,(5*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(61,61,KLBR,	(5*16)+3,(5*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(61,61,KLBR,		(5*16)+4,(5*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(61,61,KLBR,	(5*16)+5,(5*16)+5,0,0,0);
		speed=1;
		}

	else if (sub_ind == 39)
		{
		if(but==butR) mcp2515_transmit(62,62,KLBR,			(5*16)+2,(5*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(62,62,KLBR,	(5*16)+3,(5*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(62,62,KLBR,		(5*16)+4,(5*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(62,62,KLBR,	(5*16)+5,(5*16)+5,0,0,0);
		speed=1;
		}

	else if (sub_ind == 42)
		{
		if(but==butR) mcp2515_transmit(63,63,KLBR,			(5*16)+2,(5*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(63,63,KLBR,	(5*16)+3,(5*16)+3,0,0,0);
    	else if(but==butL) mcp2515_transmit(63,63,KLBR,		(5*16)+4,(5*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(63,63,KLBR,	(5*16)+5,(5*16)+5,0,0,0);
		speed=1;
		}

	else if (sub_ind == 45)
		{
		if(but==butR) mcp2515_transmit(61,61,KLBR,			(6*16)+2,(6*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(61,61,KLBR,	(6*16)+3,(6*16)+3,0,0,0);
		else if(but==butL) mcp2515_transmit(61,61,KLBR,		(6*16)+4,(6*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(61,61,KLBR,	(6*16)+5,(6*16)+5,0,0,0);
		speed=1;
		}
									
	else if (sub_ind == 48)
		{
		if(but==butR) mcp2515_transmit(62,62,KLBR,			(6*16)+2,(6*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(62,62,KLBR,	(6*16)+3,(6*16)+3,0,0,0);
		else if(but==butL) mcp2515_transmit(62,62,KLBR,		(6*16)+4,(6*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(62,62,KLBR,	(6*16)+5,(6*16)+5,0,0,0);
		speed=1;
		}							

	else if (sub_ind == 51)
		{
		if(but==butR) mcp2515_transmit(63,63,KLBR,			(6*16)+2,(6*16)+2,0,0,0);
		else if(but==butR_)	mcp2515_transmit(63,63,KLBR,	(6*16)+3,(6*16)+3,0,0,0);
		else if(but==butL) mcp2515_transmit(63,63,KLBR,		(6*16)+4,(6*16)+4,0,0,0); 
		else if(but==butL_) mcp2515_transmit(63,63,KLBR,	(6*16)+5,(6*16)+5,0,0,0);
		speed=1;
		}							

							
	else if(sub_ind==54)
		{
		if(but==butE)
			{
			//a=b[--ptr_ind];
			//sub_ind++;
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_load)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind=1;
		}
	else if(but==butU)
		{
		sub_ind=0;
		}
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(EE_KULOAD);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
		#ifdef UKU_220
	    gran(&temp_SS,300,2000);
		#else 
		#ifdef UKU_220_IPS_TERMOKOMPENSAT
	    gran(&temp_SS,300,2000);
	    #else 
		gran(&temp_SS,100,5000);
		#endif
		#endif
		lc640_write_int(EE_KULOAD,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==1)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_t_ext)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3);
		}
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(KT_EXT0);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		lc640_write_int(KT_EXT0,temp_SS);					
		speed=1;	
					
		}

	else if(sub_ind==1)
		{
		temp_SS=lc640_read_int(KT_EXT1);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		lc640_write_int(KT_EXT1,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==2)
		{
		temp_SS=lc640_read_int(KT_EXT2);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		lc640_write_int(KT_EXT2,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==3)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_t_ext_6U)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMDT);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,NUMDT);
		}
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(KT_EXT0);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		lc640_write_int(KT_EXT0,temp_SS);					
		speed=1;	
					
		}

	else if(sub_ind==1)
		{
		temp_SS=lc640_read_int(KT_EXT1);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		lc640_write_int(KT_EXT1,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==2)
		{
		temp_SS=lc640_read_int(KT_EXT2);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		lc640_write_int(KT_EXT2,temp_SS);					
		speed=1;	
					
		}
 	if(sub_ind==NUMDT)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}
			
else if(ind==iBatLog)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,6);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,6);
		}
	else if(but==butD_)
		{
		sub_ind=6;
		}				
	else if((but==butL)&&((sub_ind==0)||(sub_ind==3)||(sub_ind==4)))
		{
		tree_down(0,0);
		}		
	else if(sub_ind==0)
	     {
	     if(but==butE)
	          {
	          //b[ptr_ind++]=a;
	          //if(BAT_IS_ON[sub_ind1]==bisON) ind=iPrl_bat_in_out;
	          //else 
	               //{
	               //ind=iPdp1;
	               //ret_ind(iPrl_bat_in_out,0,10);
	               //}
	          tree_up(iPrl_bat_in_out,0,0,sub_ind1);
	          if(BAT_IS_ON[sub_ind1]!=bisON) show_mess("  Введение батареи  ",
	          								 "    уничтожит все   ",
	          								 "   предшествующие   ",
	          								 "      данные!!!     ",4000);     
	          parol_init();
	          }
	     }
	else if(sub_ind==1)
	     {
	     if(but==butR)BAT_C_NOM[sub_ind1]++;
	     else if(but==butR_)BAT_C_NOM[sub_ind1]+=10;
	     else if(but==butL)BAT_C_NOM[sub_ind1]--;
	     else if(but==butL_)BAT_C_NOM[sub_ind1]-=10;
	     gran(&BAT_C_NOM[sub_ind1],0,2000);
	     lc640_write_int(ADR_EE_BAT_C_NOM[sub_ind1],BAT_C_NOM[sub_ind1]);
	     speed=1;
	     }		     
		
	else if(sub_ind==3)
		{
		if(but==butE)
			{ 
               cap=0;
			deep=lc640_read_int(CNT_EVENT_LOG);
			ptr=lc640_read_int(PTR_EVENT_LOG);
			
			if(deep>63)
				{
				deep=0;
			     ptr=0;
			     }
			     			
			for(i=0;i<deep;i++)
				{
				lc640_read_long_ptr(EVENT_LOG+(32*ptr),av_head);
				
				if((av_head[0]=='B')&&(av_head[1]==sub_ind1)&&(av_head[2]=='K')) 	//ищем записи батарейных событий 'K'(контроли емкости)
					{
					cap++;
					content[cap-1]=ptr;
					}
					
				ptr=ptr_carry(ptr,64,-1); 
	
				} 
				
			tree_up(iBatLogKe,0,0,sub_ind1);
			
			av_j_si_max=cap;
			if(av_j_si_max>63)av_j_si_max=0;
			} 
		}




	else if(sub_ind==4)
		{
		if(but==butE)
			{ 
               cap=0;
			deep=lc640_read_int(CNT_EVENT_LOG);
			ptr=lc640_read_int(PTR_EVENT_LOG);
			
			if(deep>63)
				{
				deep=0;
			     ptr=0;
			     }
			     
			for(i=0;i<deep;i++)
				{
				lc640_read_long_ptr(EVENT_LOG+(32*ptr),av_head);
				
				if((av_head[0]=='B')/*&&(av_head[1]==sub_ind1)*/&&(av_head[2]=='Z')) 	//ищем записи батарейных событий 'z'(выравнивающий заряд)
					{
					cap++;
					content[cap-1]=ptr;
					}
					
				ptr=ptr_carry(ptr,64,-1); 
				} 
				
			tree_up(iBatLogVz,0,0,sub_ind1);   
			
			av_j_si_max=cap;
			if(av_j_si_max>63)av_j_si_max=0;
			} 
		else if(but==butR)
			{
			//vz_mem_hndl(0,5);
			//vz_mem_hndl(1,6);	       
			} 
		}

	else if(sub_ind==5)
		{
		if(but==butE)
			{ 
               cap=0;
			deep=lc640_read_int(CNT_EVENT_LOG);
			ptr=lc640_read_int(PTR_EVENT_LOG);

			if(deep>63)
				{
				deep=0;
			     ptr=0;
			     }
			
			//out_usart0 (8,0x11,*((char*)&deep),*(((char*)&deep)+1),*((char*)&ptr),*(((char*)&ptr)+1),cap,content[cap-1],i,0);
			
			for(i=0;i<deep;i++)
				{
				lc640_read_long_ptr(EVENT_LOG+(32*ptr),av_head);
				
				if((av_head[0]=='B')&&(av_head[1]==sub_ind1)&&(av_head[2]=='W')) 	//ищем записи батарейных событий 'W'(разряды)
					{
					cap++;
					content[cap-1]=ptr;
					}
					
		   	/*   	out_usart0 (8,0x22,*((char*)&deep),*(((char*)&deep)+1),*((char*)&ptr),*(((char*)&ptr)+1),cap,content[cap-1],i,0); 
				delay_ms(100);
				PORTC.7=!PORTC.7;
				#asm("wdr"); 	*/
				ptr=ptr_carry(ptr,64,-1); 
				} 
				
			/*ind=iJ_bat_wrk_sel;
			sub_ind=0;*/

			tree_up(iBatLogWrk,0,0,sub_ind1);
			
			av_j_si_max=cap;
			if(av_j_si_max>63)av_j_si_max=0;
			} 
		else if(but==butR)
			{
			//vz_mem_hndl(0,5);
			//vz_mem_hndl(1,6);	       
			} 
		}		
		 	         	
     else if(sub_ind==6)
	     {
	     if(but==butE)
	          {
			if(BAT_IS_ON[sub_ind1]!=bisON)tree_down(0,-4);
	          else tree_down(0,0);
	          }
	     }		     
		
	} 

else if(ind==iBatLogVz)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butE)
		{
		if(sub_ind==av_j_si_max)
			{
			tree_down(0,0);
			}
		} 
	else if(but==butL)
		{
		tree_down(0,0);
		}		
    //	else if(but==butR) vz_mem_hndl(sub_ind1,_sec);
	
		
	}

else if(ind==iBatLogKe)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butE)
		{
		if(sub_ind==av_j_si_max)
			{
			tree_down(0,0);
			}
		} 
	else if(but==butL)
		{
		tree_down(0,0);
		}		
    //	else if(but==butR) ke_mem_hndl(sub_ind1,_sec);		
	}

else if(ind==iBatLogWrk)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butE)
		{
		if(sub_ind==av_j_si_max)
			{
			tree_down(0,0);
			}
		else if(sub_ind<=av_j_si_max)
			{
			//ind=iWrk;
			//sub_ind2=content[sub_ind];
			index_set=0;
			//sub_ind=0;
			}	
		} 
	else if(but==butL)
		{
		tree_down(0,0);
		}		
	else if(but==butR)
		{
	    //	wrk_mem_hndl(sub_ind1);

		} 
	}

else if(ind==iAv_view)
	{
	if(but==butE)
		{
		avar_ind_stat&=~(1L<<sub_ind);
		if(avar_ind_stat)
			{
			while(!(avar_ind_stat&(1<<sub_ind)))
				{
				sub_ind++;
				if(sub_ind>=32)
					{
					tree_down(0,0);
					avar_ind_stat=0;
					}
				}
		 	}
	 	else 
			{
			tree_down(0,0);
			avar_ind_stat=0;
			}
		}
 	}

else if(ind==iAvz)
	{
	if(AVZ!=AVZ_OFF)
		{
		if(but==butU)
			{
			sub_ind--;
			if(sub_ind==3)sub_ind--;
			}
		else if(but==butD)
			{
			sub_ind++;
			if(sub_ind==3)sub_ind++;
			}
		else if(sub_ind==0)
			{
			if(but==butL)
				{
				if(AVZ==AVZ_1)AVZ=AVZ_OFF;
				else if(AVZ==AVZ_2)AVZ=AVZ_1;
				else if(AVZ==AVZ_3)AVZ=AVZ_2; 
				else if(AVZ==AVZ_6)AVZ=AVZ_3;
				else if(AVZ==AVZ_12)AVZ=AVZ_6;			
				else AVZ=AVZ_12;
				}
			else if((but==butR)||(but==butE))
				{
				if(AVZ==AVZ_1)AVZ=AVZ_2;
				else if(AVZ==AVZ_2)AVZ=AVZ_3;
				else if(AVZ==AVZ_3)AVZ=AVZ_6; 
				else if(AVZ==AVZ_6)AVZ=AVZ_12;
				else if(AVZ==AVZ_12)AVZ=AVZ_OFF;			
				else AVZ=AVZ_1;
				} 
			lc640_write_int(EE_AVZ,AVZ);
			if(AVZ!=AVZ_OFF)avz_next_date_hndl();		
			}      
		else if(sub_ind==1)
			{
			if((but==butR)||(but==butR_))
				{
				speed=1;
				AVZ_TIME++;
				} 
			else if((but==butL)||(but==butL_))
				{
				speed=1;
				AVZ_TIME--;
				}			
			gran((signed short*)&AVZ_TIME,1,24);
			lc640_write_int(EE_AVZ_TIME,AVZ_TIME);
			}	
		else if(sub_ind==4)
			{
			if((but==butE))
				{
				ind=iSpc;
				sub_ind=1;
				}	
			}        
		gran_char(&sub_ind,0,4);						               
		} 
	else if(AVZ==AVZ_OFF)
		{
		if(but==butU)
			{
			sub_ind--;
			}
		else if(but==butD)
			{
			sub_ind++;
			}
		else if(sub_ind==0)
			{
			if(but==butL)
				{
				if(AVZ==AVZ_1)AVZ=AVZ_OFF;
				else if(AVZ==AVZ_2)AVZ=AVZ_1;
				else if(AVZ==AVZ_3)AVZ=AVZ_2; 
				else if(AVZ==AVZ_6)AVZ=AVZ_3;
				else if(AVZ==AVZ_12)AVZ=AVZ_6;			
				else AVZ=AVZ_12;
				}
			else if((but==butR)||(but==butE))
				{
				if(AVZ==AVZ_1)AVZ=AVZ_2;
				else if(AVZ==AVZ_2)AVZ=AVZ_3;
				else if(AVZ==AVZ_3)AVZ=AVZ_6; 
				else if(AVZ==AVZ_6)AVZ=AVZ_12;
				else if(AVZ==AVZ_12)AVZ=AVZ_OFF;			
				else AVZ=AVZ_1;
				}   
			lc640_write_int(EE_AVZ,AVZ);
			if(AVZ!=AVZ_OFF)avz_next_date_hndl();		
			}      
		else if(sub_ind==1)
			{
			if((but==butR)||(but==butR_))
				{
				speed=1;
				AVZ_TIME++;
				} 
			else if((but==butL)||(but==butL_))
				{
				speed=1;
				AVZ_TIME--;
				}			
			gran((signed short*)&AVZ_TIME,1,20);
			lc640_write_int(EE_AVZ_TIME,AVZ_TIME);
			}	
		else if(sub_ind==2)
			{
			if((but==butE))
				{
				tree_down(0,0);
				}	
			}        
		gran_char(&sub_ind,0,2);						               
		} 
     }
		
else if(ind==iTst_RSTKM)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,18+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

		if(sub_ind==1)
			{
			sub_ind=2;
			index_set=1;
			}
		if(sub_ind==3)
			{
			sub_ind=4;
			index_set=3;
			}
		if(sub_ind==5)
			{
			sub_ind=6;
			index_set=5;
			}
		if(sub_ind==7)
			{
			sub_ind=8;
			index_set=7;
			}
		if(sub_ind==9)
			{
               sub_ind=10;
			index_set=9;
			}
		if(sub_ind==11)
			{
			sub_ind=12;
			index_set=11;
			}
		if(sub_ind==13)
			{
			sub_ind=14;
			index_set=13;
			}
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,18+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

		if(sub_ind==13)
			{
			sub_ind=12;
			}		
		if(sub_ind==11)
			{
			sub_ind=10;
			}
		if(sub_ind==9)
			{
			sub_ind=8;
			}
		if(sub_ind==7)
			{
			sub_ind=6;
			}
		if(sub_ind==5)
			{
			sub_ind=4;
			}
		if(sub_ind==3)
			{
			sub_ind=2;
			}
		if(sub_ind==1)
			{
			sub_ind=0;
			}
		}

	else if(sub_ind==0)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[0]==tstOFF) tst_state[0]=tst1;
			else if(tst_state[0]==tst1) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		else if(but==butL)
			{
			if(tst_state[0]==tst2) tst_state[0]=tst1;
			else if(tst_state[0]==tstOFF) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		}
	else if(sub_ind==2)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[1]==tstOFF) tst_state[1]=tst1;
			else if(tst_state[1]==tst1) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[1]==tst2) tst_state[1]=tst1;
			else if(tst_state[1]==tstOFF) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		}		

	else if(sub_ind==4)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[9]==tstOFF) tst_state[9]=tst1;
			else if(tst_state[9]==tst1) tst_state[9]=tst2;
			else tst_state[9]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[9]==tst2) tst_state[9]=tst1;
			else if(tst_state[9]==tstOFF) tst_state[9]=tst2;
			else tst_state[9]=tstOFF;
			}
		}
		
	else if(sub_ind==6)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[2]==tstOFF) tst_state[2]=tst1;
			else if(tst_state[2]==tst1) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[2]==tst2) tst_state[2]=tst1;
			else if(tst_state[2]==tstOFF) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		}

	else if(sub_ind==8)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[3]==tstOFF) tst_state[3]=tst1;
			else if(tst_state[3]==tst1) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[3]==tst2) tst_state[3]=tst1;
			else if(tst_state[3]==tstOFF) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		}
	else if(sub_ind==10)
		{
		if((but==butE)||(but==butR))
			{
			tst_state[10]++;
			if((tst_state[10]<(enum_tst_state)1)||(tst_state[10]>(enum_tst_state)20)) tst_state[10]=(enum_tst_state)20;
			}
		else if(but==butL)
			{
			tst_state[10]--;
			if((tst_state[10]<(enum_tst_state)1)||(tst_state[10]>(enum_tst_state)20)) tst_state[10]=(enum_tst_state)0;
			}
		}
	else if(sub_ind==12)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[4]==tstOFF) tst_state[4]=tst1;
			else if(tst_state[4]==tst1) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[4]==tst2) tst_state[4]=tst1;
			else if(tst_state[4]==tstOFF) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		}
	else if(sub_ind==14)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[7]==tstOFF) tst_state[7]=tst1;
			else tst_state[7]=tstOFF;
			}
		}
	else if(sub_ind==15)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[8]==tstOFF) tst_state[8]=tst1;
			else tst_state[8]=tstOFF;
			}
		}
	else if((sub_ind>=16)&&(sub_ind<(16+NUMIST))&&(NUMIST)&&((but==butE)))	
		{
		tree_up(iTst_bps,0,0,sub_ind-16);
		mcp2515_transmit(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		}											
	else if(sub_ind==(16+NUMIST))
		{
		if(but==butE)
			{
			bRESET=1;
			}
	
		}
	else if(sub_ind==(17+NUMIST))
		{
		if(but==butE)
			{
			bRESET_EXT=1;
			}
	
		}			
	else if(sub_ind==(18+NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
	
		}	
	}



else if(ind==iTst_3U)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,14+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

		if(sub_ind==1)
			{
			sub_ind=2;
			index_set=1;
			}
		if(sub_ind==3)
			{
			sub_ind=4;
			index_set=3;
			}
		if(sub_ind==5)
			{
			sub_ind=6;
			index_set=5;
			}
		if(sub_ind==7)
			{
			sub_ind=8;
			index_set=7;
			}
		if(sub_ind==9)
			{
               sub_ind=10;
			//index_set=9;
			}
	/*	if(sub_ind==11)
			{
			sub_ind=12;
			index_set=11;
			}*/
	/*	if(sub_ind==13)
			{
			sub_ind=14;
			index_set=13;
			}*/
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,18+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

	/*	if(sub_ind==13)
			{
			sub_ind=12;
			}*/		
	/*	if(sub_ind==11)
			{
			sub_ind=10;
			}*/
		if(sub_ind==9)
			{
			sub_ind=8;
			}
		if(sub_ind==7)
			{
			sub_ind=6;
			}
		if(sub_ind==5)
			{
			sub_ind=4;
			}
		if(sub_ind==3)
			{
			sub_ind=2;
			}
		if(sub_ind==1)
			{
			sub_ind=0;
			}
		}

	else if(sub_ind==0)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[0]==tstOFF) tst_state[0]=tst1;
			else if(tst_state[0]==tst1) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		else if(but==butL)
			{
			if(tst_state[0]==tst2) tst_state[0]=tst1;
			else if(tst_state[0]==tstOFF) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		}
	else if(sub_ind==2)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[1]==tstOFF) tst_state[1]=tst1;
			else if(tst_state[1]==tst1) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[1]==tst2) tst_state[1]=tst1;
			else if(tst_state[1]==tstOFF) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		}		

	else if(sub_ind==4)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[2]==tstOFF) tst_state[2]=tst1;
			else if(tst_state[2]==tst1) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[2]==tst2) tst_state[2]=tst1;
			else if(tst_state[2]==tstOFF) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		}
		
	else if(sub_ind==6)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[3]==tstOFF) tst_state[3]=tst1;
			else if(tst_state[3]==tst1) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[3]==tst2) tst_state[3]=tst1;
			else if(tst_state[3]==tstOFF) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		}

	else if(sub_ind==8)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[4]==tstOFF) tst_state[4]=tst1;
			else if(tst_state[4]==tst1) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[4]==tst2) tst_state[4]=tst1;
			else if(tst_state[4]==tstOFF) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		}

	else if(sub_ind==10)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[5]==tstOFF) tst_state[5]=tst1;
			else tst_state[5]=tstOFF;
			}
		}
	else if(sub_ind==11)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[6]==tstOFF) tst_state[6]=tst1;
			else tst_state[6]=tstOFF;
			}
		}
/*	else if(sub_ind==12)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[7]==tstOFF) tst_state[7]=tst1;
			else tst_state[7]=tstOFF;
			}
		}*/
	else if((sub_ind>=12)&&(sub_ind<(12+NUMIST))&&(NUMIST)&&((but==butE)))	
		{
		tree_up(iTst_bps,0,0,sub_ind-13);
		mcp2515_transmit(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		}											
	else if(sub_ind==(12+NUMIST))
		{
		if(but==butE)
			{
			bRESET=1;
			}
		}
			
	else if(sub_ind==(13+NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
	
		}	
	}



else if(ind==iTst_6U)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,14+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;

		if(sub_ind==1)
			{
			sub_ind=2;
			index_set=1;
			}
		if(sub_ind==3)
			{
			sub_ind=4;
			index_set=3;
			}
		if(sub_ind==5)
			{
			sub_ind=6;
			index_set=5;
			}
		if(sub_ind==7)
			{
			sub_ind=8;
			//index_set=7;
			}
		if(sub_ind==9)
			{
			index_set=8;
			}
		if(sub_ind==10)
			{
			sub_ind=11;
			index_set=10;
			}
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,14+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		
		if(sub_ind==10)
			{
			sub_ind=9;
			//index_set=5;
			}
		if(sub_ind==7)
			{
			sub_ind=6;
			//index_set=5;
			}
		if(sub_ind==5)
			{
			sub_ind=4;
			//index_set=5;
			}
		if(sub_ind==3)
			{
			sub_ind=2;
			//index_set=5;
			}
		if(sub_ind==1)
			{
			sub_ind=0;
			//index_set=5;
			}
		}

	else if(sub_ind==0)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[0]==tstOFF) tst_state[0]=tst1;
			else if(tst_state[0]==tst1) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		else if(but==butL)
			{
			if(tst_state[0]==tst2) tst_state[0]=tst1;
			else if(tst_state[0]==tstOFF) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		}
	else if(sub_ind==2)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[1]==tstOFF) tst_state[1]=tst1;
			else if(tst_state[1]==tst1) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[1]==tst2) tst_state[1]=tst1;
			else if(tst_state[1]==tstOFF) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		}		

	else if(sub_ind==4)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[9]==tstOFF) tst_state[9]=tst1;
			else if(tst_state[9]==tst1) tst_state[9]=tst2;
			else tst_state[9]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[9]==tst2) tst_state[9]=tst1;
			else if(tst_state[9]==tstOFF) tst_state[9]=tst2;
			else tst_state[9]=tstOFF;
			}
		}
		
	else if(sub_ind==6)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[2]==tstOFF) tst_state[2]=tst1;
			else if(tst_state[2]==tst1) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[2]==tst2) tst_state[2]=tst1;
			else if(tst_state[2]==tstOFF) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		}

	else if(sub_ind==8)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[3]==tstOFF) tst_state[3]=tst1;
			else if(tst_state[3]==tst1) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[3]==tst2) tst_state[3]=tst1;
			else if(tst_state[3]==tstOFF) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		}
	else if(sub_ind==9)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[4]==tstOFF) tst_state[4]=tst1;
			else if(tst_state[4]==tst1) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[4]==tst2) tst_state[4]=tst1;
			else if(tst_state[4]==tstOFF) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		}
	else if(sub_ind==11)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[7]==tstOFF) tst_state[7]=tst1;
			else tst_state[7]=tstOFF;
			}
		}
	else if(sub_ind==12)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[8]==tstOFF) tst_state[8]=tst1;
			else tst_state[8]=tstOFF;
			}
		}
	else if((sub_ind>=13)&&(sub_ind<(13+NUMIST))&&(NUMIST)&&((but==butE)))	
		{
		tree_up(iTst_bps,0,0,sub_ind-13);
		mcp2515_transmit(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		}											
	else if(sub_ind==(13+NUMIST))
		{
		if(but==butE)
			{
			bRESET=1;
			}
		}

	else if(sub_ind==(14+NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
	
		}	
	}

else if((ind==iTst_220)||(ind==iTst_220_380))
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,12+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;

		if(sub_ind==1)
			{
			sub_ind=2;
			index_set=1;
			}
		if(sub_ind==3)
			{
			sub_ind=4;
			index_set=3;
			}
		if(sub_ind==5)
			{
			sub_ind=6;
			//index_set=5;
			}
		if(sub_ind==7)
			{
			index_set=6;
			}
		if(sub_ind==8)
			{
			sub_ind=9;
			index_set=5;
			}
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,12+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		
		if(sub_ind==8)
			{
			sub_ind=7;
			//index_set=5;
			}
		if(sub_ind==5)
			{
			sub_ind=4;
			//index_set=5;
			}
		if(sub_ind==3)
			{
			sub_ind=2;
			//index_set=5;
			}
		if(sub_ind==1)
			{
			sub_ind=0;
			//index_set=5;
			}
		}

	else if(sub_ind==0)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[0]==tstOFF) tst_state[0]=tst1;
			else if(tst_state[0]==tst1) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		else if(but==butL)
			{
			if(tst_state[0]==tst2) tst_state[0]=tst1;
			else if(tst_state[0]==tstOFF) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		}
	else if(sub_ind==2)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[1]==tstOFF) tst_state[1]=tst1;
			else if(tst_state[1]==tst1) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[1]==tst2) tst_state[1]=tst1;
			else if(tst_state[1]==tstOFF) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		}		
		
	else if(sub_ind==4)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[2]==tstOFF) tst_state[2]=tst1;
			else if(tst_state[2]==tst1) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[2]==tst2) tst_state[2]=tst1;
			else if(tst_state[2]==tstOFF) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		}

	else if(sub_ind==6)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[3]==tstOFF) tst_state[3]=tst1;
			else if(tst_state[3]==tst1) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[3]==tst2) tst_state[3]=tst1;
			else if(tst_state[3]==tstOFF) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		}
	else if(sub_ind==7)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[4]==tstOFF) tst_state[4]=tst1;
			else if(tst_state[4]==tst1) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[4]==tst2) tst_state[4]=tst1;
			else if(tst_state[4]==tstOFF) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		}
	else if(sub_ind==9)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[7]==tstOFF) tst_state[7]=tst1;
			else tst_state[7]=tstOFF;
			}
		}
	else if(sub_ind==10)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[8]==tstOFF) tst_state[8]=tst1;
			else tst_state[8]=tstOFF;
			}
		}
	else if((sub_ind>=11)&&(sub_ind<(11+NUMIST))&&(NUMIST)&&((but==butE)))	
		{
		tree_up(iTst_bps,0,0,sub_ind-11);
		mcp2515_transmit(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		}											

	else if(sub_ind==(11+NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	else if(sub_ind==(12+NUMIST))
		{
		if(but==butE)
			{
			bRESET=1;
			}
		}			
	}



else if(ind==iTst_bps)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		
		if(sub_ind==2)
			{
			sub_ind=3;
			//index_set=2;
			}

		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		
		if(sub_ind==2)
			{
			sub_ind=1;
			//index_set=2;
			}
		}

	else if(sub_ind==0)
		{
		if(but==butR)
			{
			if(tst_state[5]==tstOFF)tst_state[5]=tst1;
			else if(tst_state[5]==tst1)tst_state[5]=tst2;
			else tst_state[5]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[5]==tstOFF)tst_state[5]=tst2;
			else if(tst_state[5]==tst1)tst_state[5]=tstOFF;
			else tst_state[5]=tst1;
			}
		}
	else if(sub_ind==1)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[6]==tstOFF) tst_state[6]=tst1;
			else if(tst_state[6]==tst1) tst_state[6]=tst2;
			else tst_state[6]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[6]==tst2) tst_state[6]=tst1;
			else if(tst_state[6]==tstOFF) tst_state[6]=tst2;
			else tst_state[6]=tstOFF;
			}
		}		
		
	else if(sub_ind==3)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}	
	}

else if(ind==iKlimat)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,7);
	
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,7);
		
		}
	else if(sub_ind==0)
	     {
	     if(but==butR)TBOXMAX++;
	     else if(but==butR_)TBOXMAX+=2;
	     else if(but==butL)TBOXMAX--;
	     else if(but==butL_)TBOXMAX-=2;
	     gran(&TBOXMAX,50,80);
	     lc640_write_int(EE_TBOXMAX,TBOXMAX);
	     speed=1;
	     }

	else if(sub_ind==1)
	     {
	     if(but==butR)TBOXVENTMAX++;
	     else if(but==butR_)TBOXVENTMAX+=2;
	     else if(but==butL)TBOXVENTMAX--;
	     else if(but==butL_)TBOXVENTMAX-=2;
	     gran(&TBOXVENTMAX,49,81);
	     lc640_write_int(EE_TBOXVENTMAX,TBOXVENTMAX);
	     speed=1;
	     }

	else if(sub_ind==2)
	     {
	     if(but==butR)TBOXREG++;
	     else if(but==butR_)TBOXREG+=2;
	     else if(but==butL)TBOXREG--;
	     else if(but==butL_)TBOXREG-=2;
	     gran(&TBOXREG,5,30);
	     lc640_write_int(EE_TBOXREG,TBOXREG);
	     speed=1;
	     }

	else if(sub_ind==3)
	     {
	     if(but==butR)TLOADDISABLE++;
	     else if(but==butR_)TLOADDISABLE+=2;
	     else if(but==butL)TLOADDISABLE--;
	     else if(but==butL_)TLOADDISABLE-=2;
	     gran(&TLOADDISABLE,49,81);
	     lc640_write_int(EE_TLOADDISABLE,TLOADDISABLE);
	     speed=1;
	     }

	else if(sub_ind==4)
	     {
	     if(but==butR)TLOADENABLE++;
	     else if(but==butR_)TLOADENABLE+=2;
	     else if(but==butL)TLOADENABLE--;
	     else if(but==butL_)TLOADENABLE-=2;
	     gran(&TLOADENABLE,44,TLOADDISABLE-5);
	     lc640_write_int(EE_TLOADENABLE,TLOADENABLE);
	     speed=1;
	     }

	else if(sub_ind==5)
	     {
	     if(but==butR)TBATDISABLE++;
	     else if(but==butR_)TBATDISABLE+=2;
	     else if(but==butL)TBATDISABLE--;
	     else if(but==butL_)TBATDISABLE-=2;
	     gran(&TBATDISABLE,49,91);
	     lc640_write_int(EE_TBATDISABLE,TBATDISABLE);
	     speed=1;
	     }

	else if(sub_ind==6)
	     {
	     if(but==butR)TBATENABLE++;
	     else if(but==butR_)TBATENABLE+=2;
	     else if(but==butL)TBATENABLE--;
	     else if(but==butL_)TBATENABLE-=2;
	     gran(&TBATENABLE,44,TBATDISABLE-5);
	     lc640_write_int(EE_TBATENABLE,TBATENABLE);
	     speed=1;
	     }
	else if(sub_ind==7)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	}




else if(ind==iNpn_set)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,simax);
	
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,simax);
		
		}
	else if(sub_ind==0)
	    {
	    if(NPN_OUT==npnoRELEVENT)NPN_OUT=npnoRELEAVBAT2;
		else if(NPN_OUT==npnoRELEAVBAT2)NPN_OUT=npnoOFF;
		else NPN_OUT=npnoRELEVENT;
	    lc640_write_int(EE_NPN_OUT,NPN_OUT);
	    
	    }
	else if(sub_ind==1)
	    {
		if(NPN_OUT==npnoOFF)
			{
			if(but==butE)			
				{
				tree_down(0,0);
				ret(0);
				}
			}
		else
			{
			/*if(NPN_SIGN==npnsAVNET)NPN_SIGN=npnsULOAD;
			else NPN_SIGN=npnsAVNET;
			lc640_write_int(EE_NPN_SIGN,NPN_SIGN);*/

			if(but==butR)UONPN++;
	     	else if(but==butR_)UONPN+=2;
	     	else if(but==butL)UONPN--;
	     	else if(but==butL_)UONPN-=2;
	     	gran(&UONPN,100,2500);
	     	lc640_write_int(EE_UONPN,UONPN);
	     	speed=1;

			}
		}
	else if(sub_ind==2)
		{
/*		if(NPN_SIGN==npnsULOAD)
			{
			if(but==butR)UONPN++;
	     	else if(but==butR_)UONPN+=2;
	     	else if(but==butL)UONPN--;
	     	else if(but==butL_)UONPN-=2;
	     	gran(&UONPN,100,2500);
	     	lc640_write_int(EE_UONPN,UONPN);
	     	speed=1;
			}
		else 
			{
			if(but==butR)TZNPN++;
	     	else if(but==butR_)TZNPN+=2;
	     	else if(but==butL)TZNPN--;
	     	else if(but==butL_)TZNPN-=2;
	     	gran(&TZNPN,10,60);
	     	lc640_write_int(EE_TZNPN,TZNPN);
	     	speed=1;
			}*/

			if(but==butR)UVNPN++;
	     	else if(but==butR_)UVNPN+=2;
	     	else if(but==butL)UVNPN--;
	     	else if(but==butL_)UVNPN-=2;
	     	gran(&UVNPN,100,2500);
	     	lc640_write_int(EE_UVNPN,UVNPN);
	     	speed=1;
		}
	else if(sub_ind==3)
		{
/*		if(NPN_SIGN==npnsULOAD)
			{
			if(but==butR)UVNPN++;
	     	else if(but==butR_)UVNPN+=2;
	     	else if(but==butL)UVNPN--;
	     	else if(but==butL_)UVNPN-=2;
	     	gran(&UVNPN,100,2500);
	     	lc640_write_int(EE_UVNPN,UVNPN);
	     	speed=1;
			}
		else 
			{
			if(but==butE)			
				{
				tree_down(0,0);
				ret(0);
				}
			}*/
			if(but==butR)TZNPN++;
	     	else if(but==butR_)TZNPN+=2;
	     	else if(but==butL)TZNPN--;
	     	else if(but==butL_)TZNPN-=2;
	     	gran(&TZNPN,10,60);
	     	lc640_write_int(EE_TZNPN,TZNPN);
	     	speed=1;
		}
	else if(sub_ind==4)
		{
/*		if(NPN_SIGN==npnsULOAD)
			{
			if(but==butR)TZNPN++;
	     	else if(but==butR_)TZNPN+=2;
	     	else if(but==butL)TZNPN--;
	     	else if(but==butL_)TZNPN-=2;
	     	gran(&TZNPN,10,60);
	     	lc640_write_int(EE_TZNPN,TZNPN);
	     	speed=1;
			}*/
			if(but==butE)			
				{
				tree_down(0,0);
				ret(0);
				}
		}
	else if(sub_ind==5)
		{
		if(NPN_SIGN==npnsULOAD)
			{
			if(but==butE)			
				{
				tree_down(0,0);
				ret(0);
				}
			}
		}


	}
else if(ind==iFWabout)
	{
	ret(1000);
	if(but==butE)
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	}

#endif		
but_an_end:
n_but=0;
}

//-----------------------------------------------
void watchdog_enable (void) 
{
LPC_WDT->WDTC=2000000;
LPC_WDT->WDCLKSEL=0;
LPC_WDT->WDMOD=3;
LPC_WDT->WDFEED=0xaa;
LPC_WDT->WDFEED=0x55;
}

//-----------------------------------------------
void watchdog_reset (void) 
{
LPC_WDT->WDFEED=0xaa;
LPC_WDT->WDFEED=0x55;
}





//***********************************************
//***********************************************
//***********************************************
//***********************************************
//***********************************************
void SysTick_Handler (void) 	 /* SysTick Interrupt Handler (1ms)    */
{
//sys_plazma++;
b2000Hz=1;

if(bTPS)
	{
	LPC_GPIO1->FIODIR|=(1UL<<26);
	LPC_GPIO1->FIOPIN^=(1UL<<26);
	}

if(++t0cnt4>=2)
	{
t0cnt4=0;
b1000Hz=1;

	bFF=(char)(GET_REG(LPC_GPIO0->FIOPIN, 27, 1));
	if(bFF!=bFF_) hz_out++;
	bFF_=bFF;

if(modbus_timeout_cnt<modbusTimeoutInMills)
	{
	modbus_timeout_cnt++;
	if(modbus_timeout_cnt>=modbusTimeoutInMills)
		{
		bMODBUS_TIMEOUT=1;
		//modbus_plazma3++;
		}
	}
else if (modbus_timeout_cnt>modbusTimeoutInMills)
	{
	modbus_timeout_cnt=0;
	bMODBUS_TIMEOUT=0;
	}

if(++t0cnt5>=20)
     {
     t0cnt5=0;
     b50Hz=1;
     }
     
if(++t0cnt>=10)
     {
     t0cnt=0;
     b100Hz=1;

     hz_out_cnt++;
     if(hz_out_cnt>=500)
	     {	
	     hz_out_cnt=0;
	     net_F=hz_out;
	     hz_out=0;
	     }

     if(++t0cnt0>=10)
	     {
	     t0cnt0=0;
	     b10Hz=1;
		beep_drv();
		if(main_10Hz_cnt<10000) main_10Hz_cnt++;
	     }

     if(t0cnt0==5)
	     {
		//beep_drv();
	     }

     if(++t0cnt1>=20)
	     {
	     t0cnt1=0;
	     b5Hz=1;
		if(bFL5)bFL5=0;
		else bFL5=1;     
	     }

     if(++t0cnt2>=50)
	     {
	     t0cnt2=0;
	     b2Hz=1;
		if(bFL2)bFL2=0;
		else bFL2=1;
	     }         

     if(++t0cnt3>=100)
	     {
	     t0cnt3=0;
	     b1Hz=1;
		 if(main_1Hz_cnt<10000) main_1Hz_cnt++;
		 if(kan_aktivity_cnt)kan_aktivity_cnt--;
	     }
     }

	}
//LPC_GPIO0->FIOCLR|=0x00000001;
  return;          



//LPC_GPIO0->FIOCLR|=0x00000001;
}


//***********************************************
__irq void timer0_interrupt(void) 
{	
/*if(BPS1_spa_leave)T0EMR_bit.EM1=0; 
else T0EMR_bit.EM1=1;
if(BPS2_spa_leave)T0EMR_bit.EM3=0; 
else T0EMR_bit.EM3=1;
T0IR = 0xff;*/
}

//===============================================
//===============================================
//===============================================
//===============================================
int main (void) 
{
char ind_reset_cnt=0;
//long i;
char mac_adr[6] = { 0x00,0x73,0x04,50,60,70 };

reset_plazma=(short)LPC_SC->RSID;
//LPC_SC->RSID=0xFF;
//i=200000;
//while(--i){};

SystemInit();

bTPS=1;

SysTick->LOAD = (SystemFrequency / 2000) - 1;
SysTick->CTRL = 0x07;

//init_timer( 0,SystemFrequency/2000/4 - 1 ); // 1ms	
//enable_timer( 0 );

//rs232_data_out_1();

bps[0]._state=bsOFF_AV_NET;
bps[1]._state=bsOFF_AV_NET;
bps[2]._state=bsOFF_AV_NET;
bps[3]._state=bsOFF_AV_NET;
bps[4]._state=bsOFF_AV_NET;
bps[5]._state=bsOFF_AV_NET;
bps[6]._state=bsOFF_AV_NET;

SET_REG(LPC_GPIO0->FIODIR, 0, 27, 1);
SET_REG(LPC_GPIO2->FIODIR, 1, 7, 1);
SET_REG(LPC_GPIO2->FIODIR, 1, 8, 1);
//LPC_GPIO1->FIODIR  |= 1<<27;                
	;
//FIO1MASK = 0x00000000;	 
//LPC_GPIO0->FIODIR  |= 1<<27;
//LPC_GPIO0->FIOSET  |= 1<<27;

///SET_REG(LPC_GPIO0->FIODIR,0,10,1); //вход частоты 

SET_REG(LPC_GPIO3->FIODIR,1,SHIFT_REL_AV_NET,1);
SET_REG(LPC_GPIO3->FIOSET,1,SHIFT_REL_AV_NET,1);  // реле аварии сети под ток


ad7705_reset();
delay_ms(20);

ad7705_write(0x21);
ad7705_write(BIN8(1101)); 
ad7705_write(0x11);
ad7705_write(0x44);


ad7705_buff[0][1]=0x7fff;
ad7705_buff[0][2]=0x7fff;
ad7705_buff[0][3]=0x7fff;
ad7705_buff[0][4]=0x7fff;
ad7705_buff[0][5]=0x7fff;
ad7705_buff[0][6]=0x7fff;
ad7705_buff[0][7]=0x7fff;
ad7705_buff[0][8]=0x7fff;
ad7705_buff[0][9]=0x7fff;
ad7705_buff[0][10]=0x7fff;
ad7705_buff[0][11]=0x7fff;
ad7705_buff[0][12]=0x7fff;
ad7705_buff[0][13]=0x7fff;
ad7705_buff[0][14]=0x7fff;
ad7705_buff[0][15]=0x7fff;
ad7705_buff[1][1]=0x7fff;
ad7705_buff[1][2]=0x7fff;
ad7705_buff[1][3]=0x7fff;
ad7705_buff[1][4]=0x7fff;
ad7705_buff[1][5]=0x7fff;
ad7705_buff[1][6]=0x7fff;
ad7705_buff[1][7]=0x7fff;
ad7705_buff[1][8]=0x7fff;
ad7705_buff[1][9]=0x7fff;
ad7705_buff[1][10]=0x7fff;
ad7705_buff[1][11]=0x7fff;
ad7705_buff[1][12]=0x7fff;
ad7705_buff[1][13]=0x7fff;
ad7705_buff[1][14]=0x7fff;
ad7705_buff[1][15]=0x7fff;

ad7705_buff_[0]=0x7fff;
ad7705_buff_[1]=0x7fff;

/*
ad7705_reset();
delay_ms(20);

ad7705_write(0x20);
ad7705_write(BIN8(1101)); 
ad7705_write(0x10);
ad7705_write(0x44);

ad7705_reset();
delay_ms(20);  

ad7705_write(0x20);
ad7705_write(BIN8(1101)); 
ad7705_write(0x10);
ad7705_write(0x44); 

delay_ms(20); */




lcd_init();  
lcd_on();
lcd_clear();
		
///LPC_GPIO4->FIODIR |= (1<<29);           /* LEDs on PORT2 defined as Output    */
rtc_init();
///pwm_init();
ind=iMn;
#ifdef UKU_GLONASS
ind=iMn_GLONASS;
#endif
#ifdef UKU_3U
ind=iMn_3U;
#endif
#ifdef UKU_RSTKM
ind=iMn_RSTKM;
#endif 
#ifdef UKU_KONTUR
ind=iMn_KONTUR;
#endif 
#ifdef UKU_6U
ind=iMn_6U;
#endif
#ifdef UKU_220
ind=iMn_220;
#endif
#ifdef UKU_220_IPS_TERMOKOMPENSAT
ind=iMn_220_IPS_TERMOKOMPENSAT;
#endif
#ifdef UKU_220_V2
ind=iMn_220_V2;
#endif
#ifdef UKU_INV
ind=iMn_INV;
#endif
//snmp_plazma=15;


//#ifdef ETHISON
//mac_adr[5]=*((char*)&AUSW_MAIN_NUMBER);
//mac_adr[4]=*(((char*)&AUSW_MAIN_NUMBER)+1);
//mac_adr[3]=*(((char*)&AUSW_MAIN_NUMBER)+2);
//mem_copy (own_hw_adr, mac_adr, 6);


//if(lc640_read_int(EE_ETH_IS_ON)==1)
	//{
	bgnd_par(		"                    ",
     		"    Инициализация   ",
     		"      Ethernet      ",
     		"                    ");
	//bitmap_hndl();
	//lcd_out(lcd_bitmap);
	//init_TcpNet ();

	//init_ETH();
	//mem_copy (&localm[NETIF_ETH], &ip_config, sizeof(ip_config));

//	}
//#endif
//event2snmp(2);

reload_hndl();
//LPC_GPIO0->FIODIR |= (0x60000000);

adc_init();

LPC_GPIO0->FIODIR|=(1<<11);
LPC_GPIO0->FIOSET|=(1<<11);


lc640_write_int(100,134);

/*
can1_init(BITRATE62_5K25MHZ); 
can2_init(BITRATE125K25MHZ);
FullCAN_SetFilter(1,0x0e9);
FullCAN_SetFilter(0,0x18e);
//FullCAN_SetFilter(0,0x09e);
*/

UARTInit(0, 9600);	/* baud rate setting */

memo_read();

mac_adr[5]=*((char*)&AUSW_MAIN_NUMBER);
mac_adr[4]=*(((char*)&AUSW_MAIN_NUMBER)+1);
mac_adr[3]=*(((char*)&AUSW_MAIN_NUMBER)+2);
mem_copy (own_hw_adr, mac_adr, 6);

snmp_Community[0]=(char)lc640_read_int(EE_COMMUNITY); 
//if((snmp_Community[0]==0)||(snmp_Community[0]==' '))snmp_Community[0]=0;
snmp_Community[1]=(char)lc640_read_int(EE_COMMUNITY+2);
if((snmp_Community[1]==0)||(snmp_Community[1]==' '))snmp_Community[1]=0;
snmp_Community[2]=(char)lc640_read_int(EE_COMMUNITY+4);
if((snmp_Community[2]==0)||(snmp_Community[2]==' '))snmp_Community[2]=0;
snmp_Community[3]=(char)lc640_read_int(EE_COMMUNITY+6);
if((snmp_Community[3]==0)||(snmp_Community[3]==' '))snmp_Community[3]=0;
snmp_Community[4]=(char)lc640_read_int(EE_COMMUNITY+8);
if((snmp_Community[4]==0)||(snmp_Community[4]==' '))snmp_Community[4]=0;
snmp_Community[5]=(char)lc640_read_int(EE_COMMUNITY+10);
if((snmp_Community[5]==0)||(snmp_Community[5]==' '))snmp_Community[5]=0;
snmp_Community[6]=(char)lc640_read_int(EE_COMMUNITY+12);
if((snmp_Community[6]==0)||(snmp_Community[6]==' '))snmp_Community[6]=0;
snmp_Community[7]=(char)lc640_read_int(EE_COMMUNITY+14);
if((snmp_Community[7]==0)||(snmp_Community[7]==' '))snmp_Community[7]=0;
snmp_Community[8]=(char)lc640_read_int(EE_COMMUNITY+16);
if((snmp_Community[8]==0)||(snmp_Community[8]==' '))snmp_Community[8]=0;
snmp_Community[9]=0; /**/

if(lc640_read_int(EE_ETH_IS_ON)==1)
	{
	bgnd_par(		"                    ",
     		"    Инициализация   ",
     		"      Ethernet      ",
     		"                    ");
	bitmap_hndl();
	lcd_out(lcd_bitmap);
	init_TcpNet ();
	lcd_out(lcd_bitmap);
	init_ETH();
	//mem_copy (&localm[NETIF_ETH], &ip_config, sizeof(ip_config));
//	lcd_out(lcd_bitmap);
	}
//sys_plazma1=sys_plazma;
ind_reset_cnt=58;

if(__ee_spc_stat==spcVZ)
	{
	if(__ee_vz_cnt)
		{
		spc_stat=spcVZ;  
		vz_cnt_h=__ee_vz_cnt/60;
		vz_cnt_h_=(lc640_read_int(EE_SPC_VZ_LENGT)-__ee_vz_cnt)/60;
		if(vz_cnt_h_<0)vz_cnt_h_=0;
		vz_cnt_s_=(short)(((lc640_read_int(EE_SPC_VZ_LENGT)-__ee_vz_cnt)*60)%3600UL);

		vz_cnt_s=0;
		}
	}
else if(__ee_spc_stat==spcKE)
	{
	spc_stat=spcKE;
	spc_bat=__ee_spc_bat;
	bat[spc_bat]._zar_cnt_ke=0;
	spc_phase=__ee_spc_phase;
	}
watchdog_enable();
if((AUSW_MAIN==2400)||(AUSW_MAIN==4800)||(AUSW_MAIN==6000))
	{
	cntrl_stat=350;
	cntrl_stat_old=350;
	}
	
//snmp_trap_send("Reload",1,2,3);

//tree_up(iDeb,2,0,0);

can_mcp2515_init();
#ifdef SC16IS740_UART
sc16is700_init((uint32_t)(MODBUS_BAUDRATE*10UL));
#endif

modbusTimeoutInMills=3000/MODBUS_BAUDRATE;
if(modbusTimeoutInMills<2)modbusTimeoutInMills=2;
modbusTimeoutInMills+=2;

socket_udp = udp_get_socket (0, UDP_OPT_SEND_CS | UDP_OPT_CHK_CS, udp_callback);
if (socket_udp != 0) 
	{
    udp_open (socket_udp, PORT_NUM);
  }

socket_tcp = tcp_get_socket (TCP_TYPE_SERVER, 0, 10, tcp_callback);
if (socket_tcp != 0) 
	{
    tcp_listen (socket_tcp, 502);
  	}
kan_aktivity_cnt=15;
			
while (1)  
	{
	bTPS=0; 
     //timer_poll ();
     main_TcpNet ();
	if(bMODBUS_TIMEOUT)
		{
		bMODBUS_TIMEOUT=0;
		//modbus_plazma++;;
		modbus_in();
		}
	if(bRXIN0) 
		{
		bRXIN0=0;
	
		uart_in0();
		}
	if(bMCP2515_IN)
		{
		bMCP2515_IN=0;
		can_in_an1();
		}		 
	/*
	if(bRXIN1) 
		{
		bRXIN1=0;
	
		uart_in1();
		}*/ 
     if(b10000Hz)
		{
		b10000Hz=0; 
		

		}

     if(b2000Hz)
		{

		if(adc_window_cnt<200)adc_window_cnt++;

		b2000Hz=0; 
		adc_drv7();
		
		}

	if(b1000Hz)
		{
		b1000Hz=0;

	    	can_mcp2515_hndl();

		#ifdef SC16IS740_UART
		sc16is700_uart_hndl();
		#endif
		}
	
	if(b100Hz)
		{
		b100Hz=0;

		//LPC_GPIO2->FIODIR|=(1<<7);
		//LPC_GPIO2->FIOPIN^=(1<<7);		

		

		if(!bRESET)but_drv();
		but_an();
		}
		 
	if(b50Hz)
		{
		b50Hz=0;

		//net_drv();
		net_drv_mcp2515();
		}

	if(b10Hz)
		{
		char i;

     timer_tick ();
     tick = __TRUE;

		b10Hz=0;
				
			
		inv_hndl();

			
		
		ind_hndl(); 
		#ifndef SIMULATOR
		bitmap_hndl();
		if(!bRESET)
			{
			lcd_out(lcd_bitmap);
			}
		#endif
		//ad7705_drv();
		//ad7705_write(0x20);

		adc_window_cnt=0;  

		ret_hndl();  
		mess_hndl();
		ret_hndl();
		ext_drv();
		avt_hndl();
		rele_hndl();
		byps_av_hndl();
		inv_av_hndl();
		dc_in_av_hndl();
		}

	if(b5Hz)
		{
		b5Hz=0;

		if(!bRESET)
			{
			ad7705_drv();
			}
		if(!bRESET)
			{
			memo_read();
			}
		LPC_GPIO1->FIODIR|=(1UL<<26);
		matemat();
		
		rele_hndl();
		if(!bRESET)avar_hndl();
		zar_superviser_drv();
		snmp_data();
		//LPC_GPIO1->FIODIR|=(1UL<<31);
		//LPC_GPIO1->FIOPIN^=(1UL<<31);


  		}

	if(b2Hz)
		{
		b2Hz=0;

				//uart_out_adr1(dig,150);
		
  		}

	if(b1Hz)
		{
		b1Hz=0;
		if(!bRESET)
			{
			watchdog_reset();
			}
		//mcp2515_transmit_adr((char*)&net_U,21);

		samokalibr_hndl();
		//num_necc_hndl();
		//zar_drv();
		ubat_old_drv();
		beep_hndl();
		//avg_hndl();
		ke_drv();
		mnemo_hndl();
		vent_hndl();

		plazma_plazma_plazma++;

		if(++ind_reset_cnt>=60)
			{
			ind_reset_cnt=0;
			lcd_init();
			lcd_on();
			lcd_clear();
			}
               
          vent_hndl();
		klimat_hndl();
		
		if(t_ext_can_nd<10) t_ext_can_nd++;
		
		//if(main_1Hz_cnt<200)main_1Hz_cnt++;


		can_reset_hndl();

/*			{
			char pal[10],i;
			pal[0]=1;
			pal[1]=2;
			pal[2]=3;
			pal[3]=4;
			pal[4]=5;
			pal[5]=6;
			pal[6]=7;

			for (i=0;i<7;i++)
				{
				putchar_sc16is700(pal[i]);
				}
			} */
		//byps._cnt++;
		time_sinc_hndl();
		system_status_hndl();
		}
	if(b1min)
		{
		b1min=0;

		if((tloaddisable_cmnd)&&(tloaddisable_cmnd<=10))
			{
			tloaddisable_cmnd--;
			if(!tloaddisable_cmnd)tloaddisable_cmnd=20;
			}
		if((tbatdisable_cmnd)&&(tbatdisable_cmnd<=10))
			{
			if(!tbatdisable_cmnd)tbatdisable_cmnd=20;
			}

		
		}
	}
}
