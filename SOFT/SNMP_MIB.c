/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    SNMP_MIB.C
 *      Purpose: SNMP Agent Management Information Base Module
 *      Rev.:    V4.12
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2010 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <Net_Config.h>
#include "LPC17xx.H"
#include "main.H"
#include "control.H"
#include "snmp_data_file.h" 

/* snmp_demo.c */
extern U8   get_button (void);
extern void LED_out (U32 val);
extern BOOL LCDupdate;
extern U8   lcd_text[2][16+1];

/* System */
extern U32  snmp_SysUpTime;

/* Local variables */
//static U8   LedOut;
//static U8   KeyIn;

/* MIB Read Only integer constants */
static const U8 sysServices = 79;
static const U16 sysMainsVoltage = 220;
static const U8 displayPsuQauntity = 2;
static const U8 TestForTableValues = 57;

 char* const aaa = "Novosibirsk, Russia";

int a_;
char aa_;
char* aaa_="abc";

/* MIB Entry event Callback functions. */
//static void write_leds (int mode);
//static void read_key (int mode);
//static void upd_display (int mode);

#ifndef UKU_KONTUR
/*----------------------------------------------------------------------------
 *      MIB Data Table
 *---------------------------------------------------------------------------*/

 MIB_ENTRY snmp_mib[] = {

  /* ---------- System MIB ----------- */

  /* SysDescr Entry */
  { MIB_OCTET_STR | MIB_ATR_RO,	     8, {OID0(1,3), 6, 1, 2, 1, 1, 1, 0},      MIB_STR("First ARM SNMP agent for SibPromAutomatika"),     NULL },
  /* SysObjectID Entry */
  { MIB_OBJECT_ID | MIB_ATR_RO,	     8, {OID0(1,3), 6, 1, 2, 1, 1, 2, 0},	    MIB_STR("\x2b\x06\x01\x04\x01\x82\x83\x1F"),    NULL },
  /* SysUpTime Entry */
  { MIB_TIME_TICKS | MIB_ATR_RO,     8, {OID0(1,3), 6, 1, 2, 1, 1, 3, 0},    4, &snmp_SysUpTime,    NULL },
  /* SysContact Entry */
  { MIB_OCTET_STR | MIB_ATR_RO,	     8, {OID0(1,3), 6, 1, 2, 1, 1, 4, 0},    MIB_STR("Skype:danilov_aa"),    NULL },
  /* SysName Entry */
  { MIB_OCTET_STR | MIB_ATR_RO,		    8, {OID0(1,3), 6, 1, 2, 1, 1, 5, 0},    MIB_STR("UKU203LAN"),    NULL },
  /* SysLocation Entry */
  { MIB_OCTET_STR | MIB_ATR_RO,		     8, {OID0(1,3), 6, 1, 2, 1, 1, 6, 0},    MIB_STR("Novosibirsk, Russia"),    NULL },
  /* SysServices Entry */
  { MIB_INTEGER | MIB_ATR_RO,			    8, {OID0(1,3), 6, 1, 2, 1, 1, 7, 0},    MIB_INT(sysServices),    NULL },

  /* ---------- Experimental MIB ----------- */

	{ MIB_OCTET_STR | MIB_ATR_RO, 12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_MESSAGE , 0},			MIB_STR(snmp_spc_trap_message),     NULL},
	{ MIB_INTEGER | MIB_ATR_RO, 	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_VALUE_0 , 0},			MIB_INT(snmp_spc_trap_value_0),     NULL},
	{ MIB_INTEGER | MIB_ATR_RO, 	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_VALUE_1 , 0},			MIB_INT(snmp_spc_trap_value_1),     NULL},
	{ MIB_INTEGER | MIB_ATR_RO, 	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_VALUE_2 , 0},			MIB_INT(snmp_spc_trap_value_2),     NULL},


  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_CODE, 0},  	MIB_INT(snmp_device_code),  		NULL},   				//код устройства
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_SERIAL, 0},	MIB_INT(snmp_sernum),	  		NULL },				//серийный номер	
  	{ MIB_OCTET_STR, 			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_LOCATION, 0},  	MIB_STR(snmp_location),  		snmp_location_write},	//местоположение устройства
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFBAT, 0}, 	MIB_INT(snmp_numofbat),  		NULL},				//количество введенных батарей
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFBPS, 0},	MIB_INT(snmp_numofbps),  		NULL},				//количество введенных источников
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFINV, 0},	MIB_INT(snmp_numofinv),  			NULL},				//количество введенных источников
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFAVT, 0}, 	MIB_INT(snmp_numofavt),  			NULL},				//количество введенных батарей
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFDT, 0},	MIB_INT(snmp_numofdt),  			NULL},				//количество введенных источников
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFSK, 0},	MIB_INT(snmp_numofsk),  			NULL},				//количество введенных источников
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFEVENTS, 0},MIB_INT(snmp_numofbat),  		NULL},				//количество введенных батарей


  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_VOLTAGE, 0},  	MIB_INT(snmp_mains_power_voltage), NULL},	//напряжение сети	
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_FREQUENCY, 0},  MIB_INT(snmp_mains_power_frequency),NULL},	//частота сети
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_STATUS, 0},  	MIB_INT(snmp_mains_power_status),  NULL},	//состояние сети 
 	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_ALARM, 0},  	MIB_INT(snmp_mains_power_alarm),  	NULL},	//аварии сети
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_VOLTAGE_PHASEA, 0},  	MIB_INT(snmp_mains_power_voltage_phaseA), NULL},	//напряжение сети	
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_VOLTAGE_PHASEB, 0},  	MIB_INT(snmp_mains_power_voltage_phaseB), NULL},	//напряжение сети	
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_VOLTAGE_PHASEC, 0},  	MIB_INT(snmp_mains_power_voltage_phaseC), NULL},	//напряжение сети	


	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOAD, DISPLAY_LOAD_VOLTAGE, 0},  				MIB_INT(snmp_load_voltage),  		NULL},	//напряжение нагрузки
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOAD, DISPLAY_LOAD_CURRENT, 0},  				MIB_INT(snmp_load_current),  		NULL},	//ток нагрузки
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOAD, DISPLAY_LOAD_POWER, 0},  				MIB_INT(snmp_load_power),  		NULL},	//мощность нагрузки

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 1},  			MIB_INT(snmp_bps_number[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 2},  			MIB_INT(snmp_bps_number[1]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 3},  			MIB_INT(snmp_bps_number[2]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 4},  			MIB_INT(snmp_bps_number[3]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 5},  			MIB_INT(snmp_bps_number[4]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 6},  			MIB_INT(snmp_bps_number[5]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 7},  			MIB_INT(snmp_bps_number[6]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 8},  			MIB_INT(snmp_bps_number[7]),  	NULL},	//Номер БПСа

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 1},  			MIB_INT(snmp_bps_voltage[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 2},  			MIB_INT(snmp_bps_voltage[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 3},  			MIB_INT(snmp_bps_voltage[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 4},  			MIB_INT(snmp_bps_voltage[3]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 5},  			MIB_INT(snmp_bps_voltage[4]),  	NULL},	//Напряжение БПС4
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 6},  			MIB_INT(snmp_bps_voltage[5]),  	NULL},	//Напряжение БПС5
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 7},  			MIB_INT(snmp_bps_voltage[6]),  	NULL},	//Напряжение БПС6
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 8},  			MIB_INT(snmp_bps_voltage[7]),  	NULL},	//Напряжение БПС7

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 1},  			MIB_INT(snmp_bps_current[0]),  	NULL},	//Ток БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 2},  			MIB_INT(snmp_bps_current[1]),  	NULL},	//Ток БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 3},  			MIB_INT(snmp_bps_current[2]),  	NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 4},  			MIB_INT(snmp_bps_current[3]),  	NULL},	//Ток БПС4
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 5},  			MIB_INT(snmp_bps_current[4]),  	NULL},	//Ток БПС5
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 6},  			MIB_INT(snmp_bps_current[5]),  	NULL},	//Ток БПС6
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 7},  			MIB_INT(snmp_bps_current[6]),  	NULL},	//Ток БПС7
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 8},  			MIB_INT(snmp_bps_current[7]),  	NULL},	//Ток БПС8

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 1},  		MIB_INT(snmp_bps_temperature[0]),  NULL},	//Ток БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 2},  		MIB_INT(snmp_bps_temperature[1]),  NULL},	//Ток БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 3},  		MIB_INT(snmp_bps_temperature[2]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 4},  		MIB_INT(snmp_bps_temperature[3]),  NULL},	//Ток БПС4
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 5},  		MIB_INT(snmp_bps_temperature[4]),  NULL},	//Ток БПС5
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 6},  		MIB_INT(snmp_bps_temperature[5]),  NULL},	//Ток БПС6
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 7},  		MIB_INT(snmp_bps_temperature[6]),  NULL},	//Ток БПС7
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 8},  		MIB_INT(snmp_bps_temperature[7]),  NULL},	//Ток БПС8

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 1},  			MIB_INT(snmp_bps_stat[0]),  NULL},			//Состояние БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 2},  			MIB_INT(snmp_bps_stat[1]),  NULL},			//Состояние БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 3},  			MIB_INT(snmp_bps_stat[2]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 4},  			MIB_INT(snmp_bps_stat[3]),  NULL},			//Состояние БПС4
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 5},  			MIB_INT(snmp_bps_stat[4]),  NULL},			//Состояние БПС5
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 6},  			MIB_INT(snmp_bps_stat[5]),  NULL},			//Состояние БПС6
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 7},  			MIB_INT(snmp_bps_stat[6]),  NULL},			//Состояние БПС7
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 8},  			MIB_INT(snmp_bps_stat[7]),  NULL},			//Состояние БПС8

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 1},  			MIB_INT(snmp_inv_number[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 2},  			MIB_INT(snmp_inv_number[1]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 3},  			MIB_INT(snmp_inv_number[2]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 4},  			MIB_INT(snmp_inv_number[3]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 5},  			MIB_INT(snmp_inv_number[4]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 6},  			MIB_INT(snmp_inv_number[5]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 7},  			MIB_INT(snmp_inv_number[6]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 8},  			MIB_INT(snmp_inv_number[7]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 9},  			MIB_INT(snmp_inv_number[8]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 10},  			MIB_INT(snmp_inv_number[9]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 11},  			MIB_INT(snmp_inv_number[10]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 12},  			MIB_INT(snmp_inv_number[11]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 13},  			MIB_INT(snmp_inv_number[12]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 14},  			MIB_INT(snmp_inv_number[13]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 15},  			MIB_INT(snmp_inv_number[14]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 16},  			MIB_INT(snmp_inv_number[15]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 17},  			MIB_INT(snmp_inv_number[16]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 18},  			MIB_INT(snmp_inv_number[17]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 19},  			MIB_INT(snmp_inv_number[18]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 20},  			MIB_INT(snmp_inv_number[19]),  	NULL},	//Номер БПСа



	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 1},  			MIB_INT(snmp_inv_voltage[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 2},  			MIB_INT(snmp_inv_voltage[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 3},  			MIB_INT(snmp_inv_voltage[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 4},  			MIB_INT(snmp_inv_voltage[3]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 5},  			MIB_INT(snmp_inv_voltage[4]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 6},  			MIB_INT(snmp_inv_voltage[5]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 7},  			MIB_INT(snmp_inv_voltage[6]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 8},  			MIB_INT(snmp_inv_voltage[7]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 9},  			MIB_INT(snmp_inv_voltage[8]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 10},  			MIB_INT(snmp_inv_voltage[9]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 11},  			MIB_INT(snmp_inv_voltage[10]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 12},  			MIB_INT(snmp_inv_voltage[11]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 13},  			MIB_INT(snmp_inv_voltage[12]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 14},  			MIB_INT(snmp_inv_voltage[13]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 15},  			MIB_INT(snmp_inv_voltage[14]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 16},  			MIB_INT(snmp_inv_voltage[15]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 17},  			MIB_INT(snmp_inv_voltage[16]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 18},  			MIB_INT(snmp_inv_voltage[17]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 19},  			MIB_INT(snmp_inv_voltage[18]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 20},  			MIB_INT(snmp_inv_voltage[19]),  	NULL},	//Напряжение БПС3


	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 1},  			MIB_INT(snmp_inv_current[0]),  	NULL},	//Ток БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 2},  			MIB_INT(snmp_inv_current[1]),  	NULL},	//Ток БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 3},  			MIB_INT(snmp_inv_current[2]),  	NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 4},  			MIB_INT(snmp_inv_current[3]),  	NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 5},  			MIB_INT(snmp_inv_current[4]),  	NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 6},  			MIB_INT(snmp_inv_current[5]),  	NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 7},  			MIB_INT(snmp_inv_current[6]),  	NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 8},  			MIB_INT(snmp_inv_current[7]),  	NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 9},  			MIB_INT(snmp_inv_current[8]),  	NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 10},  			MIB_INT(snmp_inv_current[9]),  	NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 11},  			MIB_INT(snmp_inv_current[10]),  	NULL},	//Ток БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 12},  			MIB_INT(snmp_inv_current[11]),  	NULL},	//Ток БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 13},  			MIB_INT(snmp_inv_current[12]),  	NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 14},  			MIB_INT(snmp_inv_current[13]),  	NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 15},  			MIB_INT(snmp_inv_current[14]),  	NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 16},  			MIB_INT(snmp_inv_current[15]),  	NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 17},  			MIB_INT(snmp_inv_current[16]),  	NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 18},  			MIB_INT(snmp_inv_current[17]),  	NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 19},  			MIB_INT(snmp_inv_current[18]),  	NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 20},  			MIB_INT(snmp_inv_current[19]),  	NULL},	//Ток БПС3


	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 1},  		MIB_INT(snmp_inv_temperature[0]),  NULL},	//Ток БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 2},  		MIB_INT(snmp_inv_temperature[1]),  NULL},	//Ток БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 3},  		MIB_INT(snmp_inv_temperature[2]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 4},  		MIB_INT(snmp_inv_temperature[3]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 5},  		MIB_INT(snmp_inv_temperature[4]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 6},  		MIB_INT(snmp_inv_temperature[5]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 7},  		MIB_INT(snmp_inv_temperature[6]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 8},  		MIB_INT(snmp_inv_temperature[7]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 9},  		MIB_INT(snmp_inv_temperature[8]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 10},  		MIB_INT(snmp_inv_temperature[9]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 11},  		MIB_INT(snmp_inv_temperature[10]),  NULL},	//Ток БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 12},  		MIB_INT(snmp_inv_temperature[11]),  NULL},	//Ток БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 13},  		MIB_INT(snmp_inv_temperature[12]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 14},  		MIB_INT(snmp_inv_temperature[13]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 15},  		MIB_INT(snmp_inv_temperature[14]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 16},  		MIB_INT(snmp_inv_temperature[15]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 17},  		MIB_INT(snmp_inv_temperature[16]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 18},  		MIB_INT(snmp_inv_temperature[17]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 19},  		MIB_INT(snmp_inv_temperature[18]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 20},  		MIB_INT(snmp_inv_temperature[19]),  NULL},	//Ток БПС3


	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 1},  			MIB_INT(snmp_inv_stat[0]),  NULL},			//Состояние БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 2},  			MIB_INT(snmp_inv_stat[1]),  NULL},			//Состояние БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 3},  			MIB_INT(snmp_inv_stat[2]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 4},  			MIB_INT(snmp_inv_stat[3]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 5},  			MIB_INT(snmp_inv_stat[4]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 6},  			MIB_INT(snmp_inv_stat[5]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 7},  			MIB_INT(snmp_inv_stat[6]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 8},  			MIB_INT(snmp_inv_stat[7]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 9},  			MIB_INT(snmp_inv_stat[8]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 10},  			MIB_INT(snmp_inv_stat[9]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 11},  			MIB_INT(snmp_inv_stat[10]),  NULL},		//Состояние БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 12},  			MIB_INT(snmp_inv_stat[11]),  NULL},		//Состояние БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 13},  			MIB_INT(snmp_inv_stat[12]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 14},  			MIB_INT(snmp_inv_stat[13]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 15},  			MIB_INT(snmp_inv_stat[14]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 16},  			MIB_INT(snmp_inv_stat[15]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 17},  			MIB_INT(snmp_inv_stat[16]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 18},  			MIB_INT(snmp_inv_stat[17]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 19},  			MIB_INT(snmp_inv_stat[18]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 20},  			MIB_INT(snmp_inv_stat[19]),  NULL},		//Состояние БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 1},  			MIB_INT(snmp_inv_u_prim[0]),  NULL},			//Состояние БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 2},  			MIB_INT(snmp_inv_u_prim[1]),  NULL},			//Состояние БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 3},  			MIB_INT(snmp_inv_u_prim[2]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 4},  			MIB_INT(snmp_inv_u_prim[3]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 5},  			MIB_INT(snmp_inv_u_prim[4]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 6},  			MIB_INT(snmp_inv_u_prim[5]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 7},  			MIB_INT(snmp_inv_u_prim[6]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 8},  			MIB_INT(snmp_inv_u_prim[7]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 9},  			MIB_INT(snmp_inv_u_prim[8]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 10},  			MIB_INT(snmp_inv_u_prim[9]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 11},  			MIB_INT(snmp_inv_u_prim[10]),  NULL},		//Состояние БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 12},  			MIB_INT(snmp_inv_u_prim[11]),  NULL},		//Состояние БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 13},  			MIB_INT(snmp_inv_u_prim[12]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 14},  			MIB_INT(snmp_inv_u_prim[13]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 15},  			MIB_INT(snmp_inv_u_prim[14]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 16},  			MIB_INT(snmp_inv_u_prim[15]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 17},  			MIB_INT(snmp_inv_u_prim[16]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 18},  			MIB_INT(snmp_inv_u_prim[17]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 19},  			MIB_INT(snmp_inv_u_prim[18]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_PRIM, 20},  			MIB_INT(snmp_inv_u_prim[19]),  NULL},		//Состояние БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 1},  				MIB_INT(snmp_inv_u_in[0]),  NULL},			//Состояние БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 2},  				MIB_INT(snmp_inv_u_in[1]),  NULL},			//Состояние БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 3},  				MIB_INT(snmp_inv_u_in[2]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 4},  				MIB_INT(snmp_inv_u_in[3]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 5},  				MIB_INT(snmp_inv_u_in[4]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 6},  				MIB_INT(snmp_inv_u_in[5]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 7},  				MIB_INT(snmp_inv_u_in[6]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 8},  				MIB_INT(snmp_inv_u_in[7]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 9},  				MIB_INT(snmp_inv_u_in[8]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 10},  			MIB_INT(snmp_inv_u_in[9]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 11},  			MIB_INT(snmp_inv_u_in[10]),  NULL},		//Состояние БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 12},  			MIB_INT(snmp_inv_u_in[11]),  NULL},		//Состояние БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 13},  			MIB_INT(snmp_inv_u_in[12]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 14},  			MIB_INT(snmp_inv_u_in[13]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 15},  			MIB_INT(snmp_inv_u_in[14]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 16},  			MIB_INT(snmp_inv_u_in[15]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 17},  			MIB_INT(snmp_inv_u_in[16]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 18},  			MIB_INT(snmp_inv_u_in[17]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 19},  			MIB_INT(snmp_inv_u_in[18]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_U_BUS, 20},  			MIB_INT(snmp_inv_u_in[19]),  NULL},		//Состояние БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 1},  			MIB_INT(snmp_inv_p_out[0]),  NULL},			//Состояние БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 2},  			MIB_INT(snmp_inv_p_out[1]),  NULL},			//Состояние БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 3},  			MIB_INT(snmp_inv_p_out[2]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 4},  			MIB_INT(snmp_inv_p_out[3]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 5},  			MIB_INT(snmp_inv_p_out[4]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 6},  			MIB_INT(snmp_inv_p_out[5]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 7},  			MIB_INT(snmp_inv_p_out[6]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 8},  			MIB_INT(snmp_inv_p_out[7]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 9},  			MIB_INT(snmp_inv_p_out[8]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 10},  			MIB_INT(snmp_inv_p_out[9]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 11},  			MIB_INT(snmp_inv_p_out[10]),  NULL},		//Состояние БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 12},  			MIB_INT(snmp_inv_p_out[11]),  NULL},		//Состояние БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 13},  			MIB_INT(snmp_inv_p_out[12]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 14},  			MIB_INT(snmp_inv_p_out[13]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 15},  			MIB_INT(snmp_inv_p_out[14]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 16},  			MIB_INT(snmp_inv_p_out[15]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 17},  			MIB_INT(snmp_inv_p_out[16]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 18},  			MIB_INT(snmp_inv_p_out[17]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 19},  			MIB_INT(snmp_inv_p_out[18]),  NULL},		//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_P_OUT, 20},  			MIB_INT(snmp_inv_p_out[19]),  NULL},		//Состояние БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BYPASS, DISPLAY_BYPASS_U_LOAD, 0},  			MIB_INT(snmpBypassULoad),  NULL},			//Состояние БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BYPASS, DISPLAY_BYPASS_I_LOAD, 0},  			MIB_INT(snmpBypassILoad),  NULL},			//Состояние БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BYPASS, DISPLAY_BYPASS_P_LOAD, 0},  			MIB_INT(snmpBypassPLoad),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BYPASS, DISPLAY_BYPASS_TEMPER, 0},  			MIB_INT(snmpBypassTemper),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BYPASS, DISPLAY_BYPASS_U_PRIM, 0},  			MIB_INT(snmpBypassUPrim),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BYPASS, DISPLAY_BYPASS_U_BUS, 0},  			MIB_INT(snmpBypassUBus),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BYPASS, DISPLAY_BYPASS_FLAGS, 0},  			MIB_INT(snmpBypassFlags),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BYPASS, DISPLAY_BYPASS_U_DCIN, 0},  			MIB_INT(snmpBypassUdcin),  NULL},			//Состояние БПС3

											 
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_NUMBER, 1},  				MIB_INT(snmp_bat_number[0]),  	NULL},	//Напряжение батареи №1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_NUMBER, 2},  				MIB_INT(snmp_bat_number[1]),  	NULL},	//Напряжение батареи №2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_VOLTAGE, 1},  				MIB_INT(snmp_bat_voltage[0]),  	NULL},	//Напряжение батареи №1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_VOLTAGE, 2},  				MIB_INT(snmp_bat_voltage[1]),  	NULL},	//Напряжение батареи №2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CURRENT, 1},  				MIB_INT(snmp_bat_current[0]),  	NULL},	//Ток батареи №1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CURRENT, 2},  				MIB_INT(snmp_bat_current[1]),  	NULL},	//Ток батареи №2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_TEMPERATURE, 1},  			MIB_INT(snmp_bat_temperature[0]),	NULL},	//Температура батареи №1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_TEMPERATURE, 2},  			MIB_INT(snmp_bat_temperature[1]),	NULL},	//Температура батареи №2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CAPACITY, 1},  				MIB_INT(snmp_bat_capacity[0]),  	NULL},	//Ёмкость батареи №1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CAPACITY, 2},  				MIB_INT(snmp_bat_capacity[1]),  	NULL},	//Ёмкость батареи №2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CHARGE, 1},  				MIB_INT(snmp_bat_charge[0]),  	NULL},	//Заряд батареи №1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CHARGE, 2},  				MIB_INT(snmp_bat_charge[1]),  	NULL},	//Заряд батареи №2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_STATUS, 1},  				MIB_INT(snmp_bat_status[0]),  	NULL},	//Статус батареи №1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_STATUS, 2},  				MIB_INT(snmp_bat_status[1]),  	NULL},	//Статус батареи №2


//	{ MIB_INTEGER | MIB_ATR_RO, 	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_STAT , 0},					MIB_INT(snmp_spc_stat),     NULL},
//	{ MIB_OCTET_STR | MIB_ATR_RO, 12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_MESSAGE , 0},			MIB_STR(snmp_spc_trap_message),     NULL},
//	{ MIB_INTEGER | MIB_ATR_RO, 	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_VALUE , 0},			MIB_INT(snmp_spc_trap_value),     NULL},


	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SNMP_COMMAND, COMMAND_ANSWER, 0},					MIB_INT(snmp_command),  	snmp_command_execute},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SNMP_COMMAND, COMMAND_PARAMETR, 0},					MIB_INT(snmp_command_parametr),  	NULL},		//номер первого бпса

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 1},  			MIB_INT(snmp_avt_number[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 2},  			MIB_INT(snmp_avt_number[1]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 3},  			MIB_INT(snmp_avt_number[2]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 4},  			MIB_INT(snmp_avt_number[3]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 5},  			MIB_INT(snmp_avt_number[4]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 6},  			MIB_INT(snmp_avt_number[5]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 7},  			MIB_INT(snmp_avt_number[6]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 8},  			MIB_INT(snmp_avt_number[7]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 9},  			MIB_INT(snmp_avt_number[8]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 10},  			MIB_INT(snmp_avt_number[9]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 11},  			MIB_INT(snmp_avt_number[10]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 12},  			MIB_INT(snmp_avt_number[11]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 1},  			MIB_INT(snmp_avt_stat[0]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 2},  			MIB_INT(snmp_avt_stat[1]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 3},  			MIB_INT(snmp_avt_stat[2]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 4},  			MIB_INT(snmp_avt_stat[3]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 5},  			MIB_INT(snmp_avt_stat[4]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 6},  			MIB_INT(snmp_avt_stat[5]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 7},  			MIB_INT(snmp_avt_stat[6]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 8},  			MIB_INT(snmp_avt_stat[7]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 9},  			MIB_INT(snmp_avt_stat[8]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 10},  			MIB_INT(snmp_avt_stat[9]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 11},  			MIB_INT(snmp_avt_stat[10]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 12},  			MIB_INT(snmp_avt_stat[11]),  		NULL},	//Номер БПСа

	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_VVOD_PHASE_A, 0},		MIB_INT(snmp_energy_vvod_phase_a), NULL},	//напряжение фазы A ввода
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_VVOD_PHASE_B, 0},		MIB_INT(snmp_energy_vvod_phase_b), NULL},	//напряжение фазы B ввода
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_VVOD_PHASE_C, 0},		MIB_INT(snmp_energy_vvod_phase_c), NULL},	//напряжение фазы C ввода
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_PES_PHASE_A, 0},		MIB_INT(snmp_energy_pes_phase_a), NULL},	//напряжение фазы A ПЭСа
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_PES_PHASE_B, 0},		MIB_INT(snmp_energy_pes_phase_b), NULL},	//напряжение фазы B ПЭСа
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_PES_PHASE_C, 0},		MIB_INT(snmp_energy_pes_phase_c), NULL},	//напряжение фазы C ПЭСа
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_TOTAL_ENERGY, 0},		MIB_INT(snmp_energy_total_energy), NULL},	//показания счетчика, потребленная энергия
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_CURRENT_ENERGY, 0},		MIB_INT(snmp_energy_current_energy), NULL},	//показания счетчика, потребляемая энергия



	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 1, 2, 4, 1, 0},  MIB_INT(NUMBAT),  NULL},	//количество введенных батарей

	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 1, 2, 7, 4, 0},	     MIB_STR("Novosibirsk, Russia"),     NULL},
	{ MIB_INTEGER, 			13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 1, 2, 7, 5, 0},	     MIB_INT(displayPsuQauntity),     NULL},
 /* { MIB_INTEGER | MIB_ATR_RO,  	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 2, 2, 3, 1, 0},  MIB_INT(plazma_mib),  NULL},
  { MIB_INTEGER | MIB_ATR_RO,  	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 2, 2, 3, 2, 0},  MIB_INT(plazma_mib1),  NULL},
  { MIB_INTEGER,  	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 1, 2, 1, 2, 0},    MIB_INT(LPC_RTC->SEC),    NULL}, */
  
	


	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 1},  			MIB_STR(&snmp_log[0][0]),  	NULL},	//Первое событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 2},  			MIB_STR(&snmp_log[1][0]),  	NULL},	//2-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 3},  			MIB_STR(&snmp_log[2][0]),  	NULL},	//3-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 4},  			MIB_STR(&snmp_log[3][0]),  	NULL},	//4-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 5},  			MIB_STR(&snmp_log[4][0]),  	NULL},	//5-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 6},  			MIB_STR(&snmp_log[5][0]),  	NULL},	//6-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 7},  			MIB_STR(&snmp_log[6][0]),  	NULL},	//7-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 8},  			MIB_STR(&snmp_log[7][0]),  	NULL},	//8-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 9},  			MIB_STR(&snmp_log[8][0]),  	NULL},	//9-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 10},  			MIB_STR(&snmp_log[9][0]),  	NULL},	//10-е событие из журнала

	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 11},  			MIB_STR(&snmp_log[10][0]),  	NULL},	//11-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 12},  			MIB_STR(&snmp_log[11][0]),  	NULL},	//12-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 13},  			MIB_STR(&snmp_log[12][0]),  	NULL},	//13-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 14},  			MIB_STR(&snmp_log[13][0]),  	NULL},	//14-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 15},  			MIB_STR(&snmp_log[14][0]),  	NULL},	//15-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 16},  			MIB_STR(&snmp_log[15][0]),  	NULL},	//16-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 17},  			MIB_STR(&snmp_log[16][0]),  	NULL},	//17-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 18},  			MIB_STR(&snmp_log[17][0]),  	NULL},	//18-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 19},  			MIB_STR(&snmp_log[18][0]),  	NULL},	//19-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 20},  			MIB_STR(&snmp_log[19][0]),  	NULL},	//20-е событие из журнала

	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 21},  			MIB_STR(&snmp_log[20][0]),  	NULL},	//21-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 22},  			MIB_STR(&snmp_log[21][0]),  	NULL},	//22-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 23},  			MIB_STR(&snmp_log[22][0]),  	NULL},	//23-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 24},  			MIB_STR(&snmp_log[23][0]),  	NULL},	//24-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 25},  			MIB_STR(&snmp_log[24][0]),  	NULL},	//25-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 26},  			MIB_STR(&snmp_log[25][0]),  	NULL},	//26-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 27},  			MIB_STR(&snmp_log[26][0]),  	NULL},	//27-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 28},  			MIB_STR(&snmp_log[27][0]),  	NULL},	//28-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 29},  			MIB_STR(&snmp_log[28][0]),  	NULL},	//29-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 30},  			MIB_STR(&snmp_log[29][0]),  	NULL},	//30-е событие из журнала

	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 31},  			MIB_STR(&snmp_log[30][0]),  	NULL},	//31-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 32},  			MIB_STR(&snmp_log[31][0]),  	NULL},	//32-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 33},  			MIB_STR(&snmp_log[32][0]),  	NULL},	//33-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 34},  			MIB_STR(&snmp_log[33][0]),  	NULL},	//34-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 35},  			MIB_STR(&snmp_log[34][0]),  	NULL},	//35-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 36},  			MIB_STR(&snmp_log[35][0]),  	NULL},	//36-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 37},  			MIB_STR(&snmp_log[36][0]),  	NULL},	//37-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 38},  			MIB_STR(&snmp_log[37][0]),  	NULL},	//38-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 39},  			MIB_STR(&snmp_log[38][0]),  	NULL},	//39-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 40},  			MIB_STR(&snmp_log[39][0]),  	NULL},	//40-е событие из журнала

	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 41},  			MIB_STR(&snmp_log[40][0]),  	NULL},	//41-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 42},  			MIB_STR(&snmp_log[41][0]),  	NULL},	//42-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 43},  			MIB_STR(&snmp_log[42][0]),  	NULL},	//43-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 44},  			MIB_STR(&snmp_log[43][0]),  	NULL},	//44-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 45},  			MIB_STR(&snmp_log[44][0]),  	NULL},	//45-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 46},  			MIB_STR(&snmp_log[45][0]),  	NULL},	//46-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 47},  			MIB_STR(&snmp_log[46][0]),  	NULL},	//47-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 48},  			MIB_STR(&snmp_log[47][0]),  	NULL},	//48-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 49},  			MIB_STR(&snmp_log[48][0]),  	NULL},	//49-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 50},  			MIB_STR(&snmp_log[49][0]),  	NULL},	//50-е событие из журнала

	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 51},  			MIB_STR(&snmp_log[50][0]),  	NULL},	//51-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 52},  			MIB_STR(&snmp_log[51][0]),  	NULL},	//52-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 53},  			MIB_STR(&snmp_log[52][0]),  	NULL},	//53-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 54},  			MIB_STR(&snmp_log[53][0]),  	NULL},	//54-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 55},  			MIB_STR(&snmp_log[54][0]),  	NULL},	//55-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 56},  			MIB_STR(&snmp_log[55][0]),  	NULL},	//56-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 57},  			MIB_STR(&snmp_log[56][0]),  	NULL},	//57-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 58},  			MIB_STR(&snmp_log[57][0]),  	NULL},	//58-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 59},  			MIB_STR(&snmp_log[58][0]),  	NULL},	//59-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 60},  			MIB_STR(&snmp_log[59][0]),  	NULL},	//60-е событие из журнала

	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 61},  			MIB_STR(&snmp_log[60][0]),  	NULL},	//61-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 62},  			MIB_STR(&snmp_log[61][0]),  	NULL},	//62-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 63},  			MIB_STR(&snmp_log[62][0]),  	NULL},	//63-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 64},  			MIB_STR(&snmp_log[63][0]),  	NULL},	//64-е событие из журнала



	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMSSOUNDALARMEN, 0},				MIB_INT(snmp_zv_en),  			snmp_zv_on_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMSALARMAUTODISABLE, 0},				MIB_INT(snmp_alarm_auto_disable),	snmp_alarm_auto_disable_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_BAT_TEST_TIME, 0},				MIB_INT(snmp_bat_test_time),		snmp_bat_test_time_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_MAX, 0},						MIB_INT(snmp_u_max),			snmp_u_max_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_MIN, 0},						MIB_INT(snmp_u_min),			snmp_u_min_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_0_GRAD, 0},					MIB_INT(snmp_u_0_grad),			snmp_u_0_grad_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_20_GRAD, 0},					MIB_INT(snmp_u_20_grad),			snmp_u_20_grad_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_SIGN, 0},					MIB_INT(snmp_u_sign),			snmp_u_sign_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_MIN_POWER, 0},				MIB_INT(snmp_u_min_power),		snmp_u_min_power_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_WITHOUT_BAT, 0},				MIB_INT(snmp_u_withouth_bat),		snmp_u_withouth_bat_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_IBK, 0},						MIB_INT(snmp_control_current),	snmp_control_current_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_IZMAX, 0},						MIB_INT(snmp_max_charge_current),	snmp_max_charge_current_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_IMAX, 0},						MIB_INT(snmp_max_current),		snmp_max_current_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_IMIN, 0},						MIB_INT(snmp_min_current),		snmp_min_current_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_UVZ, 0},						MIB_INT(snmp_uvz),				snmp_uvz_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_TZAS, 0},						MIB_INT(snmp_powerup_psu_timeout),	snmp_powerup_psu_timeout_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_TSIGN_BAT, 0},					MIB_INT(snmp_tsign_bat),			snmp_tsign_bat_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_TMAX_BAT, 0},					MIB_INT(snmp_tmax_bat),			snmp_tmax_bat_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_TSIGN_BPS, 0},					MIB_INT(snmp_tsign_bps),			snmp_tsign_bps_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_TMAX_BPS, 0},					MIB_INT(snmp_tmax_bps),			snmp_tmax_bps_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_BAT_PART_ALARM, 0},				MIB_INT(snmp_bat_part_alarm),		snmp_bat_part_alarm_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_POWER_CNT_ADRESS, 0},			MIB_INT(snmp_power_cnt_adress),	snmp_power_cnt_adress_write},		//номер первого бпса 


	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ENTRY_NUMBER, 1},  			MIB_INT(snmp_sk_number[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ENTRY_NUMBER, 2},  			MIB_INT(snmp_sk_number[1]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ENTRY_NUMBER, 3},  			MIB_INT(snmp_sk_number[2]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ENTRY_NUMBER, 4},  			MIB_INT(snmp_sk_number[3]),  	NULL},	//Номер БПСа

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_AKTIVITY, 1},  				MIB_INT(snmp_sk_aktiv[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_AKTIVITY, 2},  				MIB_INT(snmp_sk_aktiv[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_AKTIVITY, 3},  				MIB_INT(snmp_sk_aktiv[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_AKTIVITY, 4},  				MIB_INT(snmp_sk_aktiv[3]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER ,  		  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM_AKTIVITY, 1},  			MIB_INT(snmp_sk_alarm_aktiv[0]),  	snmp_alarm_aktiv_write1},	//Ток БПС1
	{ MIB_INTEGER ,			13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM_AKTIVITY, 2},  			MIB_INT(snmp_sk_alarm_aktiv[1]),  	snmp_alarm_aktiv_write2},	//Ток БПС2
	{ MIB_INTEGER ,			13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM_AKTIVITY, 3},  			MIB_INT(snmp_sk_alarm_aktiv[2]),  	snmp_alarm_aktiv_write3},	//Ток БПС3
	{ MIB_INTEGER ,			13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM_AKTIVITY, 4},  			MIB_INT(snmp_sk_alarm_aktiv[3]),  	snmp_alarm_aktiv_write4},	//Ток БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM, 1},  					MIB_INT(snmp_sk_alarm[0]),  NULL},	//Ток БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM, 2},  					MIB_INT(snmp_sk_alarm[1]),  NULL},	//Ток БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM, 3},  					MIB_INT(snmp_sk_alarm[2]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM, 4},  					MIB_INT(snmp_sk_alarm[3]),  NULL},	//Ток БПС3


	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_ENTRY_NUMBER, 1},  		MIB_INT(snmp_dt_number[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_ENTRY_NUMBER, 2},  		MIB_INT(snmp_dt_number[1]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_ENTRY_NUMBER, 3},  		MIB_INT(snmp_dt_number[2]),  	NULL},	//Номер БПСа

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_TEMPER, 1},  			MIB_INT(snmp_dt_temper[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_TEMPER, 2},  			MIB_INT(snmp_dt_temper[1]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_TEMPER, 3},  			MIB_INT(snmp_dt_temper[2]),  	NULL},	//Номер БПСа

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_ERROR, 1},  			MIB_INT(snmp_dt_error[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_ERROR, 2},  			MIB_INT(snmp_dt_error[1]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_ERROR, 3},  			MIB_INT(snmp_dt_error[2]),  	NULL},	//Номер БПСа


	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 1, 1},    MIB_INT(LPC_RTC->HOUR),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 1, 2},    MIB_INT(LPC_RTC->YEAR),    NULL},				  
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 2, 1},    MIB_INT(LPC_RTC->MIN),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 2, 2},    MIB_INT(LPC_RTC->YEAR),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 3, 1},     MIB_INT(LPC_RTC->SEC),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 3, 2},    MIB_INT(LPC_RTC->MONTH),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 4, 1},     MIB_INT(sysMainsVoltage),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 4, 2},    MIB_INT(LPC_RTC->HOUR),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 5, 1},     MIB_INT(sysMainsVoltage),    NULL},	    //-----------------------------------------------
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 5, 2},    MIB_INT(sysServices),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 6, 1},     MIB_INT(sysMainsVoltage),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 6, 2},    MIB_INT(sysServices),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 7, 1},     MIB_INT(sysMainsVoltage),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 7, 2},    MIB_INT(TestForTableValues),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 8, 1},     MIB_INT(sysMainsVoltage),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 8, 2},    MIB_INT(TestForTableValues),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 9, 1},     MIB_INT(sysMainsVoltage),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 9, 2},    MIB_INT(TestForTableValues),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 10, 1},     MIB_INT(sysMainsVoltage),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 10, 2},    MIB_INT(TestForTableValues),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 11, 1},    MIB_INT(sysMainsVoltage),     NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 11, 2},    MIB_INT(TestForTableValues),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 12, 1},     MIB_INT(sysMainsVoltage),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 12, 2},    MIB_INT(TestForTableValues),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 13, 1},    MIB_INT(sysMainsVoltage),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 13, 2},    MIB_INT(TestForTableValues),    NULL},
	{ MIB_OCTET_STR, 13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 1, 2, 7, 4, 0},  MIB_STR("Proverka sviazi. Проверка связи."),   NULL},

	};
#endif

const int snmp_mib_size = (sizeof(snmp_mib) / sizeof(MIB_ENTRY));

///*----------------------------------------------------------------------------
// *      MIB Callback Functions
// *---------------------------------------------------------------------------*/
//
//static void write_leds (int mode) {
//  /* No action on read access. */
//  if (mode == MIB_WRITE) {
//    LED_out (LedOut);
//  }
//}
//
//static void read_key (int mode) {
//  /* Read ARM Digital Input */
//  if (mode == MIB_READ) {
//    KeyIn = get_button();
//  }
//}
//
//static void upd_display (int mode) {
//  /* Update LCD Module display text. */
//  if (mode == MIB_WRITE) {
//    /* Write access. */
//    LCDupdate = __TRUE;
//  }
//}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
