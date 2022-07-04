#include "MODBUS_RTU.h"
#include "MODBUS_func4.h"
#include "curr_version.h"
#include "main.h"
#include "control.h"

signed short I_reg1, U_reg2, P_reg3, T_reg4;
signed char flags_inv_h[22], flags_inv_l[22];
unsigned char sk1_stat_mb, sk2_stat_mb, sk3_stat_mb, sk4_stat_mb; 

void wr_reg_func4(void){ //Заполнение регистров для функции 4 перед их отправкой
	char i;
	if(NUMBYPASS)
	{
		I_reg1=byps[0]._Iout;			//Рег1
		U_reg2=byps[0]._Uout;			//Рег2
		P_reg3=(char)(byps[0]._Pout/256);			//Рег3
		T_reg4=byps[0]._T;			//Рег4	 
	}
	else 
	{
		I_reg1=load_I;				//Рег1
		U_reg2=load_U;				//Рег2
		P_reg3=(char)(load_P/256);	//Рег3
		T_reg4=0;					//Рег4
	}
	//----------
	for(i=0;i<22;i++){
		flags_inv_l[i]=inv[i]._flags_tm&0xf7;
		flags_inv_h[i]=0;
		if(inv[i]._conn_av_stat) 	flags_inv_h[i]|=1;
		else 						flags_inv_h[i]&=~1;	 
		if(inv[i]._flags_tm_dop&0x0001)	flags_inv_h[i]|=(1<<1);
		else 							flags_inv_h[i]&=~(1<<1);
	}
	//-------------
	sk1_stat_mb=0;
	if(sk_stat[0]==ssON) sk1_stat_mb|=0x0001;
	if(sk_av_stat[0]==sasON) sk1_stat_mb|=0x0002;
	sk2_stat_mb=0;
	if(sk_stat[1]==ssON) sk2_stat_mb|=0x0001;
	if(sk_av_stat[1]==sasON) sk2_stat_mb|=0x0002;
	sk3_stat_mb=0;
	if(sk_stat[2]==ssON) sk3_stat_mb|=0x0001;
	if(sk_av_stat[2]==sasON) sk3_stat_mb|=0x0002;
	sk4_stat_mb=0;
	if(sk_stat[3]==ssON) sk4_stat_mb|=0x0001;
	if(sk_av_stat[3]==sasON) sk4_stat_mb|=0x0002;

}


//----------- Таблица адресов функц 3
//адреса распологаются по порядку, если нет регистра, то &NULL_0, &NULL_0,
unsigned char *const reg_func4 []={
&NULL_0,  //0
&NULL_0,  //0
(unsigned char*)&I_reg1+1,  //1
(unsigned char*)&I_reg1,  	//1
(unsigned char*)&U_reg2+1,  //2
(unsigned char*)&U_reg2,  	//2
(unsigned char*)&P_reg3+1,  //3
(unsigned char*)&P_reg3,  	//3
(unsigned char*)&T_reg4+1,  //4
(unsigned char*)&T_reg4,  	//4
(unsigned char*)&t_ext[0]+1,//5
(unsigned char*)&t_ext[0],  //5
(unsigned char*)&NUMBYPASS+1,//6
(unsigned char*)&NUMBYPASS,  //6
(unsigned char*)&NUMINV+1,
(unsigned char*)&NUMINV, 
&NULL_0,
(unsigned char*)&num_of_wrks_inv, 
(unsigned char*)&dcin_U+1,
(unsigned char*)&dcin_U, 
(unsigned char*)&byps[0]._UinACprim+1,	//10
(unsigned char*)&byps[0]._UinACprim,	//10
//Инвертор №1
(unsigned char*)&inv[0]._Uout+1, //11 
(unsigned char*)&inv[0]._Uout, 	 //11
(unsigned char*)&inv[0]._Iout+1,
(unsigned char*)&inv[0]._Iout,
(unsigned char*)&inv[0]._T+1,
(unsigned char*)&inv[0]._T,
(unsigned char*)&inv[0]._Pout+1,
(unsigned char*)&inv[0]._Pout,
(unsigned char*)&inv[0]._Uacin+1,	 //15
(unsigned char*)&inv[0]._Uacin,		 //15
(unsigned char*)&inv[0]._Uload+1,
(unsigned char*)&inv[0]._Uload,
(unsigned char*)&flags_inv_h[0],  //17
(unsigned char*)&flags_inv_l[0],  //17
(unsigned char*)&inv[0]._Udcin+1,
(unsigned char*)&inv[0]._Udcin,
&NULL_0,  //19
&NULL_0,  //19
&NULL_0,  //20
&NULL_0,  //20 
//Инвертор №2
(unsigned char*)&inv[1]._Uout+1, //21 
(unsigned char*)&inv[1]._Uout, 	 //21
(unsigned char*)&inv[1]._Iout+1,
(unsigned char*)&inv[1]._Iout,
(unsigned char*)&inv[1]._T+1,
(unsigned char*)&inv[1]._T,
(unsigned char*)&inv[1]._Pout+1,
(unsigned char*)&inv[1]._Pout,
(unsigned char*)&inv[1]._Uacin+1,	 
(unsigned char*)&inv[1]._Uacin,		 
(unsigned char*)&inv[1]._Uload+1,
(unsigned char*)&inv[1]._Uload,
(unsigned char*)&flags_inv_h[1],  
(unsigned char*)&flags_inv_l[1],  
(unsigned char*)&inv[1]._Udcin+1,
(unsigned char*)&inv[1]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //30
&NULL_0,  //30 
//Инвертор №3
(unsigned char*)&inv[2]._Uout+1, //31 
(unsigned char*)&inv[2]._Uout, 	 //31
(unsigned char*)&inv[2]._Iout+1,
(unsigned char*)&inv[2]._Iout,
(unsigned char*)&inv[2]._T+1,
(unsigned char*)&inv[2]._T,
(unsigned char*)&inv[2]._Pout+1,
(unsigned char*)&inv[2]._Pout,
(unsigned char*)&inv[2]._Uacin+1,	 
(unsigned char*)&inv[2]._Uacin,		 
(unsigned char*)&inv[2]._Uload+1,
(unsigned char*)&inv[2]._Uload,
(unsigned char*)&flags_inv_h[2],  
(unsigned char*)&flags_inv_l[2],  
(unsigned char*)&inv[2]._Udcin+1,
(unsigned char*)&inv[2]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //40
&NULL_0,  //40 
//Инвертор №4
(unsigned char*)&inv[3]._Uout+1, //41 
(unsigned char*)&inv[3]._Uout, 	 //41
(unsigned char*)&inv[3]._Iout+1,
(unsigned char*)&inv[3]._Iout,
(unsigned char*)&inv[3]._T+1,
(unsigned char*)&inv[3]._T,
(unsigned char*)&inv[3]._Pout+1,
(unsigned char*)&inv[3]._Pout,
(unsigned char*)&inv[3]._Uacin+1,	 
(unsigned char*)&inv[3]._Uacin,		 
(unsigned char*)&inv[3]._Uload+1,
(unsigned char*)&inv[3]._Uload,
(unsigned char*)&flags_inv_h[3],  
(unsigned char*)&flags_inv_l[3],  
(unsigned char*)&inv[3]._Udcin+1,
(unsigned char*)&inv[3]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //50
&NULL_0,  //50
//Инвертор №5
(unsigned char*)&inv[4]._Uout+1, //51 
(unsigned char*)&inv[4]._Uout, 	 //51
(unsigned char*)&inv[4]._Iout+1,
(unsigned char*)&inv[4]._Iout,
(unsigned char*)&inv[4]._T+1,
(unsigned char*)&inv[4]._T,
(unsigned char*)&inv[4]._Pout+1,
(unsigned char*)&inv[4]._Pout,
(unsigned char*)&inv[4]._Uacin+1,	 
(unsigned char*)&inv[4]._Uacin,		 
(unsigned char*)&inv[4]._Uload+1,
(unsigned char*)&inv[4]._Uload,
(unsigned char*)&flags_inv_h[4],  
(unsigned char*)&flags_inv_l[4],  
(unsigned char*)&inv[4]._Udcin+1,
(unsigned char*)&inv[4]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //60
&NULL_0,  //60
//Инвертор №6
(unsigned char*)&inv[5]._Uout+1, //61 
(unsigned char*)&inv[5]._Uout, 	 //61
(unsigned char*)&inv[5]._Iout+1,
(unsigned char*)&inv[5]._Iout,
(unsigned char*)&inv[5]._T+1,
(unsigned char*)&inv[5]._T,
(unsigned char*)&inv[5]._Pout+1,
(unsigned char*)&inv[5]._Pout,
(unsigned char*)&inv[5]._Uacin+1,	 
(unsigned char*)&inv[5]._Uacin,		 
(unsigned char*)&inv[5]._Uload+1,
(unsigned char*)&inv[5]._Uload,
(unsigned char*)&flags_inv_h[5],  
(unsigned char*)&flags_inv_l[5],  
(unsigned char*)&inv[5]._Udcin+1,
(unsigned char*)&inv[5]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //70
&NULL_0,  //70
//Инвертор №7
(unsigned char*)&inv[6]._Uout+1, //71 
(unsigned char*)&inv[6]._Uout, 	 //71
(unsigned char*)&inv[6]._Iout+1,
(unsigned char*)&inv[6]._Iout,
(unsigned char*)&inv[6]._T+1,
(unsigned char*)&inv[6]._T,
(unsigned char*)&inv[6]._Pout+1,
(unsigned char*)&inv[6]._Pout,
(unsigned char*)&inv[6]._Uacin+1,	 
(unsigned char*)&inv[6]._Uacin,		 
(unsigned char*)&inv[6]._Uload+1,
(unsigned char*)&inv[6]._Uload,
(unsigned char*)&flags_inv_h[6],  
(unsigned char*)&flags_inv_l[6],  
(unsigned char*)&inv[6]._Udcin+1,
(unsigned char*)&inv[6]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //80
&NULL_0,  //80
//Инвертор №8
(unsigned char*)&inv[7]._Uout+1, //81 
(unsigned char*)&inv[7]._Uout, 	 //81
(unsigned char*)&inv[7]._Iout+1,
(unsigned char*)&inv[7]._Iout,
(unsigned char*)&inv[7]._T+1,
(unsigned char*)&inv[7]._T,
(unsigned char*)&inv[7]._Pout+1,
(unsigned char*)&inv[7]._Pout,
(unsigned char*)&inv[7]._Uacin+1,	 
(unsigned char*)&inv[7]._Uacin,		 
(unsigned char*)&inv[7]._Uload+1,
(unsigned char*)&inv[7]._Uload,
(unsigned char*)&flags_inv_h[7],  
(unsigned char*)&flags_inv_l[7],  
(unsigned char*)&inv[7]._Udcin+1,
(unsigned char*)&inv[7]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //90
&NULL_0,  //90
//Инвертор №9
(unsigned char*)&inv[8]._Uout+1, //91 
(unsigned char*)&inv[8]._Uout, 	 //91
(unsigned char*)&inv[8]._Iout+1,
(unsigned char*)&inv[8]._Iout,
(unsigned char*)&inv[8]._T+1,
(unsigned char*)&inv[8]._T,
(unsigned char*)&inv[8]._Pout+1,
(unsigned char*)&inv[8]._Pout,
(unsigned char*)&inv[8]._Uacin+1,	 
(unsigned char*)&inv[8]._Uacin,		 
(unsigned char*)&inv[8]._Uload+1,
(unsigned char*)&inv[8]._Uload,
(unsigned char*)&flags_inv_h[8],  
(unsigned char*)&flags_inv_l[8],  
(unsigned char*)&inv[8]._Udcin+1,
(unsigned char*)&inv[8]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //100
&NULL_0,  //100
//Инвертор №10
(unsigned char*)&inv[9]._Uout+1, //101 
(unsigned char*)&inv[9]._Uout, 	 //101
(unsigned char*)&inv[9]._Iout+1,
(unsigned char*)&inv[9]._Iout,
(unsigned char*)&inv[9]._T+1,
(unsigned char*)&inv[9]._T,
(unsigned char*)&inv[9]._Pout+1,
(unsigned char*)&inv[9]._Pout,
(unsigned char*)&inv[9]._Uacin+1,	 
(unsigned char*)&inv[9]._Uacin,		 
(unsigned char*)&inv[9]._Uload+1,
(unsigned char*)&inv[9]._Uload,
(unsigned char*)&flags_inv_h[9],  
(unsigned char*)&flags_inv_l[9],  
(unsigned char*)&inv[9]._Udcin+1,
(unsigned char*)&inv[9]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //110
&NULL_0,  //110
//Инвертор №11
(unsigned char*)&inv[10]._Uout+1, //111 
(unsigned char*)&inv[10]._Uout, 	 //111
(unsigned char*)&inv[10]._Iout+1,
(unsigned char*)&inv[10]._Iout,
(unsigned char*)&inv[10]._T+1,
(unsigned char*)&inv[10]._T,
(unsigned char*)&inv[10]._Pout+1,
(unsigned char*)&inv[10]._Pout,
(unsigned char*)&inv[10]._Uacin+1,	 
(unsigned char*)&inv[10]._Uacin,		 
(unsigned char*)&inv[10]._Uload+1,
(unsigned char*)&inv[10]._Uload,
(unsigned char*)&flags_inv_h[10],  
(unsigned char*)&flags_inv_l[10],  
(unsigned char*)&inv[10]._Udcin+1,
(unsigned char*)&inv[10]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //120
&NULL_0,  //120
//Инвертор №12
(unsigned char*)&inv[11]._Uout+1, //121 
(unsigned char*)&inv[11]._Uout, 	 //121
(unsigned char*)&inv[11]._Iout+1,
(unsigned char*)&inv[11]._Iout,
(unsigned char*)&inv[11]._T+1,
(unsigned char*)&inv[11]._T,
(unsigned char*)&inv[11]._Pout+1,
(unsigned char*)&inv[11]._Pout,
(unsigned char*)&inv[11]._Uacin+1,	 
(unsigned char*)&inv[11]._Uacin,		 
(unsigned char*)&inv[11]._Uload+1,
(unsigned char*)&inv[11]._Uload,
(unsigned char*)&flags_inv_h[11],  
(unsigned char*)&flags_inv_l[11],  
(unsigned char*)&inv[11]._Udcin+1,
(unsigned char*)&inv[11]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //130
&NULL_0,  //130
//Инвертор №13
(unsigned char*)&inv[12]._Uout+1, //131 
(unsigned char*)&inv[12]._Uout, 	 //131
(unsigned char*)&inv[12]._Iout+1,
(unsigned char*)&inv[12]._Iout,
(unsigned char*)&inv[12]._T+1,
(unsigned char*)&inv[12]._T,
(unsigned char*)&inv[12]._Pout+1,
(unsigned char*)&inv[12]._Pout,
(unsigned char*)&inv[12]._Uacin+1,	 
(unsigned char*)&inv[12]._Uacin,		 
(unsigned char*)&inv[12]._Uload+1,
(unsigned char*)&inv[12]._Uload,
(unsigned char*)&flags_inv_h[12],  
(unsigned char*)&flags_inv_l[12],  
(unsigned char*)&inv[12]._Udcin+1,
(unsigned char*)&inv[12]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //140
&NULL_0,  //140 
//Инвертор №14
(unsigned char*)&inv[13]._Uout+1, //141 
(unsigned char*)&inv[13]._Uout, 	 //141
(unsigned char*)&inv[13]._Iout+1,
(unsigned char*)&inv[13]._Iout,
(unsigned char*)&inv[13]._T+1,
(unsigned char*)&inv[13]._T,
(unsigned char*)&inv[13]._Pout+1,
(unsigned char*)&inv[13]._Pout,
(unsigned char*)&inv[13]._Uacin+1,	 
(unsigned char*)&inv[13]._Uacin,		 
(unsigned char*)&inv[13]._Uload+1,
(unsigned char*)&inv[13]._Uload,
(unsigned char*)&flags_inv_h[13],  
(unsigned char*)&flags_inv_l[13],  
(unsigned char*)&inv[13]._Udcin+1,
(unsigned char*)&inv[13]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //150
&NULL_0,  //150
//Инвертор №15
(unsigned char*)&inv[14]._Uout+1, 	//151 
(unsigned char*)&inv[14]._Uout,  	//151
(unsigned char*)&inv[14]._Iout+1,
(unsigned char*)&inv[14]._Iout,
(unsigned char*)&inv[14]._T+1,
(unsigned char*)&inv[14]._T,
(unsigned char*)&inv[14]._Pout+1,
(unsigned char*)&inv[14]._Pout,
(unsigned char*)&inv[14]._Uacin+1,	 
(unsigned char*)&inv[14]._Uacin,		 
(unsigned char*)&inv[14]._Uload+1,
(unsigned char*)&inv[14]._Uload,
(unsigned char*)&flags_inv_h[14],  
(unsigned char*)&flags_inv_l[14],  
(unsigned char*)&inv[14]._Udcin+1,
(unsigned char*)&inv[14]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //160
&NULL_0,  //160
//Инвертор №16
(unsigned char*)&inv[15]._Uout+1, 	//161 
(unsigned char*)&inv[15]._Uout,  	//161
(unsigned char*)&inv[15]._Iout+1,
(unsigned char*)&inv[15]._Iout,
(unsigned char*)&inv[15]._T+1,
(unsigned char*)&inv[15]._T,
(unsigned char*)&inv[15]._Pout+1,
(unsigned char*)&inv[15]._Pout,
(unsigned char*)&inv[15]._Uacin+1,	 
(unsigned char*)&inv[15]._Uacin,		 
(unsigned char*)&inv[15]._Uload+1,
(unsigned char*)&inv[15]._Uload,
(unsigned char*)&flags_inv_h[15],  
(unsigned char*)&flags_inv_l[15],  
(unsigned char*)&inv[15]._Udcin+1,
(unsigned char*)&inv[15]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //170
&NULL_0,  //170
//Инвертор №17
(unsigned char*)&inv[16]._Uout+1, 	//171 
(unsigned char*)&inv[16]._Uout,  	//171
(unsigned char*)&inv[16]._Iout+1,
(unsigned char*)&inv[16]._Iout,
(unsigned char*)&inv[16]._T+1,
(unsigned char*)&inv[16]._T,
(unsigned char*)&inv[16]._Pout+1,
(unsigned char*)&inv[16]._Pout,
(unsigned char*)&inv[16]._Uacin+1,	 
(unsigned char*)&inv[16]._Uacin,		 
(unsigned char*)&inv[16]._Uload+1,
(unsigned char*)&inv[16]._Uload,
(unsigned char*)&flags_inv_h[16],  
(unsigned char*)&flags_inv_l[16],  
(unsigned char*)&inv[16]._Udcin+1,
(unsigned char*)&inv[16]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //180
&NULL_0,  //180
//Инвертор №18
(unsigned char*)&inv[17]._Uout+1, 	//181 
(unsigned char*)&inv[17]._Uout,  	//181
(unsigned char*)&inv[17]._Iout+1,
(unsigned char*)&inv[17]._Iout,
(unsigned char*)&inv[17]._T+1,
(unsigned char*)&inv[17]._T,
(unsigned char*)&inv[17]._Pout+1,
(unsigned char*)&inv[17]._Pout,
(unsigned char*)&inv[17]._Uacin+1,	 
(unsigned char*)&inv[17]._Uacin,		 
(unsigned char*)&inv[17]._Uload+1,
(unsigned char*)&inv[17]._Uload,
(unsigned char*)&flags_inv_h[17],  
(unsigned char*)&flags_inv_l[17],  
(unsigned char*)&inv[17]._Udcin+1,
(unsigned char*)&inv[17]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //190
&NULL_0,  //190
//Инвертор №19
(unsigned char*)&inv[18]._Uout+1, 	//191 
(unsigned char*)&inv[18]._Uout,  	//191
(unsigned char*)&inv[18]._Iout+1,
(unsigned char*)&inv[18]._Iout,
(unsigned char*)&inv[18]._T+1,
(unsigned char*)&inv[18]._T,
(unsigned char*)&inv[18]._Pout+1,
(unsigned char*)&inv[18]._Pout,
(unsigned char*)&inv[18]._Uacin+1,	 
(unsigned char*)&inv[18]._Uacin,		 
(unsigned char*)&inv[18]._Uload+1,
(unsigned char*)&inv[18]._Uload,
(unsigned char*)&flags_inv_h[18],  
(unsigned char*)&flags_inv_l[18],  
(unsigned char*)&inv[18]._Udcin+1,
(unsigned char*)&inv[18]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //200
&NULL_0,  //200
//Инвертор №20
(unsigned char*)&inv[19]._Uout+1, 	//201 
(unsigned char*)&inv[19]._Uout,  	//201
(unsigned char*)&inv[19]._Iout+1,
(unsigned char*)&inv[19]._Iout,
(unsigned char*)&inv[19]._T+1,
(unsigned char*)&inv[19]._T,
(unsigned char*)&inv[19]._Pout+1,
(unsigned char*)&inv[19]._Pout,
(unsigned char*)&inv[19]._Uacin+1,	 
(unsigned char*)&inv[19]._Uacin,		 
(unsigned char*)&inv[19]._Uload+1,
(unsigned char*)&inv[19]._Uload,
(unsigned char*)&flags_inv_h[19],  
(unsigned char*)&flags_inv_l[19],  
(unsigned char*)&inv[19]._Udcin+1,
(unsigned char*)&inv[19]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //210
&NULL_0,  //210
//Инвертор №21
(unsigned char*)&inv[20]._Uout+1, 	//211 
(unsigned char*)&inv[20]._Uout,  	//211
(unsigned char*)&inv[20]._Iout+1,
(unsigned char*)&inv[20]._Iout,
(unsigned char*)&inv[20]._T+1,
(unsigned char*)&inv[20]._T,
(unsigned char*)&inv[20]._Pout+1,
(unsigned char*)&inv[20]._Pout,
(unsigned char*)&inv[20]._Uacin+1,	 
(unsigned char*)&inv[20]._Uacin,		 
(unsigned char*)&inv[20]._Uload+1,
(unsigned char*)&inv[20]._Uload,
(unsigned char*)&flags_inv_h[20],  
(unsigned char*)&flags_inv_l[20],  
(unsigned char*)&inv[20]._Udcin+1,
(unsigned char*)&inv[20]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //220
&NULL_0,  //220
//Инвертор №22
(unsigned char*)&inv[21]._Uout+1, 	//221 
(unsigned char*)&inv[21]._Uout,  	//221
(unsigned char*)&inv[21]._Iout+1,
(unsigned char*)&inv[21]._Iout,
(unsigned char*)&inv[21]._T+1,
(unsigned char*)&inv[21]._T,
(unsigned char*)&inv[21]._Pout+1,
(unsigned char*)&inv[21]._Pout,
(unsigned char*)&inv[21]._Uacin+1,	 
(unsigned char*)&inv[21]._Uacin,		 
(unsigned char*)&inv[21]._Uload+1,
(unsigned char*)&inv[21]._Uload,
(unsigned char*)&flags_inv_h[21],  
(unsigned char*)&flags_inv_l[21],  
(unsigned char*)&inv[21]._Udcin+1,
(unsigned char*)&inv[21]._Udcin,
&NULL_0,  //
&NULL_0,  //
&NULL_0,  //230
&NULL_0,  //230
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
&NULL_0,//240
&NULL_0,//240
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
&NULL_0,//250
&NULL_0,//250
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
&NULL_0,  //260
&NULL_0,  //260
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
&NULL_0,//270
&NULL_0,//270
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
&NULL_0,//280
&NULL_0,//280
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
&NULL_0,//290
&NULL_0,//290
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
&NULL_0,//300
&NULL_0,//300
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
&NULL_0,  //310
&NULL_0,  //310
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
&NULL_0,//320
&NULL_0,//320
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
&NULL_0,//330
&NULL_0,//330
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
&NULL_0,//340
&NULL_0,//340
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
&NULL_0,//350
&NULL_0,//350
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
&NULL_0,  //360
&NULL_0,  //360
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
&NULL_0,//370
&NULL_0,//370
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
&NULL_0,//380
&NULL_0,//380
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
&NULL_0,//390
&NULL_0,//390
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
(unsigned char*)&f_out+1,//400
(unsigned char*)&f_out,//400
(unsigned char*)&byps[0]._Uout+1,//401
(unsigned char*)&byps[0]._Uout,//401
(unsigned char*)&byps[1]._Uout+1,//402
(unsigned char*)&byps[1]._Uout,//402
(unsigned char*)&byps[2]._Uout+1,//403
(unsigned char*)&byps[2]._Uout,//403
(unsigned char*)&byps[0]._Iout+1,//404
(unsigned char*)&byps[0]._Iout,//404
(unsigned char*)&byps[1]._Iout+1,//405
(unsigned char*)&byps[1]._Iout,//405
(unsigned char*)&byps[2]._Iout+1,//406
(unsigned char*)&byps[2]._Iout,//406
(unsigned char*)&byps[0]._Pout+1,//407
(unsigned char*)&byps[0]._Pout,//407
(unsigned char*)&byps[1]._Pout+1,//408
(unsigned char*)&byps[1]._Pout,//408
(unsigned char*)&byps[2]._Pout+1,//409
(unsigned char*)&byps[2]._Pout,//409
&NULL_0,//410
(unsigned char*)&byps[0]._T,//410
&NULL_0,//411
(unsigned char*)&byps[1]._T,//411
&NULL_0,//412
(unsigned char*)&byps[2]._T,//412
(unsigned char*)&byps[0]._UinACprim+1,//413
(unsigned char*)&byps[0]._UinACprim,//413
(unsigned char*)&byps[1]._UinACprim+1,//414
(unsigned char*)&byps[1]._UinACprim,//414
(unsigned char*)&byps[2]._UinACprim+1,//415
(unsigned char*)&byps[2]._UinACprim,//415
(unsigned char*)&byps[0]._UinACinvbus+1,//416
(unsigned char*)&byps[0]._UinACinvbus,//416
(unsigned char*)&byps[1]._UinACinvbus+1,//417
(unsigned char*)&byps[1]._UinACinvbus,//417
(unsigned char*)&byps[2]._UinACinvbus+1,//418
(unsigned char*)&byps[2]._UinACinvbus,//418
&NULL_0,//419
(unsigned char*)&byps[0]._flags,//419
&NULL_0,//420
(unsigned char*)&byps[1]._flags,//420
&NULL_0,//421
(unsigned char*)&byps[2]._flags,//421
//   1000000 работа от инверторов(0 - от сети)
//   0100000 приоритет инверторы (0 - сеть)
//	 0000100 температура больше 80 Г.ц.
//	 0000010 температура больше 70 Г.ц.
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
&NULL_0,//430
&NULL_0,//430
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
&NULL_0,//440
&NULL_0,//440
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
&NULL_0,//450
&NULL_0,//450

&NULL_0,
(unsigned char*)&sk1_stat_mb, //Рег451	Состояние  сухого контакта №1, (нулевой бит - физическое состояние, 1 - замкнут, 0 - разомкнут, первый бит - аварийность, 1 - авария, 0 - норма)
&NULL_0,
(unsigned char*)&sk2_stat_mb, //Рег452	Состояние  сухого контакта №1, (нулевой бит - физическое состояние, 1 - замкнут, 0 - разомкнут, первый бит - аварийность, 1 - авария, 0 - норма)
&NULL_0,
(unsigned char*)&sk3_stat_mb, //Рег453	Состояние  сухого контакта №1, (нулевой бит - физическое состояние, 1 - замкнут, 0 - разомкнут, первый бит - аварийность, 1 - авария, 0 - норма)
&NULL_0,
(unsigned char*)&sk4_stat_mb, //Рег454	Состояние  сухого контакта №1, (нулевой бит - физическое состояние, 1 - замкнут, 0 - разомкнут, первый бит - аварийность, 1 - авария, 0 - норма)
(unsigned char*)&sk_spec_reg+1,	//Рег455	Спецрегистр состояния СК
(unsigned char*)&sk_spec_reg,   //Рег455
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,//460
&NULL_0,//460
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
(unsigned char*)&HARDVARE_VERSION+1,  //Рег 470  	аппаратная версия
(unsigned char*)&HARDVARE_VERSION,	  //Рег 470  	аппаратная версия
(unsigned char*)&SOFT_VERSION+1,	  //Рег 471  	версия ПО
(unsigned char*)&SOFT_VERSION,		  //Рег 471  	версия ПО
(unsigned char*)&BUILD+1,			  //Рег 472  	номер компиляции ПО
(unsigned char*)&BUILD,				  //Рег 472  	номер компиляции ПО
(unsigned char*)&BUILD_YEAR+1,		  //Рег 473  	год	компиляции ПО
(unsigned char*)&BUILD_YEAR,		  //Рег 473  	год	компиляции ПО
(unsigned char*)&BUILD_MONTH+1,		  //Рег 474  	месяц компиляции ПО
(unsigned char*)&BUILD_MONTH,		  //Рег 474  	месяц компиляции ПО
(unsigned char*)&BUILD_DAY+1,		  //Рег 475  	день компиляции ПО
(unsigned char*)&BUILD_DAY,			  //Рег 475  	день компиляции ПО
(unsigned char*)&AUSW_MAIN_NUMBER+1,  //Рег 476	   Зав.номер изделия
(unsigned char*)&AUSW_MAIN_NUMBER,
(unsigned char*)&AUSW_MAIN_NUMBER+3,  //Рег 477	   Зав.номер изделия
(unsigned char*)&AUSW_MAIN_NUMBER+2,  //Рег 477

&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,


};
//количество байт в массиве, включая нулевой байт:= (максимальный номер регистра+1)*2
// задать в .h #define MODBUS_FUNC_4_LENGTH 956 

