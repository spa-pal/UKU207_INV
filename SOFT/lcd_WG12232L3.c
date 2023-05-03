//#include "global_define.h"
#include "lcd_AGM1232_uku207_3.h"
#include "LPC17xx.H"
#include "main.h"
#include "common_func.h"
char lcd_bitmap2[512];

const char bit_mask_const[8]={0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

//-----------------------------------------------
void bitmap_hndl_2(void)
{
char row, row_;
signed char col, col_;
short ptr,ptr_;

char i;
short ii;

/*char bh2_plazma[15];

bh2_plazma[0]=3;
bh2_plazma[1]=130;
bh2_plazma[2]=4;
bh2_plazma[3]=15;
bh2_plazma[4]=7;
bh2_plazma[5]=131;
bh2_plazma[6]=5;
bh2_plazma[3]=115;
bh2_plazma[8]=33;
bh2_plazma[9]=30;
bh2_plazma[10]=3;
bh2_plazma[11]=130;
bh2_plazma[12]=4;
bh2_plazma[13]=15;
bh2_plazma[14]=7;*/

/*for(i=0;i<32;i++)
	{ 
	lcd_bitmap[i]=(i&0x01) ? 0xff : 0x00;
	}
for(ii=32;ii<64;ii++)
	{ 
	lcd_bitmap[ii]=(ii&0x01) ? 0x55 : 0x02;
	}
for(ii=122;ii<154;ii++)
	{ 
	lcd_bitmap[ii]=(ii&0x01) ? 0xff : 0x00;
	}
for(ii=244;ii<276;ii++)
	{ 
	lcd_bitmap[ii]=(ii&0x01) ? 0x55 : 0xaa;
	}  

for(ii=366;ii<398;ii++)
	{ 
	lcd_bitmap[ii]=(ii&0x01) ? 0x33 : 0xcc;
	} */

/*
lcd_buffer[0]='A';
lcd_buffer[1]='B';
lcd_buffer[2]='B'; */

//lcd_buffer[20]='A';

bitmap_hndl();
/*
lcd_bitmap[0]=0xf2;
lcd_bitmap[1]=0x02;
lcd_bitmap[2]=0x04;
lcd_bitmap[3]=0x08;
lcd_bitmap[4]=0x10;
lcd_bitmap[5]=0x20;
lcd_bitmap[6]=0x30;
lcd_bitmap[7]=0x40;
lcd_bitmap[8]=0xff;
lcd_bitmap[9]=0x80;
lcd_bitmap[10]=0x40;
lcd_bitmap[11]=0x20;
lcd_bitmap[12]=0x10;
lcd_bitmap[13]=0x08;
lcd_bitmap[14]=0x04;
lcd_bitmap[15]=0xf3;
lcd_bitmap[16]=0x33;*/

/*for(ii=0;ii<512;ii++)
	{ 
	lcd_bitmap2[ii]=(char)ii;
	}*/
/*
for(i=0;i<16;i++)
	{ 
	lcd_bitmap2[i]=lcd_bitmap[i];
	}*/
//LPC_GPIO0->FIODIR|=(1<<8);
//LPC_GPIO0->FIOSET|=(1<<8);
/*
lcd_bitmap[0]=0xff;
lcd_bitmap[1]=0xff;

lcd_bitmap[7]=0xff;
lcd_bitmap[8]=0xf4;
lcd_bitmap[9]=0xf4;	*/

/*lcd_bitmap[122]=0xff;
lcd_bitmap[123]=0xff;

lcd_bitmap[129]=0xff;
lcd_bitmap[130]=0xf5;
lcd_bitmap[131]=0xf5;*/

/*
lcd_bitmap[244]=0xff;
lcd_bitmap[245]=0xff;

lcd_bitmap[249]=0xfe;
lcd_bitmap[250]=0xfd;
lcd_bitmap[251]=0xff;
lcd_bitmap[252]=0xf9;
lcd_bitmap[253]=0xf9; */

/*
lcd_bitmap[366]=0xff;
lcd_bitmap[367]=0xff;

lcd_bitmap[371]=0xfe;
lcd_bitmap[372]=0xfd;
lcd_bitmap[373]=0xff;
lcd_bitmap[374]=0xf7;
lcd_bitmap[375]=0xf7;*/
/*
bh2_plazma[0]=lcd_bitmap[122+6];
bh2_plazma[1]=lcd_bitmap[123+6];
bh2_plazma[2]=lcd_bitmap[124+6];
bh2_plazma[3]=lcd_bitmap[125+6];
bh2_plazma[4]=lcd_bitmap[126+6]; */
//bh2_plazma[5]=lcd_bitmap[127];

for(row=0;row<4;row++)	 //по 4 текстовым строкам
	{
	for(col=0;col<15;col++)	 //по 16 8-битным столбикам
		{
		for(row_=0;row_<8;row_++)	 //по каждому из 8 пикселей в строке
			{
			ptr_=((short)row*128)+((short)row_*16)+((short)col);
	
			lcd_bitmap2[ptr_]=0;//0x0f+row_;//row+16;

		   	ptr=((short)row*122)+((short)col*8);

			if((row==1)&&(col==0))
				{
/*				bh2_plazma[10]=row;
				bh2_plazma[11]=col;
				bh2_plazma[12]=(char)ptr;
				bh2_plazma[13]=(char)(ptr>>8);
				bh2_plazma[5]=lcd_bitmap[128+0];
				bh2_plazma[6]=lcd_bitmap[128+1];
				bh2_plazma[7]=lcd_bitmap[128+2];
				bh2_plazma[8]=lcd_bitmap[128+3];
				bh2_plazma[9]=lcd_bitmap[128+4];  */
				}
			for(col_=7;col_>=0;col_--)	 //по каждому из 8 пикселей в этом столбике
				{
				//ptr=((short)row*128)+((short)col*8)+((short)col_);
				//if(ptr_==384) lcd_bitmap2[ptr_]|=bit_mask_const[col_];
				if(lcd_bitmap[ptr]&0x01) lcd_bitmap2[ptr_]|=bit_mask_const[col_];
				lcd_bitmap[ptr++]>>=1;
				}
			}
		}
	}

/*for(ii=0;ii<128;ii++)
	{ 
	lcd_bitmap2[ii]=0;
	}

lcd_bitmap2[5]=0x01;
lcd_bitmap2[7]=0x05;
lcd_bitmap2[9]=0x15;
lcd_bitmap2[11]=0x55;
lcd_bitmap2[12]=0x01;
lcd_bitmap2[13]=0x55;

lcd_bitmap2[5+32]=0x55;
lcd_bitmap2[7+32]=0x55;
lcd_bitmap2[9+32]=0x55;
lcd_bitmap2[11+32]=0x55;
lcd_bitmap2[13+32]=0x55;*/


/*
lcd_bitmap2[5+64]=bh2_plazma[0];
lcd_bitmap2[7+64]=bh2_plazma[1];
lcd_bitmap2[9+64]=bh2_plazma[2];
lcd_bitmap2[11+64]=bh2_plazma[3];
lcd_bitmap2[13+64]=bh2_plazma[4];
lcd_bitmap2[5+96]=bh2_plazma[5];
lcd_bitmap2[7+96]=bh2_plazma[6];
lcd_bitmap2[9+96]=bh2_plazma[7];
lcd_bitmap2[11+96]=bh2_plazma[8];
lcd_bitmap2[13+96]=bh2_plazma[9];
lcd_bitmap2[5+112]=bh2_plazma[10];
lcd_bitmap2[7+112]=bh2_plazma[11];
lcd_bitmap2[9+112]=bh2_plazma[12];
lcd_bitmap2[11+112]=bh2_plazma[13];
lcd_bitmap2[13+112]=bh2_plazma[14];	*/
//lcd_bitmap2[0]|=0xff;
//lcd_bitmap2[128]|=0xff;
//lcd_bitmap2[256]|=0xff;
//lcd_bitmap2[384]|=0xff;
//LPC_GPIO0->FIOCLR|=(1<<8);
}

//-----------------------------------------------
void strob_us(short in)
{
LPC_GPIO0->FIODIR|=(1<<8);
LPC_GPIO0->FIOSET|=(1<<8);
delay_us(in);
LPC_GPIO0->FIOCLR|=(1<<8);
}

//-----------------------------------------------
void strob_ms(short in)
{
LPC_GPIO0->FIODIR|=(1<<8);
LPC_GPIO0->FIOSET|=(1<<8);
delay_ms(in);
LPC_GPIO0->FIOCLR|=(1<<8);
}

//-----------------------------------------------
void lcd_debug_start(void)
{


lcd_init();
lcd_clear(); 
}

//-----------------------------------------------
void lcd_debug_loop(void)
{

delay_ms(100);
//strob_ms(10);
bgnd_par("1      АВАРИЯ!     2",
		 "3    Батарея №#    4",
		 "5   не подключена  6",
		 "56  Инициализация  8");
//bitmap_hndl();

int2lcd('#',123,0);

lcd_out(0);
}


//-----------------------------------------------
void lcd1_chk(void)
{
LPC_GPIO1->FIODIR&=(~((1<<D0)|(1<<D1)|(1<<D2)|(1<<D3)|(1<<D4)|(1<<D5)|(1<<D6)|(1<<D7)));
LPC_GPIO1->FIOSET|=(1<<RW);

__nop();
__nop();
__nop();
__nop();
__nop(); 
__nop();
__nop();
__nop();
__nop();
__nop(); 

LPC_GPIO1->FIOCLR|=(1<<A0);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop(); 

LPC_GPIO1->FIOSET|=(1<<E1);

delay_us(1);

chk1:

if(LPC_GPIO1->FIOPIN&(1<<D7)) goto chk1;

}

//-----------------------------------------------
void lcd2_chk(void)
{
LPC_GPIO1->FIODIR&=(~((1<<D0)|(1<<D1)|(1<<D2)|(1<<D3)|(1<<D4)|(1<<D5)|(1<<D6)|(1<<D7)));
LPC_GPIO1->FIOSET|=(1<<RW);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();

LPC_GPIO1->FIOCLR|=(1<<A0);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();

LPC_GPIO1->FIOSET|=(1<<E2);

chk2:

if(LPC_GPIO1->FIOPIN&(1<<D7)) goto chk2;

}

//-----------------------------------------------
void lcd1_wr(char in)
{

LPC_GPIO1->FIODIR|=((1UL<<A0)|(1UL<<E1)|(1UL<<RW));

lcd1_chk();

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();

LPC_GPIO1->FIOCLR|=(1UL<<A0);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();

LPC_GPIO1->FIOCLR|=(1UL<<RW);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();

LPC_GPIO1->FIODIR|=((1<<D0)|(1<<D1)|(1<<D2)|(1<<D3)|(1<<D4)|(1<<D5)|(1<<D6)|(1<<D7));
LPC_GPIO1->FIOSET|=(((long)in)<<D0);
LPC_GPIO1->FIOCLR|=(((long)(~in))<<D0)&(0xffUL<<D0);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();

LPC_GPIO1->FIOCLR|=(1<<E1);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();

}



//-----------------------------------------------
void lcd2_wr_(char in)
{

LPC_GPIO1->FIODIR|=(1<<A0);
LPC_GPIO1->FIODIR|=((1<<RW)|(1<<E2));


lcd2_chk();
     
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();

LPC_GPIO1->FIOCLR|=(1<<A0);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();

LPC_GPIO1->FIOCLR|=(1<<RW);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();

LPC_GPIO1->FIODIR|=((1<<D0)|(1<<D1)|(1<<D2)|(1<<D3)|(1<<D4)|(1<<D5)|(1<<D6)|(1<<D7));
LPC_GPIO1->FIOSET|=(((long)in)<<D0);
LPC_GPIO1->FIOCLR|=(((long)(~in))<<D0)&(0xffUL<<D0);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();

LPC_GPIO1->FIOCLR|=(1<<E2);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();

}

//-----------------------------------------------
char data1_wr(char in)
{
//__disable_irq();
LPC_GPIO1->FIODIR|=((1<<A0)|(1<<E1));
LPC_GPIO1->FIODIR|=(1<<RW);

lcd1_chk();

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop(); /*_*/

LPC_GPIO1->FIOSET|=(1<<A0);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop(); /**/

LPC_GPIO1->FIOCLR|=(1<<RW);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop(); /**/

LPC_GPIO1->FIODIR|=((1<<D0)|(1<<D1)|(1<<D2)|(1<<D3)|(1<<D4)|(1<<D5)|(1<<D6)|(1<<D7));
LPC_GPIO1->FIOSET|=(((long)in)<<D0);
LPC_GPIO1->FIOCLR|=(((long)(~in))<<D0)&(0xffUL<<D0);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop(); /**/

LPC_GPIO1->FIOCLR|=(1<<E1);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop(); /**/

return in;
}

//-----------------------------------------------
void data2_wr_(char in)
{
LPC_GPIO1->FIODIR|=(1<<A0);
LPC_GPIO1->FIODIR|=((1<<RW)|(1<<E2));

lcd2_chk();

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();

LPC_GPIO1->FIOSET|=(1<<A0);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();

LPC_GPIO1->FIOCLR|=(1<<RW);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();

LPC_GPIO1->FIODIR|=((1<<D0)|(1<<D1)|(1<<D2)|(1<<D3)|(1<<D4)|(1<<D5)|(1<<D6)|(1<<D7));
LPC_GPIO1->FIOSET|=(((long)in)<<D0);
LPC_GPIO1->FIOCLR|=(((long)(~in))<<D0)&(0xffUL<<D0);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();

LPC_GPIO1->FIOCLR|=(1<<E2);

__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();

}

//-----------------------------------------------
void lcd_set_page(char in)
{
lcd1_wr(_SET_PAGE_|in);
//lcd2_wr(_SET_PAGE_|in);
}

//-----------------------------------------------
void lcd_set_col(char in)
{
lcd1_wr(_SET_COLUMN_|in);
//lcd2_wr(_SET_COLUMN_|in);
}    

//-----------------------------------------------
void lcd_set_raw(char in)
{
lcd1_wr(_SET_RAW_|in);
//lcd2_wr(_SET_RAW_|in);
}    

//-----------------------------------------------
void lcd_init(void) 
{
int i/*,ii*/;



delay_ms(100);


//SET_REG(LPC_GPIO0->FIODIR,1,29,1);
//LPC_GPIO0->FIODIR|=(1<<RES)|(1<<29);

//SET_REG(LPC_GPIO0->FIOSET,1,29,1);

//delay_us(10);	
//LPC_GPIO0->FIODIR|=(1<<RES)/*|(1<<29)*/;
//strob_us(10);
//SET_REG(LPC_GPIO0->FIOSET,1,29,1);
//delay_us(1);
//SET_REG(LPC_GPIO0->FIOCLR,1,29,1); 

					
LPC_GPIO1->FIODIR|=(1<<D0)|(1<<D1)|(1<<D2)|(1<<D3)|(1<<D4)|(1<<D5)|(1<<D6)|(1<<D7);
LPC_GPIO1->FIODIR|=(1<<E1)|(1<<A0)|(1<<E2)|(1<<RW);
LPC_GPIO0->FIODIR|=(1<<RES)|(1<<29);



//for(i=0;i<2;i++)
//SET_REG(LPC_GPIO0->FIOSET,1,29,1);
//delay_us(1);
//SET_REG(LPC_GPIO0->FIOCLR,1,29,1); 
//SET_REG(LPC_GPIO0->FIOSET,1,29,1);
//SET_REG(LPC_GPIO0->FIOCLR,1,30,1);
/*for(i=0;i<100;i++)
	{*/											 //SET_REG(LPC_GPIO0->FIOCLR,1,29,1);
LPC_GPIO0->FIOCLR|=(1<<RES);
delay_us(50);
LPC_GPIO0->FIOSET|=(1<<RES);
delay_us(50);
/*	SET_REG(LPC_GPIO0->FIOCLR,1,29,1); */
	//}	

//SET_REG(LPC_GPIO0->FIOCLR,1,29,1);

/*for(i=0;i<166000;i++)
	{
	__nop();
	}*/

lcd1_wr(0x3f);
delay_us(150);
//strob_us(10);	
/*
for(i=0;i<166000;i++)
	{
	__nop();
	}*/

lcd1_wr(0x3f);

delay_us(50);
//strob_us(10);
/*
for(i=0;i<166000;i++)
	{
	__nop();
	}	*/

lcd1_wr(0x0c);

delay_us(150);
//strob_us(10);

//for(i=0;i<166000;i++)
/*	{
	__nop();
	}*/



/*
for(i=0;i<166000;i++)
	{
	__nop();
	}*/

/*lcd1_wr(0x01);

for(i=0;i<166000;i++)
	{
	__nop();
	} */

/*lcd1_wr(0x04);

for(i=0;i<166000;i++)
	{
	__nop();
	}*/
//lcd_reset();
//E1d=1;
//E2d=1;
//A0d=1;
//RWd=1;

//E1=0;
//E2=0;
/*IO1CLR|=(1<<E1);
IO1CLR|=(1<<E2);
//A0=1;
IO1SET|=(1<<A0);
//RW=1;
IO1SET|=(1<<RW);

lcd1_wr(_RESET_);
lcd2_wr(_RESET_);*/
     //LCD_PORTc=0xFF;
     //LCD_PORT=0xFF;
//disp_page0(); 

//IO0CLR|=(1<<A0)|(1<<RW);
}

//-----------------------------------------------
void status(void)
{

LPC_GPIO1->FIOCLR=(1<<A0);//clr P3.2 ; A0=0 for read
LPC_GPIO1->FIOSET=(1<<RW);//setb P3.6 ; wr=1
LPC_GPIO1->FIOSET=(1<<E1);//setb P3.3 ;Left side enabled
__nop();;//mov P1, #0FFH ; load input
stat1: //clr P3.5 ; rd=0
//nop
__nop();
__nop();
__nop();
//nop
//nop
//mov a, P1 ; move status
//anl a, #80H ;check bit 8
//setb P3.5 ; rd=1
//jnz stat1 ; jump if A not 0

if(LPC_GPIO1->FIOPIN&(1<<D7)) goto stat1;
LPC_GPIO1->FIOCLR=(1<<E1);//clr P3.3 ;

//ret
}

//-----------------------------------------------
void delay(void)
{
signed short i;

for(i=0;i<100;i++)
	{
	__nop();
	}
}

//-----------------------------------------------
void ltstrobe(char in)
{
status();
LPC_GPIO1->FIOSET=(1<<RW);//setb p3.5 ;RD=1
LPC_GPIO1->FIOCLR=(1<<A0);//clr p3.2 ;A=0
LPC_GPIO1->FIOSET=(1<<E1);//setb p3.3 ;CS1=1
LPC_GPIO1->FIOSET=(1<<E2);//setb p3.4 ;CS2=1
LPC_GPIO1->FIOSET|=(((long)in)<<D0);
LPC_GPIO1->FIOCLR|=(((long)(~in))<<D0)&(0xffUL<<D0);
//delay();
LPC_GPIO1->FIOCLR=(1<<E1);//clr p3.3 ;CS1=0
LPC_GPIO1->FIOCLR=(1<<E2);//clr p3.4 ;CS2=0
LPC_GPIO1->FIOCLR=(1<<RW);//clr p3.5 ;RD=0
//ret
}

//-----------------------------------------------
void lcd_init_(void) 
{
int i/*,ii*/;
#if(RES<16)
	{
	LPC_PINCON->PINSEL0&=~(1<<(RES*2))&~(1<<((RES*2)+1));
	}
#else 	
	{
	LPC_PINCON->PINSEL1&=~(1<<((RES-16)*2))&~(1<<(((RES-16)*2)+1));
	}
#endif	
LPC_PINCON->PINSEL2&=~(1<<((A0-16)*2))&~(1<<(((A0-16)*2)+1));
LPC_PINCON->PINSEL2&=~(1<<((E1-16)*2))&~(1<<(((E1-16)*2)+1));	
LPC_PINCON->PINSEL2&=~(1<<((E2-16)*2))&~(1<<(((E2-16)*2)+1));
LPC_PINCON->PINSEL2&=~(1<<((RW-16)*2))&~(1<<(((RW-16)*2)+1));
				





#if(D0<16)
	{
	LPC_PINCON->PINSEL0&=~(1<<(D0*2))&~(1<<((D0*2)+1));
	}
#else 	
	{
	LPC_PINCON->PINSEL1&=~(1<<((D0-16)*2))&~(1<<(((D0-16)*2)+1));
	}
#endif

#if(D1<16)
	{
	LPC_PINCON->PINSEL0&=~(1<<(D1*2))&~(1<<((D1*2)+1));
	}
#else 	
	{
	LPC_PINCON->PINSEL1&=~(1<<((D1-16)*2))&~(1<<(((D1-16)*2)+1));
	}
#endif
	
#if(D2<16)
	{
	LPC_PINCON->PINSEL0&=~(1<<(D2*2))&~(1<<((D2*2)+1));
	}
#else 	
	{
	LPC_PINCON->PINSEL1&=~(1<<((D2-16)*2))&~(1<<(((D2-16)*2)+1));
	}
#endif
	
#if(D3<16)
	{
	LPC_PINCON->PINSEL0&=~(1<<(D3*2))&~(1<<((D3*2)+1));
	}
#else 	
	{
	LPC_PINCON->PINSEL1&=~(1<<((D3-16)*2))&~(1<<(((D3-16)*2)+1));
	}	
#endif
	
#if(D4<16)
	{
	LPC_PINCON->PINSEL0&=~(1<<(D4*2))&~(1<<((D4*2)+1));
	}
#else 	
	{
	LPC_PINCON->PINSEL1&=~(1<<((D4-16)*2))&~(1<<(((D4-16)*2)+1));
	}
#endif

#if(D5<16)
	{
	LPC_PINCON->PINSEL0&=~(1<<(D5*2))&~(1<<((D5*2)+1));
	}
#else 	
	{
	LPC_PINCON->PINSEL1&=~(1<<((D5-16)*2))&~(1<<(((D5-16)*2)+1));
	}
#endif
	
#if(D6<16)
	{
	LPC_PINCON->PINSEL0&=~(1<<(D6*2))&~(1<<((D6*2)+1));
	}
#else 	
	{
	LPC_PINCON->PINSEL1&=~(1<<((D6-16)*2))&~(1<<(((D6-16)*2)+1));
	}
#endif
	
#if(D7<16)
	{
	LPC_PINCON->PINSEL0&=~(1<<(D7*2))&~(1<<((D7*2)+1));
	}
#else 	
	{
	LPC_PINCON->PINSEL1&=~(1<<((D7-16)*2))&~(1<<(((D7-16)*2)+1));
	}
#endif
	


	
					
LPC_GPIO1->FIODIR|=(1<<D0)|(1<<D1)|(1<<D2)|(1<<D3)|(1<<D4)|(1<<D5)|(1<<D6)|(1<<D7)|(1<<RES);
LPC_GPIO0->FIODIR|=(1<<E2)|(1<<RW)|(1<<E1)|(1<<A0);


for(i=0;i<10;i++)
	{
	LPC_GPIO1->FIOCLR|=(1<<RES);
	}
for(i=0;i<500;i++)
	{
	__nop();
	}
for(i=0;i<50000;i++)
	{
	LPC_GPIO1->FIOSET|=(1<<RES);
	}	

/*
mov a, #0E2H ;Reset column address, display
acall ltstrobe ;startline, page address counter=0.*/
ltstrobe(0xE2);

/*
mov a, #0AfH ;display=On
lcall ltstrobe*/
//ltstrobe(0xAF);

/*
mov a, #0C0H ;Starts on first line
lcall ltstrobe*/
//ltstrobe(0xC0);

/*
mov a, #0A4H ;static driver=Off
lcall ltstrobe*/
//ltstrobe(0xA4);

/*
mov a, #0A9H ;duty cycle=1/32
lcall ltstrobe*/
//ltstrobe(0xA9);

/*
mov a, #0A0H ;ADC=forward
lcall ltstrobe*/
//ltstrobe(0xA0);

/*
mov a, #0EEH ;Read Modified Write=End
lcall ltstrobe*/
//ltstrobe(0xEE);

/*
lcall clrsc ;Clear screen routine
mov R3, #0
mov R4, #0B8H
*/

}



//-----------------------------------------------
void lcd_clear(void)
{
char row,col,i;

for(row=0;row<32;row++)
	{
	lcd1_wr(0x80+row);
	lcd1_wr(0x80);

/*for(page=0;page<=Max_page;page++)

	{
	lcd_set_page(page);
	lcd_set_col(0);
	for(col=0;col<=Max_Col;col++)
         	{
         	data1_wr(0x01);
	    	//data2_wr(0x00);
     	}
     }  */

for (i=0;i<8;i++)
	{
	data1_wr(0x00);
	data1_wr(0x00);
	}
}
/*
data1_wr(0x01);
data1_wr(0x00);
data1_wr(0x00);
data1_wr(0x00);
data1_wr(0x00);
data1_wr(0x05);
data1_wr(0x15);*/
}





//-----------------------------------------------
void lcd_on(void) 
{
lcd1_wr(_DISPLAY_ON_);
//lcd2_wr(_DISPLAY_ON_);
}

//-----------------------------------------------
void lcd_off(void)
{
lcd1_wr(_DISPLAY_OFF_);
//lcd2_wr(_DISPLAY_OFF_);
}

//-----------------------------------------------
void lcd_out(char* adr)
{
static char mem;
short lcd_ptr;

char d1,d2;
char row,col,i;
//strob_us(1);

bitmap_hndl_2();

if(mem)
	{
	mem=0;
	d1=0x55;
	d2=0xff;
	}
else 
	{
	mem=1;
	d2=0x55;
	d1=0xff;
	} 

lcd_ptr=0;

for(row=0;row<32;row++)
	{
	//strob_us(10);
	lcd1_wr(0x80+row);
	//strob_us(10);
	lcd1_wr(0x80);
   	//strob_us(10);
/*for(page=0;page<=Max_page;page++)

	{
	lcd_set_page(page);
	lcd_set_col(0);
	for(col=0;col<=Max_Col;col++)
         	{
         	data1_wr(0x01);
	    	//data2_wr(0x00);
     	}
     }  */

	/*for (i=0;i<8;i++)
		{
		if(i==0) strob_us(10);
		data1_wr(d1);
		if(i==0) strob_us(10);
		data1_wr(d2);
		} */

	for (i=0;i<16;i++)
		{
		if(i==0) strob_us(1);
		data1_wr(lcd_bitmap2[lcd_ptr++]);
		//data1_wr(0x01);
		}

	}

}

