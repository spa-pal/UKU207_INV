
extern unsigned char modbus_buf[20];
extern short modbus_crc16;
extern char modbus_timeout_cnt;
extern char bMODBUS_TIMEOUT;
extern unsigned char modbus_rx_buffer[30];	//�����, ���� ���������� ����������� ������� ���������� ���������� �� ������ �����
extern unsigned char modbus_an_buffer[30];	//�����, ���� ��� ����� ���������� ��� �������
extern unsigned char modbus_rx_buffer_ptr;	//��������� �� ������� ������� ������������ ������
extern unsigned char modbus_rx_counter;		//���������� �������� ����, ������������ ��� ������� ����������� ������� � ��� �����������
extern signed short modbusTimeoutInMills;
extern short modbus_plazma;				//�������
extern short modbus_plazma1;				//�������
extern short modbus_plazma2;				//�������
extern short modbus_plazma3;				//�������
extern char modbus_cmnd_cnt,modbus_cmnd,modbus_self_cmnd_cnt;
extern short modbus_crc_plazma[2];
extern short modbus_rtu_plazma[10];

//-----------------------------------------------
unsigned short CRC16_2(char* buf, short len);
/* //o_1
void modbus_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity);
//-----------------------------------------------
void modbus_hold_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot);
//-----------------------------------------------
void modbus_register_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr);
//-----------------------------------------------
void modbus_hold_register_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr, char prot);
//-----------------------------------------------
void modbus_input_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot);
//-----------------------------------------------
void modbus_in(void);
//-----------------------------------------------
void modbus_hold_register_write(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_value, char prot);
*/ //o_1