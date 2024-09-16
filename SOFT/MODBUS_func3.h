extern signed short UB20_minus_DU;
extern char sntp_ip1_mb, sntp_ip2_mb, sntp_ip3_mb, sntp_ip4_mb;
extern char numbat_mb;
extern unsigned char *const reg_func3 [];
extern unsigned char sk_sign_0_mb, sk_zvuk_en_0_mb, sk_lcd_en_0_mb;
extern unsigned char sk_sign_1_mb, sk_zvuk_en_1_mb, sk_lcd_en_1_mb;
extern unsigned char sk_sign_2_mb, sk_zvuk_en_2_mb, sk_lcd_en_2_mb;
extern unsigned char sk_sign_3_mb, sk_zvuk_en_3_mb, sk_lcd_en_3_mb;
extern unsigned short tventmax_mb;
extern unsigned short modbus_log_deep;
extern unsigned short modbus_log_ptr;
extern unsigned char modbus_log_data_byte[32];

int lc640_read_int(int ADR);

void wr_reg_func3(unsigned long start_adr, unsigned long end_adr);
//количество байт в массиве, включая нулевой байт:= (максимальный номер регистра+1)*2
#define MODBUS_FUNC_3_LENGTH 200UL
