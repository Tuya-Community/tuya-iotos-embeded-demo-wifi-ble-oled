/*
 * @file name: 
 * @Descripttion: 
 * @Author: zgw
 * @email: liang.zhang@tuya.com
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Date: 2021-03-10 10:45:56
 * @LastEditors: zgw
 * @LastEditTime: 2021-03-12 10:40:59
 */

#include "sh1106.h"
#include "soc_i2c.h"

#define OLED_COLUMN_NUMBER              128
#define OLED_LINE_NUMBER                32
#define OLED_PAGE_NUMBER                OLED_LINE_NUMBER/8

UCHAR_T *OLED_GRAM[4][16];

CONST STATIC UCHAR_T clear_buff[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
CONST STATIC UCHAR_T full_buff[] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

CONST UCHAR_T oled_init_cmd[25] =
{
  
  /*0xae,0X00,0X10,0x40,0X81,0XCF,0xff,0xa1,0xa4,
  0xA6,0xc8,0xa8,0x3F,0xd5,0x80,0xd3,0x00,0XDA,0X12,
  0x8d,0x14,0xdb,0x40,0X20,0X02,0xd9,0xf1,0xAF*/
       0xAE,//关闭显示
	
       0xD5,//设置时钟分频因子 震荡频率
       0x80,  //[3:0],分频因子;[7:4],震荡频率

       0xA8,//设置驱动路数
       0X1F,//默认(1/32)
	
       0xD3,//设置显示偏移
       0X00,//默认为0
	
       0x40,//设置显示开始行 [5:0],行数.
	
       0x8D,//电荷泵设置
       0x14,//bit2,开启/关闭
       0x20,//设置内存地址模式
       0x02,//[1:0],00,列地址模式; 01,行地址模式; 10,页地址模式; 默认10;
       0xA1,//段重定义设置,bit0:0,0->0;1,0->127;
	
       0xC8,//设置COM扫描方向; bit3: 0,普通模式; 1,重定义模式 COM[N-1]->COM0;N:驱动路数
	   
       0xDA,//设置COM硬件引脚设置
       0x02,//[5:4]配置  
	   
       0x81,//对比度设置l
       0xEF,//1~255;默认0x7F (亮度设置,越大越亮)
	   
       0xD9,//设置预充电周期
       0xf1,//[3:0],PHASE 1;[7:4],PHASE 2;
	   
       0xDB,//设置VCOMH 电压倍率
       0x30,//[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;
	   
       0xA4,//全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
	   
       0xA6,//设置显示方式;bit0:1,反相显示;0,正常显示 
       
       0xAF,//开启显示     
};

//I2C acknowledge
typedef enum{
    ACK                      = 0,
    NO_ACK                   = 1,
}sh1106_ack;


STATIC VOID __iic_send_ack(sh1106_ack ack)
{
    if(TRUE == ack) {
        vI2CSDASet();
    }else {
        vI2CSDAReset();
    }
    vI2CSCLSet();

    vI2CDelay(5);

    vI2CSCLReset();

    vI2CDelay(5);      
}

STATIC BOOL_T __iic_recv_ack(VOID)
{   
    BOOL_T recv_ack;

    vI2CSCLSet();

    vI2CDelay(5);
    
    recv_ack = ucI2CSDAInputRead();
    vI2CSCLReset();

    vI2CDelay(5);
    
    return recv_ack;
}

STATIC VOID __iic_send_byte(UCHAR_T sendbyte)
{
    UCHAR_T i = 0;

    for(i = 0x80; i > 0; i >>= 1)
    {
        if((sendbyte & i) == 0) {
            vI2CSDAReset();
        } else {
            vI2CSDASet();
        }
        vI2CDelay(1);   
        vI2CSCLSet();
        vI2CDelay(1); //vI2CDelay(5);
        vI2CSCLReset();
        vI2CDelay(1);

    }

    vI2CSDASet();
    
    vI2CDelay(1);
    vI2CSCLSet();
    vI2CDelay(1);
    vI2CSCLReset();
    
    //__iic_recv_ack();
}


STATIC int __iic_start(VOID)
{
    vI2CSDASet();
    vI2CDelay(1);//vI2CDelay(5);
    vI2CSCLSet();
    vI2CDelay(1);//vI2CDelay(5);
    vI2CSDAReset();
    vI2CDelay(1);//vI2CDelay(5);
    /*
    if(ucI2CSDAOutputRead()) {    
        PR_NOTICE("iic start error!");
        return -1;
    }
    */
    vI2CSCLReset();
    
    __iic_send_byte(0x78);

    return 0;
}


STATIC int __iic_stop(VOID)
{
    vI2CSDAReset();
    vI2CSCLReset();
    vI2CDelay(1);//vI2CDelay(5);
    if(ucI2CSDAInputRead()) {    /* confirm SDA status is ready */
        PR_NOTICE("BH1750 stop1 error!");
        return -1;
    }
    
    vI2CSCLSet();

    vI2CDelay(1);//vI2CDelay(5);

    vI2CSDASet();

    if(!ucI2CSDAOutputRead()) {    /* confirm SDA status is set correct */
        PR_NOTICE("iic stop2 error!");
        return -1;
    }

    vI2CDelay(1);//vI2CDelay(5);

    return 0;        
}

VOID sh1106_send_cmd(UCHAR_T command)
{
    __iic_start();
    __iic_send_byte(0x00);
    __iic_send_byte(command);
    __iic_stop();
}

VOID sh1106_send_data(UCHAR_T data)
{
    __iic_start();
    __iic_send_byte(0x40);
    __iic_send_byte(data);
    __iic_stop();    
}

VOID sh1106_page_set(UCHAR_T page)
{
    sh1106_send_cmd(0xb0 + page);
}

VOID sh1106_column_set(UCHAR_T column)
{
    sh1106_send_cmd(0x10|(column >> 4));     //设置列地址高位
    sh1106_send_cmd(0x00|(column & 0x0f));   //设置列地址低位  
    	 
}

VOID tuya_sh1106_full_or_clear(C_F_CMD_T cmd)
{
    UCHAR_T page,column;
    for(page = 0; page < OLED_PAGE_NUMBER; page++) {         //page loop 
        sh1106_page_set(page);
        sh1106_column_set(0);	  
	    for(column = 0; column < OLED_COLUMN_NUMBER; column++) {      //column loop
            if(cmd) {
                sh1106_send_data(0xff);
            }else {
                sh1106_send_data(0x00);
            }
        }
    }
}


VOID tuya_sh1106_display_single_pic(UCHAR_T start_page, UCHAR_T page_num, UCHAR_T start_column, UCHAR_T pic_width, CONST UCHAR_T *ptr_pic)
{
    UCHAR_T page,column;
    for(page = start_page; page < page + page_num; page++){       //page loop
	    sh1106_page_set(page);
	    sh1106_column_set(start_column);	  
	    for(column = start_column; column < start_column + pic_width; column++) {	  //column loop
            sh1106_send_data(*ptr_pic++);
        }
    }    
}


UCHAR_T tuya_sh1106_init(sh1106_init_t* param)
{
    UCHAR_T error = 0;

    int opRet = -1;

    i2c_pin_t i2c_config = {
        .ucSDA_IO = param ->SDA_PIN,
        .ucSCL_IO = param ->SCL_PIN,
    };
    opRet = opSocI2CInit(&i2c_config);          /* SDA&SCL GPIO INIT */
    PR_NOTICE("SocI2CInit = %d",opRet);

    UCHAR_T i;
    for(i = 0; i  < 25; i++) {
        sh1106_send_cmd(oled_init_cmd[i]);
    }
}


VOID tuya_sh1106_clear(VOID)
{    
    UCHAR_T i,j,k;
    UCHAR_T *p;
    for(i = 0; i < 4; i++) {
        for(j = 0; j < 16; j++) {
            OLED_GRAM[i][j] = clear_buff;
        }
    }
    for(i = 0; i < OLED_PAGE_NUMBER; i++) {
        sh1106_page_set(i);
        sh1106_column_set(0);	  
        for(j = 0; j < (OLED_COLUMN_NUMBER/8); j++) {
            p = OLED_GRAM[i][j];
            for(k = 0; k < 8; k++) {
                sh1106_send_data(*p);
                p++;
            }
        }
    }
}

VOID tuya_sh1106_full(VOID)
{    
    UCHAR_T i,j,k;
    UCHAR_T *p;
    for(i = 0; i < 4; i++) {
        for(j = 0; j < 16; j++) {
            OLED_GRAM[i][j] = full_buff;
        }
    }
    for(i = 0; i < OLED_PAGE_NUMBER; i++) {
        sh1106_page_set(i);
        sh1106_column_set(0);	  
        for(j = 0; j < (OLED_COLUMN_NUMBER/8); j++) {
            p = OLED_GRAM[i][j];
            for(k = 0; k < 8; k++) {
                sh1106_send_data(*p);
                p++;
            }
        }
    }
}

VOID tuya_sh1106_display(VOID)
{    
    UCHAR_T i,j,k;
    UCHAR_T *p;
    for(i = 0; i < OLED_PAGE_NUMBER; i++) {
        sh1106_page_set(i);
        sh1106_column_set(0);	  
        for(j = 0; j < (OLED_COLUMN_NUMBER/8); j++) {
            p = OLED_GRAM[i][j];
            for(k = 0; k < 8; k++) {
                sh1106_send_data(*p);
                p++;
            }
        }
    }
}

VOID tuya_sh1106_gram_point_set(UCHAR_T x, UCHAR_T y, CONST UCHAR_T *ptr_pic)
{   
    UCHAR_T i;
    UCHAR_T *p;

    if((x < 4)&&(y < 16)) {
        OLED_GRAM[x][y] = ptr_pic;
    }
    p = OLED_GRAM[x][y];
    
    sh1106_page_set(x);
    sh1106_column_set(y*8);

    for(i = 0; i < 8; i++) {
        sh1106_send_data(*p);
        p++;
    }
}

