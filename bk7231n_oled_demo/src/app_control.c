/*
 * @Author: zgw
 * @email: liang.zhang@tuya.com
 * @LastEditors: zgw
 * @file name: sht21.h
 * @Description: SHT21 IIC drive src file
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Date: 2020-12-16 18:51:29
 * @LastEditTime: 2021-03-12 14:56:23
 */

#include "app_control.h"
//#include "soc_adc.h"
#include "tuya_gpio.h"
#include "tuya_uart.h"
#include "BkDriverUart.h"
#include "sys_timer.h"
#include "sh1106.h"
#include "sht21.h"
#include "uni_time.h"
#include "soc_i2c.h"
/***********************************************************
*************************types define***********************
***********************************************************/
typedef enum
{
    LOW = 0,
    HIGH,
}default_level;

typedef struct {
    float temperature;
    float humidity;
}DEVICE_DATA_T;

APP_CTRL_DATA_T app_ctrl_data = {0};
DEVICE_DATA_T device_data = {0};
/***********************************************************
*************************IO control device define***********
***********************************************************/


/***********************************************************
*************************about adc init*********************
***********************************************************/
#define OLED_PIX_HEIGHT              (16)

CONST UCHAR_T diplay_buffer_time[] = {
/*--  文字:  0  --*/
/*--  宋体12;  此字体下对应的点阵为：宽x高=8x16   --*/
0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x0F,0x10,0x20,0x20,0x10,0x0F,0x00,

/*--  文字:  1  --*/
/*--  宋体12;  此字体下对应的点阵为：宽x高=8x16   --*/
0x00,0x00,0x10,0x10,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,

/*--  文字:  2  --*/
/*--  宋体12;  此字体下对应的点阵为：宽x高=8x16   --*/
0x00,0x70,0x08,0x08,0x08,0x08,0xF0,0x00,0x00,0x30,0x28,0x24,0x22,0x21,0x30,0x00,

/*--  文字:  3  --*/
/*--  宋体12;  此字体下对应的点阵为：宽x高=8x16   --*/
0x00,0x30,0x08,0x08,0x08,0x88,0x70,0x00,0x00,0x18,0x20,0x21,0x21,0x22,0x1C,0x00,

/*--  文字:  4  --*/
/*--  宋体12;  此字体下对应的点阵为：宽x高=8x16   --*/
0x00,0x00,0x80,0x40,0x30,0xF8,0x00,0x00,0x00,0x06,0x05,0x24,0x24,0x3F,0x24,0x24,

/*--  文字:  5  --*/
/*--  宋体12;  此字体下对应的点阵为：宽x高=8x16   --*/
0x00,0xF8,0x88,0x88,0x88,0x08,0x08,0x00,0x00,0x19,0x20,0x20,0x20,0x11,0x0E,0x00,

/*--  文字:  6  --*/
/*--  宋体12;  此字体下对应的点阵为：宽x高=8x16   --*/
0x00,0xE0,0x10,0x88,0x88,0x90,0x00,0x00,0x00,0x0F,0x11,0x20,0x20,0x20,0x1F,0x00,

/*--  文字:  7  --*/
/*--  宋体12;  此字体下对应的点阵为：宽x高=8x16   --*/
0x00,0x18,0x08,0x08,0x88,0x68,0x18,0x00,0x00,0x00,0x00,0x3E,0x01,0x00,0x00,0x00,

/*--  文字:  8  --*/
/*--  宋体12;  此字体下对应的点阵为：宽x高=8x16   --*/
0x00,0x70,0x88,0x08,0x08,0x88,0x70,0x00,0x00,0x1C,0x22,0x21,0x21,0x22,0x1C,0x00,

/*--  文字:  9  --*/
/*--  宋体12;  此字体下对应的点阵为：宽x高=8x16   --*/
0x00,0xF0,0x08,0x08,0x08,0x10,0xE0,0x00,0x00,0x01,0x12,0x22,0x22,0x11,0x0F,0x00,

/*--  文字:  :  --*/
/*--  宋体12;  此字体下对应的点阵为：宽x高=8x16   --*/
0x00,0x00,0x00,0xC0,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,

/*--  文字:  T  --*/
/*--  宋体12;  此字体下对应的点阵为：宽x高=8x16   --*/
0x18,0x08,0x08,0xF8,0x08,0x08,0x18,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,

/*--  文字:  H  --*/
/*--  宋体12;  此字体下对应的点阵为：宽x高=8x16   --*/
0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x20,0x3F,0x21,0x01,0x01,0x21,0x3F,0x20,

/*--  文字:  %  --*/
/*--  宋体12;  此字体下对应的点阵为：宽x高=8x16   --*/
0xF0,0x08,0xF0,0x80,0x60,0x18,0x00,0x00,0x00,0x31,0x0C,0x03,0x1E,0x21,0x1E,0x00,

/*--  文字:  ℃  --*/
/*--  宋体12;  此字体下对应的点阵为：宽x高=8x16   --*/
0x06,0x09,0x09,0xE6,0xF8,0x0C,0x04,0x02,0x00,0x00,0x00,0x07,0x1F,0x30,0x20,0x40,

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

/***********************************************************
*************************about iic init*********************
***********************************************************/
#define IIC_SDA_SHT21_PIN                         (6)
#define IIC_SCL_SHT21_PIN                         (7)

#define IIC_SDA_OLED_PIN                         (22)
#define IIC_SCL_OLED_PIN                         (20)

STATIC sht21_init_t sht21_int_param = {IIC_SDA_SHT21_PIN, IIC_SCL_SHT21_PIN, SHT2x_RES_10_13BIT};

STATIC sh1106_init_t sh1106_init_param = {IIC_SDA_OLED_PIN, IIC_SCL_OLED_PIN};
/***********************************************************
*************************function***************************
***********************************************************/
STATIC VOID app_display_time(VOID)
{   
    UCHAR_T temp,hum;

    temp = app_ctrl_data.Temp_current;
    hum = app_ctrl_data.Humidity_current;
    
    if(temp < 10) {
        tuya_sh1106_gram_point_set(1,4,&diplay_buffer_time[15*OLED_PIX_HEIGHT]);
        tuya_sh1106_gram_point_set(2,4,&diplay_buffer_time[15*OLED_PIX_HEIGHT+8]);
    }else {
        tuya_sh1106_gram_point_set(1,4,&diplay_buffer_time[(temp/10)*OLED_PIX_HEIGHT]);
        tuya_sh1106_gram_point_set(2,4,&diplay_buffer_time[(temp/10)*OLED_PIX_HEIGHT+8]);
    }
    tuya_sh1106_gram_point_set(1,5,&diplay_buffer_time[(temp%10)*OLED_PIX_HEIGHT]);
    tuya_sh1106_gram_point_set(2,5,&diplay_buffer_time[(temp%10)*OLED_PIX_HEIGHT+8]);

    if(hum < 10) {
        tuya_sh1106_gram_point_set(1,10,&diplay_buffer_time[15*OLED_PIX_HEIGHT]);
        tuya_sh1106_gram_point_set(2,10,&diplay_buffer_time[15*OLED_PIX_HEIGHT+8]);
    }else {
        tuya_sh1106_gram_point_set(1,10,&diplay_buffer_time[(hum/10)*OLED_PIX_HEIGHT]);
        tuya_sh1106_gram_point_set(2,10,&diplay_buffer_time[(hum/10)*OLED_PIX_HEIGHT+8]);
    } 
    tuya_sh1106_gram_point_set(1,11,&diplay_buffer_time[(hum%10)*OLED_PIX_HEIGHT]);
    tuya_sh1106_gram_point_set(2,11,&diplay_buffer_time[(hum%10)*OLED_PIX_HEIGHT+8]);

    tuya_sh1106_display();
}

STATIC VOID __ctrl_gpio_init(CONST TY_GPIO_PORT_E port, CONST BOOL_T high)
{   
    //Set IO port as output mode
    tuya_gpio_inout_set(port, FALSE);
    //Set IO port level
    tuya_gpio_write(port, high);
}


VOID app_device_init(VOID)
{
    INT_T op_ret = 0;

    //OLED IIC driver init
    tuya_sh1106_init(&sh1106_init_param);
    tuya_hal_system_sleep(500);

    tuya_sh1106_clear();

    //fill pattern:'T'
    tuya_sh1106_gram_point_set(1,2,&diplay_buffer_time[11*OLED_PIX_HEIGHT]);
    tuya_sh1106_gram_point_set(2,2,&diplay_buffer_time[11*OLED_PIX_HEIGHT+8]);

    //fill pattern:'H'
    tuya_sh1106_gram_point_set(1,8,&diplay_buffer_time[12*OLED_PIX_HEIGHT]);
    tuya_sh1106_gram_point_set(2,8,&diplay_buffer_time[12*OLED_PIX_HEIGHT+8]);

    //fill pattern:':'
    tuya_sh1106_gram_point_set(1,3,&diplay_buffer_time[10*OLED_PIX_HEIGHT]);
    tuya_sh1106_gram_point_set(2,3,&diplay_buffer_time[10*OLED_PIX_HEIGHT+8]);
    tuya_sh1106_gram_point_set(1,9,&diplay_buffer_time[10*OLED_PIX_HEIGHT]);
    tuya_sh1106_gram_point_set(2,9,&diplay_buffer_time[10*OLED_PIX_HEIGHT+8]);

    //fill pattern:'℃'
    tuya_sh1106_gram_point_set(1,6,&diplay_buffer_time[14*OLED_PIX_HEIGHT]);
    tuya_sh1106_gram_point_set(2,6,&diplay_buffer_time[14*OLED_PIX_HEIGHT+8]);
    
    //fill pattern:'%'
    tuya_sh1106_gram_point_set(1,12,&diplay_buffer_time[13*OLED_PIX_HEIGHT]);
    tuya_sh1106_gram_point_set(2,12,&diplay_buffer_time[13*OLED_PIX_HEIGHT+8]);
}



VOID app_get_sensor_data(VOID)
{   
    SHORT_T hum;
    SHORT_T temp;
    
    tuya_sht21_init(&sht21_int_param);
    
    hum = tuya_sht21_measure(HUMIDITY);
    device_data.humidity = tuya_sht21_cal_RH(hum);
    if(device_data.humidity > 0){ //eliminate invalid humidity values less than 0
        app_ctrl_data.Humidity_current = (UCHAR_T)device_data.humidity;
        PR_NOTICE("humidity = %d",app_ctrl_data.Humidity_current);
    }

    temp = tuya_sht21_measure(TEMP);
    device_data.temperature = tuya_sht21_cal_temperature(temp);
    app_ctrl_data.Temp_current = (UCHAR_T)device_data.temperature;
    PR_NOTICE("tempre = %d",app_ctrl_data.Temp_current);
}

VOID app_ctrl_handle(VOID)
{   
    sh1106_init_t* p;
    p = &sh1106_init_param;
    i2c_pin_t i2c_config = {
        .ucSDA_IO = p->SDA_PIN,
        .ucSCL_IO = p->SCL_PIN,
    };
    opSocI2CInit(&i2c_config);          /* SDA&SCL GPIO INIT */

    app_display_time();
}

VOID app_ctrl_all_off(VOID)
{   

}

 