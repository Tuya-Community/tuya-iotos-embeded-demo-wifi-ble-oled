/*
 * @file name: 
 * @Descripttion: 
 * @Author: zgw
 * @email: liang.zhang@tuya.com
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Date: 2021-03-10 10:46:14
 * @LastEditors: zgw
 * @LastEditTime: 2021-03-12 14:14:20
 */

#ifndef __SH1106_H__
#define __SH1106_H__

#include "uni_log.h"
#include "tuya_cloud_error_code.h"

typedef struct 
{
    UCHAR_T SDA_PIN;            ///< SDA pin
    UCHAR_T SCL_PIN;            ///< SCL pin 
}sh1106_init_t;

typedef enum {
    clear,
    full
}C_F_CMD_T;



UCHAR_T tuya_sh1106_init(sh1106_init_t* param);

VOID tuya_sh1106_full_or_clear(C_F_CMD_T cmd);
VOID tuya_sh1106_clear(VOID);
VOID tuya_sh1106_full(VOID);
VOID tuya_sh1106_display(VOID);
/**
 * @berief: display a single pattern or text.
 * @param IN start_page:start page position(0~3); page_num:number of pages(1~4); start_column:start column position(0~127);pic_width:pattern or text width;*ptr_pic:buffer address
 * @return: none
 * @retval: none
 */
VOID tuya_sh1106_display_single_pic(UCHAR_T start_page, UCHAR_T page_num, UCHAR_T start_column, UCHAR_T pic_width, CONST UCHAR_T *ptr_pic);
VOID tuya_sh1106_gram_point_set(UCHAR_T x, UCHAR_T y, CONST UCHAR_T *ptr_pic);

#endif