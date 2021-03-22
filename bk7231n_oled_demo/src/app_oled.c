/*
 * @Author: zgw
 * @email: liang.zhang@tuya.com
 * @LastEditors: zgw
 * @file name: app_plant.c
 * @Description: light init process include file
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Date: 2020-12-21 11:30:03
 * @LastEditTime: 2021-03-12 15:02:51
 */

#include "app_oled.h"
#include "app_control.h"
#include "uni_time_queue.h"
#include "sys_timer.h"
#include "tuya_iot_wifi_api.h"
#include "FreeRTOS.h"
#include "tuya_hal_thread.h"
#include "uni_thread.h"
#include "queue.h"
#include "semphr.h"

#define TASKDELAY_SEC         1000


/* 互斥信号量句柄 */
SemaphoreHandle_t xSemaphore = NULL;

extern APP_CTRL_DATA_T app_ctrl_data;

STATIC VOID sensor_data_get_thread(PVOID_T pArg);
STATIC VOID sensor_data_deal_thread(PVOID_T pArg);
STATIC VOID sensor_data_report_thread(PVOID_T pArg);

#define BEEP_PORT                    (14)
#define BEEP_LEVEL                   HIGH

OPERATE_RET app_oled_init(IN APP_OLED_MODE mode)
{
    OPERATE_RET op_ret = OPRT_OK;

    if(APP_NORMAL == mode) {
        app_device_init();

        xSemaphore = xSemaphoreCreateMutex();
        //create ADC sensor data collection thread
        tuya_hal_thread_create(NULL, "thread_data_get", 512*4, TRD_PRIO_4, sensor_data_get_thread, NULL);

        tuya_hal_thread_create(NULL, "thread_data_deal", 512*4, TRD_PRIO_4, sensor_data_deal_thread, NULL);

        tuya_hal_thread_create(NULL, "thread_data_report", 512*4, TRD_PRIO_4, sensor_data_report_thread, NULL);
    }else {
        //not factory test mode
    }

    return op_ret;
}

STATIC VOID sensor_data_get_thread(PVOID_T pArg)
{   
    while(1) {
        xSemaphoreTake(xSemaphore, 500);

        PR_DEBUG("sensor_data_get_thread");
        app_get_sensor_data();

        xSemaphoreGive(xSemaphore);
        tuya_hal_system_sleep(TASKDELAY_SEC*5);
    }
}

STATIC VOID sensor_data_deal_thread(PVOID_T pArg)
{   
    while(1) {
        xSemaphoreTake(xSemaphore, 500);
        app_ctrl_handle();
        
        xSemaphoreGive(xSemaphore);
        tuya_hal_system_sleep(TASKDELAY_SEC);
    }
}

STATIC VOID sensor_data_report_thread(PVOID_T pArg)
{   
    while(1) {
        tuya_hal_system_sleep(TASKDELAY_SEC*10);
        app_report_all_dp_status();
    }
}


VOID app_report_all_dp_status(VOID)
{
    OPERATE_RET op_ret = OPRT_OK;

    INT_T dp_cnt = 0;
    dp_cnt = 2;

    TY_OBJ_DP_S *dp_arr = (TY_OBJ_DP_S *)Malloc(dp_cnt*SIZEOF(TY_OBJ_DP_S));
    if(NULL == dp_arr) {
        PR_ERR("malloc failed");
        return;
    }

    memset(dp_arr, 0, dp_cnt*SIZEOF(TY_OBJ_DP_S));

    dp_arr[0].dpid = DPID_TEMP_CURRENT;
    dp_arr[0].type = PROP_VALUE;
    dp_arr[0].time_stamp = 0;
    dp_arr[0].value.dp_value = app_ctrl_data.Temp_current;

    dp_arr[1].dpid = DPID_HUMIDITY_VALUE;
    dp_arr[1].type = PROP_VALUE;
    dp_arr[1].time_stamp = 0;
    dp_arr[1].value.dp_value = app_ctrl_data.Humidity_current;

    op_ret = dev_report_dp_json_async(NULL,dp_arr,dp_cnt);
    Free(dp_arr);
    if(OPRT_OK != op_ret) {
        PR_ERR("dev_report_dp_json_async relay_config data error,err_num",op_ret);
    }

    PR_DEBUG("dp_query report_all_dp_data");
    return;
}


VOID deal_dp_proc(IN CONST TY_OBJ_DP_S *root)
{
    UCHAR_T dpid;

    dpid = root->dpid;
    PR_DEBUG("dpid:%d",dpid);
    
    switch (dpid) {
    
    case DPID_HUMIDITY_VALUE:
        PR_DEBUG("set led switch:%d",root->value.dp_bool);

        break;

    default:
        break;
    }
    
    app_report_all_dp_status();

    return;

}
