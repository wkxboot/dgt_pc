#ifndef  __SCALE_TASK_H__
#define  __SCALE_TASK_H__


extern osThreadId scale_task_hdl;
extern osMessageQId scale_task_msg_q_id;


void scale_task(void const *argument);



#define  SCALE_TASK_MODBUS_SERIAL_PORT               1
#define  SCALE_TASK_MODBUS_SERIAL_BAUDRATE           115200
#define  SCALE_TASK_MODBUS_SERIAL_DATABITS           8
#define  SCALE_TASK_MODBUS_SERIAL_STOPBITS           2

#define  SCLAE_TASK_DEFAULT_SLAVE_ADDR               247



#define  SCALE_TASK_SUCCESS                           1
#define  SCALE_TASK_FAILURE                           2


#define  SCALE_TASK_ADDR_VALUE_MAX                    0xFE
#define  SCALE_TASK_WEIGHT_ERR_VALUE                  0x7FFF
#define  SCALE_TASK_MAX_WEIGHT_VALUE                  (32767.0)  /*最大32767  g*/
#define  SCALE_TASK_MIN_WEIGHT_VALUE                  (-32767.0) /*最小-32767 g*/

#define  SCALE_TASK_MSG_PUT_TIMEOUT_VALUE             10
#define  SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE            osWaitForever    



/*SCALE MODBUS REG*/
#define  SCALE_TASK_SYS_UNLOCK_VALUE                   0x5AA5
#define  SCALE_TASK_CALIBRATE_AUTO_VALUE               0x7FFFFFFFUL
#define  SCALE_TASK_REMOVE_TAR_WEIGHT_VALUE            0x7FFFFFFFUL

#define  SCALE_TASK_NET_WEIGHT_REG_ADDR                82
#define  SCALE_TASK_NET_WEIGHT_REG_CNT                 2

#define  SCALE_TASK_CALIBRATE_ZERO_CODE_REG_ADDR       36
#define  SCALE_TASK_CALIBRATE_ZERO_CODE_REG_CNT        2

#define  SCALE_TASK_CALIBRATE_ZERO_MEASURE_REG_ADDR    38
#define  SCALE_TASK_CALIBRATE_ZERO_MEASURE_REG_CNT     2

#define  SCALE_TASK_CALIBRATE_FULL_CODE_REG_ADDR       40
#define  SCALE_TASK_CALIBRATE_FULL_CODE_REG_CNT        2

#define  SCALE_TASK_CALIBRATE_FULL_MEASURE_REG_ADDR    42
#define  SCALE_TASK_CALIBRATE_FULL_MEASURE_REG_CNT     2


#define  SCALE_TASK_REMOVE_TAR_WEIGHT_REG_ADDR         84
#define  SCALE_TASK_REMOVE_TAR_WEIGHT_REG_CNT          2

#define  SCALE_TASK_ADDR_REG_ADDR                      0
#define  SCALE_TASK_ADDR_REG_CNT                       1

#define  SCALE_TASK_UNLOCK_REG_ADDR                    5
#define  SCALE_TASK_UNLOCK_REG_CNT                     1







#endif