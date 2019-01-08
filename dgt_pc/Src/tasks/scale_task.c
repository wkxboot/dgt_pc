#include "board.h"
#include "cmsis_os.h"
#include "serial.h"
#include "modbus.h"
#include "task_msg.h"
#include "scale_task.h"
#include "protocol_task.h"
#include "firmware_version.h"
#include "tasks_init.h"
#include "sensor_id.h"
#include "log.h"
#define LOG_MODULE_NAME   "[scale]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG 

osThreadId scale_task_hdl;
osMessageQId scale_task_msg_q_id;

#define  PROBE_SLAVE_ADDR_ERR_TIMEOUT    200
extern uint16_t wdg_timeout;
extern int modbus_serial_handle;

extern serial_hal_driver_t modbus_serial_driver;

/*modbus rtu rs485操作*/
void modbus_rtu_send_pre()
{
  bsp_modbus_485_enable_write();  
}
/*modbus rtu rs485操作*/
void modbus_rtu_send_after()
{
  bsp_modbus_485_enable_read();   
}

void scale_task(void const * argument)
{
  int rc;
  osEvent os_msg;
  osStatus status;
  protocol_task_msg_t req_msg,rsp_msg;

  modbus_t *ctx;
  uint16_t  read_value[2];
  uint16_t  write_value[2];
  int32_t   net_weight;
 
  osMessageQDef(scale_task_msg_q,8,uint32_t);
  scale_task_msg_q_id = osMessageCreate(osMessageQ(scale_task_msg_q),protocol_task_hdl);
  log_assert(scale_task_msg_q_id); 
 
  ctx = modbus_new_rtu(SCALE_TASK_MODBUS_SERIAL_PORT,
                       SCALE_TASK_MODBUS_SERIAL_BAUDRATE,
                       SCALE_TASK_MODBUS_SERIAL_DATABITS, 
                       SCALE_TASK_MODBUS_SERIAL_STOPBITS,
                       &modbus_serial_driver);
  log_assert(ctx);
  modbus_serial_handle = ctx->s;
  rc = modbus_connect(ctx);
  log_assert(rc == 0);
 
  /*等待任务同步*/
  xEventGroupSync(tasks_sync_evt_group_hdl,TASKS_SYNC_EVENT_SCALE_TASK_RDY,TASKS_SYNC_EVENT_ALL_TASKS_RDY,osWaitForever);
  log_debug("scale task sync ok.\r\n");
  
  while(1){   
  os_msg = osMessageGet(scale_task_msg_q_id,SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE);
 
  if(os_msg.status == osEventMessage){
     req_msg = *(protocol_task_msg_t*)&os_msg.value.v;
 
  /*向protocol_task回应净重值*/
  if(req_msg.type ==  PROTOCOL_TASK_MSG_REQ_NET_WEIGHT){
     modbus_set_slave(ctx,req_msg.value);
     rsp_msg.reserved = SCALE_TASK_SUCCESS;
     rc = modbus_read_registers(ctx,SCALE_TASK_NET_WEIGHT_REG_ADDR,SCALE_TASK_NET_WEIGHT_REG_CNT,read_value);
     if(rc <= 0){
        net_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
        rsp_msg.reserved = SCALE_TASK_FAILURE;
     }else{
        net_weight = read_value[0]<<16 |read_value[1];
       if(net_weight >= SCALE_TASK_MAX_WEIGHT_VALUE ||
          net_weight <= SCALE_TASK_MIN_WEIGHT_VALUE){
          net_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
       }
     }
    rsp_msg.type = PROTOCOL_TASK_MSG_RESPONSE_NET_WEIGHT;
    rsp_msg.value = (int16_t)net_weight;
    status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)(&rsp_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
    log_assert(status == osOK);
  }

 /*向protocol_task回应0点校准结果*/
 if(req_msg.type ==  PROTOCOL_TASK_MSG_REQ_CALIBRATE_ZERO){
    modbus_set_slave(ctx,req_msg.reserved);
    rsp_msg.value = SCALE_TASK_SUCCESS;
    write_value[0]=SCALE_TASK_CALIBRATE_AUTO_VALUE >> 16;
    write_value[1]=SCALE_TASK_CALIBRATE_AUTO_VALUE & 0xFFFF;
    rc = modbus_write_registers(ctx,SCALE_TASK_CALIBRATE_ZERO_CODE_REG_ADDR,SCALE_TASK_CALIBRATE_ZERO_CODE_REG_CNT,write_value);
    if(rc <= 0){
       /*校准错误直接返回失败*/
       rsp_msg.value = SCALE_TASK_FAILURE;
       goto calibrate_zero_msg_handle;
    } 
   write_value[0] = 0;
   write_value[1] = 0;
   rc = modbus_write_registers(ctx,SCALE_TASK_CALIBRATE_ZERO_MEASURE_REG_ADDR,SCALE_TASK_CALIBRATE_ZERO_MEASURE_REG_CNT,write_value);
   if(rc <= 0){
      /*校准错误直接返回失败*/
      rsp_msg.value = SCALE_TASK_FAILURE;
      goto calibrate_zero_msg_handle;
  } 
calibrate_zero_msg_handle:    
   rsp_msg.type = PROTOCOL_TASK_MSG_RESPONSE_CALIBRATE_ZERO;
   status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)(&rsp_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
   log_assert(status == osOK);
  }
 
  /*向protocol_task回应full点校准结果*/
  if(req_msg.type ==  PROTOCOL_TASK_MSG_REQ_CALIBRATE_FULL){
     modbus_set_slave(ctx,req_msg.reserved);
     rsp_msg.value = SCALE_TASK_SUCCESS;
     write_value[0]=SCALE_TASK_CALIBRATE_AUTO_VALUE >> 16;
     write_value[1]=SCALE_TASK_CALIBRATE_AUTO_VALUE & 0xFFFF;
     rc = modbus_write_registers(ctx,SCALE_TASK_CALIBRATE_FULL_CODE_REG_ADDR,SCALE_TASK_CALIBRATE_FULL_CODE_REG_CNT,write_value);
     if(rc <= 0){
        /*校准错误直接返回失败*/
        rsp_msg.value = SCALE_TASK_FAILURE;
        goto calibrate_full_msg_handle;
     } 
     write_value[0] = 0;
     write_value[1] = req_msg.value;
     rc = modbus_write_registers(ctx,SCALE_TASK_CALIBRATE_FULL_MEASURE_REG_ADDR,SCALE_TASK_CALIBRATE_FULL_MEASURE_REG_CNT,write_value);
     if(rc <= 0){
       /*校准错误直接返回失败*/
        rsp_msg.value = SCALE_TASK_FAILURE;
        goto calibrate_full_msg_handle;
     } 
calibrate_full_msg_handle:    
     rsp_msg.type = PROTOCOL_TASK_MSG_RESPONSE_CALIBRATE_FULL;
     status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)(&rsp_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
     log_assert(status == osOK);  
  } 
  
 /*向protocol_task回应remov tar weight结果*/
 if(req_msg.type ==  PROTOCOL_TASK_MSG_REQ_REMOVE_TAR_WEIGHT){
    modbus_set_slave(ctx,req_msg.value);
    rsp_msg.value = SCALE_TASK_SUCCESS;
    write_value[0]=SCALE_TASK_CLEAR_ZERO_WEIGHT_VALUE;
    rc = modbus_write_registers(ctx,SCALE_TASK_CLEAR_ZERO_WEIGHT_REG_ADDR,SCALE_TASK_CLEAR_ZERO_WEIGHT_REG_CNT,write_value);
    if(rc <= 0){
      /*置零误直接返回失败*/
       rsp_msg.value = SCALE_TASK_FAILURE;
       goto remov_tar_weight_msg_handle;
    }   
    write_value[0]=SCALE_TASK_REMOVE_TAR_WEIGHT_VALUE >> 16;
    write_value[1]=SCALE_TASK_REMOVE_TAR_WEIGHT_VALUE & 0xFFFF;
    rc = modbus_write_registers(ctx,SCALE_TASK_REMOVE_TAR_WEIGHT_REG_ADDR,SCALE_TASK_REMOVE_TAR_WEIGHT_REG_CNT,write_value);
    if(rc <= 0){
       /*去皮错误直接返回失败*/
       rsp_msg.value = SCALE_TASK_FAILURE;
       goto remov_tar_weight_msg_handle;
    } 
remov_tar_weight_msg_handle:    
    rsp_msg.type = PROTOCOL_TASK_MSG_RESPONSE_REMOVE_TAR_WEIGHT;
    status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)(&rsp_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
    log_assert(status == osOK);  
  }   
 
  /*向protocol_task回应set slave addr结果*/
  if(req_msg.type ==  PROTOCOL_TASK_MSG_REQ_SET_ADDR){
     modbus_set_slave(ctx,req_msg.value);
     rsp_msg.value = SCALE_TASK_SUCCESS;
     write_value[0] = SCALE_TASK_SYS_UNLOCK_VALUE;
     rc = modbus_write_registers(ctx,SCALE_TASK_UNLOCK_REG_ADDR,SCALE_TASK_UNLOCK_REG_CNT,write_value);
     if(rc <= 0){
       /*系统解锁错误直接返回失败*/
        rsp_msg.value  = SCALE_TASK_FAILURE;
        goto set_addr_msg_handle;
     }
     write_value[0]= req_msg.reserved;
     rc = modbus_write_registers(ctx,SCALE_TASK_ADDR_REG_ADDR,SCALE_TASK_ADDR_REG_CNT,write_value);
     if(rc <= 0){
       /*设置地址错误直接返回失败*/
       rsp_msg.value  = SCALE_TASK_FAILURE;
       goto set_addr_msg_handle;
     } 

set_addr_msg_handle:    
   rsp_msg.type = PROTOCOL_TASK_MSG_RESPONSE_SET_ADDR;
    status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)(&rsp_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
    log_assert(status == osOK); 
  } 
 
  /*向protocol_task回应传感器ID*/
  if(req_msg.type ==  PROTOCOL_TASK_MSG_REQ_SENSOR_ID){
     rsp_msg.type = PROTOCOL_TASK_MSG_RESPONSE_SENSOR_ID;
     rsp_msg.value = SENSOR_ID;
     status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)(&rsp_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
     log_assert(status == osOK);      
  }
 
 
  /*向protocol_task回应版本号*/
  if(req_msg.type ==  PROTOCOL_TASK_MSG_REQ_VERSION){
     rsp_msg.type = PROTOCOL_TASK_MSG_RESPONSE_VERSION;
     rsp_msg.value = FIRMWARE_VERSION_HEX;
     status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)(&rsp_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
     log_assert(status == osOK);   
  } 
  
  
 }
 }
}