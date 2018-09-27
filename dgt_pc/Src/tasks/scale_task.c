#include "board.h"
#include "cmsis_os.h"
#include "serial.h"
#include "modbus.h"
#include "task_msg.h"
#include "scale_task.h"
#include "protocol_task.h"
#include "version.h"
#include "sensor_id.h"
#include "log.h"
#define LOG_MODULE_NAME   "[scale]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG 

osThreadId scale_task_hdl;
osMessageQId scale_task_msg_q_id;
extern int modbus_serial_handle;
task_msg_t protocol_msg;

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

/*从地址0-0xfe逐个尝试读地址，直到找到为止*/
static int scale_task_probe_slave_addr(modbus_t *ctx)
{
uint8_t addr;
uint16_t read_value[1];
int rc;

for(addr = 0; addr <= SCLAE_TASK_DEFAULT_SLAVE_ADDR;addr++){
modbus_set_slave(ctx,addr); 
rc = modbus_read_registers(ctx,SCALE_TASK_ADDR_REG_ADDR,SCALE_TASK_ADDR_REG_CNT,read_value);
if(rc > 0){
log_debug("probe addr ok.slave addr :%d.\r\n",addr);
return addr;
}
}
log_error("probe addr err.can not find addr.\r\n");  
return -1;
}


void scale_task(void const * argument)
{
 osEvent os_msg;
 osStatus status;
 task_msg_t *msg;
 int rc;
 int result; 
 int slave_addr;
 modbus_t *ctx;
 uint16_t  read_value[2];
 uint16_t  write_value[2];
 int16_t   net_weight;
 
 osMessageQDef(scale_task_msg_q,6,uint32_t);
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
 slave_addr = scale_task_probe_slave_addr(ctx);
 if(slave_addr < 0){
 slave_addr = SCLAE_TASK_DEFAULT_SLAVE_ADDR;
 log_error("slave addr default:%d.\r\n",slave_addr);
 }
 rc = modbus_set_slave(ctx,slave_addr); 
 log_assert(rc == 0);
 rc = modbus_connect(ctx);
 log_assert(rc == 0);
 
 while(1){   
 os_msg = osMessageGet(scale_task_msg_q_id,SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE);
 
 if(os_msg.status == osEventMessage){
 msg = (task_msg_t*)os_msg.value.v;
 
 /*向protocol_task回应净重值*/
 if(msg->type ==  REQ_NET_WEIGHT){
  rc = modbus_read_registers(ctx,SCALE_TASK_NET_WEIGHT_REG_ADDR,SCALE_TASK_NET_WEIGHT_REG_CNT,read_value);
  if(rc != 0){
  net_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
  }else{
  net_weight = read_value[0]<<16 |read_value[1];
  if(net_weight >= SCALE_TASK_MAX_WEIGHT_VALUE ||
     net_weight <= SCALE_TASK_MIN_WEIGHT_VALUE){
  net_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
  }
 }
 protocol_msg.type = RESPONSE_NET_WEIGHT;
 protocol_msg.net_weight = net_weight;
 status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
 log_assert(status == osOK);
 }

 /*向protocol_task回应0点校准结果*/
 if(msg->type ==  REQ_CALIBRATE_ZERO){
  result = SCALE_TASK_SUCCESS;
  write_value[0]=SCALE_TASK_CALIBRATE_AUTO_VALUE >> 16;
  write_value[1]=SCALE_TASK_CALIBRATE_AUTO_VALUE & 0xFFFF;
  rc = modbus_write_registers(ctx,SCALE_TASK_CALIBRATE_ZERO_CODE_REG_ADDR,SCALE_TASK_CALIBRATE_ZERO_CODE_REG_CNT,write_value);
  if(rc != 0){
  /*校准错误直接返回失败*/
  result = SCALE_TASK_FAILURE;
  goto calibrate_zero_msg_handle;
  } 
  rc = modbus_write_registers(ctx,SCALE_TASK_CALIBRATE_ZERO_MEASURE_REG_ADDR,SCALE_TASK_CALIBRATE_ZERO_MEASURE_REG_CNT,write_value);
  if(rc != 0){
  /*校准错误直接返回失败*/
  result = SCALE_TASK_FAILURE;
  goto calibrate_zero_msg_handle;
  } 
calibrate_zero_msg_handle:    
    protocol_msg.type = RESPONSE_CALIBRATE_ZERO;
    protocol_msg.result = result;
    status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);  
  }
 
 /*向protocol_task回应full点校准结果*/
 if(msg->type ==  REQ_CALIBRATE_FULL){
  result = SCALE_TASK_SUCCESS;
  write_value[0]=SCALE_TASK_CALIBRATE_AUTO_VALUE >> 16;
  write_value[1]=SCALE_TASK_CALIBRATE_AUTO_VALUE & 0xFFFF;
  rc = modbus_write_registers(ctx,SCALE_TASK_CALIBRATE_FULL_CODE_REG_ADDR,SCALE_TASK_CALIBRATE_FULL_CODE_REG_CNT,write_value);
  if(rc != 0){
  /*校准错误直接返回失败*/
  result = SCALE_TASK_FAILURE;
  goto calibrate_full_msg_handle;
  } 
  rc = modbus_write_registers(ctx,SCALE_TASK_CALIBRATE_FULL_MEASURE_REG_ADDR,SCALE_TASK_CALIBRATE_FULL_MEASURE_REG_CNT,write_value);
  if(rc != 0){
  /*校准错误直接返回失败*/
  result = SCALE_TASK_FAILURE;
  goto calibrate_full_msg_handle;
  } 
calibrate_full_msg_handle:    
    protocol_msg.type = RESPONSE_CALIBRATE_FULL;
    protocol_msg.result = result;
    status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);  
  } 
  
 /*向protocol_task回应remov tar weight结果*/
 if(msg->type ==  REQ_REMOVE_TAR_WEIGHT){
  result = SCALE_TASK_SUCCESS;
  write_value[0]=SCALE_TASK_REMOVE_TAR_WEIGHT_VALUE >> 16;
  write_value[1]=SCALE_TASK_REMOVE_TAR_WEIGHT_VALUE & 0xFFFF;
  rc = modbus_write_registers(ctx,SCALE_TASK_REMOVE_TAR_WEIGHT_REG_ADDR,SCALE_TASK_REMOVE_TAR_WEIGHT_REG_CNT,write_value);
  if(rc != 0){
  /*去皮错误直接返回失败*/
  result = SCALE_TASK_FAILURE;
  goto remov_tar_weight_msg_handle;
  } 

remov_tar_weight_msg_handle:    
    protocol_msg.type = RESPONSE_REMOVE_TAR_WEIGHT;
    protocol_msg.result = result;
    status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);  
  }   
 
 /*向protocol_task回应set slave addr结果*/
 if(msg->type ==  REQ_SET_ADDR){
  result = SCALE_TASK_SUCCESS;
  write_value[0] = SCALE_TASK_SYS_UNLOCK_VALUE;
  rc = modbus_write_registers(ctx,SCALE_TASK_UNLOCK_REG_ADDR,SCALE_TASK_UNLOCK_REG_CNT,write_value);
  if(rc != 0){
  /*系统解锁错误直接返回失败*/
  result = SCALE_TASK_FAILURE;
  goto set_addr_msg_handle;
  }
  write_value[0]=msg->scale_addr;
  rc = modbus_write_registers(ctx,SCALE_TASK_ADDR_REG_ADDR,SCALE_TASK_ADDR_REG_CNT,write_value);
  if(rc != 0){
  /*设置地址错误直接返回失败*/
  result = SCALE_TASK_FAILURE;
  goto set_addr_msg_handle;
  } 

set_addr_msg_handle:    
  if(result == SCALE_TASK_SUCCESS){
    slave_addr = msg->scale_addr;
    }
    protocol_msg.type = RESPONSE_SET_ADDR;
    protocol_msg.result = result;
    status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);  
  } 
  
 
 
  /*向protocol_task回应当前地址值*/
  if(msg->type ==  REQ_ADDR){
   slave_addr = modbus_get_slave(ctx);
   if(slave_addr < 0){
   slave_addr = SCLAE_TASK_DEFAULT_SLAVE_ADDR;
   }
   protocol_msg.type = RESPONSE_ADDR;
   protocol_msg.scale_addr = slave_addr;
   status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
   log_assert(status == osOK);   
  } 
 
 
  /*向protocol_task回应传感器ID*/
  if(msg->type ==  REQ_SENSOR_ID){
   protocol_msg.type = RESPONSE_SENSOR_ID;
   protocol_msg.sensor_id = SENSOR_ID;
   status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
   log_assert(status == osOK);     
  }
 
 
  /*向protocol_task回应版本号*/
  if(msg->type ==  REQ_VERSION){
   protocol_msg.type = RESPONSE_VERSION;
   protocol_msg.version = FIRMWARE_VERSION;
   status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
   log_assert(status == osOK);   
  } 
 }
 }
}