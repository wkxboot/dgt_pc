#ifndef  __TASKS_INIT_H__
#define  __TASKS_INIT_H__


extern EventGroupHandle_t tasks_sync_evt_group_hdl;


#define  TASKS_SYNC_EVENT_SCALE_TASK_RDY            (1<<0)
#define  TASKS_SYNC_EVENT_PROTOCOL_TASK_RDY         (1<<1)
#define  TASKS_SYNC_EVENT_WDT_TASK_RDY              (1<<2)
#define  TASKS_SYNC_EVENT_ALL_TASKS_RDY             ((1<<2)-1)








#endif