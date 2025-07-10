#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_

#include <stddef.h>
#include <stdint.h>

typedef void (*Task_t)(void);

typedef struct {
    Task_t   pTask;
    uint32_t Period; // in milliseconds
    uint32_t Delay;  // in milliseconds

    uint32_t TaskID; // Unique identifier for the task

} sTasks;

extern sTasks            SCH_tasks[];        // Array of tasks in the scheduler
extern uint8_t           current_task_index; // Current number of tasks in the scheduler
extern uint8_t           task_id_counter;    // Counter for generating unique TaskIDs
extern volatile uint32_t SCH_tick_counter;   // Current tick count

#define SCH_MAX_TASKS 32 // Maximum number of tasks
#define TICKS         10 // 10 ms tick

uint32_t get_tick(void);
void     SCH_Init(void);
void     SCH_Add_Task(Task_t, uint32_t DELAY, uint32_t PERIOD);
void     SCH_Update(void);
void     SCH_Dispatch_Tasks(void);
void     SCH_Delete_Task(uint32_t TaskID);

#endif /* INC_SCHEDULER_H_ */
