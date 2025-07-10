#include "scheduler.h"

sTasks  SCH_tasks[SCH_MAX_TASKS];
volatile uint8_t current_task_index = 0;

void SCH_Init(void) {
    for (uint8_t i = 0; i < SCH_MAX_TASKS; i++) {
        SCH_tasks[i].pTask  = NULL;
        SCH_tasks[i].Period = 0;
        SCH_tasks[i].Delay  = 0;
        SCH_tasks[i].RunMe  = 0;
        SCH_tasks[i].TaskID = i; // Assign a unique TaskID
    }
}

void SCH_Add_Task(Task_t pTask, uint32_t DELAY, uint32_t PERIOD) {
    if (current_task_index < SCH_MAX_TASKS) {
        SCH_tasks[current_task_index].pTask  = pTask;
        SCH_tasks[current_task_index].Period = PERIOD / TICKS; // Convert to ticks
        SCH_tasks[current_task_index].Delay  = DELAY / TICKS;  // Convert to ticks
        SCH_tasks[current_task_index].RunMe  = 0;
        current_task_index++;
    }
}

void SCH_Update(void) {
    for (uint8_t i = 0; i < current_task_index; i++) {
        if (SCH_tasks[i].pTask != NULL) {
            if (SCH_tasks[i].Delay > 0) {
                SCH_tasks[i].Delay--;
            } else {
                if (SCH_tasks[i].Period > 0) {
                    // Reset delay after it reaches zero
                    SCH_tasks[i].Delay = SCH_tasks[i].Period;
                }
                SCH_tasks[i].RunMe++;
            }
        }
    }
}

void SCH_Dispatch_Tasks(void) {
    for (uint8_t i = 0; i < current_task_index; i++) {
        if (SCH_tasks[i].RunMe > 0) {
            SCH_tasks[i].pTask(); // Execute the task
            SCH_tasks[i].RunMe--; // Decrement RunMe after executing

            if (SCH_tasks[i].Period > 0) {
                // If the task has a period, reset the delay
                SCH_tasks[i].Delay = SCH_tasks[i].Period;
            } else {
                // If the task has no period, remove it
                SCH_Delete_Task(SCH_tasks[i].TaskID);
                i--; // Adjust index after deletion
            }
        }
    }
}

void SCH_Delete_Task(uint32_t TaskID) {
    for (uint8_t i = 0; i < current_task_index; i++) {
        if (SCH_tasks[i].TaskID == TaskID) {
            // Shift tasks down to remove the task
            for (uint8_t j = i; j < current_task_index - 1; j++) {
                SCH_tasks[j] = SCH_tasks[j + 1];
            }
            SCH_tasks[current_task_index - 1].pTask  = NULL; // Clear the last task
            SCH_tasks[current_task_index - 1].Period = 0;
            SCH_tasks[current_task_index - 1].Delay  = 0;
            SCH_tasks[current_task_index - 1].RunMe  = 0;
            SCH_tasks[current_task_index - 1].TaskID = 0; // Reset the last task ID
            current_task_index--;
            break;
        }
    }
}
