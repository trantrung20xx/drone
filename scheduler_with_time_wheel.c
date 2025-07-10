#include "scheduler.h"

#define TIME_WHEEL_SIZE 100 // Số slot, mỗi slot ứng với 1 tick (10ms), tổng cộng 1 giây

typedef struct TaskNode {
    sTasks           task;
    struct TaskNode* next;
} TaskNode;

TaskNode* time_wheel[TIME_WHEEL_SIZE] = {0};
uint32_t  current_slot                = 0;

void SCH_Init(void) {
    for (uint32_t i = 0; i < TIME_WHEEL_SIZE; i++) {
        time_wheel[i] = NULL;
    }
    current_slot = 0;
}

void SCH_Add_Task(Task_t pTask, uint32_t DELAY, uint32_t PERIOD) {
    uint32_t ticks_delay = DELAY / TICKS;
    uint32_t slot        = (current_slot + ticks_delay) % TIME_WHEEL_SIZE;

    TaskNode* node    = (TaskNode*)malloc(sizeof(TaskNode));
    node->task.pTask  = pTask;
    node->task.Period = PERIOD / TICKS;
    node->task.Delay  = ticks_delay;
    node->task.RunMe  = 0;
    node->task.TaskID = (uint32_t)node; // Dùng địa chỉ làm ID
    node->next        = time_wheel[slot];
    time_wheel[slot]  = node;
}

void SCH_Update(void) {
    TaskNode* node = time_wheel[current_slot];
    while (node) {
        node->task.RunMe++;
        node = node->next;
    }
    current_slot = (current_slot + 1) % TIME_WHEEL_SIZE;
}

void SCH_Dispatch_Tasks(void) {
    TaskNode** pnode = &time_wheel[current_slot];
    while (*pnode) {
        if ((*pnode)->task.RunMe > 0) {
            (*pnode)->task.pTask();
            (*pnode)->task.RunMe = 0;
            // Nếu là task lặp lại, lên lịch lại
            if ((*pnode)->task.Period > 0) {
                SCH_Add_Task(
                    (*pnode)->task.pTask,
                    (*pnode)->task.Period * TICKS,
                    (*pnode)->task.Period * TICKS
                );
            }
            // Xóa task khỏi slot hiện tại
            TaskNode* to_delete = *pnode;
            *pnode              = (*pnode)->next;
            free(to_delete);
        } else {
            pnode = &((*pnode)->next);
        }
    }
}

void SCH_Delete_Task(uint32_t TaskID) {
    for (uint32_t i = 0; i < TIME_WHEEL_SIZE; i++) {
        TaskNode** pnode = &time_wheel[i];
        while (*pnode) {
            if ((*pnode)->task.TaskID == TaskID) {
                TaskNode* to_delete = *pnode;
                *pnode              = (*pnode)->next;
                free(to_delete);
                return;
            } else {
                pnode = &((*pnode)->next);
            }
        }
    }
}
