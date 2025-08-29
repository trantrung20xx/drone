#include "scheduler.h"

sTasks            SCH_tasks[SCH_MAX_TASKS];
uint8_t           current_task_index = 0; // Current number of tasks in the scheduler
uint8_t           task_id_counter    = 1; // Counter for generating unique TaskIDs
volatile uint32_t SCH_tick_counter   = 0; // Current tick count

uint32_t get_tick(void)
{
    return SCH_tick_counter; // Return the current tick count
}

inline void SCH_Init(void)
{
    current_task_index = 0; // Reset task index
    task_id_counter    = 1; // Reset task ID counter
    SCH_tick_counter   = 0; // Reset tick counter
}

// Swap two tasks in the scheduler
void swap(sTasks *a, sTasks *b)
{
    sTasks tmp = *a;
    *a         = *b;
    *b         = tmp;
}

void heapify_up(uint32_t idx)
{
    while (idx > 0)
    {
        uint32_t parent = (idx - 1) / 2;
        if ((int32_t)(SCH_tasks[idx].Delay - SCH_tasks[parent].Delay) < 0)
        {
            swap(&SCH_tasks[idx], &SCH_tasks[parent]);
            idx = parent;
        }
        else
        {
            break;
        }
    }
}

void heapify_down(uint32_t idx)
{
    while (1)
    {
        uint32_t left     = 2 * idx + 1;
        uint32_t right    = 2 * idx + 2;
        uint32_t smallest = idx;
        if (left < current_task_index &&
            (int32_t)(SCH_tasks[left].Delay - SCH_tasks[smallest].Delay) < 0)
            smallest = left;
        if (right < current_task_index &&
            (int32_t)(SCH_tasks[right].Delay - SCH_tasks[smallest].Delay) < 0)
            smallest = right;
        if (smallest != idx)
        {
            swap(&SCH_tasks[idx], &SCH_tasks[smallest]);
            idx = smallest;
        }
        else
            break; // No more swaps needed
    }
}

void SCH_Add_Task(Task_t pTask, uint32_t DELAY, uint32_t PERIOD)
{
    if (current_task_index >= SCH_MAX_TASKS)
    {
        return; // Scheduler is full
    }

    // Initialize the new task
    uint32_t now                         = get_tick();
    SCH_tasks[current_task_index].pTask  = pTask;
    SCH_tasks[current_task_index].Period = PERIOD / TICKS;        // Convert to ticks
    SCH_tasks[current_task_index].Delay  = now + (DELAY / TICKS); // Convert to ticks
    SCH_tasks[current_task_index].TaskID = task_id_counter++;     // Assign a unique TaskID
    current_task_index++;                                         // Add a new task to the scheduler
    heapify_up(current_task_index - 1);                           // Reheapify the heap
}

inline void SCH_Update(void)
{
    SCH_tick_counter++; // Increment the scheduler tick
}

void SCH_Dispatch_Tasks(void)
{
    while (current_task_index > 0)
    {
        uint32_t now = get_tick();

        // if root not ready yet (using signed diff to handle wrap), exit
        if ((int32_t)(now - SCH_tasks[0].Delay) < 0)
            break;

        // copy root task (safer if pTask modifies scheduler)
        sTasks task = SCH_tasks[0];

        // Execute task (note: if task calls scheduler APIs, behavior may be complex)
        task.pTask();

        if (task.Period > 0)
        {
            // compute next absolute delay robustly (catch up missed periods)
            uint32_t now2 = get_tick();
            uint32_t next = task.Delay + task.Period;
            if ((int32_t)(next - now2) <= 0)
            {
                uint32_t diff = now2 - task.Delay;
                uint32_t k    = diff / task.Period + 1;
                task.Delay += k * task.Period;
            }
            else
            {
                task.Delay = next;
            }

            // place updated task at root and heapify down
            SCH_tasks[0] = task;
            heapify_down(0);
        }
        else
        {
            // one-shot: remove root and restore heap
            SCH_tasks[0] = SCH_tasks[current_task_index - 1];
            current_task_index--;
            if (current_task_index > 0)
                heapify_down(0);
        }
    }
}

void SCH_Delete_Task(uint32_t TaskID)
{
    for (uint32_t i = 0; i < current_task_index; i++)
    {
        if (SCH_tasks[i].TaskID == TaskID)
        {
            // Replace the task to be deleted with the last task in the heap
            SCH_tasks[i] = SCH_tasks[current_task_index - 1];
            current_task_index--; // Decrease the task count
            // try moving up (if new element is earlier), then down (if later)
            heapify_up(i);
            if (i < current_task_index) // Check if i is still valid
                heapify_down(i);
            return; // Task found and deleted
        }
    }
}
