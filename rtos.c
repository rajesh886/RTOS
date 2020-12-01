// RTOS Framework - Fall 2020
// J Losh

// Student Name:
// Rajesh Rayamajhi

// Add xx_ prefix to all files in your project
// xx_rtos.c
// xx_tm4c123gh6pm_startup_ccs.c
// xx_other files
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDS on these pins:
// Blue:   PF2 (on-board)
// Red:    PE1
// Orange: PE2
// Yellow: PE3
// Green:  PE4
// PBs on these pins
// PB0:    PA2
// PB1:    PA3
// PB2:    PA4
// PB3:    PA5
// PB4:    PA6
// PB5:    PA7
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "wait.h"

// bitbanding references for the off-board and on-board LEDs
#define BLUE_LED         (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define RED_LED          (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))
#define ORANGE_LED       (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))
#define YELLOW_LED       (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4)))
#define GREEN_LED        (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))

//bitbanding addresses for PUSH BUTTONS
//all the push buttons are on the port A
#define PUSH_BUTTON0  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))
#define PUSH_BUTTON1  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON2  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define PUSH_BUTTON3  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))
#define PUSH_BUTTON4  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define PUSH_BUTTON5  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))

//Port E masks
#define BLUE_LED_MASK 4
#define RED_LED_MASK 2
#define ORANGE_LED_MASK 4
#define YELLOW_LED_MASK 8
#define GREEN_LED_MASK 16


//Port A masks
#define PUSH_BUTTON_MASK0 4
#define PUSH_BUTTON_MASK1 8
#define PUSH_BUTTON_MASK2 16
#define PUSH_BUTTON_MASK3 32
#define PUSH_BUTTON_MASK4 64
#define PUSH_BUTTON_MASK5 128


//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

//User data for shell commands is processed here.
#define MAX_CHARS 80
#define MAX_FIELDS 5
typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
}USER_DATA;

//this struct is used by shell thread to print the processes information like task name, pid, cpu percentage, priority and state of the tasks.
typedef struct _shell_data
{
    char data[12];                    //temporary char array for storing the char characters for printing the char array and copying to the 2d char array.
    uint16_t tasks_total;             //keeps track of number of tasks
    char tasks_name[10][10];          //this 2d char array stores the name of the tasks
    uint32_t pid[10];                 //pid of task is stored here.
    uint16_t cputime[10];             //cpu of tasks
    char priority[10][3];             //priority of each task
    uint8_t state[10];                //state of each task
    char delay[12][5];                //if any task is delayed, then the number of ms of delay is stored here.
    char resources[12][11];           //if any task is blocked, then name of the semaphore that is blocking is stored here.
    uint16_t kerneltime;              //time used by kernel to make multi-threading is stored here.
    uint8_t semaphores_total;         //number of semaphores
    char semaphores_names[5][12];     //semaphore names are stored here
    char semaphores_count[5][4];      //count variable of each semaphore is stored here.
    char waiting_processes[5][12];    //name of the task that is blocked by the semaphore is stored.
    uint8_t tempState;
}shell_data;

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
typedef struct _semaphore
{
    uint16_t count;                         //no of items in semaphore
    uint16_t queueSize;                     //no of items waiting to use process
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    uint32_t lastWaitThread;
    char name[16];
} semaphore;

semaphore semaphores[MAX_SEMAPHORES];

uint8_t semaphoreCount = 1;

//uint8_t keyPressed, keyReleased, flashReq, resource;

#define keyPressed  1
#define keyReleased 2
#define flashReq    3
#define resource    4

// task
#define STATE_INVALID      0 // no task
#define STATE_UNRUN        1 // task has never been run
#define STATE_READY        2 // has run, can resume at any time
#define STATE_DELAYED      3 // has run, but now awaiting timer
#define STATE_BLOCKED      4 // has run, but now blocked by semaphore
#define STATE_SUSPENDED    5 // has run, but now killed by user

#define MAX_TASKS 12                // maximum number of valid tasks
uint8_t taskCurrent = 0;            // index of last dispatched task
uint8_t taskCount = 0;              // total number of valid tasks
uint32_t initial_time = 0;          //initial recording of time for the task that is running
uint32_t final_time = 0;            //final recording of time for the task that was running before the task was switched in pendsv
uint32_t time_difference;           //time difference of initial time and final time
uint32_t total_time = 0;            //total time of all the tasks
uint32_t total_time_temp = 0;       //temporary variable to store the total time taken by all the tasks for the calculation
uint32_t j = 0;                     //global variable to swap the buffer
uint32_t N = 0;                     //variable to keep track of the Number of times the systick isr is called.
int8_t sched_mode;                  //round robin or priority
int8_t preempt_mode = 0;            //preemption on = true and off = false
int8_t pi_mode = 0;                 //priority inheritance on or off
int8_t mpu_mode = 0;                //0 - mpu off and 1 - mpu on
uint8_t count = 0;

// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // location of original stack pointer
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // 0=highest to 15=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
    uint32_t task_time;            //time taken by each task
    uint32_t swapBuffer[2];        //this buffer stores the previous and current time for the task depending upon the value of j
    uint32_t permissionmask;       //variable that determines the access of the region of each task in MPU
} tcb[MAX_TASKS];

//Allocating 28KiB space for the heap in stack...2KiB for the MSP stack and remaining 2KiB for the OS variables/kernel
#pragma DATA_SECTION(mystack, ".heap")
uint32_t mystack[7168];
uint32_t heap = (uint32_t*)mystack ;    //pointer to the beginning address of the heap in my stack
                                        //the beginning address of the heap is 0x20001000

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
}

// REQUIRED: Implement prioritization to 16 levels
uint32_t last_searched_task[16];   //stores the last searched task for each priority. If all the tasks are searched for the current priority, then the beginning index for the last searched task
                                    //starts from the index 0 for the current priority
uint16_t priority_num = 0;        // global variable that keeps track of priority number
uint8_t run_task = 16;             //global variable that is used to run the task idle. Since idle is at index 0 and priority 15, start the index searching from index 0.
                                  // When idle runs, run_task variable becomes zero and the priority scheduling for the idle starts from the index 1. Otherwise, the prev_task
                                  //always becomes zero for the idle and the other tasks never gets chance to run and stays in priority level 15 forever hanging there.
int rtosScheduler()
{
    bool ok;
    ok = false;
    //Round Robin Scheduler
    if(sched_mode == 1)
    {
        bool ok;
        static uint8_t task = 0xFF;
        ok = false;
        while (!ok)
        {
            task++;
            if (task >= MAX_TASKS)
                task = 0;
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        }
        return task;
    }

    //default is priority scheduling
    else if(sched_mode == 0)
    {
        while (!ok)
        {
            //get the last dispatched task for the current priority
            uint8_t prev_task = last_searched_task[priority_num];
            //if the last dispatched task was not zero then begin the task scheduling from the
            //last dispatched task + 1. The last dispatched task was already scheduled
            if((prev_task > 0) || (run_task == 0))
            {
                prev_task++;
            }
            //go through all the tasks for the current priority
            while(prev_task < MAX_TASKS)
            {
                if((tcb[prev_task].currentPriority == priority_num && (tcb[prev_task].state == STATE_UNRUN || tcb[prev_task].state == STATE_READY)))
                {
                    //add the last dispatched task in the array
                    last_searched_task[priority_num] = prev_task;
                    // if the priority level is 15, reset run_task to zero.
                    //run_task becomes zero and the task index starts from 1 for the priority number 15.
                    if(priority_num == 15)
                    {
                        run_task = prev_task;
                    }
                    ok = true;
                    return prev_task;
                }
                prev_task++;
            }

            // if we go through all the tasks then set the last dispatched task to be 0
            //increment the priority level
            if(prev_task == MAX_TASKS)
            {
               last_searched_task[priority_num] = 0;
               priority_num = priority_num + 1;
            }
            // if the priority level reaches to max then reset back to zero
            if(priority_num == 16) priority_num = 0;
        }
    }
}

bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0;
    bool found = false;
    // REQUIRED: store the thread name
    // add task if room in task list
    // allocate stack space for a thread and assign to sp below
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            //allocating the stack size for the current task
            tcb[i].spInit = heap + stackBytes;
            tcb[i].sp = heap;
            //stack size gets incremented for the next task
            heap = heap + stackBytes;
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            copy(name,tcb[i].name);
            //adding the permission task bit field for the current task
            //task 0 -> 1, task 1 -> 2, task 2 -> 4, task 3 -> 8, task 4 -> 16, task 5 -> 31, task 6 -> 64, task 7 -> 128, task 8 -> 256, task 9 -> 512, task 10 -> 1024, task 11 -> 2048
            //uint16_t num = (tcb[i].sp - 0x20000000) / 1024;
            tcb[i].permissionmask = 1<<i;
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(_fn fn)
{
    __asm(" SVC #15");
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void destroyThread(_fn fn)
{
    __asm(" SVC #16");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    __asm(" SVC #17");
}

//int8_t createSemaphore(uint8_t count, char str[])
//{
//    int8_t index = -1;
//    if (semaphoreCount < MAX_SEMAPHORES)
//    {
//        semaphores[semaphoreCount].count = count;
//        copy(str,semaphores[semaphoreCount].name);
//        index = semaphoreCount;
//        semaphoreCount++;
//    }
//    return index;
//}

bool createSemaphore(uint8_t s, uint8_t count, char str[])
{
    bool ok = (s<MAX_SEMAPHORES);
    if(ok)
    {
        semaphores[s].count = count;
        copy(str,semaphores[s].name);
        semaphoreCount++;
        return ok;
    }
}

// REQUIRED: modify this function to start the operating system, using all created tasks
void startRtos()
{
    taskCurrent = rtosScheduler();
    uint32_t SP = tcb[taskCurrent].spInit;
    //when fn is called, the stack pointer(PSP) goes up
    //so need to subtract PSP by 8 byte.
    setPSP(SP-8);

    TIMER1_CTL_R |= TIMER_CTL_TAEN;                                 // turn-on timer before the thread starts to work

    tcb[taskCurrent].state = STATE_READY;
    setASP();

    _fn fn = tcb[taskCurrent].pid;

    //Initializing MPU Control register. It is disabled at first.
      NVIC_MPU_CTRL_R |= 0x00000000;

    //Enabling MPU Region for RAM, peripherals, and bitbanded addresses
      NVIC_MPU_NUMBER_R = 0x0;
    //Configuring base register
      NVIC_MPU_BASE_R |= 0x00000000;          //base address of the RAM, peripherals and bitbanded region.
      NVIC_MPU_BASE_R |= 0x00000000;          //disabling valid region number
      NVIC_MPU_BASE_R |= 0x0;                 //Region rule is 0.
    // Configuration of Attribute Register
      NVIC_MPU_ATTR_R |= 0x10000000;          // Instruction fetches are disabled.
      NVIC_MPU_ATTR_R |= 0x03000000;          // Full Access Mode
      NVIC_MPU_ATTR_R |= 0x00000000;          // Setting TEX field for Flash
      NVIC_MPU_ATTR_R |= 0x00040000;          // Shareable
      NVIC_MPU_ATTR_R |= 0x00000000;          // Not Cacheable
      NVIC_MPU_ATTR_R |= 0x00000000;          // Not Bufferable
      NVIC_MPU_ATTR_R |= 0x00000000;          // Sub region is enabled.
      NVIC_MPU_ATTR_R |= 0x0000003E;          // Region Size Mask for RAM, peripheral and bitbanded (Size = log2(4*1024*1024*1024)-1 )
      NVIC_MPU_ATTR_R |= 0x01;                // Region Enabled


      //Enabling MPU Region for 256 KiB flash
      NVIC_MPU_NUMBER_R = 0x1;
      //Configuring base register
      NVIC_MPU_BASE_R |= 0x00000000;      //base address of the Flash region.
      NVIC_MPU_BASE_R |= 0x00000000;      //disabling valid region number
      NVIC_MPU_BASE_R |= 0x1;             //Region rule is 1.
      //Configuration of Attribute Register
      NVIC_MPU_ATTR_R |= 0x00000000;      // Instruction fetches are enabled.
      NVIC_MPU_ATTR_R |= 0x03000000;      // Full Access Mode
      NVIC_MPU_ATTR_R |= 0x00000000;      // Setting TEX field for Flash
      NVIC_MPU_ATTR_R |= 0x00000000;      // Not Shareable
      NVIC_MPU_ATTR_R |= 0x00020000;      // Cacheable
      NVIC_MPU_ATTR_R |= 0x00000000;      // Not Bufferable
      NVIC_MPU_ATTR_R |= 0x00000000;
      NVIC_MPU_ATTR_R |= 0x00000022;      // Region Size Mask for flash (Size = log2(256*1024)-1 = 17)
      NVIC_MPU_ATTR_R |= 0x01;            // Region Enabled


     //Enabling mutliple MPU Regions for 32 KiB RAM
     // NVIC_MPU_NUMBER_R &= 0x0;
      NVIC_MPU_NUMBER_R = 0x2;
     //Configuring base register
      NVIC_MPU_BASE_R |= 0x20000000;      //base address of the region.
      NVIC_MPU_BASE_R |= 0x00000000;      //disabling valid region number
      NVIC_MPU_BASE_R |= 0x2;             //Region rule is 2.
     // Configuration of Attribute Register
      NVIC_MPU_ATTR_R |= 0x10000000;      // Instruction fetches are disabled.
      NVIC_MPU_ATTR_R |= 0x01000000;      // RW - Priv / No RW for UnPriv Mode
      NVIC_MPU_ATTR_R |= 0x00000000;      // Setting TEX field for RAM
      NVIC_MPU_ATTR_R |= 0x00040000;      // Shareable
      NVIC_MPU_ATTR_R |= 0x00020000;      // Cacheable
      NVIC_MPU_ATTR_R |= 0x00000000;      // Not Bufferable
      if(taskCurrent < 4)
      {
        NVIC_MPU_ATTR_R &= ~0x0000FF00;
        NVIC_MPU_ATTR_R |= tcb[taskCurrent].permissionmask << 12;
      }
      else if(taskCurrent >= 4)
      {
        NVIC_MPU_ATTR_R &= ~0x0000FF00;
      }
      NVIC_MPU_ATTR_R |= 0x00000000;
      NVIC_MPU_ATTR_R |= (12 << 1);      // Region Size Mask for 8KiB
      NVIC_MPU_ATTR_R |= 0x01;            // Region Enabled


      NVIC_MPU_NUMBER_R = 0x3;
     //Configuring base register
      NVIC_MPU_BASE_R |= 0x20002000;      //base address of the MPU region.
      NVIC_MPU_BASE_R |= 0x00000000;      //disabling valid region number
      NVIC_MPU_BASE_R |= 0x3;             //Region rule is 3.
     // Configuration of Attribute Register
      NVIC_MPU_ATTR_R |= 0x10000000;      // Instruction fetches are disabled.
      NVIC_MPU_ATTR_R |= 0x01000000;      // RW - Priv / No RW for UnPriv Mode
      NVIC_MPU_ATTR_R |= 0x00000000;      // Setting TEX field for RAM
      NVIC_MPU_ATTR_R |= 0x00040000;      // Shareable
      NVIC_MPU_ATTR_R |= 0x00020000;      // Cacheable
      NVIC_MPU_ATTR_R |= 0x00000000;      // Not Bufferable

      if(taskCurrent > 3 && taskCurrent < 12)
      {
        NVIC_MPU_ATTR_R &= ~0x0000FF00;
        //NVIC_MPU_ATTR_R |= 0x00000C00;
        NVIC_MPU_ATTR_R |= tcb[taskCurrent].permissionmask << 4;
      }
      else
      {
          NVIC_MPU_ATTR_R &= ~0x0000FF00;
      }
      NVIC_MPU_ATTR_R |= (12 << 1);      // Region Size Mask for 8KiB
      NVIC_MPU_ATTR_R |= 0x01;            // Region Enabled


      NVIC_MPU_NUMBER_R = 0x4;
      //Configuring base register
      NVIC_MPU_BASE_R |= 0x20004000;      //base address of the MPU region.
      NVIC_MPU_BASE_R |= 0x00000000;      //disabling valid region number
      NVIC_MPU_BASE_R |= 0x4;             //Region rule is 4.
      // Configuration of Attribute Register
      NVIC_MPU_ATTR_R |= 0x10000000;      // Instruction fetches are disabled.
      NVIC_MPU_ATTR_R |= 0x01000000;      // RW - Priv / No RW for UnPriv Mode
      NVIC_MPU_ATTR_R |= 0x00000000;      // Setting TEX field for RAM
      NVIC_MPU_ATTR_R |= 0x00040000;      // Shareable
      NVIC_MPU_ATTR_R |= 0x00020000;      // Cacheable
      NVIC_MPU_ATTR_R |= 0x00000000;      // Not Bufferable
      if(taskCurrent > 11 && taskCurrent < 20)
      {
          NVIC_MPU_ATTR_R &= ~0x0000FF00;
          NVIC_MPU_ATTR_R |= tcb[taskCurrent].permissionmask >> 4;
      }
      else
      {
          NVIC_MPU_ATTR_R &= ~0x0000FF00;
      }

      NVIC_MPU_ATTR_R |= (12 << 1);      // Region Size Mask for 8KiB
      NVIC_MPU_ATTR_R |= 0x01;            // Region Enabled


      NVIC_MPU_NUMBER_R = 0x5;
     //Configuring base register
      NVIC_MPU_BASE_R |= 0x20006000;      //base address of the MPU region.
      NVIC_MPU_BASE_R |= 0x00000000;      //disabling valid region number
      NVIC_MPU_BASE_R |= 0x5;             //Region rule is 5.
     // Configuration of Attribute Register
      NVIC_MPU_ATTR_R |= 0x10000000;      // Instruction fetches are disabled.
      NVIC_MPU_ATTR_R |= 0x01000000;      // RW - Priv / No RW for UnPriv Mode
      NVIC_MPU_ATTR_R |= 0x00000000;      // Setting TEX field for RAM
      NVIC_MPU_ATTR_R |= 0x00040000;      // Shareable
      NVIC_MPU_ATTR_R |= 0x00020000;      // Cacheable
      NVIC_MPU_ATTR_R |= 0x00000000;      // Not Bufferable
      if(taskCurrent > 19 && taskCurrent < 28)
      {
         NVIC_MPU_ATTR_R &= ~0x0000FF00;
         NVIC_MPU_ATTR_R |= tcb[taskCurrent].permissionmask >> 12;
      }
      else
      {
          NVIC_MPU_ATTR_R &= ~0x0000FF00;
      }
      NVIC_MPU_ATTR_R |= (12 << 1);      // Region Size Mask for 8KiB
      NVIC_MPU_ATTR_R |= 0x01;            // Region Enabled


      NVIC_MPU_CTRL_R |= 0x00000004 | 0x00000002 | 0x00000001;        // MPU Default Region | MPU Enabled During Faults | MPU enable

      NVIC_SYS_HND_CTRL_R |= 0x00040000; //Enabling usage fault
      NVIC_SYS_HND_CTRL_R |= 0x00020000; //Enabling bus fault
      NVIC_SYS_HND_CTRL_R |= 0x00010000; //Enabling memory management fault

      usermode();                                                     //turning on the privileged and unprivileged mode.
      fn();
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
//      NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_SVC  // SVC Call Pending
    __asm(" SVC #0");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    __asm(" SVC #1");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(int8_t semaphore)
{
    __asm(" SVC #2");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(int8_t semaphore)
{
    __asm(" SVC #3");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    //systick is called 1000 times in a second.
    //everytime the systick isr is called, go through all the tasks and
    //if the task is delayed, decrement the ticks and when it reaches to zero
    //mark the state of the task as ready
    uint8_t i = 0;
    while(i < MAX_TASKS)
    {
        if(tcb[i].state == STATE_DELAYED)
        {
            tcb[i].ticks--;
            if(tcb[i].ticks == 0)
            {
                tcb[i].state = STATE_READY;
            }
        }
        i++;
    }

    N++;
    uint8_t k = 0;
    //when N reaches to 1000, reset the initial and final time of the task to zero.
    //Save the task time of each task to the buffer based on the value of j. If j is zero, it stores in the first index of buffer
    //if j is 1, it stores in the second index of buffer
    //reset the variable task time to zero for recording the new values of each task, change the current value of j
    //to switch the buffer. Also reset the total time after storing the total time in the new temporary value for calculating the
    //percentage of cpu used by each task later.
    if(N == 1000)
    {
        N = 0;
        initial_time = 0;
        final_time = 0;
        while(k < MAX_TASKS)
        {
          tcb[k].swapBuffer[j] = tcb[k].task_time;
          tcb[k].task_time = 0;
          k++;
        }
        j = 1-j;
        total_time_temp = total_time;
        total_time = 0;
    }

    //preemption mode
    if (preempt_mode == 1)
    {
        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
    }
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    //step 7
    //H/W cannnot save registers R4-R11 and R13-R15
    //saving registers R4-R11 and R13-R15 in the PSP
    __asm("     MRS R0, PSP");
    __asm("     STR R4, [R0, #-4]");
    __asm("     SUB R0, R0, #4");
    __asm("     STR R5, [R0, #-4]");
    __asm("     SUB R0, R0, #4");
    __asm("     STR R6, [R0, #-4]");
    __asm("     SUB R0, R0, #4");
    __asm("     STR R7, [R0, #-4]");
    __asm("     SUB R0, R0, #4");
    __asm("     STR R8, [R0, #-4]");
    __asm("     SUB R0, R0, #4");
    __asm("     STR R9, [R0, #-4]");
    __asm("     SUB R0, R0, #4");
    __asm("     STR R10, [R0, #-4]");
    __asm("     SUB R0, R0, #4");
    __asm("     STR R11, [R0, #-4]");
    __asm("     SUB R0, R0, #4");

    //saving the PSP
    tcb[taskCurrent].spInit = getPSP();

    //get the final time by reading the timer
    final_time = TIMER1_TAV_R;
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                    //turn off timer1
    time_difference = final_time - initial_time;
    total_time += time_difference;                      //addding the task time to the total time
    tcb[taskCurrent].task_time += time_difference;      //everytime the task runs, the task time time is added in the task_time variable

    taskCurrent = rtosScheduler();

    //Enabling mutliple MPU Regions for 32 KiB RAM
    // NVIC_MPU_NUMBER_R &= 0x0;
     NVIC_MPU_NUMBER_R = 0x2;
    //Configuring base register
     NVIC_MPU_BASE_R |= 0x20000000;      //base address of the region.
     NVIC_MPU_BASE_R |= 0x00000000;      //disabling valid region number
     NVIC_MPU_BASE_R |= 0x2;             //Region rule is 2.
    // Configuration of Attribute Register
     NVIC_MPU_ATTR_R |= 0x10000000;      // Instruction fetches are disabled.
     NVIC_MPU_ATTR_R |= 0x01000000;      // RW - Priv / No RW for UnPriv Mode
     NVIC_MPU_ATTR_R |= 0x00000000;      // Setting TEX field for RAM
     NVIC_MPU_ATTR_R |= 0x00040000;      // Shareable
     NVIC_MPU_ATTR_R |= 0x00020000;      // Cacheable
     NVIC_MPU_ATTR_R |= 0x00000000;      // Not Bufferable
     if(taskCurrent < 4)
     {
         NVIC_MPU_ATTR_R &= ~0x0000FF00;
       NVIC_MPU_ATTR_R |= tcb[taskCurrent].permissionmask << 12;
     }
     else if(taskCurrent >= 4)
     {
       NVIC_MPU_ATTR_R &= ~0x0000FF00;
     }
     //NVIC_MPU_ATTR_R |= 0x00000000;
     NVIC_MPU_ATTR_R |= (12 << 1);      // Region Size Mask for 8KiB
     NVIC_MPU_ATTR_R |= 0x01;            // Region Enabled


     NVIC_MPU_NUMBER_R = 0x3;
    //Configuring base register
     NVIC_MPU_BASE_R |= 0x20002000;      //base address of the MPU region.
     NVIC_MPU_BASE_R |= 0x00000000;      //disabling valid region number
     NVIC_MPU_BASE_R |= 0x3;             //Region rule is 3.
    // Configuration of Attribute Register
     NVIC_MPU_ATTR_R |= 0x10000000;      // Instruction fetches are disabled.
     NVIC_MPU_ATTR_R |= 0x01000000;      // RW - Priv / No RW for UnPriv Mode
     NVIC_MPU_ATTR_R |= 0x00000000;      // Setting TEX field for RAM
     NVIC_MPU_ATTR_R |= 0x00040000;      // Shareable
     NVIC_MPU_ATTR_R |= 0x00020000;      // Cacheable
     NVIC_MPU_ATTR_R |= 0x00000000;      // Not Bufferable

     if(taskCurrent > 3 && taskCurrent < 12)
     {
         NVIC_MPU_ATTR_R &= ~0x0000FF00;
       NVIC_MPU_ATTR_R |= tcb[taskCurrent].permissionmask  << 4;
     }
     else
     {
         NVIC_MPU_ATTR_R &= ~0x0000FF00;
     }
     NVIC_MPU_ATTR_R |= (12 << 1);      // Region Size Mask for 8KiB
     NVIC_MPU_ATTR_R |= 0x01;            // Region Enabled


     NVIC_MPU_NUMBER_R = 0x4;
     //Configuring base register
     NVIC_MPU_BASE_R |= 0x20004000;      //base address of the MPU region.
     NVIC_MPU_BASE_R |= 0x00000000;      //disabling valid region number
     NVIC_MPU_BASE_R |= 0x4;             //Region rule is 4.
     // Configuration of Attribute Register
     NVIC_MPU_ATTR_R |= 0x10000000;      // Instruction fetches are disabled.
     NVIC_MPU_ATTR_R |= 0x01000000;      // RW - Priv / No RW for UnPriv Mode
     NVIC_MPU_ATTR_R |= 0x00000000;      // Setting TEX field for RAM
     NVIC_MPU_ATTR_R |= 0x00040000;      // Shareable
     NVIC_MPU_ATTR_R |= 0x00020000;      // Cacheable
     NVIC_MPU_ATTR_R |= 0x00000000;      // Not Bufferable
     if(taskCurrent > 11 && taskCurrent < 20)
     {
         NVIC_MPU_ATTR_R &= ~0x0000FF00;
         NVIC_MPU_ATTR_R |= tcb[taskCurrent].permissionmask << 8;
     }
     else
     {
         NVIC_MPU_ATTR_R &= ~0x0000FF00;
     }

     NVIC_MPU_ATTR_R |= (12 << 1);      // Region Size Mask for 8KiB
     NVIC_MPU_ATTR_R |= 0x01;            // Region Enabled


     NVIC_MPU_NUMBER_R = 0x5;
    //Configuring base register
     NVIC_MPU_BASE_R |= 0x20006000;      //base address of the MPU region.
     NVIC_MPU_BASE_R |= 0x00000000;      //disabling valid region number
     NVIC_MPU_BASE_R |= 0x5;             //Region rule is 5.
    // Configuration of Attribute Register
     NVIC_MPU_ATTR_R |= 0x10000000;      // Instruction fetches are disabled.
     NVIC_MPU_ATTR_R |= 0x01000000;      // RW - Priv / No RW for UnPriv Mode
     NVIC_MPU_ATTR_R |= 0x00000000;      // Setting TEX field for RAM
     NVIC_MPU_ATTR_R |= 0x00040000;      // Shareable
     NVIC_MPU_ATTR_R |= 0x00020000;      // Cacheable
     NVIC_MPU_ATTR_R |= 0x00000000;      // Not Bufferable
     if(taskCurrent > 19 && taskCurrent < 28)
     {
         NVIC_MPU_ATTR_R &= ~0x0000FF00;
        NVIC_MPU_ATTR_R |= tcb[taskCurrent].permissionmask << 8;
     }
     else
     {
         NVIC_MPU_ATTR_R &= ~0x0000FF00;
     }
     NVIC_MPU_ATTR_R |= (12 << 1);      // Region Size Mask for 8KiB
     NVIC_MPU_ATTR_R |= 0x01;            // Region Enabled

    //resetting the counter value to 0
    TIMER1_TAV_R=0;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;     //Enable the timer
    initial_time =TIMER1_TAV_R;

    if(tcb[taskCurrent].state == STATE_READY){

        setPSP(tcb[taskCurrent].spInit);                //get back the saved PSP

        //get back all the saved registers that were pushed into the PSP
        __asm("     MRS R0, PSP");
        __asm("     LDR R4, [R0, #-4], #-4");
        __asm("     LDR R5, [R0, #-4], #-4");
        __asm("     LDR R6, [R0, #-4], #-4");
        __asm("     LDR R7, [R0, #-4], #-4");
        __asm("     LDR R8, [R0, #-4], #-4");
        __asm("     LDR R9, [R0, #-4], #-4");
        __asm("     LDR R10, [R0, #-4], #-4");
        __asm("     LDR R11, [R0, #-4], #-4");
    }
    else {
        //If the task was not ready, make the current task state to ready.
        tcb[taskCurrent].state = STATE_READY;

        setPSP((tcb[taskCurrent].spInit)-32);   //There are 8 registers that gets pushed into the PSP so need to decrement the PSP by 32.

        _fn fn = tcb[taskCurrent].pid;          //get the pointer to the function of the task

        uint32_t *PSP = (uint32_t*) getPSP();

        *PSP  = 0;
        *(PSP + 1) = 1;
        *(PSP + 2) = 2;
        *(PSP + 3) = 3;
        *(PSP + 4) = 12;
        *(PSP + 5) = 0xFFFFFFFD;
        *(PSP + 6) = fn;                          //set the program counter register to the function pointer.
        *(PSP + 7) = 0x61000000;

    }
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    uint32_t R0, R1, R2, R3, R12, LR, PC, xPSR;

    uint32_t *PSP = (uint32_t*) getPSP();

    R0 = *PSP;
    R1 = *(PSP+1);
    R2 = *(PSP+2);
    R3 = *(PSP+3);
    R12 = *(PSP+4);
    LR = *(PSP+5);
    PC = *(PSP+6);
    xPSR = *(PSP+7);

    uint32_t n = getSVCNumber();

    uint8_t numTasks = 0;           //common variable that is used to run the loop

    semaphore *temp_semaphore;      //pointer to the semaphore

    shell_data *temp_shell;         //pointer to the shell_data struct

    char *pid_name = (char*)R0;     //If the name was passed as an argument in the function, extract the name by making a pointer to the char

    temp_shell = R0;                //for the shell_data, an address of the struct is passsed
                                    //this will point to the address of the struct is the struct was passed.

    uint32_t process_time = 0;      //record the task time

    switch(n)
    {
        case 0:  //yield
        {
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV ; //turning on the pendSV exception
            break;
        }
        case 1: //sleep
        {
            tcb[taskCurrent].ticks = R0;
            tcb[taskCurrent].state = STATE_DELAYED;
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV ;
            NVIC_ST_RELOAD_R = 39999;
            NVIC_ST_CTRL_R |= 0x7; //Enabling clk_src(system clock), inten and enable bit
            break;
        }

        case 2: //wait
        {
            if(semaphores[R0].count > 0)
            {
                semaphores[R0].count--;
                //code and discard
//                if(R0==3)
//                {
//                    semaphores[R0].lastWaitThread = taskCurrent;
//                }
                //code and discard
                //semaphores[R0].lastWaitThread = taskCurrent;   //last task that used semaphore
            }
            else{
                tcb[taskCurrent].state = STATE_BLOCKED;
                tcb[taskCurrent].semaphore = &semaphores[R0];
                semaphores[R0].processQueue[semaphores[R0].queueSize] = taskCurrent;
                semaphores[R0].queueSize++;
                //code and discard
//            if(pi_mode == 0)
//            {
//                if(semaphores[R0].lastWaitThread == 6 && taskCurrent == 6)
//                {
//                    tcb[1].currentPriority = 0;
//                }
//            }
//            if(pi_mode == 0 && R0 == 3)
//            {
//                if(tcb[semaphores[R0].lastWaitThread].priority < tcb[taskCurrent].priority)
//                {
//                    tcb[semaphores[R0].lastWaitThread].currentPriority = tcb[taskCurrent].priority;
//                }
//            }
                //Priority inheritance mode
                if(pi_mode == 1)
                {
                    while(numTasks < taskCount)
                    {
                        //If the task that is in the queue of the semaphore has the higher priority(i.e lower value number) than the task that is
                        //holding the semaphore and running, then change the priority of the task that is holding the semaphore to the
                        //higher priority which will be the same as the priority of the task that is being held in the process queue.
                        if(tcb[numTasks].semaphore == &semaphores[R0])           //checking to see if any task has been holdin the resource
                        {
                            //if the task that is holding the semaphore has the lower priority, change it back to the
                            // priority of the task that is the process queue.
                            if(tcb[semaphores[R0].processQueue[0]].priority< tcb[numTasks].priority)
                            {
                                tcb[numTasks].currentPriority = tcb[semaphores[R0].processQueue[0]].priority;
                                break;
                            }
                        }
                        numTasks++;
                    }
                }

                //allow tasks switches again
                NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            }
            break;
        }


        case 3: //post
        {
            semaphores[R0].count++;
            //priority inheritance
            //when posting back, if the current priority is not same as the priority
            //in the tcb block change the current priority back to the priority.
            if(tcb[taskCurrent].currentPriority != tcb[taskCurrent].priority)
            {
                tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority;
            }

            if(semaphores[R0].queueSize > 0)
            {
                tcb[semaphores[R0].processQueue[0]].state = STATE_READY;
                semaphores[R0].count--;
                //code and discard
//            if(taskCurrent == 1 && pi_mode == 0)
//            {
//               tcb[1].currentPriority = tcb[1].priority;
//            }
                uint8_t j = 0;
                for(j = 0; j<semaphores[R0].queueSize; j++)
                {
                    semaphores[R0].processQueue[j] = semaphores[R0].processQueue[j+1];
                }
                semaphores[R0].queueSize--;

            }

            break;
        }

        case 5: //ps
        {
            //pass the total task count to the shell data
            temp_shell->tasks_total = taskCount;

            while(numTasks < taskCount)
            {
                //copy the name of the task to shell data
                copy(tcb[numTasks].name, temp_shell->tasks_name[numTasks]);

                temp_shell->tasks_name[numTasks][getLength(temp_shell->tasks_name[numTasks])] = '\0';

                //pass the pid
                temp_shell->pid[numTasks] = tcb[numTasks].pid;

                //get the current past task time from the swap buffer
                uint32_t t = tcb[numTasks].swapBuffer[1-j];

                //process_time = (t * 10000)/(40e6)
                //total N is 40*10^6
                process_time = (t/4000);

                //pass the cpu percentage
                temp_shell->cputime[numTasks] = process_time;

                //pass the current priority
                IntegerToString(tcb[numTasks].currentPriority,temp_shell->data);
                copy(temp_shell->data, temp_shell->priority[numTasks]);

                //pass the state of the task
                uint8_t tempState = tcb[numTasks].state;
                temp_shell->state[numTasks] = tempState;

                //if the task is delayed, save its ticks
                if(tempState == 3)
                {

                   IntegerToString(tcb[numTasks].ticks,temp_shell->data);
                   copy(temp_shell->data,temp_shell->delay[numTasks]);

                }

                //if the task is blocked, record the semaphore
                if(tempState == 4)
                {

                    temp_semaphore = tcb[numTasks].semaphore;

                    copy(temp_semaphore->name,temp_shell->resources[numTasks]);
                    temp_shell->resources[numTasks][10] = '\0';


                }

                numTasks++;

            }

            //calculate the kernel time
            uint32_t kernel_time = (40000000 - total_time_temp);
            kernel_time = kernel_time/4000;

            //pass the kernel time
            temp_shell->kerneltime = kernel_time;

            break;

        }

        case 6: //ipcs
        {
            //record the semaphore count
            temp_shell->semaphores_total = semaphoreCount;
            numTasks = numTasks + 1;
               while(numTasks < semaphoreCount)
               {
                   //record the semaphores name
                   copy(semaphores[numTasks].name,temp_shell->semaphores_names[numTasks]);

                   //record the count of the semaphore
                   IntegerToString(semaphores[numTasks].count,temp_shell->data);
                   copy(temp_shell->data,temp_shell->semaphores_count[numTasks]);

                   //record the task that is waiting in the queue.
                   int8_t queue_size = semaphores[numTasks].queueSize;
                   uint8_t queue_var = 0;
                   for(queue_var = 0; queue_var < queue_size ; queue_var++)
                   {
                       copy(tcb[semaphores[numTasks].processQueue[queue_var]].name,temp_shell->waiting_processes[numTasks]);
                   }
                   numTasks++;
               }
               break;
        }

        case 7:  //kill pidnumber
        {
            //idle cannot be killed so start the numTasks from 1
               numTasks = 1;
               if((uint32_t)tcb[0].pid == R0)
               {
                  putsUart0("Cannot kill the Idle process.\r\n");
               }
               while(numTasks < taskCount)
               {
                  if((uint32_t)tcb[numTasks].pid == R0)
                  {
                      //get the pointer to the semaphore of the task if any
                      temp_semaphore = tcb[numTasks].semaphore;

                      //get the queue size
                      int8_t queue_size = temp_semaphore->queueSize;
                      uint8_t queue_var = 0;

                      //set the queue size in the semaphore to the 0 if the task that is being killed
                      //is blocked.
                      if(tcb[numTasks].state == STATE_BLOCKED)
                      {
                          temp_semaphore->queueSize = 0;
                          //code and discard
       //                   for(queue_var = 0; queue_var < queue_size ; queue_var++)
       //                   {
       //                      if(temp_semaphore->processQueue[queue_var] == numTasks)
       //                      {
       //                          temp_semaphore->processQueue[queue_var] = 0;
       //                          temp_semaphore->queueSize--;
       //                      }
       //                      temp_semaphore->processQueue[queue_var] = temp_semaphore->processQueue[queue_var+1];
       //                   }

                      }

                      //if the task( that is being killed) is either ready or delayed, post the semaphore.
                      else if(tcb[numTasks].state == STATE_READY || tcb[numTasks].state == STATE_DELAYED)
                      {
                          //if the task has any semaphores
                          if(tcb[numTasks].semaphore != 0)
                          {
                             tcb[numTasks].currentPriority = tcb[numTasks].priority;
                             temp_semaphore->count++;
                             if(queue_size>0)
                             {
                                 //make the state of the task that was in the queue to the ready
                                 tcb[temp_semaphore->processQueue[0]].state = STATE_READY;
                                 temp_semaphore->count--;
                                 tcb[temp_semaphore->processQueue[0]].semaphore = temp_semaphore;

                                 //move the task in the semaphore queue.
                                 for(queue_var = 0; queue_var < queue_size ; queue_var++)
                                 {
                                    temp_semaphore->processQueue[queue_var] = temp_semaphore->processQueue[queue_var+1];
                                 }
                                 temp_semaphore->queueSize--;
                             }
                          }

                      }
                      //mark the state of the task to the suspended
                       tcb[numTasks].state = STATE_SUSPENDED;
                  }
                   numTasks++;
               }

               break;
        }

        case 8: //pi on/off
        {
            pi_mode = R0;
            break;
        }

        case 9: //preempt on/off
        {
            preempt_mode = R0;
            break;
        }

        case 10: //sched rr/pio
        {
            sched_mode = R0;
            break;

        }

        case 11: //pidof processname
        {
            while(numTasks < taskCount)
            {
                if(compare(tcb[numTasks].name,pid_name))
                {
                    uint16_t val = tcb[numTasks].pid;
                    uint32_tToHex1(val);
                    putsUart0("\r\n");
                }
                numTasks++;
            }

            break;
        }

        case 12: //run processname
        {
            while(numTasks < taskCount)
            {
                if(compare(tcb[numTasks].name,pid_name))
                {
                    if(tcb[numTasks].state != STATE_READY)
                    {
                        //checking to see of the task has any semaphore
                        if(tcb[numTasks].semaphore !=0 )
                        {
                            //get the semaphore
                            temp_semaphore = tcb[numTasks].semaphore;
                            //if the queue size is greater than zero, put the task to the end of the queue.
                            if(temp_semaphore->queueSize > 0)
                            {
                                temp_semaphore->processQueue[(temp_semaphore->queueSize)-1] = numTasks;
                            }
                            //add the task to the queue.
                            else
                            {
                                temp_semaphore->processQueue[0]=numTasks;
                            }

                            temp_semaphore->queueSize++;
                            //mark the state of the task as blocked
                            tcb[numTasks].state = STATE_BLOCKED;
                        }
                        else
                            tcb[numTasks].state = STATE_READY;
                    }
                }
                numTasks++;
            }

            break;
        }

        case 13: //reboot
        {
            NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ|NVIC_APINT_VECT_RESET;
            break;
        }


        case 14: //mpu on/off
        {
            mpu_mode = R0;
            break;
        }

        case 15: //restarthread
        {
            while(numTasks < taskCount)
            {
                if(tcb[numTasks].pid == R0)
                {
                    //checking to see of the task has any semaphore
                    if(tcb[numTasks].semaphore !=0 )
                    {
                        //get the semaphore
                        temp_semaphore = tcb[numTasks].semaphore;
                        //if the queue size is greater than zero, put the task to the end of the queue.
                        if(temp_semaphore->queueSize > 0)
                        {
                            temp_semaphore->processQueue[(temp_semaphore->queueSize)-1] = numTasks;
                        }
                        //add the task to the queue.
                        else
                        {
                            temp_semaphore->processQueue[0]=numTasks;
                        }

                        temp_semaphore->queueSize++;
                        //mark the state of the task as blocked
                        tcb[numTasks].state = STATE_BLOCKED;
                    }
                    else
                        tcb[numTasks].state = STATE_READY;
                    break;
                }
                numTasks++;
            }
            break;
        }

        case 16: //destroy thread
        {
            //same code as kill
            //see the description of the kill pidnumber (SVC number is 7)
            while(numTasks < taskCount)
            {
                if(tcb[numTasks].pid == R0)
                {
                    temp_semaphore = tcb[numTasks].semaphore;
                    int8_t queue_size = temp_semaphore->queueSize;
                    uint8_t queue_var = 0;
                    if(tcb[numTasks].state == STATE_BLOCKED)
                    {
                        temp_semaphore->queueSize = 0;
 //                   for(queue_var = 0; queue_var < queue_size ; queue_var++)
 //                   {
 //                      if(temp_semaphore->processQueue[queue_var] == numTasks)
 //                      {
 //                          temp_semaphore->processQueue[queue_var] = 0;
 //                          temp_semaphore->queueSize--;
 //                      }
 //                      temp_semaphore->processQueue[queue_var] = temp_semaphore->processQueue[queue_var+1];
 //                   }

                    }
                    else if(tcb[numTasks].state == STATE_READY || tcb[numTasks].state == STATE_DELAYED)
                    {
                        if(tcb[numTasks].semaphore != 0)
                        {
                            tcb[numTasks].semaphore = 0;
                            tcb[numTasks].currentPriority = tcb[numTasks].priority;
                            temp_semaphore->count++;
                            if(queue_size>0)
                            {
                                tcb[temp_semaphore->processQueue[0]].state = STATE_READY;
                                temp_semaphore->count--;
                                tcb[temp_semaphore->processQueue[0]].semaphore = temp_semaphore;
                                for(queue_var = 0; queue_var < queue_size ; queue_var++)
                                {
                                    temp_semaphore->processQueue[queue_var] = temp_semaphore->processQueue[queue_var+1];
                                }
                                temp_semaphore->queueSize--;
                            }
                        }

                    }
                    tcb[numTasks].state = STATE_INVALID;
                    break;
                }
                numTasks++;
            }
            break;
        }

        case 17: //setthreadPriority
        {
            while(numTasks < taskCount)
            {
                if(tcb[numTasks].pid == R0)
                {
                    tcb[numTasks].currentPriority=R1;
                    tcb[numTasks].priority=R1;
                    break;
                }
                numTasks++;
            }
            break;
        }
    }
}

// REQUIRED: code this function
void mpuFaultIsr(){
    putsUart0("MPU fault in process ");
    putsUart0(tcb[taskCurrent].name);
    putsUart0("\r\n");

    uint32_t R0, R1, R2, R3, R12, LR, PC, xPSR;

    uint32_t *PSP = (uint32_t*) getPSP();
    putsUart0("PSP = ");
    uint32_tToHex(PSP);
    putsUart0("\r\n");

    uint32_t *MSP = (uint32_t*) getMSP();
    putsUart0("MSP = ");
    uint32_tToHex(MSP);
    putsUart0("\r\n");

    putsUart0("MFAULTFLAG = ");
    uint32_tToHex(NVIC_FAULT_STAT_R & 0xFF);
    putsUart0("\r\n");

    putsUart0("Offending instruction at ");
    PC = *(PSP+6);
    uint32_tToHex(PC);
    putsUart0(" tried to access ");
    uint32_tToHex(NVIC_MM_ADDR_R);
    putsUart0("\r\n");

    putsUart0("Process stack dump:\r\n");
    R0=*PSP;
    putsUart0("R0 = ");
    uint32_tToHex(R0);
    putsUart0("\r\n");

    R1 = *(PSP+1);
    putsUart0("R1 = ");
    uint32_tToHex(R1);
    putsUart0("\r\n");

    R2 = *(PSP+2);
    putsUart0("R2 = ");
    uint32_tToHex(R2);
    putsUart0("\r\n");

    R3 = *(PSP+3);
    putsUart0("R3 = ");
    uint32_tToHex(R3);
    putsUart0("\r\n");

    R12 = *(PSP+4);
    putsUart0("R12 = ");
    uint32_tToHex(R12);
    putsUart0("\r\n");

    LR = *(PSP+5);
    putsUart0("LR = ");
    uint32_tToHex(LR);
    putsUart0("\r\n");

    PC = *(PSP+6);
    putsUart0("PC = ");
    uint32_tToHex(PC);
    putsUart0("\r\n");

    xPSR = *(PSP+7);
    putsUart0("xPSR = ");
    uint32_tToHex(xPSR);
    putsUart0("\r\n");

    NVIC_SYS_HND_CTRL_R &= ~NVIC_SYS_HND_CTRL_MEMP; //Clear the pending bit

    tcb[taskCurrent].state = STATE_INVALID;

    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV ; //turning on the pendSV exception

}


// REQUIRED: code this function
void hardFaultIsr(){
    putsUart0("Hard fault in process ");
    putsUart0(tcb[taskCurrent].name);
    putsUart0("\r\n");

    uint32_t *PSP = (uint32_t*) getPSP();
    putsUart0("PSP = ");
    uint32_tToHex(PSP);
    putsUart0("\r\n");

    uint32_t *MSP = (uint32_t*) getMSP();
    putsUart0("MSP = ");
    uint32_tToHex(MSP);
    putsUart0("\r\n");

    uint32_t hflag = NVIC_HFAULT_STAT_R;
    putsUart0("HFLAG = ");
    uint32_tToHex(hflag);
    putsUart0("\r\n");
}
// REQUIRED: code this function
void busFaultIsr()
{
    putsUart0("Bus fault in process ");
    putsUart0(tcb[taskCurrent].name);
    putsUart0("\r\n");
}

// REQUIRED: code this function
void usageFaultIsr()
{
    putsUart0("Usage fault in process ");
    putsUart0(tcb[taskCurrent].name);
    putsUart0("\r\n");
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
extern void usermode();
extern void privilegedmode();
extern void setPSP(uint32_t sp);
extern void setASP();
extern uint32_t getPSP();
extern uint32_t getMSP();

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons, and uart
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    //Timer code
    //Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);

    GPIO_PORTF_DIR_R |= BLUE_LED_MASK;
    GPIO_PORTE_DIR_R |= RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK | GREEN_LED_MASK ;
    GPIO_PORTA_DIR_R &= ~PUSH_BUTTON_MASK0 | ~PUSH_BUTTON_MASK1 | ~PUSH_BUTTON_MASK2 | ~PUSH_BUTTON_MASK3 | ~PUSH_BUTTON_MASK4 | ~PUSH_BUTTON_MASK5;

    GPIO_PORTF_DR2R_R |= BLUE_LED_MASK;
    GPIO_PORTE_DR2R_R |= RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK | GREEN_LED_MASK;

    GPIO_PORTF_DEN_R |= BLUE_LED_MASK;
    GPIO_PORTE_DEN_R |= RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK | GREEN_LED_MASK;
    GPIO_PORTA_DEN_R |= PUSH_BUTTON_MASK0 | PUSH_BUTTON_MASK1 | PUSH_BUTTON_MASK2 | PUSH_BUTTON_MASK3 | PUSH_BUTTON_MASK4 | PUSH_BUTTON_MASK5 ;

    GPIO_PORTA_PUR_R |= PUSH_BUTTON_MASK0 | PUSH_BUTTON_MASK1 | PUSH_BUTTON_MASK2 | PUSH_BUTTON_MASK3 | PUSH_BUTTON_MASK4 | PUSH_BUTTON_MASK5;

    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                                // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;                          // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;     // configure for periodic mode (count down)
    TIMER1_TAV_R = 0x00;
//    TIMER1_CTL_R |= TIMER_CTL_TAEN;                                 // turn-on timer

    initUart0();
    setUart0BaudRate(115200, 40e6);
}

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{
    uint8_t sum = 0;
    if(!PUSH_BUTTON0) sum+=1;
    if(!PUSH_BUTTON1) sum+=2;
    if(!PUSH_BUTTON2) sum+=4;
    if(!PUSH_BUTTON3) sum+=8;
    if(!PUSH_BUTTON4) sum+=16;
    if(!PUSH_BUTTON5) sum+=32;

    return sum;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

//this function copies the char array from the first array to the second string
void copy(char string1[], char string2[])
{
    uint8_t i=0;
    //run the loop until the index in the array reached to the null character
    for (i = 0; string1[i] != '\0'; ++i)
    {
       string2[i] = string1[i];
    }
    string2[i] = '\0';
}

//this function gets the length of the string i.e char array
uint16_t getLength(char string[])
{
    uint8_t i=0;
    while(string[i] != '\0')
    {
        i++;
    }
    return i;
}

//this function gets the number that was passed in the assembly code to run the svcIsr
uint8_t getSVCNumber()
{
    uint32_t *PSP = (uint32_t*) getPSP();
    uint32_t *PC;

    //first get the program counter and decrement the program counter by 2 to reach the assembly line of instruction to grab the number.
    PC = *(PSP+6) - 2;
    return *PC;
}

//this function converts uint32_t value into hexadecimal value and prints the result.
//this function does not print the zeros of the hexadecimal
void uint32_tToHex1(uint32_t decimal)
{
    uint32_t quotient, remainder;
    int i,j = 0;
    char hex[20];

    quotient = decimal;

    while (quotient != 0)
    {
       remainder = quotient % 16;

       if (remainder < 10)
       {
           hex[j++] = 48 + remainder;
       }

       else
       {
           hex[j++] = 55 + remainder;
       }

       quotient = quotient / 16;
     }

    for (i = j-1; i >= 0; i--)
           putcUart0(hex[i]);
}

//this function converts uint32_t value into hexadecimal value and prints the result.
//this function prints the zeros of the hexadecimal
void uint32_tToHex(uint32_t decimal)
{
    uint32_t quotient, remainder;
    int i,j = 0;
    char hex[9];

    quotient = decimal;

    while (quotient != 0)
    {
       remainder = quotient % 16;
       if (remainder < 10)
       {
           hex[j++] = 48 + remainder;
       }

       else
       {
           hex[j++] = 55 + remainder;
       }

          quotient = quotient / 16;
     }

    //if the quotient reaches to zero because there are less then 4 hexadecimal numbers,
    //fill the char array with the zeros.
    if(quotient == 0 && j != 8)
    {
        while(j!=8)
        {
            hex[j++] = 48;
        }
    }

    for (i = j; i >= 0; i--)
           putcUart0(hex[i]);
}

//function that converts the integer to string
void IntegerToString(uint32_t number, char string[])
{
    uint32_t i, rem, length = 0, n;

    n = number;
    //Initially, if n is zero, put the null character in the end of the char array.
    if(n == 0)
    {
        string[length] = '0';
        string[length+1] = '\0';
    }

    else
    {
        //first get the length of the number
        while (n != 0)
        {
            length++;
            n /= 10;
        }

        for (i = 1; i < (length + 1) ; i++)
        {
            rem = number % 10;
            number = number / 10;
            //ascii value of '0' is 48
            //adding 48 to remainder gives the ascii char
            //index starts from the last index
            string[length - i] = rem + '0';
        }
        //put the null char in the end of the char array
        string[length] = '\0';
    }
}

//function that converts the hexadecimal number to the integer
uint32_t hex2int(char hex[]) {

    uint32_t total = 0;

    int i =0;

    int len = 0;

    int len1 = 0;

    //get the length of the hexadecimal
    while (hex[i] != 0)
    {
        len++;
        i++;
    }

    i = 0;

    while (hex[i] != 0) {

        //get the first number in the hexadecimal
        uint32_t var = hex[i];

        //comparison with the ascii table
        if (var >= '0' && var <= '9') var = (var - '0');

        else if (var >= 'a' && var <='f') var = var - 'a' + 10;

        else if (var >= 'A' && var <='F') var = var - 'A' + 10;

        //multiply with the number 16 with len times
        for(len1 = 0; len1<(len-1); len1++)
        {
            var = var * 16;
        }

        //decrement the length
        len--;

        //add the var to the total number
        total += var;

        i++;
    }

    return total;
}

//This function receives characters from the user interface
void getsUart0(USER_DATA* data)
{
  uint8_t count = 0;
  while(true){

    char c;
    c = getcUart0();

    if(c==127||c==8)  //checking if the character entered is DEL or backspace
    {
       if(count>0)
       {
         count--;
         continue;
       }
       else{
       continue;
       }
    }

    else if(c==10 || c==13) //checking if the character entered is Line feed or carriage return
    {
       data->buffer[count]='\0';
       return;
    }

    else if(c >= 32)
    {
       if(count==MAX_CHARS) //if the count of characters in the buffer is equal to the maximum, then put the null
                             //character in the end of the buffer. Else, increment the count.
       {
         data->buffer[count]='\0';
         return;
       }

       else{
       data->buffer[count++]=c;
       continue;
       }

     }

  }
}

/* this function checks whether the character is alphabet or not
 * 65-90 = A - Z & 97 - 122 = a - z */
bool alpha(char c)
{
  if((c>=65 && c<=90) || (c>=97 && c<=122))
  {
      return true;
  }
  else{
      return false;
  }
}

/* this function checks whether the character is numeric or not
 * 48 - 57 = 0 - 9, 45 is hyphen & 46 is dot*/
bool numeric(char c)
{
    if((c<=9) || (c>=48 && c<=57) || (c>=45 && c<=46))
    {
       return true;
    }
    else{
       return false;
    }
}

/* This is a function that takes the buffer string from the getsUart0() function and
processes the string in-place and returns information about the parsed fields in
fieldCount, fieldPosition, and fieldType. */
void parseFields(USER_DATA* data)
{
    uint8_t Count = 0; //number of field counts
    uint8_t i = 0;     //keeps track of offset of the field within the buffer

    while( (data->buffer[i] != 0))
    {
        if(data->fieldCount == MAX_FIELDS)
        {

            return;
        }

        if(i == 0 && ( (alpha(data->buffer[i])) || (numeric(data->buffer[i])) ) )
        {
          if( (alpha(data->buffer[i])) )
          {
             data->fieldPosition[Count] = i;
             data->fieldType[Count] = 97;     //record the type of field. 97 - a and 110 - n
             data->fieldCount = Count+1;
             Count++;
          }
          else if( numeric(data->buffer[i]) )
          {

              data->fieldPosition[Count] = i;
              data->fieldType[Count] = 110;
              data->fieldCount = Count+1;
              Count++;

          }

        }

        if( !(alpha(data->buffer[i])) && !(numeric(data->buffer[i])) )
        {
            data->buffer[i] = 0;

            if(alpha(data->buffer[i+1]))
            {

                data->fieldPosition[Count] = i+1;
                data->fieldType[Count] = 97;
                data->fieldCount = Count+1;
                Count++;


            }

            else if(numeric(data->buffer[i+1]))
            {

                data->fieldPosition[Count] = i+1;
                data->fieldType[Count] = 110;
                data->fieldCount = Count+1;
                Count++;

            }
         }

        i++;
    }
}

/* Returns the value of a field requested if the field number is in range or NULL otherwise */
char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if(fieldNumber <= data->fieldCount)
    {
       if(data->fieldType[fieldNumber] == 97)
       {
          return &data->buffer[data->fieldPosition[fieldNumber]];
       }
    }

      return '\0';
}

/* Returns a pointer to the field requested if the field number is in range and the field type
is numeric or 0 otherwise */
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    char num[5];
    if(fieldNumber <= data->fieldCount)
    {
        if(data->fieldType[fieldNumber] == 110)
        {
            //this part of code will run until the null char is reached.
            uint8_t count = data->fieldPosition[fieldNumber];
            uint8_t pos=0;
            while(data->buffer[count] != 0)
            {
                num[pos] = data->buffer[count];
                pos++;
                count++;
            }
            num[pos] = '\0';
            int32_t value = hex2int(num);
            return value;
        }
    }

    return 0;
}

//this function compares two strings.
bool compare(char *str1, char *str2)
{
    while (*str1 == *str2 || *str1+32 == *str2)
    {
        if (*str1 == '\0' && *str2 == '\0')
            return true;
        str1++;
        str2++;
    }

    return false;
}

/* This function returns true if the command matches the first field and the number of
arguments (excluding the command field) is greater than or equal to the requested
number of minimum arguments */
bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    char *name = &data->buffer[data->fieldPosition[0]];

    if(compare(name, strCommand))
    {
        if((data->fieldCount)-1 >= minArguments)
        {
            return true;
        }

 }

  return false;
}

void ps(shell_data* shelldata)
{
    __asm("  SVC #5");
}

void ipcs(shell_data* shelldata)
{
    __asm("   SVC #6");
}

void kill(uint32_t val){
    __asm("  SVC #7");
}

void pi(int8_t val){
    __asm("  SVC #8");
}

void preempt(int8_t val)
{
    __asm("  SVC #9");
}

void sched(int8_t val)
{
    __asm("  SVC #10");
}

void pidof(char name[])
{
    __asm("  SVC #11");
}

void proc_name(char name[])
{
    __asm("  SVC #12");
}

void reboot()
{
    __asm("  SVC #13");
}

void mpu(int8_t val)
{
    __asm("  SVC #14");
}
// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void errant()
{
    uint32_t* p = (uint32_t*)0x20000000;
    while(true)
    {
        while (readPbs() == 32)
        {
            *p = 0;
        }

        yield();
    }

}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

// REQUIRED: add processing for the shell commands through the UART here
void shell()
{

    USER_DATA data;

    shell_data shelldata;


    while (true)
    {
        getsUart0(&data);

        putsUart0(data.buffer);

        putsUart0("\r\n");

        parseFields(&data);

        bool valid = false;

        if(isCommand(&data, "reboot", 0))
        {
          reboot();
          valid=true;
        }

        if(isCommand(&data, "ps", 0))
        {
            ps(&shelldata);
            putsUart0("Name\t\t PID\t\t%CPU\t\tPriority\t\tState\r\n");
            uint8_t p = 0;
            while(p < shelldata.tasks_total)
            {
                putsUart0(shelldata.tasks_name[p]);

                shelldata.tempState = getLength(shelldata.tasks_name[p]);
                //put the tab based on the length of the string
                if( shelldata.tempState <= 7)
                {
                    putsUart0("\t\t");
                }
                else
                {
                    putsUart0("\t");
                }
                //clear the array after printing the char array to the screen
                while(shelldata.tempState >= 1)
                {
                   shelldata.tasks_name[p][shelldata.tempState-1] = 0;
                   shelldata.tempState--;
                }

                //printing the pid number
                uint32_tToHex1(shelldata.pid[p]);
                putsUart0("\t\t");

                shelldata.tempState = shelldata.cputime[p] / 100;
                IntegerToString(shelldata.tempState, shelldata.data);
                putcUart0(shelldata.data[0]);
                putcUart0(shelldata.data[1]);
                putcUart0(46);
                shelldata.tempState = shelldata.cputime[p] % 100;
                IntegerToString(shelldata.tempState, shelldata.data);
                putcUart0(shelldata.data[0]);
                putcUart0(shelldata.data[1]);

                putsUart0("\t\t");
                shelldata.semaphores_total = 0;

                putsUart0(shelldata.priority[p]);
                putsUart0("\t\t");

                shelldata.tempState = shelldata.state[p];
                if(shelldata.tempState == 0)
                {
                    putsUart0("INVALID");
                }
                if(shelldata.tempState == 1)
                {
                    putsUart0("UNRUN");
                }
                if(shelldata.tempState == 2)
                {
                    putsUart0("READY");
                }
                if(shelldata.tempState == 3)
                {
                    putsUart0("DELAYED FOR ");
                    putsUart0(shelldata.delay[p]);
                    putsUart0(" ms");
                }
                if(shelldata.tempState == 4)
                {
                    putsUart0("BLOCKED BY ");
                    copy(shelldata.resources[p],shelldata.data);
                    putsUart0(shelldata.data);

                }
                if(shelldata.tempState == 5)
                {
                    putsUart0("SUSPENDED");

                }
                putsUart0("\t");

               p++;
               putsUart0("\r\n");

            }
            shelldata.semaphores_total = 0;
            putsUart0("Kernel Time:        ");
            shelldata.tempState = shelldata.kerneltime / 100;
            IntegerToString(shelldata.tempState, shelldata.data);
            putcUart0(shelldata.data[0]);
            putcUart0(shelldata.data[1]);
            putcUart0(46);
            shelldata.tempState = shelldata.kerneltime % 100;
            IntegerToString(shelldata.tempState, shelldata.data);
            putcUart0(shelldata.data[0]);
            putcUart0(shelldata.data[1]);


            putsUart0("\t\t");
            shelldata.semaphores_total = 0;

            valid = true;
        }

        if(isCommand(&data, "ipcs", 0))
        {
            ipcs(&shelldata);
            putsUart0("Name\t\tCount\tWaiting Threads\r\n");
            uint8_t i = 0;
            while(i < shelldata.semaphores_total)
            {
                putsUart0(shelldata.semaphores_names[i]);
                putsUart0("\t");

                putsUart0(shelldata.semaphores_count[i]);
                putsUart0("\t");

                putsUart0(shelldata.waiting_processes[i]);

                i++;
                putsUart0("\r\n");
            }
            shelldata.semaphores_total = 0;
            valid = true;
        }

        if(isCommand(&data, "kill", 1))
        {
            kill(getFieldInteger(&data,1));

            valid = true;
        }

        if(isCommand(&data, "pi", 1))
        {
            if(compare(getFieldString(&data,1),"on"))
            {
                pi(1);
                putsUart0("priority inheritance is on.\r\n");
            }
            else if(compare(getFieldString(&data,1),"off"))
            {
                pi(0);
                putsUart0("priority inheritance is off.\r\n");
            }

            valid = true;
        }

        if(isCommand(&data, "preempt", 1))
        {
            if(compare(getFieldString(&data,1),"on"))
            {
                preempt(1);
                //putsUart0("preemption is on.\r\n");
            }
            else if(compare(getFieldString(&data,1),"off"))
            {
                preempt(0);
                //putsUart0("preemption is off.\r\n");
            }
            valid = true;
        }

        if(isCommand(&data, "sched", 1))
        {
            if(compare(getFieldString(&data,1),"rr"))
            {
                sched(1);
                putsUart0("Round robin scheduler is on.\r\n");
            }
            else if(compare(getFieldString(&data,1),"prio"))
            {
                sched(0);
                putsUart0("priority scheduling is on.\r\n");
            }
            valid = true;
        }

        if(isCommand(&data, "pidof", 1))
        {
            pidof(getFieldString(&data,1));
            valid = true;
        }

        if(isCommand(&data, "run", 1))
        {
            proc_name(getFieldString(&data,1));
            valid = true;
        }

        if(isCommand(&data, "mpu", 1))
        {
            if(compare(getFieldString(&data,1),"on"))
            {
                mpu(1);
            }
            else if(compare(getFieldString(&data,1),"off"))
            {
                mpu(0);
            }
            valid = true;
        }

        putsUart0("\r\n");

        if(!valid)
            putsUart0("Invalid Command\r\n");
    }

}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    initUart0();
    initRtos();
    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
//    keyPressed = createSemaphore(1, "keyPressed");
//    keyReleased = createSemaphore(0, "keyReleased");
//    flashReq = createSemaphore(5, "flashReq");
//    resource = createSemaphore(1, "resource");

    bool s1 = createSemaphore(keyPressed, 1, "keyPressed");
    bool s2 = createSemaphore(keyReleased, 0, "keyReleased");
    bool s3 = createSemaphore(flashReq, 5, "flashReq");
    bool s4 = createSemaphore(resource, 1, "resource");

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 15, 1024);
    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 12, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 8, 1024);
    ok &= createThread(oneshot, "OneShot", 4, 1024);
    ok &= createThread(readKeys, "ReadKeys", 12, 1024);
    ok &= createThread(debounce, "Debounce", 12, 1024);
    ok &= createThread(important, "Important", 0, 1024);
    ok &= createThread(uncooperative, "Uncoop", 10, 1024);
    ok &= createThread(errant, "Errant", 8, 1024);
    ok &= createThread(shell, "Shell", 8, 1024);

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}
