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

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED         (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define RED_LED          (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))
#define ORANGE_LED       (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))
#define YELLOW_LED       (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4)))
#define GREEN_LED        (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))

//bitbanding addresses for PUSH BUTTONS
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

#define MAX_CHARS 80
#define MAX_FIELDS 5
typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
}USER_DATA;

//shell
struct _shell
{
    char str[21];              //name of the pid
    uint32_t pid;                       //address of the pid
    int8_t mode;                       //round robin or priority
    int8_t preempt;                       //preemption on = true and off = false
    int8_t pi;                            //priority inheritance on or off
}myshell;

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
typedef struct _semaphore
{
    uint16_t count;                         //no of items in semaphore
    uint16_t queueSize;                     //no of items waiting to use process
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    char name[16];
} semaphore;

semaphore semaphores[MAX_SEMAPHORES];

uint8_t semaphoreCount = 0;

uint8_t keyPressed, keyReleased, flashReq, resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks
uint32_t initial_time = 0;
uint32_t final_time = 0;
uint32_t time_difference;
uint32_t total_time = 0;
uint32_t total_time_temp = 0;
uint32_t j = 0; //global variable to swap the buffer
uint32_t N = 0;     //variable to keep track of the Number of times the systick isr is called.

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
    uint32_t t1;
    uint32_t swapBuffer[2];
} tcb[MAX_TASKS];

//Allocating 28KiB space for the heap in stack...2KiB for the MSP stack and remaining 2KiB for the OS variables/kernel
#pragma DATA_SECTION(mystack, ".heap")
uint32_t mystack[7168];
uint32_t heap = (uint32_t*)mystack ;

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
uint32_t last_searched_task[16];
uint16_t prio = 0;
int rtosScheduler()
{
    bool ok;
    ok = false;
    //Round Robin Scheduler
    if(myshell.mode == 1)
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

    else if(myshell.mode == 0)
    {
        while (!ok)
        {
            uint8_t prev_task = last_searched_task[prio];
            uint8_t i = 0;
            if(prev_task > 0)
            {
                prev_task++;
                i++;
            }
            while(prev_task < MAX_TASKS)
            {

                if((tcb[prev_task].currentPriority == prio && (tcb[prev_task].state == STATE_UNRUN || tcb[prev_task].state == STATE_READY)))
                {

                    last_searched_task[prio] = prev_task;

                    ok = true;
                    return prev_task;
                }
                prev_task++;

                i++;
            }

            if(prev_task == MAX_TASKS)
            {
               last_searched_task[prio] = 0;
               prio = prio + 1;
            }

            if(prio == 15) prio = 0;
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
            tcb[i].spInit = heap + stackBytes;
            tcb[i].sp = heap;
            heap = heap + stackBytes;
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            copy(name,tcb[i].name);
//            char str[100];
//            putsUart0(tcb[i].name);
//            putsUart0("\r\n");
//            sprintf(str,"%p\r\n", tcb[i].pid);
//            putsUart0(str);
//            sprintf(str,"%d\r\n", tcb[i].priority);
//            putsUart0(str);
//            sprintf(str,"%p\r\n", tcb[i].spInit);
//            putsUart0(str);
//            sprintf(str,"%p\r\n", tcb[i].sp);
//            putsUart0(str);
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
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void destroyThread(_fn fn)
{
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
}

int8_t createSemaphore(uint8_t count, char str[])
{
    int8_t index = -1;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        semaphores[semaphoreCount].count = count;
        copy(str,semaphores[semaphoreCount].name);
        index = semaphoreCount;
        semaphoreCount++;
    }
    return index;
}

// REQUIRED: modify this function to start the operating system, using all created tasks
void startRtos()
{
    taskCurrent = rtosScheduler();
    uint32_t SP = tcb[taskCurrent].spInit;
    setPSP(SP);

    //Timer code
    //Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                                // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;                          // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;     // configure for periodic mode (count down)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                                 // turn-on timer

    tcb[taskCurrent].state = STATE_READY;
    setASP();
    _fn fn = tcb[taskCurrent].pid;
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
    if(N == 1000)
    {
        while(k < MAX_TASKS)
        {
          tcb[k].swapBuffer[j] = tcb[k].t1;
          k++;
        }
        j = 1-j;
        total_time_temp = total_time;
        total_time = 0;
    }
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    //debugging code
    //BLUE_LED = 1;
    //step 6 code and discard
//    __asm("     MRS R0, PSP");
//    __asm("     STR R4, [R0, #-4]");
//    __asm("     SUB R0, R0, #4");
//    __asm("     STR R5, [R0, #-4]");
//    __asm("     SUB R0, R0, #4");
//    __asm("     STR R6, [R0, #-4]");
//    __asm("     SUB R0, R0, #4");
//    __asm("     STR R7, [R0, #-4]");
//    __asm("     SUB R0, R0, #4");
//    __asm("     STR R8, [R0, #-4]");
//    __asm("     SUB R0, R0, #4");
//    __asm("     STR R9, [R0, #-4]");
//    __asm("     SUB R0, R0, #4");
//    __asm("     STR R10, [R0, #-4]");
//    __asm("     SUB R0, R0, #4");
//    __asm("     STR R11, [R0, #-4]");
//    __asm("     SUB R0, R0, #4");
//    tcb[taskCurrent].sp = getPSP();
//    taskCurrent = rtosScheduler();
//    setPSP(tcb[taskCurrent].sp);
//    __asm("     MRS R0, PSP");
//    __asm("     LDR R4, [R0, #-4], #-4");
//    __asm("     LDR R5, [R0, #-4], #-4");
//    __asm("     LDR R6, [R0, #-4], #-4");
//    __asm("     LDR R7, [R0, #-4], #-4");
//    __asm("     LDR R8, [R0, #-4], #-4");
//    __asm("     LDR R9, [R0, #-4], #-4");
//    __asm("     LDR R10, [R0, #-4], #-4");
//    __asm("     LDR R11, [R0, #-4], #-4");


    //step 7
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
    tcb[taskCurrent].sp = getPSP();

    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;       //turn off timer1
    final_time =TIMER1_TAV_R;
    TIMER1_TAV_R=0;

    time_difference = final_time - initial_time;
    total_time += time_difference;
    tcb[taskCurrent].t1 = time_difference;

    taskCurrent = rtosScheduler();
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
    initial_time =TIMER1_TAV_R;

    if(tcb[taskCurrent].state == STATE_READY){
        setPSP(tcb[taskCurrent].sp);
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
        tcb[taskCurrent].state = STATE_READY;
        setPSP((tcb[taskCurrent].spInit)-32);
        _fn fn = tcb[taskCurrent].pid;

        uint32_t *PSP = (uint32_t*) getPSP();

        *PSP  = 0;
        *(PSP + 1) = 1;
        *(PSP + 2) = 2;
        *(PSP + 3) = 3;
        *(PSP + 4) = 12;
        *(PSP + 5) = 0xFFFFFFFD;
        *(PSP + 6) = fn;
        *(PSP + 7) = 0x61000000;

        //code and discard part below
        //char str[100];
//        sprintf(str,"%p\r\n", PSP);
//        putsUart0(str);
//        sprintf(str,"%p\r\n", (PSP + 1));
//        putsUart0(str);
//        sprintf(str,"%p\r\n", (PSP + 2));
//        putsUart0(str);
//        sprintf(str,"%p\r\n", (PSP + 3));
//        putsUart0(str);
//        sprintf(str,"%p\r\n",(PSP + 4));
//        putsUart0(str);
//        sprintf(str,"%p\r\n", (PSP + 5));
//        putsUart0(str);
//        sprintf(str,"%p\r\n", (PSP + 6));
//        putsUart0(str);
//        sprintf(str,"%p\r\n", (PSP + 7));
//        putsUart0(str);

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
    uint8_t n = getSVCNumber();
    uint8_t numTasks = 0;
    semaphore *temp_semaphore;
    char *pid_name = (char*)R0;


    switch(n)
    {
    case 0:
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV ; //turning on the pendSV exception .... debugging code to verify
        break;

    case 1:
        tcb[taskCurrent].ticks = R0;
        tcb[taskCurrent].state = STATE_DELAYED;
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV ;
        NVIC_ST_RELOAD_R = 39999;
        //NVIC_ST_CURRENT_R = 0;
        NVIC_ST_CTRL_R |= 0x7; //Enabling clk_src(system clock), inten and enable bit
        break;

    case 2:
        if(semaphores[R0].count > 0){
            semaphores[R0].count--;
        }
        else{
            tcb[taskCurrent].state = STATE_BLOCKED;
            tcb[taskCurrent].semaphore = &semaphores[R0];
            semaphores[R0].processQueue[semaphores[R0].queueSize] = taskCurrent;
            semaphores[R0].queueSize++;
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        }
        break;

    case 3:
        semaphores[R0].count++;
        if(semaphores[R0].queueSize > 0)
        {
            tcb[semaphores[R0].processQueue[0]].state = STATE_READY;
            semaphores[R0].count--;
            uint8_t j = 0;
            for(j = 0; j<semaphores[R0].queueSize; j++)
            {
                semaphores[R0].processQueue[j] = semaphores[R0].processQueue[j+1];
            }
            semaphores[R0].queueSize--;
        }
//        if(tcb[semaphores[R0].processQueue[0]].currentPriority < tcb[taskCurrent].priority){
//            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
//         }
        break;

    case 5:
        putsUart0("Name\t\t PID\t\t%CPU\t\tPriority\t\tState\r\n");
        while(numTasks < taskCount)
        {
            copy(tcb[numTasks].name, myshell.str);
            putsUart0(myshell.str);
            uint16_t len = getLength(myshell.str);
            if( len <= 7)
            {
                putsUart0("\t\t");
            }
            else
            {
                putsUart0("\t");
            }

            uint16_t val = tcb[numTasks].pid;
            uint32_tToHex(val);
            putsUart0("\t\t");

            //Location holder for %CPU
            uint32_t t = tcb[numTasks].swapBuffer[1-j];
            uint32_t process_time = (t*10000)/total_time_temp;
            IntegerToString(process_time,myshell.str);
            putsUart0(myshell.str);
            putsUart0("\t\t");

            IntegerToString(tcb[numTasks].currentPriority,myshell.str);
            putsUart0(myshell.str);
            putsUart0("\t\t");

            uint8_t tempState = tcb[numTasks].state;
            if(tempState == 0)
            {
                putsUart0("INVALID");
            }
            if(tempState == 1)
            {
                putsUart0("UNRUN");
            }
            if(tempState == 2)
            {
                putsUart0("READY");
            }
            if(tempState == 3)
            {
                putsUart0("DELAYED FOR ");
                IntegerToString(tcb[numTasks].ticks,myshell.str);
                putsUart0(myshell.str);
                putsUart0("ms");
            }
            if(tempState == 4)
            {
                putsUart0("BLOCKED BY ");
                temp_semaphore = tcb[numTasks].semaphore;
                copy(temp_semaphore->name,myshell.str);
                putsUart0(myshell.str);

            }
            putsUart0("\t");

            numTasks++;
            putsUart0("\r\n");

        }

        break;

    case 6:
        putsUart0("Name\t\tCount\tWaiting Threads\r\n");
        while(numTasks < semaphoreCount)
        {
            copy(semaphores[numTasks].name,myshell.str);
            putsUart0(myshell.str);
            putsUart0("\t");

            IntegerToString(semaphores[numTasks].count,myshell.str);
            putsUart0(myshell.str);
            putsUart0("\t");

            int8_t queue_size = semaphores[numTasks].queueSize;
            for(queue_size = semaphores[numTasks].queueSize; queue_size >= 0 ; queue_size--)
            {
                copy(tcb[semaphores[numTasks].processQueue[queue_size]].name,myshell.str);
                putsUart0(myshell.str);
                putsUart0("   ");
            }

            numTasks++;
            putsUart0("\r\n");
        }
        break;

    case 7:
        numTasks = 1;
        if((uint32_t)tcb[0].pid == R0)
        {
           putsUart0("Cannot kill the Idle process.\r\n");
        }
        while((numTasks) < taskCount)
        {
           if((uint32_t)tcb[numTasks].pid == R0)
           {
                tcb[numTasks].state = STATE_INVALID;
           }
            numTasks++;
        }

        break;
    case 8:
        myshell.pi = R0;
        break;
    case 9:
        myshell.preempt = R0;
        break;
    case 10:
        myshell.mode = R0;
        break;

    case 11:

        while(numTasks < taskCount)
        {
            if(compare(tcb[numTasks].name,pid_name))
            {

                uint16_t val = tcb[numTasks].pid;
                uint32_tToHex(val);
                putsUart0("\r\n");
            }
            numTasks++;
        }

        break;

    case 12:
        while(numTasks < taskCount)
        {
            if(compare(tcb[numTasks].name,pid_name))
            {
                if(tcb[numTasks].state != STATE_READY)
                {
                    tcb[numTasks].state = STATE_READY;
                }
            }
            numTasks++;
        }

        break;

    case 13:
        NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ|NVIC_APINT_VECT_RESET;
        break;
    }
}

// REQUIRED: code this function
void mpuFaultIsr()
{
}

// REQUIRED: code this function
void hardFaultIsr()
{
}

// REQUIRED: code this function
void busFaultIsr()
{
}

// REQUIRED: code this function
void usageFaultIsr()
{
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
void copy(char s1[], char s2[]){
    uint8_t i=0;
    for (i = 0; s1[i] != '\0'; ++i) {
            s2[i] = s1[i];
        }
    s2[i] = '\0';
}

uint16_t getLength(char str[])
{
    uint8_t i=0;
    while(str[i] != '\0')
    {
        i++;
    }
    return i;
}

uint8_t getSVCNumber()
{
    uint32_t *PSP = (uint32_t*) getPSP();
    uint32_t *PC;
    PC = *(PSP+6) - 2;

    return *PC;
}

//this function converts uint32_t value into hexadecimal value and prints the result.
void uint32_tToHex(uint32_t decimal){
    uint32_t quotient, remainder;
    int i,j = 0;
    char hex[100];

    quotient = decimal;

    while (quotient != 0)
    {
       remainder = quotient % 16;
       if (remainder < 10){
           hex[j++] = 48 + remainder;
       }

       else{
           hex[j++] = 55 + remainder;
       }

          quotient = quotient / 16;
     }

//    if(quotient == 0 && j != 8){
//        while(j!=8){
//            hex[j++] = 48;
//        }
//    }

    for (i = j-1; i >= 0; i--)
           putcUart0(hex[i]);
}

void IntegerToString(uint32_t num, char str[])
{
    uint32_t i, rem, len = 0, n;

    n = num;
    if(n == 0)
    {
        str[len] = '0';
        str[len+1] = '\0';
    }

    else
    {
        while (n != 0)
        {
            len++;
            n /= 10;
        }
        for (i = 0; i < len; i++)
        {
            rem = num % 10;
            num = num / 10;
            str[len - (i + 1)] = rem + '0';
        }

        str[len] = '\0';

    }
}

int StringToInteger(char* str)
{
    uint32_t num = 0;
    int i;

    for (i = 0; str[i] != '\0'; ++i)
        num = num * 10 + str[i] - '0';


    return num;
}

uint32_t hex2int(char hex[]) {
    uint32_t val = 0;
    int i =0;
    while (hex[i] != 0) {
        // get current character then increment
        uint8_t byte = hex[i];
        // transform hex character to the 4bit equivalent number, using the ascii table indexes
        if (byte >= '0' && byte <= '9') byte = byte - '0';
        else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
        else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;
        // shift 4 to make space for new digit, and add the 4 bits of the new digit
        val = (val << 4) | (byte & 0xF);
        i++;
    }
    return val;
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

void ps()
{
    __asm(" SVC #5");
}

void ipcs()
{
    __asm(" SVC #6");
}

void kill(uint32_t val){
    __asm(" SVC #7");
}

void pi(int8_t val){
    __asm(" SVC #8");
}

void preempt(int8_t val)
{
    __asm(" SVC #9");
}

void sched(int8_t val)
{
    __asm(" SVC #10");
}

void pidof(char name[])
{
    __asm(" SVC #11");
}

void proc_name(char name[])
{
    __asm(" SVC #12");
}

void reboot()
{
    __asm(" SVC #13");
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

//void idle2()
//{
//    while(true)
//    {
//        //code and discard
//        //char str[100];
//        //sprintf(str,"%p\r\n", tcb[taskCurrent].sp);
//        //putsUart0(str);
//        //Debugging code
////        __asm("  MOV R0, #0");
////        __asm("  MOV R1, #1");
////        __asm("  MOV R2, #2");
////        __asm("  MOV R3, #3");
////        __asm("  MOV R12,#12");
////        __asm("  MOV R4,#4");
////        __asm("  MOV R5,#5");
//        YELLOW_LED = 1;
//        waitMicrosecond(1000);
//        YELLOW_LED = 0;
//        yield();
//    }
//}

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
            ps();
            valid = true;
        }

        if(isCommand(&data, "ipcs", 0))
        {
            ipcs();
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
            }
            else if(compare(getFieldString(&data,1),"off"))
            {
                pi(0);
            }

            valid = true;
        }

        if(isCommand(&data, "preempt", 1))
        {
            if(compare(getFieldString(&data,1),"on"))
            {
                preempt(1);
            }
            else if(compare(getFieldString(&data,1),"off"))
            {
                preempt(0);
            }
            valid = true;
        }

        if(isCommand(&data, "sched", 1))
        {
            if(compare(getFieldString(&data,1),"rr"))
            {
                sched(1);
            }
            else if(compare(getFieldString(&data,1),"prio"))
            {
                sched(0);
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
    keyPressed = createSemaphore(1, "keyPressed");
    keyReleased = createSemaphore(0, "keyReleased");
    flashReq = createSemaphore(5, "flashReq");
    resource = createSemaphore(1, "resource");

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 15, 1024);
    //Code and discard
//    ok &=  createThread(idle2, "Idle2", 14, 1024);

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
