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

//void waitPbPress0()
//{
//    while(PUSH_BUTTON0);
//}
//
//void waitPbPress1()
//{
//    while(PUSH_BUTTON1);
//}
//
//void waitPbPress2()
//{
//    while(PUSH_BUTTON2);
//}
//
//void waitPbPress3()
//{
//    while(PUSH_BUTTON3);
//}
//
//void waitPbPress4()
//{
//    while(PUSH_BUTTON4);
//}
//
//void waitPbPress5()
//{
//    while(PUSH_BUTTON5);
//}


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

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
typedef struct _semaphore
{
    uint16_t count;                         //no of items in semaphore
    uint16_t queueSize;                     //no of items waiting to use process
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
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
uint8_t lastSearchedIndex[15];

int rtosScheduler()
{
    //Round Robin Scheduler
//    bool ok;
//    static uint8_t task = 0xFF;
//    ok = false;
//    while (!ok)
//    {
//        task++;
//        if (task >= MAX_TASKS)
//            task = 0;
//        ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
//    }
//    return task;

    //Priority Scheduler step 2
    bool ok;
    uint8_t prev_task = 0;
    uint8_t next_task = 1;
    ok = false;
    while (!ok)
    {
        while(next_task < MAX_TASKS)
        {
            if(tcb[prev_task].priority >= tcb[next_task].priority)
            {
                if(tcb[next_task].state == STATE_READY || tcb[next_task].state == STATE_UNRUN)
                    prev_task = next_task;
            }
            next_task++;
        }
        ok = (tcb[prev_task].state == STATE_READY || tcb[prev_task].state == STATE_UNRUN);
    }
    return prev_task;
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
            char str[100];
            putsUart0(tcb[i].name);
            putsUart0("\r\n");
            sprintf(str,"%p\r\n", tcb[i].pid);
            putsUart0(str);
            sprintf(str,"%d\r\n", tcb[i].priority);
            putsUart0(str);
            sprintf(str,"%p\r\n", tcb[i].spInit);
            putsUart0(str);
            sprintf(str,"%p\r\n", tcb[i].sp);
            putsUart0(str);
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

int8_t createSemaphore(uint8_t count)
{
    int8_t index = -1;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        semaphores[semaphoreCount].count = count;
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
    taskCurrent = rtosScheduler();
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
            semaphores[R0].processQueue[semaphores[R0].queueSize++] = taskCurrent;
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
    }
    //NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV ; //turning on the pendSV exception .... debugging code to verify
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

uint8_t getSVCNumber()
{
    uint32_t *PSP = (uint32_t*) getPSP();
    uint32_t *PC;
    PC = *(PSP+6) - 2;

    return *PC;
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
    while (true)
    {
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

    //debugging code for heap allocation verification
//    heap = heap + 1024;
//    char str[100];
//    sprintf(str, "%p", heap);
//    putsUart0(str);
//    putsUart0("\r\n");


    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    keyPressed = createSemaphore(1);
    keyReleased = createSemaphore(0);
    flashReq = createSemaphore(5);
    resource = createSemaphore(1);

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
    //ok &= createThread(uncooperative, "Uncoop", 10, 1024);
    //ok &= createThread(errant, "Errant", 8, 1024);
    ok &= createThread(shell, "Shell", 8, 1024);

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}
