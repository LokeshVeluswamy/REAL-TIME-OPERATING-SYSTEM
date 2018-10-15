// RTOS Framework - Spring 2018
// J Losh

// Student Name:   LOKESH VELUSWAMY
// TO DO: Add your name on this line.  Do not include your ID number.

// Submit only two .c files in an e-mail to me (not in a compressed file):
// xx_rtos.c   Single-file with your project code
// (xx is a unique number that will be issued in class)
// Please do not include .intvecs section in your code submissions
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

//#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include <hw_nvic.h>
#include <hw_types.h>


// REQUIRED: correct these bitbanding references for the off-board LEDs
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))  ///off board Led
#define YELLOW_LED    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4))) ///off board Led
#define ORANGE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4))) ///off board Led
#define RED_LED     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))   ///off board Led
#define BLUE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))   ///off board Led

/// PUSH BUTTONS

#define PB4  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))   ///// Red Board Push Button
#define PB0  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))
#define PB1 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define PB2  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define PB3  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();


uint32_t exsp;  /// Variables to Set and Get Stack Pointer
uint32_t jxsp;   //
uint32_t temp;
bool pre_flag=0;  // Flag to start the preemption operation only after the rtos-start is run
bool pre_empt=1;  // Flag to turn on preemption and Co-Op (default Pre emption ON)
uint32_t flag_count=0;

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
  uint16_t count;
  uint16_t queueSize;
  uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_BLOCKED    3 // has run, but now blocked by semaphore
#define STATE_DELAYED    4 // has run, but now awaiting timer

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

struct _tcb
{
  uint8_t state;                 // see STATE_ values above
  void *pid;                     // used to uniquely identify thread
  void *sp;                      // location of stack pointer for thread
  uint8_t priority;              // 0=highest, 7=lowest
  uint8_t currentPriority;       // used for priority inheritance
  uint32_t ticks;                // ticks until sleep complete
  uint8_t skipcount;             // Holds the Count to schedule the corresponding task  at an instant of time
  char name[16];                 // name of task used in ps command
  void *semaphore;               // pointer to the semaphore that is blocking the thread
  uint32_t t1;                   // Intial Cpu time
  uint32_t t2;                   // Final Cpu time
  uint32_t time;                 // Total time used by the thread
  double past_time;
  double cpu_time;             // Total cpu time to calculate %
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread
double total_time;     /// Variable to Calcualte the CPU time
double alpha=0.99;     /// Alpha value to Calculate the Cpu Time


//// Variables to accept the input and process the size and position
#define  max 80                   /// maximum array of type, position, field

//#define  repeatcount 3000
#define  max_packet_size 50       /////Maximum Packet Size
char str[max];                    /// String which takes input
char* str1;                       /// string to store the string got i.e getstring is stored
// step3 position, type and field
char position[max];               /// string of position
char type[max];                   ////string of type
uint32_t field=0;                 ///// variables used in step 3
uint8_t i=0;
uint8_t count=0;               /// Below are the variables used in step3 for Accepting the input from the User
    char c;
    uint8_t y=0;
    uint8_t l=0;
    uint8_t valid=0;
    char* strcmd;
    char* disp[max];

    bool pi_flag=1;
    bool kill_flag=0; /// variable for kill and create thread by user input
    uint8_t inc3=0;   // Global variable for kill and create thread by user input to store the local count value
//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

void rtosInit()
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
  // REQUIRED: initialize systick for 1ms system timer
  NVIC_ST_RELOAD_R=39999;
    NVIC_ST_CURRENT_R=1;
    NVIC_ST_CTRL_R|=0x0007;
    NVIC_INT_CTRL_R|=0x04000000;

}
void setSP(void *temp)            //// Function to set the Local Thread Stack Pointer to the  Internal Stack Pointer
{
    __asm("   MOV SP,R0");
    __asm("   BX LR");
}
uint32_t getSP()                /////  Function to set the Internal Stack Pointer to the Local Thread Stack Pointer
{
    __asm("    MOV R0,SP");
}
void rtosStart()
{
  // REQUIRED: add code to call the first task to be run
  _fn fn;
  jxsp = getSP();                            /// Storing the Redundant SP. i.e the Stack Before the Thread or Rtos has Started
  taskCurrent = rtosScheduler();            /// Scheduler finds a thread in state Ready to Run
  setSP(tcb[taskCurrent].sp);               /// Setting the Stack Pointer of Internal Thread to The Internal Stack
  tcb[taskCurrent].state=STATE_READY;
  fn=(_fn) tcb[taskCurrent].pid;            /// Giving the Process Id to the Thread
  pre_flag=1;                               /// Flag to Enable Pre-Emption Operation
  (*fn)();

  // Add code to initialize the SP with tcb[task_current].sp;
}
bool get_Ok(){}       /// Funtion to Get Ok

bool createThread(_fn fn, char name[], int priority)          /////// CREATE THREAD
{
    bool ok;
    __asm("   MOV R4,R0");           /// Registers Used to Hold the Argument Value to pass the arguments to the SVC
    __asm("   MOV R5,R1");
    __asm("   MOV R6,R2");
    __asm("   SVC #0x15");          /// Calling the SVC (Kernal Operation)
    __asm("   MOV R0,R8");         // After Returning From the SVC The Value of OK is Retrived from Register R8
    ok=get_Ok();                    // Retriving Ok
  return ok;
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)                 ////// DESTROY THREAD
{
    __asm("   MOV R4,R0");             /// Registers Used to Hold the Argument Value to pass the arguments to the SVC
    __asm("   SVC #0x12");             /// Calling the SVC (Kernal Operation)
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)          //// SET THREAD PRIORITY
{
    for(i=0;i<taskCount;i++)                        /// Elevating the Priority of the Lengthy Funtion the the Priority Level Defined
    {
        if(tcb[i].pid==fn)
        {
            tcb[i].priority=priority;               /// Assigining Priority to Current Priority
            tcb[i].currentPriority=priority;        /// Assigining Priority to  Priority
        }
    }
}

struct semaphore* createSemaphore(uint8_t count)
{
  struct semaphore *pSemaphore = 0;
  if (semaphoreCount < MAX_SEMAPHORES)
  {
    pSemaphore = &semaphores[semaphoreCount++];
    pSemaphore->count = count;
  }
  return pSemaphore;
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv

void yield()            ////YIELD
{
  // push registers, call scheduler, pop registers, return to new function
    __asm("   SVC #0x10");          /// Calling SVC (Kernel Function) No arguments to Pass

}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)    //// SLEEP
{
  // push registers, set state to delayed, store timeout, call scheduler, pop registers,
  // return to new function (separate unrun or ready processing)
    __asm("   MOV R4,R0");     // Registers Used to Hold the Argument Value to pass the arguments to the SVC
    __asm("   SVC #0x11");      /// Calling the SVC (Kernal Operation)
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)    ////WAIT
{
       __asm("   MOV R4,R0");        // Registers Used to Hold the Argument Value to pass the arguments to the SVC
       __asm("   SVC #0x14");       /// Calling the SVC (Kernal Operation)

}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)            ///POST
{
    __asm("   MOV R4,R0");              // Registers Used to Hold the Argument Value to pass the arguments to the SVC
    __asm("   SVC #0x13");          /// Calling the SVC (Kernal Operation)
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()                                    ////////RTOS SCHEDULER
{
  bool ok;
  static uint8_t task = 0xFF;
  ok = false;
  while (!ok)
  {
    task++;
    if (task >= MAX_TASKS)
      task = 0;
    if(tcb[task].skipcount>=tcb[task].currentPriority)
    {
    ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
   if(ok)
       {
       tcb[task].t1=TIMER1_TAV_R;                                         ////// LOAD THE INITIAL TIMER VALUE TO THE TASK. T1 VARIABLE
       tcb[taskCurrent].t2=TIMER1_TAV_R;                                  /// LOAD THE FINAL TIMER VALUE TO THE T2 OF THE CURRENT TASK FOR TIME CALCULATION BEFORE THE TASK IS SWITCHED
       tcb[taskCurrent].time=tcb[taskCurrent].t1-tcb[taskCurrent].t2;    /// CALCULATE THE FINAL TIME DIFFERENCE BTW THE T1 AND T2 OF CURRENT TASK BEFORE THE TASK IS SWITCHED
       }
    tcb[task].skipcount=0;

    }
    else
        tcb[task].skipcount++;
  }
  return task;
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()                                                         //// SYSTICK TIMER ISR
{
    flag_count++;
    if(flag_count>=100000)         /// Code to Clear the Time value for every 100 seconds if the Thread is Inactive
    {
        flag_count=0;                /// Clear the Flag Count
        uint8_t j=0;
        for(j=0;j<taskCount;j++) tcb[j].time=0;       // Clear the time of all the Threads to Zero
     }
    uint32_t i;
    for (i = 0; i < MAX_TASKS; i++)                      ////// Scan through the Tasks
    {
    if(tcb[i].state == STATE_DELAYED)                      /// If the State of the Task is  Delayed
    {
        tcb[i].ticks--;                                  // Decrement the Ticks
        if (tcb[i].ticks==0)                             // If the Ticks is zero
        {
            tcb[i].state = STATE_READY;                   /// Switch the state to State Ready
        }
    }
    }
    if (pre_empt==1)
    {
    if(pre_flag==1)                                      //// If the Pre-Emptive flag is 1
        NVIC_INT_CTRL_R |= 0x10000000;             //// Scan through If there are any Higher Priority Task Every 1ms( Time Duration)
    }
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()                                          ///// PendSV Interrupt Isr
{

                       __asm("   PUSH{R11}");                  //// Registers that the Callee Functions Must Save
                       __asm("   PUSH{R10}");                 //
                       __asm("   PUSH{R9}");                  //
                       __asm("   PUSH{R8}");                  //
                       __asm("   PUSH{R7}");                  //
                       __asm("   PUSH{R6}");                  //
                       __asm("   PUSH{R5}");                  //
                       __asm("   PUSH{R4}");                  //
        exsp = getSP();                           /// Context Save Using the GETSP
        tcb[taskCurrent].sp = (void*)exsp;        /// GET SP
       setSP((void*)jxsp);                        // SET REDUNDANT SP
        taskCurrent = rtosScheduler();            // Call the Scheduler
       setSP( tcb[taskCurrent].sp);                ///// we can also set sp before entering the state but we do it in each thread state
        if(tcb[taskCurrent].state==STATE_READY)    /// If the state is Already RUN
        {
        setSP(tcb[taskCurrent].sp);                // SET THE SP
        tcb[taskCurrent].state=STATE_READY;        // SET THE STATE TO STATE READY
        __asm("   POP{R4}");                    ////// The Context Saved Variables is Popped to return to the normal tast which was run Before
        __asm("   POP{R5}");                      ///
        __asm("   POP{R6}");                      ///
        __asm("   POP{R7}");                      ///
        __asm("   POP{R8}");                      ///
        __asm("   POP{R9}");                      ///
        __asm("   POP{R10}");                     ///
        __asm("   POP{R11}");                   // Till This Registers were the Callee Save Registers
        __asm("   POP{R1}");                    ///// These Registers Were Hardware Pushed LOOK into Assembly to find These Register Pushes
        __asm("   POP{R2}");                        //
        __asm("   POP{R3}");                        //
        __asm("   POP{LR}");                      //// Pop the LR to Return to the Task
        }
        if(tcb[taskCurrent].state==STATE_UNRUN)            //// Every new thread runs this UNRUN state Initially
        {
            setSP(tcb[taskCurrent].sp);
                    tcb[taskCurrent].state=STATE_READY;
                    uint32_t virpsr=0x01000000;          /// variable declaration will push the value in the stack i.e xpsr so no need to worry
                     _fn fn=tcb[taskCurrent].pid;         /// pid of the current thread is pushed first then psr
                    __asm(" PUSH{R0}");
                    __asm(" PUSH{LR}");                  //// caller variables to be saved From this Registers// TO Create an Illusion that the Task was Already Run - START
                    __asm(" PUSH{R12}");                //
                    __asm(" PUSH{R3}");                 //   THE FORMAT OF THIS PUSHES ARE PRE-DEFINED CHECKED INTO THE MEMORY BROWSER FOR PUSH FORMAT
                    __asm(" PUSH{R2}");                 //
                    __asm(" PUSH{R1}");                 //
                    __asm(" PUSH{R0}");                 ////  TILL HERE-END
        }
        __asm("   MOV LR, #0x0000FFF9");                ////// The LR is loaded with the value of #fff9
        __asm("   MOVT LR,#0xFFFF");                    ////// The higher 2 bytes of LR is loaded with #FFFF
        __asm("   BX LR");                              ////// LR is poped to return to the Task
}

// REQUIRED: modify this function to add support for the service call

uint8_t PutR0_uint(){}                              /// Function to Return the RO value  which is 8-bit
struct semaphore *PutR0_semaphore(){}              ////// Funtion to Retrive the psemaphore i.e Pointer to the Semaphore
void return_ok(bool ok)                               //// Function to Return the Ok
{
    __asm("  MOV R8,R0");                             // Register the store the Status of the OK
}
uint32_t PutR0_func(){}                            // Register to Retrive the Value of the FN 32-bit Return Value
unsigned char* PutR0_str(){}                       // Function to Return Pointer to the Name Variable in Create Thread
// REQUIRED: in preemptive code, add code to handle synchronization primitives

void svCallIsr()                                                            //////SVCALL ISR
{
    __asm(" LDR R1, [SP, #0x58]") ;                                        //// Get the Stacked Program Counter which holds the Address for the Call value used in the SVCALL
    __asm(" LDRB R0, [R1, #-2]") ;                                         /// Get the SVC Call Value which is Stored in PC-2 Location only the LOWER BYTE .I.E SO LDRB

    uint8_t svcno=PutR0_uint();                                           /// FUNCTION CALL TO PUT THE CASE STATUS STORED IN RO AFTER EXECUTING PREVIOUS ASM LANGUAGE TO SVCNO
    uint32_t tick=0;                                                    /// Local Variables used in the SVCALL FOR DIFFERENT CASES -START
    _fn fn=0;                                                           //
    uint8_t j=0;                                                        //
    struct semaphore *pSemaphore;                                       //
    uint8_t priority;                                                   //
    char* name;                                                         ////// - END
    switch(svcno)
    {
    case 16:        //// YIELD
    {
        tcb[taskCurrent].state=STATE_READY;                           /// AFTER YIELD MAKE THE STATE TO STATE READY TO FIND THE OTHER TASK
        NVIC_INT_CTRL_R |= 0x10000000;                                /// SET THE PENDSV BIT TO CALL PENDSVISR
        break;
    }
    case 17:     ///SLEEP
    {
      __asm("   MOV R0,R4");                                            /// RETRIVE THE ARGUMENTS USED BY SLEEP PUSHED IN THE CALL FUNCTION
      tick=PutR0_uint();                                                // RETIVE THE TICKS VALUE FROM THE RO BY CALLING A FUNCTION
      tcb[taskCurrent].ticks=tick;                                       //// STORE THE TICK VALUE IN THE TASK
      tcb[taskCurrent].state=STATE_DELAYED;                              ////  MAKE THE STATE DELAYED
      NVIC_INT_CTRL_R |= 0x10000000;                                     /// SET THE PENDSV BIT TO CALL PENDSVISR
    break;
    }
    case 18:  ///DELETE THREAD
    {
        uint8_t l=0;                                                    /// Local Variables for Delete Thread
        uint8_t k=0;
    __asm("   MOV R0,R4");                                              // Retrive the Arguments Used in Delete Thread pushed in the call Function
       fn=(_fn) PutR0_uint();                                           // Retrive the FN VALUE BY CALLING A FUNCITON RETURN TYPE OF 32-BIT
       for(i=0;i<taskCount;i++)
       {                                                                // FIND THE TASK
           if(fn==tcb[i].pid)
           {
               tcb[i].state=STATE_INVALID;                              // Delete the TASK By following Operations
               tcb[i].pid=0;                                            // PID = 0
               tcb[i].cpu_time=0;                                       //  CPU TIME =0;
               tcb[i].time=0;                                           //   TIME CALCULATION =0;
               tcb[i].past_time=0;                                      //   PREVIOUS TIME FOR IIR SHOULD BE MADE 0;
               tcb[i].t1=0;                                             // INITIAL TIME MADE ZERO
               tcb[i].t2=0;                                             // FINAL TIME MADE ZERO

              for (j=0;j<=taskCount;j++)                                // THIS FOLLOWING CODE CHECKS FOR THE SIMILAR SEMAPHORE TASK AND IF IT IS BLOCKED MAKE IT RUN IF COUNT NOT EQUAL TO ZERO
                    {
                     if ((tcb[i].semaphore)==(tcb[j].semaphore))       //
                     {
                         if (tcb[j].state==STATE_BLOCKED)              //
                         {
                             tcb[j].state=STATE_READY;                 // Make the task Ready again if it is blocked
                             for (k=0;k<=semaphoreCount;k++)           // Scan through the semaphore table
                             {
                                 for (l=0;l<=MAX_QUEUE_SIZE;l++)
                                   {
                             if(tcb[j].pid==semaphores[k].processQueue[l])
                             {
                             semaphores[k].processQueue[l]=0;         // Remove the task from the SEMAPHORE PROCESS QUEUE
                             semaphores[k].queueSize--;               // Decrement the Queue size
                             break;
                             }
                                   }
                             }
                         }
                     }
                    }
               break;
           }
       }

       for(j=0;j<semaphoreCount;j++)                                  ////// This function is Used to  Move the Tasks in the Process Queue further
       {
           for(i=0;i<semaphores[j].queueSize;i++)
           {
               if (semaphores[j].processQueue[i]==(uint32_t)fn)
               {
                   semaphores[j].processQueue[i]=0;                  // Remove the task from the SEMAPHORE PROCESS QUEUE
                   semaphores[j].queueSize--;                       // Decrement the Queue size
                   while(i<semaphores[j].queueSize)
                   {
                       if (semaphores[j].processQueue[i]==0)
                       {
                           semaphores[j].processQueue[i]=semaphores[j].processQueue[i+1];   // Move the Tasks Further
                       }
                   }
                   break;
               }
           }
       }
       for(k=0;k<taskCount;k++)                                /// Function to Manage the task Creation and Task Deletion (This Funtion is Present in both Create thread and Delete Thread )
        {
           if(strcmp(tcb[k].name,"\0")==0)                                //
               taskCount--;                                               // Decrement the Task Count if it is a Redundant Task
        }
       break;
    }
    case 19:   // POST
    {
       uint8_t i=0;
       __asm("   MOV R0,R4");                                 /// Retrive the Arguments Pushed in the post call
       pSemaphore=PutR0_semaphore();                          // Retrive the pointer to the semaphore  // Funtion is Struct
       (pSemaphore->count)++;                                 /// Increment the Count Value if the Semaphore is Posted
       if ((pSemaphore->queueSize)>0)
       {
           (pSemaphore->queueSize)--;                          /// Remove the TASK From the Queue
           (pSemaphore->count)--;
           tcb[taskCurrent].currentPriority=tcb[taskCurrent].priority;    /// This is Done Because Restoring the Original Priority to the Current Priority
           tcb[taskCurrent].state=STATE_READY;                            //  Current Task is Made Ready
          for (i = 0;i<taskCount;i++)                                     /// Scan through the Table
           {
               if(pSemaphore->processQueue[pSemaphore->queueSize]==(uint32_t)tcb[i].pid)   /// If the Blocked task is found
               {
                   tcb[i].state=STATE_READY;                            /// The blocked Task is Made State Ready
                  NVIC_INT_CTRL_R |= 0x10000000;                        /// Pendsv bit is Set to Call Pendsv
               }
           }
       }
       break;
    }
    case 20:   //WAIT
       {
          __asm("   MOV R0,R4");                                     /// Retrive the Arguments Pushed in the Wait call
          pSemaphore=PutR0_semaphore();                               // Retrive the pointer to the semaphore  // Funtion is Struct
          tcb[taskCurrent].semaphore=pSemaphore;                        /// Store the Semaphore
                       if(pSemaphore->count>0)
                       {
                           pSemaphore->count--;                            // Decrement the count
                       }
                       else
                       {
                           uint8_t i=0;                                                                        ///// if the Count is not available the block the state
                          pSemaphore->processQueue[pSemaphore->queueSize]=(uint32_t) tcb[taskCurrent].pid;      // Write the state into the Process Queue
                          pSemaphore->queueSize++;                                                              // Increment the Queue Size
                          tcb[taskCurrent].state=STATE_BLOCKED;                                                 // Block the State
                          if(pi_flag==1)                                                                        // Enable and Disable the PI
                          {
                          for (i=0; i<=taskCount;i++)
                          {
                           if ((tcb[taskCurrent].semaphore)==(tcb[i].semaphore))                              // If the Tasks Share a Common Variable
                           {
                               if ((tcb[i].currentPriority)>(tcb[taskCurrent].currentPriority))               // Check the Priority
                               {
                                   tcb[i].currentPriority=tcb[taskCurrent].currentPriority;                   // if the Priority is less the Elevate the Priority to the Calling Function
                               }
                           }
                          }
                          }
                          NVIC_INT_CTRL_R |= 0x10000000;                                                        /// Set the Pendsv Bit to Call the Pend SV
                       }

           break;
       }
    case 21:    ////CREATE THREAD TASK
    {
        uint8_t k=0;
        __asm("  MOV R0,R4");                                        // Retrive the fn value argument passed in the Create Thread
          fn=(_fn) PutR0_func();                                     //
        __asm("  MOV R0,R5");                                        // Retrive the value of pointer to the name field
          name=PutR0_str();                                             //
        __asm(" MOV R0,R6");                                        // Retrive the value of priority
       priority=PutR0_uint();
       bool ok = false;
        uint8_t i = 0;
        bool found = false;
        // REQUIRED: store the thread name
        // add task if room in task list
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
            if(kill_flag)
                {
                i=inc3; /// flag to handle kill and create thread
                }
            if((strcmp("Flash4Hz",name)==0)&(priority==0)) i=2;          /// This line of Code is to handle the Task Count Value and the Position for correct operation
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            tcb[i].sp = &stack[i][255];
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            tcb[i].skipcount=0;
            strcpy(tcb[i].name,name);      /// Thread name is stored
            // increment task count
            taskCount++;
            kill_flag=0; ///variable to handle kill and create thread functions
            ok = true;
          }
        }
        for(k=0;k<semaphoreCount;k++)                                 /// This part of Code is to check for a common semaphore task is Created and the task that should use the semaphore
        {
            if(tcb[i].semaphore==&semaphores[k])                      /// Retrive the Semaphore used by the Task that is Being Killed
            {
                for(j=0;j<taskCount;j++)
                {
                    if ((tcb[i].semaphore==tcb[j].semaphore)&(i!=j)&(tcb[j].state==STATE_INVALID))       /// Scan throught the table of threads and match the semaphore(single Common Semaphore) and check whether the state is invalid
                    {
                if(semaphores[k].count==0)       // if the Count is Zero but the Task should run intially after Creation
                    {
                    semaphores[k].count++;       // Increment the count ie post during the Creation of Thread which shares a common semaphore
                    break;
                    }
                    }
                    else if((tcb[i].semaphore!=tcb[j].semaphore)&(i!=j)&(tcb[j].state==STATE_INVALID)&(tcb[j].semaphore!=resource)&(tcb[i].semaphore!=resource)&(tcb[i].semaphore==keyPressed)|(tcb[i].semaphore==flashReq))       /// If the Thread doesnot share a common Semaphore( semaphore other than resource )i.e for Keypressed and Keyreleased and FlashReq for Oneshot
                    {
                         if(semaphores[k].count==0)       // if the Count is Zero but the Task should run intially after Creation
                            {
                            semaphores[k].count++;       // Increment the count ie post during the Creation of Thread which shares a common semaphore
                            break;
                            }
                     }
                }
            }
        }
        for(k=0;k<taskCount;k++)                     //// This Part of Code handles the task Count value during the creation of thread and Deletion
             {
                if(strcmp(tcb[k].name,"\0")==0)        // We utilize the name of the thread to increment or decrement the task count properly
                    taskCount--;
             }
        // REQUIRED: allow tasks switches again
        return_ok(ok);
        break;
    }
    case 22:              ///CPUTIME CALCULATION
    {
       uint32_t int_part=0;                           /// Local Variables to be used in Cpu time Calculation
       double frac_part=0;                              //
       uint8_t inc=0;                                   //
       total_time=0;                                    //
       putsUart0("\n \r");
       for(inc=0;inc<taskCount;inc++)             // Calculate the Total time is aggreate of all the task times
       {
           tcb[inc].cpu_time=0;
        total_time+=tcb[inc].time;                  // Total Time Calculation of all the threads
       }
       for(inc=0;inc<taskCount;inc++)              // Process Status Display which includes name, process id, cpu% , state
       {
               putsUart0(" TASK NAME=");           // To display Name
               putsUart0(tcb[inc].name);
               putsUart0("\t");
               putsUart0("Process id=");          // To dispay the Process id
               ltoa(tcb[inc].pid,disp);           // Integer to Array Conversion Built in function
               putsUart0(disp);
               putsUart0("\t");
               tcb[inc].past_time=((alpha*tcb[inc].time)+((1-alpha)*tcb[inc].past_time));       //Cpu time calculation Using the IIR filter using the 1% of the past value
               tcb[inc].cpu_time=(tcb[inc].past_time*100/total_time);                         // Cpu time
               int_part=(int)tcb[inc].cpu_time;                            /// Code to display the cpu % in decimal by seperating the integer and fractional part
               frac_part=tcb[inc].cpu_time-int_part;                       // Fractional Part of the cpu time
               frac_part*=10000;                                           // converting to decimal
               putsUart0(" CPU %=");                                       // Cpu percent Display
               ltoa(int_part,disp);                                        // Conversion of Integer to Array
               putsUart0(disp);                                             //
               putsUart0(".");                                              //
               ltoa(frac_part,disp);                                        //
               putsUart0(disp);                                             //
               putsUart0("\t");                                           //
               if (tcb[inc].state==0) putsUart0("STATE=INVALID");               // Thread State Display
               if (tcb[inc].state==1) putsUart0("STATE=UNRUN");                 //
               if (tcb[inc].state==2) putsUart0("STATE=READY");                 // Based on the State value of the Thread
               if (tcb[inc].state==3) putsUart0("STATE=BLOCKED");               //
               if (tcb[inc].state==4) putsUart0("STATE=DELAYED");               //
               putsUart0("\n \t \r");
       }
       break;
    }
    }

}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
  // REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
  //           5 pushbuttons, and uart
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
     SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

     // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
     SYSCTL_GPIOHBCTL_R = 0;

     // Enable GPIO port F peripherals
     SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA |SYSCTL_RCGC2_GPIOB |SYSCTL_RCGC2_GPIOF ;

     // Configure LED and pushbutton pins
     GPIO_PORTA_DIR_R = 0xE0;  // bits 1 and 3 are outputs, other pins are inputs
     GPIO_PORTA_DR2R_R = 0xE0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
     GPIO_PORTA_DEN_R = 0xFC;  // enable LEDs and pushbuttons
     GPIO_PORTA_PUR_R = 0x1C;  // enable internal pull-up for push button
     GPIO_PORTB_DIR_R = 0x10;  // bits 1 and 3 are outputs, other pins are inputs
     GPIO_PORTB_DR2R_R = 0x10; // set drive strength to 2mA (not needed since default configuration -- for clarity)
     GPIO_PORTB_DEN_R = 0x50;  // enable LEDs and pushbuttons
     GPIO_PORTB_PUR_R = 0x40;  // enable internal pull-up for push button
     GPIO_PORTF_DIR_R = 0x04;  // bits 1 and 3 are outputs, other pins are inputs
     GPIO_PORTF_DR2R_R = 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
     GPIO_PORTF_DEN_R = 0x14;  // enable LEDs and pushbuttons
     GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button


     // Configure UART0 pins
         SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
         GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
         GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
         GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

         // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
         UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
         UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
         UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
         UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
         UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
         UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

        // Configure Timer 1 as the time base for interrupts
                  SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
                  TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
                  TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
                  TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
                  TIMER1_TAILR_R = 0xFFFFFFFF;
                  TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
                  NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
                  TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
	                                            // Approx clocks per us
  __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
  __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
  __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
  __asm("             NOP");                  // 5
  __asm("             B    WMS_LOOP1");       // 5*3
  __asm("WMS_DONE1:   SUB  R0, #1");          // 1
  __asm("             CBZ  R0, WMS_DONE0");   // 1
  __asm("             B    WMS_LOOP0");       // 1*3
  __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
    temp=0;
            if(!PB0 & PB1 & PB2 & PB3 & PB4)      /// Check in PB0 is Pressed
                {
                temp=1;
                }
            if(PB0 & !PB1 & PB2 & PB3 & PB4)        /// Check in PB1 is Pressed
                {
                temp=2;
                }
            if(PB0 & PB1 & !PB2 & PB3 & PB4)        /// Check in PB2 is Pressed
                   {
                       temp=4;
                   }
            if(PB0 & PB1 & PB2 & !PB3 & PB4)        /// Check in PB3 is Pressed
                   {
                       temp=8;
                   }
            if(PB0 & PB1 & PB2 & PB3 & !PB4)        /// Check in PB4 is Pressed
                   {
                       temp=16;
                   }
  return temp;
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
void idle2()
{
  while(true)
  {
    YELLOW_LED = 1;
    waitMicrosecond(1000);
    YELLOW_LED = 0;
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
  waitMicrosecond(1000);
  // give another process a chance to run
  yield();
}

void lengthyFn()
{
  uint16_t i;
  while(true)
  {
    wait(resource);
    for (i = 0; i < 4000; i++)
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
      createThread(flash4Hz, "Flash4Hz", 0);
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

void shell()
{
  while (true)                                  //////// PROGRAM FROM THE PREVIOUS YEARS STRING ACCEPTANCE CODE
  {
    // REQUIRED: add processing for the shell commands through the UART here
            count=0;
            // step 3 variables to be cleared when starting a new string
            memset(str,0,l);                                 // Clear the String to be used
            memset(position,0,l);                                // Clear the String to be used
            memset(type,0,l);                                    // Clear the String to be used
            uint8_t x=0;
            field=0;
            // step 4 variables to be cleared when starting a new string
            valid=0;
            while(1)
            {
     rxinput:      c = getcUart0();                     // Get the String
                   if (c <= 32)
                   {
                    if(c==8)            /// If input is a Backspace then backspace the previous input
                    {

                                     if(count==0) goto rxinput;
                                        else
                                        {
                                            count=count-1;
                                            goto rxinput;
                                        }

                    }
                    else if (c==32)
                    {
                        str[count+1]=32;
                    }
                        else if (c==13)
                    {
                        str[count++]='\0';
                        l=strlen(str);                              // Store the Input and Process the input to find position type and field (** Important i have removed '&' from the delimiter so <process name> & works
                        for (y=0; y <= l ;y++)
                                                   {
                                                       while (((str[y]>0) && (str[y]<38))||((str[y]>38) && (str[y]<48))|| ((str[y]>57) && (str[y]<65))||((str[y]>90) && (str[y]<97))||((str[y]>123) && (str[y]<127)))
                                                               {
                                                           str[y]=0;
                                                               }



                                                       if ((str[y]>=48) && (str[y]<=57))
                                                       {
                                                           position[x]=y;
                                                           type[x]='n';
                                                           x=x+1;
                                                           field=x;
                                                       while(((str[y]>=48) && (str[y]<=57)) && ((str[y+1]>=48) && (str[y+1]<=57))) y=y+1;
                                                       }

                                                       if ((str[y]>=65) && (str[y]<=90))
                                                       {
                                                           position[x]=y;
                                                           type[x]='a';
                                                           x=x+1;
                                                           field=x;
                                                          while(((str[y]>=65) && (str[y]<=90)) && ((str[y+1]>=65) && (str[y+1]<=90)))y=y+1;
                                                       }


                                                           if ((str[y]>=97) && (str[y]<=122))
                                                           {
                                                               position[x]=y;
                                                               type[x]='a';
                                                                x=x+1;
                                                               field=x;
                                                           while(((str[y]>=97) && (str[y]<=122)) && ((str[y+1]>=97) && (str[y+1]<=122))) y=y+1;
                                                           }


                    }

             ////////processing Process Status
                        if ((iscommand("ps",0)))
                               {
                            __asm("  SVC #0x16");         // SVC PROCESS STATUS COMMAND
                                    valid=1;
                               }
                        ////////processing IPCS
                                 if ((iscommand("ipcs",0)))        // processing IPCS
                                    {
                                     uint8_t inc1=0;
                                     uint8_t z=0;
                                     putsUart0("\n \r");
                                     for(inc1=0;inc1<semaphoreCount;inc1++)        /// Table to display the name of Semaphore
                                        {
                                         if (inc1==0)
                                         {
                                         putsUart0(" Semaphore= KeyPressed");
                                         putsUart0("\t");
                                         }
                                         if (inc1==1)
                                         {
                                         putsUart0(" Semaphore= KeyReleased");
                                         putsUart0("\t");
                                         }
                                         if (inc1==2)
                                         {
                                         putsUart0(" Semaphore= Flashreq");
                                         putsUart0("\t");
                                         }
                                         if (inc1==3)
                                         {
                                         putsUart0(" Semaphore= Resource");
                                         putsUart0("\t");
                                         }
                                         putsUart0(" Count=");                       /// Table to display the Count from the Semaphore Queue
                                         ltoa(semaphores[inc1].count,disp);
                                         putsUart0(disp);
                                         putsUart0("\t");
                                         putsUart0("Queue Size=");                  //// Table to Display the Queue Size
                                         ltoa(semaphores[inc1].queueSize,disp);
                                         putsUart0(disp);
                                         putsUart0("\t");
                                         putsUart0("Process id=");
                                         for(z=0;z<semaphores[inc1].queueSize;z++)        /// Code to Display the Process Queue
                                            {
                                             ltoa(semaphores[inc1].processQueue[z],disp);
                                             putsUart0(disp);
                                             putsUart0("\t");
                                            }
                                         putsUart0("\n \t \r");
                                        }
                                      valid=1;
                                    }
                                 if ((iscommand("kill",1)))                 /// Kill Command
                                   {
                                     putsUart0("\n \r");
                                     if(type[1]=='n')    /// If the Input field is Number the Accept the data
                                     {
                                     destroyThread(atoi(&str[position[1]]));
                                     putsUart0("\n \r Thread Killed \n \r ");
                                     valid=1;
                                     }
                                   }
                                 if ((iscommand("reboot",0)))        /// Reboot Command
                                    {
                                     putsUart0("\n \r");
                                     HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
                                    }
                                 if ((iscommand("pi",1)))          /// Code to Turn on and Off the Priority Inheritance
                                     {
                                      putsUart0("\n \r");
                                      if(strcmp("on",&str[position[1]])==0)
                                      {
                                          pi_flag=1;
                                          putsUart0("\n \r Priority Inheritance is On\n\r");
                                      }
                                      else if(strcmp("off",&str[position[1]])==0)
                                      {
                                          pi_flag=0;
                                          putsUart0("\n \r Priority Inheritance is Off\n\r");
                                      }
                                      putsUart0("\n \r");
                                      valid=1;
                                     }
                                 if ((iscommand("preempt",1)))            // Code to Turn On and Off the Preemption
                                      {
                                       putsUart0("\n \r");
                                       if(strcmp("on",&str[position[1]])==0)
                                       {
                                           pre_empt=1;
                                           putsUart0("\n \r Preemption is On\n\r");
                                       }
                                       else if(strcmp("off",&str[position[1]])==0)
                                       {
                                           pre_empt=0;
                                           putsUart0("\n \r Preemption is Off\n\r");
                                       }
                                       putsUart0("\n \r");
                                       valid=1;
                                      }
                                 if ((iscommand("pidof",1)))               /// Code to display the Pid number of a Thread
                                     {
                                     uint8_t inc2=0;
                                     uint8_t z=0;
                                     char display[16]={};
                                     putsUart0("\n \r");
                                     if (type[1]=='a')
                                     {
                                     for(inc2=0;inc2<taskCount;inc2++)
                                     {
                                         memset(display,0,strlen(display));
                                         for(z=0;z<strlen(tcb[inc2].name);z++)             ///// Store the name to the local Array
                                         {
                                             if((tcb[inc2].name[z]>=65)&(tcb[inc2].name[z]<=90))
                                              display[z]=tcb[inc2].name[z]+32;
                                             else
                                                 display[z]=tcb[inc2].name[z];
                                         }
                                         if(strcmp(display,&str[position[1]])==0)
                                         {
                                             ltoa(tcb[inc2].pid,disp);
                                             putsUart0(disp);
                                             putsUart0("\n \t \r");
                                             valid=1;
                                             break;
                                         }
                                       }
                                     }
                                     }
                                 else                     //// Code to Create Thread by giving the input
                                 {
                                     _fn fn;
                                     uint8_t z=0;
                                     char display[16]={};
                                     putsUart0("\n \r");
                                     for(inc3=0;inc3<taskCount;inc3++)               ////// Access the Tasks
                                     {
                                         memset(display,0,strlen(display));
                                         for(z=0;z<strlen(tcb[inc3].name);z++)             // Store the String to the Local Array
                                         {
                                             if((tcb[inc3].name[z]>=65)&(tcb[inc3].name[z]<=90))
                                              display[z]=tcb[inc3].name[z]+32;
                                             else
                                                 display[z]=tcb[inc3].name[z];
                                         }
                                         display[z]='&';                                // Concatenate the '&'
                                         if(strcmp(display,&str[position[0]])==0)
                                         {
                                         if (tcb[inc3].state==STATE_INVALID)        // Check the State is Invalid and Create the Thread
                                         {
                                             kill_flag=1;
                                         if((strcmp("idle&",&str[position[0]])==0)&(strcmp(display,&str[position[0]])==0)) createThread(idle, "Idle", 7);                       // Code to Create Idle
                                         if((strcmp("lengthyfn&",&str[position[0]])==0)&(strcmp(display,&str[position[0]])==0)) createThread(lengthyFn, "LengthyFn", 6);        // Code to Create Lengthy fn
                                         if((strcmp("flash4hz&",&str[position[0]])==0)&(strcmp(display,&str[position[0]])==0))   createThread(flash4Hz, "Flash4Hz", 2);         // Code to Create flash4hz
                                         if((strcmp("oneshot&",&str[position[0]])==0)&(strcmp(display,&str[position[0]])==0))  createThread(oneshot, "OneShot", 2);             // Code to create Oneshot
                                         if((strcmp("debounce&",&str[position[0]])==0)&(strcmp(display,&str[position[0]])==0))  createThread(debounce, "Debounce", 6);          // Code to Create Debounce
                                         if((strcmp("readkeys&",&str[position[0]])==0)&(strcmp(display,&str[position[0]])==0))  createThread(readKeys, "ReadKeys", 6);          // Code to Create Readkeys
                                         if((strcmp("important&",&str[position[0]])==0)&(strcmp(display,&str[position[0]])==0))  createThread(important, "Important", 0);       // Code to Create Important
                                         if((strcmp("uncoop&",&str[position[0]])==0)&(strcmp(display,&str[position[0]])==0)) createThread(uncooperative, "Uncoop", 5);          // Code to Create Uncoop
                                         if((strcmp("shell&",&str[position[0]])==0)&(strcmp(display,&str[position[0]])==0)) createThread(shell, "Shell", 4);                    // Code to Create Shell
                                             putsUart0(tcb[inc3].name);
                                             putsUart0(" ");
                                             putsUart0("Thread is made Ready");
                                             }
                                             else
                                                 putsUart0("Thread is Already made Ready");
                                             putsUart0("\n \t \r");
                                             valid=1;
                                             break;
                                         }
                                     }
                                 }


                    break;
                         }

          else
             {
               putsUart0("\r\nInvalid Input\r\n");   /// unwanted character print invalid
               break;
            }
            }
                           if (count>max)
                          {
                              str[count++]='\0';
                              putsUart0("\r\n Wordlength Reached\r\n");
                              break;
                          }

                          else
                          {
                              if ((c>=65)&&(c<=90)) c=c+32;             //////////code to make the input insensitive
                              str[count++]=c;
                          }

           }      //// end of the continuous while loop to recieve the data


                if (valid==1);                   //////// IF NOT VALID THEN  ERROR IS DEFINED
                else
                   {
                    putsUart0("\n\rError\r\n");
                   }

}
}
//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------
  // Blocking function that writes a serial character when the UART buffer is not full
  void putcUart0(char c)
  {
      while (UART0_FR_R & UART_FR_TXFF);
      UART0_DR_R = c;
  }

  // Blocking function that writes a string when the UART buffer is not full
  void putsUart0(char* string)
  {
      uint8_t i;
      for (i = 0; i < strlen(string); i++)
        putcUart0(string[i]);
  }

  // Blocking function that returns with serial data once the buffer is not empty
  char getcUart0()
  {
      while (UART0_FR_R & UART_FR_RXFE)
      {
      yield();
      }
      return UART0_DR_R & 0xFF;
  }

  int iscommand(char* strcmd, uint8_t minargs )      ///////// To compare the Entered String with Predefined Commands
  {
    if((strcmp(&str[position[0]],strcmd)==0) && ((field-1) >= minargs))       /////change to (field-1) >= minargs) if to take more than minagrs as arguments
    {
        return 1;
    }
    else return 0;
  }
void  Timer1Isr()
{
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
  bool ok;

  // Initialize hardware
  initHw();
  rtosInit();

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

  // Add required idle process
  ok =  createThread(idle, "Idle", 7);


  // Add other processes
 // ok &=  createThread(idle2, "Idle2", 0);
   ok &= createThread(lengthyFn, "LengthyFn", 6);
   ok &= createThread(flash4Hz, "Flash4Hz", 2);
  ok &= createThread(oneshot, "OneShot", 2);
  ok &= createThread(readKeys, "ReadKeys", 6);
   ok &= createThread(debounce, "Debounce", 6);
    ok &= createThread(important, "Important", 0);
   ok &= createThread(uncooperative, "Uncoop", 5);
  ok &= createThread(shell, "Shell", 4);


  // Start up RTOS
  if (ok)
    rtosStart(); // never returns
  else
    RED_LED = 1;

return 0;
}
