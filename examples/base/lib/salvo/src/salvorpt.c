/************************************************************
The contents of this file are subject to the Pumpkin Salvo
License (the "License").  You may not use this file except
in compliance with the License. You may obtain a copy of
the License at http://www.pumpkininc.com, or from
warranty@pumpkininc.com.

Software distributed under the License is distributed on an
"AS IS" basis, WITHOUT WARRANTY OF ANY KIND, either express
or implied. See the License for specific language governing
the warranty and the rights and limitations under the
License.

The Original Code is Salvo - The RTOS that runs in tiny
places(TM). Copyright (C) 1995-2006 Pumpkin, Inc. and its
Licensor(s). All Rights Reserved.

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvorpt.c,v $
$Author: aek $
$Revision: 3.26 $
$Date: 2008-04-27 16:23:53-07 $

Function to generate a human-readable "status report" of
the tasks, events, etc.

************************************************************/

/* Temporarily modified to work (better) with Keil C51! */
/*  aek 6/27/2002 */

#include <salvo.h>


/* stack checking is suppressed in this module */
#if 1
#undef OSIncCallDepth
#undef OSDecCallDepth
#define OSIncCallDepth()
#define OSDecCallDepth()
#endif

#if	OSUSE_PRINTF_P_FORMAT
#define	OSPTR_FORMAT		"  %08p",		
#else
#define	OSPTR_FORMAT		" %08Xh", (unsigned) // works for 8- and 16-bit targets	
#endif	

#define CRLF                 "\n\r"
#define FIXED_WIDTH           1
#define VARIABLE_WIDTH        0

#if !defined(max)
#define max(a,b)    ((a) >= (b) ? (a) : (b))
#endif

#if !defined(min)
#define min(a,b)    ((a) <= (b) ? (a) : (b))
#endif

void OSPrintTcbBanner( void );
void OSPrintEcbBanner( void );

char strOSCtxSwsWIdle[] = "CtxSws, total=idle+eligible:";
char strOSCtxSws[]      = "CtxSws:";

/************************************************************
****                                                     ****
**                                                         **
Dprintf()

Adapted from a posting to comp.lang.c by Michael Wojcik
on April 24, 1995.

**                                                         **
****                                                     ****
************************************************************/
#if OSENABLE_DEBUGGING_PRINTFS
#define DPRINTF(x) Dprintf x
#else
#define DPRINTF(x)
#endif

#if OSENABLE_DEBUGGING_PRINTFS
#include <stdarg.h>
static void Dprintf(const char *Format, ...)
{
   va_list Arguments;
   char buf[256];

   /* format the output */
   va_start(Arguments, Format);
   vsprintf(buf, Format, Arguments);
   va_end(Arguments);

   /* output the data */
   printf(buf);

   return;
}
#endif

void TestFn(void);

void TestFn(void)
{
    DPRINTF(("%d\n", 27));
}

/************************************************************
****                                                     ****
**                                                         **
OSPrintTcbP(tasks, tcbP, flag)

Given a pointer to a tcb, display the tcb's number
(usually the same as the tID), a code for the NULL pointer,
or show just a blank to indicate that it's invalid or
unknown (this happens when pointers are left as orphans,
etc.).

**                                                         **
****                                                     ****
************************************************************/
void OSPrintTcbP( OStypeID      tasks,
                  OStypeTcbP    tcbP,
                  OStypeBoolean fixedWidth )
{
    OStypeID i;


    OSIncCallDepth();

    /* NULL condition happens a lot ...                 */
    if ( tcbP == 0 ) {
        DPRINTF(("  ."));
    }
    /* see if the pointer points to a task ...             */
    else {
        i = tasks;
        do
        {
            if ( tcbP == OSTCBP(i) )
            {
                if ( fixedWidth )
                {
                    DPRINTF(("t%2u", (unsigned) i));
                }
                else
                {
                    DPRINTF(("t%u", (unsigned) i));
                }

                OSDecCallDepth();
                return;
            }
        } while ( --i );

        /* nope, it's a bad pointer. */
        DPRINTF(("   "));
    }

    OSDecCallDepth();
}


#if  OSENABLE_EVENTS
void OSPrintEcbP( OStypeID      events,
                  OStypeEcbP    ecbP,
                  OStypeBoolean fixedWidth )
{
    OStypeID i;


    OSIncCallDepth();

    if ( ecbP == 0 )
    {
        DPRINTF(("  ."));
    }
    else
    {
        i = events;
        do
        {
            if ( ecbP == OSECBP(i) )
            {
                if ( fixedWidth )
                {
                    DPRINTF(("e%2u", (unsigned) i));
                }
                else
                {
                    DPRINTF(("e%u", (unsigned) i));
                }

                OSDecCallDepth();
                return;
            }
        } while ( --i );

        DPRINTF(("   "));
    }

    OSDecCallDepth();
}
#endif


/************************************************************
****                                                     ****
**                                                         **
OSPrintTcb(tasks, events, tcbP)

Display the fields of the specified tcb.

**                                                         **
****                                                     ****
************************************************************/
void OSPrintTcb( OStypeID   tasks,
                 OStypeID   events,
                 OStypeTcbP tcbP )
{
    OStypeState state;

    OSIncCallDepth();

    /* Help the compiler (esp ez8cc, which chokes on	*/
    /*  switch(tcbP->status.bits.state)) by saving    	*/
    /*  this bitfield  locally in <state>.            	*/
    state = tcbP->status.bits.state;


    /* show current task with an '*' after it.        	*/
    if (OSTaskRunning(tcbP))
    {
        DPRINTF(("* "));
    }
    else
    {
        DPRINTF(("  "));
    }


    /* task status (only 8 allowed values).			  	*/
    switch ( state ) {
        case OSTCB_DESTROYED:
            DPRINTF(("dstr"));
            break;
        case OSTCB_TASK_STOPPED:
            DPRINTF(("stop"));
            break;
        case OSTCB_TASK_DELAYED:
            DPRINTF(("dlyd"));
            break;
        case OSTCB_TASK_SIGNALED:
            DPRINTF(("sig "));
            break;
        case OSTCB_TASK_WAITING:
            DPRINTF(("wait"));
            break;
        case OSTCB_TASK_ELIGIBLE:
            DPRINTF(("elig"));
            break;
        case OSTCB_TASK_TIMED_OUT:
            DPRINTF((" to "));
            break;
        case OSTCB_CYCLIC_TIMER:
            DPRINTF(("undf"));
            break;
        default:
            break;
    }

    /* raw dump of tcb contents.                    	*/
    /* task priority.                                	*/
    #if !OSRPT_HIDE_INVALID_POINTERS
    DPRINTF((" %3u", (unsigned) tcbP->status.bits.prio));

    /* task start/resume address.                    	*/
    DPRINTF((OSPTR_FORMAT tcbP->tFP));

    /* task ID of next task in priority queue.        	*/
    OSPrintTcbP(tasks, tcbP->u2.nextTcbP, 1);

    /* event ID.                                    	*/
    DPRINTF(("  "));
    OSPrintEcbP(events, tcbP->u1.ecbP, 1);

    /* delay value.                                    */
    #if   OSBYTES_OF_DELAYS == 1
    DPRINTF(("  %3u   ", (unsigned) tcbP->dly.delay));
    #elif OSBYTES_OF_DELAYS == 2
    DPRINTF((" %5u  ",   (unsigned) tcbP->dly.delay));
    #elif OSBYTES_OF_DELAYS == 3
    DPRINTF((" %8u  ",   (unsigned) tcbP->dly.delay));
    #elif OSBYTES_OF_DELAYS == 4
    DPRINTF((" %11u  ",  (unsigned) tcbP->dly.delay));
    #endif

    /* task ID of next task in delay queue.            */
    OSPrintTcbP(tasks, tcbP->nextDlyTcbP, 1);

    /* more readable dump of tcb contents.            */
    #else
    /* show start/resume address and priority only     */
    /*  if active.                                    */
    if (state != OSTCB_DESTROYED)
    {
        DPRINTF((" %3u", (unsigned) tcbP->status.bits.prio));
        DPRINTF((OSPTR_FORMAT tcbP->u3.tFP));

        DPRINTF(("  "));
        OSPrintTcbP(tasks, tcbP->u2.nextTcbP, FIXED_WIDTH);

        /* event ID */
        DPRINTF(("  "));
        if ( state == OSTCB_TASK_ELIGIBLE )
        {
            DPRINTF(("n/a"));
        }
        else
        {
            #if OSENABLE_EVENTS && OSENABLE_TIMEOUTS
            OSPrintEcbP(events, tcbP->u1.ecbP, FIXED_WIDTH);
            #endif
        }
    }

    /* waiting delay and pointers */
    switch (state) {
        case OSTCB_TASK_DELAYED:
        case OSTCB_TASK_WAITING:
            #if OSENABLE_TIMEOUTS
            DPRINTF(("  "));
            OSPrintTcbP(tasks, tcbP->nextDlyTcbP, FIXED_WIDTH);
            #endif

            #if OSENABLE_DELAYS || OSENABLE_TIMEOUTS
            #if   OSBYTES_OF_DELAYS == 1
            DPRINTF(("  %3u   ", (unsigned) tcbP->dly.delay));
            #elif OSBYTES_OF_DELAYS == 2
            DPRINTF((" %5u  ",   (unsigned) tcbP->dly.delay));
            #elif OSBYTES_OF_DELAYS == 3
            DPRINTF((" %8u  ",   (unsigned) tcbP->dly.delay));
            #elif OSBYTES_OF_DELAYS == 4
            DPRINTF((" %11u  ",  (unsigned) tcbP->dly.delay));
            #endif
            #endif
            break;

        default:
            break;
    }
    #endif /* #if !OSRPT_HIDE_INVALID_POINTERS */

    DPRINTF((CRLF));

    OSDecCallDepth();
}


/************************************************************
****                                                     ****
**                                                         **
OSPrintEcb(tasks, events, ecbP)

Display the fields of the specified ecb.

**                                                         **
****                                                     ****
************************************************************/
#if OSENABLE_EVENTS
void OSPrintEcb( OStypeID   tasks,
                 OStypeEcbP ecbP )
{
    OStypeTcbP tcbP;


    OSIncCallDepth();

    /* separate whatever came before (e.g. eventID)     */
    /*  from the rest.                                    */
    DPRINTF(("  "));

    /* local handle.                                    */
    tcbP = ecbP->tcbP;

    /* event type and value (format depends on type).     */
    #if OSUSE_EVENT_TYPES
    switch (ecbP->type) {
        case OSEV_DESTROYED:
            DPRINTF(("dstr"));
            break;

        #if OSENABLE_BINARY_SEMAPHORES
        case OSEV_BINSEM:
            DPRINTF(("BSem "));
            OSPrintTcbP(tasks, tcbP, FIXED_WIDTH);
            DPRINTF((" %8d", (int) ecbP->event.binSem));
            break;
        #endif

        #if OSENABLE_EVENT_FLAGS
        case OSEV_EFLAG:
            DPRINTF(("EFlg "));
            OSPrintTcbP(tasks, tcbP, FIXED_WIDTH);
            DPRINTF((" %08Xh", (int) ecbP->event.efcbP->eFlag));
            break;
        #endif

        #if OSENABLE_SEMAPHORES
        case OSEV_SEM:
            DPRINTF((" Sem "));
            OSPrintTcbP(tasks, tcbP, FIXED_WIDTH);
            DPRINTF((" %8d",  (int) ecbP->event.sem));
            break;
        #endif

        #if OSENABLE_MESSAGES
        case OSEV_MSG:
            DPRINTF((" Msg "));
            OSPrintTcbP(tasks, tcbP, FIXED_WIDTH);
            DPRINTF((OSPTR_FORMAT ecbP->event.msgP));
            break;
        #endif

        #if OSENABLE_MESSAGE_QUEUES
        case OSEV_MSGQ:
            DPRINTF(("MsgQ "));
            OSPrintTcbP(tasks, tcbP, FIXED_WIDTH);
            DPRINTF((OSPTR_FORMAT *(ecbP->event.mqcbP->outPP)));
            DPRINTF(("  count: %d", (int) ecbP->event.mqcbP->count));
            break;
        #endif

        default:
            DPRINTF((" ERR "));
            break;
    }


    /* raw display of event type and value.                */
    #else

    /* N.B. this line causes an error when generating a library with OSUSE_EVENT_TYPES set to FALSE */
    #if OSRPT_HIDE_INVALID_POINTERS
    if ( ecbP->type != OSEV_DESTROYED ) {
    #endif

        OSPrintTcbP(tasks, ecbP->tcbP, FIXED_WIDTH);
        DPRINTF((" %4xh", ecbP->event.sem));

    #if OSRPT_HIDE_INVALID_POINTERS
    }
    #endif

    #endif /* #if OSUSE_EVENT_TYPES */

    DPRINTF((CRLF));

    OSDecCallDepth();
}
#endif


/************************************************************
****                                                     ****
**                                                         **
OSPrintEcbBanner()
OSPrintTcbBanner()

Simple banners to be placed above ecb/tcb display.

**                                                         **
****                                                     ****
************************************************************/
void OSPrintTcbBanner( void )
{
    /* banner for tasks.                                */
    DPRINTF((CRLF "task stat prio    addr   t->  e->"));

    #if OSENABLE_DELAYS && OSENABLE_TIMEOUTS
    DPRINTF(("  d-> delay" CRLF));
    #elif OSENABLE_DELAYS && !OSENABLE_TIMEOUTS
    DPRINTF(("delay" CRLF));
    #else
    DPRINTF((CRLF));
    #endif
}


#if OSENABLE_EVENTS
void OSPrintEcbBanner( void )
{
    /* banner for events.                                */
    #if OSUSE_EVENT_TYPES
    DPRINTF((CRLF "evnt type t->    value" CRLF));
    #else
    DPRINTF((CRLF "evnt t->    value" CRLF));
    #endif
}
#endif


/************************************************************
****                                                     ****
**                                                         **
OSRpt()

Display the status of the system, in a manner suitable for
viewing on a conventional 80-char-wide terminal screen.

**                                                         **
****                                                     ****
************************************************************/
void OSRpt( OStypeID tasks,
            OStypeID events )
{
    union {
        OStypeID    i;

        #if OSENABLE_DELAYS || OSENABLE_TIMEOUTS
        OStypeDelay delay;
        #endif
    } u1;
    OStypeTcbP  tcbP;



    OSIncCallDepth();

    /* version string.                                    */
    DPRINTF((CRLF "Salvo %c.%c.%c  ",
      '0' + OSVER_MAJOR,
      '0' + OSVER_MINOR,
      '0' + OSVER_SUBMINOR));
    /* used stack depth.                                */
    #if OSENABLE_STACK_CHECKING
    DPRINTF(("Max call...rtn stack depth: %u", OSmaxStkDepth));
    #endif

    /* number of context switches.                         */
    #if OSGATHER_STATISTICS && OSENABLE_COUNTS

     #if OSENABLE_IDLE_COUNTER

      #if ( OSBYTES_OF_COUNTS <= 2 )
    DPRINTF((CRLF "%s %u = %u + %u", strOSCtxSwsWIdle,
      OSctxSws+OSidleCtxSws, OSidleCtxSws, OSctxSws));
      #elif ( OSBYTES_OF_COUNTS <= 4 )
    DPRINTF((CRLF "%s %lu = %lu + %lu", strOSCtxSwsWIdle,
      OSctxSws+OSidleCtxSws, OSidleCtxSws, OSctxSws));
      #endif

     #else

      #if ( OSBYTES_OF_COUNTS <= 2 )
    DPRINTF((CRLF "%s %u", strOSCtxSws, OSctxSws));
    #elif ( OSBYTES_OF_COUNTS <= 4 )
    DPRINTF((CRLF "%s %lu", strOSCtxSws, OSctxSws));
    #endif

    #endif

    #endif


    /* display system information.                         */
    #if OSLOGGING && !OSGATHER_STATISTICS
    DPRINTF((CRLF "Errors: %u  Warnings: %u  ",
      OSerrs, OSwarns));
    #endif

    #if !OSLOGGING && OSGATHER_STATISTICS && OSENABLE_TIMEOUTS
    DPRINTF((CRLF "Timeouts: %u  ",
      OStimeouts));
    #endif

    #if OSLOGGING && OSGATHER_STATISTICS && OSENABLE_TIMEOUTS
    DPRINTF((CRLF "Errors: %u  Warnings: %u  Timeouts: %u  ",
      OSerrs, OSwarns, OStimeouts));
    #endif


      /* always show ticks.                                 */
    #if OSENABLE_TICKS
    #if OSBYTES_OF_TICKS <= 2
    DPRINTF(("Ticks: %u", OStimerTicks));
    #else
    DPRINTF(("Ticks: %lu", OStimerTicks));
    #endif
    #endif


    /* show eligible queue. tcbP test prevents runaway    */
    /*  pointers from indefinitely extending this loop.    */
    DPRINTF((CRLF "EligQ: "));
    tcbP = OSeligQP;
    #if !OSENABLE_ERROR_CHECKING
    while ( tcbP ) {
    #else
    while ( ( tcbP ) && ( tcbP <= OSTCBP(tasks) ) ) {
    #endif
        OSPrintTcbP(tasks, tcbP, VARIABLE_WIDTH);
        if ( tcbP->status.bits.state == OSTCB_TASK_TIMED_OUT ) /* ?aek or TASK_WAITING_TO? */
        {
          DPRINTF(("(t)"));
        }

        tcbP = tcbP->u2.nextTcbP;
        if ( tcbP )
        {
            DPRINTF((","));
        }
    }


    /* show delay queue. Compute total delay of tasks    */
    /*  in queue.                                        */
    #if OSENABLE_DELAYS || OSENABLE_TIMEOUTS
    DPRINTF((CRLF "DelayQ: "));
    u1.delay = 0;
    tcbP = OSdelayQP;
    #if !OSENABLE_ERROR_CHECKING
    while ( tcbP ) {
    #else
    while ( ( tcbP ) && ( tcbP <= OSTCBP(tasks) ) ) {
    #endif
        OSPrintTcbP(tasks, tcbP, VARIABLE_WIDTH);
        u1.delay += tcbP->dly.delay;

        #if !OSENABLE_TIMEOUTS
        tcbP = tcbP->u2.nextTcbP;
        #else
        tcbP = tcbP->nextDlyTcbP;
        #endif

        if ( tcbP )
        {
            DPRINTF((","));
        }
    }
    if ( u1.delay )
    {
        DPRINTF(("  Total delay: %u ticks", (int) u1.delay));
    }
    #endif


    /* Now show the actual tcbs.                        */
    OSPrintTcbBanner();
    for ( u1.i = 1 ; u1.i <= tasks ; ++u1.i ) {

        #if OSRPT_SHOW_ONLY_ACTIVE
        if ( (OSTCBP(u1.i))->status.bits.state != OSTCB_DESTROYED ) {
        #endif

            DPRINTF((" %2u", (unsigned) u1.i));
            OSPrintTcb(tasks, events, OSTCBP(u1.i));

        #if OSRPT_SHOW_ONLY_ACTIVE
        }
        #endif
    }


    /* Now show the actual ecbs.                        */
    #if OSENABLE_EVENTS
    OSPrintEcbBanner();
    for ( u1.i = 1 ; u1.i <= events ; ++u1.i ) {
        DPRINTF((" %2u", (unsigned) u1.i));
        OSPrintEcb(tasks, OSECBP(u1.i));
    }
    #endif

    OSDecCallDepth();
}

