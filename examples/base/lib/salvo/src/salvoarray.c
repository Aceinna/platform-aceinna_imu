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
places(TM). Copyright (C) 1995-2008 Pumpkin, Inc. and its
Licensor(s). All Rights Reserved.
 
$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\Src\\salvoarray.c,v $
$Author: aek $
$Revision: 3.6 $
$Date: 2008-04-27 14:45:46-07 $

Array functions

************************************************************/

#include <salvo.h>

#if OSUSE_ARRAYS

/************************************************************ 
****                                                     ****
**                                                         **
OSRtnTcbPfmA()

Return tcb pointer for the highest-priority task in the
priority array.

NOTE: OSRtnTcbPfmA deals with the issue of empty arrays vs.
taskID (starting at 0) by explicitly returning a null pointer
only if the array itself is empty.

Returns:     OSNOERR (0) if priority array is empty
            ptr to tcb if priority array is non-empty
**                                                         **
****                                                     ****
************************************************************/
OStypeTcbP OSRtnTcbPfmA( OStypePrioA array )
{
    OStypeID tID;

    
    if ( array ) {
    
//#if OSOPTIMIZE_FOR_SPEED
    
        #if OSARRAY_SIZE_IS_WORD
        if ( array & 0x8000 ) tID = 15;
        if ( array & 0x4000 ) tID = 14;
        if ( array & 0x2000 ) tID = 13;
        if ( array & 0x1000 ) tID = 12;
        if ( array & 0x0800 ) tID = 11;
        if ( array & 0x0400 ) tID = 10;
        if ( array & 0x0200 ) tID = 9;
        if ( array & 0x0100 ) tID = 8;
        #endif
        
        if ( array & 0x0080 ) tID = 7;
        if ( array & 0x0040 ) tID = 6; 
        if ( array & 0x0020 ) tID = 5;
        if ( array & 0x0010 ) tID = 4;
        if ( array & 0x0008 ) tID = 3;
        if ( array & 0x0004 ) tID = 2;
        if ( array & 0x0002 ) tID = 1;
        if ( array & 0x0001 ) tID = 0;
        
//#else

//#define TABLE 0xFFFFAA50
//    array <<= 1;
//    tID = ((TABLE >> array) & 0x3);    

//#endif
        
        /* careful! don't want to make this a compile-    */
        /*  time constant!.                                */
    
        return OSTCBP(tID);

    }

    return (OStypeTcbP) 0;
}    


/* this code snippet was in OSInit() before I realized    */
/*  that explicit initialization is a real problem and is */
/*  also unnnecessary.                                    */

#if 0
     #if OSENABLE_TASKS
    /* initialize tcbs ...    */
    for ( tID = OSTASKS ; tID-- ; )    {
      #if OSUSE_ARRAYS
      /* arrays required the tID in the prio field,    */
      /*  and if speed-optimized, the single-1 bit    */
      /*  pattern in the prioABits field.            */
        OSInitTcb(tcbP);
        tcbP->status.bits.prio = tID;
        #if OSOPTIMIZE_FOR_SPEED
        tcbP->prioABits = OSBits[tID];
        #endif
        tcbP++;
      #else /* #if OSUSE_ARRAYS */
      /* tcb initialization for queues amounts to    */
      /*  just zeroing everything ...                */
        OSInitTcb(tcbP++);
      #endif /* #if OSUSE_ARRAYS */
      }
     #endif /* #if OSENABLE_TASKS */
#endif


#endif /* #if OSUSE_ARRAYS */

