/************************************************************ 
Copyright (C) 1995-2008 Pumpkin, Inc. and its
Licensor(s). Freely distributable.

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvoprg.h,v $
$Author: aek $
$Revision: 3.10 $
$Date: 2008-04-27 14:45:20-07 $

This file is used to include / inject compiler-specific 
pragmas where needed in the Salvo code. 
    
See comments in salvomcg.h on why this file doesn't have
include guards.


************************************************************/
#ifndef __SALVOPRG_H
#define __SALVOPRG_H

/************************************************************
****                                                     ****
**                                                         **
MPLAB-C18 requires an explicitly defined segment. Can only
do all-or-nothing -- that's why OSLOC_ALL cannot be used.


ICCAVR users can force Salvo's global objects into a unique
program area called salvoram. When used, specify the start
and end addresses of this program area via:  
          
 	-bsalvoram:0xstart.0xend    
                       
as a linker option. Otherwise the salvoram program area will
simply follow the data program area (default). 

**                                                         **
****                                                     ****
************************************************************/
#if defined(__OSMEM_C) 

	/* prevent Salvo global objects from being  		*/
	/*  initialized. This "lasts" for just the 			*/
	/*  immediately-following object.					*/
	#if   ( OSCOMPILER == OSIAR_ICC ) && ( OSTARGET == OSPIC18 ) 
	
	    #pragma object_attribute=OSIAR_PIC18_ATTR_ALL
	    
	    #if !defined(OSIAR_PIC18_ATTR_ALL)
		    #error salvoprg.h: OSIAR_PIC18_ATTR_ALL undefined.
	    #endif
    
    #endif
    
    /* for MPLAB-C18, force the Salvo vars into the		*/
    /*  access bank, or into unique banks as req'd by 	*/
    /*  the Salvo arrays								*/
    #if (OSCOMPILER == OSMPLAB_C18)
    
    	#if (OSMPLAB_C18_LOC_ALL_NEAR == TRUE) 
    	
			#pragma udata access OSVars
			
		#else
		
			#if   defined(__OSTCBAREA_MEM_C)
			#pragma udata OSVarsTcbArea
			#elif defined(__OSECBAREA_MEM_C)
			#pragma udata OSVarsEcbArea
			#elif defined(__OSEFCBAREA_MEM_C)
			#pragma udata OSVarsEfcbArea
			#elif defined(__OSMQCBAREA_MEM_C)
			#pragma udata OSVarsMqcbArea
			#else
			#pragma udata OSVars
			#endif
			
		#endif
		
	#endif
	
    /* for ICCAVR, place each Salvo global object in	*/
    /*  the <salvoram> data area						*/
	#if ( (OSCOMPILER == OSIMAGECRAFT) && (OSTARGET == OSAVR) )
	
		#if !defined(__OSENDSALVOVARS_MEM_C)
			#pragma data:salvoram 
		#else
			#pragma data:data
		#endif
	 
	#endif  
    
#endif 


#endif /* __SALVOPRG_H */
