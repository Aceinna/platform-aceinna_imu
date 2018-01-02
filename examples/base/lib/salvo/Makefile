###########################################################################################
# The contents of this file are subject to the Pumpkin Salvo
# License (the "License").  You may not use this file except
# in compliance with the License. You may obtain a copy of
# the License at http://www.pumpkininc.com, or from
# warranty@pumpkininc.com.
# 
# Software distributed under the License is distributed on an
# "AS IS" basis, WITHOUT WARRANTY OF ANY KIND, either express
# or implied. See the License for specific language governing
# the warranty and the rights and limitations under the
# License.
# 
# The Original Code is Salvo - the Cooperative, Multitasking
# RTOS. Copyright (C) 1995-2001 Pumpkin, Inc. and its
# Licensor(s). All Rights Reserved.
# 
# $Source: C:\\RCS\\D\\Pumpkin\\Salvo\\src\\Makefile,v $
# $Author: aek $
# $Revision: 3.64 $
# $Date: 2011-07-06 16:36:33-07 $
# 
#   Makefile for Salvo freeware and standard libraries for 
# compilers with command-line interfaces.
# 
#   Runs under Cygwin's bash shell, and presumably others, too. Created and tested
# under GNU make 3.79.1.
# 


###########################################################################################
#
#   The approach here to parsing the target name (a library name) and then recursively
# calling make with a bunch of passed variables based on the library name was proposed
# by Jonathan Kamens (jik@kamens.brookline.ma.us) in response to aek's query in the
# gnu.utils.help newgroup on 2001-08-31.
#
#   The idea is to use a Unix/Linux shell and regular expression to extract the chars
# of interest from a string, and then return a key based on those chars.
#
#	Note the following about expr. It means that searches forcibly begin at the
# start of the string:
#	
#	`expr' supports pattern matching and other string operators.  These
#  have lower precedence than both the numeric and relational operators (in
#  the next sections).
#  
#  `STRING : REGEX'
#       Perform pattern matching.  The arguments are coerced to strings
#       and the second is considered to be a (basic, a la GNU `grep')
#       regular expression, with a `^' implicitly prepended.  The first
#       argument is then matched against this regular expression.
#
#  If the match succeeds and REGEX uses `\(' and `\)', the `:'
#       expression returns the part of STRING that matched the
#       subexpression; otherwise, it returns the number of characters
#       matched.
#
#  Therefore to extract buried patterns, do something like this:
#
#	  expr 'sfiaravrt-7s-sf-ec-64-i.r90' : '.*\(-ec\)' 
#

# $(1) is target, $(2) is regexp to retrieve index chars
INDEX=$(shell expr '$(1)' : '$(2)')


# $(1) is index chars, $(2) is list of key/value pairs
PAIR=$(filter $(1)@%,$(2))


# $(1) is index chars, $(2) is individual key/value pair
VAL=$(patsubst $(1)@%,%,$(2))


# $(1) is target, $(2) is regexp to retrieve index chars, $(3) is list
# of key/value pairs
GETVAL = $(call VAL,$(call INDEX,$(1),$(2)),$(call PAIR,$(call INDEX,$(1),$(2)),$(3)))


###########################################################################################
#
# Defaults if user hasn't specified them on the command line.
#
# CLC is for custom library configurations -- CLC=0 means no custom library config.
#
# CVER is for compiler version -- CVER=1 means "compiler version v1.x"
#
ifndef ($(CLC))
  CLC = 0
endif

ifndef ($(CVER))
  CVER = 1
endif


###########################################################################################
#
#   T means library type.
#   T = f means freeware library
#   T = l means standard library
#
TYPE_VALS    = salvof@OSF salvol@OSL


###########################################################################################
#
#   F means family (a combination of compiler used AND target processor).
#   See Makefile2 for more information.
#
FAMILY_VALS  = p200@p200 p210@p210 p211@p211 p212@p212 p222@p222 \
               p400@p400 p401@p401 p402@p402 p40a@p40a p40b@p40b \
               p40c@p40c p411@p411 p412@p412 p41b@p41b p41c@p41c \
               p422@p422 p42c@p42c p432@p432 p43c@p43c p926@p926 \
               p700@p700 p704@p704 p710@p710 p714@p714 \
               p80s@p80s p80l@p80l p81s@p81s p81l@p81l \
               p82s@p82s p82l@p82l p83s@p83s p83l@p83l \
               p86s@p86s p86l@p86l p87s@p87s p87l@p87l \
               c18@c18 \
               iar18@iar18 \
               cv8@cv8 \
               gccx86@gccx86 gcc6811@gcc6811 \
               aq430@aq430 \
               ra430@ra430 ra430x@ra430x \
               ht430@ht430 \
               iar430@iar430 iar430x@iar430x \
               icc430@icc430 \
               icc11@icc11 \
               c51@c51 cx51@cx51 51@51 \
               ez8@ez8 \
               tic28x@tic28x tic24x@tic24x \
               iccavr@iccavr iccatmega@iccatmega iccatm128@iccatm128 iccatm256@iccatm256 \
               gccavr@gccavr gccatm256@gccatm256 \
               iaravr@iaravr \
               kcarm@kcarm \
               iararm@iararm \
               iccarm@iccarm \
               gccarm@gccarm \
               mcc30@mcc30 \
               mcc32@mcc32 \
               armrvcm3@armrvcm3 \
               gccarmcm3@gccarmcm3 \
               armrv4@armrv4 \
               eps1c17@eps1c17 \
               mscx86@mscx86
              
###########################################################################################
#
#   SU means library name suffix
#	suffixes are appended directly to the family name, e.g. iaravr4
#
SUFFIX_VALS  = 0@0 1@1 2@2 3@3 4@4 5@5 6@6 7@7


###########################################################################################
#
#   END means endian-ness
#	B = big-endian
#	L = little-endian
#
ENDIAN_VALS  = l@OSL L@OSL b@OSB B@OSB 


###########################################################################################
#
#   ISET means instruction set
#	A = ARM instruction set
#	T = Thumb instruction set
# L = MIPS4K 32-bit instructions
# S = MIPS4K 16-bit instructions
#
INSTRUCTION_SET_VALS  = a@OSA A@OSA t@OST T@OST l@OSL L@OSL s@OSS S@OSS 


###########################################################################################
#
#   IARAVR_CORE tells the compiler to generate instructions from the enhanced instruction
#    set that is available on some AVR derivatives, e.g. the ATmega161.
#
#	- = not selected 
#	E = selected
#
IARAVR_CORE_VALS = e@OSE E@OSE -@OSNONE 


###########################################################################################
#
#   IARAVR_FLASH tells the compiler that the intended target processor does not have more 
#    than 64 Kbytes program flash memory (small flash), and that the AVR core therefore does
#    not have the RAMPZ register or the ELPM instruction. It can only be used together with
#    the -v2, -v3, and -v4 processor options. 
#
#	- = not selected
#	S = selected
#
IARAVR_FLASH_VALS = s@OSS S@OSS -@OSNONE 


###########################################################################################
#
#   IARAVR_DOUBLES forces the compiler to use 64-bit doubles instead of 32-bit doubles,
#    which is the default.
#
#	- = not selected
#	D = selected
#
IARAVR_DOUBLES_VALS = d@OSD D@OSD -@OSNONE 


###########################################################################################
#
#   M means memory model.
#   M = s means small memory model (the default)
#   M = c means compact memory model
#   M = m means medium or middle memory model
#   M = l means large memory model
#   M = t means tiny memory model
#   M = r means regular memory model
#
MODEL_VALS = c@OSC C@OSC l@OSL L@OSL m@OSM M@OSM r@OSR R@OSR s@OSS S@OSS t@OST T@OST


###########################################################################################
#
#   O means library option
#   - = no option
#   I = library includes embedded source-level debug information
#
OPTION_VALS = i@OSI I@OSI -@OSNONE


###########################################################################################
#
#   IW means interwork option
#   - = no option
#   I = Iinterworkable library (ARM7 TDMI)
#
INTERWORK_VALS = i@OSI I@OSI -@OSNONE


###########################################################################################
#
#   G means memory type (for Salvo global objects).
#   G = - means the default memory type
#   G = d means data memory type
#   G = f means far memory type
#   G = i means idata memory type
#   G = n means near memory type
#   G = x means xdata memory type
#   G = t means tiny memory type
#   G = h means huge memory type
#
GLOBAL_VALS = d@OSD D@OSD f@OSF F@OSF h@OSH H@OSH i@OSI I@OSI n@OSN N@OSN t@OST T@OST x@OSX X@OSX -@OSNONE


###########################################################################################
#
#   C means library configuration.
#   C = a means library supports multitasking, delays and events
#   C = d means library supports delays only
#   C = e means library supports events only
#   C = m means library supports multitasking only
#   C = t means library supports multitasking, delays and events with timeouts
#   C = s for SE libraries only
#   C = y for tiny libraries only
#
CONFIG_VALS  = a@OSA A@OSA d@OSD D@OSD e@OSE E@OSE m@OSM M@OSM s@OSS S@OSS t@OST T@OST y@OSY Y@OSY


###########################################################################################
#
#   V means library variant.
#   V = - means no variant
#   V = a means services can be called from anywhere , simultaneously 
#   V = b means services can only be called from the background only
#   V = e means services can only be called from either the background or the foreground, but not both
#   V = i means that debugging information is included in the library
#   V = f means services can only be called from the background only
#
VARIANT_VALS = a@OSA A@OSA b@OSB B@OSB e@OSE E@OSE f@OSF F@OSF -@OSNONE


###########################################################################################
#
#   include target variables
#
include salvotargets.mk


###########################################################################################
#
#   phony targets to force make
#
.PHONY: 	$(ARM_RVCM3_STD_FAMILIES)	$(ARM_RVCM3_OPT_FAMILIES)	\
	$(ARM_RV4_STD_FAMILIES)	  	$(ARM_RV4_OPT_FAMILIES) 					\
	$(GCC_ARM_STD_FAMILIES)     $(GCC_ARM_OPT_FAMILIES) 				  \
	$(GCC_AVR_STD_FAMILIES)     $(GCC_AVR_OPT_FAMILIES) 				  \
	$(GCC_ATM256_STD_FAMILIES)  $(GCC_ATM256_OPT_FAMILIES) 			  \
	$(GCC_S1C17_STD_FAMILIES)   $(GCC_S1C17_OPT_FAMILIES) 				\
	$(GCC_ARMCM3_STD_FAMILIES)  $(GCC_ARMCM3_OPT_FAMILIES) 			  \
	$(ICC_AVR_STD_FAMILIES) 	$(ICC_AVR_OPT_FAMILIES) 			  \
	$(ICC_ATMEGA_STD_FAMILIES) 	$(ICC_ATMEGA_OPT_FAMILIES) 			  \
	$(ICC_ATM128_STD_FAMILIES) 	$(ICC_ATM128_OPT_FAMILIES) 			  \
	$(ICC_ATM256_STD_FAMILIES) 	$(ICC_ATM256_OPT_FAMILIES) 			  \
	$(IAR_430_STD_FAMILIES) 	$(IAR_430_OPT_FAMILIES) 				\
	$(IAR_430X_STD_FAMILIES) 	$(IAR_430X_OPT_FAMILIES) 				\
	$(IAR_ARM_STD_FAMILIES) 	$(IAR_ARM_OPT_FAMILIES) 					\
	$(IAR_AVR_STD_FAMILIES) 	$(IAR_AVR_OPT_FAMILIES) 					\
	$(ICC_ARM_STD_FAMILIES) 	$(ICC_ARM_OPT_FAMILIES)						\
	$(KEIL_ARM_STD_FAMILIES)  	$(KEIL_ARM_OPT_FAMILIES) 					\
	$(MCC_PIC24_STD_FAMILIES)	$(MCC_PIC24_OPT_FAMILIES)         \
	$(MCC_PIC32_STD_FAMILIES)	$(MCC_PIC32_OPT_FAMILIES)         \
	$(MSC_X86_STD_FAMILIES)		$(MSC_X86_OPT_FAMILIES)         \
	$(RA_430_STD_FAMILIES) 	    $(RA_430_OPT_FAMILIES) 				   	\
	$(RA_430X_STD_FAMILIES)   	$(RA_430X_OPT_FAMILIES) 					
	


###########################################################################################
#
#   help targets: (default rule)
#
liblist listlib help :
	@echo "  Salvo make utility."
	@echo "  In addition to individual libraries (e.g. sfp42cab.lib), the following"
	@echo "targets can be specified by family. For each one (e.g. p4xx), all types,"
	@echo "configurations and variants will be built. The appropriate compiler must" 
	@echo "be installed for each family."
	@echo " all" 
	@echo "  8051"
	@echo "   $(HT_C51_FAMILIES)"
	@echo "   $(KEIL_C51_FAMILIES)"
	@echo "   $(KEIL_Cx51_FAMILIES)"
	@echo "  PIC24/dsPIC"
	@echo "   $(MCC_PIC24_STD_FAMILIES)		$(MCC_PIC24_OPT_FAMILIES)"
	@echo "  PIC32"
	@echo "   $(MCC_PIC32_STD_FAMILIES)		$(MCC_PIC32_OPT_FAMILIES)"
	@echo "  ARM"
	@echo "   $(GCC_ARM_STD_FAMILIES)		$(GCC_ARM_OPT_FAMILIES)"
	@echo "   $(GCC_ARMCM3_STD_FAMILIES)	$(GCC_ARMCM3_OPT_FAMILIES)"
	@echo "   $(IAR_ARM_STD_FAMILIES)		$(IAR_ARM_OPT_FAMILIES)"
	@echo "   $(IAR_ARM_STD_FAMILIES_V5)    $(IAR_ARM_OPT_FAMILIES_V5)"
	@echo "   $(IAR_ARM_STD_FAMILIES_V6)    $(IAR_ARM_OPT_FAMILIES_V6)"
	@echo "   $(ICC_ARM_STD_FAMILIES)	    	$(ICC_ARM_OPT_FAMILIES)"
	@echo "   $(KEIL_ARM_STD_FAMILIES)		$(KEIL_ARM_OPT_FAMILIES)"
	@echo "   $(ARM_RVCM3_STD_FAMILIES)		$(ARM_RVCM3_OPT_FAMILIES)"
	@echo "   $(ARM_RV4_STD_FAMILIES)		$(ARM_RV4_OPT_FAMILIES)"
	@echo "  AVR"
	@echo "   $(GCC_AVR_STD_FAMILIES)"
	@echo "   $(GCC_ATM256_STD_FAMILIES)"
	@echo "   iaravr-v2"
	@echo "    iaravr[0|1|2|3|4|5|6]-v2"
	@echo "   iaravr-v3"
	@echo "    iaravr[0|1|2|3|4|5|6]-v3"
	@echo "   $(ICC_AVR_STD_FAMILIES_V6)    $(ICC_AVR_OPT_FAMILIES_V6)"
	@echo "   $(ICC_ATMEGA_STD_FAMILIES_V6) $(ICC_ATMEGA_OPT_FAMILIES_V6)"
	@echo "   $(ICC_AVR_STD_FAMILIES_V7)    $(ICC_AVR_OPT_FAMILIES_V7)"	
	@echo "   $(ICC_ATMEGA_STD_FAMILIES_V7) $(ICC_ATMEGA_OPT_FAMILIES_V7)"	
	@echo "   $(ICC_ATM128_STD_FAMILIES_V7) $(ICC_ATM128_OPT_FAMILIES_V7)"	
	@echo "   $(ICC_ATM256_STD_FAMILIES_V7) $(ICC_ATM256_OPT_FAMILIES_V7)"	
	@echo "  $(HT_V8C_cv8_FAMILIES)"
	@echo "  $(ZLG_EZ8_FAMILIES)"
	@echo "  $(GCC_6811_FAMILIES)"
	@echo "  $(GCC_x86_FAMILIES)"
	@echo "  m68hcxx"
	@echo "   $(ICC_11_FAMILIES)"
	@echo "  msp430"
	@echo "   $(AQ_430_FAMILIES)"
	@echo "   $(HT_430_FAMILIES)"
	@echo "   $(IAR_430_STD_FAMILIES_V1)  $(IAR_430_OPT_FAMILIES_V1)"
	@echo "   $(IAR_430_STD_FAMILIES_V2)  $(IAR_430_OPT_FAMILIES_V2)"
	@echo "   $(IAR_430_STD_FAMILIES_V3)  $(IAR_430_OPT_FAMILIES_V3)"
	@echo "   $(IAR_430X_STD_FAMILIES_V3) $(IAR_430X_OPT_FAMILIES_V3)"
	@echo "   $(IAR_430_STD_FAMILIES_V4)  $(IAR_430_OPT_FAMILIES_V4)"
	@echo "   $(IAR_430X_STD_FAMILIES_V4) $(IAR_430X_OPT_FAMILIES_V4)"
	@echo "   $(ICC_430_FAMILIES)"
	@echo "   $(RA_430_STD_FAMILIES_V1)   $(RA_430_OPT_FAMILIES_V1)"
	@echo "   $(RA_430_STD_FAMILIES_V2)   $(RA_430_OPT_FAMILIES_V2)"
	@echo "   $(RA_430X_STD_FAMILIES_V2)  $(RA_430X_OPT_FAMILIES_V2)"
	@echo "  pic"
	@echo "   $(MPLAB_C18_FAMILIES)"
	@echo "   $(IAR_PIC18_FAMILIES)"
	@echo "   p2xx"
	@echo "    $(HT_PICC_p2xx_FAMILIES)" 
	@echo "   p4xx"
	@echo "    $(HT_PICC_p4xx_FAMILIES)"
	@echo "   p7xx"
	@echo "    $(HT_PICC_p7xx_FAMILIES)" 
	@echo "   p8xx"
	@echo "    $(HT_PICC_p8xx_FAMILIES)"
	@echo "  S1C17"
	@echo "   $(GCC_S1C17_STD_FAMILIES)		$(GCC_S1C17_OPT_FAMILIES)"	
	@echo "  tms320c2000"
	@echo "    $(TI_C2400_FAMILIES)"
	@echo "    $(TI_C2800_FAMILIES)"
	@echo "  x86"
	@echo "    $(MSC_X86_STD_FAMILIES)  $(MSC_X86_OPT_FAMILIES)"
	


###########################################################################################
#
#   housekeeping targets:
#
# Note that rmobjs is incomplete -- the one in Makefile2 is complete!
#
rmobjs :	
	@rm -f *.obj *.OBJ *.o *.opp *.O *.r43 *.r49 *.r70 *.r90 
	@rm -f $(LIBDIR)/*.obj $(LIBDIR)/*.OBJ $(LIBDIR)/*.o $(LIBDIR)/*.opp \
	$(LIBDIR)/*.O $(LIBDIR)/*.r43 $(LIBDIR)/*.r49 $(LIBDIR)/*.r90  \
	$(LIBDIR)/*.hzo

rmlsts :	
	@rm -f *.lst *.LST

rmsdbs :	
	@rm -f *.sdb *.SDB

rmrlfs :	
	@rm -f *.rlf *.RLF


###########################################################################################
#
#   maintenance targets:
#
clean:	rmobjs rmlsts rmsdbs rmrlfs

clobber: clean
	@cd ../Lib/EPS1C17  	; rm -f $(GCC_S1C17_STD_LIBRARIES)    ; rm -f $(GCC_S1C17_OPT_LIBRARIES)
	@cd ../Lib/GCCARM	  	; rm -f $(GCC_ARM_STD_LIBRARIES)  		; rm -f $(GCC_ARM_OPT_LIBRARIES)
	@cd ../Lib/GCCARM	  	; rm -f $(GCC_ARMCM3_STD_LIBRARIES)  	; rm -f $(GCC_ARMCM3_OPT_LIBRARIES)
	@cd ../Lib/IARARM	  	; rm -f $(IAR_ARM_STD_LIBRARIES)  		; rm -f $(IAR_ARM_OPT_LIBRARIES)
	@cd ../Lib/ICCARM	  	; rm -f $(ICC_ARM_STD_LIBRARIES)  		; rm -f $(ICC_ARM_OPT_LIBRARIES)
	@cd ../Lib/KCARM	  	; rm -f $(KEIL_ARM_STD_LIBRARIES) 		; rm -f $(KEIL_ARM_OPT_LIBRARIES)
	@cd ../Lib/MCC30	  	; rm -f $(MCC_PIC24_STD_LIBRARIES)    ; rm -f $(MCC_PIC24_OPT_LIBRARIES)
	@cd ../Lib/MCC32	  	; rm -f $(MCC_PIC32_STD_LIBRARIES)   	; rm -f $(MCC_PIC32_OPT_LIBRARIES)
	@cd ../Lib/MSCX86	  	; rm -f $(MSC_X86_STD_LIBRARIES)   	    ; rm -f $(MSC_X86_OPT_LIBRARIES)
	@cd ../Lib/ARMRV		  ; rm -f $(ARM_RVCM3_STD_LIBRARIES)   	; rm -f $(ARM_RVCM3_OPT_LIBRARIES)
	@cd ../Lib/ARMRV	  	; rm -f $(ARM_RV4_STD_LIBRARIES)   		; rm -f $(ARM_RV4_OPT_LIBRARIES)
	@cd ../Lib/gcc6811		; rm -f $(GCC_6811_LIBRARIES)
	@cd ../Lib/gccx86		  ; rm -f $(GCC_x86_LIBRARIES)
	@cd ../Lib/GCCAVR 		; rm -f $(GCC_AVR_LIBRARIES)
	@cd ../Lib/GCCAVR 		; rm -f $(GCC_ATM256_LIBRARIES)
	@cd ../Lib/iaravr-v2	; rm -f $(IAR_AVR_LIBRARIES)
	@cd ../Lib/iaravr-v3	; rm -f $(IAR_AVR_LIBRARIES)
	@cd ../Lib/iccavr 		; rm -f $(ICC_AVR_LIBRARIES)
	@cd ../Lib/iccavr		  ; rm -f $(ICC_ATMEGA_LIBRARIES)
	@cd ../Lib/cv8     		; rm -f $(HT_V8C_cv8_LIBRARIES)
	@cd ../Lib/htpicc   	; rm -f $(HT_PICC_p2xx_LIBRARIES)
	@cd ../Lib/htpicc 		; rm -f $(HT_PICC_p4xx_LIBRARIES)
	@cd ../Lib/htpicc 		; rm -f $(HT_PICC_p7xx_LIBRARIES)
	@cd ../Lib/htpicc18		; rm -f $(HT_PICC_p8xx_LIBRARIES)
	@cd ../Lib/c18 		  	; rm -f $(MPLAB_C18_LIBRARIES)
	@cd ../Lib/iar18		  ; rm -f $(IAR_PIC18_LIBRARIES)
	@cd ../Lib/ht51		  	; rm -f $(HT_C51_LIBRARIES)
	@cd ../Lib/aq430	  	; rm -f $(AQ_430_LIBRARIES)
	@cd ../Lib/ht430	   	; rm -f $(HT_430_LIBRARIES)
	@cd ../Lib/iar430-v1	; rm -f $(IAR_430_LIBRARIES)
	@cd ../Lib/iar430-v2	; rm -f $(IAR_430_LIBRARIES)
	@cd ../Lib/iar430-v3	; rm -f $(IAR_430_LIBRARIES)
	@cd ../Lib/iar430-v4	; rm -f $(IAR_430_LIBRARIES)
	@cd ../Lib/icc430 		; rm -f $(ICC_430_LIBRARIES)
	@cd ../Lib/ra430-v1		; rm -f $(RA_430_LIBRARIES)
	@cd ../Lib/ra430-v2		; rm -f $(RA_430_LIBRARIES)
	@cd ../Lib/icc11  		; rm -f $(ICC_11_LIBRARIES)
	@cd ../Lib/kc51			  ; rm -f $(KEIL_C51_LIBRARIES)
	@cd ../Lib/kc51		  	; rm -f $(KEIL_Cx51_LIBRARIES)
	@cd ../Lib/z8encore		; rm -f $(ZLG_EZ8_LIBRARIES)
	@cd ../Lib/tic2000		; rm -f $(TI_C2400_LIBRARIES)
	@cd ../Lib/tic2000		; rm -f $(TI_C2800_LIBRARIES)
	@rm -f lib.err


###########################################################################################
#
#   library targets:
#
all:
	$(MAKE) $(ARM_RV4_STD_LIBRARIES)
	$(MAKE) $(ARM_RVCM3_STD_LIBRARIES)
	$(MAKE) $(GCC_S1C17_STD_LIBRARIES)
	$(MAKE) $(GCC_ARM_STD_LIBRARIES)
	$(MAKE) $(GCC_ARMCM3_STD_LIBRARIES)
	$(MAKE) $(IAR_ARM_STD_LIBRARIES)
	$(MAKE) $(IAR_430_STD_LIBRARIES) CVER=1
	$(MAKE) $(IAR_430_STD_LIBRARIES) CVER=2
	$(MAKE) $(IAR_430_STD_LIBRARIES) CVER=3
	$(MAKE) $(IAR_430X_STD_LIBRARIES) CVER=3
	$(MAKE) $(IAR_430_STD_LIBRARIES) CVER=4
	$(MAKE) $(IAR_430X_STD_LIBRARIES) CVER=4
	$(MAKE) $(ICC_ARM_STD_LIBRARIES)
	$(MAKE) $(KEIL_ARM_STD_LIBRARIES)
	$(MAKE) $(MCC_PIC24_STD_LIBRARIES)
	$(MAKE) $(MCC_PIC32_STD_LIBRARIES)
	$(MAKE) $(MSC_X86_STD_LIBRARIES)
	$(MAKE) $(HT_C51_LIBRARIES)
	$(MAKE) $(KEIL_C51_LIBRARIES)
	$(MAKE) $(KEIL_Cx51_LIBRARIES)
	$(MAKE) $(GCC_AVR_LIBRARIES)
	$(MAKE) $(GCC_ATM256_LIBRARIES)
	$(MAKE) $(IAR_AVR_LIBRARIES) CVER=2
	$(MAKE) $(IAR_AVR_LIBRARIES) CVER=3
	$(MAKE) $(ICC_AVR_STD_LIBRARIES) CVER=6
	$(MAKE) $(ICC_ATMEGA_STD_LIBRARIES) CVER=6
	$(MAKE) $(ICC_AVR_STD_LIBRARIES) CVER=7	
	$(MAKE) $(ICC_ATMEGA_STD_LIBRARIES) CVER=7	
	$(MAKE) $(ICC_ATM128_STD_LIBRARIES) CVER=7	
	$(MAKE) $(ICC_ATM256_STD_LIBRARIES) CVER=7	
	$(MAKE) $(HT_V8C_cv8_LIBRARIES)
	$(MAKE) $(ZLG_EZ8_LIBRARIES)
	$(MAKE) $(GCC_6811_LIBRARIES)
	$(MAKE) $(GCC_x86_LIBRARIES)
	$(MAKE) $(ICC_11_LIBRARIES)
	$(MAKE) $(AQ_430_LIBRARIES)
	$(MAKE) $(HS_430_LIBRARIES)
	$(MAKE) $(ICC_430_LIBRARIES)
	$(MAKE) $(RA_430_STD_LIBRARIES) CVER=1
	$(MAKE) $(RA_430_STD_LIBRARIES) CVER=2
	$(MAKE) $(RA_430X_STD_LIBRARIES) CVER=2
	$(MAKE) $(MPLAB_C18_LIBRARIES)
	$(MAKE) $(IAR_PIC18_LIBRARIES)
	$(MAKE) $(HT_PICC_p2xx_LIBRARIES)
	$(MAKE) $(HT_PICC_p4xx_LIBRARIES)
	$(MAKE) $(HT_PICC_p7xx_LIBRARIES)
	$(MAKE) $(HT_PICC_p8xx_LIBRARIES)
	$(MAKE) $(TI_C2400_LIBRARIES)
	$(MAKE) $(TI_C2800_LIBRARIES)
	@echo Makefile: All libraries appear to have been made successfully.
	
arm ARM:
#	$(MAKE) $(GCC_ARM_STD_LIBRARIES)
	$(MAKE) $(GCC_ARMCM3_STD_LIBRARIES)
#	$(MAKE) $(IAR_ARM_STD_LIBRARIES)
	$(MAKE) $(ARM_RVCM3_STD_LIBRARIES)
	$(MAKE) $(ARM_RV4_STD_LIBRARIES)
	$(MAKE) $(ICC_ARM_STD_LIBRARIES)
#	$(MAKE) $(KEIL_ARM_STD_LIBRARIES)


dsPIC DSPIC dspic:
  $(MAKE) $(MCC_PIC24_STD_LIBRARIES) CVER=2
  $(MAKE) $(MCC_PIC24_STD_LIBRARIES) CVER=3
	
	
s1C17 S1C17 S1c17 s1c17:
	$(MAKE) $(GCC_S1C17_STD_LIBRARIES)


x86 X86:
	$(MAKE) $(MSC_X86_STD_LIBRARIES)


PIC32 pic32:
	$(MAKE) $(MCC_PIC32_STD_LIBRARIES)

		
avr AVR:
	$(MAKE) $(GCC_AVR_STD_LIBRARIES)	
	$(MAKE) $(GCC_ATM256_STD_LIBRARIES)	
	$(MAKE) $(IAR_AVR_STD_LIBRARIES) CVER=2
	$(MAKE) $(IAR_AVR_STD_LIBRARIES) CVER=3	
	$(MAKE) $(ICC_AVR_STD_LIBRARIES) CVER=6	
	$(MAKE) $(ICC_ATMEGA_STD_LIBRARIES)	CVER=6
	$(MAKE) $(ICC_AVR_STD_LIBRARIES) CVER=7	
	$(MAKE) $(ICC_ATMEGA_STD_LIBRARIES)	CVER=7	
	$(MAKE) $(ICC_ATM128_STD_LIBRARIES)	CVER=7	
	$(MAKE) $(ICC_ATM256_STD_LIBRARIES)	CVER=7	
	
m68hcxx M68HCXX:
	$(MAKE) $(ICC_11_LIBRARIES)
	
	
msp430 MSP430:
#	$(MAKE) $(AQ_430_LIBRARIES)	
#	$(MAKE) $(HT_430_LIBRARIES)
	$(MAKE) $(IAR_430_STD_LIBRARIES) CVER=1
	$(MAKE) $(IAR_430_STD_LIBRARIES) CVER=2
	$(MAKE) $(IAR_430_STD_LIBRARIES) CVER=3
	$(MAKE) $(IAR_430X_STD_LIBRARIES) CVER=3
	$(MAKE) $(IAR_430_STD_LIBRARIES) CVER=4
	$(MAKE) $(IAR_430X_STD_LIBRARIES) CVER=4
#	$(MAKE) $(ICC_430_LIBRARIES)
	$(MAKE) $(RA_430_STD_LIBRARIES) CVER=1
	$(MAKE) $(RA_430_STD_LIBRARIES) CVER=2
	$(MAKE) $(RA_430X_STD_LIBRARIES) CVER=2	
	
8051:	
	$(MAKE) $(HT_C51_LIBRARIES)
	$(MAKE) $(KEIL_C51_LIBRARIES)
	$(MAKE) $(KEIL_Cx51_LIBRARIES)
	
pic PIC:	
	$(MAKE) $(MPLAB_C18_LIBRARIES)
	$(MAKE) $(IAR_PIC18_LIBRARIES)
	$(MAKE) $(HT_PICC_p2xx_LIBRARIES)
	$(MAKE) $(HT_PICC_p4xx_LIBRARIES)
	$(MAKE) $(HT_PICC_p7xx_LIBRARIES)
	$(MAKE) $(HT_PICC_p8xx_LIBRARIES)

tms320c2000 TMS320C2000:	
	$(MAKE) $(TI_C2400_LIBRARIES)
	$(MAKE) $(TI_C2800_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSIAR_IAR  OSTARGET == OSAVR
#
iaravr-v2:
	$(MAKE) $(IAR_AVR0_STD_LIBRARIES) CVER=2
	$(MAKE) $(IAR_AVR1_STD_LIBRARIES) CVER=2
	$(MAKE) $(IAR_AVR2_STD_LIBRARIES) CVER=2
	$(MAKE) $(IAR_AVR3_STD_LIBRARIES) CVER=2
	$(MAKE) $(IAR_AVR4_STD_LIBRARIES) CVER=2
	$(MAKE) $(IAR_AVR5_STD_LIBRARIES) CVER=2
	$(MAKE) $(IAR_AVR6_STD_LIBRARIES) CVER=2
	
iaravr0-v2:
	$(MAKE) $(IAR_AVR0_STD_LIBRARIES) CVER=2
		
iaravr1-v2:
	$(MAKE) $(IAR_AVR1_STD_LIBRARIES) CVER=2
		
iaravr2-v2:
	$(MAKE) $(IAR_AVR2_STD_LIBRARIES) CVER=2	
	
iaravr3-v2:
	$(MAKE) $(IAR_AVR3_STD_LIBRARIES) CVER=2
		
iaravr4-v2:
	$(MAKE) $(IAR_AVR4_STD_LIBRARIES) CVER=2
		
iaravr5-v2:
	$(MAKE) $(IAR_AVR5_STD_LIBRARIES) CVER=2
		
iaravr6-v2:
	$(MAKE) $(IAR_AVR6_STD_LIBRARIES) CVER=2
		
iaravr7-v2:
	$(MAKE) $(IAR_AVR7_STD_LIBRARIES) CVER=2
		

iaravr-v3:
	$(MAKE) $(IAR_AVR0_STD_LIBRARIES) CVER=3
	$(MAKE) $(IAR_AVR1_STD_LIBRARIES) CVER=3
	$(MAKE) $(IAR_AVR2_STD_LIBRARIES) CVER=3
	$(MAKE) $(IAR_AVR3_STD_LIBRARIES) CVER=3
	$(MAKE) $(IAR_AVR4_STD_LIBRARIES) CVER=3
	$(MAKE) $(IAR_AVR5_STD_LIBRARIES) CVER=3
	$(MAKE) $(IAR_AVR6_STD_LIBRARIES) CVER=3
	
iaravr0-v3:
	$(MAKE) $(IAR_AVR0_STD_LIBRARIES) CVER=3
		
iaravr1-v3:
	$(MAKE) $(IAR_AVR1_STD_LIBRARIES) CVER=3
		
iaravr2-v3:
	$(MAKE) $(IAR_AVR2_STD_LIBRARIES) CVER=3	
	
iaravr3-v3:
	$(MAKE) $(IAR_AVR3_STD_LIBRARIES) CVER=3
		
iaravr4-v3:
	$(MAKE) $(IAR_AVR4_STD_LIBRARIES) CVER=3
		
iaravr5-v3:
	$(MAKE) $(IAR_AVR5_STD_LIBRARIES) CVER=3
		
iaravr6-v3:
	$(MAKE) $(IAR_AVR6_STD_LIBRARIES) CVER=3
		
iaravr7-v3:
	$(MAKE) $(IAR_AVR7_STD_LIBRARIES) CVER=3
		
iaravr-v4:
	$(MAKE) $(IAR_AVR0_STD_LIBRARIES) CVER=4
	$(MAKE) $(IAR_AVR1_STD_LIBRARIES) CVER=4
	$(MAKE) $(IAR_AVR2_STD_LIBRARIES) CVER=4
	$(MAKE) $(IAR_AVR3_STD_LIBRARIES) CVER=4
	$(MAKE) $(IAR_AVR4_STD_LIBRARIES) CVER=4
	$(MAKE) $(IAR_AVR5_STD_LIBRARIES) CVER=4
	$(MAKE) $(IAR_AVR6_STD_LIBRARIES) CVER=4
	
iaravr0-v4:
	$(MAKE) $(IAR_AVR0_STD_LIBRARIES) CVER=4
		
iaravr1-v4:
	$(MAKE) $(IAR_AVR1_STD_LIBRARIES) CVER=4
	
iaravr2-v4:
	$(MAKE) $(IAR_AVR2_STD_LIBRARIES) CVER=4	
	
iaravr3-v4:
	$(MAKE) $(IAR_AVR3_STD_LIBRARIES) CVER=4
		
iaravr4-v4:
	$(MAKE) $(IAR_AVR4_STD_LIBRARIES) CVER=4
		
iaravr5-v4:
	$(MAKE) $(IAR_AVR5_STD_LIBRARIES) CVER=4
		
iaravr6-v4:
	$(MAKE) $(IAR_AVR6_STD_LIBRARIES) CVER=4
		
iaravr7-v4:
	$(MAKE) $(IAR_AVR7_STD_LIBRARIES) CVER=4
		


###########################################################################################
#
#   OSCOMPILER == OSIAR_ICC OSTARGET == OSMSP430
#
$(IAR_430_STD_FAMILIES_V1) :
	$(MAKE) $(IAR_430_STD_LIBRARIES) CVER=1

$(IAR_430_STD_FAMILIES_V2) :
	$(MAKE) $(IAR_430_STD_LIBRARIES) CVER=2

$(IAR_430_STD_FAMILIES_V3) :
	$(MAKE) $(IAR_430_STD_LIBRARIES) CVER=3

$(IAR_430X_STD_FAMILIES_V3) :
	$(MAKE) $(IAR_430X_STD_LIBRARIES) CVER=3

$(IAR_430_STD_FAMILIES_V4) :
	$(MAKE) $(IAR_430_STD_LIBRARIES) CVER=4

$(IAR_430X_STD_FAMILIES_V4) :
	$(MAKE) $(IAR_430X_STD_LIBRARIES) CVER=4



###########################################################################################
#
#   OSCOMPILER == OSGCC_S1C17 OSTARGET == OSEPSON_S1C17
#
$(GCC_S1C17_STD_FAMILIES) :
	$(MAKE) $(GCC_S1C17_STD_LIBRARIES)

$(GCC_S1C17_OPT_FAMILIES) :
	$(MAKE) $(GCC_S1C17_OPT_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSMSC_CPP OSTARGET == OSIX86
#
$(MSC_X86_STD_FAMILIES) :
	$(MAKE) $(MSC_X86_STD_LIBRARIES)

$(MSC_X86_OPT_FAMILIES) :
	$(MAKE) $(MSC_X86_OPT_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSGCC_ARM OSTARGET == OSARM7
#
$(GCC_ARM_STD_FAMILIES) :
	$(MAKE) $(GCC_ARM_STD_LIBRARIES)

$(GCC_ARM_OPT_FAMILIES) :
	$(MAKE) $(GCC_ARM_OPT_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSGCC_CM3 OSTARGET == OSARMCM3
#
$(GCC_ARMCM3_STD_FAMILIES) :
	$(MAKE) $(GCC_ARMCM3_STD_LIBRARIES)

$(GCC_ARMCM3_OPT_FAMILIES) :
	$(MAKE) $(GCC_ARMCM3_OPT_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSMCC_PIC24 OSTARGET == OSPIC24
#
$(MCC_PIC24_STD_FAMILIES) :
	$(MAKE) $(MCC_PIC24_STD_LIBRARIES)

$(MCC_PIC24_OPT_FAMILIES) :
	$(MAKE) $(MCC_PIC24_OPT_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSMCC_PIC32 OSTARGET == OSPIC32
#
$(MCC_PIC32_STD_FAMILIES) :
	$(MAKE) $(MCC_PIC32_STD_LIBRARIES)

$(MCC_PIC32_OPT_FAMILIES) :
	$(MAKE) $(MCC_PIC32_OPT_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSRVDS && OSTARGET == OSARMCM3
#
$(ARM_RVCM3_STD_FAMILIES) :
	$(MAKE) $(ARM_RVCM3_STD_LIBRARIES)
	
$(ARM_RVCM3_OPT_FAMILIES) :
	$(MAKE) $(ARM_RVCM3_OPT_LIBRARIES)
	
	
###########################################################################################
#
#   OSCOMPILER == OSRVDS && OSTARGET == OSARM7
#
$(ARM_RV4_STD_FAMILIES) :
	$(MAKE) $(ARM_RV4_STD_LIBRARIES)
	
$(ARM_RV4_OPT_FAMILIES) :
	$(MAKE) $(ARM_RV4_OPT_LIBRARIES)
	
	
###########################################################################################
#
#   OSCOMPILER == OSIAR_ICC OSTARGET == OSARM7
#
$(IAR_ARM_STD_FAMILIES) :
	$(MAKE) $(IAR_ARM_STD_LIBRARIES)
	
$(IAR_ARM_OPT_FAMILIES) :
	$(MAKE) $(IAR_ARM_OPT_LIBRARIES)
	
	
###########################################################################################
#
#   OSCOMPILER == OSICC OSTARGET == OSARM7
#
$(ICC_ARM_STD_FAMILIES) :
	$(MAKE) $(ICC_ARM_STD_LIBRARIES)
	
$(ICC_ARM_OPT_FAMILIES) :
	$(MAKE) $(ICC_ARM_OPT_LIBRARIES)
	
	
###########################################################################################
#
#   OSCOMPILER == OSKEIL_CARM  OSTARGET == OSARM7
#
$(KEIL_ARM_STD_FAMILIES) :
	$(MAKE) $(KEIL_ARM_STD_LIBRARIES)
	
$(KEIL_ARM_OPT_FAMILIES) :
	$(MAKE) $(KEIL_ARM_OPT_LIBRARIES)
	




###########################################################################################
#
#   OSCOMPILER == OSHT_PICC  OSTARGET == OSPIC12 || OSPIC16 || OSPIC17 || OSPIC18
#
$(HT_PICC_p2xx_FAMILIES) \
$(HT_PICC_p4xx_FAMILIES) \
$(HT_PICC_p7xx_FAMILIES) \
$(HT_PICC_p8xx_FAMILIES) :
	$(MAKE) $(HT_PICC_$@_LIBRARIES)

p2xx p4xx p7xx p8xx: 
	$(MAKE) $(HT_PICC_$@_FAMILIES)

###########################################################################################
#
#   OSCOMPILER == OSMPLAB_C18  OSTARGET == OSPIC18
#
$(MPLAB_C18_FAMILIES) :
	$(MAKE) $(MPLAB_C18_$@_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSIAR_ICC OSTARGET == OSPIC18
#
$(IAR_PIC18_FAMILIES) :
	$(MAKE) $(IAR_PIC18_$@_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSHT_V8C  OSTARGET == OSV8
#
$(HT_V8C_cv8_FAMILIES) :
	$(MAKE) $(HT_V8C_$@_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSHT_C51  OSTARGET == OS8051
#
$(HT_C51_FAMILIES) :
	$(MAKE) $(HT_C51_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSGCC  OSTARGET == OSX86
#
$(GCC_x86_FAMILIES) :
	$(MAKE) $(GCC_x86_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSGCC  OSTARGET == OS6811
#
$(GCC_6811_FAMILIES) :
	$(MAKE) $(GCC_6811_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSGCC  OSTARGET == OSAVR
#
$(GCC_AVR_STD_FAMILIES) :
	$(MAKE) $(GCC_AVR_STD_LIBRARIES)
	
$(GCC_AVR_OPT_FAMILIES) :
	$(MAKE) $(GCC_AVR_OPT_LIBRARIES)


$(GCC_ATM256_STD_FAMILIES) :
	$(MAKE) $(GCC_ATM256_STD_LIBRARIES)
	
$(GCC_ATM256_OPT_FAMILIES) :
	$(MAKE) $(GCC_ATM256_OPT_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSAQ_430 OSTARGET == OSMSP430
#
$(AQ_430_FAMILIES) :
	$(MAKE) $(AQ_430_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSRA_CROSSSTUDIO OSTARGET == OSMSP430
#
$(RA_430_STD_FAMILIES_V1) :
	$(MAKE) $(RA_430_STD_LIBRARIES) CVER=1

$(RA_430_STD_FAMILIES_V2) :
	$(MAKE) $(RA_430_STD_LIBRARIES) CVER=2

$(RA_430X_STD_FAMILIES_V2) :
	$(MAKE) $(RA_430X_STD_LIBRARIES) CVER=2



###########################################################################################
#
#   OSCOMPILER == IAR OSTARGET == ARM7
#
$(IAR_ARM_STD_FAMILIES_V5) :
	$(MAKE) $(IAR_ARM_STD_LIBRARIES) CVER=5
	
$(IAR_ARM_STD_FAMILIES_V6) :
	$(MAKE) $(IAR_ARM_STD_LIBRARIES) CVER=6




###########################################################################################
#
#   OSCOMPILER == OSHT_430C OSTARGET == OSMSP430
#
$(HT_430_FAMILIES) :
	$(MAKE) $(HT_430_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSIMAGECRAFT OSTARGET == OS68HC11
#
$(ICC_11_FAMILIES) :
	$(MAKE) $(ICC_11_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSIMAGECRAFT OSTARGET == OSMSP430
#
$(ICC_430_FAMILIES) :
	$(MAKE) $(ICC_430_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSIMAGECRAFT OSTARGET == OSAVR
#
$(ICC_AVR_STD_FAMILIES_V6) :
	$(MAKE) $(ICC_AVR_STD_LIBRARIES) CVER=6
	
$(ICC_ATMEGA_STD_FAMILIES_V6) :
	$(MAKE) $(ICC_ATMEGA_STD_LIBRARIES) CVER=6
	
	
$(ICC_AVR_STD_FAMILIES_V7) :
	$(MAKE) $(ICC_AVR_STD_LIBRARIES) CVER=7	

$(ICC_ATMEGA_STD_FAMILIES_V7) :
	$(MAKE) $(ICC_ATMEGA_STD_LIBRARIES) CVER=7	

$(ICC_ATM128_STD_FAMILIES_V7) :
	$(MAKE) $(ICC_ATM128_STD_LIBRARIES) CVER=7	

$(ICC_ATM256_STD_FAMILIES_V7) :
	$(MAKE) $(ICC_ATM256_STD_LIBRARIES) CVER=7	


###########################################################################################
#
#   OSCOMPILER == OSZDSII OSTARGET == OSEZ8
#
$(ZLG_EZ8_FAMILIES) :
	$(MAKE) $(ZLG_EZ8_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSTI_CCS_DSP OSTARGET == OSTMS320C28X
#
$(TI_C2800_FAMILIES) :
	$(MAKE) $(TI_C2800_LIBRARIES)

$(TI_C2400_FAMILIES) :
	$(MAKE) $(TI_C2400_LIBRARIES)


###########################################################################################
#
#   OSCOMPILER == OSKEIL_C51  OSTARGET == OS8051
#
$(KEIL_C51_FAMILIES) :
	$(MAKE) $(KEIL_C51_LIBRARIES)
	
$(KEIL_Cx51_FAMILIES) :
	$(MAKE) $(KEIL_Cx51_LIBRARIES)


###########################################################################################
#
#   targets: libraries for HI-TECH PICC, PICC-Lite and PICC-18
#
$(HT_PICC_p2xx_LIBRARIES) \
$(HT_PICC_p4xx_LIBRARIES) \
$(HT_PICC_p7xx_LIBRARIES) \
$(HT_PICC_p8xx_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call GETVAL,$@,\(..\),$(TYPE_VALS)) \
		F=$(call GETVAL,$@,..\(....\)..\.lib,$(FAMILY_VALS)) \
		C=$(call GETVAL,$@,.*\(.\).\.lib,$(CONFIG_VALS)) \
		V=$(call GETVAL,$@,.*\(.\)\.lib,$(VARIANT_VALS)) \
		CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for MPLAB-C18 and PIC18 targets
#
$(MPLAB_C18_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		 T=$(call GETVAL,$@,\(..\),$(TYPE_VALS)) \
		 F=$(call GETVAL,$@,..\(...\)...\.lib,$(FAMILY_VALS)) \
		 M=$(call GETVAL,$@,.*\(.\)..\.lib,$(MODEL_VALS)) \
		 G=$(call GETVAL,$@,.*\(.\).\.lib,$(GLOBAL_VALS)) \
		 C=$(call GETVAL,$@,.*\(.\)\.lib,$(CONFIG_VALS)) \
		 V=OSNONE \
		 CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for IAR PIC18CXXX and PIC18 targets
#
$(IAR_PIC18_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		 T=$(call GETVAL,$@,\(..\),$(TYPE_VALS)) \
		 F=$(call GETVAL,$@,..\(.....\).....\.r49,$(FAMILY_VALS)) \
		 O=$(call GETVAL,$@,.*\(.\)....\.r49,$(OPTION_VALS)) \
		 M=$(call GETVAL,$@,.*\(.\)..\.r49,$(MODEL_VALS)) \
		 G=$(call GETVAL,$@,.*\(.\).\.r49,$(GLOBAL_VALS)) \
		 C=$(call GETVAL,$@,.*\(.\)\.r49,$(CONFIG_VALS)) \
		 V=OSNONE \
		 CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for HI-TECH V8C
#
$(HT_V8C_cv8_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call GETVAL,$@,\(..\),$(TYPE_VALS)) \
		F=$(call GETVAL,$@,..\(...\)..\.lib,$(FAMILY_VALS)) \
		C=$(call GETVAL,$@,.*\(.\).\.lib,$(CONFIG_VALS)) \
		V=$(call GETVAL,$@,.*\(.\)\.lib,$(VARIANT_VALS)) \
		CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for GNU C (GCC) and 6811 targets
#
$(GCC_6811_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call GETVAL,$@,...\(..\),$(TYPE_VALS)) \
		F=$(call GETVAL,$@,.....\(.......\)..\.a,$(FAMILY_VALS)) \
		C=$(call GETVAL,$@,....*\(.\).\.a,$(CONFIG_VALS)) \
		V=$(call GETVAL,$@,....*\(.\)\.a,$(VARIANT_VALS)) \
		CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for GNU C (GCC) and x86 targets
#
$(GCC_x86_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call GETVAL,$@,...\(..\),$(TYPE_VALS)) \
		F=$(call GETVAL,$@,.....\(......\)..\.a,$(FAMILY_VALS)) \
		C=$(call GETVAL,$@,....*\(.\).\.a,$(CONFIG_VALS)) \
		V=$(call GETVAL,$@,....*\(.\)\.a,$(VARIANT_VALS)) \
		CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for GNU AVR-GCC C Compiler
#
#	libsalvo[f|l][gccavr|gccatm256][-|i][a|d|e|m|t].a
#
$(GCC_AVR_STD_LIBRARIES) \
$(GCC_AVR_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call GETVAL,$@,...\(......\),$(TYPE_VALS)) \
		F=$(call GETVAL,$@,.*\(......\)..\.a,$(FAMILY_VALS)) \
		O=$(call GETVAL,$@,.*\(.\).\.a,$(OPTION_VALS)) \
		C=$(call GETVAL,$@,.*\(.\)\.a,$(CONFIG_VALS)) \
		V=OSNONE \
		CLC=$(CLC)
		
				
$(GCC_ATM256_STD_LIBRARIES) \
$(GCC_ATM256_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call GETVAL,$@,...\(......\),$(TYPE_VALS)) \
		F=$(call GETVAL,$@,.*\(.........\)..\.a,$(FAMILY_VALS)) \
		O=$(call GETVAL,$@,.*\(.\).\.a,$(OPTION_VALS)) \
		C=$(call GETVAL,$@,.*\(.\)\.a,$(CONFIG_VALS)) \
		V=OSNONE \
		CLC=$(CLC)
		
				
###########################################################################################
#
#   targets: libraries for GNU xgcc Compiler for S1C17
#
#	libsalvo[f|l][eps1c17][s|m|r][-|i][a|d|e|m|t].a
#
$(GCC_S1C17_STD_LIBRARIES) \
$(GCC_S1C17_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call GETVAL,$@,...\(......\),$(TYPE_VALS)) \
		F=$(call GETVAL,$@,.*\(.......\)...\.a,$(FAMILY_VALS)) \
		M=$(call GETVAL,$@,.*\(.\)..\.a,$(MODEL_VALS)) \
		O=$(call GETVAL,$@,.*\(.\).\.a,$(OPTION_VALS)) \
		C=$(call GETVAL,$@,.*\(.\)\.a,$(CONFIG_VALS)) \
		V=OSNONE \
		CLC=$(CLC)
		
				
###########################################################################################
#
#   targets: libraries for Microsoft C/C++ compiler for x86
#
#	salvo[f|l][mscx86][-|i][a|d|e|m|t].lib
#
$(MSC_X86_STD_LIBRARIES) \
$(MSC_X86_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call GETVAL,$@,\(......\),$(TYPE_VALS)) \
		F=$(call GETVAL,$@,.*\(......\)..\.lib,$(FAMILY_VALS)) \
		O=$(call GETVAL,$@,.*\(.\).\.lib,$(OPTION_VALS)) \
		C=$(call GETVAL,$@,.*\(.\)\.lib,$(CONFIG_VALS)) \
		V=OSNONE \
		CLC=$(CLC)
		
				
###########################################################################################
#
#   targets: libraries for Archelon / Quadravox AQ430 compiler
#
$(AQ_430_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call GETVAL,$@,\(..\),$(TYPE_VALS)) \
		F=$(call GETVAL,$@,..\(.....\)..\.lib,$(FAMILY_VALS)) \
		O=$(call GETVAL,$@,.*\(.\).\.lib,$(OPTION_VALS)) \
		C=$(call GETVAL,$@,.*\(.\)\.lib,$(CONFIG_VALS)) \
		V=OSNONE \
		CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for HI-TECH MSP430-C compiler
#
$(HT_430_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call GETVAL,$@,\(..\),$(TYPE_VALS)) \
		F=$(call GETVAL,$@,..\(.....\)..\.lib,$(FAMILY_VALS)) \
		O=$(call GETVAL,$@,.*\(.\).\.lib,$(OPTION_VALS)) \
		C=$(call GETVAL,$@,.*\(.\)\.lib,$(CONFIG_VALS)) \
		V=OSNONE \
		CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for ImageCraft ICC11 C compiler
#
$(ICC_11_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call GETVAL,$@,...\(..\),$(TYPE_VALS)) \
		F=$(call GETVAL,$@,.....\(.....\)..\.a,$(FAMILY_VALS)) \
		O=$(call GETVAL,$@,....*\(.\).\.a,$(OPTION_VALS)) \
		C=$(call GETVAL,$@,....*\(.\)\.a,$(CONFIG_VALS)) \
		V=OSNONE \
		CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for ImageCraft ICC430 C compiler
#
$(ICC_430_LIBRARIES) \
$(ICC_AVR_STD_LIBRARIES) \
$(ICC_AVR_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call GETVAL,$@,...\(......\),$(TYPE_VALS)) \
		F=$(call GETVAL,$@,.*\(......\)..\.a,$(FAMILY_VALS)) \
		O=$(call GETVAL,$@,.*\(.\).\.a,$(OPTION_VALS)) \
		C=$(call GETVAL,$@,.*\(.\)\.a,$(CONFIG_VALS)) \
		V=OSNONE \
		CVER=$(CVER) \
		CLC=$(CLC)

$(ICC_ATMEGA_STD_LIBRARIES) \
$(ICC_ATMEGA_OPT_LIBRARIES) \
$(ICC_ATM128_STD_LIBRARIES) \
$(ICC_ATM128_OPT_LIBRARIES) \
$(ICC_ATM256_STD_LIBRARIES) \
$(ICC_ATM256_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call GETVAL,$@,...\(......\),$(TYPE_VALS)) \
		F=$(call GETVAL,$@,.*\(.........\)..\.a,$(FAMILY_VALS)) \
		O=$(call GETVAL,$@,.*\(.\).\.a,$(OPTION_VALS)) \
		C=$(call GETVAL,$@,.*\(.\)\.a,$(CONFIG_VALS)) \
		V=OSNONE \
		CVER=$(CVER) \
		CLC=$(CLC)


###########################################################################################
#
#   IAR AVR compiler		
#
#   IARAVR_CORE tells the compiler to generate instructions from the enhanced instruction
#    set that is available on some AVR derivatives, e.g. the ATmega161.
#
#   IARAVR_CPU selects the target AVR family (0-6). 
#
#   IARAVR_DATA_MODEL selects the data model (tiny, small, large). 
#
#   IARAVR_DEBUG forces the compiler to embed debugging information into the library. 
#
#   IARAVR_FLASH tells the compiler that the intended target processor does not have more 
#    than 64 Kbytes program flash memory (small flash), and that the AVR core therefore does
#    not have the RAMPZ register or the ELPM instruction. It can only be used together with
#    the -v2, -v3, and -v4 processor options. 
#
#   IARAVR_DOUBLES forces the compiler to use 64-bit doubles instead of 32-bit doubles,
#    which is the default.
#
#	Example: sfiaravrt-7s-ec-sf-64-i.r90
#
#
#IARAVR_CORE_VALS       = -ec@TRUE
#IARAVR_CPU_VALS        = 0@0 1@1 2@2 3@3 4@4 5@5 6@6 7@7
#IARAVR_DATA_MODEL_VALS = l@OSL L@OSL s@OSS S@OSS t@OST T@OST
#IARAVR_DEBUG_VALS      = -i@TRUE
#IARAVR_DOUBLES_VALS    = -64@TRUE 
#IARAVR_FLASH_VALS      = -sf@TRUE  
#
#	salvo[f|l][iaravr0|1|2|3|4|5|6|7][l|s|t][-|i][a|d|e|m|t].r90
#


$(IAR_AVR_STD_LIBRARIES) \
$(IAR_AVR_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call    GETVAL,$@,\(......\),$(TYPE_VALS)) \
		F=$(call    GETVAL,$@,.*\(......\)....\.r90,$(FAMILY_VALS)) \
		SU=$(call   GETVAL,$@,.*\(.\)...\.r90,$(SUFFIX_VALS)) \
		M=$(call    GETVAL,$@,.*\(.\)..\.r90,$(MODEL_VALS)) \
		O=$(call    GETVAL,$@,.*\(.\).\.r90,$(OPTION_VALS)) \
		C=$(call    GETVAL,$@,.*\(.\)\.r90,$(CONFIG_VALS)) \
		CVER=$(CVER) \
		CLC=$(CLC)


$(IAR_AVR7_LIBRARIES):
	$(MAKE) -f Makefile2 $@ \
		T=$(call                  GETVAL,$@,\(..\),             $(TYPE_VALS)) \
		F=$(call                  GETVAL,$@,..\(......\),       $(FAMILY_VALS)) \
		C=$(call                  GETVAL,$@,........\(.\),      $(CONFIG_VALS)) \
		V=OSNONE \
		IARAVR_CPU=$(call         GETVAL,$@,..........\(.\),    $(IARAVR_CPU_VALS)) \
		IARAVR_DATA_MODEL=$(call  GETVAL,$@,...........\(.\).*, $(IARAVR_DATA_MODEL_VALS)) \
		IARAVR_CORE=$(call        GETVAL,$@,.*\(-ec\),          $(IARAVR_CORE_VALS)) \
		IARAVR_FLASH=$(call       GETVAL,$@,.*\(-sf\),          $(IARAVR_FLASH_VALS)) \
		IARAVR_DOUBLES=$(call     GETVAL,$@,.*\(-64\),          $(IARAVR_DOUBLES_VALS)) \
		IARAVR_DEBUG=$(call       GETVAL,$@,.*\(-i\),           $(IARAVR_DEBUG_VALS)) \
		CVER=$(CVER) \
		CLC=$(CLC)
		
#
#	end IAR AVR compiler
#
###########################################################################################


###########################################################################################
#
#   targets: libraries for ZDSII EZ8 compiler
#
$(ZLG_EZ8_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call GETVAL,$@,\(..\),$(TYPE_VALS)) \
		F=$(call GETVAL,$@,..\(...\)...\.lib,$(FAMILY_VALS)) \
		O=$(call GETVAL,$@,.*\(.\)..\.lib,$(OPTION_VALS)) \
		M=$(call GETVAL,$@,.*\(.\).\.lib,$(MODEL_VALS)) \
		C=$(call GETVAL,$@,.*\(.\)\.lib,$(CONFIG_VALS)) \
		V=OSNONE \
		CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for TI C24X / C28X compiler
#
$(TI_C2400_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call GETVAL,$@,\(..\),$(TYPE_VALS)) \
		F=$(call GETVAL,$@,..\(......\)..\.lib,$(FAMILY_VALS)) \
		O=$(call GETVAL,$@,.*\(.\).\.lib,$(OPTION_VALS)) \
		C=$(call GETVAL,$@,.*\(.\)\.lib,$(CONFIG_VALS)) \
		V=OSNONE \
		CLC=$(CLC)

$(TI_C2800_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call GETVAL,$@,\(..\),$(TYPE_VALS)) \
		F=$(call GETVAL,$@,..\(......\)...\.lib,$(FAMILY_VALS)) \
		M=$(call GETVAL,$@,.*\(.\)..\.lib,$(MODEL_VALS)) \
		O=$(call GETVAL,$@,.*\(.\).\.lib,$(OPTION_VALS)) \
		C=$(call GETVAL,$@,.*\(.\)\.lib,$(CONFIG_VALS)) \
		V=OSNONE \
		CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for KEIL C51 and 8051 targets
#
$(KEIL_C51_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		 T=$(call GETVAL,$@,\(..\),$(TYPE_VALS)) \
		 F=$(call GETVAL,$@,..\(...\)....\.lib,$(FAMILY_VALS)) \
		 M=$(call GETVAL,$@,.*\(.\)...\.lib,$(MODEL_VALS)) \
		 G=$(call GETVAL,$@,.*\(.\)..\.lib,$(GLOBAL_VALS)) \
		 C=$(call GETVAL,$@,.*\(.\).\.lib,$(CONFIG_VALS)) \
		 V=$(call GETVAL,$@,.*\(.\)\.lib,$(VARIANT_VALS)) \
		 CLC=$(CLC)

$(KEIL_Cx51_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		 T=$(call GETVAL,$@,\(..\),$(TYPE_VALS)) \
		 F=$(call GETVAL,$@,..\(....\)....\.lib,$(FAMILY_VALS)) \
		 M=$(call GETVAL,$@,.*\(.\)...\.lib,$(MODEL_VALS)) \
		 G=$(call GETVAL,$@,.*\(.\)..\.lib,$(GLOBAL_VALS)) \
		 C=$(call GETVAL,$@,.*\(.\).\.lib,$(CONFIG_VALS)) \
		 V=$(call GETVAL,$@,.*\(.\)\.lib,$(VARIANT_VALS)) \
		 CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for HT C51 and 8051 targets
#
$(HT_C51_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		 T=$(call GETVAL,$@,\(..\),$(TYPE_VALS)) \
		 F=$(call GETVAL,$@,..\(..\)....\.lib,$(FAMILY_VALS))  \
		 M=$(call GETVAL,$@,.*\(.\)...\.lib,$(MODEL_VALS)) \
		 G=$(call GETVAL,$@,.*\(.\)..\.lib,$(GLOBAL_VALS)) \
		 C=$(call GETVAL,$@,.*\(.\).\.lib,$(CONFIG_VALS)) \
		 V=$(call GETVAL,$@,.*\(.\)\.lib,$(VARIANT_VALS)) \
		 CLC=$(CLC)

###########################################################################################
#
#   targets: libraries for IAR MSP430 compiler
#
#	salvo[f|l][iar430|iar430x][-|i][a|d|e|m|t].r43
#
$(IAR_430_STD_LIBRARIES) \
$(IAR_430_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call    GETVAL,$@,\(......\),$(TYPE_VALS)) \
		F=$(call    GETVAL,$@,.*\(......\)..\.r43,$(FAMILY_VALS)) \
		O=$(call    GETVAL,$@,.*\(.\).\.r43,$(OPTION_VALS)) \
		C=$(call    GETVAL,$@,.*\(.\)\.r43,$(CONFIG_VALS)) \
		CLC=$(CLC)

$(IAR_430X_STD_LIBRARIES) \
$(IAR_430X_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call    GETVAL,$@,\(......\),$(TYPE_VALS)) \
		F=$(call    GETVAL,$@,.*\(.......\)..\.r43,$(FAMILY_VALS)) \
		O=$(call    GETVAL,$@,.*\(.\).\.r43,$(OPTION_VALS)) \
		C=$(call    GETVAL,$@,.*\(.\)\.r43,$(CONFIG_VALS)) \
		CLC=$(CLC)


###########################################################################################
#
# targets: libraries for Rowley Associates CrossWorks for MSP430 compiler
#
#	libsalvo[f|l][ra430|ra430x][-|i][a|d|e|m|t].hza
#
$(RA_430_STD_LIBRARIES) \
$(RA_430_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call    GETVAL,$@,...\(......\),$(TYPE_VALS)) \
		F=$(call    GETVAL,$@,.*\(.....\)..\.hza,$(FAMILY_VALS)) \
		O=$(call    GETVAL,$@,.*\(.\).\.hza,$(OPTION_VALS)) \
		C=$(call    GETVAL,$@,.*\(.\)\.hza,$(CONFIG_VALS)) \
		CLC=$(CLC)

$(RA_430X_STD_LIBRARIES) \
$(RA_430X_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call    GETVAL,$@,...\(......\),$(TYPE_VALS)) \
		F=$(call    GETVAL,$@,.*\(......\)..\.hza,$(FAMILY_VALS)) \
		O=$(call    GETVAL,$@,.*\(.\).\.hza,$(OPTION_VALS)) \
		C=$(call    GETVAL,$@,.*\(.\)\.hza,$(CONFIG_VALS)) \
		CLC=$(CLC)


###########################################################################################
#
# targets: libraries for IAR ARM compiler
#
#	libsalvo[f|l]iararm[4|][b|l][a|t][-|i][-|i][a|d|e|m|t].a
#
$(IAR_ARM_STD_LIBRARIES) \
$(IAR_ARM_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call    GETVAL,$@,...\(......\),$(TYPE_VALS)) \
		F=$(call    GETVAL,$@,.*\(......\)......\.a,$(FAMILY_VALS)) \
		SU=$(call   GETVAL,$@,.*\(.\).....\.a,$(SUFFIX_VALS)) \
		END=$(call  GETVAL,$@,.*\(.\)....\.a,$(ENDIAN_VALS)) \
		ISET=$(call GETVAL,$@,.*\(.\)...\.a,$(INSTRUCTION_SET_VALS)) \
		IW=$(call   GETVAL,$@,.*\(.\)..\.a,$(INTERWORK_VALS)) \
		O=$(call    GETVAL,$@,.*\(.\).\.a,$(OPTION_VALS)) \
		C=$(call    GETVAL,$@,.*\(.\)\.a,$(CONFIG_VALS)) \
		CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for MPLAB C30 compiler
#
#	libsalvo[f|l]mcc30[s|l][-|i][a|d|e|m|t].a
#
$(MCC_PIC24_STD_LIBRARIES) \
$(MCC_PIC24_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call    GETVAL,$@,...\(......\),$(TYPE_VALS)) \
		F=$(call    GETVAL,$@,.*\(.....\)...\.a,$(FAMILY_VALS)) \
		M=$(call    GETVAL,$@,.*\(.\)..\.a,$(MODEL_VALS)) \
		O=$(call    GETVAL,$@,.*\(.\).\.a,$(OPTION_VALS)) \
		C=$(call    GETVAL,$@,.*\(.\)\.a,$(CONFIG_VALS)) \
		CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for MPLAB C32 compiler
#
#	libsalvo[f|l]mcc32[l|s][-|i][a|d|e|m|t].a
#
$(MCC_PIC32_STD_LIBRARIES) \
$(MCC_PIC32_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call    GETVAL,$@,...\(......\),$(TYPE_VALS)) \
		F=$(call    GETVAL,$@,.*\(.....\)...\.a,$(FAMILY_VALS)) \
		ISET=$(call GETVAL,$@,.*\(.\)..\.a,$(INSTRUCTION_SET_VALS)) \
		O=$(call    GETVAL,$@,.*\(.\).\.a,$(OPTION_VALS)) \
		C=$(call    GETVAL,$@,.*\(.\)\.a,$(CONFIG_VALS)) \
		CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for ARM RealView C compiler and ARM7TDMI
#
#	salvo[f|l]armrv4[b|l][a|t][-|i][-|i][a|d|e|m|t].lib
#
$(ARM_RV4_STD_LIBRARIES) \
$(ARM_RV4_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call    GETVAL,$@,\(......\),$(TYPE_VALS)) \
		F=$(call    GETVAL,$@,.*\(armrv4\).....\.lib,$(FAMILY_VALS)) \
		END=$(call  GETVAL,$@,.*\(.\)....\.lib,$(ENDIAN_VALS)) \
		ISET=$(call GETVAL,$@,.*\(.\)...\.lib,$(INSTRUCTION_SET_VALS)) \
		IW=$(call   GETVAL,$@,.*\(.\)..\.lib,$(INTERWORK_VALS)) \
		O=$(call    GETVAL,$@,.*\(.\).\.lib,$(OPTION_VALS)) \
		C=$(call    GETVAL,$@,.*\(.\)\.lib,$(CONFIG_VALS)) \
		CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for ARM RealView C compiler and Cortex M3
#
#	salvo[f|l]armrvcm3[-|i][a|d|e|m|t].a
#
$(ARM_RVCM3_STD_LIBRARIES) \
$(ARM_RVCM3_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call    GETVAL,$@,\(......\),$(TYPE_VALS)) \
		F=$(call    GETVAL,$@,.*\(armrvcm3\)..\.lib,$(FAMILY_VALS)) \
		O=$(call    GETVAL,$@,.*\(.\).\.lib,$(OPTION_VALS)) \
		C=$(call    GETVAL,$@,.*\(.\)\.lib,$(CONFIG_VALS)) \
		CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for GCC ARM C compiler
#
#	libsalvo[f|l]gccarm[3|4|5][b|l][a|t][-|i][a|d|e|m|t].a
#
$(GCC_ARM_STD_LIBRARIES) \
$(GCC_ARM_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call    GETVAL,$@,...\(......\),$(TYPE_VALS)) \
		F=$(call    GETVAL,$@,.*\(......\).....\.a,$(FAMILY_VALS)) \
		SU=$(call   GETVAL,$@,.*\(.\)....\.a,$(SUFFIX_VALS)) \
		END=$(call  GETVAL,$@,.*\(.\)...\.a,$(ENDIAN_VALS)) \
		ISET=$(call GETVAL,$@,.*\(.\)..\.a,$(INSTRUCTION_SET_VALS)) \
		IW=$(call   GETVAL,$@,.*\(.\).\.a,$(INTERWORK_VALS)) \
		C=$(call    GETVAL,$@,.*\(.\)\.a,$(CONFIG_VALS)) \
		CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for GCC ARM C compiler
#
#	libsalvo[f|l]gccarmcm3[a|d|e|m|t].a
#
$(GCC_ARMCM3_STD_LIBRARIES) \
$(GCC_ARMCM3_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call    GETVAL,$@,...\(......\),$(TYPE_VALS)) \
		F=$(call    GETVAL,$@,.*\(.........\).\.a,$(FAMILY_VALS)) \
		C=$(call    GETVAL,$@,.*\(.\)\.a,$(CONFIG_VALS)) \
		CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for ImageCraft ARM C compiler
#
#	libsalvo[f|l]iccarm[3|4|5][b|l][a|t][-|i][-|i][a|d|e|m|t].a
#
$(ICC_ARM_STD_LIBRARIES) \
$(ICC_ARM_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call    GETVAL,$@,...\(......\),$(TYPE_VALS)) \
		F=$(call    GETVAL,$@,.*\(......\)......\.a,$(FAMILY_VALS)) \
		SU=$(call   GETVAL,$@,.*\(.\).....\.a,$(SUFFIX_VALS)) \
		END=$(call  GETVAL,$@,.*\(.\)....\.a,$(ENDIAN_VALS)) \
		ISET=$(call GETVAL,$@,.*\(.\)...\.a,$(INSTRUCTION_SET_VALS)) \
		IW=$(call   GETVAL,$@,.*\(.\)..\.a,$(INTERWORK_VALS)) \
		O=$(call    GETVAL,$@,.*\(.\).\.a,$(OPTION_VALS)) \
		C=$(call    GETVAL,$@,.*\(.\)\.a,$(CONFIG_VALS)) \
		CLC=$(CLC)


###########################################################################################
#
#   targets: libraries for Keil CARM compiler
#
#	salvo[f|l]kcarm[4|][b|l][a|t][-|i][a|d|e|m|t].lib
#
$(KEIL_ARM_STD_LIBRARIES) \
$(KEIL_ARM_OPT_LIBRARIES) :
	$(MAKE) -f Makefile2 $@ \
		T=$(call    GETVAL,$@,\(......\),$(TYPE_VALS)) \
		F=$(call    GETVAL,$@,.*\(.....\).....\.lib,$(FAMILY_VALS)) \
		SU=$(call   GETVAL,$@,.*\(.\)....\.lib,$(SUFFIX_VALS)) \
		END=$(call  GETVAL,$@,.*\(.\)...\.lib,$(ENDIAN_VALS)) \
		ISET=$(call GETVAL,$@,.*\(.\)..\.lib,$(INSTRUCTION_SET_VALS)) \
		IW=$(call   GETVAL,$@,.*\(.\).\.lib,$(INTERWORK_VALS)) \
		C=$(call    GETVAL,$@,.*\(.\)\.lib,$(CONFIG_VALS)) \
		CLC=$(CLC)




