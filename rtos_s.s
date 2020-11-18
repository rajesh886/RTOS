;-----------------------------------------------------------------------------
; Device includes, defines, and assembler directives
;-----------------------------------------------------------------------------

   .def usermode
   .def privilegedmode
   .def setPSP
   .def setASP
   .def getPSP
   .def getMSP


;-----------------------------------------------------------------------------
; Register values and large immediate values
;-----------------------------------------------------------------------------

.thumb
.const
PSP_LOCATION  .field 0x20008000


;-----------------------------------------------------------------------------
; Subroutines
;-----------------------------------------------------------------------------

.text

usermode:
			   MRS    R0, CONTROL  			 ; get pointer to control register
               ORR    R0, #0x1               ; set the TMPL bit in the control register
               MSR    CONTROL, R0
               BX     LR                     ; return from subroutine

privilegedmode:
               MRS    R4, CONTROL  			 ; get pointer to control register
               AND    R4, #0x00               ; set the TMPL bit in the control register
               MSR    CONTROL, R4
               BX     LR                     ; return from subroutine

setPSP:

               MSR    PSP, R0
               DSB
               ISB
               BX     LR                     ; return from subroutine


setASP:
			   MRS    R0, CONTROL  			 ; get pointer to control register
               ORR    R0, R0, #2               ; set the ASP bit in the control register
               MSR    CONTROL, R0
               DSB
               ISB
               BX     LR                     ; return from subroutine

getPSP:
			   MRS    R0, PSP  			 	 ; get pointer to PSP register
               BX     LR                     ; return from subroutine

getMSP:
			   MRS    R0, MSP  			 	 ; get pointer to MSP register
               BX     LR                     ; return from subroutine

.endm
