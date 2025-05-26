  INCLUDE core_cm4_constants.s        ; Load Constant Definitions
      INCLUDE stm32l476xx_constants.s      

      IMPORT      System_Clock_Init
      IMPORT      UART2_Init
      IMPORT      USART2_Write
      IMPORT 	  config_NVIC_in_C
        
     
      AREA    main, CODE, READONLY
      EXPORT      __main                        ; make __main visible to linker
	  EXPORT SysTick_Handler
        ENTRY      
   

;door open close angle = 257 (180 degrees)
floorOneAngle EQU 0 ;move one floor
floorTwoAngle EQU 514
floorThreeAngle EQU 1028
floorFourAngle EQU 1542
currentFloorAngle EQU 0

systick_init PROC
	
;;;;; SysTick Initialization ;;;;;;;;;;;;;;;
        
	  ;Disable SysTick
	  LDR r1, =SysTick
	  LDR r2, =0x0
	  STR r2, [r1, #0]
	  
	  ;Set Reload
	  LDR r1, =SysTick
	  LDR r0, =reload
	  SUB r0, r0, #1
	  STR r0, [r1, #4]
	  
	  ;Reset Counter Value
	  LDR r1, =SysTick
	  LDR r2, =0x0
	  STR r2, [r1, #8]
	  
	  ;Set Interrupt Priority
	  PUSH {LR, R0}
	  BL config_NVIC_in_C
	  POP {LR, R0}
	  LDR r1, =SysTick
	  LDR r2, [r1, #0]
	  
	  ;Enable SysTick Interrupt
	  ORR r2, #0x02
	  STR r2, [r1, #0]
	  
	  ;Select Processor Clock
	  LDR r1, =SysTick
	  LDR r2, [r1, #0]
	  ORR r2, #0x04
	  STR r2, [r1, #0]
	  
	  ;Enable SysTick
	  LDR r1, =SysTick
	  LDR r2, [r1, #0]
	  ORR r2, #0x01
	  STR r2, [r1, #0]
	  
	  BX LR
	  
	  ENDP
	  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

                     
__main      PROC
	
	BL systick_init
   

;*********INITIALIZATION*******************;    
      BL System_Clock_Init
      BL UART2_Init
 
       
    ; Enable clock for GPIO port B and port C
    LDR R0, =RCC_BASE          
    LDR R1, [r0, #RCC_AHB2ENR]  
    ORR R1, R1,#0x6              
    STR R1, [R0, #RCC_AHB2ENR]
   
      ;initialize reset button
      LDR r0,=GPIOC_BASE
      LDR R1,[R0,#GPIO_MODER]
      BIC R1,R1,#0x00300000
      STR R1,[R0,#GPIO_MODER]
      
    ; Configure PC0-PC3 as output
    LDR R0, =GPIOC_BASE
    LDR R1, [R0,#GPIO_MODER]
    BIC R1, R1, #0x000000FF  ; Clear bits 0-3 for PC0-PC3
    ORR R1, R1, #0x00000055        ; Set bits 0-3 for PC0-PC3 as output
    STR R1, [R0,#GPIO_MODER]
     
    ;configure PC 4 5 6 7 as SSD output
    LDR R0,=GPIOC_BASE
    LDR R1,[R0,#GPIO_MODER]
    BIC R1,R1,#0x0000FF00
    ORR R1,R1,#0x00005500
    STR R1,[R0,#GPIO_MODER]
     
    ; Configure PB0, PB1, PB5, PB8 as input
    LDR R0, =GPIOB_BASE
    LDR R1, [R0,#GPIO_MODER]
    BIC R1,R1,#0x00030000
    BIC R1,R1,#0x00000C00
    BIC R1,R1,#0x0000000F
    STR R1, [R0,#GPIO_MODER]
     
    ;configure PB 2 3 6 7 for stepper motor output and PB 12 13 14 15
    LDR r0, =GPIOB_BASE
    LDR r1, [r0, #GPIO_MODER]
    BIC r1, #0xFF000000
    BIC r1, #0x0000F000
    BIC r1, #0x000000F0
    MOV r2, #0x00005050
    ORR r1, r2
    MOV r2, #0x55000000
    ORR R1,R2
    STR r1, [r0, #GPIO_MODER]
    LDR r1, [r0, #GPIO_OTYPER]
    BIC R1,#0xF000
    BIC r1, #0x000000CC
    STR r1, [r0, #GPIO_OTYPER]
     
      ;configure input and output for keypad
      ;PA 0 1 4 6 as input
      ;initializing GPIOA (input)
      LDR R5, =GPIOA_BASE           ;load base address of GPIOB    
      LDR R6, [R5, #GPIO_MODER]     ;load offset value
      BIC R6,R6, #0x0000000F              ;bit clear bits
      BIC R6,R6,#0x00003300         ;bit clear
      STR R6, [R5, #GPIO_MODER]     ;store the modified value
     
      ;initializing GPIOB (output for keypad)
      ;PB 4 9 10 11
      LDR R6, =GPIOB_BASE           ;load base address    
      LDR R7, [R6, #GPIO_MODER]     ;load in offset value
      BIC R7,R7,#0x00F00000
      BIC R7,R7,#0x000C0000
      BIC R7,R7,#0x00000300        ;clear bits
      ORR R7,R7,#0x00500000
      ORR R7,R7,#0x00040000
      ORR R7,R7,#0x00000100        ;set bits                
      STR R7, [R6, #GPIO_MODER] ;store modified value
        
     
     
      LDR R10,=floorOneAngle; current floor angle
      LDR R11,=floorOneAngle ; called floor angle
        BL floor1_reached

     
     
main_loop
;************************CHECK EXTERNAL BUTTON PRESS*************************
   ; Check if BUTTON 1 (PB1) is pressed
    LDR R0, =GPIOB_BASE

    LDR R1, [R0, #GPIO_IDR]
    BL delay
    AND R1,R1,#0x1
    CMP R1,#0x1; Test bit 1 for PB0
    BEQ button1_pressed      ; Branch if PB0 is pressed
    MOV R2,#0x00000FFF
    BIC R1,R1,R2
    ; Check if BUTTON 2 (PB1) is pressed
 
    LDR R1, [R0, #GPIO_IDR]
    AND R1,R1,#0x2
    CMP R1,#0x2; Test bit 2 for PB1
    BEQ button2_pressed      ; Branch if PB2 is pressed
    MOV R2,#0x00000FFF
    BIC R1,R1,R2
    ; Check if BUTTON 3 (PB5) is pressed
   
    LDR R1, [R0, #GPIO_IDR]
    AND R1,R1,#0x20
    CMP R1,#0x20; Test bit 3 for PB5
    BEQ.W button3_pressed      ; Branch if PB5 is pressed
    MOV R2,#0x00000FFF
    BIC R1,R1,R2
    ; Check if BUTTON 4 (PB5) is pressed
   
    LDR R1, [R0, #GPIO_IDR]
    AND R1,R1,#0x100
    CMP R1,#0x100 ; Test bit 8 for PB8
    BEQ.W button4_pressed      ; Branch if PB5 is pressed
    MOV R2,#0x00000FFF
    BIC R1,R1,R2
      
	;Check if Reset is pushed
      LDR R0,=GPIOC_BASE
      
      LDR R1,[R0,#GPIO_IDR]
      AND R1,R1,#0x400
      CMP R1,#0x400
      BLEQ resetPressed
      BEQ.W button1_pressed
      MOV R2,#0xFFF
      BIC R1,R1,R2
    ;check if keypad has been pressed
    BL keypadScan
    ; Repeat main loop
    B main_loop
      LTORG

;***************BUTTON 1 PRESSED**********************
button1_pressed
      ;set called floor
      LDR R11,=floorOneAngle
    ; Turn on LED 1
    LDR R0, =GPIOC_BASE
    LDR R1, [R0,#GPIO_ODR]
    BIC R1,R1,#0x0000000F
    ORR R1, R1, #0x01      ; Set bit 0 for LED 1
    STR R1, [R0,#GPIO_ODR]
    BL delay
    LDR R0, =message_floor1
    MOV R1, #14               ; write to tera term
    BL USART2_Write
    BL delay
    ;compare current and called floors (will always move down or not at all to floor 1)
    CMP R11,R10
    BEQ openDoor1
    BNE moveDown
moveDown
    RSB R2, R11,R10
move_down
    PUSH {r2}   ;preserve max angle
      LDR R1,=floorThreeAngle
      CMP R2,R1
      BLEQ floor3_reached
      LDR R1,=floorTwoAngle
      CMP R2,R1
      BLEQ floor2_reached
    BL bstep2       ; Move down one step and update angle
    POP {r2}      ;restore r2 
      SUBS R2,#1
      BNE move_down  ; If not a multiple of 514, continue moving down
    B openDoor1
openDoor1
    ;set current floor
    LDR R10,=floorOneAngle
      PUSH {LR}
      BL floor1_reached
      POP {LR}
    ;open door
    B openDoor
    LTORG
;**********************BUTTON 2 PRESSED*********************
button2_pressed
           
    ;set called floor
    LDR R11,=floorTwoAngle
    ; Turn on LED 2
    LDR R0, =GPIOC_BASE
    LDR R1, [R0,#GPIO_ODR]
    BIC R1,R1,#0x0000000F
    ORR R1, R1, #0x00000002      ; Set bit 1 for LED 2
    STR R1, [R0,#GPIO_ODR]
    BL delay
    LDR R0, =message_floor2
    MOV R1, #14               ; write to tera term
    BL USART2_Write
    BL delay
    ;decide if needs to move up or move down
    CMP R11,R10
    BLT moveDown2
    BGT moveUp2
    BEQ openDoor2
moveUp2
      SUB R2,R11,R10
move_up2
    PUSH {R2}     ;preserve max angle
    BL fstep2       ; Move down one step and update angle
    POP {R2}      ;restore r2
      SUBS R2, #1
    BNE move_up2  ;will continue to move forward until the angle is 0
    B openDoor2
moveDown2
      RSB R2, R11,R10
move_down2
    PUSH {r2}   ;preserve max angle
      LDR R1,=floorTwoAngle
      CMP R2,R1
      BLEQ floor3_reached
    BL bstep2       ; Move down one step and update angle
    POP {r2}      ;restore r2
      SUBS r2, #1
    BNE move_down2  ;will continue to move forward until the angle is 0
    B openDoor2
openDoor2
    ;set current floor
    LDR R10, =floorTwoAngle
      PUSH {LR}
      BL floor2_reached
      POP {LR}
    B openDoor
    LTORG  
;**********************BUTTON 3 PRESSED****************************
button3_pressed
    ;set called floor
    LDR R11,=floorThreeAngle
    ; Turn on LED 3
    LDR R0, =GPIOC_BASE
    LDR R1, [R0,#GPIO_ODR]
    BIC R1,R1,#0x0000000F
    ORR R1, R1, #0x04      ; Set bit 2 for LED 3
    STR R1, [R0,#GPIO_ODR]
    BL delay
    LDR R0, =message_floor3
    MOV R1, #14               ; write to tera term
    BL USART2_Write
    BL delay
    ;decide if needs to move up or move down
    CMP R11,R10
    BGT moveUp3
    BLT moveDown3
    BEQ openDoor3
     
moveUp3
      SUB R2,R11,R10
move_up3
    PUSH {R2}     ;preserve max angle
      LDR R1,=floorTwoAngle
      CMP R2,R1
      BLEQ floor2_reached
    BL fstep2       ; Move down one step and update angle
    POP {R2}      ;restore
      SUBS R2, #1
    BNE move_up3  ;will continue to move forward until the angle is 0
    B openDoor3
moveDown3
      RSB R2, R11,R10
move_down3
      PUSH {r2}   ;preserve max angle
    BL bstep2       ; Move down one step and update angle
    POP {r2}      ;restore r2
    SUBS R2, #1
    BNE move_down3  ;will continue to move forward until the angle is 0
    B openDoor3
openDoor3
    LDR R10, =floorThreeAngle
      PUSH {LR}
      BL floor3_reached
      POP {LR}
    ;set current floor
    B openDoor
    LTORG
;****************************BUTTON 4 PRESSED************************
button4_pressed
    ;set called floor
    LDR R11,=floorFourAngle
    ; Turn on LED 4
    LDR R0, =GPIOC_BASE
    LDR R1, [R0,#GPIO_ODR]
    BIC R1,R1,#0x0000000F
    ORR R1, R1, #0x08      ; Set bit 3 for LED 4
    STR R1, [R0,#GPIO_ODR]
    BL delay
    ; Delay
    LDR R0, =message_floor4
    MOV R1, #14               ;write to tera term
    BL USART2_Write  
    BL delay
    CMP R11,R10
    BNE moveUp4
    BEQ openDoor4
moveUp4
      SUB R2,R11,R10
move_up4
      
    PUSH {R2}     ;preserve max angle
      LDR R1,=floorThreeAngle
      CMP R2,R1
      BLEQ floor2_reached
      LDR R1,=floorTwoAngle
      CMP R2,R1
      BLEQ floor3_reached
      
    BL fstep2       ; Move down one step and update angle
      POP {R2}      ;restore r2
    SUBS R2, #1
    BNE move_up4  ;will continue to move forward until the angle is 0
    B openDoor4
openDoor4
      ;set current floor
    LDR R10,=floorFourAngle
      PUSH {LR}
      BL floor4_reached
      POP {LR}
    B openDoor
resetPressed
      LDR R0, =resetP
    MOV R1, #13        ; Write to Tera Term
      PUSH{LR}
    BL USART2_Write
      POP {LR}
      B button1_pressed

floor1_reached
    ; Write floor 1 message to Tera Term
    LDR R0, =reachedOne
    MOV R1, #10        ; Write to Tera Term
      PUSH {LR}
    BL USART2_Write
      POP {LR}
      LDR r0,=GPIOC_BASE
      LDR r1, [r0, #GPIO_ODR]   ; Load in the ODR register address into r1 from r0
      BIC r1, #0xF0           ; Clear the bits from the r1 register
      ORR R1,R1,#0x10
      STR r1, [r0, #GPIO_ODR]   ; Store the data back from r1 to r0 with the ODR shift
    BX LR

floor2_reached
    ; Write floor 2 message to Tera Term
    LDR R0, =reachedTwo
    MOV R1, #10        ; Write to Tera Term
      PUSH{LR}
    BL USART2_Write
      POP {LR}
      LDR r0,=GPIOC_BASE
      LDR r1, [r0, #GPIO_ODR]   ; Load in the ODR register address into r1 from r0
      BIC r1, #0xF0           ; Clear the bits from the r1 register
      ORR R1,R1,#0x20
      STR r1, [r0, #GPIO_ODR]   ; Store the data back from r1 to r0 with the ODR shift
    BX LR

floor3_reached
    ; Write floor 3 message to Tera Term
    LDR R0, =reachedThree
    MOV R1, #10        ; Write to Tera Term
      PUSH {LR}
    BL USART2_Write
      POP {LR}
      LDR r0,=GPIOC_BASE
      LDR r1, [r0, #GPIO_ODR]   ; Load in the ODR register address into r1 from r0
      BIC r1, #0xF0           ; Clear the bits from the r1 register
      ORR R1,R1,#0x30
      STR r1, [r0, #GPIO_ODR]   ; Store the data back from r1 to r0 with the ODR shift
      BX LR

floor4_reached
    ; Write floor 4 message to Tera Term
    LDR R0, =reachedFour
    MOV R1, #10        ; Write to Tera Term
      PUSH {LR}
    BL USART2_Write
      POP {LR}
      LDR r0,=GPIOC_BASE
      LDR r1, [r0, #GPIO_ODR]   ; Load in the ODR register address into r1 from r0
      BIC r1, #0xF0           ; Clear the bits from the r1 register
      ORR R1,R1,#0x40
      STR r1, [r0, #GPIO_ODR]   ; Store the data back from r1 to r0 with the ODR shift
      BX LR        

;**********************OPEN DOOR***************************************
openDoor
    LDR R0, =arrived
    MOV R1, #7               ;write to tera term
    BL USART2_Write  
    ;turn off arrival light
    LDR R0, =GPIOC_BASE
    LDR R1, [R0,#GPIO_ODR]
    BIC R1,R1,#0x0000000F
    STR R1, [R0,#GPIO_ODR]
    LDR r2, =257                  ;this is the max angle to compare to after forward and backward steps
move_forward
    PUSH {r2}     ;preserve max angle
    BL fstep      ;one forward step
    POP {r2}      ;restore r2
    SUBS r2, #1
    BNE move_forward    ;will continue to move forward until the angle is 0
    BL delay
    LDR R0, =doorOpen
    MOV R1, #12               ;write to tera term
    BL USART2_Write
longdelay
    ; Delay loop
    MOV r8, #0xFFFF           ; Initialize delay counter
    LSL r8, #9                      ;modify this value to change the speed of the delay
longdelay_loop
    SUBS r8, r8, #1           ; Decrement delay counter
    CMP r8,#0x0
    BNE longdelay_loop            ; Repeat delay loop until r8 becomes zero
;************************CLOSE DOOR*******************************8
doorCloseF
    LDR r2, =257
move_backward
    PUSH {r2}     ;preserve max angle
    BL bstep;ne forward step
    POP {r2}      ;restore r2
    SUBS r2, #1
    BNE move_backward;ntinue to move forward until the angle is 0
    BL delay
    LDR R0, =doorClose
    MOV R1, #12               ;write to tera term
    BL USART2_Write  
    BL delay
;****make registers hold a value if a person is in the elevator and compare those values to branch accordingly;
    B main_loop              ; Branch to main_loop if not equal (button pressed)
fstep ;completes one forward step of the motor
    ;************ A AND NOT B***********
    LDR r3, =GPIOB_BASE
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, #0x000000CC
    ORR r4, #0x00000084
    STR r4, [r3, #GPIO_ODR]
    PUSH {LR}
    BL delay
    POP {LR}
      ;************A AND B******************
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0x000000CC
    ORR r4, r4, #0x00000044
    STR r4, [r3, #GPIO_ODR]
    PUSH {LR}
    BL delay
    POP {LR}
      ;*************NOT A AND B***************
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0x000000CC
    ORR r4, r4, #0x00000048
    STR r4, [r3, #GPIO_ODR]
    PUSH {LR}
    BL delay
    POP {LR}
      ;*****NOT A AND NOT B ********************
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0x000000CC
    ORR r4, r4, #0x00000088
    STR r4, [r3, #GPIO_ODR]
    PUSH {LR}
    BL delay
    POP {LR}
      ;*******RESET ALL**********
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0x000000CC
    STR r4, [r3, #GPIO_ODR]
    BX LR
bstep ;perform the opposite order of forward to complete one backstep of the motor
      ;*************NOT A AND NOT B****************
    LDR r3, =GPIOB_BASE
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0x000000CC
    ORR r4, r4, #0x00000088
    STR r4, [r3, #GPIO_ODR]
    PUSH {LR}
    BL delay
    POP {LR}
      ;**************NOT A AND B*****************
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0x000000CC
    ORR r4, r4, #0x00000048
    STR r4, [r3, #GPIO_ODR]
    PUSH {LR}
    BL delay
    POP {LR}
      ;*****************A AND NOT B******************
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0x000000CC
    ORR r4, r4, #0x00000044
    STR r4, [r3, #GPIO_ODR]
    PUSH {LR}
    BL delay
    POP {LR}
      ;*************A AND B*******************
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0x000000CC
    ORR r4, r4, #0x00000084
    STR r4, [r3, #GPIO_ODR]
    PUSH {LR}
    BL delay
    POP {LR}
      ;************** RESET ALL************
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0x000000CC
    STR r4, [r3, #GPIO_ODR]
    BX LR
;************************KEY PAD PRESS*******************************
keypadScan
     
      BL delay
; Reset the ODR to look for any down button
      LDR r7, [r6, #GPIO_ODR]    
      MOV r7, #0x00000000              ;pulls all rows low  
      STR r7, [r6, #GPIO_ODR]    
      BL delay                      ;debounce                        
      LDR r8, [r5, #GPIO_IDR]  
      AND r8, #0x00000053    ;anding with the corresponding pins will result in the same mask if nothing has been pressed    
      CMP r8, #0x00000053            
      BEQ main_loop
      MOV r8, #0x00000E00 ;pull row 1 low                  
      STR r8, [r6, #GPIO_ODR]    
      BL delay                      
      MOV r8, #0x00000000           ;reset IDR  
      LDR r8, [r5, #GPIO_IDR]
      AND r9, r8, #0x00000001     ;mask for pin 1 (floor 1 press)      
      CMP r9, #0x00000001
     
      MOVNE R12,#0x0
      BNE button1_pressed      
      AND r9, r8, #0x00000002   ;mask for pin 2 (floor 2 press)    
      CMP r9, #0x00000002
     
      MOVNE R12,#0x0
      BNE button2_pressed        
      AND r9, r8, #0x00000010  ;mask for pin 3 (floor 3 press)      
      CMP r9, #0x00000010
     
      MOVNE R12,#0x0
      BNE button3_pressed    
      MOV r8, #0x00000C10       ;pull row 2 low      
      STR r8, [r6, #GPIO_ODR]    
      BL delay                
      MOV r8, #0x00000000           ;clear input
      LDR r8, [r5, #GPIO_IDR]  
      AND r9, r8, #0x00000001   ;row 2 pin 1 (floor 4 press)    
      CMP r9, #0x00000001
     
      MOVNE R12,#0x0
      BNE button4_pressed  

bstep2 ;completes one forward step of the motor
      ;************ A AND NOT B***********
    LDR r3, =GPIOB_BASE
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, #0xF000
    ORR r4, #0x9000
    STR r4, [r3, #GPIO_ODR]
    PUSH {LR}
    BL delay
    POP {LR}
      ;************A AND B******************
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0xF000
    ORR r4, r4, #0x5000
    STR r4, [r3, #GPIO_ODR]
    PUSH {LR}
    BL delay
    POP {LR}
      ;*************NOT A AND B***************
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0xF000
    ORR r4, r4, #0x6000
    STR r4, [r3, #GPIO_ODR]
    PUSH {LR}
    BL delay
    POP {LR}
      ;*****NOT A AND NOT B ********************
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0xF000
    ORR r4, r4, #0xA000
    STR r4, [r3, #GPIO_ODR]
    PUSH {LR}
    BL delay
    POP {LR}
      ;*******RESET ALL**********
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0xF000
    STR r4, [r3, #GPIO_ODR]
    BX LR
fstep2      ;perform the opposite order of forward to complete one backstep of the motor
      ;*************NOT A AND NOT B****************
    LDR r3, =GPIOB_BASE
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0xF000
    ORR r4, r4, #0xA000
    STR r4, [r3, #GPIO_ODR]
    PUSH {LR}
    BL delay
    POP {LR}
      ;**************NOT A AND B*****************
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0xF000
    ORR r4, r4, #0x6000
    STR r4, [r3, #GPIO_ODR]
    PUSH {LR}
    BL delay
    POP {LR}
      ;*****************A AND B******************
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0xF000
    ORR r4, r4, #0x5000
    STR r4, [r3, #GPIO_ODR]
    PUSH {LR}
    BL delay
    POP {LR}
      ;*************A AND NOT B*******************
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0xF000
    ORR r4, r4, #0x9000
    STR r4, [r3, #GPIO_ODR]
    PUSH {LR}
    BL delay
    POP {LR}
      ;************** RESET ALL************
    LDR r4, [r3, #GPIO_ODR]
    BIC r4, r4, #0xF000
    STR r4, [r3, #GPIO_ODR]
    BX LR

delay       ;delay
    LDR r2, =90000
delay_loop
    SUBS r2, #1
    BNE delay_loop
    BX LR    
      ENDP
		  
SysTick_Handler PROC

	PUSH {LR, R1, R2}

	POP {LR, R1, R2}
	BX LR

	  ENDP
		  
		  
		  
		  
      ALIGN            
      AREA myData, DATA, READWRITE
      ALIGN
resetP DCB "Reset Pressed",0
currentAngle DCD 0x2000
reachedOne DCB "At Floor 1",0
reachedTwo DCB "At floor 2",0
reachedThree DCB "At floor 3",0
reachedFour DCB "At floor 4",0
arrived DCB "arrived",0
movingDown DCB "moving down",0
movingUp DCB "moving Up",0
doorOpen DCB "door opening",0
doorClose DCB "door closing",0
keypad1 DCB "moving to floor 1",0
keypad2 DCB "moving to floor 2",0
keypad3 DCB "moving to floor 3",0
keypad4 DCB "moving to floor 4",0
message_floor1 DCB "Floor 1 called", 0
message_floor2 DCB "Floor 2 called", 0  
message_floor3 DCB "Floor 3 called", 0  
message_floor4 DCB "Floor 4 called", 0  
reload DCD 159999
      END