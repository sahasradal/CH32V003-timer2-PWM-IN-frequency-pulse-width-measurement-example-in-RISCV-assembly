# Timer1 base time for 1 second.result displayed in decimal format in USART,leading zero suppressed and print left aligned to terminal screen ,works fine
# zero suppress is good for LCD etc, left aligning by shift print is good for terminal screens.
# division with fraction is used, routine has 1/1000 precision
# Timer2 external mode2 ,Default mapping (CH1/ETR/PD4, CH2/PD3,CH3/PC0, CH4/PD7). 
#BRR = BAUD  #13.6=38400,#4.4 = 115200,#52.1 = 9600,#8.7 =  57600
#USARTx_TX Full-duplex mode Push-pull multiplexed outputs,#USARTx_RX Full-duplex mode Floating input or pull-up input
#PD5 -tx
#PD6 -rx
#PD4 - ETR pin for measuring frequency


# 00: Default mapping (CH1/ETR/PD4, CH2/PD3,CH3/PC0, CH4/PD7).PWM-IN mode, input on pd4 ch1, works fine,display decimzl
# 1HZ granularity from 15Hz to 1000Hz (1kHZ) signal, above 1khz 2hz resolution ,above 2kz 4 hz resolution
#BRR = BAUD  
#13.6=38400,#4.4 = 115200,#52.1 = 9600,#8.7 =  57600
#USARTx_TX Full-duplex mode Push-pull multiplexed outputs,#USARTx_RX Full-duplex mode Floating input or pull-up input
#PD5 -tx,#PD6 -rx,#PD4 -input
fclk = 24000000   		# 24Mhz RCO internal , AHB =8Mhz by default
state = 0x2000000C 		# located in SRAM
result1 = 0x20000010 		# 0x20000010 to 0x20000018 is used for storing result in decimal format
result2 = 0x20000014
fraction2 = 0x20000018
scratch = 0x2000001C
result_lo = 0x20000020
result_hi = 0x20000024
modulo = 0x20000028
fraction = 0x2000002C
result_lo_m = 0x20000030
result_hi_m = 0x20000034
dividend = 0x20000038 
divisor = 0x2000003C
scratch = 0x20000040
mem = 0x20000044

include CH32V003_reg1.asm

vtable:
	j reset_handler		#  longs 0x00000000 # RESERVED 0
align 4
  longs   0x00000000 # RESERVED 1
  longs   0x00000000 #pack <l longs NMI_IRQhandler
  longs   0x00000000 #pack <l HardFault_IRQhandler
  longs   0x00000000 # RESERVED 4
  longs   0x00000000 # RESERVED 5
  longs   0x00000000 # RESERVED 6
  longs   0x00000000 # RESERVED 7
  longs   0x00000000 # RESERVED 8
  longs   0x00000000 # RESERVED 9
  longs   0x00000000 # RESERVED 10
  longs   0x00000000 # RESERVED 11
  longs   0x00000000 # pack <l SysTick_IRQhandler	#; place the address of the mtime ISR subroutine in the vector table position 7,assembler will store isr address here, longs 0x00000000 # RESERVED 12	
  longs   0x00000000 # RESERVED 13
  longs   0x00000000 #pack <l SW_Software_IRQhandler
  longs   0x00000000 # RESERVED 15
  longs   0x00000000 #pack <l WWDG_IRQhandler
  longs   0x00000000 #pack <l PVD_IRQhandler
  longs   0x00000000 #pack <l FLASH_IRQhandler
  longs   0x00000000 #pack <l RCC_IRQhandler
  longs   0x00000000 #pack <l EXTI7_0_IRQhandler
  longs   0x00000000 #pack <l AWU_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH1_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH2_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH3_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH4_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH5_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH6_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH7_IRQhandler
  longs   0x00000000 #pack <l ADC1_IRQhandler
  longs   0x00000000 #pack <l I2C1_EV_IRQhandler
  longs   0x00000000 #pack <l I2C1_ER_IRQhandler
  longs   0x00000000 #pack <l USART1_IRQhandler
  longs   0x00000000 #pack <l SPI1_IRQhandler
  longs   0x00000000 #pack <l TIM1BRK_IRQhandler
  longs   0x00000000 #pack <l TIM1UP_IRQhandler
  longs   0x00000000 #pack <l TIM1TRG_COM_IRQhandler
  longs   0x00000000 #pack <l TIM1CC_IRQhandler
  longs   0x00000000 #pack <l TIM2_IRQhandler

reset_handler:


    	li sp, STACK			# load stack pointer with stack end address
	 
    	li t0, vtable			#BASEADDR[31:2],The interrupt vector table base address,which needs to be 1KB aligned
    	ori t0, t0, 3			#BASEADDR[31:2],1: Identify by absolute address,1: Address offset based on interrupt number *4
    	#csrrw zero,t0, mtvec		# write to mtvec
	longs 0x30529073  
    
   	li t0,main
	longs 0x34129073          	#csrw	mepc,t0 :mepc updated with address of main
	longs 0x30200073		# mret ( return from interrupt)	.
  
	align 4
main:
	nop


#enable periphrel clocks
	li x10,R32_RCC_APB2PCENR	# load address of APB2PCENR register to x10 ,for enabling GPIO A,D,C peripherals
	lw x11,0(x10)			# load contents from peripheral register R32_RCC_APB2PCENR pointed by x10
	li x7,((1<<2)|(1<<4)|(1<<5)|(1<<11)|(1<<14)|(1<<0))	# 1<<IOPA_EN,1<<IOPC_EN,1<<IOPD_EN,1<<USART_EN,1<<TIM1_EN,1<<AF_EN
	or x11,x11,x7			# or values 
	sw x11,0(10)			# store modified enable values in R32_RCC_APB2PCENR
	li x10,R32_RCC_APB1PCENR
	lw x11,0(x10)
	ori,x11,x11,(1<<0)		# timer2 clock enable
	sw x11,0(x10)

#configure GPIO 
	li x10,R32_GPIOD_CFGLR		# load pointer x10 with address of R32_GPIOD_CFGLR , GPIO configuration register
	lw x11,0(x10)			# load contents from register pointed by x10
	li x7,~((0xf<<20)|(0xf<<24)|(0xf<<16))	#clear pd4,pd5,pd6. we need to setup PD5 & PD6 for usart tx and rx and pd4 for ETR
	and x11,x11,x7			# clear pd4,pd5,pd6 mode and cnf bits for selected pin D4,D5,D6
	li x7,((0x8<<24)|(0xB<<20)|(0x4<<16))	# pd6 = input with PU/PD,pd5= multiplex pushpull output 50mhz,pd4 as floating input
	or x11,x11,x7			# OR value to register
	sw x11,0(x10)			# store in R32_GPIOD_CFGLR

#enable pull up for input	
	li x10,R32_GPIOD_OUTDR		# enable pullup resistor by setting OUTDR register
	lw x11,0(x10)			# this setting of GPIO_OUTDR for pullup resistor effects if corresonding pin is selected as input
	li x7,(1<<6)			# when PD6 is input with resistor selected 1= pullup and 0 = pulldown
	or x11,x11,x7
	sw x11,0(x0)

#configure USART baud
	li x10,R32_USART_BRR		# USART BAUD setting
	lw x11,0(x10)			# copy R32_USART_BRR to x11
	li x7,((52<<4)|(1<<0))		# 52.1 in BRR =9600
	or x11,x11,x7			# or registers
	sw x11,0(x10)			# store in R32_USART_BRR

#setup UART control and enable	
	li x10,R32_USART_CTLR1		# load x10 with R32_USART_CTLR1 address
	lw x11,0(x10)			# load to x11 contents
	li x7,(1<<13)|(1<<3)|(1<<2)	# enable USART UE, TX,RX bits		# UE 
	or x11,x11,x7
	sw x11,0(x10)			# store back new values


timer2_pwmin_config:
	li x10,R16_TIM2_CHCTLR1		# x10 points timer2 compare capture register
	lw x11,0(x10)
	ori x11,x11,((1<<0)|(2<<8))	# Set TI1 (TI1FP1) to be the input of IC1 signal. Set CC1S to 01b.Set TI1 (TI1FP2) as the input of IC2 signal. Set CC2S to 10b.The channel 2 is internally connected to the channel 1, and this is to measure the duty cycle
	sw x11,0(x10)
	
	li x10,R16_TIM2_CCER		# x10 points timer2 compare capture register
	lw x11,0(x10)
	ori x11,x11,(1<<5)		# Select TI1FP2 to set to falling edge active. Set CC2P to 1 falling edge, CC1P =0 rising edge
	sw x11,0(x10)

	li x10,R16_TIM2_SMCFGR		# 
	lw x11,0(x10)
	ori x11,x11,((5<<4)|(4<<0))	# 101: Filtered timer input 1 (TI1FP1) is the clock.Set the SMS to reset mode, i.e. 100b
	sw x11,0(x10)

	li x10,R16_TIM2_INTFR		# x10 points to timer2 flag register
	sw zero,0(x10)

	li x10,R16_TIM2_PSC		# x10 points to timer2 prescaler register
	li x11,(7)			# counts at the 1mhz (timer1 update), 8000000/7+1 = 1Mhz, each count 1us
	sw x11,0(x10)

	li x10,R16_TIM2_ATRLR		# x10 points timer2 auto reload register, 
	li x11,(65535)			# max value before overflow (min frequency = 1000000/65535 = 15Hz
	sw x11,0(x10)

	li x10,R16_TIM2_CCER		# x10 points timer2 compare capture enable register
	lw x11,0(x10)
	ori x11,x11,((1<<0)|(1<<4))	# Compare capture channel1 & chanenel2  enabled
	sw x11,0(x10)

	li x10,R16_TIM2_CTLR1
	lw x11,0(x10)
	ori x11,x11,(1<<0)		# enable timer 2
	sw x11,0(x10)



#main endless loop for uart transmit
example:

	li x10,name			# load address of label "name" to x10, string to be transmitted
string_loop:
	lb x8,0(x10)			# load 1 byte from 0 offset of "name"
	beqz x8,finish			# if byte in x8 null branch to label "finish"
	call USART_TX			# call subroutine USART_TX to transmit byte
	addi x10,x10,1			# increase pointer by 1 byte
	j string_loop			# jump back to label string_loop until null is encountered
finish:
	
	li x10,R16_TIM2_CH2CVR		# comparison capture registers (CHxCVR) that support comparison with the main counter (CNT) to output pulses
	lw x11,0(x10)			# read compare capture value(pulse width)
	li x10,result1			# laod x10 with address of result1 register
	sw x11,0(x10)			# copy state register to x7

	li x10,R16_TIM2_CH1CVR		# comparison capture registers (CHxCVR) that support comparison with the main counter (CNT) to output pulses
	lw x11,0(x10)			# read compare capture value(period/cycle time)
	li x10,result2			# laod x10 with address of result2 register
	sw x11,0(x10)			# copy state register to x7

	li x10,result1			# load address of result1
	li x11,result2			# load address of result2
	lw x7,0(x10)			# copy to x7 duty cycle, on time
	lw x6,0(x11)			# copy to x6 frequency time
	sub x6,x6,x7			# subtract duty cycle from frequency
	sw x6,0(x10)			# result in result1
	li x10,dividend			# load address of divedend register
	sw x6,0(x10)			# store subtract value in dividend register
	li x10,divisor			# load address of divisor register
	li x11,100			# load x11 with divisor value 100
	sw x11,0(x10)			# store 100 in divisor register
	call division			# result in state , (wave period-pulse on time/ 100)
	li x10,result_lo		# load address of state register
	lw x7,0(x10)			# copy (wave period-pulse on time/ 100) to x7
	sub x11,x11,x7			# subtract result of division from 100, give percentage of on time
	li x10,result1			# store duty cycle percentage in result1 register
	sw x11,0(x10)			# store duty cycle percentage
	li x10,fraction			# need to copy fraction to fraction2 else overwritten when next time called
	li x11,fraction2		# x11 points to fraction2
	lw x7,0(x10)			# copy fraction
	sw x7,0(x11)			# save value in fraction to fraction2
	
	li x10,result2			# load result2 address
	lw x11,0(x10)			# copy wave period in microseconds
	li x10,divisor			# load address of divisor register
	sw x11,0(x10)			# store wave period in divisor register
	li x10,dividend			# load address of divident register
	li x11,1000000			# load x11 with 1000000, total clock count i 1 second, 8mhz prescaled to 1 mhz, each count 1us
	sw x11,0(x10)			# store in divident register
	call division			# call division subroutine ,1000000/measured wave period = frequency in Hz
	li x10,result_lo		# load address of state register, division result in state
	lw x11,0(x10)			# copy result to x11
	li x10,result2			# load address of result2
	sw x11,0(x10)			# store frequency in Hz in result2

	li x10,result1			#  point x10 to result1 for D_ASCII,result1 has whole number of duty percentage
	call D_ASCII			# subroutine to convert binary value to ASCII format decimal numbers
	call shift_print		# subroutine shifts the value to be left aligned, no leading blanks
	li x8,'.'			# load deimal point
	call USART_TX			# call uart
	li x10,fraction2		# point x10 to fraction2 ,fraction 2 holds frcion part of the duty cycle
	call D_ASCII			# subroutine to convert binary value to ASCII format decimal numbers
	call shift_print		# subroutine shifts the value to be left aligned, no leading blanks
	li x8,' '			# load space (0x20)
	call USART_TX			# call uart
	li x8,'%'			# load ascii H
	call USART_TX			# call uart
	li x8,0x0d			# line feed
	call USART_TX			# call uart
	li x8,0x0a			# carriage feed
	call USART_TX			# call uart

	li x10,result2			# point x10 to result2 for D_ASCII,frequency whole number in result2
	call D_ASCII			# subroutine to convert binary value to ASCII format decimal numbers
	call shift_print		# subroutine shifts the value to be left aligned, no leading blanks
	li x8,'.'			# load deimal point
	call USART_TX			# call uart
	li x10,fraction			# point x10 to fraction, holds fractional part of frequency reading
	call D_ASCII			# subroutine to convert binary value to ASCII format decimal numbers
	call shift_print		# subroutine shifts the value to be left aligned, no leading blanks
	li x8,' '			# load space (0x20)
	call USART_TX			# call uart
	li x8,'H'			# load ascii H
	call USART_TX			# call uart
	li x8,'z'			# load ascii Z
	call USART_TX			# call uart
	li x8,0x0d			# line feed
	call USART_TX			# call uart
	li x8,0x0a			# carriage feed
	call USART_TX			# call uart




blank_line:
	li x8,' '
	call USART_TX			# call uart
	li x8,0x0d			# line feed
	call USART_TX			# call uart
	li x8,0x0a			# carriage feed
	call USART_TX			# call uart

	call delay			# delay for man in front of terminal


	j finish
#####################################################################
#
#####################################################################	
	
USART_TX:
	addi sp,sp,-16			# add space in stack
	sw ra,0(sp)			# push ra
	sw x7,4(sp)			# push x7
	sw x10,8(sp)			# push x10
	sw x11,12(sp)			# push x11

	li x10,R32_USART_STATR		# load address of usart status register
	lw x11,0(x10)			# load contents of status register in x11
	andi x11,x11,(1<<7)		# mask out 7th bit, transmit buffer empty flag
	beqz x11,USART_TX		# if 0 transmit buffer full, wait until bit is set
	#li x8,0x30
	mv x7,x8			# move byte in x8 to x7
	li x10,R32_USART_DATAR		# x10 has the address of data register
	sb x7,0(x10)			#store byte in x7 to data register
TC_check:
	li x10,R32_USART_STATR		# get contents of status register again
	lw x11,0(x10)
	andi x11,x11,(1<<6)		# check transmit complete bit
	beqz x11,TC_check		# wait if bit is 0 , when transmit complete = 1
		
	lw x11,12(sp)			# pop x11
	lw x10,8(sp)			# pop x10
	lw x7,4(sp)			# pop x7
	lw ra,0(sp)			# pop ra
	addi sp,sp,16			# set SP back 16 bytes
	ret				# return to caller

########################################

###################################################
delay:	
	addi sp,sp,-8			# move sp 2 words
	sw ra,0(sp)			# push ra
	sw x6,4(sp)			# push x6
	li x6,2000000			# load an arbitarary value 20000000 to t1 register		
dloop:
	addi x6,x6,-1			# subtract 1 from t1
	bne x6,zero,dloop		# if t1 not equal to 0 branch to label loop
	lw x6,4(sp)			# pop x6
	lw ra,0(sp)			# pop ra
	addi sp,sp,8			# sp back 2 words
	ret				# return to caller
###########################################################
name:
string SAJEEV SANKARAN CH32V003 UART
eol:
bytes 0x0d,0x0a,0x00
##########################################################
##########################################################################################################
# converts 1 byte into ASCII represented hexadecimal value
##########################################################################################################
bin_to_ascii:
	addi sp,sp,-4
	sw ra,0(sp)
	mv a3,t2
	andi a3,a3,0xf0
	srli a3,a3,4
	slti a4,a3,10			# set a4 to 1 if a3 is less than 10 ,10and higher a4=0
	beqz a4 ,letter1
	ori a3,a3,0x30
	#mv a0,a3
	mv x8,a3
	call USART_TX
	j low_nibble
letter1:
	addi a3,a3,0x37
	#mv a0,a3
	mv x8,a3
	call USART_TX
low_nibble:
	mv a3,t2
	andi a3,a3,0x0f
	slti a4,a3,10			# set a4 to 1 if a3 is less than 10 ,10and higher a4=0
	beqz a4 ,letter2
	ori a3,a3,0x30
	#mv a0,a3
	mv x8,a3
	call USART_TX
	j exit_bin_to_ascii
letter2:
	addi a3,a3,0x37
	#mv a0,a3
	mv x8,a3
	call USART_TX
exit_bin_to_ascii:
	lw ra,0(sp)
	addi sp,sp,4
	ret
########################################################################################################################################################
# division routine with fixed point, load divisor and dividend in sram before calling, whole number in result_lo register ,fraction in fractio register
#########################################################################################################################################################

# a1 = nimber1 # low
# a2 = number2 # high
# a5 = workreg
# a3= low register = a2:a1
# a4 =hi register = a4:a3
# t1 carry

division:
	addi sp,sp,-24		# adjust stack pointer
	sw a1,20(sp)		# PUSH
	sw a2,16(sp)		# PUSH
	sw a3,12(sp)		# PUSH
	sw a4,8(sp)		# PUSH
	sw a5,4(sp)		# PUSH
	sw ra,0(sp)		# PUSH
	li t1,0			# initialize t1 to 0
	li t2,0			# initialize t2 to 0
	li a2,0			# initialize a2 to 0
	li a4,0			# initialize a4 to 0
	li a5,0			# initialize a5 to 0 

	li x10,dividend		# numerator loaded here
	lw a1,0(x10)		# divident x11
	li x10,divisor		# divisor in result1,x10 points to sram result1
	lw a2,0(x10)		# divisor
	li a3,0x01		# result & also used to count
	li a4,0		 	# result hi register
	li a5,0			# workig register/remainder

division_loop:
	li x3,0			# clear carry ,x3 is carry register
	call ROLa1		# rotate left a1 register
	call ROLa5		# rotate left a5 register
	bnez x3,div32b		# if carry is set ,branch to div32b
	mv t0,a2		# copy divisor a2 to t0
	mv t1,a5		# copy working register to t1
	#sub t2,t1,t0
	sltu x3,t1,t0		# set x3(carry) to 1 if t1 is less than t0 ,if a subtraction will result in borrow x3 is set
	bnez x3,div32c		# if carry set branch to div32c
div32b:
	mv t0,a2		# move divisor to t0
	mv t1,a5		# move working register to t1
	sub t2,t1,t0		# subtract divisor from work register
	mv a5,t2		# update the change in t2 to a5
	li x3,1			# set carry bit
	j div32d		# jump to div32d
div32c:
	li x3,0			# clear carry
div32d:
	call ROLa3		# rotate a3 left
	beqz x3,division_loop	# if carry is clear we have not reached the 1 we loaded in the a3 result register at the set up time
	li t0,result_lo		# let t0 point to result_lo
	sw a3,0(t0)		# store result of division in address  pointed by t0
	li t0,modulo		# point to address of modulo
	sw a5,0(t0)		# a5 has remainder of division 
	beqz a5,nofraction	# if no remainder after division, not fraction

fraction1:
	call soft_mul	 	# result in result_lo_m and result_hi_m, multiply modulo/remainder with 1000 to later divide again for fraction part
	li t0,result_lo_m	# point t0 to result_lo_m , 1000 x modulo	
	lw a1,0(t0)		# load 1000*modulo to a1
#	mv a1,a5		# move modulo to a1, number to be divided again
	li t0,divisor		# point t0 to divisor
	lw a2,0(t0)		# copy divisoor to a2
#	li a2,21		# divisor
	li a3,0x01		# result & also used to count
	li a4,0		 	# result hi register
	li a5,0			# working register/remainder
division_1floop:
	li x3,0			# clear carry ,x3 is carry register
	call ROLa1		# rotate left a1 register
	call ROLa5		# rotate left a5 register
	bnez x3,div32bf1	# if carry is set ,branch to div32b
	mv t0,a2		# copy divisor a2 to t0
	mv t1,a5		# copy working register to t1
	#sub t2,t1,t0
	sltu x3,t1,t0		# set x3(carry) to 1 if t1 is less than t0 ,if a subtraction will result in borrow x3 is set
	bnez x3,div32cf1	# if carry set branch to div32c
div32bf1:
	mv t0,a2		# move divisor to t0
	mv t1,a5		# move working register to t1
	sub t2,t1,t0		# subtract divisor from work register
	mv a5,t2		# update the change in t2 to a5
	li x3,1			# set carry bit
	j div32df1		# jump to div32d
div32cf1:
	li x3,0			# clear carry
div32df1:
	call ROLa3		# rotate a3 left
	beqz x3,division_1floop	# if carry is clear we have not reached the 1 we loaded in the a3 result register at the set up time
	li t0,fraction		# let t0 point to result_lo
	sw a3,0(t0)		# store result of division in address  pointed by t0
	li t0,modulo		# point to address of modulo
	sw a5,0(t0)		# a5 has remainder of division 
	j here
nofraction:			# reach here when first division had no remainder
	li t0,fraction		# point t0 to fraction register
	sw zero,0(t0)		# store 0 in fraction register
here:
	lw ra,0(sp)		# POP
	lw a5,4(sp)		# POP
	lw a4,8(sp)		# POP
	lw a3,12(sp)		# POP
	lw a2,16(sp)		# POP
	lw a1,20(sp)		# POP
	addi sp,sp,24		# adjust stack pointer
	ret			# return to caller who called division

#SUBROOUTINES

ROLa1:
	li x3,0			# clear carry
	mv t1,a1		# we need to rotate a1 through carry. copy a1 to t1
	li t0,0x80000000	# extract the MSB with bitmask 0x80000000
	and t1,t1,t0		# ANDing t1 with t0 extracts the MSB
	bnez t1,setcarry1	# if t1 is not 0, MSB is 1 else MSB is 0. If 1 branch to setcarry1
	slli a1,a1,1		# a 0 is shifted to lsb and x3 (carry) stays 0 if previous operation resulted 0 in t1
	ret			# return to caller
setcarry1:
	li x3,1			# set carry , load 1 in x3 as above left shift resulted in 1 
	slli a1,a1,1		# shift a1 1 step left, 0 is shifted into lsb and carry has the 1 shifted out from msb
	ret			# return to caller

ROLa5:
	mv t1,a5		# we need to rotate a5 through carry. copy a5 to t1
	li t0,0x80000000	# extract the MSB with bitmask 0x80000000
	and t1,t1,t0		# ANDing t1 with t0 extracts the MSB
	bnez t1,setcarry2	# if t1 is not 0, MSB is 1 else MSB is 0. If 1 branch to setcarry2
	slli a5,a5,1		# a 0 is shifted to lsb and x3 (carry) stays 0 if previous operation resulted 0 in t1
	or a5,a5,x3		# shift in bit in carry by ORing x3. this bit in carry is from previous subroutine call
	li x3,0			# clear carry as bit in carry now has been shifted to a5 lsb
	ret			# return to caller
setcarry2:
	slli a5,a5,1		# we reach here if msb of a5 is 1, shift a5 1 step left
	or a5,a5,x3		# shift lhs a5 1 step
	li x3,1			# set carry to 1
	ret			# return to caller

ROLa3:
	mv t1,a3		# move result register to t1
	li t0,0x80000000	# load bit mask for top bit
	and t1,t1,t0		# and t1 and t0 to extract msb
	bnez t1,setcarry3	# if t1 is not 0, branch to setcarry2
	slli a3,a3,1		# left shift a3 1 step
	or a3,a3,x3		# OR in bit stored in x3(carry) from previous subroutine
	li x3,0			# clear carry, 0 shifted out from from msb is stored 
	ret			# return to caller
setcarry3:
	slli a3,a3,1		# shift a3 to left to OR in the bit in carry
	or a3,a3,x3		# shift in the bit in carry to lsb
	li x3,1			# set carry to 1 as msb of a3 waas 1
	ret			# return to caller

####################################
# multiplication routine , multiplies a1 and a2
####################################
soft_mul:			# 32 bit multiplication subroutine
	addi sp,sp,-24		# adjust stack pointer
	sw ra,0(sp)		# PUSH
	sw a1,4(sp)		# PUSH
	sw a2,8(sp)		# PUSH
	sw a3,12(sp)		# PUSH
	sw a4,16(sp)		# PUSH
	sw a5,20(sp)		# PUSH

	#li a1,0xffffffff 	# multiplicand
	li t0,modulo		# point t0 to modulo register
	lw a1,0(t0)		# copy remainder/modulo to a1 , multiplicand
 	li a2,1000		# multiplier
 	li a3,0x00000000 	# result_lo
 	li a4,0x00000000 	# result_hi
	li a5,0			# working register 
 
 
 start:
 	call ROR		# rotate right multiplier to test lsb is 0 or 1
 	bnez x3,multiply	# if lsb =1 branch to repeated adding of multiplicant to result register
 finishmul:
 	call RLL2		# shift multiplicand left or multiply by 2
 	beqz a2,exit_proc	# if a2 is 0 branch to exit_proc
 	J start			# repeat loop
 exit_proc:
 	#j exit_proc
	li t0,result_lo_m	# point t0 to result_lo_m register
	sw a3,0(t0)		# store result of multiplication( modulo * 1000)
	li t0,result_hi_m	# point t0 to result_hi_m register to store carry
	sw a4,0(t0)		# store high byte result of multiplication( modulo * 1000)
	lw a5,20(sp)		# POP
	lw a4,16(sp)		# POP
	lw a3,12(sp)		# POP
	lw a2,8(sp)		# POP
	lw a1,4(sp)		# POP
	lw ra,0(sp)		# POP
 	ret			# return to division routine
 multiply:
 	add a5,a3,a1		# add multiplicant to low result register and store final result in a5 for processing
 	sltu a0 a5,a3		# set a0 to 1 if result of addition a3:a1 i a5 is greater than a3
 	sltu x3,a5,a1		# set x3 to 1 if result of addition a3:a1 in a5 is greater than a1
 	or a0,a0,x3		# or a0 and x3 , if 1 carry if a0 = 0 no carry
 	bnez a0,carryset	# if a0 = 1 carry set, branch to label carry set
 	mv a3,a5		# result in working register copied to a3 low result register
 	J finishmul		# jump to label finishmul
 carryset:			# reach here only if carryset
 	mv a3,a5		# copy a5 to low result a3
 	addi a4,a4,1		# add carry to a4 high register result
 	J finishmul		# jump to label finishmul
 
 ROR:
 	li x3,0			# clear carry
 	mv t0,a2		# copy number in a2 to t0
 	andi t0,t0,1		# extract lsb is 0 or 1
 	beqz t0,zzz		# if lab is 0 branch to zzz
 		li x3,1		# if lsb is 1 carry occured , load 1 in carry register x3
 	srli a2,a2,1		# shift right a2 by 1 postion 
 	ret			# return to caller
 zzz:				# reach here if lsb =0
 	li x3,0			# load x3 0 indicating carry bit is 0
 	srli a2,a2,1		# right shift multiplier once. divide multiplier by 2
 	ret			# return to caller
 
 ROL:
 	li x3,0			# 
 	mv t0,a2
 	li x3,0x80000000
 	and t0,t0,x3
 	beqz t0,zzz1
 	li x3,1			# carry
 	slli a2,a2,1
 	ret
 zzz1:
 	li x3,0
 	slli a2,a2,1
 	ret
 
 RLL2:				# rotate left 2 registers a3:a5
 	mv a5,a4		# copy contents of a4 to a5
 	li x3,0			# clear x3
 	mv t0,a1		# copy multiplicant to t0
 	li x3 ,0x80000000	# load x3 MSB bitmask
 	and t0,t0,x3		# and with 0x800000000 to extract the MSB
 	bnez t0,OR1		# if MSB = 1 branch to OR1 label
 	slli a1,a1,1		# shift left 1 position a1 register ( multiplicant)
 	slli a5,a5,1		# shift left 1 position working register with value of a4 register ( multiplicant)
 	beqz a2,exit		# if multiplier register is 0 exit
 	mv a4,a5		# copy back the shifter multiplicant to a4
 	ret
 OR1:
 	mv a5,a4
 	slli a1,a1,1
 	slli a5,a5,1
 	li x3,1
 	or a5,a5,x3
 	beqz a2,exit
 	mv a4,a5
 	ret
exit:
	ret

	

#########################################################################################################################################
# D_ASCII subroutine for converting binary in result1 to DECIMAL (ASCII), point with x10 to register with value to be converted
#########################################################################################################################################

D_ASCII:			
	addi sp,sp,-32		# adjust stack pointer
	sw ra,28(sp)		# PUSH
	sw x15,24(sp)		# PUSH
	sw x11,20(sp)		# PUSH
	sw x8,16(sp)		# PUSH
	sw x7,12(sp)		# PUSH
	sw x5,8(sp)		# PUSH
	sw x4,4(sp)		# PUSH
	sw t1,0(sp)		# PUSH
	li x4,0			# clear register
	li x5,0			# clear register
	li x7,0			# clear register
	li x8,0			# clear register
	li x15,0		# clear register
	
#	li x10,0x20000010	# result1
	lw x4,0(x10)		# copy value from memory pointed by x10 to x4,this routine to be called after pointing to register with required value
#	li x4,0xffffffff	# 32bit word to be converted into ascii chars
	li x7,1000000000	# divisor
Y1:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X1		# if result negative(not divisible) branch to X1
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y1			# jump to label Y1 till not divisible
X1:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	li x10,mem		# set pointer x10 to SRAM register mem to store the byte
	sb x15,0(x10)		# store byte in mem+0
	li x15,0		# clear result
	li x7,100000000		# load x7 with new divisor
Y2:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X2		# if result negative(not divisible) branch to X2
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y2			# jump to label Y2 till not divisible
X2:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+1 to store the byte
	sb x15,0(x10)		# store byte in mem+1
	li x15,0		# clear result
	li x7,10000000		# load x7 with new divisor
Y3:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X3		# if result negative(not divisible) branch to X3
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y3			# jump to label Y3 till not divisible
X3:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+2 to store the byte
	sb x15,0(x10)		# store byte in mem+2
	li x15,0		# clear result
	li x7,1000000		# load x7 with new divisor
Y4:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X4		# if result negative(not divisible) branch to X4
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y4			# jump to label Y4 till not divisible
X4:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+3		
	sb x15,0(x10)		# store byte in mem+3
	li x15,0		# clear result
	li x7,100000		# load x7 with new divisor
Y5:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X5		# if result negative(not divisible) branch to X5
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y5			# jump to label Y5 till not divisible
X5:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+4 to store the byte
	sb x15,0(x10)		# store byte in mem+4
	li x15,0		# clear result
	li x7,10000		# load x7 with new divisor
Y6:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X6		# if result negative(not divisible) branch to X6
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y6			# jump to label Y6 till not divisible
X6:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+5 to store the byte
	sb x15,0(x10)		# store byte in mem+5
	li x15,0		# clear result
	li x7,1000		# load x7 with new divisor
Y7:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X7		# if result negative(not divisible) branch to X7
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y7			# jump to label Y7 till not divisible
X7:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+6 to store the byte
	sb x15,0(x10)		# store byte in mem+6
	li x15,0		# clear result
	li x7,100		# load x7 with new divisor
Y8:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X8		# if result negative(not divisible) branch to X8
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y8			# jump to label Y8 till not divisible
X8:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+7 to store the byte
	sb x15,0(x10)		# store byte in mem+7
	li x15,0		# clear result
	li x7,10		# load x7 with new divisor
Y9:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X9		# if result negative(not divisible) branch to X9
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y9			# jump to label Y9 till not divisible
X9:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+8 to store the byte
	sb x15,0(x10)		# store byte in mem+8
	li x15,0		# clear result
	mv x15,x4
X10:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+9 to store the byte	
	sb x15,0(x10)		# store byte in mem+9

	lw t1,0(sp)		# POP
	lw x4,4(sp)		# POP
	lw x5,8(sp)		# POP
	lw x7,12(sp)		# POP
	lw x8,16(sp)		# POP
	lw x11,20(sp)		# POP
	lw x15,24(sp)		# POP
	lw ra,28(sp)		# POP
	addi sp,sp,32		# adjust stack pointer
	ret			# return to caller
######################################################

################################################################################
# Prints 10 bytes from result1 to USART with lead 0 suppress, values to be stored in mem0-mem9
################################################################################
print:
	addi sp,sp,-20		# adjust stack pointer
	sw ra,16(sp)		# PUSH
	sw x11,12(sp)		# PUSH
	sw x10,8(sp)		# PUSH
	sw x8,4(sp)		# PUSH
	sw t1,0(sp)		# PUSH

	li x11,scratch		# point x11 to scratch register in SRAM
	sw zero,0(x11)		# clear scratch register

	li x10,mem		# point to address mem ,top byte stored in mem0 lowest byte in mem+9, need to print top byte 1st
	li t1,0			# byte counter loaded with 10 , total 10 bytes to be printed
	li x15,10		# max count of 10bytes in x15

Z:
	lb x8,0(x10)		# load byte from result1, msb to lsb
	
	li x4,0x30		# load ascii 0 in x4
	beq x8,x4,supress0	# if result1 byte in x8 is equal to ascii 0 in x4 branch to suppress0 label
	li x11,scratch		# point x11 to scratch if x8 is not 0,that means we have found the 1st byte that is not 0, all leading 0s finished
	li x5,1			# load x5 with 1
	sw x5,0(x11)		# store in scratch register in sram , used as a flag to indicate all leading 0s are finished

print1:
	addi x10,x10,1		# increase the address by 1 byte
	addi t1,t1,1		# increase the byte counter once
	call USART_TX		# call uart
	bne t1,x15,Z		# if t1 not equal to 10 as loaded in x15 loop back to print1 till al 10 bytes are transmitted via usart
	lw t1,0(sp)		# POP
	lw x8,4(sp)		# POP
	lw x10,8(sp)		# POP
	lw x11,12(sp)		# POP
	lw ra,16(sp)		# POP
	addi sp,sp,20		# adjust stack pointer
	ret			# return to caller
supress0:
	li x11,scratch		# point x11 to scratch	
	lw x5,0(x11)		# copy value of scratch to x5
	bnez x5,no_more_supress	# if x5 is not 0 branch to label "no_more_supress"
	li x8,0x20		# if x5 is 0 load x8 with space/blank	
	li x4,9			# load x4 with value 9, suppose the whole value is 0, we dont want to display blank space, test this is 9th byte 2nd last byte
	beq t1,x4,last0is0	# if t1 equals 9 in x4 branch to label "last0is0" which will keep last 0 as 0 on screen
no_more_supress:
	J print1		# no 0 suppression jump to print1	
last0is0:
	li x8,0x30		# load ascii 0 for last 0
	J print1		# jump to print1


###############################################################################################
# prints 10 bytes from mem register, all leading zeros are not printed, aligns numbers to left
###############################################################################################

shift_print:
	addi sp,sp,-20		# adjust stack pointer
	sw ra,16(sp)		# PUSH
	sw x11,12(sp)		# PUSH
	sw x10,8(sp)		# PUSH
	sw x8,4(sp)		# PUSH
	sw t1,0(sp)		# PUSH

	li x11,scratch		# point x11 to scratch register in SRAM
	sw zero,0(x11)		# clear scratch register

	li x10,mem		# point to address mem ,top byte stored in mem0 lowest byte in mem+9, need to print top byte 1st
	li t1,0			# byte counter loaded with 10 , total 10 bytes to be printed
	li x15,10		# max count of 10bytes in x15

Z:
	lb x8,0(x10)		# load byte from result1, msb to lsb
	
	li x4,0x30		# load ascii 0 in x4
	beq x8,x4,shift_supress0	# if result1 byte in x8 is equal to ascii 0 in x4 branch to suppress0 label
	li x11,scratch		# point x11 to scratch if x8 is not 0,that means we have found the 1st byte that is not 0, all leading 0s finished
	li x5,1			# load x5 with 1
	sw x5,0(x11)		# store in scratch register in sram , used as a flag to indicate all leading 0s are finished

shift_print1:
	addi x10,x10,1		# increase the address by 1 byte
	addi t1,t1,1		# increase the byte counter once
	call USART_TX		# call uart(print)
shift:
	bne t1,x15,Z		# if t1 not equal to 10 as loaded in x15 loop back to print1 till al 10 bytes are transmitted via usart
	lw t1,0(sp)		# POP
	lw x8,4(sp)		# POP
	lw x10,8(sp)		# POP
	lw x11,12(sp)		# POP
	lw ra,16(sp)		# POP
	addi sp,sp,20		# adjust stack pointer
	ret			# return to caller
shift_supress0:
	li x11,scratch		# point x11 to scratch	
	lw x5,0(x11)		# copy value of scratch to x5
	bnez x5,shift_no_more_supress	# if x5 is not 0 branch to label "no_more_supress"
	li x8,0x20		# if x5 is 0 load x8 with space/blank	
	li x4,9			# load x4 with value 9, suppose the whole value is 0, we dont want to display blank space, test this is 9th byte 2nd last byte
	beq t1,x4,last0is0	# if t1 equals 9 in x4 branch to label "shift_last0is0" which will keep last 0 as 0 on screen
	addi x10,x10,1		# if a leading 0,increase the address by 1 byte
	addi t1,t1,1		# if a leading 0,increase the byte counter once
	J shift			# jump to label "shift" so that print to uart is avoided and next byte is loaded from mem, this aligns the print to LHS of screen
	
shift_no_more_supress:
	J shift_print1		# no 0 suppression jump to print1	
shift_last0is0:
	li x8,0x30		# load ascii 0 for last 0,0 in unit position is printed on screen
	J shift_print1		# jump to shift_print1