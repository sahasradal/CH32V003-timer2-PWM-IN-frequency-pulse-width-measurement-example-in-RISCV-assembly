# 00: Default mapping (CH1/ETR/PD4, CH2/PD3,CH3/PC0, CH4/PD7).PWM-IN mode, input on pd4 ch1, works fine,display hexadecimal
# 1HZ granularity from 15Hz to 1000Hz (1kHZ) signal, above 1khz 2hz resolution ,above 2kz 4 hz resolution
#BRR = BAUD  
#13.6=38400
#4.4 = 115200
#52.1 = 9600
#8.7 =  57600
fclk = 24000000   # 24Mhz RCO internal , AHB =8Mhz by default
state = 0x2000000C
#USARTx_TX Full-duplex mode Push-pull multiplexed outputs
#USARTx_RX Full-duplex mode Floating input or pull-up input
#PD5 -tx
#PD6 -rx
#PD4 -input
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
	li x10,state			# load address of state register
	lw x7,0(x10)			# copy (wave period-pulse on time/ 100) to x7
	sub x11,x11,x7			# subtract result of division from 100, give percentage of on time
	li x10,result1			# store duty cycle percentage in result1 register
	sw x11,0(x10)			# store duty cycle percentage
	
	li x10,result2			# load result2 address
	lw x11,0(x10)			# copy wave period in microseconds
	li x10,divisor			# load address of divisor register
	sw x11,0(x10)			# store wave period in divisor register
	li x10,dividend			# load address of divident register
	li x11,1000000			# load x11 with 1000000, total clock count i 1 second, 8mhz prescaled to 1 mhz, each count 1us
	sw x11,0(x10)			# store in divident register
	call division			# call division subroutine ,1000000/measured wave period = frequency in Hz
	li x10,state			# load address of state register, division result in state
	lw x11,0(x10)			# copy result to x11
	li x10,result2			# load address of result2
	sw x11,0(x10)			# store frequency in Hz in result2

read_duty:
	li x10,result1			# load address of result1 which holds the duty cycle in percentage
	li t1,4				# t1 counter = 4
	
readloop3:
	lb t2,3(x10)			# loads 1 byte to t2 from result
	call bin_to_ascii		# convert bin to ascii
	addi x10,x10,-1			# decrease pointer
	addi t1,t1,-1			# decrease counter
	bnez t1,readloop3		# loop till counter is 0
	li x8,' '
	call USART_TX			# call uart
	li x8,'%'
	call USART_TX			# call uart
	li x8,0x0d			# line feed
	call USART_TX			# call uart
	li x8,0x0a			# carriage feed
	call USART_TX			# call uart


read_Hz:
	li x10,result2			# load address of result1 which holds the frequency in Hz
	li t1,4				# t1 counter = 4
	
readloop4:
	lb t2,3(x10)			# loads 1 byte to t2 from result
	call bin_to_ascii		# convert bin to ascii
	addi x10,x10,-1			# decrease pointer
	addi t1,t1,-1			# decrease counter
	bnez t1,readloop4		# loop till counter is 0
	li x8,' '
	call USART_TX			# call uart
	li x8,'H'
	call USART_TX			# call uart
	li x8,'z'			# line feed
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
#################################################################

#################################################################

# a1 = nimber1 # low
# a2 = number2 # high
# a5 = workreg
# a3= low register = a2:a1
# a4 =hi register = a4:a3
# t1 carry

division:
	addi sp,sp,-24
	sw a1,20(sp)
	sw a2,16(sp)
	sw a3,12(sp)
	sw a4,8(sp)
	sw a5,4(sp)
	sw t1,0(sp)
	li t1,0				# initialize t1 to 0
	li t2,0				# initialize t2 to 0
	li a2,0				# initialize a2 to 0
	li a4,0				# initialize a4 to 0
	li a5,0				# initialize a5 to 0 
	li x10,dividend			# numerator loaded here
	lw a1,0(x10)			# divident x11
	li x10,divisor			# divisor in result1,x10 points to sram result1
	lw a3,0(x10)			# load word in state to x13(divisor)
X:
	sub a5,a1,a3			# subtract divisor from dividend and store remainder in a5
	sltu t1,a1,a3			# set t1 if a1 is less than a3
	bnez t1,carry			# if t1 not equal 0 branch to carry
	mv a1,a5			# move remainder to a1 from a5
	sub a5,a2,a4			# subtract the high registers a4 from a2
	addi t2,t2,1			# increase t2 by 1 for each successful subtraction (result)
	J X				# loop to X till t1 is set
carry:
	li t0, state			# load address of state in t0
	sw t2,0(t0)			# store result in t2 in state
#	li t0, result2			# load address of result2 in t0
#	sw a1,0(t0)			# store a1 high byte of result in result2 = 0
	lw t1,0(sp)
	lw a5,4(sp)
	lw a4,8(sp)
	lw a3,12(sp)
	lw a2,16(sp)
	lw a1,20(sp)
	addi sp,sp,24
	ret