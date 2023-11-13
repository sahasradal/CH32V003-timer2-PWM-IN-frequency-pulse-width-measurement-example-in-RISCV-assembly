.data
result_lo: .word 0
result_hi: .word 0
modulo: .word 0
fraction: .word 0
result_lo_m: .word 0
result_hi_m: .word 0


.text
	li a1,100     	 	# number to be dividend
	li a2,21		# divisor
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
	la t0,result_lo		# let t0 point to result_lo
	sw a3,0(t0)		# store result of division in address  pointed by t0
	la t0,modulo		# point to address of modulo
	sw a5,0(t0)		# a5 has remainder of division 
	beqz a5,nofraction

fraction1:
	call soft_mul	 # result in result_lo_m and result_hi_m
	la t0,result_lo_m
	lw a1,0(t0)
#	mv a1,a5		# move modulo to a1, number to be divided again
	li a2,21		# divisor
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
	la t0,fraction		# let t0 point to result_lo
	sw a3,0(t0)		# store result of division in address  pointed by t0
	la t0,modulo		# point to address of modulo
	sw a5,0(t0)		# a5 has remainder of division 
	j here
nofraction:
	la t0,fraction
	sw zero,0(t0)	
here:
	j here

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


soft_mul:
	addi sp,sp,-24
	sw ra,0(sp)
	sw a1,4(sp)
	sw a2,8(sp)
	sw a3,12(sp)
	sw a4,16(sp)
	sw a5,20(sp)

	#li a1,0xffffffff 	# multiplicand
	la t0,modulo
	lw a1,0(t0)
 	li a2,1000		# multiplier
 	li a3,0x00000000 	# result_lo
 	li a4,0x00000000 	# result_hi
	li a5,0			# working register 
 
 
 start:
 	call ROR		# rotate right multiplier to test lsb is 0 or 1
 	bnez x3,multiply	# if lsb =1 branch to repeated adding of multiplicant to result register
 finishmul:
 	call RLL2		# shift multiplicand left or multiply by 2
 	beqz a2,exit_proc
 	J start			# repeat loop
 exit_proc:
 	#j exit_proc
	la t0,result_lo_m
	sw a3,0(t0)
	la t0,result_hi_m
	sw a4,0(t0)
	lw a5,20(sp)
	lw a4,16(sp)
	lw a3,12(sp)
	lw a2,8(sp)
	lw a1,4(sp)
	lw ra,0(sp)
 	ret
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



