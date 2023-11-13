# CH32V003-timer2-PWM-IN-frequency-pulse-width-measurement-example-in-RISCV-assembly
CH32V00_PWMinput_ver0 displays in hexadecimal values, PD4 input ,#PD5 -tx, #PD6 -rx , 9600 baud
CH32V00_PWMinput_ver1 displays in decimal values, PD4 input ,#PD5 -tx, #PD6 -rx , 9600 baud
Assembled with bronzebeard assembler.
burned with WCH utility and elink

PWM_IN_ver2 added which can measure fractional part. Displays float values of frequency and duty cycle
modified code to suppress leading zeros and left align the results to the terminal screen. removes leading space in results.

