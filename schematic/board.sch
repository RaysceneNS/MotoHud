EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:adc-dac
LIBS:microcontrollers
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:interface
LIBS:philips
LIBS:display
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:bt-hc-05
LIBS:sensors
LIBS:board-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Moto HUD"
Date ""
Rev "A"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ATTINY1634-SU U4
U 1 1 5A47FC90
P 3400 3250
F 0 "U4" H 2650 4450 50  0000 C CNN
F 1 "ATTINY1634-SU" H 3950 2250 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-20W_7.5x12.8mm_Pitch1.27mm" H 3400 3500 50  0001 C CIN
F 3 "" H 3400 3700 50  0001 C CNN
	1    3400 3250
	1    0    0    -1  
$EndComp
$Comp
L LM2936-3.3 U3
U 1 1 5A47FD22
P 9600 1450
F 0 "U3" H 9450 1575 50  0000 C CNN
F 1 "LM2937-3.3" H 9600 1575 50  0000 L CNN
F 2 "" H 9600 1675 50  0001 C CIN
F 3 "" H 9600 1400 50  0001 C CNN
	1    9600 1450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR19
U 1 1 5A47FE00
P 9600 2000
F 0 "#PWR19" H 9600 1750 50  0001 C CNN
F 1 "GND" H 9600 1850 50  0000 C CNN
F 2 "" H 9600 2000 50  0001 C CNN
F 3 "" H 9600 2000 50  0001 C CNN
	1    9600 2000
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR18
U 1 1 5A47FE18
P 9100 1300
F 0 "#PWR18" H 9100 1150 50  0001 C CNN
F 1 "+12V" H 9100 1440 50  0000 C CNN
F 2 "" H 9100 1300 50  0001 C CNN
F 3 "" H 9100 1300 50  0001 C CNN
	1    9100 1300
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5A480276
P 9100 1700
F 0 "C1" H 9125 1800 50  0000 L CNN
F 1 "0.1uF" H 9125 1600 50  0000 L CNN
F 2 "" H 9138 1550 50  0001 C CNN
F 3 "" H 9100 1700 50  0001 C CNN
	1    9100 1700
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5A4802CB
P 10100 1700
F 0 "C2" H 10125 1800 50  0000 L CNN
F 1 "10uF" H 10125 1600 50  0000 L CNN
F 2 "" H 10138 1550 50  0001 C CNN
F 3 "" H 10100 1700 50  0001 C CNN
	1    10100 1700
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR20
U 1 1 5A48057C
P 10250 1300
F 0 "#PWR20" H 10250 1150 50  0001 C CNN
F 1 "+3V3" H 10250 1440 50  0000 C CNN
F 2 "" H 10250 1300 50  0001 C CNN
F 3 "" H 10250 1300 50  0001 C CNN
	1    10250 1300
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR3
U 1 1 5A480844
P 2400 2150
F 0 "#PWR3" H 2400 2000 50  0001 C CNN
F 1 "+3V3" H 2400 2290 50  0000 C CNN
F 2 "" H 2400 2150 50  0001 C CNN
F 3 "" H 2400 2150 50  0001 C CNN
	1    2400 2150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR4
U 1 1 5A48087A
P 2400 4150
F 0 "#PWR4" H 2400 3900 50  0001 C CNN
F 1 "GND" H 2400 4000 50  0000 C CNN
F 2 "" H 2400 4150 50  0001 C CNN
F 3 "" H 2400 4150 50  0001 C CNN
	1    2400 4150
	1    0    0    -1  
$EndComp
$Comp
L HC-05 U2
U 1 1 5A48141C
P 6300 3350
F 0 "U2" H 5750 4200 60  0000 C CNN
F 1 "HC-05" H 5850 4100 60  0000 C CNN
F 2 "" H 5400 4200 60  0000 C CNN
F 3 "" H 5400 4200 60  0000 C CNN
	1    6300 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR10
U 1 1 5A48215D
P 5300 4100
F 0 "#PWR10" H 5300 3850 50  0001 C CNN
F 1 "GND" H 5300 3950 50  0000 C CNN
F 2 "" H 5300 4100 50  0001 C CNN
F 3 "" H 5300 4100 50  0001 C CNN
	1    5300 4100
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR9
U 1 1 5A48217B
P 5000 3850
F 0 "#PWR9" H 5000 3700 50  0001 C CNN
F 1 "+3V3" H 5000 3990 50  0000 C CNN
F 2 "" H 5000 3850 50  0001 C CNN
F 3 "" H 5000 3850 50  0001 C CNN
	1    5000 3850
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR13
U 1 1 5A4825C3
P 6650 4550
F 0 "#PWR13" H 6650 4300 50  0001 C CNN
F 1 "GND" H 6650 4400 50  0000 C CNN
F 2 "" H 6650 4550 50  0001 C CNN
F 3 "" H 6650 4550 50  0001 C CNN
	1    6650 4550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR16
U 1 1 5A4825E1
P 7300 3950
F 0 "#PWR16" H 7300 3700 50  0001 C CNN
F 1 "GND" H 7300 3800 50  0000 C CNN
F 2 "" H 7300 3950 50  0001 C CNN
F 3 "" H 7300 3950 50  0001 C CNN
	1    7300 3950
	1    0    0    -1  
$EndComp
$Comp
L MCP9700AT-E/TT U1
U 1 1 5A482CB2
P 1650 3000
F 0 "U1" H 1400 3250 50  0000 C CNN
F 1 "MCP9700AT-E/TT" H 1700 3250 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 1650 2600 50  0001 C CNN
F 3 "" H 1500 3250 50  0001 C CNN
	1    1650 3000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR2
U 1 1 5A482DC0
P 1650 3400
F 0 "#PWR2" H 1650 3150 50  0001 C CNN
F 1 "GND" H 1650 3250 50  0000 C CNN
F 2 "" H 1650 3400 50  0001 C CNN
F 3 "" H 1650 3400 50  0001 C CNN
	1    1650 3400
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR1
U 1 1 5A482E04
P 1650 2600
F 0 "#PWR1" H 1650 2450 50  0001 C CNN
F 1 "+3V3" H 1650 2740 50  0000 C CNN
F 2 "" H 1650 2600 50  0001 C CNN
F 3 "" H 1650 2600 50  0001 C CNN
	1    1650 2600
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5A482E81
P 4800 1850
F 0 "R1" V 4880 1850 50  0000 C CNN
F 1 "47K" V 4800 1850 50  0000 C CNN
F 2 "" V 4730 1850 50  0001 C CNN
F 3 "" H 4800 1850 50  0001 C CNN
	1    4800 1850
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5A482EB6
P 5000 1850
F 0 "R2" V 5080 1850 50  0000 C CNN
F 1 "10K" V 5000 1850 50  0000 C CNN
F 2 "" V 4930 1850 50  0001 C CNN
F 3 "" H 5000 1850 50  0001 C CNN
	1    5000 1850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR8
U 1 1 5A48301E
P 5000 1650
F 0 "#PWR8" H 5000 1400 50  0001 C CNN
F 1 "GND" H 5000 1500 50  0000 C CNN
F 2 "" H 5000 1650 50  0001 C CNN
F 3 "" H 5000 1650 50  0001 C CNN
	1    5000 1650
	-1   0    0    1   
$EndComp
$Comp
L +12V #PWR7
U 1 1 5A483042
P 4800 1650
F 0 "#PWR7" H 4800 1500 50  0001 C CNN
F 1 "+12V" H 4800 1790 50  0000 C CNN
F 2 "" H 4800 1650 50  0001 C CNN
F 3 "" H 4800 1650 50  0001 C CNN
	1    4800 1650
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5A483188
P 5850 1850
F 0 "R3" V 5930 1850 50  0000 C CNN
F 1 "47K" V 5850 1850 50  0000 C CNN
F 2 "" V 5780 1850 50  0001 C CNN
F 3 "" H 5850 1850 50  0001 C CNN
	1    5850 1850
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 5A4831EB
P 6050 1850
F 0 "R4" V 6130 1850 50  0000 C CNN
F 1 "47K" V 6050 1850 50  0000 C CNN
F 2 "" V 5980 1850 50  0001 C CNN
F 3 "" H 6050 1850 50  0001 C CNN
	1    6050 1850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR12
U 1 1 5A4832A0
P 6050 1650
F 0 "#PWR12" H 6050 1400 50  0001 C CNN
F 1 "GND" H 6050 1500 50  0000 C CNN
F 2 "" H 6050 1650 50  0001 C CNN
F 3 "" H 6050 1650 50  0001 C CNN
	1    6050 1650
	-1   0    0    1   
$EndComp
$Comp
L +5V #PWR11
U 1 1 5A4832FB
P 5850 1650
F 0 "#PWR11" H 5850 1500 50  0001 C CNN
F 1 "+5V" H 5850 1790 50  0000 C CNN
F 2 "" H 5850 1650 50  0001 C CNN
F 3 "" H 5850 1650 50  0001 C CNN
	1    5850 1650
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 5A4833D8
P 5250 1850
F 0 "C3" H 5275 1950 50  0000 L CNN
F 1 "0.1uF" H 5275 1750 50  0000 L CNN
F 2 "" H 5288 1700 50  0001 C CNN
F 3 "" H 5250 1850 50  0001 C CNN
	1    5250 1850
	1    0    0    -1  
$EndComp
NoConn ~ 5400 2950
NoConn ~ 5400 3050
NoConn ~ 5400 3150
NoConn ~ 5400 3250
NoConn ~ 5400 3350
NoConn ~ 5400 3450
NoConn ~ 5400 3550
NoConn ~ 5400 3650
NoConn ~ 5400 3750
NoConn ~ 7200 2750
NoConn ~ 7200 2850
NoConn ~ 7200 2950
NoConn ~ 7200 3050
NoConn ~ 7200 3150
NoConn ~ 7200 3250
NoConn ~ 7200 3350
NoConn ~ 7200 3450
NoConn ~ 7200 3550
NoConn ~ 7200 3650
NoConn ~ 7200 3850
NoConn ~ 4350 2300
NoConn ~ 4350 2400
NoConn ~ 4350 2800
NoConn ~ 4350 3200
NoConn ~ 4350 3300
NoConn ~ 4350 3400
NoConn ~ 4350 3900
NoConn ~ 5950 4450
NoConn ~ 6050 4450
NoConn ~ 6150 4450
NoConn ~ 6250 4450
NoConn ~ 6350 4450
NoConn ~ 6450 4450
NoConn ~ 6550 4450
$Comp
L PWR_FLAG #FLG1
U 1 1 5A4A8C3E
P 6750 1450
F 0 "#FLG1" H 6750 1525 50  0001 C CNN
F 1 "PWR_FLAG" H 6750 1600 50  0000 C CNN
F 2 "" H 6750 1450 50  0001 C CNN
F 3 "" H 6750 1450 50  0001 C CNN
	1    6750 1450
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG2
U 1 1 5A4A8C68
P 7150 1450
F 0 "#FLG2" H 7150 1525 50  0001 C CNN
F 1 "PWR_FLAG" H 7150 1600 50  0000 C CNN
F 2 "" H 7150 1450 50  0001 C CNN
F 3 "" H 7150 1450 50  0001 C CNN
	1    7150 1450
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR15
U 1 1 5A4A8C9D
P 7150 1550
F 0 "#PWR15" H 7150 1400 50  0001 C CNN
F 1 "+5V" H 7150 1690 50  0000 C CNN
F 2 "" H 7150 1550 50  0001 C CNN
F 3 "" H 7150 1550 50  0001 C CNN
	1    7150 1550
	-1   0    0    1   
$EndComp
$Comp
L +12V #PWR14
U 1 1 5A4A8CC7
P 6750 1550
F 0 "#PWR14" H 6750 1400 50  0001 C CNN
F 1 "+12V" H 6750 1690 50  0000 C CNN
F 2 "" H 6750 1550 50  0001 C CNN
F 3 "" H 6750 1550 50  0001 C CNN
	1    6750 1550
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR17
U 1 1 5A4A8DA2
P 7550 1550
F 0 "#PWR17" H 7550 1300 50  0001 C CNN
F 1 "GND" H 7550 1400 50  0000 C CNN
F 2 "" H 7550 1550 50  0001 C CNN
F 3 "" H 7550 1550 50  0001 C CNN
	1    7550 1550
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG3
U 1 1 5A4A9336
P 7550 1450
F 0 "#FLG3" H 7550 1525 50  0001 C CNN
F 1 "PWR_FLAG" H 7550 1600 50  0000 C CNN
F 2 "" H 7550 1450 50  0001 C CNN
F 3 "" H 7550 1450 50  0001 C CNN
	1    7550 1450
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x08 J1
U 1 1 5A4AA013
P 3650 5150
F 0 "J1" H 3650 5550 50  0000 C CNN
F 1 "Conn_01x08" H 3650 4650 50  0000 C CNN
F 2 "" H 3650 5150 50  0001 C CNN
F 3 "" H 3650 5150 50  0001 C CNN
	1    3650 5150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR6
U 1 1 5A4AA1BA
P 3300 5700
F 0 "#PWR6" H 3300 5450 50  0001 C CNN
F 1 "GND" H 3300 5550 50  0000 C CNN
F 2 "" H 3300 5700 50  0001 C CNN
F 3 "" H 3300 5700 50  0001 C CNN
	1    3300 5700
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR5
U 1 1 5A4AA227
P 3100 5450
F 0 "#PWR5" H 3100 5300 50  0001 C CNN
F 1 "+3V3" H 3100 5590 50  0000 C CNN
F 2 "" H 3100 5450 50  0001 C CNN
F 3 "" H 3100 5450 50  0001 C CNN
	1    3100 5450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9100 1300 9100 1550
Wire Wire Line
	9100 1450 9300 1450
Connection ~ 9100 1450
Wire Wire Line
	9600 1750 9600 2000
Wire Wire Line
	9100 1850 10100 1850
Connection ~ 9600 1850
Wire Wire Line
	10100 1550 10100 1450
Wire Wire Line
	9900 1450 10250 1450
Wire Wire Line
	10250 1450 10250 1300
Connection ~ 10100 1450
Wire Wire Line
	2400 2150 2400 2200
Wire Wire Line
	2400 2200 2450 2200
Wire Wire Line
	2400 4100 2400 4150
Wire Wire Line
	5400 3950 5300 3950
Wire Wire Line
	5300 3950 5300 4100
Wire Wire Line
	5400 3850 5000 3850
Wire Wire Line
	7200 3950 7300 3950
Wire Wire Line
	6650 4450 6650 4550
Wire Wire Line
	4350 3100 4850 3100
Wire Wire Line
	4850 3100 4850 2850
Wire Wire Line
	4850 2850 5400 2850
Wire Wire Line
	4350 2900 4800 2900
Wire Wire Line
	4800 2900 4800 2750
Wire Wire Line
	4800 2750 5400 2750
Wire Wire Line
	4350 2200 7450 2200
Wire Wire Line
	7200 3750 7450 3750
Wire Wire Line
	7450 3750 7450 2200
Wire Wire Line
	1650 3300 1650 3400
Wire Wire Line
	1650 2600 1650 2700
Wire Wire Line
	4350 2700 4500 2700
Wire Wire Line
	4500 2700 4500 1800
Wire Wire Line
	4500 1800 2050 1800
Wire Wire Line
	2050 1800 2050 3000
Wire Wire Line
	4800 2000 5250 2000
Wire Wire Line
	4800 2000 4800 2500
Wire Wire Line
	4800 1650 4800 1700
Wire Wire Line
	5000 1650 5000 1700
Wire Wire Line
	4800 2500 4350 2500
Wire Wire Line
	6050 1650 6050 1700
Wire Wire Line
	5850 1650 5850 1700
Wire Wire Line
	6050 2000 5850 2000
Wire Wire Line
	5850 2000 5850 2400
Wire Wire Line
	5850 2400 5000 2400
Wire Wire Line
	5000 2400 5000 2600
Wire Wire Line
	5000 2600 4350 2600
Wire Wire Line
	5000 1700 5250 1700
Connection ~ 5000 2000
Wire Wire Line
	6750 1550 6750 1450
Wire Wire Line
	7150 1450 7150 1550
Wire Wire Line
	2400 4100 2450 4100
Wire Wire Line
	7550 1450 7550 1550
Wire Wire Line
	3300 5700 3300 5550
Wire Wire Line
	3300 5550 3450 5550
Wire Wire Line
	3100 5450 3450 5450
Wire Wire Line
	3450 5450 3450 5350
Wire Wire Line
	4350 4100 4350 4500
Wire Wire Line
	4350 4500 3000 4500
Wire Wire Line
	3000 4500 3000 5250
Wire Wire Line
	3000 5250 3450 5250
Wire Wire Line
	4350 4000 4400 4000
Wire Wire Line
	4400 4000 4400 4550
Wire Wire Line
	4400 4550 3050 4550
Wire Wire Line
	3050 4550 3050 5150
Wire Wire Line
	3050 5150 3450 5150
Wire Wire Line
	4350 3800 4450 3800
Wire Wire Line
	4450 3800 4450 4600
Wire Wire Line
	4450 4600 3100 4600
Wire Wire Line
	3100 4600 3100 5050
Wire Wire Line
	3100 5050 3450 5050
Wire Wire Line
	4350 3700 4500 3700
Wire Wire Line
	4500 3700 4500 4650
Wire Wire Line
	4500 4650 3150 4650
Wire Wire Line
	3150 4650 3150 4950
Wire Wire Line
	3150 4950 3450 4950
Wire Wire Line
	4350 3600 4550 3600
Wire Wire Line
	4550 3600 4550 4700
Wire Wire Line
	4550 4700 3200 4700
Wire Wire Line
	3200 4700 3200 4850
Wire Wire Line
	3200 4850 3450 4850
$EndSCHEMATC
