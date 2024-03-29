EESchema Schematic File Version 4
LIBS:balance-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L balance-rescue:TM4C123GH6PMI-TM4C123GH6PMI U1
U 1 1 5DB37352
P 2800 3550
F 0 "U1" H 2800 6026 50  0000 C CNN
F 1 "TM4C123GH6PMI" H 2800 3550 50  0001 L BNN
F 2 "TM4C123GH6PMI:QFP50P1200X1200X160-64N" H 2800 3550 50  0001 L BNN
F 3 "MCU 32-Bit Tiva C ARM Cortex M4F RISC 256KB Flash 1.2V/3.3V 64-Pin LQFP Tray/Tube" H 2800 3550 50  0001 L BNN
F 4 "https://www.digikey.com/product-detail/en/texas-instruments/TM4C123GH6PMI/296-35848-ND/4036462?utm_source=snapeda&utm_medium=aggregator&utm_campaign=symbol" H 2800 3550 50  0001 L BNN "Field4"
F 5 "LQFP-64 Texas Instruments" H 2800 3550 50  0001 L BNN "Field5"
F 6 "Texas Instruments" H 2800 3550 50  0001 L BNN "Field6"
F 7 "296-35848-ND" H 2800 3550 50  0001 L BNN "Field7"
F 8 "TM4C123GH6PMI" H 2800 3550 50  0001 L BNN "Field8"
	1    2800 3550
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_Servo M1
U 1 1 5DB3AFD2
P 5900 1450
F 0 "M1" H 6232 1515 50  0000 L CNN
F 1 "Motor_Servo" H 6232 1424 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 5900 1260 50  0001 C CNN
F 3 "http://forums.parallax.com/uploads/attachments/46831/74481.png" H 5900 1260 50  0001 C CNN
	1    5900 1450
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_Servo M2
U 1 1 5DB3BF9E
P 5900 2000
F 0 "M2" H 6232 2065 50  0000 L CNN
F 1 "Motor_Servo" H 6232 1974 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 5900 1810 50  0001 C CNN
F 3 "http://forums.parallax.com/uploads/attachments/46831/74481.png" H 5900 1810 50  0001 C CNN
	1    5900 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 5350 3600 5450
Wire Wire Line
	3600 5550 3600 5450
Connection ~ 3600 5450
Wire Wire Line
	3600 5650 3600 5550
Connection ~ 3600 5550
Wire Wire Line
	3600 5650 3600 5750
Connection ~ 3600 5650
Connection ~ 3600 5750
$Comp
L power:GND #PWR03
U 1 1 5DB4901D
P 3600 5900
F 0 "#PWR03" H 3600 5650 50  0001 C CNN
F 1 "GND" H 3605 5727 50  0000 C CNN
F 2 "" H 3600 5900 50  0001 C CNN
F 3 "" H 3600 5900 50  0001 C CNN
	1    3600 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 5750 3600 5850
Wire Wire Line
	3600 5900 3600 5850
Connection ~ 3600 5850
Wire Wire Line
	3600 1950 3650 1950
Wire Wire Line
	3650 1950 3650 1850
Wire Wire Line
	3650 1250 3600 1250
Wire Wire Line
	3600 1350 3650 1350
Connection ~ 3650 1350
Wire Wire Line
	3650 1350 3650 1250
Wire Wire Line
	3600 1450 3650 1450
Connection ~ 3650 1450
Wire Wire Line
	3650 1450 3650 1350
Wire Wire Line
	3600 1550 3650 1550
Connection ~ 3650 1550
Wire Wire Line
	3650 1550 3650 1450
Wire Wire Line
	3600 1850 3650 1850
Connection ~ 3650 1850
Wire Wire Line
	3650 1850 3650 1550
$Comp
L power:GND #PWR06
U 1 1 5DB52FAE
P 4550 1550
F 0 "#PWR06" H 4550 1300 50  0001 C CNN
F 1 "GND" V 4555 1422 50  0000 R CNN
F 2 "" H 4550 1550 50  0001 C CNN
F 3 "" H 4550 1550 50  0001 C CNN
	1    4550 1550
	0    -1   -1   0   
$EndComp
$Comp
L Device:Crystal Y1
U 1 1 5DB55642
P 4950 5050
F 0 "Y1" H 4950 5318 50  0000 C CNN
F 1 "16MHz" H 4950 5227 50  0000 C CNN
F 2 "Crystal:Crystal_SMD_5032-2Pin_5.0x3.2mm" H 4950 5050 50  0001 C CNN
F 3 "~" H 4950 5050 50  0001 C CNN
	1    4950 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 5DB5A83D
P 4800 5400
F 0 "C9" H 4686 5354 50  0000 R CNN
F 1 "10pF" H 4686 5445 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4838 5250 50  0001 C CNN
F 3 "~" H 4800 5400 50  0001 C CNN
	1    4800 5400
	1    0    0    1   
$EndComp
$Comp
L Device:C C10
U 1 1 5DB5AD7A
P 5100 5400
F 0 "C10" H 5215 5446 50  0000 L CNN
F 1 "10pF" H 5215 5355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5138 5250 50  0001 C CNN
F 3 "~" H 5100 5400 50  0001 C CNN
	1    5100 5400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5DB5B346
P 4950 5750
F 0 "#PWR08" H 4950 5500 50  0001 C CNN
F 1 "GND" H 4955 5577 50  0000 C CNN
F 2 "" H 4950 5750 50  0001 C CNN
F 3 "" H 4950 5750 50  0001 C CNN
	1    4950 5750
	1    0    0    -1  
$EndComp
Text Label 4800 4700 2    50   ~ 0
OSC1
Text Label 5100 4700 0    50   ~ 0
OSC0
Text Label 3600 3150 0    50   ~ 0
OSC1
Text Label 2000 1450 2    50   ~ 0
OSC0
Wire Wire Line
	4800 5050 4800 5250
Wire Wire Line
	5100 5050 5100 5250
Wire Wire Line
	4800 5550 4800 5650
Wire Wire Line
	4800 5650 4950 5650
Wire Wire Line
	4950 5650 4950 5750
Wire Wire Line
	5100 5550 5100 5650
Wire Wire Line
	5100 5650 4950 5650
Connection ~ 4950 5650
Wire Wire Line
	4800 4700 4800 5050
Connection ~ 4800 5050
Wire Wire Line
	5100 4700 5100 5050
Connection ~ 5100 5050
$Comp
L power:+3.3V #PWR04
U 1 1 5DB63A87
P 3650 900
F 0 "#PWR04" H 3650 750 50  0001 C CNN
F 1 "+3.3V" H 3665 1073 50  0000 C CNN
F 2 "" H 3650 900 50  0001 C CNN
F 3 "" H 3650 900 50  0001 C CNN
	1    3650 900 
	1    0    0    -1  
$EndComp
Connection ~ 3650 1250
$Comp
L Device:C C7
U 1 1 5DB64FAA
P 4200 1350
F 0 "C7" V 3948 1350 50  0000 C CNN
F 1 ".1uF" V 4039 1350 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4238 1200 50  0001 C CNN
F 3 "~" H 4200 1350 50  0001 C CNN
	1    4200 1350
	0    1    1    0   
$EndComp
$Comp
L Device:C C3
U 1 1 5DB78254
P 4150 3100
F 0 "C3" V 3898 3100 50  0000 C CNN
F 1 ".1uF" V 3989 3100 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4188 2950 50  0001 C CNN
F 3 "~" H 4150 3100 50  0001 C CNN
	1    4150 3100
	0    1    1    0   
$EndComp
$Comp
L Device:C C2
U 1 1 5DB799E7
P 4150 2700
F 0 "C2" V 3898 2700 50  0000 C CNN
F 1 ".1uF" V 4000 2700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4188 2550 50  0001 C CNN
F 3 "~" H 4150 2700 50  0001 C CNN
	1    4150 2700
	0    1    1    0   
$EndComp
$Comp
L Device:C C4
U 1 1 5DB7AD11
P 4150 3500
F 0 "C4" V 3898 3500 50  0000 C CNN
F 1 ".1uF" V 3989 3500 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4188 3350 50  0001 C CNN
F 3 "~" H 4150 3500 50  0001 C CNN
	1    4150 3500
	0    1    1    0   
$EndComp
$Comp
L Device:C C5
U 1 1 5DB7B21B
P 4150 3900
F 0 "C5" V 3898 3900 50  0000 C CNN
F 1 "3.3uF" V 3989 3900 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4188 3750 50  0001 C CNN
F 3 "~" H 4150 3900 50  0001 C CNN
	1    4150 3900
	0    1    1    0   
$EndComp
$Comp
L Device:C C6
U 1 1 5DB7B71C
P 4150 4300
F 0 "C6" V 3898 4300 50  0000 C CNN
F 1 "3.3uF" V 3989 4300 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4188 4150 50  0001 C CNN
F 3 "~" H 4150 4300 50  0001 C CNN
	1    4150 4300
	0    1    1    0   
$EndComp
Wire Wire Line
	3650 900  3650 1250
$Comp
L power:GND #PWR05
U 1 1 5DB86887
P 4450 2300
F 0 "#PWR05" H 4450 2050 50  0001 C CNN
F 1 "GND" V 4455 2172 50  0000 R CNN
F 2 "" H 4450 2300 50  0001 C CNN
F 3 "" H 4450 2300 50  0001 C CNN
	1    4450 2300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3650 1950 3650 2300
Wire Wire Line
	3650 2300 4000 2300
Connection ~ 3650 1950
$Comp
L power:+5V #PWR016
U 1 1 5DBA82FF
P 7400 1150
F 0 "#PWR016" H 7400 1000 50  0001 C CNN
F 1 "+5V" H 7415 1323 50  0000 C CNN
F 2 "" H 7400 1150 50  0001 C CNN
F 3 "" H 7400 1150 50  0001 C CNN
	1    7400 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5DBA946A
P 7400 1600
F 0 "#PWR017" H 7400 1350 50  0001 C CNN
F 1 "GND" H 7405 1427 50  0000 C CNN
F 2 "" H 7400 1600 50  0001 C CNN
F 3 "" H 7400 1600 50  0001 C CNN
	1    7400 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 1150 7400 1300
Wire Wire Line
	7400 1300 7550 1300
Wire Wire Line
	7400 1600 7400 1500
Wire Wire Line
	7400 1500 7550 1500
$Comp
L power:+5V #PWR09
U 1 1 5DBBBBD0
P 5400 1150
F 0 "#PWR09" H 5400 1000 50  0001 C CNN
F 1 "+5V" H 5415 1323 50  0000 C CNN
F 2 "" H 5400 1150 50  0001 C CNN
F 3 "" H 5400 1150 50  0001 C CNN
	1    5400 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5DBBC1C2
P 5500 2400
F 0 "#PWR010" H 5500 2150 50  0001 C CNN
F 1 "GND" H 5505 2227 50  0000 C CNN
F 2 "" H 5500 2400 50  0001 C CNN
F 3 "" H 5500 2400 50  0001 C CNN
	1    5500 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 1150 5400 1450
Wire Wire Line
	5400 1450 5600 1450
Wire Wire Line
	5400 1450 5400 2000
Wire Wire Line
	5400 2000 5600 2000
Connection ~ 5400 1450
Wire Wire Line
	5600 1550 5500 1550
Wire Wire Line
	5500 1550 5500 1750
Wire Wire Line
	5600 2100 5500 2100
Connection ~ 5500 2100
Wire Wire Line
	5500 2100 5500 2300
Text Label 5100 1900 2    50   ~ 0
Servo1
Text Label 5100 1350 2    50   ~ 0
Servo0
$Comp
L Regulator_Linear:LM2937xS U2
U 1 1 5DBE8F86
P 5850 3250
F 0 "U2" H 5850 3492 50  0000 C CNN
F 1 "LM2937xS" H 5850 3401 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-263-3_TabPin2" H 5850 3475 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm2937.pdf" H 5850 3200 50  0001 C CNN
	1    5850 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 5DBEA3D8
P 5200 3550
F 0 "C11" H 5315 3596 50  0000 L CNN
F 1 "0.1uF" H 5315 3505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5238 3400 50  0001 C CNN
F 3 "~" H 5200 3550 50  0001 C CNN
	1    5200 3550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C12
U 1 1 5DBEA98E
P 7000 3550
F 0 "C12" H 7115 3596 50  0000 L CNN
F 1 "10uF" H 7115 3505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7038 3400 50  0001 C CNN
F 3 "~" H 7000 3550 50  0001 C CNN
	1    7000 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 3250 5200 3250
Wire Wire Line
	5200 3400 5200 3250
Connection ~ 5200 3250
Wire Wire Line
	5200 3250 5550 3250
Wire Wire Line
	7000 3250 7000 3400
Wire Wire Line
	5200 3700 5200 3850
Wire Wire Line
	5200 3850 5850 3850
Wire Wire Line
	7000 3850 7000 3700
Wire Wire Line
	5850 3550 5850 3850
Connection ~ 5850 3850
$Comp
L power:+5V #PWR07
U 1 1 5DBF7399
P 4950 3250
F 0 "#PWR07" H 4950 3100 50  0001 C CNN
F 1 "+5V" V 4965 3378 50  0000 L CNN
F 2 "" H 4950 3250 50  0001 C CNN
F 3 "" H 4950 3250 50  0001 C CNN
	1    4950 3250
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR015
U 1 1 5DBF7B3E
P 7200 3250
F 0 "#PWR015" H 7200 3100 50  0001 C CNN
F 1 "+3.3V" V 7215 3378 50  0000 L CNN
F 2 "" H 7200 3250 50  0001 C CNN
F 3 "" H 7200 3250 50  0001 C CNN
	1    7200 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	7000 3250 7200 3250
Connection ~ 7000 3250
$Comp
L power:GND #PWR011
U 1 1 5DBFA4D4
P 5850 4050
F 0 "#PWR011" H 5850 3800 50  0001 C CNN
F 1 "GND" H 5855 3877 50  0000 C CNN
F 2 "" H 5850 4050 50  0001 C CNN
F 3 "" H 5850 4050 50  0001 C CNN
	1    5850 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 3850 5850 4050
Wire Wire Line
	5850 3850 6700 3850
Wire Wire Line
	6150 3250 6200 3250
$Comp
L Device:R R2
U 1 1 5DCA4F28
P 6200 3500
F 0 "R2" H 6270 3546 50  0000 L CNN
F 1 "68ohm" H 6270 3455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6130 3500 50  0001 C CNN
F 3 "~" H 6200 3500 50  0001 C CNN
	1    6200 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5DCA58B4
P 6550 3650
F 0 "D1" H 6543 3395 50  0000 C CNN
F 1 "LED_GREEN" H 6543 3486 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" H 6550 3650 50  0001 C CNN
F 3 "~" H 6550 3650 50  0001 C CNN
	1    6550 3650
	-1   0    0    1   
$EndComp
Wire Wire Line
	6200 3250 6200 3350
Connection ~ 6200 3250
Wire Wire Line
	6200 3250 7000 3250
Wire Wire Line
	6200 3650 6400 3650
Wire Wire Line
	6700 3650 6700 3850
Connection ~ 6700 3850
Wire Wire Line
	6700 3850 7000 3850
$Comp
L Connector:Conn_01x05_Male J2
U 1 1 5DCADF27
P 1300 3550
F 0 "J2" H 1408 3931 50  0000 C CNN
F 1 "JTAG" H 1408 3840 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 1300 3550 50  0001 C CNN
F 3 "~" H 1300 3550 50  0001 C CNN
	1    1300 3550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5DCB811D
P 1700 3850
F 0 "#PWR02" H 1700 3600 50  0001 C CNN
F 1 "GND" H 1705 3677 50  0000 C CNN
F 2 "" H 1700 3850 50  0001 C CNN
F 3 "" H 1700 3850 50  0001 C CNN
	1    1700 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 3750 1700 3750
Wire Wire Line
	1700 3750 1700 3850
$Comp
L Connector:Conn_01x04_Female J5
U 1 1 5DCBD4A2
P 7450 2600
F 0 "J5" H 7478 2576 50  0000 L CNN
F 1 "UART (Bluetooth)" H 7478 2485 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 7450 2600 50  0001 C CNN
F 3 "~" H 7450 2600 50  0001 C CNN
	1    7450 2600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR018
U 1 1 5DCBE660
P 7100 2350
F 0 "#PWR018" H 7100 2200 50  0001 C CNN
F 1 "+5V" H 7115 2523 50  0000 C CNN
F 2 "" H 7100 2350 50  0001 C CNN
F 3 "" H 7100 2350 50  0001 C CNN
	1    7100 2350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5DCBF339
P 6900 2600
F 0 "#PWR014" H 6900 2350 50  0001 C CNN
F 1 "GND" V 6905 2472 50  0000 R CNN
F 2 "" H 6900 2600 50  0001 C CNN
F 3 "" H 6900 2600 50  0001 C CNN
	1    6900 2600
	0    1    1    0   
$EndComp
Text Label 7000 2700 2    50   ~ 0
RX
Text Label 7000 2800 2    50   ~ 0
TX
Wire Wire Line
	7100 2350 7100 2500
Wire Wire Line
	7100 2500 7250 2500
Wire Wire Line
	6900 2600 7250 2600
Wire Wire Line
	7000 2700 7250 2700
Wire Wire Line
	7000 2800 7250 2800
Text Label 2000 1750 2    50   ~ 0
RX
Text Label 2000 1850 2    50   ~ 0
TX
$Comp
L touchscreen:Resistive_Touch J1
U 1 1 5DCF4582
P 1050 5100
F 0 "J1" V 1465 5117 50  0000 C CNN
F 1 "Resistive_Touch" V 1374 5117 50  0000 C CNN
F 2 "Connector_FFC-FPC:TE_84952-4_1x04-1MP_P1.0mm_Horizontal" H 1050 5450 50  0001 C CNN
F 3 "" H 1050 5100 50  0001 C CNN
	1    1050 5100
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR020
U 1 1 5DD0B846
P 8200 5900
F 0 "#PWR020" H 8200 5650 50  0001 C CNN
F 1 "GND" H 8205 5727 50  0000 C CNN
F 2 "" H 8200 5900 50  0001 C CNN
F 3 "" H 8200 5900 50  0001 C CNN
	1    8200 5900
	1    0    0    -1  
$EndComp
Text Label 7600 5550 2    50   ~ 0
BZZZ
$Comp
L Device:R R3
U 1 1 5DD15560
P 6550 4950
F 0 "R3" H 6620 4996 50  0000 L CNN
F 1 "10k" H 6620 4905 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6480 4950 50  0001 C CNN
F 3 "~" H 6550 4950 50  0001 C CNN
	1    6550 4950
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5DD161A1
P 6550 5550
F 0 "SW1" V 6596 5502 50  0000 R CNN
F 1 "SW_Push" V 6505 5502 50  0000 R CNN
F 2 "2-1437565-8:SW_SMD_2-1437565-8_4_4.50_6.00_6.00_5.00" H 6550 5750 50  0001 C CNN
F 3 "~" H 6550 5750 50  0001 C CNN
	1    6550 5550
	0    -1   -1   0   
$EndComp
Text Label 6300 5200 2    50   ~ 0
RESET
$Comp
L power:+3.3V #PWR012
U 1 1 5DD2FE89
P 6550 4650
F 0 "#PWR012" H 6550 4500 50  0001 C CNN
F 1 "+3.3V" H 6565 4823 50  0000 C CNN
F 2 "" H 6550 4650 50  0001 C CNN
F 3 "" H 6550 4650 50  0001 C CNN
	1    6550 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5DD30841
P 6550 5900
F 0 "#PWR013" H 6550 5650 50  0001 C CNN
F 1 "GND" H 6555 5727 50  0000 C CNN
F 2 "" H 6550 5900 50  0001 C CNN
F 3 "" H 6550 5900 50  0001 C CNN
	1    6550 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 4650 6550 4800
Wire Wire Line
	6550 5100 6550 5200
Wire Wire Line
	6550 5200 6300 5200
Wire Wire Line
	6550 5200 6550 5350
Connection ~ 6550 5200
Wire Wire Line
	6550 5750 6550 5900
$Comp
L Device:R R1
U 1 1 5DD72687
P 1550 1550
F 0 "R1" V 1343 1550 50  0000 C CNN
F 1 "1Mohm" V 1434 1550 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1480 1550 50  0001 C CNN
F 3 "~" H 1550 1550 50  0001 C CNN
	1    1550 1550
	0    1    1    0   
$EndComp
Wire Wire Line
	1700 1550 2000 1550
$Comp
L power:+3.3V #PWR01
U 1 1 5DD7C9A7
P 1250 1550
F 0 "#PWR01" H 1250 1400 50  0001 C CNN
F 1 "+3.3V" V 1265 1678 50  0000 L CNN
F 2 "" H 1250 1550 50  0001 C CNN
F 3 "" H 1250 1550 50  0001 C CNN
	1    1250 1550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1250 1550 1400 1550
Text Label 2000 2950 2    50   ~ 0
Servo0
Text Label 2000 3150 2    50   ~ 0
Servo1
$Comp
L Transistor_BJT:MMBT3904 Q1
U 1 1 5DDD413F
P 8100 5550
F 0 "Q1" H 8291 5596 50  0000 L CNN
F 1 "MMBT3904" H 8291 5505 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8300 5475 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3904.pdf" H 8100 5550 50  0001 L CNN
	1    8100 5550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5DDE1DD4
P 7750 5550
F 0 "R4" V 7543 5550 50  0000 C CNN
F 1 "100ohm" V 7634 5550 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 7680 5550 50  0001 C CNN
F 3 "~" H 7750 5550 50  0001 C CNN
	1    7750 5550
	0    1    1    0   
$EndComp
Wire Wire Line
	8200 5200 8200 5350
$Comp
L power:+3.3V #PWR019
U 1 1 5DE66A59
P 8200 4600
F 0 "#PWR019" H 8200 4450 50  0001 C CNN
F 1 "+3.3V" H 8215 4773 50  0000 C CNN
F 2 "" H 8200 4600 50  0001 C CNN
F 3 "" H 8200 4600 50  0001 C CNN
	1    8200 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 4600 8200 4800
Wire Wire Line
	1700 5050 2000 5050
Wire Wire Line
	1700 5150 2000 5150
Wire Wire Line
	1700 5250 2000 5250
Wire Wire Line
	1500 3350 2000 3350
Wire Wire Line
	1500 3450 2000 3450
Wire Wire Line
	1500 3550 2000 3550
Wire Wire Line
	1500 3650 2000 3650
$Comp
L Transistor_BJT:MMBT3904 Q2
U 1 1 5DF15346
P 8950 3000
F 0 "Q2" H 9141 3046 50  0000 L CNN
F 1 "MMBT3904" H 9141 2955 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9150 2925 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3904.pdf" H 8950 3000 50  0001 L CNN
	1    8950 3000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5DF15B4E
P 8600 3000
F 0 "R5" V 8393 3000 50  0000 C CNN
F 1 "100ohm" V 8484 3000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 8530 3000 50  0001 C CNN
F 3 "~" H 8600 3000 50  0001 C CNN
	1    8600 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	9050 1700 9050 1900
$Comp
L power:GND #PWR022
U 1 1 5DF1A08A
P 9600 3450
F 0 "#PWR022" H 9600 3200 50  0001 C CNN
F 1 "GND" H 9605 3277 50  0000 C CNN
F 2 "" H 9600 3450 50  0001 C CNN
F 3 "" H 9600 3450 50  0001 C CNN
	1    9600 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5DF1B160
P 9050 2050
F 0 "R6" H 9120 2096 50  0000 L CNN
F 1 "50ohm" H 9120 2005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 8980 2050 50  0001 C CNN
F 3 "~" H 9050 2050 50  0001 C CNN
	1    9050 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 5DF1CF54
P 9050 2500
F 0 "D2" V 9089 2383 50  0000 R CNN
F 1 "LED_BLUE" V 8998 2383 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" H 9050 2500 50  0001 C CNN
F 3 "~" H 9050 2500 50  0001 C CNN
	1    9050 2500
	0    -1   -1   0   
$EndComp
Text Label 8450 3000 2    50   ~ 0
Blue
$Comp
L Transistor_BJT:MMBT3904 Q3
U 1 1 5DF5A832
P 10150 3000
F 0 "Q3" H 10341 3046 50  0000 L CNN
F 1 "MMBT3904" H 10341 2955 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 10350 2925 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3904.pdf" H 10150 3000 50  0001 L CNN
	1    10150 3000
	-1   0    0    -1  
$EndComp
$Comp
L Device:LED D3
U 1 1 5DF68862
P 10050 2500
F 0 "D3" V 10089 2382 50  0000 R CNN
F 1 "LED_RED" V 9998 2382 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" H 10050 2500 50  0001 C CNN
F 3 "~" H 10050 2500 50  0001 C CNN
	1    10050 2500
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R7
U 1 1 5DF68FDB
P 10050 2050
F 0 "R7" H 10120 2096 50  0000 L CNN
F 1 "68ohm" H 10120 2005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9980 2050 50  0001 C CNN
F 3 "~" H 10050 2050 50  0001 C CNN
	1    10050 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 1700 9600 1700
Wire Wire Line
	9600 1700 9600 1500
Wire Wire Line
	10050 1900 10050 1700
Wire Wire Line
	10050 1700 9600 1700
Connection ~ 9600 1700
Wire Wire Line
	9050 3350 9600 3350
Wire Wire Line
	9600 3350 9600 3450
Wire Wire Line
	9600 3350 10050 3350
Connection ~ 9600 3350
$Comp
L Device:R R8
U 1 1 5DF96E59
P 10500 3000
F 0 "R8" V 10293 3000 50  0000 C CNN
F 1 "100ohm" V 10384 3000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10430 3000 50  0001 C CNN
F 3 "~" H 10500 3000 50  0001 C CNN
	1    10500 3000
	0    1    1    0   
$EndComp
Text Label 10650 3000 0    50   ~ 0
Red
Text Label 2000 5350 2    50   ~ 0
Blue
Text Label 2000 5750 2    50   ~ 0
Red
$Comp
L Connector:Conn_01x04_Male J3
U 1 1 5E002740
P 1500 5050
F 0 "J3" H 1608 5331 50  0000 C CNN
F 1 "Conn_01x04_Male" H 1608 5240 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 1500 5050 50  0001 C CNN
F 3 "~" H 1500 5050 50  0001 C CNN
	1    1500 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 4950 1700 4950
Connection ~ 1700 4950
Wire Wire Line
	1700 4950 2000 4950
Wire Wire Line
	1700 5050 1200 5050
Connection ~ 1700 5050
Wire Wire Line
	1700 5150 1200 5150
Connection ~ 1700 5150
Wire Wire Line
	1700 5250 1200 5250
Connection ~ 1700 5250
$Comp
L power:+3.3V #PWR021
U 1 1 5DF1614C
P 9600 1500
F 0 "#PWR021" H 9600 1350 50  0001 C CNN
F 1 "+3.3V" H 9615 1673 50  0000 C CNN
F 2 "" H 9600 1500 50  0001 C CNN
F 3 "" H 9600 1500 50  0001 C CNN
	1    9600 1500
	1    0    0    -1  
$EndComp
$Comp
L balance-rescue:CMT-8530S-SMT-CMT-8530S-SMT LS1
U 1 1 5E11A03B
P 8200 5000
F 0 "LS1" H 8304 5046 50  0000 L CNN
F 1 "CMT-8530S-SMT" H 8304 4955 50  0000 L CNN
F 2 "CMT-8530S-SMT:CUI_CMT-8530S-SMT" H 8200 5000 50  0001 L BNN
F 3 "CUI" H 8200 5000 50  0001 L BNN
F 4 "8.5 mm, 3.6 Vo-p, 90 dB, Surface Mount _SMT_, Magnetic Audio Transducer Buzzer" H 8200 5000 50  0001 L BNN "Field4"
F 5 "CMT-8530S-SMT-TR" H 8200 5000 50  0001 L BNN "Field5"
F 6 "Unavailable" H 8200 5000 50  0001 L BNN "Field6"
F 7 "https://www.cuidevices.com/product/audio/buzzers/audio-transducers/cmt-8530s-smt-tr?utm_source=snapeda.com&utm_medium=referral&utm_campaign=snapedaBOM" H 8200 5000 50  0001 L BNN "Field7"
F 8 "None" H 8200 5000 50  0001 L BNN "Field8"
F 9 "CUI Devices" H 8200 5000 50  0001 L BNN "Field9"
	1    8200 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 5750 8200 5900
Text Label 2000 1250 2    50   ~ 0
RESET
$Comp
L Connector_Generic:Conn_02x03_Top_Bottom J6
U 1 1 5E1A3466
P 9700 4350
F 0 "J6" H 9750 4667 50  0000 C CNN
F 1 "Conn_02x03_Top_Bottom" H 9750 4576 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical" H 9700 4350 50  0001 C CNN
F 3 "~" H 9700 4350 50  0001 C CNN
	1    9700 4350
	1    0    0    -1  
$EndComp
Text Label 2000 4250 2    50   ~ 0
PD1
Text Label 2000 4350 2    50   ~ 0
PD2
Text Label 2000 4450 2    50   ~ 0
PD3
Text Label 9500 4350 2    50   ~ 0
PD1
Text Label 9500 4450 2    50   ~ 0
PD2
Text Label 10000 4250 0    50   ~ 0
PD3
$Comp
L Device:C C8
U 1 1 5E287F4C
P 4200 1750
F 0 "C8" V 3948 1750 50  0000 C CNN
F 1 "3.3uF" V 4039 1750 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4238 1600 50  0001 C CNN
F 3 "~" H 4200 1750 50  0001 C CNN
	1    4200 1750
	0    1    1    0   
$EndComp
Wire Wire Line
	4350 1350 4450 1350
Wire Wire Line
	4450 1350 4450 1550
Wire Wire Line
	4450 1550 4550 1550
Wire Wire Line
	4450 1550 4450 1750
Wire Wire Line
	4450 1750 4350 1750
Connection ~ 4450 1550
Wire Wire Line
	4050 1750 3900 1750
Wire Wire Line
	3900 1750 3900 1650
Wire Wire Line
	3900 1350 4050 1350
Wire Wire Line
	3600 1750 3900 1750
Connection ~ 3900 1750
Wire Wire Line
	3600 1650 3900 1650
Connection ~ 3900 1650
Wire Wire Line
	3900 1650 3900 1350
Connection ~ 4000 2300
$Comp
L Device:C C1
U 1 1 5DB77D16
P 4150 2300
F 0 "C1" V 3898 2300 50  0000 C CNN
F 1 ".1uF" V 3989 2300 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4188 2150 50  0001 C CNN
F 3 "~" H 4150 2300 50  0001 C CNN
	1    4150 2300
	0    1    1    0   
$EndComp
Connection ~ 4000 2700
Connection ~ 4300 2700
Connection ~ 4300 3100
Connection ~ 4300 3500
Wire Wire Line
	4000 2300 4000 2700
Wire Wire Line
	4300 2300 4300 2700
Wire Wire Line
	4300 3500 4300 3900
Wire Wire Line
	4300 3100 4300 3500
Wire Wire Line
	4300 2700 4300 3100
Wire Wire Line
	4300 2300 4450 2300
Connection ~ 4300 2300
Wire Wire Line
	4000 2700 4000 3100
Connection ~ 4000 3100
Wire Wire Line
	4000 3100 4000 3500
Connection ~ 4000 3500
Wire Wire Line
	4000 3500 4000 3900
Connection ~ 4000 3900
Wire Wire Line
	4000 3900 4000 4300
Wire Wire Line
	4300 4300 4300 3900
Connection ~ 4300 3900
Text Label 9500 5250 2    50   ~ 0
PA2
Text Label 9500 5350 2    50   ~ 0
PA3
Text Label 9500 5450 2    50   ~ 0
PA4
Text Label 9500 5550 2    50   ~ 0
PA5
Text Label 9500 5650 2    50   ~ 0
PA6
Text Label 2000 1950 2    50   ~ 0
PA2
Text Label 2000 2050 2    50   ~ 0
PA3
Text Label 2000 2150 2    50   ~ 0
PA4
Text Label 2000 2250 2    50   ~ 0
PA5
Text Label 2000 2350 2    50   ~ 0
PA6
Text Label 2000 2450 2    50   ~ 0
PA7
$Comp
L power:+3.3V #PWR024
U 1 1 5E3F294F
P 10350 4350
F 0 "#PWR024" H 10350 4200 50  0001 C CNN
F 1 "+3.3V" V 10365 4478 50  0000 L CNN
F 2 "" H 10350 4350 50  0001 C CNN
F 3 "" H 10350 4350 50  0001 C CNN
	1    10350 4350
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR023
U 1 1 5E3F361D
P 10200 4550
F 0 "#PWR023" H 10200 4300 50  0001 C CNN
F 1 "GND" H 10205 4377 50  0000 C CNN
F 2 "" H 10200 4550 50  0001 C CNN
F 3 "" H 10200 4550 50  0001 C CNN
	1    10200 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 4350 10350 4350
Wire Wire Line
	10000 4450 10200 4450
Wire Wire Line
	10200 4450 10200 4550
$Comp
L Device:C C13
U 1 1 5DB8D9AF
P 5250 1600
F 0 "C13" H 5136 1554 50  0000 R CNN
F 1 "10uF" H 5136 1645 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5288 1450 50  0001 C CNN
F 3 "~" H 5250 1600 50  0001 C CNN
	1    5250 1600
	1    0    0    1   
$EndComp
$Comp
L Device:C C14
U 1 1 5DB93690
P 5250 2150
F 0 "C14" H 5136 2104 50  0000 R CNN
F 1 "10uF" H 5136 2195 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5288 2000 50  0001 C CNN
F 3 "~" H 5250 2150 50  0001 C CNN
	1    5250 2150
	1    0    0    1   
$EndComp
Wire Wire Line
	5100 1350 5600 1350
Wire Wire Line
	5100 1900 5600 1900
Wire Wire Line
	5250 1450 5400 1450
Wire Wire Line
	5250 1750 5500 1750
Connection ~ 5500 1750
Wire Wire Line
	5500 1750 5500 2100
Wire Wire Line
	5250 2000 5400 2000
Connection ~ 5400 2000
Wire Wire Line
	5250 2300 5500 2300
Wire Wire Line
	5500 2300 5500 2400
Connection ~ 5500 2300
Text Label 2000 4150 2    50   ~ 0
PD0
Text Label 9500 4250 2    50   ~ 0
PD0
Wire Wire Line
	9050 2200 9050 2350
Wire Wire Line
	9050 2650 9050 2800
Wire Wire Line
	9050 3200 9050 3350
Wire Wire Line
	10050 3350 10050 3200
Wire Wire Line
	10050 2800 10050 2650
Wire Wire Line
	10050 2350 10050 2200
Text Label 2000 5450 2    50   ~ 0
BZZZ
Text Label 2000 4050 2    50   ~ 0
PC7
Text Label 2000 3950 2    50   ~ 0
PC6
Text Label 2000 3850 2    50   ~ 0
PC5
Text Label 2000 3750 2    50   ~ 0
PC4
$Comp
L Connector_Generic:Conn_02x05_Top_Bottom J7
U 1 1 5DBC45D4
P 9700 5450
F 0 "J7" H 9750 5867 50  0000 C CNN
F 1 "Conn_02x05_Top_Bottom" H 9750 5776 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical" H 9700 5450 50  0001 C CNN
F 3 "~" H 9700 5450 50  0001 C CNN
	1    9700 5450
	1    0    0    -1  
$EndComp
Text Label 10000 5250 0    50   ~ 0
PA7
Text Label 10000 5350 0    50   ~ 0
PC4
Text Label 10000 5450 0    50   ~ 0
PC5
Text Label 10000 5550 0    50   ~ 0
PC6
Text Label 10000 5650 0    50   ~ 0
PC7
$Comp
L Connector:Barrel_Jack_Switch J4
U 1 1 5DC06347
P 7850 1400
F 0 "J4" H 7620 1442 50  0000 R CNN
F 1 "Barrel_Jack_Switch" H 7620 1351 50  0000 R CNN
F 2 "Connector_BarrelJack:BarrelJack_CUI_PJ-102AH_Horizontal" H 7900 1360 50  0001 C CNN
F 3 "~" H 7900 1360 50  0001 C CNN
	1    7850 1400
	-1   0    0    -1  
$EndComp
Text Notes 7050 3750 0    50   ~ 0
less than 3 ohm ESR required
$EndSCHEMATC
