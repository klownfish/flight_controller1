EESchema Schematic File Version 4
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
Text GLabel 4350 1100 0    50   Input ~ 0
GND
Text GLabel 6800 1300 2    50   Input ~ 0
GND
Wire Wire Line
	6800 1300 6600 1300
Text GLabel 7050 1400 2    50   Input ~ 0
3V3
Text Label 4300 2300 0    50   ~ 0
MOSI
Text Label 4300 2400 0    50   ~ 0
MISO
Text GLabel 9550 850  0    50   Input ~ 0
3V3
Text GLabel 9550 950  0    50   Input ~ 0
GND
Text Label 7000 2000 2    50   ~ 0
SDA
Text Label 7000 1900 2    50   ~ 0
SCL
Wire Wire Line
	9450 1250 10100 1250
Wire Wire Line
	9450 1350 10100 1350
Wire Wire Line
	9450 1450 10100 1450
Wire Wire Line
	9450 1550 10100 1550
Text GLabel 950  1150 0    50   Input ~ 0
GND
Wire Wire Line
	950  1150 1350 1150
Wire Wire Line
	1350 1150 1350 1100
Wire Wire Line
	9450 1650 10100 1650
Text Label 9450 1650 0    50   ~ 0
RF_RST
Text Label 9450 1550 0    50   ~ 0
RF_CS
Text Label 9450 1450 0    50   ~ 0
MOSI
Text Label 9450 1350 0    50   ~ 0
MISO
Text Label 9450 1250 0    50   ~ 0
SCK
Wire Wire Line
	4250 1500 4650 1500
Text Label 4250 1500 0    50   ~ 0
RF_G0
Text Label 9450 1150 0    50   ~ 0
RF_G0
Wire Wire Line
	1450 1150 1450 1100
Wire Wire Line
	9550 950  10100 950 
Wire Wire Line
	9550 850  10100 850 
Text Notes 7400 7500 0    50   ~ 0
Rocket1 flight controller\n
Wire Wire Line
	6600 2100 7000 2100
Text Label 7000 2100 2    50   ~ 0
BAT_READ
Text GLabel 9100 5600 2    50   Input ~ 0
GND
Text Label 4200 2200 0    50   ~ 0
FLASH_CS
Wire Wire Line
	9450 1150 10100 1150
Text Label 6950 2500 2    50   ~ 0
SCK
Text Label 6950 2400 2    50   ~ 0
EXT_RX
Text Label 6950 2300 2    50   ~ 0
EXT_TX
Wire Wire Line
	4200 2200 4650 2200
Wire Wire Line
	4300 2400 4650 2400
Wire Wire Line
	4300 2300 4650 2300
Wire Wire Line
	4250 1700 4650 1700
Text Label 4250 1700 0    50   ~ 0
RF_CS
Text GLabel 4650 2700 0    50   Input ~ 0
GND
Wire Wire Line
	3250 1150 3400 1150
Text GLabel 3900 1150 3    50   Input ~ 0
5V
Text GLabel 7300 750  1    50   Input ~ 0
5V
Text GLabel 2850 1750 3    50   Input ~ 0
GND
Wire Wire Line
	3400 1200 3400 1150
Wire Wire Line
	2250 1150 2250 1200
Wire Wire Line
	2250 1650 2850 1650
Wire Wire Line
	3400 1500 3400 1650
Wire Wire Line
	2250 1500 2250 1650
Wire Wire Line
	2850 1600 2850 1650
Connection ~ 2850 1650
Wire Wire Line
	2850 1650 3400 1650
Wire Wire Line
	2850 1750 2850 1650
Wire Wire Line
	2450 1150 2250 1150
Wire Wire Line
	7800 2350 7500 2350
Text GLabel 7650 2350 3    50   Input ~ 0
3V3
Wire Wire Line
	7300 750  7300 1100
Wire Wire Line
	6600 1200 7300 1200
Wire Wire Line
	6700 1100 6600 1100
Wire Wire Line
	7000 1100 7300 1100
Connection ~ 7300 1100
Wire Wire Line
	7300 1100 7300 1200
Text GLabel 8750 5900 1    50   Input ~ 0
3V3
Text Notes 10600 7650 0    50   ~ 0
1.1
Text Label 6950 1800 2    50   ~ 0
RGB_TX
Wire Wire Line
	9950 5200 10400 5200
Text GLabel 10400 5200 2    50   Input ~ 0
GND
Text GLabel 10250 4600 2    50   Input ~ 0
5V
Wire Wire Line
	1050 4850 1050 4950
Wire Wire Line
	1050 4950 1250 4950
Wire Wire Line
	1050 4750 1050 4650
Wire Wire Line
	1050 4650 1250 4650
Wire Wire Line
	2200 5050 2000 5050
Wire Wire Line
	2000 5050 2000 4950
Connection ~ 2000 4950
Wire Wire Line
	2000 4950 2200 4950
Wire Wire Line
	2200 4850 2000 4850
Wire Wire Line
	2000 4850 2000 4950
Wire Wire Line
	2200 4750 2000 4750
Wire Wire Line
	2000 4750 2000 4650
Connection ~ 2000 4650
Wire Wire Line
	2000 4650 2200 4650
Wire Wire Line
	2200 4550 2000 4550
Wire Wire Line
	2000 4550 2000 4650
Wire Wire Line
	9100 5600 9000 5600
Text Label 3300 4650 2    50   ~ 0
P_IN_2
Wire Wire Line
	2950 4650 3300 4650
Text Label 3300 4950 2    50   ~ 0
P_EN
Wire Wire Line
	2950 4950 3300 4950
Wire Wire Line
	6600 1400 7050 1400
Text Label 4300 2100 0    50   ~ 0
P_IN_1
Text Label 4300 2000 0    50   ~ 0
P_IN_2
Text Label 4300 1900 0    50   ~ 0
P_IN_3
Text Label 8900 3750 2    50   ~ 0
P_EN
Wire Notes Line
	8650 6400 11150 6400
Wire Notes Line
	11150 6400 11150 5500
Wire Notes Line
	11150 5500 8650 5500
Wire Notes Line
	8650 5500 8650 6400
Text Notes 8700 5450 0    50   ~ 0
Flash memory - 128MBit
Wire Notes Line
	9250 1800 11150 1800
Wire Notes Line
	11150 1800 11150 550 
Wire Notes Line
	11150 550  9250 550 
Wire Notes Line
	9250 550  9250 1800
Text Notes 9300 500  0    50   ~ 0
Radio
Wire Wire Line
	7500 2050 7500 2000
Wire Wire Line
	7800 1900 7800 2050
Connection ~ 1250 4650
Connection ~ 1250 4950
Connection ~ 1900 4650
Wire Wire Line
	1900 4650 2000 4650
Connection ~ 1900 4950
Wire Wire Line
	1900 4950 2000 4950
Wire Wire Line
	1250 4950 1900 4950
Text GLabel 3000 4350 2    50   Input ~ 0
GND
Wire Wire Line
	2600 4350 2600 4400
Wire Wire Line
	2600 4350 2600 4300
Wire Wire Line
	2600 4300 2550 4300
Connection ~ 2600 4350
Wire Wire Line
	3000 4350 2900 4350
NoConn ~ 4650 2500
NoConn ~ 4650 2600
NoConn ~ 6600 2700
NoConn ~ 6600 2600
NoConn ~ 10100 1050
Wire Wire Line
	6950 2500 6600 2500
Wire Notes Line
	700  2450 4000 2450
Wire Notes Line
	4000 2450 4000 750 
Wire Notes Line
	4000 750  700  750 
Wire Notes Line
	700  750  700  2450
Text Notes 750  700  0    50   ~ 0
Power
Wire Wire Line
	2950 4850 3500 4850
Wire Wire Line
	1250 4650 1900 4650
Wire Notes Line
	9200 5300 11200 5300
Wire Notes Line
	11200 5300 11200 4300
Wire Notes Line
	9200 4300 11200 4300
Wire Notes Line
	9200 4300 9200 5300
Wire Notes Line
	8300 3100 9500 3100
Wire Notes Line
	9500 1850 8300 1850
Text Notes 9250 4250 0    50   ~ 0
RGB LED
Wire Wire Line
	1050 6050 1050 6150
Wire Wire Line
	1050 6150 1250 6150
Wire Wire Line
	1050 5950 1050 5850
Wire Wire Line
	1050 5850 1250 5850
Wire Wire Line
	2200 6250 2000 6250
Wire Wire Line
	2000 6250 2000 6150
Connection ~ 2000 6150
Wire Wire Line
	2000 6150 2200 6150
Wire Wire Line
	2200 6050 2000 6050
Wire Wire Line
	2000 6050 2000 6150
Wire Wire Line
	2200 5950 2000 5950
Wire Wire Line
	2000 5950 2000 5850
Connection ~ 2000 5850
Wire Wire Line
	2000 5850 2200 5850
Wire Wire Line
	2200 5750 2000 5750
Wire Wire Line
	2000 5750 2000 5850
Text Label 3300 5850 2    50   ~ 0
P_IN_3
Wire Wire Line
	2950 5850 3300 5850
Text Label 3300 6150 2    50   ~ 0
P_EN
Wire Wire Line
	2950 6150 3300 6150
Connection ~ 1250 5850
Connection ~ 1250 6150
Connection ~ 1900 5850
Wire Wire Line
	1900 5850 2000 5850
Connection ~ 1900 6150
Wire Wire Line
	1900 6150 2000 6150
Wire Wire Line
	1250 6150 1900 6150
Wire Wire Line
	2600 5500 2550 5500
Wire Wire Line
	2950 6050 3500 6050
Wire Wire Line
	1250 5850 1900 5850
Wire Wire Line
	1050 7300 1050 7400
Wire Wire Line
	1050 7400 1250 7400
Wire Wire Line
	1050 7200 1050 7100
Wire Wire Line
	1050 7100 1250 7100
Wire Wire Line
	2200 7500 2000 7500
Wire Wire Line
	2000 7500 2000 7400
Connection ~ 2000 7400
Wire Wire Line
	2000 7400 2200 7400
Wire Wire Line
	2200 7300 2000 7300
Wire Wire Line
	2000 7300 2000 7400
Wire Wire Line
	2200 7200 2000 7200
Wire Wire Line
	2000 7200 2000 7100
Connection ~ 2000 7100
Wire Wire Line
	2000 7100 2200 7100
Wire Wire Line
	2200 7000 2000 7000
Wire Wire Line
	2000 7000 2000 7100
Text Label 3300 7100 2    50   ~ 0
P_IN_4
Wire Wire Line
	2950 7100 3300 7100
Text Label 3300 7400 2    50   ~ 0
P_EN
Wire Wire Line
	2950 7400 3300 7400
Connection ~ 1250 7100
Connection ~ 1250 7400
Connection ~ 1900 7100
Wire Wire Line
	1900 7100 2000 7100
Connection ~ 1900 7400
Wire Wire Line
	1900 7400 2000 7400
Wire Wire Line
	1250 7400 1900 7400
Text GLabel 3000 6800 2    50   Input ~ 0
GND
Wire Wire Line
	2600 6800 2600 6850
Wire Wire Line
	2600 6800 2600 6750
Wire Wire Line
	2600 6750 2550 6750
Connection ~ 2600 6800
Wire Wire Line
	3000 6800 2900 6800
Wire Wire Line
	2950 7300 3500 7300
Wire Wire Line
	1250 7100 1900 7100
Wire Wire Line
	1050 3650 1050 3750
Wire Wire Line
	1050 3750 1250 3750
Wire Wire Line
	1050 3550 1050 3450
Wire Wire Line
	1050 3450 1250 3450
Wire Wire Line
	2200 3850 2000 3850
Wire Wire Line
	2000 3850 2000 3750
Connection ~ 2000 3750
Wire Wire Line
	2000 3750 2200 3750
Wire Wire Line
	2200 3650 2000 3650
Wire Wire Line
	2000 3650 2000 3750
Wire Wire Line
	2200 3550 2000 3550
Wire Wire Line
	2000 3550 2000 3450
Connection ~ 2000 3450
Wire Wire Line
	2000 3450 2200 3450
Wire Wire Line
	2200 3350 2000 3350
Wire Wire Line
	2000 3350 2000 3450
Text Label 3300 3450 2    50   ~ 0
P_IN_1
Wire Wire Line
	2950 3450 3300 3450
Text Label 3300 3750 2    50   ~ 0
P_EN
Wire Wire Line
	2950 3750 3300 3750
Connection ~ 1250 3450
Connection ~ 1250 3750
Connection ~ 1900 3450
Wire Wire Line
	1900 3450 2000 3450
Connection ~ 1900 3750
Wire Wire Line
	1900 3750 2000 3750
Wire Wire Line
	1250 3750 1900 3750
Wire Wire Line
	2600 3100 2550 3100
Wire Wire Line
	2950 3650 3500 3650
Wire Wire Line
	1250 3450 1900 3450
Wire Notes Line
	650  2800 650  7750
Wire Notes Line
	650  7750 4250 7750
Wire Notes Line
	650  2800 4250 2800
Wire Notes Line
	4250 2800 4250 7750
Text Label 7200 2200 2    50   ~ 0
UNSAFE_P_EN
Wire Wire Line
	6600 2000 7500 2000
Wire Wire Line
	6600 1900 7800 1900
Wire Wire Line
	6600 2200 7200 2200
Text Notes 9650 3050 0    50   ~ 0
Expansion port
Text Notes 8350 1800 0    50   ~ 0
Buzzer
Text Label 7900 3450 2    50   ~ 0
UNSAFE_P_EN
Wire Wire Line
	7350 3450 7900 3450
Wire Notes Line
	9150 5150 9150 3200
Text Notes 7050 3150 0    50   ~ 0
ARMING
Text Notes 700  2750 0    50   ~ 0
Pyro channels
Wire Notes Line
	9600 3700 11150 3700
Wire Notes Line
	11150 3700 11150 3100
Wire Notes Line
	9600 3100 11150 3100
Text GLabel 2050 7500 3    50   Input ~ 0
GND
Text GLabel 2050 6250 3    50   Input ~ 0
GND
Text GLabel 2050 5050 3    50   Input ~ 0
GND
Text GLabel 2050 3850 3    50   Input ~ 0
GND
Wire Wire Line
	2600 3100 2600 3200
Text Notes 2700 3250 0    50   ~ 0
decoupling capacitor shared
Wire Wire Line
	2600 5500 2600 5600
NoConn ~ 2950 3550
NoConn ~ 2950 5950
NoConn ~ 2950 7200
NoConn ~ 2950 4750
Wire Wire Line
	6950 2400 6600 2400
Wire Wire Line
	6600 2300 6950 2300
Wire Wire Line
	3500 7300 3500 6050
Connection ~ 3500 4850
Wire Wire Line
	3500 4850 3500 3650
Connection ~ 3500 6050
Wire Wire Line
	3500 6050 3500 5400
Text GLabel 3800 5400 2    50   Input ~ 0
GND
Connection ~ 3500 5400
Wire Wire Line
	3500 4850 3500 5400
Text Label 4300 1800 0    50   ~ 0
P_IN_4
Wire Wire Line
	4300 2100 4650 2100
Wire Wire Line
	4650 2000 4300 2000
Wire Wire Line
	4300 1900 4650 1900
Wire Wire Line
	4650 1800 4300 1800
Wire Notes Line
	7150 6450 8350 6450
Wire Notes Line
	8350 6450 8350 5550
Wire Notes Line
	8350 5550 7150 5550
Wire Notes Line
	7150 5550 7150 6450
Text Notes 7200 5500 0    50   ~ 0
Mechanical
Wire Notes Line
	8300 1850 8300 3100
Wire Wire Line
	8900 3750 8650 3750
Connection ~ 2250 1150
Wire Wire Line
	1750 1150 2250 1150
Connection ~ 1750 1150
Wire Wire Line
	1450 1150 1750 1150
Wire Wire Line
	1300 1750 1750 1750
Text Label 1300 1750 0    50   ~ 0
BAT_READ
Wire Wire Line
	1750 1750 1750 1800
Connection ~ 1750 1750
Wire Wire Line
	1750 2200 1750 2100
Wire Wire Line
	1750 1650 1750 1750
Text GLabel 1750 2200 3    50   Input ~ 0
GND
Wire Wire Line
	1750 1150 1750 1350
Text GLabel 1750 1250 0    50   Input ~ 0
12V
Connection ~ 3400 1150
Wire Wire Line
	3700 1150 3900 1150
Text GLabel 2550 6750 0    50   Input ~ 0
12V_LIMIT
Text GLabel 2550 5500 0    50   Input ~ 0
12V_LIMIT
Text GLabel 2550 4300 0    50   Input ~ 0
12V_LIMIT
Text GLabel 2550 3100 0    50   Input ~ 0
12V_LIMIT
$Comp
L custom:RFM69HCW U2
U 1 1 5FF35928
P 10300 950
F 0 "U2" H 10625 1275 50  0000 C CNN
F 1 "RFM69HCW" H 10625 1184 50  0000 C CNN
F 2 "custom:Adafruit-RFM69" H 10600 200 50  0001 C CNN
F 3 "" H 10600 200 50  0001 C CNN
	1    10300 950 
	1    0    0    -1  
$EndComp
$Comp
L custom:VXO7805-500-M U6
U 1 1 60AF0463
P 2850 1200
F 0 "U6" H 2850 1565 50  0000 C CNN
F 1 "VXO7805-500-M" H 2850 1474 50  0000 C CNN
F 2 "custom:VXO7805-500-M" H 2850 1350 50  0001 C CNN
F 3 "" H 2850 1350 50  0001 C CNN
	1    2850 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 60B1165C
P 3400 1350
F 0 "C4" H 3515 1396 50  0000 L CNN
F 1 "10u" H 3515 1305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 3438 1200 50  0001 C CNN
F 3 "~" H 3400 1350 50  0001 C CNN
	1    3400 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 60B188C5
P 2250 1350
F 0 "C3" H 2365 1396 50  0000 L CNN
F 1 "20u" H 2365 1305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2288 1200 50  0001 C CNN
F 3 "~" H 2250 1350 50  0001 C CNN
	1    2250 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 60B7B177
P 7800 2200
F 0 "R4" H 7870 2246 50  0000 L CNN
F 1 "4.7k" H 7870 2155 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 7730 2200 50  0001 C CNN
F 3 "~" H 7800 2200 50  0001 C CNN
	1    7800 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 60B7C0E8
P 7500 2200
F 0 "R3" H 7570 2246 50  0000 L CNN
F 1 "4.7k" H 7570 2155 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 7430 2200 50  0001 C CNN
F 3 "~" H 7500 2200 50  0001 C CNN
	1    7500 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D4
U 1 1 60BC9BA4
P 6850 1100
F 0 "D4" H 6850 883 50  0000 C CNN
F 1 "D_Schottky" H 6850 974 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-323_HandSoldering" H 6850 1100 50  0001 C CNN
F 3 "~" H 6850 1100 50  0001 C CNN
	1    6850 1100
	-1   0    0    1   
$EndComp
$Comp
L custom:xt-30 U5
U 1 1 60BF778C
P 1400 900
F 0 "U5" H 1528 888 50  0000 L CNN
F 1 "xt-30" H 1528 797 50  0000 L CNN
F 2 "Connector_AMASS:AMASS_XT30U-M_1x02_P5.0mm_Vertical" H 1400 950 50  0001 C CNN
F 3 "" H 1400 950 50  0001 C CNN
	1    1400 900 
	1    0    0    -1  
$EndComp
$Comp
L LED:WS2812B D2
U 1 1 60BC12A4
P 9950 4900
F 0 "D2" H 9900 4500 50  0000 L CNN
F 1 "WS2812B" H 9800 4400 50  0000 L CNN
F 2 "custom:ws2812-through-hole-5mm" H 10000 4600 50  0001 L TNN
F 3 "https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf" H 10050 4525 50  0001 L TNN
	1    9950 4900
	-1   0    0    1   
$EndComp
$Comp
L custom:terminal-2 U8
U 1 1 60C96ECC
P 950 4800
F 0 "U8" H 992 4535 50  0000 C CNN
F 1 "terminal-2" H 992 4626 50  0000 C CNN
F 2 "custom:terminal-small" H 1050 4650 50  0001 C CNN
F 3 "" H 1050 4650 50  0001 C CNN
	1    950  4800
	-1   0    0    1   
$EndComp
$Comp
L Device:D_TVS D8
U 1 1 60C9A0F2
P 1250 4800
F 0 "D8" V 1204 4880 50  0000 L CNN
F 1 "D_TVS" V 1295 4880 50  0000 L CNN
F 2 "Diode_SMD:D_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1250 4800 50  0001 C CNN
F 3 "~" H 1250 4800 50  0001 C CNN
	1    1250 4800
	0    1    1    0   
$EndComp
$Comp
L custom:teensy4 U1
U 1 1 608C0A1A
P 5050 1050
F 0 "U1" H 5625 1265 50  0000 C CNN
F 1 "teensy4" H 5625 1174 50  0000 C CNN
F 2 "custom:teensy4" H 5850 1150 50  0001 C CNN
F 3 "" H 5850 1150 50  0001 C CNN
	1    5050 1050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 61157282
P 1900 4800
F 0 "C6" H 2015 4846 50  0000 L CNN
F 1 "100n" H 2015 4755 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 1938 4650 50  0001 C CNN
F 3 "~" H 1900 4800 50  0001 C CNN
	1    1900 4800
	-1   0    0    1   
$EndComp
$Comp
L custom:BTN7030-1EPA U12
U 1 1 60CBBA72
P 2650 4850
F 0 "U12" H 2700 5250 50  0000 C CNN
F 1 "BTN7030-1EPA" H 2700 5150 50  0000 C CNN
F 2 "custom:Infineon_PG-TSDSO-14-22-bottom" H 2700 5150 50  0001 C CNN
F 3 "" H 2700 5150 50  0001 C CNN
	1    2650 4850
	-1   0    0    1   
$EndComp
$Comp
L Device:C C10
U 1 1 6118FBB7
P 2750 4350
F 0 "C10" V 2498 4350 50  0000 C CNN
F 1 "100n" V 2589 4350 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2788 4200 50  0001 C CNN
F 3 "~" H 2750 4350 50  0001 C CNN
	1    2750 4350
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_DPST_x2 SW1
U 1 1 61298F6A
P 8300 3550
F 0 "SW1" H 8250 3650 50  0001 C CNN
F 1 "SW_DPST_x2" H 8300 3694 50  0001 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8300 3550 50  0001 C CNN
F 3 "~" H 8300 3550 50  0001 C CNN
	1    8300 3550
	0    1    1    0   
$EndComp
$Comp
L custom:terminal-2 U9
U 1 1 61894837
P 950 6000
F 0 "U9" H 992 5735 50  0000 C CNN
F 1 "terminal-2" H 992 5826 50  0000 C CNN
F 2 "custom:terminal-small" H 1050 5850 50  0001 C CNN
F 3 "" H 1050 5850 50  0001 C CNN
	1    950  6000
	-1   0    0    1   
$EndComp
$Comp
L Device:D_TVS D10
U 1 1 6189483D
P 1250 6000
F 0 "D10" V 1204 6080 50  0000 L CNN
F 1 "D_TVS" V 1295 6080 50  0000 L CNN
F 2 "Diode_SMD:D_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1250 6000 50  0001 C CNN
F 3 "~" H 1250 6000 50  0001 C CNN
	1    1250 6000
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 61894859
P 3650 5400
F 0 "R8" V 3443 5400 50  0000 C CNN
F 1 "10k" V 3534 5400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 3580 5400 50  0001 C CNN
F 3 "~" H 3650 5400 50  0001 C CNN
	1    3650 5400
	0    1    1    0   
$EndComp
$Comp
L Device:C C7
U 1 1 61894860
P 1900 6000
F 0 "C7" H 2015 6046 50  0000 L CNN
F 1 "100n" H 2015 5955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 1938 5850 50  0001 C CNN
F 3 "~" H 1900 6000 50  0001 C CNN
	1    1900 6000
	-1   0    0    1   
$EndComp
$Comp
L custom:BTN7030-1EPA U13
U 1 1 6189486D
P 2650 6050
F 0 "U13" H 2700 6450 50  0000 C CNN
F 1 "BTN7030-1EPA" H 2700 6350 50  0000 C CNN
F 2 "custom:Infineon_PG-TSDSO-14-22-bottom" H 2700 6350 50  0001 C CNN
F 3 "" H 2700 6350 50  0001 C CNN
	1    2650 6050
	-1   0    0    1   
$EndComp
$Comp
L custom:terminal-2 U10
U 1 1 618A3F45
P 950 7250
F 0 "U10" H 992 6985 50  0000 C CNN
F 1 "terminal-2" H 992 7076 50  0000 C CNN
F 2 "custom:terminal-small" H 1050 7100 50  0001 C CNN
F 3 "" H 1050 7100 50  0001 C CNN
	1    950  7250
	-1   0    0    1   
$EndComp
$Comp
L Device:D_TVS D12
U 1 1 618A3F4B
P 1250 7250
F 0 "D12" V 1204 7330 50  0000 L CNN
F 1 "D_TVS" V 1295 7330 50  0000 L CNN
F 2 "Diode_SMD:D_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1250 7250 50  0001 C CNN
F 3 "~" H 1250 7250 50  0001 C CNN
	1    1250 7250
	0    1    1    0   
$EndComp
$Comp
L Device:C C8
U 1 1 618A3F6E
P 1900 7250
F 0 "C8" H 2015 7296 50  0000 L CNN
F 1 "100n" H 2015 7205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 1938 7100 50  0001 C CNN
F 3 "~" H 1900 7250 50  0001 C CNN
	1    1900 7250
	-1   0    0    1   
$EndComp
$Comp
L custom:BTN7030-1EPA U14
U 1 1 618A3F7B
P 2650 7300
F 0 "U14" H 2700 7700 50  0000 C CNN
F 1 "BTN7030-1EPA" H 2700 7600 50  0000 C CNN
F 2 "custom:Infineon_PG-TSDSO-14-22-bottom" H 2700 7600 50  0001 C CNN
F 3 "" H 2700 7600 50  0001 C CNN
	1    2650 7300
	-1   0    0    1   
$EndComp
$Comp
L Device:C C12
U 1 1 618A3F81
P 2750 6800
F 0 "C12" V 2498 6800 50  0000 C CNN
F 1 "100n" V 2589 6800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2788 6650 50  0001 C CNN
F 3 "~" H 2750 6800 50  0001 C CNN
	1    2750 6800
	0    1    1    0   
$EndComp
$Comp
L custom:terminal-2 U7
U 1 1 618C77D9
P 950 3600
F 0 "U7" H 992 3335 50  0000 C CNN
F 1 "terminal-2" H 992 3426 50  0000 C CNN
F 2 "custom:terminal-small" H 1050 3450 50  0001 C CNN
F 3 "" H 1050 3450 50  0001 C CNN
	1    950  3600
	-1   0    0    1   
$EndComp
$Comp
L Device:D_TVS D6
U 1 1 618C77DF
P 1250 3600
F 0 "D6" V 1204 3680 50  0000 L CNN
F 1 "D_TVS" V 1295 3680 50  0000 L CNN
F 2 "Diode_SMD:D_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1250 3600 50  0001 C CNN
F 3 "~" H 1250 3600 50  0001 C CNN
	1    1250 3600
	0    1    1    0   
$EndComp
$Comp
L Device:C C5
U 1 1 618C7802
P 1900 3600
F 0 "C5" H 2015 3646 50  0000 L CNN
F 1 "100n" H 2015 3555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 1938 3450 50  0001 C CNN
F 3 "~" H 1900 3600 50  0001 C CNN
	1    1900 3600
	-1   0    0    1   
$EndComp
$Comp
L custom:BTN7030-1EPA U11
U 1 1 618C780F
P 2650 3650
F 0 "U11" H 2700 4050 50  0000 C CNN
F 1 "BTN7030-1EPA" H 2700 3950 50  0000 C CNN
F 2 "custom:Infineon_PG-TSDSO-14-22-bottom" H 2700 3950 50  0001 C CNN
F 3 "" H 2700 3950 50  0001 C CNN
	1    2650 3650
	-1   0    0    1   
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 60C608F5
P 7350 5700
F 0 "H3" H 7450 5746 50  0000 L CNN
F 1 "Logo" H 7450 5655 50  0000 L CNN
F 2 "custom:avatar" H 7350 5700 50  0001 C CNN
F 3 "~" H 7350 5700 50  0001 C CNN
	1    7350 5700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 60B194B9
P 1750 1950
F 0 "R2" H 1820 1996 50  0000 L CNN
F 1 "2k" H 1820 1905 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 1680 1950 50  0001 C CNN
F 3 "~" H 1750 1950 50  0001 C CNN
	1    1750 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 60B19085
P 1750 1500
F 0 "R1" H 1820 1546 50  0000 L CNN
F 1 "6.04k" H 1820 1455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 1680 1500 50  0001 C CNN
F 3 "~" H 1750 1500 50  0001 C CNN
	1    1750 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D1
U 1 1 60FD69FF
P 3550 1150
F 0 "D1" H 3550 933 50  0000 C CNN
F 1 "D_Schottky" H 3550 1024 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-323_HandSoldering" H 3550 1150 50  0001 C CNN
F 3 "~" H 3550 1150 50  0001 C CNN
	1    3550 1150
	-1   0    0    1   
$EndComp
$Comp
L Device:C C2
U 1 1 5FF15A10
P 9000 5750
F 0 "C2" H 9150 5750 50  0000 C CNN
F 1 "100p" H 9150 5650 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9038 5600 50  0001 C CNN
F 3 "https://www.mouser.se/datasheet/2/212/KEM_C1007_X8R_ULTRA_150C_SMD-1102703.pdf" H 9000 5750 50  0001 C CNN
	1    9000 5750
	-1   0    0    1   
$EndComp
NoConn ~ 9650 4900
Wire Wire Line
	7350 3750 7550 3750
Text GLabel 7900 4050 3    50   Input ~ 0
GND
$Comp
L Device:R R10
U 1 1 60D2E25B
P 7900 3900
F 0 "R10" H 7970 3946 50  0000 L CNN
F 1 "4.7k" H 7970 3855 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 7830 3900 50  0001 C CNN
F 3 "~" H 7900 3900 50  0001 C CNN
	1    7900 3900
	1    0    0    -1  
$EndComp
$Comp
L custom:buffer U15
U 1 1 60DC1EDD
P 7500 4250
F 0 "U15" V 7300 4450 50  0000 L CNN
F 1 "NL17SZ17" V 7200 3900 50  0000 L CNN
F 2 "custom:SC-74-a" H 7300 4500 50  0001 C CNN
F 3 "" H 7300 4500 50  0001 C CNN
	1    7500 4250
	0    1    1    0   
$EndComp
Wire Wire Line
	7550 4000 7550 3750
Connection ~ 7550 3750
Wire Wire Line
	7550 3750 7900 3750
Text GLabel 7250 4350 0    50   Input ~ 0
GND
Text GLabel 7850 4350 2    50   Input ~ 0
3V3
$Comp
L Device:LED D7
U 1 1 60DFFEAA
P 8600 5000
F 0 "D7" H 8500 5100 50  0000 L CNN
F 1 "SAFE" H 8450 5200 50  0000 L CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8600 5000 50  0001 C CNN
F 3 "~" H 8600 5000 50  0001 C CNN
	1    8600 5000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R11
U 1 1 60E1E942
P 8600 4600
F 0 "R11" V 8393 4600 50  0000 C CNN
F 1 "400" V 8484 4600 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 8530 4600 50  0001 C CNN
F 3 "~" H 8600 4600 50  0001 C CNN
	1    8600 4600
	0    1    1    0   
$EndComp
$Comp
L Device:LED D5
U 1 1 60E5C291
P 8300 4600
F 0 "D5" H 8450 4500 50  0000 R CNN
F 1 "ARMED" H 8450 4400 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8300 4600 50  0001 C CNN
F 3 "~" H 8300 4600 50  0001 C CNN
	1    8300 4600
	-1   0    0    1   
$EndComp
Wire Wire Line
	8150 5000 8050 5000
Text GLabel 8750 4600 2    50   Input ~ 0
GND
Wire Wire Line
	8050 5000 8050 4700
Wire Wire Line
	8050 4600 8150 4600
Connection ~ 8050 4700
Wire Wire Line
	8050 4700 8050 4600
Wire Wire Line
	7550 4700 8050 4700
Connection ~ 7900 3750
Wire Wire Line
	7900 3750 7950 3750
Wire Wire Line
	7350 3450 7350 3750
$Comp
L Device:R R12
U 1 1 60F93FFE
P 8300 5000
F 0 "R12" V 8093 5000 50  0000 C CNN
F 1 "400" V 8184 5000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 8230 5000 50  0001 C CNN
F 3 "~" H 8300 5000 50  0001 C CNN
	1    8300 5000
	0    1    1    0   
$EndComp
Text GLabel 8750 5000 2    50   Input ~ 0
3V3
Wire Notes Line
	7000 5150 7000 3200
Wire Notes Line
	7000 3200 9150 3200
Wire Notes Line
	9500 1850 9500 3100
Wire Wire Line
	10200 2100 10400 2100
Text GLabel 10200 2100 0    50   Input ~ 0
3V3
Text GLabel 10200 2250 0    50   Input ~ 0
GND
Wire Wire Line
	10200 2250 10400 2250
Wire Wire Line
	10400 2250 10400 2300
Wire Wire Line
	10000 2400 10400 2400
Wire Wire Line
	10000 2500 10400 2500
Text Label 10000 2400 0    50   ~ 0
SCL
Text Label 10000 2500 0    50   ~ 0
SDA
Wire Notes Line
	11150 2900 11150 2000
Wire Notes Line
	11150 2000 9950 2000
Wire Notes Line
	9950 2000 9950 2900
Wire Notes Line
	9950 2900 11150 2900
Text Notes 10000 1950 0    50   ~ 0
10-DOF
NoConn ~ 10400 2600
NoConn ~ 10400 2700
NoConn ~ 10400 2800
$Comp
L custom:GY91 U3
U 1 1 608CCC88
P 10500 2250
F 0 "U3" V 10649 2678 50  0000 L CNN
F 1 "GY91" V 10740 2678 50  0000 L CNN
F 2 "custom:GY91" H 10700 2750 50  0001 C CNN
F 3 "" H 10700 2750 50  0001 C CNN
	1    10500 2250
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole H6
U 1 1 60D92CBE
P 7750 5700
F 0 "H6" H 7850 5746 50  0000 L CNN
F 1 "Aesir" H 7850 5655 50  0000 L CNN
F 2 "custom:aesir" H 7750 5700 50  0001 C CNN
F 3 "~" H 7750 5700 50  0001 C CNN
	1    7750 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 1100 4650 1100
Wire Wire Line
	4650 1200 4350 1200
Wire Wire Line
	4650 1300 4350 1300
Text Label 4350 1300 0    50   ~ 0
CAN_TX
$Comp
L Interface_CAN_LIN:SN65HVD230 U4
U 1 1 60E22CF3
P 5150 4050
F 0 "U4" H 4800 4450 50  0000 C CNN
F 1 "SN65HVD230" H 4800 4350 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 5150 3550 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn65hvd230.pdf" H 5050 4450 50  0001 C CNN
	1    5150 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 60E25C0F
P 5800 3800
F 0 "R13" H 5870 3846 50  0000 L CNN
F 1 "120" H 5870 3755 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 5730 3800 50  0001 C CNN
F 3 "~" H 5800 3800 50  0001 C CNN
	1    5800 3800
	0    -1   -1   0   
$EndComp
Text GLabel 5150 4500 3    50   Input ~ 0
GND
Text GLabel 5150 3650 1    50   Input ~ 0
3V3
Text GLabel 6400 4250 0    50   Input ~ 0
GND
$Comp
L Device:D_Schottky D9
U 1 1 60F3ACE1
P 6200 4350
F 0 "D9" V 6154 4430 50  0000 L CNN
F 1 "D_Schottky" V 6245 4430 50  0000 L CNN
F 2 "Diode_SMD:D_SOD-323_HandSoldering" H 6200 4350 50  0001 C CNN
F 3 "~" H 6200 4350 50  0001 C CNN
	1    6200 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 4350 5950 4350
Text GLabel 5950 4350 0    50   Input ~ 0
5V
Text Label 4450 4050 0    50   ~ 0
CAN_RX
Text Label 4450 3950 0    50   ~ 0
CAN_TX
Wire Wire Line
	4450 3950 4750 3950
Wire Wire Line
	4450 4050 4750 4050
Wire Wire Line
	5150 4450 5150 4500
Wire Wire Line
	5150 4450 4750 4450
Wire Wire Line
	4750 4450 4750 4250
Connection ~ 5150 4450
$Comp
L custom:w25nxx U16
U 1 1 61030402
P 9900 5850
F 0 "U16" H 9775 6165 50  0000 C CNN
F 1 "w25nxx" H 9775 6074 50  0000 C CNN
F 2 "Package_SO:SOIC-16W_7.5x10.3mm_P1.27mm" H 9900 6150 50  0001 C CNN
F 3 "" H 9900 6150 50  0001 C CNN
	1    9900 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 5900 9350 5800
Wire Wire Line
	9350 5800 9400 5800
Wire Wire Line
	9350 5900 9400 5900
Connection ~ 9350 5900
Wire Wire Line
	9350 5900 9350 6000
Wire Wire Line
	9350 6000 9400 6000
Connection ~ 9000 5900
Wire Wire Line
	9000 5900 9350 5900
Text Label 10350 5900 2    50   ~ 0
SCK
Text Label 10350 6000 2    50   ~ 0
MOSI
Text Label 9200 6250 0    50   ~ 0
MISO
Wire Wire Line
	9200 6250 9400 6250
Wire Wire Line
	10350 6000 10150 6000
Wire Wire Line
	10350 5900 10150 5900
Text GLabel 10200 6150 2    50   Input ~ 0
GND
Text GLabel 10200 6250 2    50   Input ~ 0
3V3
Wire Wire Line
	10150 6150 10200 6150
Wire Wire Line
	10150 6250 10200 6250
Text Label 9400 6150 2    50   ~ 0
FLASH_CS
$Comp
L Device:R R5
U 1 1 60B09806
P 8900 6150
F 0 "R5" H 8970 6196 50  0000 L CNN
F 1 "10k" H 8970 6105 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 8830 6150 50  0001 C CNN
F 3 "~" H 8900 6150 50  0001 C CNN
	1    8900 6150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9050 6150 9400 6150
Wire Wire Line
	8750 5900 8750 6150
Wire Wire Line
	8750 5900 9000 5900
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 6120BFA2
P 6650 4150
F 0 "J2" H 6600 3800 50  0000 L CNN
F 1 "JST_XH_4" H 6450 3700 50  0000 L CNN
F 2 "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical" H 6650 4150 50  0001 C CNN
F 3 "~" H 6650 4150 50  0001 C CNN
	1    6650 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 61230C8F
P 5350 3700
F 0 "C1" V 5100 3700 50  0000 C CNN
F 1 "100n" V 5200 3700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 5388 3550 50  0001 C CNN
F 3 "~" H 5350 3700 50  0001 C CNN
	1    5350 3700
	0    1    1    0   
$EndComp
Text GLabel 5550 3700 1    50   Input ~ 0
GND
Wire Wire Line
	5150 3650 5150 3700
Wire Wire Line
	5550 3700 5500 3700
Wire Wire Line
	5200 3700 5150 3700
Connection ~ 5150 3700
Wire Wire Line
	5150 3700 5150 3750
Wire Notes Line
	4400 4750 6950 4750
Wire Notes Line
	6950 4750 6950 3400
Wire Notes Line
	4400 3400 6950 3400
Wire Notes Line
	4400 3400 4400 4750
Text Label 4250 1600 0    50   ~ 0
RF_RST
Wire Wire Line
	4250 1600 4650 1600
Text Label 6800 1500 2    50   ~ 0
BUZZ
Wire Wire Line
	6800 1500 6600 1500
Text Label 4350 1200 0    50   ~ 0
CAN_RX
Wire Notes Line
	7000 5150 9150 5150
Text Notes 4450 3350 0    50   ~ 0
CAN\n
$Comp
L Device:C C9
U 1 1 61474EC2
P 9700 4600
F 0 "C9" V 9500 4500 50  0000 C CNN
F 1 "100n" V 9600 4500 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9738 4450 50  0001 C CNN
F 3 "~" H 9700 4600 50  0001 C CNN
	1    9700 4600
	0    1    1    0   
$EndComp
Wire Wire Line
	9850 4600 9950 4600
Connection ~ 9950 4600
Wire Wire Line
	9550 4600 9450 4600
Text GLabel 9450 4600 3    50   Input ~ 0
GND
Wire Notes Line
	9600 3700 9600 3100
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J1
U 1 1 60E4FCC0
P 10200 3500
F 0 "J1" H 10250 3817 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 10250 3726 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical" H 10200 3500 50  0001 C CNN
F 3 "~" H 10200 3500 50  0001 C CNN
	1    10200 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	10550 3400 10500 3400
Wire Wire Line
	10550 3400 10550 3350
Text GLabel 10950 3350 2    50   Input ~ 0
5V
Wire Wire Line
	10650 3350 10550 3350
Wire Wire Line
	10700 3500 10500 3500
Text Label 10700 3500 2    50   ~ 0
SDA
Text Label 10800 3600 2    50   ~ 0
EXT_RX
Wire Wire Line
	10800 3600 10500 3600
Wire Wire Line
	9700 3600 10000 3600
Text Label 9700 3600 0    50   ~ 0
EXT_TX
Wire Wire Line
	10000 3500 9750 3500
Text Label 9750 3500 0    50   ~ 0
SCL
Wire Wire Line
	9850 3350 9950 3350
Wire Wire Line
	9950 3400 10000 3400
Wire Wire Line
	9950 3350 9950 3400
Text GLabel 9850 3350 0    50   Input ~ 0
GND
$Comp
L Device:D_Schottky D11
U 1 1 60C77537
P 10800 3350
F 0 "D11" H 10800 3500 50  0000 C CNN
F 1 "D_Schottky" H 10900 3450 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-323_HandSoldering" H 10800 3350 50  0001 C CNN
F 3 "~" H 10800 3350 50  0001 C CNN
	1    10800 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:Buzzer BZ1
U 1 1 609840B9
P 8600 2300
F 0 "BZ1" H 8752 2329 50  0000 L CNN
F 1 "Buzzer" H 8752 2238 50  0000 L CNN
F 2 "custom:CX-0905C" V 8575 2400 50  0001 C CNN
F 3 "https://www.mouser.se/datasheet/2/670/cpt_147_85t-2306768.pdf" V 8575 2400 50  0001 C CNN
	1    8600 2300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8700 2400 8700 2450
Text GLabel 8500 2400 3    50   Input ~ 0
5V
$Comp
L Device:R R9
U 1 1 60D04245
P 9000 2900
F 0 "R9" H 9070 2946 50  0000 L CNN
F 1 "1k" H 9070 2855 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 8930 2900 50  0001 C CNN
F 3 "~" H 9000 2900 50  0001 C CNN
	1    9000 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 60BECE16
P 9200 2600
F 0 "R6" H 9130 2554 50  0000 R CNN
F 1 "10k" H 9130 2645 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 9130 2600 50  0001 C CNN
F 3 "~" H 9200 2600 50  0001 C CNN
	1    9200 2600
	-1   0    0    1   
$EndComp
Wire Wire Line
	9200 3050 9450 3050
Wire Wire Line
	9000 3050 9200 3050
Connection ~ 9200 3050
Wire Wire Line
	9200 2750 9200 3050
Text Label 9450 3050 2    50   ~ 0
BUZZ
Text GLabel 9250 2450 2    50   Input ~ 0
GND
Wire Wire Line
	8700 2450 8800 2450
Wire Wire Line
	9250 2450 9200 2450
$Comp
L Transistor_BJT:MMBT3904 Q1
U 1 1 60DFD24D
P 9000 2550
F 0 "Q1" V 9328 2550 50  0000 C CNN
F 1 "MMBT3904" V 9237 2550 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9200 2475 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/2N3903-D.PDF" H 9000 2550 50  0001 L CNN
	1    9000 2550
	0    -1   -1   0   
$EndComp
Connection ~ 9200 2450
$Comp
L power:Earth #PWR0101
U 1 1 60E66FD6
P 5300 7350
F 0 "#PWR0101" H 5300 7100 50  0001 C CNN
F 1 "Earth" H 5300 7200 50  0001 C CNN
F 2 "" H 5300 7350 50  0001 C CNN
F 3 "~" H 5300 7350 50  0001 C CNN
	1    5300 7350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 7350 5400 7350
Text GLabel 5400 7350 2    50   Input ~ 0
GND
$Comp
L Device:R R14
U 1 1 60EE48E8
P 8100 3750
F 0 "R14" H 8170 3796 50  0000 L CNN
F 1 "1k" H 8170 3705 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 8030 3750 50  0001 C CNN
F 3 "~" H 8100 3750 50  0001 C CNN
	1    8100 3750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8250 3750 8300 3750
Wire Wire Line
	8300 3350 8600 3350
Text GLabel 8600 3350 2    50   Input ~ 0
GND
Wire Wire Line
	8350 3750 8300 3750
Connection ~ 8300 3750
$Comp
L Device:R R7
U 1 1 60E0DAC5
P 8500 3750
F 0 "R7" H 8570 3796 50  0000 L CNN
F 1 "1k" H 8570 3705 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 8430 3750 50  0001 C CNN
F 3 "~" H 8500 3750 50  0001 C CNN
	1    8500 3750
	0    -1   -1   0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 60F9797B
P 7350 6000
F 0 "H1" H 7450 6003 50  0000 L CNN
F 1 "MountingHole_Pad" H 7450 5958 50  0001 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2_Pad_Via" H 7350 6000 50  0001 C CNN
F 3 "~" H 7350 6000 50  0001 C CNN
	1    7350 6000
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 60FCE71F
P 7750 6000
F 0 "H4" H 7850 6003 50  0000 L CNN
F 1 "MountingHole_Pad" H 7850 5958 50  0001 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2_Pad_Via" H 7750 6000 50  0001 C CNN
F 3 "~" H 7750 6000 50  0001 C CNN
	1    7750 6000
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 60FCE8F7
P 7350 6300
F 0 "H2" H 7450 6303 50  0000 L CNN
F 1 "MountingHole_Pad" H 7450 6258 50  0001 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2_Pad_Via" H 7350 6300 50  0001 C CNN
F 3 "~" H 7350 6300 50  0001 C CNN
	1    7350 6300
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H5
U 1 1 60FCEBC5
P 7750 6300
F 0 "H5" H 7850 6303 50  0000 L CNN
F 1 "MountingHole_Pad" H 7850 6258 50  0001 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2_Pad_Via" H 7750 6300 50  0001 C CNN
F 3 "~" H 7750 6300 50  0001 C CNN
	1    7750 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 6100 7750 6100
Wire Wire Line
	7750 6100 8000 6100
Wire Wire Line
	8000 6100 8000 6250
Wire Wire Line
	8000 6400 7750 6400
Connection ~ 7750 6100
Wire Wire Line
	7350 6400 7750 6400
Connection ~ 7750 6400
Wire Wire Line
	8000 6250 8050 6250
Connection ~ 8000 6250
Wire Wire Line
	8000 6250 8000 6400
Text GLabel 8050 6250 2    50   Input ~ 0
GND
Wire Wire Line
	6350 4350 6450 4350
Wire Wire Line
	6400 4250 6450 4250
$Comp
L Device:Jumper_NC_Small JP1
U 1 1 60E9BDE6
P 6150 3800
F 0 "JP1" H 6150 4012 50  0000 C CNN
F 1 "Jumper_NC_Small" H 6300 3900 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 6150 3800 50  0001 C CNN
F 3 "~" H 6150 3800 50  0001 C CNN
	1    6150 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 3800 6050 3800
Wire Wire Line
	5550 4150 6250 4150
Wire Wire Line
	5550 4050 5650 4050
Wire Wire Line
	5650 3800 5650 4050
Connection ~ 5650 4050
Wire Wire Line
	5650 4050 6450 4050
Wire Wire Line
	6250 3800 6250 4150
Connection ~ 6250 4150
Wire Wire Line
	6250 4150 6450 4150
Wire Wire Line
	9950 4600 10250 4600
$Comp
L custom:buffer U17
U 1 1 60E44EE6
P 10850 4850
F 0 "U17" V 10650 5050 50  0000 L CNN
F 1 "NL17SZ17" V 10550 4500 50  0000 L CNN
F 2 "custom:SC-74-a" H 10650 5100 50  0001 C CNN
F 3 "" H 10650 5100 50  0001 C CNN
	1    10850 4850
	-1   0    0    1   
$EndComp
Wire Wire Line
	10750 4600 10750 4500
Wire Wire Line
	10750 4500 10850 4500
Text GLabel 10850 4500 2    50   Input ~ 0
GND
Text GLabel 10800 5250 2    50   Input ~ 0
5V
Wire Wire Line
	10800 5250 10750 5250
Wire Wire Line
	10750 5250 10750 5200
Wire Wire Line
	10400 4900 10250 4900
Wire Wire Line
	6600 1800 6950 1800
$Comp
L Device:Jumper_NO_Small JP2
U 1 1 60F77C26
P 6200 5650
F 0 "JP2" H 6200 5835 50  0000 C CNN
F 1 "Jumper" H 6200 5744 50  0000 C CNN
F 2 "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical" H 6200 5650 50  0001 C CNN
F 3 "~" H 6200 5650 50  0001 C CNN
	1    6200 5650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R15
U 1 1 60F8CC65
P 6100 5800
F 0 "R15" H 6170 5846 50  0000 L CNN
F 1 "2k" H 6170 5755 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 6030 5800 50  0001 C CNN
F 3 "~" H 6100 5800 50  0001 C CNN
	1    6100 5800
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR0102
U 1 1 60F8DD9B
P 6100 5950
F 0 "#PWR0102" H 6100 5700 50  0001 C CNN
F 1 "Earth" H 6100 5800 50  0001 C CNN
F 2 "" H 6100 5950 50  0001 C CNN
F 3 "~" H 6100 5950 50  0001 C CNN
	1    6100 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 5650 6500 5650
Text GLabel 6500 5650 2    50   Input ~ 0
3V3
Wire Wire Line
	6100 5650 5750 5650
Connection ~ 6100 5650
Wire Wire Line
	6600 1600 7000 1600
Text Label 7000 1600 2    50   ~ 0
LIFTOFF
Text Label 5750 5650 0    50   ~ 0
LIFTOFF
Text Label 11100 4650 2    50   ~ 0
RGB_TX
Wire Wire Line
	11100 4900 11100 4650
Wire Wire Line
	11100 4650 10850 4650
$Comp
L Device:Jumper_NO_Small JP3
U 1 1 610789BA
P 1200 3050
F 0 "JP3" H 1200 3235 50  0000 C CNN
F 1 "Jumper_NO_Small" H 1200 3144 50  0000 C CNN
F 2 "custom:R_1812_4532Metric_Pad1.30x3.40mm_HandSolder_no_silk" H 1200 3050 50  0001 C CNN
F 3 "~" H 1200 3050 50  0001 C CNN
	1    1200 3050
	1    0    0    -1  
$EndComp
Text GLabel 1100 3050 0    50   Input ~ 0
12V
Text GLabel 1300 3050 2    50   Input ~ 0
12V_LIMIT
$EndSCHEMATC
