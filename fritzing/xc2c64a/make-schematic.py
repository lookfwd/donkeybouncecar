ffrom="""IO_22
IO_23
IO_24
RB2
RB3
RB4/IC7
RB5/IC8
RB6
RB7
RB8
VDD0
VSS0
OSC1
OSC2
RC13/TX1
RC14/RX1
INT0/RE8
OC2/RD1
RD3
VSS1

VDD1
RD2
OC1/RD0
RF6/SCK1
PGD/SCL
PGC/SDA
RF5/TX2
RF4/RX2
RF1
RF0
VSS2
VDD2
RE5
RE4
RE3
RE2
RE1
RE0
AVSS
AVDD

DSPIC30F4011


"""


to="""IO_22
        IO_23
        IO_24
        GND
        IO_25
        IO_26
        VDDIO1
        IO_27
        TDI
        TMS
        TCK

        IO_28
        IO_29
        IO_30
        VDDINT
        IO_31
        GND
        IO_32
        IO_15
        IO_14
        
        GND
        VDDIO2
        IO_10
        IO_9
        IO_8
        IO_7
        IO_6
        IO_5
        IO_4

        IO_3
        VDDAUX
        IO_2
        IO_1
        IO_0
        IO_16
        IO_17
        IO_18
        IO_19
        IO_20
        IO_21

XC2C64A_VQ44

"""

for i in list(range(21, 41))[::-1]:
    ffrom += f"\n>{i}</text>"
    to  +=  f"\n>{i+4}<\/text>"

for i in list(range(20, 40))[::-1]:
    ffrom += f"\nconnector{i}terminal"
    to  +=  f"\nconnector{i+4}terminal"
    ffrom += f"\nconnector{i}pin"
    to  +=  f"\nconnector{i+4}pin"


l1 = [i.strip() for i in ffrom.split('\n') if i.strip()]
l2 = [i.strip() for i in to.split('\n') if i.strip()]

print("sed " + " ".join(f"-e 's/{i.replace('/', '.')}/{j}/'" for i, j in zip(l1, l2)))