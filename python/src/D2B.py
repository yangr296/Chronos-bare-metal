# This is a python library that converts stim parameters in decimal
# to binary for direct use in SPI communication with the DAC. 
################################################################################
# DAC8832(Texas Instruments) info:
# Biopolar mode 
# (MSB)1111 1111 1111 1111 = +V_REF * (32,767 / 32,768)
# (MSB)1000 0000 0000 0001 = +V_REF * (1 / 32,768)
# (MSB)1000 0000 0000 0000 = 0V
# (MSB)0111 1111 1111 1111 = -V_REF * (1 / 32,768)
# (MSB)0000 0000 0000 0000 = -V_REF * (32,767 / 32,768)
################################################################################
# LT1990-10 (Analog Devices) info:
# I = 10 * V_DAC / R_SENSE
# V_DAC = I * R_SENSE / 10 
# R_SESNSE = 100 Ohm
################################################################################
import math
from numpy import clip

V_REF = 3.3  # Reference voltage for DAC
R_SENSE = 100 # Sense resistor value in Ohms
MAX_CURRENT = math.floor(3.3 * 10 / R_SENSE * 1000000)  # Maximum current in uA
# Input value is in uA
def decimal_to_binary(decimal_value, bits=16):
    V_DAC = (decimal_value * R_SENSE / 10.0)/1000000  # Convert uA to V
    DAC = V_DAC / V_REF   # Normalize to V_REF
    #print(f"V_DAC: {V_DAC} V, DAC: {DAC}")
    DAC = clip(DAC, -1,1)  # Clamp to -1 to 1
    code = int((DAC + 1) * 32768)  # Convert to 16-bit code
    code = clip(code, 0, 65535)  # Ensure code is within 16-bit range
    #print(f"Code: {code} (Decimal), Hex: {hex(code)}")
    return code