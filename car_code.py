import board
import digitalio
import time
import pwmio

s1 = digitalio.DigitalInOut(board.D13)
s1.direction = digitalio.Direction.INPUT
s1.pull = digitalio.Pull.UP

s2 = digitalio.DigitialInOut(board.D12)
s2.direction = digitalio.Direction.INPUT
s2.pull = digitalio.Pull.UP

clkwse = pwmio.PWMOut(board.D10, frequency = 500, duty_cycle = 0)
counterclkwse = pwmio.PWMOut(board.D9, frequency = 500, duty_cycle = 0)

while True:
    
    if not s1.value and s2.value:
        clkwse.duty_cycle = 35000
        counterclkwse.duty_cycle = 0
        time.sleep(0.05)
        
    elif not s2.value and s1.value:
        clkwse.duty_cycle = 0
        counterclkwse.duty_cycle = 35000
        time.sleep(0.05)
        
    else:
        clkwse.duty_cycle = 0
        counterclkwse.duty_cycle = 0
        time.sleep(0.05)
