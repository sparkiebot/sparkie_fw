from machine import Pin, PWM
import time
import math 
A0 = 20
A1 = 21
ENC_A0 = 14
ENC_A1 = 15

B0 = 18
B1 = 19
ENC_B0 = 17
ENC_B1 = 16

MOTOR_MAX_RPM = 185

UINT_16 = 65535

PWM_PER_RPM = UINT_16 / MOTOR_MAX_RPM

REDUCTION_RATE = 56.0
PULSES_PER_RPM = 11.0

def sign(n):
    if n < 0:
        return -1
    else:
        return 1

def clamp(n, min, max): 
    if n < min: 
        return min
    elif n > max: 
        return max
    else: 
        return n 

class Motor():
    
    a_pwm: PWM
    b_pwm: PWM
    enc_a: Pin
    enc_b: Pin
    
    pulses: int
    speed: float
    direction: int
    
    def __init__(self, a, b, enc_a, enc_b, max_rpm = MOTOR_MAX_RPM):
        self.pulses = 0
        self.speed = 0
        self.direction = 0
        
        self.a_pwm = PWM(Pin(a), freq = 500)
        self.b_pwm = PWM(Pin(b), freq = 500)
        
        self.enc_a = (Pin(enc_a, Pin.IN))
        self.enc_a.irq(trigger=Pin.IRQ_RISING, handler=self.encoder_int_a)
        self.enc_b = (Pin(enc_b, Pin.IN))
        
    
    def encoder_int_a(self, pin):
        if(self.enc_b.value() == 1):
            self.pulses += 1
        else:
            self.pulses -= 1
        
    def set_dir(self, direction):
        self.direction = direction
        
    def set_raw_speed(self, speed):
        if self.direction > 0:
            self.a_pwm.duty_u16(int(speed))
            self.b_pwm.duty_u16(0)
        else:
            self.a_pwm.duty_u16(0)
            self.b_pwm.duty_u16(int(speed))

    def update(self, dt):
        rev = self.pulses / (REDUCTION_RATE * PULSES_PER_RPM)
        self.speed = 60 * rev / (dt)
        self.pulses = 0

class PID():
    
    motor: Motor
    p: float
    i: float
    d: float
    
    kp: float
    ki: float
    kd: float
    
    e: float
    last_e: float
    
    goal: float
    
    deadband_offset: int
    
    def __init__(self, motor, deadband_offset, kp, ki ,kd):
        self.motor = motor
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.p = 0
        self.i = 0
        self.d = 0
        
        self.e = 0
        self.last_e = 0
        
        self.goal = 0
        self.deadband_offset = deadband_offset
        
    def set_speed(self, speed):
        self.goal = abs(speed)
        self.motor.set_dir(speed)
        
    def compute(self, dt):
        self.e = (self.goal - abs(self.motor.speed))
        
        self.p = self.kp * self.e
        
        self.i += self.ki * self.e
            
        self.d = self.kd * ((self.e - self.last_e))
        
        val = self.p + self.i + self.d
        
        val += self.deadband_offset
       
        return clamp(val, 0, UINT_16)


left = Motor(A0, A1, ENC_A0, ENC_A1)
l_pid = PID(left, 12000,  50, 1.5, -10)
right = Motor(B0, B1, ENC_B0, ENC_B1)
rpm = 0
right.set_raw_speed(rpm)
left.set_raw_speed(rpm)

l_pid.set_speed(abs(rpm))


rpm = int(input("RPM: "))
l_pid.set_speed(abs(rpm))
#right.set_raw_speed(rpm)
maxs = 0
a = 0.9
speed = 0


try:
    while True:
        dt = 33 / 1000
        right.update(dt)
        left.update(dt)
        computed_speed = l_pid.compute(dt)
        left.set_raw_speed(computed_speed)
        
        time.sleep_ms(33)

        
        #speed = speed * a + (1-a) * left.speed
        speed = left.speed
        
        if maxs < speed:
            maxs = speed
        
        print(f"left: {speed} goal: {abs(rpm)} pid: {computed_speed / PWM_PER_RPM} e: {l_pid.e}")
except:
    left.set_raw_speed(0)
    print(maxs)
