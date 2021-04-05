import machine
import time
"""
SMBUS in MICROPYTHON

THIS CODE IS TAKEN FROM https://makersportal.com/blog/2019/11/11/raspberry-pi-python-accelerometer-gyroscope-magnetometer
Written on November 15, 2019 by Joshua Hrisko

THIS CODE IS ONLY ADAPTED TO MICROPYTHON NO OTHER CHANGES WHERE MADE

PIN LAYOUT PICO:
sda: Pin(8)
scl: (9)

FROM SMBUS2 TO MICROPYTHON:
write_byte_data(i2c_addr, register, value, force=None)
Write a byte to a given register.
Parameters
• i2c_addr (int) – i2c address
• register (int) – Register to write to
• value (int) – Byte value to transmit
• force (Boolean) –

read_byte_data(i2c_addr, register, force=None)
Read a single byte from a designated register.
Parameters
• i2c_addr (int) – i2c address
• register (int) – Register to read
• force (Boolean) –
Returns Read byte value
Return type int

TO MIRCOPYTHON:
I2C.writeto_mem(addr, memaddr, buf, *, addrsize=8)
Write buf to the slave specified by addr starting from the memory address specified by memaddr.
The argument addrsize specifies the address size in bits

I2C.readfrom_mem(addr, memaddr, nbytes, *, addrsize=8)
Read nbytes from the slave specified by addr starting from the memory address specified by memaddr.
The argument addrsize specifies the address size in bits. Returns a bytes object with the data read.

"""

def i2c_write(addr, reg, value):
    # writeto_mem needs an bytearray
    # bytearray([value]) creates an one element byte
    bus.writeto_mem(addr, reg, bytearray([value]), addrsize=8)
    
def i2c_read(addr, reg):
    # convert byte to integer
    # as it is a one byte element big or little does not make a difference
    return int.from_bytes(bus.readfrom_mem(addr,reg, 1, addrsize=8), "little")#, signed=False)

def MPU6050_start():
    # alter sample rate (stability)
    samp_rate_div = 0 # sample rate = 8 kHz/(1+samp_rate_div)
    
    i2c_write(MPU6050_ADDR, SMPLRT_DIV, samp_rate_div)
    time.sleep(0.1)
    # reset all sensors
    i2c_write(MPU6050_ADDR,PWR_MGMT_1,0x00)
    time.sleep(0.1)
    # power management and crystal settings
    i2c_write(MPU6050_ADDR, PWR_MGMT_1, 0x01)
    time.sleep(0.1)
    #Write to Configuration register
    i2c_write(MPU6050_ADDR, CONFIG, 0)
    time.sleep(0.1)
    #Write to Gyro configuration register
    gyro_config_sel = [0b00000,0b010000,0b10000,0b11000] # byte registers
    gyro_config_vals = [250.0,500.0,1000.0,2000.0] # degrees/sec
    gyro_indx = 0
    i2c_write(MPU6050_ADDR, GYRO_CONFIG, int(gyro_config_sel[gyro_indx]))
    time.sleep(0.1)
    #Write to Accel configuration register
    accel_config_sel = [0b00000,0b01000,0b10000,0b11000] # byte registers
    accel_config_vals = [2.0,4.0,8.0,16.0] # g (g = 9.81 m/s^2)
    accel_indx = 0                            
    i2c_write(MPU6050_ADDR, ACCEL_CONFIG, int(accel_config_sel[accel_indx]))
    time.sleep(0.1)
    # interrupt register (related to overflow of data [FIFO])
    i2c_write(MPU6050_ADDR, INT_ENABLE, 1)
    time.sleep(0.1)
    return gyro_config_vals[gyro_indx],accel_config_vals[accel_indx]
    
def read_raw_bits(register):
    # read accel and gyro values
    high = i2c_read(MPU6050_ADDR, register)
    low = i2c_read(MPU6050_ADDR, register+1)

    # combine higha and low for unsigned bit value
    value = ((high << 8) | low)
    
    # convert to +- value
    if(value > 32768):
        value -= 65536
    return value

def mpu6050_conv():
    # raw acceleration bits
    acc_x = read_raw_bits(ACCEL_XOUT_H)
    acc_y = read_raw_bits(ACCEL_YOUT_H)
    acc_z = read_raw_bits(ACCEL_ZOUT_H)

    # raw temp bits
    t_val = read_raw_bits(TEMP_OUT_H) # uncomment to read temp
    
    # raw gyroscope bits
    gyro_x = read_raw_bits(GYRO_XOUT_H)
    gyro_y = read_raw_bits(GYRO_YOUT_H)
    gyro_z = read_raw_bits(GYRO_ZOUT_H)

    #convert to acceleration in g and gyro dps
    a_x = (acc_x/(2.0**15.0))*accel_sens
    a_y = (acc_y/(2.0**15.0))*accel_sens
    a_z = (acc_z/(2.0**15.0))*accel_sens

    w_x = (gyro_x/(2.0**15.0))*gyro_sens
    w_y = (gyro_y/(2.0**15.0))*gyro_sens
    w_z = (gyro_z/(2.0**15.0))*gyro_sens

    temp = ((t_val)/333.87)+21.0 # uncomment and add below in return
    return a_x,a_y,a_z,w_x,w_y,w_z, temp

def AK8963_start():
    i2c_write(AK8963_ADDR,AK8963_CNTL,0x00)
    time.sleep(0.1)
    AK8963_bit_res = 0b0001 # 0b0001 = 16-bit
    AK8963_samp_rate = 0b0110 # 0b0010 = 8 Hz, 0b0110 = 100 Hz
    AK8963_mode = (AK8963_bit_res <<4)+AK8963_samp_rate # bit conversion
    i2c_write(AK8963_ADDR,AK8963_CNTL,AK8963_mode)
    time.sleep(0.1)
    
def AK8963_reader(register):
    # read magnetometer values
    low = i2c_read(AK8963_ADDR, register-1)
    high = i2c_read(AK8963_ADDR, register)
    # combine higha and low for unsigned bit value
    value = ((high << 8) | low)
    # convert to +- value
    if(value > 32768):
        value -= 65536
    return value

def AK8963_conv():
    # raw magnetometer bits

    loop_count = 0
    while 1:
        mag_x = AK8963_reader(HXH)
        mag_y = AK8963_reader(HYH)
        mag_z = AK8963_reader(HZH)

        # the next line is needed for AK8963
        # print(bin(i2c_read(AK8963_ADDR,AK8963_ST2)))
        if bin(i2c_read(AK8963_ADDR,AK8963_ST2))=='0b10000': ###!!=======
            break
        loop_count+=1
        
    #convert to acceleration in g and gyro dps
    m_x = (mag_x/(2.0**15.0))*mag_sens
    m_y = (mag_y/(2.0**15.0))*mag_sens
    m_z = (mag_z/(2.0**15.0))*mag_sens

    return m_x,m_y,m_z
    
# MPU6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
TEMP_OUT_H   = 0x41
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
#AK8963 registers
AK8963_ADDR   = 0x0C
AK8963_ST1    = 0x02
HXH          = 0x04
HYH          = 0x06
HZH          = 0x08
AK8963_ST2   = 0x09
AK8963_CNTL  = 0x0A
mag_sens = 4900.0 # magnetometer sensitivity: 4800 uT

# start I2C driver
sda=machine.Pin(8)
scl=machine.Pin(9)
bus=machine.I2C(0,sda=sda, scl=scl, freq=400000) #sstart comm with i2c bus
gyro_sens,accel_sens = MPU6050_start() # instantiate gyro/accel
print("CONFIG", gyro_sens,accel_sens)
AK8963_start() # instantiate magnetometer
