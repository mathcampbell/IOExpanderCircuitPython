import time
import board
import busio
from adafruit_bus_device.i2c_device import I2CDevice

IOE_DEFAULT_ADDRESS = 0x18
IOE_REG_CHIP_ID = 0x00
IOE_REG_ADC_VREF = 0x08
IOE_REG_INTERRUPT_CONFIG = 0x0C
IOE_REG_INTERRUPT_FLAG = 0x0E
IOE_REG_GPIO = 0x10
IOE_REG_GPIO_DIR = 0x14
IOE_REG_GPIO_INT_EN = 0x16
IOE_REG_PWM0 = 0x18
IOE_REG_PWM1 = 0x1A
IOE_REG_PWM_CONTROL = 0x1C
IOE_REG_PWM_PERIOD = 0x1E
IOE_NUM_PINS = 14
IOE_LOW = 0x00
IOE_HIGH = 0x01

class BreakoutIOExpander:
    IOE_REG_ADC_CTRL = 0x38
    IOE_REG_PULL_UP_ENABLE = 0x46
    IOE_REG_OUTPUT_STATE = 0x50
    IOE_REG_PWM_CFG = 0x70

    PIN_MODE_ADC = 0b01010
    PIN_MODE_PWM = 0b00101
    PIN_MODE_OD = 0b00011
    PIN_MODE_PU = 0b10000
    PIN_MODE_IN = 0b00010
    PIN_MODE_PP = 0b00001
    PIN_MODE_QB = 0b00000
    PIN_MODE_IO = 0b00000
    
    TYPE_IO = 0b00
    TYPE_PWM = 0b01
    TYPE_ADC = 0b10
    TYPE_ADC_OR_PWM = 0b11
    
    
    PxM1 = [0x71, 0x73, -1, 0x6c]
    PxM2 = [0x72, 0x74, -1, 0x6d]
    Px = [0x40, 0x50, -1, 0x70]
    PxS = [0xc2, 0xc4, 0x75, 0xc0]
    
    
    MASK_P = [0x00, 0x01, -1, 0x03]
    PWML = [0x9a, 0x9b, 0x9c, 0x9d, 0xca, 0xcb]
    PWMH = [0x92, 0x93, 0x94, 0x95, 0xc7, 0xc8]
   # ADCRL = [0x40, 0x42, 0x44, 0x46, 0x48, 0x4A, 0x4C, 0x4E]
   # ADCRH = [0x41, 0x43, 0x45, 0x47, 0x49, 0x4B, 0x4D, 0x4F]
    ADCRH = 0x83
    ADCRL = 0x82
    ADCMPL = 0x8e
    ADCMPH = 0x8f
    REG_P0 = 0x40
    REG_P1 = 0x50
    REG_P2 = 0x60
    REG_P3 = 0x70
    
    BIT_ADDRESSED_REGS = [REG_P0, REG_P1, REG_P2, REG_P3]
    

    
    REG_MASK_P0 = 0x00
    REG_MASK_P1 = 0x01
    REG_MASK_P3 = 0x03
    
    MODE_IO = PIN_MODE_IO
    MODE_OUTPUT = PIN_MODE_PP
    MODE_PULLUP = PIN_MODE_PU
    MODE_PWM = PIN_MODE_PWM
    MODE_ADC = PIN_MODE_ADC
    MODE_INPUT = PIN_MODE_IN

    ADCCON0 = 0xa8
    ADCCON1 = 0xa1
    ADCCON2 = 0xa2
    
    AINDIDS = 0xb6
    
    PIOCON0 = 0x9e
    PIOCON1 = 0xc9
    
    adc_enabled = False
    
    vref = 4.7
    
    adc_channel = {
        7:(7),
        8:(6),
        9:(5),
        10:(1),
        11:(3),
        12:(4),
        13:(2),
        14:(0)
    }

    pin_map = {
            1: (1, 5),
            2: (1, 0),
            3: (1, 2),
            4: (1, 4),
            5: (0, 0),
            6: (0, 1),
            7: (1, 1),
            8: (0, 3),
            9: (0, 4),
            10: (3, 0),
            11: (0, 6),
            12: (0, 5),
            13: (0, 7),
            14: (1, 7),
    } 
    

    def __init__(self, i2c, address=IOE_DEFAULT_ADDRESS):
        self.i2c_device = I2CDevice(i2c, address)
        self._adc_vref = IOE_REG_ADC_VREF
        self._interrupt_enabled = False
        self._interrupt_flag = False
        self._pin_interrupts = [False] * IOE_NUM_PINS

    def _read_register(self, register):
        with self.i2c_device as i2c:
            data = bytearray(1)
            data[0] = register
            i2c.write_then_readinto(data, data, out_end=1, in_end=1)
        
            return data[0]

    def _read_register12(self, reg_l, reg_h):
        #Read two (4+8bit) registers from the device, as a single read if they are consecutive.
        if reg_h == reg_l + 1:
            with self.i2c_device as i2c:
                msg_w = bytearray([reg_l])
                msg_r = bytearray(2)
                i2c.write_then_readinto(msg_w, msg_r)

            return (msg_r[0] & 0x0F) | (msg_r[1] << 4)
        else:
            return (self._read_register(reg_h, 1)[0] << 4) | self._read_register(reg_l, 1)[0]

    def _read_register16(self, reg_l, reg_h):
        #Read two (8+8bit) registers from the device, as a single read if they are consecutive.
        if reg_h == reg_l + 1:
            with self.i2c_device as i2c:
                msg_w = bytearray([reg_l])
                msg_r = bytearray(2)
                i2c.write_then_readinto(msg_w, msg_r)

            return (msg_r[0]) | (msg_r[1] << 8)
        else:
            return self._read_register(reg_l, 1)[0] | (self._read_register(reg_h, 1)[0] << 8)

    def _write_register(self, register, data):
        with self.i2c_device as i2c:

            buffer = bytearray([register, data])  # Create a bytearray with register and data

            i2c.write(buffer)
        
    
    def _write_register16(self, reg_l, reg_h, data):
    #"Write two (8+8bit) registers to the device, as a single write if they are consecutive."
        val_l = data & 0xFF
        val_h = (data >> 8) & 0xFF
        if reg_h == reg_l + 1:
            buffer = bytearray([reg_l, val_l, val_h])
            self._write_register(buffer)
        else:
            self._write_register(reg_l, val_l)
            self._write_register(reg_h, val_h)
            
    def get_chip_id(self):
        return self._read_register(IOE_REG_CHIP_ID, 1)

    def set_address(self, address):
        self.address = address

    def get_adc_vref(self):
        vref = self._read_register(IOE_REG_ADC_VREF, 1)
        return int(vref[0])

    def set_adc_vref(self, value):
        self._write_register(IOE_REG_ADC_VREF, [value])

    def enable_interrupt_out(self, enable=True):
        if enable:
            self._write_register(IOE_REG_INTERRUPT_CONFIG, [0x80])
        else:
            self._write_register(IOE_REG_INTERRUPT_CONFIG, [0x00])

    def disable_interrupt_out(self):
        self.enable_interrupt_out(False)

    def get_interrupt_flag(self):
        return self._read_register(IOE_REG_INTERRUPT_FLAG, 1)

    def clear_interrupt_flag(self):
        self._write_register(IOE_REG_INTERRUPT_FLAG, [0xFF])

    def set_pin_interrupt(self, pin, enable=True):
        int_en = self._read_register(IOE_REG_GPIO_INT_EN, 1)[0]
        if enable:
            int_en |= (1 << pin)
        else:
            int_en &= ~(1 << pin)
        self._write_register(IOE_REG_GPIO_INT_EN, [int_en])

    def pwm_load(self, channel):
        self._write_register(IOE_REG_PWM_CONTROL, [(channel << 4) | 0x01])
        while self._read_register(IOE_REG_PWM_CONTROL, 1)[0] & 0x01:
            pass

    def pwm_loading(self):
        return self._read_register(IOE_REG_PWM_CONTROL, 1)[0] & 0x01

    def pwm_clear(self, channel):
        self._write_register(IOE_REG_PWM_CONTROL, [(channel << 4) | 0x02])
        while self._read_register(IOE_REG_PWM_CONTROL, 1)[0] & 0x02:
            pass

    def pwm_clearing(self):
        return self._read_register(IOE_REG_PWM_CONTROL, 1)[0] & 0x02

    def set_pwm_control(self, channel, enable=True, invert=False):
        pwm_control = (channel << 4) | (1 << 3) if invert else (channel << 4)
        if enable:
            pwm_control |= (1 << 2)
        self._write_register(IOE_REG_PWM_CONTROL, [pwm_control])

    def set_pwm_period(self, channel, period):
        self._write_register(IOE_REG_PWM_PERIOD + (channel * 2), [period >> 8, period & 0xFF])

    def disable_adc(self, pin):

        channel = self.adc_channel[pin]
        self.clear_bit(self.AINDIDS, channel)

    
    def enable_adc(self, pin):

        channel = self.adc_channel[pin]
        
        self.set_bit(self.AINDIDS, channel)
        
        
        
    def set_mode(self, pin, mode):
        print("Setting mode for pin {}: {}".format(pin, mode))
        if pin < 1 or pin > 14:
                raise ValueError("Invalid pin")

        if mode not in [self.MODE_INPUT, self.MODE_OUTPUT, self.MODE_PULLUP, self.MODE_IO, self.MODE_PWM, self.MODE_ADC]:
                raise ValueError("Invalid mode")

        if mode == self.MODE_PWM and pin > 6:
                raise ValueError("Only pins 1-6 can be set as PWM")

        if mode == self.MODE_ADC and pin < 7:
                raise ValueError("Only pins 7-14 can be set as ADC")
        
        # Get the bit and port
        port, bit = self.pin_map[pin]
        
        #print("Port: ", port)
        #print("Bit: ", bit)
        
        
        schmitt_state = self._read_register(self.PxS[port])

        gpio_mode = mode & 0b11;
 
        io_type = (mode >> 2) & 0b11
        initialState = mode >> 4

       
        pm1 = self._read_register(self.PxM1[port])
        pm2 = self._read_register(self.PxM2[port])
    
        
        pm1 &= ~(1 << bit)
        pm2 &= ~(1 << bit)
       

        pm1 |= (gpio_mode >> 1) << bit
        pm2 |= (gpio_mode & 1) << bit
        
        
        self._write_register(self.PxM1[port], pm1)
        self._write_register(self.PxM2[port], pm2)
        
        newpm1 = self._read_register(self.PxM1[port])
        newpm2 = self._read_register(self.PxM2[port])
        
 
        
        if mode == self.MODE_INPUT or mode == self.MODE_PULLUP:
      
                self.change_bit(self.PxS[port], bit, 1)
       
        # Set the bit to initialState
       
        self._write_register(self.Px[port], (initialState <<3)|bit)

        # Deal with PWM registers
        if mode == self.MODE_PWM:
                self._write_register(self.PxM1[port], 0 << bit)
                self._write_register(self.PxM2[port], 1 << bit)
                self._write_register(self.PWML[bit - 1], 0xFF)
                self._write_register(self.PWMH[bit - 1], 0xFF)
        
    def _get_mode(self, pin):
        if pin < 1 or pin > 14:
            raise ValueError("Invalid pin")
        
        port, bit = self.pin_map[pin]
        
        pxm1 = self._read_register(self.PxM1[port])
        pxm2 = self._read_register(self.PxM2[port])
        px = self._read_register(self.Px[port])
        pxs = self._read_register(self.PxS[port])
        
        gpio_mode = ((pxm1 >> bit) & 0b1) << 1 | (pxm2 >> bit) & 0b1
        io_type = (pxs >> bit) & 0b11
        initial_state = (px >> bit) & 0b1
        print("the returned gpio_mode is {}, io_type is {}, initial state is {}".format(gpio_mode, io_type, initial_state))
        mode = gpio_mode | (io_type << 2) | (initial_state << 4)
        print("returned mode is {}".format(mode))
        
        return mode

        

    def _get_pin_value(self, pin, mode):
      #  print("Calling get_pin_value, pin: {}, mode: {}".format(pin, mode))
      #  print("Calling get_mode")
       # print("Mode returns as {}".format(self._get_mode(pin)))
        
        
        # Check if the pin number is valid
        if pin not in self.pin_map:
            raise ValueError("Invalid pin number")

        # Get the logical pin and port
        port, bit = self.pin_map[pin]

        # Check if the mode is valid
        if mode not in [self.MODE_IO, self.MODE_OUTPUT, self.MODE_PULLUP, self.MODE_INPUT, self.MODE_PWM, self.MODE_ADC]:
            raise ValueError("Invalid mode")

        # Form the register commands
        if mode in [self.MODE_INPUT, self.MODE_OUTPUT, self.MODE_PULLUP, self.MODE_IO]:
#             
            register = self.Px[port]
                
#             value = self._read_register(register)
#            
#             pinvalue = bool(value & (1 << bit))  # directly calculate the pin value
            
            pinvalue = bool(self.get_bit(register, bit))
            
           
            #if not pinvalue:
                #print ("pin {} is {}".format(pin, pinvalue))
            return pinvalue
                  
        elif mode & self.MODE_PWM:
            register = self.PWMH[bit - 1]
            value = self._read_register(register)
            return value
        
        elif mode & self.MODE_ADC:
                current_adc_channel = self.adc_channel[pin]
                
                # Enable the ADC circuit
                self.clear_bits(self.ADCCON0, 0x0f)
                self.set_bit(self.ADCCON1, 0)
                self.set_bit(self.AINDIDS, 0) 
                self.enable_adc(pin)
                 
                con0value = self._read_register(self.ADCCON0)
                con0value = con0value & ~0x0f
                con0value = con0value | current_adc_channel
                con0value = con0value & ~(1 << 7)
                con0value = con0value | (1 << 6)
                self._write_register(self.ADCCON0, con0value)
                
                
                
                starttime = time.time()
                
                while not (self.get_bit(self.ADCCON0, 7)):
                    time.sleep(0.001)
                    if time.time() - starttime >= 1:
                        return -1
                
                reading = self._read_register12(self.ADCRL, self.ADCRH)
                comparator = self._read_register12(self.ADCMPL, self.ADCMPH)

                self._write_register(self.ADCCON0, 0)
                self.disable_adc(pin)
                
                # Disable the ADC circuit
                self.clear_bit(self.ADCCON1, 0)
                
                #print("Reading is: {}".format(reading))
                
               # return (combined_reading / 65535) * self.vref
                #return (reading / 4095) * self.vref
                return reading


    def _set_pin_value(self, pin, value):
        if pin < 1 or pin > 14:
                raise ValueError("Invalid pin")
        # Get the logical pin and port
        port, bit = self.pin_map[pin]
        
        if value == False:
            self.clear_bit(self.Px[port], bit)
        else:
            self.set_bit(self.Px[port], bit)
       

    def set_pin(self, pin, value):
        self._set_pin_value(pin, value)

    def get_pin(self, pin, mode):
        return self._get_pin_value(pin, mode)
    
    def get_bit(self, register, bit):
       # return self._read_register(register) & (1 << bit);
        value = self._read_register(register)
        return bool(value & (1 << bit))
        
    def set_bit(self, register, bit):
        self.set_bits(register, (1 << bit))
        
    def set_bits(self, register, bits):
        if register == self.REG_P0 or register == self.REG_P1 or register == self.REG_P2 or register == self.REG_P3:
            for bit in range(8):
                if (bits & (1 << bit)):
                    #print("Setting special bits: reg:{},  bits {}".format(register, bits))
                    self._write_register(register, 0b0000 | (bit & 0b111))
                    
            return
        
        value = self._read_register(register)
        
       # print("Setting bits: reg:{}, value:{} | bits {}".format(register, value, bits))
        self._write_register(register, value | bits)
       # print("Bits set:  {}".format(self._read_register(register)))
        
    def clear_bits(self, register, bits):
        # Clear the specified bits (using a bit mask) in a register.
        if register == self.REG_P0 or register == self.REG_P1 or register == self.REG_P2 or register == self.REG_P3:
            for bit in range(8):
                if (bits & (1 << bit)):
                    self._write_register(register, 0b0000 | (bit & 0b111))
                    
            return
        
        value = self._read_register(register)
        
        self._write_register(register, value & ~bits)
        
    def clear_bit(self, register, bit):
        #clear the specified bit (nth position from the right) in a register
        self.clear_bits(register, bit)
        
    def change_bit(self, register, bit, state):
        # Toggle one register bit
        #print("reg:{}, bit:{}, state:{}".format(register, bit, state))
        if(state):
            self.set_bit(register, bit)
        else:
            self.clear_bit(register, bit)
    
                
            
        