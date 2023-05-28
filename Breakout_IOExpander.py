import time
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

    IOE_PIN_IN = 0x00
    IOE_PIN_IN_PU = 0x10
    IOE_PIN_OUT = 0x02
    IOE_PIN_OD = 0x03
    IOE_PIN_PWM = 0x04
    IOE_PIN_ADC = 0x0A

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
        
        if register == self.IOE_REG_ADC_CTRL:
            return [(data[0] >> i) & 0x01 for i in range(1)]
        else:
            print("result is: ", data)
            return list(data[0:1])

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
            buffer = bytearray([register]) + bytearray(data)
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

    def set_mode(self, pin, mode):
        port = pin // 8
        pin_bit = pin % 8
        gpio_mode = mode & 0x03
        pullup_mode = (mode >> 4) & 0b1
        adc_mode = (mode >> 1) & 0b1
        pwm_mode = (mode >> 2) & 0b1
        print("pwm_mode =", pwm_mode)
        print("adc_mode =", adc_mode)
        print("pullup_mode =", pullup_mode)
        print("gpio_mode =", gpio_mode)
        print("current pin is {}, mode is {}".format(pin, mode))
        # Set the GPIO mode
        gpio_dir = self._read_register(IOE_REG_GPIO_DIR + port)[0]
        if mode == self.IOE_PIN_OUT:
            gpio_dir |= (1 << pin_bit)
        else:
            gpio_dir &= ~(1 << pin_bit)
        self._write_register(IOE_REG_GPIO_DIR + port, [gpio_dir])

        # Set the pull-up mode
        pullup_en = self._read_register(self.IOE_REG_PULL_UP_ENABLE + port)[0]
        if mode == self.IOE_PIN_IN_PU:
            pullup_en |= (1 << pin_bit)
        else:
            pullup_en &= ~(1 << pin_bit)
        self._write_register(self.IOE_REG_PULL_UP_ENABLE + port, [pullup_en])

        # Set the ADC mode
        adc_ctrl = self._read_register(self.IOE_REG_ADC_CTRL)[0]
        if mode == self.IOE_PIN_ADC:
            adc_ctrl |= (1 << pin_bit)
        else:
            adc_ctrl &= ~(1 << pin_bit)
        self._write_register(self.IOE_REG_ADC_CTRL, [adc_ctrl])


        # Set the PWM mode
        pwm_cfg = self._read_register(self.IOE_REG_PWM_CFG)[0]
        if mode == self.IOE_PIN_PWM:
            pwm_cfg |= (1 << pin_bit)
        else:
            pwm_cfg &= ~(1 << pin_bit)
        self._write_register(self.IOE_REG_PWM_CFG, [pwm_cfg])
#		  port = pin // 8
#         pin_bit = pin % 8
#         gpio_mode = mode & 0x03
#         pullup_mode = (mode >> 4) & 0b1
#         adc_mode = (mode >> 1) & 0b1
#         pwm_mode = (mode >> 2) & 0b1
#         print("pwm_mode =", pwm_mode)
#         print("adc_mode =", adc_mode)
#         print("pullup_mode =", pullup_mode)
#         print("gpio_mode =", gpio_mode)
#         print("current pin is {}, mode is {}".format(pin, mode))
#         # Set the GPIO mode
#         gpio_dir = self._read_register(IOE_REG_GPIO_DIR + port)[0]
#         if gpio_mode == self.IOE_PIN_OUT:
#             gpio_dir |= (1 << pin_bit)
#         else:
#             gpio_dir &= ~(1 << pin_bit)
#         self._write_register(IOE_REG_GPIO_DIR + port, [gpio_dir])
# 
#         # Set the pull-up mode
#         pullup_en = self._read_register(self.IOE_REG_PULL_UP_ENABLE + port)[0]
#         if pullup_mode:
#             pullup_en |= (1 << pin_bit)
#         else:
#             pullup_en &= ~(1 << pin_bit)
#         self._write_register(self.IOE_REG_PULL_UP_ENABLE + port, [pullup_en])
# 
#         # Set the ADC mode
#         adc_ctrl = self._read_register(self.IOE_REG_ADC_CTRL)[0]
#         if adc_mode:
#             adc_ctrl |= (1 << pin_bit)
#         else:
#             adc_ctrl &= ~(1 << pin_bit)
#         self._write_register(self.IOE_REG_ADC_CTRL, [adc_ctrl])
# 
# 
#         # Set the PWM mode
#         pwm_cfg = self._read_register(self.IOE_REG_PWM_CFG)[0]
#         if pwm_mode:
#             pwm_cfg |= (1 << pin_bit)
#         else:
#             pwm_cfg &= ~(1 << pin_bit)
#         self._write_register(self.IOE_REG_PWM_CFG, [pwm_cfg])

        

        
    def get_pin_mode(self, pin):
        pin_reg = IOE_REG_GPIO_DIR + (pin // 8)
        pin_bit = 1 << (pin % 8)
        config = self._read_register(pin_reg)
        if config is None or len(config) == 0:
            return None
        config = config[0]
        
        if (config & pin_bit) != 0:
            return self.IOE_PIN_OUT
        elif (config & self.IOE_PIN_PWM) != 0:
            return self.IOE_PIN_PWM
        elif (config & self.IOE_PIN_OD) != 0:
            return self.IOE_PIN_OD
        elif (config & self.IOE_PIN_IN_PU) != 0:
            return self.IOE_PIN_IN_PU
        elif (config & self.IOE_PIN_IN) != 0:
            return self.IOE_PIN_IN
        else:
            return None
        
    def set_adc_mode(self, pin):
        if pin < 7 or pin > 14:
            raise ValueError("Invalid ADC pin")

        # Calculate the ADC control byte
        control = self.IOE_REG_ADC_CTRL | (1 << (pin - 7))

        # Write the control byte to set the ADC mode
        self._write_register(self.IOE_REG_ADC_CTRL, [control])

    def set_gpio_mode(self, port, pin, mode):
        if mode < self.IOE_PIN_IN or mode > self.IOE_PIN_ADC and mode != self.IOE_PIN_IN_PU:
            raise ValueError("Invalid pin mode, mode is: ", mode)
        pin_reg = IOE_REG_GPIO_DIR + port
        pin_bit = 1 << pin
        config = self._read_register(pin_reg)
        if config is None or len(config) == 0:
            return
        config = config[0]
        if mode == self.IOE_PIN_IN:
            config &= ~pin_bit
        elif mode == self.IOE_PIN_IN_PU:
            config &= ~pin_bit
            config |= pin_bit  # Enable pull-up
            self._write_register(self.IOE_REG_PULL_UP_ENABLE, pin_bit)
        elif mode == self.IOE_PIN_OUT:
            config |= pin_bit
            self._write_register(self.IOE_REG_OUTPUT_STATE, pin_bit)
        elif mode == self.IOE_PIN_OD:
            config |= pin_bit
            config |= pin_bit
        elif mode == self.IOE_PIN_PWM:
            config |= pin_bit
            self._write_register(self.IOE_REG_PWM_CFG, pin_bit)
        else:
            raise ValueError("ADC mode not supported for this function")
        data = config.to_bytes(1, 'big')
        self._write_register(pin_reg, data)

    def _get_pin_value(self, pin):
        if pin < 1 or pin > IOE_NUM_PINS:
            raise ValueError("Invalid pin")

        io_pin = self.get_pin_mode(pin)
        print("getting pin value for pin {}: {}".format(pin, io_pin))
        
        if io_pin == self.IOE_PIN_ADC:
            if io_pin.adc_channel > 8:
                self._write_register(self.IOE_REG_ADC_CTRL, 1 << (io_pin.adc_channel - 8))
            else:
                self._write_register(self.IOE_REG_ADC_CTRL, 1 << io_pin.adc_channel)

            con0value = self._read_register(self.IOE_REG_ADC_CTRL)[0]
            con0value = con0value & ~0x0F
            con0value = con0value | io_pin.adc_channel

            con0value = con0value & ~(1 << 7)  # ADCF - Clear the conversion complete flag
            con0value = con0value | (1 << 6)  # ADCS - Set the ADC conversion start flag
            self._write_register(self.IOE_REG_ADC_CTRL, con0value)

            adc_timeout = 1.0  # Default timeout of 1.0 second
            t_start = time.monotonic()
            while not self._read_register(self.IOE_REG_ADC_CTRL)[0] & (1 << 7):
                if time.monotonic() - t_start >= adc_timeout:
                    raise RuntimeError("Timeout waiting for ADC conversion!")
                time.sleep(0.001)

            reading = self._read_register16(self.IOE_REG_ADCRL, self.IOE_REG_ADCRH)
            return (reading / 4095.0) * self._vref
        elif io_pin == self.IOE_PIN_PWM:
            return None
        else:
            #if self._debug:
            #    print("Reading IO from pin {}".format(pin))
            pv = self._read_register(IOE_REG_GPIO)[0] & (1 << pin)

            return bool(pv)

    def _set_pin_value(self, port, pin, value):
        if port < 0 or port > 1 or pin < 0 or pin > 7:
            raise ValueError("Invalid pin")

        pin_reg = self.IOE_REG_GPIO + port
        pin_bit = 1 << pin
        current_value = self._read_register(pin_reg)[0]

        if value:
            new_value = current_value | pin_bit
        else:
            new_value = current_value & ~pin_bit

        self._write_register(pin_reg, [new_value])


    def set_pin(self, pin, value):
        self._set_pin_value(pin, value)

    def get_pin(self, pin):
        return self._get_pin_value(pin)