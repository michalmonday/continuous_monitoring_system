''' Usage example:
from pynq import Overlay
from pynq import allocate
from continuous_monitoring_system_controller import ContinuousMonitoringSystemController
BASE_DIR = Path('/home/xilinx/design_files')
PATH = BASE_DIR 
base = Overlay(str(BASE_DIR / 'imported_design.bit'))
cms_ctrl = ContinuousMonitoringSystemController(base.axi_gpio_cms_ctrl)

# Triggerring (exact address must match to start/stop trace)
cms_ctrl.set_trigger_trace_start_address(0x80000000)
cms_ctrl.set_trigger_trace_end_address(0x800000C)
cms_ctrl.set_trigger_trace_start_address_enabled(True)
cms_ctrl.set_trigger_trace_end_address_enabled(True)

# Filtering (in example below any address between 0x80000000 and 0x8FFFFFFF will be collected)
cms_ctrl.set_monitored_address_range_lower_bound(0x80000000)
cms_ctrl.set_monitored_address_range_upper_bound(0x8FFFFFFF)
cms_ctrl.set_monitored_address_range_lower_bound_enabled(True)
cms_ctrl.set_monitored_address_range_upper_bound_enabled(True)
'''


class ContinuousMonitoringSystemController:
    # Addresses match "continuous_monitoring_system.v" file. 
    ADDR_TRIGGER_TRACE_START_ADDRESS_ENABLED = 0
    ADDR_TRIGGER_TRACE_END_ADDRESS_ENABLED = 1
    ADDR_TRIGGER_TRACE_START_ADDRESS = 2
    ADDR_TRIGGER_TRACE_END_ADDRESS = 3
    ADDR_MONITORED_ADDRESS_RANGE_LOWER_BOUND_ENABLED = 4
    ADDR_MONITORED_ADDRESS_RANGE_UPPER_BOUND_ENABLED = 5
    ADDR_MONITORED_ADDRESS_RANGE_LOWER_BOUND = 6
    ADDR_MONITORED_ADDRESS_RANGE_UPPER_BOUND = 7
    ADDR_WFI_REACHED = 8

    def __init__(self, axi_gpio):
        self.axi_gpio = axi_gpio

    def send_data_to_cms(self, data, address):
        ''' Single AXI GPIO block is used to interact with CMS module "ctrl" inputs having 73 bits.
            For that reason a shift register is used. It takes 16 bit data input, 1 shift signal (posedge activated), 
            and outputs 64 bits to "ctrl_wdata" CMS module. GPIO maps to:
                0-15  : shift register input bits (16 bits)
                16    : shift register shift signal
                14-24 : "ctrl_addr" of CMS module (8 bits)
                25    : "ctrl_write_enable" of CMS module
        '''
        # write enable = low
        self.axi_gpio[25].write(0)
        # write address
        self.axi_gpio[14:25].write(address)
        # send data through shift register 
        # (send 4 * 16 bits, resulting in 64bit ctrl_wdata input of CMS module)
        for i in range(4):
            # shift bits signal = low
            self.axi_gpio[16].write(0)
            # write LSB first
            to_send = (data >> (i * 16)) & 0xFFFF
            self.axi_gpio[0:16].write(to_send)
            # shift signal = high (posedge activated)
            self.axi_gpio[16].write(1)
    
    ###############################################
    # Trigger control functions (start/stop trace when certain program counter value is executed)
    def set_trigger_trace_start_address_enabled(self, enable=True):
        self.send_data_to_cms(enable, __class__.ADDR_TRIGGER_TRACE_START_ADDRESS_ENABLED)

    def set_trigger_trace_end_address_enabled(self, enable=True):
        self.send_data_to_cms(enable, __class__.ADDR_TRIGGER_TRACE_END_ADDRESS_ENABLED)

    def set_trigger_trace_start_address(self, value):
        self.send_data_to_cms(value, __class__.ADDR_TRIGGER_TRACE_START_ADDRESS)

    def set_trigger_trace_end_address(self, value):
        self.send_data_to_cms(value, __class__.ADDR_TRIGGER_TRACE_END_ADDRESS)

    ###############################################
    # Monitored address range functions (collect data only when program counter is in certain range)
    def set_monitored_address_range_lower_bound_enabled(self, enable=True):
        self.send_data_to_cms(enable, __class__.ADDR_MONITORED_ADDRESS_RANGE_LOWER_BOUND_ENABLED)

    def set_monitored_address_range_upper_bound_enabled(self, enable=True):
        self.send_data_to_cms(enable, __class__.ADDR_MONITORED_ADDRESS_RANGE_UPPER_BOUND_ENABLED)

    def set_monitored_address_range_lower_bound(self, value):
        self.send_data_to_cms(value, __class__.ADDR_MONITORED_ADDRESS_RANGE_LOWER_BOUND)

    def set_monitored_address_range_upper_bound(self, value):
        self.send_data_to_cms(value, __class__.ADDR_MONITORED_ADDRESS_RANGE_UPPER_BOUND)

    ###############################################
    # If a program has "wait for interrupt" (wfi) instruction at the end, 
    # the CMS will stop the trace. If we decide to run/trace another program
    # we may want to use this function, otherwise trace will not start.
    # (btw another way to reset the wfi wait is to load the Overlay 'bit/hwh' again)
    def reset_wfi_wait(self):
        self.send_data_to_cms(0, __class__.ADDR_WFI_REACHED)