from machine import Pin, SPI
import time
import uselect
import sys
import json

# --- LoRa Registers ---
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_PA_CONFIG = 0x09
REG_FIFO_ADDR_PTR = 0x0D
REG_FIFO_TX_BASE = 0x0E
REG_FIFO_RX_BASE = 0x0F
REG_IRQ_FLAGS = 0x12
REG_RX_NB_BYTES = 0x13
REG_PKT_SNR_VALUE = 0x19
REG_PKT_RSSI_VALUE = 0x1A
REG_MODEM_CONFIG_1 = 0x1D
REG_MODEM_CONFIG_2 = 0x1E
REG_PAYLOAD_LENGTH = 0x22
REG_MODEM_CONFIG_3 = 0x26
REG_SYNC_WORD = 0x39  # <--- NEW: Register for Sync Word

# --- Modes ---
MODE_LONG_RANGE_MODE = 0x80
MODE_SLEEP = 0x00
MODE_STDBY = 0x01
MODE_TX = 0x03
MODE_RX_CONTINUOUS = 0x05

class SX1278:
    def __init__(self, spi, cs, rst):
        self.spi = spi
        self.cs = cs
        self.rst = rst
        self.cs.init(Pin.OUT)
        self.rst.init(Pin.OUT)
        self.cs.value(1)

    def reset(self):
        self.rst.value(0)
        time.sleep(0.01)
        self.rst.value(1)
        time.sleep(0.01)

    def write_register(self, reg, value):
        self.cs.value(0)
        self.spi.write(bytes([reg | 0x80, value]))
        self.cs.value(1)

    def read_register(self, reg):
        self.cs.value(0)
        self.spi.write(bytes([reg & 0x7F]))
        data = self.spi.read(1)
        self.cs.value(1)
        return data[0]

    def init_lora(self):
        print("Initializing LoRa...")
        self.reset()
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP)
        time.sleep(0.01)
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY)

        # 1. Set Frequency (433MHz)
        self.write_register(REG_FRF_MSB, 0x6C)
        self.write_register(REG_FRF_MID, 0x40)
        self.write_register(REG_FRF_LSB, 0x00)

        # 2. Set Power
        self.write_register(REG_PA_CONFIG, 0x8F)

        # 3. Set Modem Config (BW=125, SF=7, CR=4/5)
        self.write_register(REG_MODEM_CONFIG_1, 0x72)
        self.write_register(REG_MODEM_CONFIG_2, 0x74)
        self.write_register(REG_MODEM_CONFIG_3, 0x04)

        # 4. Set Sync Word (Network ID) - CRITICAL FIX
        # Use 0xF3 (or any value). Must match on Receiver!
        self.write_register(REG_SYNC_WORD, 0xF3)

        self.write_register(REG_FIFO_TX_BASE, 0x00)
        self.write_register(REG_FIFO_RX_BASE, 0x00)
        print("✓ LoRa Ready (Sync Word: 0xF3)")
        return True

    def send(self, message):
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY)
        self.write_register(REG_FIFO_ADDR_PTR, 0x00)

        if isinstance(message, str):
            message = message.encode()

        self.cs.value(0)
        self.spi.write(bytes([REG_FIFO | 0x80]))
        self.spi.write(message)
        self.cs.value(1)

        self.write_register(REG_PAYLOAD_LENGTH, len(message))
        self.write_register(REG_IRQ_FLAGS, 0xFF)
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX)

        start = time.ticks_ms()
        while not (self.read_register(REG_IRQ_FLAGS) & 0x08):
            if time.ticks_diff(time.ticks_ms(), start) > 1000:
                print("TX Timeout")
                return False

        self.write_register(REG_IRQ_FLAGS, 0xFF)
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS)
        return True

    def receive(self):
        irq = self.read_register(REG_IRQ_FLAGS)
        if irq & 0x40: # RX Done
            # Check for Payload CRC Error
            if irq & 0x20:
                self.write_register(REG_IRQ_FLAGS, 0xFF)
                return None # Ignore CRC errors silently

            length = self.read_register(REG_RX_NB_BYTES)
            current_addr = self.read_register(0x10) # RegFifoRxCurrentAddr
            self.write_register(REG_FIFO_ADDR_PTR, current_addr)

            self.cs.value(0)
            self.spi.write(bytes([REG_FIFO & 0x7F]))
            payload = self.spi.read(length)
            self.cs.value(1)

            self.write_register(REG_IRQ_FLAGS, 0xFF)

            # --- CRITICAL FILTERING ---
            try:
                # ONLY return if it is valid readable text
                return payload.decode('utf-8')
            except UnicodeError:
                # If it's garbage bytes like b'\x80...', ignore it!
                return None
        return None

# --- Setup ---
spi = SPI(0, baudrate=5000000, polarity=0, phase=0, sck=Pin(2), mosi=Pin(3), miso=Pin(4))
cs = Pin(5, Pin.OUT)
rst = Pin(6, Pin.OUT)
lora = SX1278(spi, cs, rst)

# Listen to USB Serial
poll_obj = uselect.poll()
poll_obj.register(sys.stdin, uselect.POLLIN)

if lora.init_lora():
    lora.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS)

    while True:
        msg = lora.receive()
        if msg:
            print(msg)

        if poll_obj.poll(0):
            line = sys.stdin.readline()
            if line:
                line = line.strip()
                if line:
                    print(f"TX: {line}")
                    lora.send(line)

        time.sleep(0.01)
