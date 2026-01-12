from machine import Pin, SPI
import time
import sys

# Register addresses
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_PA_CONFIG = 0x09
REG_FIFO_TX_BASE = 0x0E
REG_FIFO_RX_BASE = 0x0F
REG_FIFO_ADDR_PTR = 0x0D
REG_FIFO = 0x00
REG_IRQ_FLAGS = 0x12
REG_RX_NB_BYTES = 0x13
REG_MODEM_CONFIG_1 = 0x1D
REG_MODEM_CONFIG_2 = 0x1E
REG_MODEM_CONFIG_3 = 0x26
REG_PAYLOAD_LENGTH = 0x22
REG_PKT_SNR_VALUE = 0x19
REG_PKT_RSSI_VALUE = 0x1A
REG_VERSION = 0x42
REG_SYNC_WORD = 0x39

# Modes
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

    def read_register(self, reg):
        self.cs.value(0)
        self.spi.write(bytes([reg & 0x7F]))
        data = self.spi.read(1)
        self.cs.value(1)
        return data[0]

    def write_register(self, reg, value):
        self.cs.value(0)
        self.spi.write(bytes([reg | 0x80, value]))
        self.cs.value(1)

    def init_lora(self):
        print("Initializing LoRa...")
        self.reset()
        time.sleep(0.1)

        # Sleep mode
        self.write_register(REG_OP_MODE, MODE_SLEEP)
        time.sleep(0.01)

        # Enable LoRa mode
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP)
        time.sleep(0.01)

        # Standby mode
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY)
        time.sleep(0.01)

        # Check version
        version = self.read_register(REG_VERSION)
        if version != 0x12:
            print(f"Wrong version: 0x{version:02X}")
            return False

        # Set frequency to 433MHz
        self.write_register(REG_FRF_MSB, 0x6C)
        self.write_register(REG_FRF_MID, 0x40)
        self.write_register(REG_FRF_LSB, 0x00)

        # Set TX power
        self.write_register(REG_PA_CONFIG, 0x8F)

        # Set modem config
        self.write_register(REG_MODEM_CONFIG_1, 0x72)  # BW=125kHz, CR=4/5
        self.write_register(REG_MODEM_CONFIG_2, 0x74)  # SF=7, CRC ON
        self.write_register(REG_MODEM_CONFIG_3, 0x04)  # AGC ON

        # Set sync word (must match transmitter)
        self.write_register(REG_SYNC_WORD, 0xF3)

        # Set FIFO addresses
        self.write_register(REG_FIFO_TX_BASE, 0x00)
        self.write_register(REG_FIFO_RX_BASE, 0x00)

        print("✓ Initialization complete!")
        return True

    def receive(self):
        irq = self.read_register(REG_IRQ_FLAGS)

        # Check if RX done flag is set (bit 6)
        if irq & 0x40:
            # Check if payload CRC error (bit 5)
            if irq & 0x20:
                self.write_register(REG_IRQ_FLAGS, 0xFF)
                return None

            # Get received bytes count
            rx_bytes = self.read_register(REG_RX_NB_BYTES)

            # Get FIFO RX current address
            rx_addr = self.read_register(0x10)

            # Set FIFO pointer to RX address
            self.write_register(REG_FIFO_ADDR_PTR, rx_addr)

            # Read message from FIFO
            self.cs.value(0)
            self.spi.write(bytes([REG_FIFO & 0x7F]))
            message = self.spi.read(rx_bytes)
            self.cs.value(1)

            # Get signal quality
            snr = self.read_register(REG_PKT_SNR_VALUE)
            rssi = self.read_register(REG_PKT_RSSI_VALUE)

            # Convert SNR (signed)
            if snr & 0x80:
                snr = ((~snr + 1) & 0xFF) / -4.0
            else:
                snr = snr / 4.0

            # Calculate RSSI for 433MHz (LF port)
            rssi = -164 + rssi

            # Clear IRQ flags
            self.write_register(REG_IRQ_FLAGS, 0xFF)

            try:
                msg_text = message.decode('utf-8')
                return {
                    'message': msg_text,
                    'rssi': rssi,
                    'snr': snr
                }
            except:
                return None

        return None

    def send(self, message):
        # Convert string to bytes
        if isinstance(message, str):
            message = message.encode()

        # Set to standby
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY)
        time.sleep(0.01)

        # Set FIFO pointer to TX base
        self.write_register(REG_FIFO_ADDR_PTR, 0x00)

        # Write message to FIFO
        self.cs.value(0)
        self.spi.write(bytes([REG_FIFO | 0x80]))
        self.spi.write(message)
        self.cs.value(1)

        # Set payload length
        self.write_register(REG_PAYLOAD_LENGTH, len(message))

        # Clear IRQ flags
        self.write_register(REG_IRQ_FLAGS, 0xFF)

        # Start transmission
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX)

        # Wait for TX done (bit 3 of IRQ flags)
        start = time.ticks_ms()
        while not (self.read_register(REG_IRQ_FLAGS) & 0x08):
            if time.ticks_diff(time.ticks_ms(), start) > 1000:
                print("TX Timeout")
                return False

        # Clear IRQ flags
        self.write_register(REG_IRQ_FLAGS, 0xFF)

        # Return to RX mode
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS)

        return True

# Setup SPI
spi = SPI(0, baudrate=5000000, polarity=0, phase=0,
          sck=Pin(2), mosi=Pin(3), miso=Pin(4))
cs_pin = Pin(5, Pin.OUT)
rst_pin = Pin(6, Pin.OUT)

try:
    # Initialize LoRa
    lora = SX1278(spi, cs_pin, rst_pin)

    if lora.init_lora():
        print("\n✅ Transceiver ready!")
        print("📡 Listening for messages...")
        print("Type a message and press Enter to send")
        print("Press Ctrl+C to stop\n")

        # Start in RX mode
        lora.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS)

        msg_count = 0

        while True:
            # Check for received messages
            data = lora.receive()
            if data:
                msg_count += 1
                print("=" * 50)
                print(f"📨 Message #{msg_count} Received!")
                print("-" * 50)
                print(f"Content: {data['message']}")
                print(f"RSSI:    {data['rssi']} dBm")
                print(f"SNR:     {data['snr']:.1f} dB")
                print("=" * 50)
                print()

            # Check for user input
            if lora.send(f"hello #{msg_count}"):
                print("✓ Sent!")
            else:
                print("✗ Failed!")

            time.sleep(0.01)

    else:
        print("❌ Failed to initialize LoRa module!")

except KeyboardInterrupt:
    print("\n\n⏹ Transceiver stopped")
except Exception as e:
    print(f"\n❌ Error: {e}")
    import sys
    sys.print_exception(e)

print("=" * 50)