# LoRa Transceiver with Raspberry Pi Pico

**A complete SX1278 LoRa wireless communication system for Raspberry Pi Pico using MicroPython. Long-range IoT connectivity for remote sensing and device communication.**

---

## What is LoRa Transceiver?

LoRa (Long Range) transceiver is a wireless communication technology that enables long-distance, low-power data transmission. This project implements a full-duplex LoRa communication system using the SX1278 module with Raspberry Pi Pico, perfect for IoT applications, environmental monitoring, and remote data collection.

### Key Capabilities

- **📡 Extended Range**: Up to 10+ km communication distance
- **🔄 Full-Duplex Operation**: Send and receive simultaneously
- **📊 Signal Quality Metrics**: Monitor RSSI and SNR in real-time
- **🛡️ Reliable Transmission**: Built-in CRC error detection and validation
- **🎯 433 MHz ISM Band**: License-free frequency for industrial applications
- **⚡ Ultra-Low Power Consumption**: Sleep modes for battery-powered devices
- **🔌 Easy Integration**: MicroPython on Raspberry Pi Pico

## Hardware Setup

### Required Components

| Component | Specification | Purpose |
|-----------|---------------|---------|
| **Microcontroller** | Raspberry Pi Pico (RP2040) | Main processor |
| **LoRa Module** | SX1278 433MHz | Wireless transceiver |
| **Power Supply** | 3.3V / 5V regulated | Module power |
| **Antenna** | 433MHz dipole/whip | RF transmission |

### Raspberry Pi Pico Pin Configuration

Wiring for SX1278 LoRa module to Raspberry Pi Pico:

| SX1278 Pin | Function | Pico GPIO | Wire Color |
|------------|----------|-----------|-----------|
| MOSI | SPI Data In | GP3 | Yellow |
| MISO | SPI Data Out | GP4 | Green |
| SCK | SPI Clock | GP2 | Blue |
| NSS/CS | Chip Select | GP5 | Orange |
| RST | Reset | GP6 | Red |
| GND | Ground | GND | Black |
| VCC | Power 3.3V | 3V3 | Red/Power |

## Software Implementation

### SX1278 LoRa Driver Class

The `SX1278` class implements complete SPI communication, register control, and LoRa protocol handling:

```python
class SX1278:
    def __init__(self, spi, cs, rst)
    def reset(self)
    def read_register(reg)
    def write_register(reg, value)
    def init_lora()
    def send(message)
    def receive()
```

### Core Functions

#### Initialization Method: `init_lora()`
- Resets module hardware
- Enables LoRa mode (sets bit 7 in OP_MODE register)
- Configures frequency to 433 MHz
- Sets transmit power to maximum (~20 dBm)
- Applies modem configuration (BW, spreading factor, CRC)
- Verifies module version (0x12 for SX1278)

#### Transmission Method: `send(message)`
- Encodes message to bytes
- Sets FIFO pointer to TX base address
- Writes payload to FIFO buffer
- Sets payload length register
- Initiates transmission
- Waits for completion (1000ms timeout)
- Returns to RX mode on success

#### Reception Method: `receive()`
- Polls IRQ flags for RX completion
- Validates CRC on received packet
- Extracts payload from FIFO
- Measures signal quality (RSSI/SNR)
- Returns structured data dictionary

### SX1278 Register Map

| Register Name | Address | Function |
|---------------|---------|----------|
| OP_MODE | 0x01 | Mode control (sleep, standby, TX, RX) |
| FRF_MSB/MID/LSB | 0x06-08 | Frequency tuning |
| PA_CONFIG | 0x09 | Power amplifier gain |
| FIFO_TX_BASE | 0x0E | TX FIFO starting address |
| FIFO_RX_BASE | 0x0F | RX FIFO starting address |
| FIFO_ADDR_PTR | 0x0D | Current FIFO address pointer |
| FIFO | 0x00 | FIFO data register |
| IRQ_FLAGS | 0x12 | Interrupt status flags |
| RX_NB_BYTES | 0x13 | Received packet length |
| MODEM_CONFIG_1 | 0x1D | Bandwidth and coding rate |
| MODEM_CONFIG_2 | 0x1E | Spreading factor and CRC |
| MODEM_CONFIG_3 | 0x26 | Automatic gain control |
| PAYLOAD_LENGTH | 0x22 | Packet length |
| PKT_SNR_VALUE | 0x19 | Signal-to-noise ratio |
| PKT_RSSI_VALUE | 0x1A | Signal strength |
| SYNC_WORD | 0x39 | Network synchronization word |
| VERSION | 0x42 | Module version (0x12) |

### Operating Modes

| Mode Name | Value | Application |
|-----------|-------|-------------|
| Sleep | 0x00 | Low-power idle state |
| Standby | 0x01 | Ready for TX/RX |
| Transmit | 0x03 | Sending data |
| RX Continuous | 0x05 | Always listening for packets |

## LoRa Configuration & Performance

### Physical Layer Settings

```python
# Frequency Configuration
Frequency: 433 MHz (ISM Band - unlicensed)
FRF_MSB: 0x6C
FRF_MID: 0x40
FRF_LSB: 0x00
Calculated Frequency: 433.0 MHz

# Modulation Settings
Bandwidth (BW): 125 kHz
Coding Rate (CR): 4/5
Spreading Factor (SF): 7
Preamble Length: 8
CRC: Enabled
Sync Word: 0xF3 (default)
```

### Performance Metrics

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Data Rate** | 5.47 kbps | ~43-50 bytes/sec |
| **Time on Air** | 410 ms | Per 23-byte packet |
| **Range** | 1-10+ km | Line-of-sight, depends on environment |
| **TX Power** | +20 dBm | ~100 mW maximum output |
| **Receiver Sensitivity** | -139 dBm | Minimum detectable signal |
| **SPI Speed** | 5 MHz | Clock frequency |

### Signal Quality Indicators

#### RSSI (Received Signal Strength Indicator)
- **Definition**: Power level of received radio signal
- **Unit**: dBm (decibels relative to 1 milliwatt)
- **Range**: -164 to 0 dBm
- **Interpretation**:
  - -60 dBm or better: Excellent signal
  - -90 to -60 dBm: Good signal
  - -120 to -90 dBm: Fair signal
  - Below -120 dBm: Weak signal
- **Calculation**: `RSSI = -164 + register_value`

#### SNR (Signal-to-Noise Ratio)
- **Definition**: Ratio of signal power to noise power
- **Unit**: dB (decibels)
- **Range**: -20 to +10 dB
- **Interpretation**:
  - Above +5 dB: Strong, reliable link
  - 0 to +5 dB: Usable connection
  - Below 0 dB: Poor signal quality, errors likely
- **Calculation**: Convert 8-bit two's complement to decimal, divide by 4

## Development Roadmap

### Current Features ✓
- SX1278 driver with full register control
- 433 MHz long-range communication
- RSSI and SNR monitoring
- CRC error detection
- MicroPython support

### Planned Enhancements
- [ ] Frequency hopping for interference avoidance
- [ ] Message encryption (AES-128)
- [ ] Multi-packet fragmentation support
- [ ] Adaptive power control based on RSSI
- [ ] Network addressing and routing
- [ ] Web dashboard with real-time statistics
- [ ] OTA firmware updates
- [ ] Mesh networking capability

## Performance Comparison with Other Technologies

| Technology | Range | Power | Cost | Use Case |
|-----------|-------|-------|------|----------|
| **LoRa (433 MHz)** | 10+ km | Ultra-low | $$ | IoT sensors, remote monitoring |
| Bluetooth LE | 100 m | Low | $ | Wearables, personal devices |
| WiFi | 100 m | Medium | $ | Home networks, local data |
| Cellular 4G | Global | High | $$$ | Mobile, cloud connectivity |
| Zigbee | 100 m | Low | $$ | Home automation mesh |

## Community & Support

- **Repository**: ismailone23/LoRa-transceiver
- **Platform**: Raspberry Pi Pico (RP2040)
- **Language**: MicroPython
- **License**: Open Source

## FAQ

**Q: Can I use this with 868 MHz frequency?**
A: Yes, the code supports frequency configuration. Change FRF registers for 868 MHz (Europe) or 915 MHz (US).

**Q: What is the maximum message length?**
A: The FIFO buffer supports up to 255 bytes, but practically 250 bytes per packet is recommended.

**Q: How do I synchronize multiple modules?**
A: Ensure all modules use the same sync word (0xF3), frequency, bandwidth, and spreading factor.

**Q: Can I use other LoRa modules?**
A: Yes, compatible SX127x modules work. Version register must read 0x12 for SX1278.

**Q: What's the maximum transmit power?**
A: +20 dBm (~100 mW) on 433 MHz band. Check local regulations for maximum EIRP.

**Q: How do I reduce power consumption?**
A: Use sleep mode when not transmitting, increase message intervals, or reduce spreading factor.

## Related Resources

- **[Raspberry Pi Pico Documentation](https://www.raspberrypi.org/documentation/microcontrollers/)**
- **[MicroPython Documentation](https://micropython.org/)**
- **[SX1278 Datasheet](https://www.semtech.com/products/wireless-rf/lora-transceivers/sx1278)**
- **[LoRa Alliance](https://lora-alliance.org/)**
- **[ISM Band Regulations](https://en.wikipedia.org/wiki/ISM_band)**

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | Jan 2026 | Initial Pico release, SX1278 support, RX/TX demo |

---

**Project Status**: ✅ Active Development
**Last Updated**: January 12, 2026
**Contributors**: IoT Prototype Rover Team
**Compatibility**: Raspberry Pi Pico (RP2040), MicroPython 1.19+
