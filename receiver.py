from machine import Pin, SPI, PWM, time_pulse_us, I2C, UART
import time
import json
import dht
import math

# Pin Definitions
LORA_SCK, LORA_MOSI, LORA_MISO, LORA_CS, LORA_RST = 2, 3, 4, 5, 6
MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_IN4 = 13, 14, 15, 16
MOTOR_EN_A, MOTOR_EN_B = 17, 18
ULTRASONIC_TRIG, ULTRASONIC_ECHO, DHT_PIN = 10, 11, 12
PCA_SDA, PCA_SCL = 20, 21
SERVO_FL, SERVO_FR, SERVO_RL, SERVO_RR = 0, 1, 2, 3
GPS_UART, GPS_BAUD = 0, 9600
COMPASS_I2C, COMPASS_SDA, COMPASS_SCL = 1, 26, 27

# Navigation parameters
ARRIVAL_RADIUS_M = 3.0
FORWARD_TOLERANCE_DEG = 15.0
NAV_SPEED = 60
OBSTACLE_DISTANCE_CM = 30

# Compass calibration
MAG_OFFSET_X = 32765.0
MAG_OFFSET_Y = 48073.0
DECLINATION_DEG = -0.4

class QMC5883L:
    ADDR = 0x0D
    def __init__(self, i2c, offset_x=0, offset_y=0, declination=0):
        self.i2c = i2c
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.declination = declination
        self.init_sensor()

    def init_sensor(self):
        try:
            self.i2c.writeto_mem(self.ADDR, 0x0B, bytes([0x01]))
            self.i2c.writeto_mem(self.ADDR, 0x09, bytes([0xD5]))
            self.i2c.writeto_mem(self.ADDR, 0x0A, bytes([0x00]))
            time.sleep_ms(10)
            return True
        except:
            return False

    def read_heading(self):
        try:
            data = self.i2c.readfrom_mem(self.ADDR, 0x00, 6)
            x = int.from_bytes(data[0:2], 'little', True)
            y = int.from_bytes(data[2:4], 'little', True)
            x_cal = x - self.offset_x
            y_cal = y - self.offset_y
            heading = math.degrees(math.atan2(y_cal, x_cal)) + self.declination
            while heading < 0: heading += 360
            while heading >= 360: heading -= 360
            return heading
        except:
            return None

class PCA9685:
    def __init__(self, i2c, address=0x40):
        self.i2c = i2c
        self.address = address
        self.reset()
    def _write(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, bytearray([value]))
    def _read(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]
    def reset(self):
        self._write(0x00, 0x00)
    def freq(self, freq=None):
        if freq is None:
            return 50
        prescale = int(25000000.0 / 4096.0 / freq + 0.5) - 1
        old_mode = self._read(0x00)
        new_mode = (old_mode & 0x7F) | 0x10
        self._write(0x00, new_mode)
        self._write(0xFE, prescale)
        self._write(0x00, old_mode)
        time.sleep(0.005)
        self._write(0x00, old_mode | 0xa1)
    def pwm(self, index, on, off):
        self._write(0x06 + 4 * index, on & 0xFF)
        self._write(0x07 + 4 * index, on >> 8)
        self._write(0x08 + 4 * index, off & 0xFF)
        self._write(0x09 + 4 * index, off >> 8)
    def duty(self, index, value): self.pwm(index, 0, value)

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
        self.reset()
        self.write_register(0x01, 0x80)
        time.sleep(0.01)
        self.write_register(0x01, 0x81)
        time.sleep(0.01)
        self.write_register(0x06, 0x6C)
        self.write_register(0x07, 0x40)
        self.write_register(0x08, 0x00)
        self.write_register(0x09, 0x8F)
        self.write_register(0x1D, 0x72)
        self.write_register(0x1E, 0x74)
        self.write_register(0x26, 0x04)
        self.write_register(0x39, 0xF3)
        self.write_register(0x0E, 0x00)
        self.write_register(0x0F, 0x00)
        return True

    def receive(self):
        irq = self.read_register(0x12)
        if irq & 0x40:
            length = self.read_register(0x13)
            current = self.read_register(0x10)
            self.write_register(0x0D, current)
            self.cs.value(0)
            self.spi.write(bytes([0x00 & 0x7F]))
            payload = self.spi.read(length)
            self.cs.value(1)
            self.write_register(0x12, 0xFF)
            try:
                return payload.decode('utf-8')
            except:
                return None
        return None

    def send(self, message):
        self.write_register(0x01, 0x81)
        self.write_register(0x0D, 0x00)
        if isinstance(message, str):
            message = message.encode()
        self.cs.value(0)
        self.spi.write(bytes([0x00 | 0x80]))
        self.spi.write(message)
        self.cs.value(1)
        self.write_register(0x22, len(message))
        self.write_register(0x12, 0xFF)
        self.write_register(0x01, 0x83)
        start = time.ticks_ms()
        while not (self.read_register(0x12) & 0x08):
            if time.ticks_diff(time.ticks_ms(), start) > 1000:
                return False
        self.write_register(0x12, 0xFF)
        self.write_register(0x01, 0x85)
        return True

def wrap_360(deg):
    while deg < 0: deg += 360
    while deg >= 360: deg -= 360
    return deg

def angle_diff_signed(target, current):
    diff = wrap_360(target) - wrap_360(current)
    if diff > 180: diff -= 360
    if diff < -180: diff += 360
    return diff

def distance_meters(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi/2)**2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def bearing_degrees(lat1, lon1, lat2, lon2):
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    y = math.sin(dlambda) * math.cos(phi2)
    x = (math.cos(phi1) * math.sin(phi2) -
         math.sin(phi1) * math.cos(phi2) * math.cos(dlambda))
    return wrap_360(math.degrees(math.atan2(y, x)))

class Robot:
    def __init__(self):
        self.m1 = Pin(MOTOR_IN1, Pin.OUT)
        self.m2 = Pin(MOTOR_IN2, Pin.OUT)
        self.m3 = Pin(MOTOR_IN3, Pin.OUT)
        self.m4 = Pin(MOTOR_IN4, Pin.OUT)
        self.ena = PWM(Pin(MOTOR_EN_A))
        self.ena.freq(1000)
        self.enb = PWM(Pin(MOTOR_EN_B))
        self.enb.freq(1000)

        # for servos
        self.pca = None
        try:
            i2c_pca = I2C(0, sda=Pin(PCA_SDA), scl=Pin(PCA_SCL), freq=400000)
            self.pca = PCA9685(i2c_pca)
            self.pca.freq(50)
            self.manual_steer(90)
        except:
            pass

        # Compass
        self.compass = None
        try:
            i2c_compass = I2C(COMPASS_I2C, sda=Pin(COMPASS_SDA), scl=Pin(COMPASS_SCL), freq=400000)
            self.compass = QMC5883L(i2c_compass, MAG_OFFSET_X, MAG_OFFSET_Y, DECLINATION_DEG)
        except:
            pass

        # GPS
        self.uart = UART(GPS_UART, GPS_BAUD)
        self.current_lat = None
        self.current_lon = None

        # Ultrasonic and DHT
        self.trig = Pin(ULTRASONIC_TRIG, Pin.OUT)
        self.echo = Pin(ULTRASONIC_ECHO, Pin.IN)
        try:
            self.dht = dht.DHT11(Pin(DHT_PIN))
        except:
            self.dht = None

        # Auto-drive state
        self.auto_mode = False
        self.dest_lat = None
        self.dest_lon = None

    def set_servo_angle(self, channel, angle):
        if not self.pca:
            return
        angle = max(0, min(180, angle))
        pulse_us = 500 + (angle / 180.0) * 1900
        duty = int((pulse_us / 20000) * 4096)
        self.pca.duty(channel, duty)

    def manual_steer(self, input_angle):
        if not self.pca:
            return
        input_angle = max(45, min(135, input_angle))
        dev = input_angle - 90
        front, rear = 90 + dev, 90 - dev

        self.set_servo_angle(SERVO_FL, front)
        self.set_servo_angle(SERVO_FR, front)
        time.sleep(0.02)
        self.set_servo_angle(SERVO_RL, rear)
        self.set_servo_angle(SERVO_RR, rear)
        time.sleep(0.02)
        time.sleep(0.3)

        for i in [SERVO_FL, SERVO_FR, SERVO_RL, SERVO_RR]:
            self.pca.duty(i, 0)

    def get_distance_cm(self):
        self.trig.low()
        time.sleep_us(2)
        self.trig.high()
        time.sleep_us(10)
        self.trig.low()
        try:
            dur = time_pulse_us(self.echo, 1, 60000)
            if dur > 0:
                return round((dur * 0.0343) / 2, 1)
        except:
            pass
        return -1

    def update_gps(self):
        if self.uart.any():
            try:
                data = self.uart.read()
                if data:
                    lines = data.decode().split('\n')
                    for line in lines:
                        if line.startswith('$GPRMC'):
                            parts = line.split(',')
                            if len(parts) > 6 and parts[2] == 'A':
                                try:
                                    lat_str = parts[3]
                                    lat = float(lat_str[:2]) + float(lat_str[2:])/60.0
                                    if parts[4] == 'S':
                                        lat = -lat

                                    lon_str = parts[5]
                                    lon = float(lon_str[:3]) + float(lon_str[3:])/60.0
                                    if parts[6] == 'W':
                                        lon = -lon

                                    self.current_lat = lat
                                    self.current_lon = lon
                                except:
                                    pass
            except:
                pass

    def get_telemetry(self):
        dist = self.get_distance_cm()
        temp, hum = 0, 0

        if self.dht:
            try:
                self.dht.measure()
                temp = self.dht.temperature()
                hum = self.dht.humidity()
            except:
                pass

        heading = None
        if self.compass:
            heading = self.compass.read_heading()

        return json.dumps({
            "d": distance_meters(self.current_lat, self.current_lon,
                                  self.dest_lat, self.dest_lon),
            "t": temp,
            "h": hum,
            "lat": self.current_lat,
            "lon": self.current_lon,
            "hdg": heading,
            "auto": self.auto_mode
        })

    def move(self, cmd, speed):
        duty = int((speed / 100) * 65535)
        self.ena.duty_u16(duty)
        self.enb.duty_u16(duty)
        if cmd == 'w':
            self.m1.high()
            self.m2.low()
            self.m3.high()
            self.m4.low()
        elif cmd == 's':
            self.m1.low()
            self.m2.high()
            self.m3.low()
            self.m4.high()
        elif cmd == 'a':
            self.m1.low()
            self.m2.high()
            self.m3.high()
            self.m4.low()
        elif cmd == 'd':
            self.m1.high()
            self.m2.low()
            self.m3.low()
            self.m4.high()
        elif cmd == 'x':
            self.m1.low()
            self.m2.low()
            self.m3.low()
            self.m4.low()

    def auto_navigate(self):
        if not self.auto_mode or not self.dest_lat or not self.dest_lon:
            return

        if self.current_lat is None or self.current_lon is None:
            print("Waiting for GPS...")
            self.move('x', 0)
            return

        # Check obstacle
        dist_cm = self.get_distance_cm()
        if dist_cm > 0 and dist_cm < OBSTACLE_DISTANCE_CM:
            print(f"Obstacle detected at {dist_cm}cm - stopping")
            self.move('x', 0)
            return

        # Calculate distance and bearing
        dist_m = distance_meters(self.current_lat, self.current_lon,
                                  self.dest_lat, self.dest_lon)

        # Check arrival
        if dist_m <= ARRIVAL_RADIUS_M:
            print("ARRIVED AT DESTINATION!")
            self.move('x', 0)
            self.auto_mode = False
            return

        target_bearing = bearing_degrees(self.current_lat, self.current_lon,
                                         self.dest_lat, self.dest_lon)

        # Get current heading from compass
        heading = self.compass.read_heading() if self.compass else None
        if heading is None:
            print("No compass data")
            self.move('x', 0)
            return

        # Calculate turn needed
        turn = angle_diff_signed(target_bearing, heading)

        print(f"Dist:{dist_m:.1f}m Brg:{target_bearing:.0f}° Hdg:{heading:.0f}° Turn:{turn:.0f}°")

        # Navigate
        if abs(turn) <= FORWARD_TOLERANCE_DEG:
            self.manual_steer(90)
            self.move('w', NAV_SPEED)
            print("GO FORWARD")
        elif turn > 0:
            # Turn right
            steer_angle = 90 + min(45, abs(turn) / 2)
            self.manual_steer(int(steer_angle))
            self.move('w', NAV_SPEED - 10)
            print(f"TURN RIGHT {abs(turn):.0f}°")
        else:
            # Turn left
            steer_angle = 90 - min(45, abs(turn) / 2)
            self.manual_steer(int(steer_angle))
            self.move('w', NAV_SPEED - 10)
            print(f"TURN LEFT {abs(turn):.0f}°")

spi = SPI(0, baudrate=5000000, polarity=0, phase=0,
          sck=Pin(LORA_SCK), mosi=Pin(LORA_MOSI), miso=Pin(LORA_MISO))
cs = Pin(LORA_CS, Pin.OUT)
rst = Pin(LORA_RST, Pin.OUT)
lora = SX1278(spi, cs, rst)
bot = Robot()
last_send = time.ticks_ms()

if lora.init_lora():
    lora.write_register(0x01, 0x85)
    while True:
        bot.update_gps()

        msg = lora.receive()
        if msg:
            print(f"RX: {msg}")
            try:
                data = json.loads(msg)
                cmd = data.get("cmd", "x")

                if cmd == "auto":
                    bot.auto_mode = True
                    bot.dest_lat = data.get("lat")
                    bot.dest_lon = data.get("lon")

                elif cmd == "stop_auto":
                    bot.auto_mode = False
                    bot.move('x', 0)
                    print("Auto-drive stopped")

                elif cmd == "servo":
                    bot.auto_mode = False
                    bot.manual_steer(data.get("ang", 90))

                else:
                    bot.auto_mode = False
                    bot.move(cmd, data.get("spd", 75))
            except Exception as e:
                print(f"Command error: {e}")

        if bot.auto_mode:
            bot.auto_navigate()

        if time.ticks_diff(time.ticks_ms(), last_send) > 2000:
            telemetry = bot.get_telemetry()
            lora.send(telemetry)
            last_send = time.ticks_ms()

        time.sleep(0.01)
else:
    print("LoRa Failed")
