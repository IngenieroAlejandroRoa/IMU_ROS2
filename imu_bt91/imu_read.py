import asyncio
from bleak import BleakClient

ADDRESS = "C7:B5:46:C9:36:9D"
CHAR_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"

def to_int16(lo, hi):
    value = (hi << 8) | lo
    if value >= 32768:
        value -= 65536
    return value

def parse_packet(data: bytes):
    if len(data) != 20:
        print(f"Paquete con longitud inesperada: {len(data)} bytes -> {data.hex(' ')}")
        return None

    if data[0] != 0x55 or data[1] != 0x61:
        print(f"Trama desconocida: {data.hex(' ')}")
        return None

    values = []
    for i in range(2, 20, 2):
        values.append(to_int16(data[i], data[i+1]))

    ax_raw, ay_raw, az_raw = values[0:3]
    gx_raw, gy_raw, gz_raw = values[3:6]
    roll_raw, pitch_raw, yaw_raw = values[6:9]

    # Escalas probables
    ax = ax_raw / 32768.0 * 16.0
    ay = ay_raw / 32768.0 * 16.0
    az = az_raw / 32768.0 * 16.0

    gx = gx_raw / 32768.0 * 2000.0
    gy = gy_raw / 32768.0 * 2000.0
    gz = gz_raw / 32768.0 * 2000.0

    roll = roll_raw / 32768.0 * 180.0
    pitch = pitch_raw / 32768.0 * 180.0
    yaw = yaw_raw / 32768.0 * 180.0

    return {
        "ax": ax,
        "ay": ay,
        "az": az,
        "gx": gx,
        "gy": gy,
        "gz": gz,
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
    }

def callback(sender, data):
    parsed = parse_packet(data)
    if parsed is None:
        return
    print(
        f"Acc[g]   X={parsed['ax']:7.3f}  Y={parsed['ay']:7.3f}  Z={parsed['az']:7.3f} | "
        f"Gyro[°/s] X={parsed['gx']:7.3f}  Y={parsed['gy']:7.3f}  Z={parsed['gz']:7.3f} | "
        f"Ang[°] Roll={parsed['roll']:7.2f}  Pitch={parsed['pitch']:7.2f}  Yaw={parsed['yaw']:7.2f}"
    )

async def main():
    async with BleakClient(ADDRESS) as client:
        print("✅ Conectado a la IMU")
        await client.start_notify(CHAR_UUID, callback)
        print("📡 Leyendo datos convertidos... Ctrl+C para salir")
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("\n⏹️ Detenido por usuario")
        finally:
            await client.stop_notify(CHAR_UUID)

if __name__ == "__main__":
    asyncio.run(main())
