from dataclasses import dataclass
from typing import Optional


GRAVITY = 9.80665


@dataclass
class ImuSample:
    ax_g: float
    ay_g: float
    az_g: float
    gx_dps: float
    gy_dps: float
    gz_dps: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


def to_int16(lo: int, hi: int) -> int:
    value = (hi << 8) | lo
    if value >= 32768:
        value -= 65536
    return value


def parse_bt91_packet(data: bytes) -> Optional[ImuSample]:
    if len(data) != 20:
        return None
    if data[0] != 0x55 or data[1] != 0x61:
        return None

    values = [to_int16(data[i], data[i + 1]) for i in range(2, 20, 2)]
    ax_raw, ay_raw, az_raw = values[0:3]
    gx_raw, gy_raw, gz_raw = values[3:6]
    roll_raw, pitch_raw, yaw_raw = values[6:9]

    return ImuSample(
        ax_g=ax_raw / 32768.0 * 16.0,
        ay_g=ay_raw / 32768.0 * 16.0,
        az_g=az_raw / 32768.0 * 16.0,
        gx_dps=gx_raw / 32768.0 * 2000.0,
        gy_dps=gy_raw / 32768.0 * 2000.0,
        gz_dps=gz_raw / 32768.0 * 2000.0,
        roll_deg=roll_raw / 32768.0 * 180.0,
        pitch_deg=pitch_raw / 32768.0 * 180.0,
        yaw_deg=yaw_raw / 32768.0 * 180.0,
    )
