"""
pytest tests for can_simulator.py

Covers all pure functions that do NOT require a running CARLA server.
Run with:  pytest test_can_simulator.py -v
"""

import struct
import time
import os
import tempfile
import pytest

from can_simulator import (
    CAN_ID_VEHICLE_SPEED, CAN_ID_STEERING, CAN_ID_BRAKE_PRESSURE,
    CAN_ID_THROTTLE, CAN_ID_YAW_RATE, CAN_ID_LONGITUDINAL_ACCEL,
    CAN_ID_RADAR_TARGET, CAN_ID_NAMES, CANFrame, CANDatalogger,
    build_vehicle_speed_frame, build_steering_frame, build_brake_pressure_frame,
    build_throttle_frame, build_yaw_rate_frame, build_longitudinal_accel_frame,
    build_radar_target_frame, _clamp, _pack_uint16, _pack_int16, _pack_uint8,
)


# ---- CAN ID constants ----
class TestCANIDConstants:
    def test_ids_in_standard_range(self):
        ids = [CAN_ID_VEHICLE_SPEED, CAN_ID_STEERING, CAN_ID_BRAKE_PRESSURE,
               CAN_ID_THROTTLE, CAN_ID_YAW_RATE, CAN_ID_LONGITUDINAL_ACCEL,
               CAN_ID_RADAR_TARGET]
        for cid in ids:
            assert 0 <= cid <= 0x7FF

    def test_all_ids_have_names(self):
        ids = [CAN_ID_VEHICLE_SPEED, CAN_ID_STEERING, CAN_ID_BRAKE_PRESSURE,
               CAN_ID_THROTTLE, CAN_ID_YAW_RATE, CAN_ID_LONGITUDINAL_ACCEL,
               CAN_ID_RADAR_TARGET]
        for cid in ids:
            assert cid in CAN_ID_NAMES

    def test_exactly_7_unique_ids(self):
        ids = [CAN_ID_VEHICLE_SPEED, CAN_ID_STEERING, CAN_ID_BRAKE_PRESSURE,
               CAN_ID_THROTTLE, CAN_ID_YAW_RATE, CAN_ID_LONGITUDINAL_ACCEL,
               CAN_ID_RADAR_TARGET]
        assert len(ids) == 7 and len(set(ids)) == 7


# ---- Clamp and pack helpers ----
class TestClamp:
    def test_within(self):           assert _clamp(50, 0, 100) == 50
    def test_below(self):            assert _clamp(-10, 0, 100) == 0
    def test_above(self):            assert _clamp(200, 0, 100) == 100
    def test_boundaries(self):
        assert _clamp(0, 0, 100) == 0
        assert _clamp(100, 0, 100) == 100


class TestPackUint16:
    def test_normal(self):     assert _pack_uint16(100) == b'\x00\x64'
    def test_zero(self):       assert _pack_uint16(0) == b'\x00\x00'
    def test_max(self):        assert _pack_uint16(65535) == b'\xff\xff'
    def test_overflow(self):   assert _pack_uint16(99999) == b'\xff\xff'
    def test_negative(self):   assert _pack_uint16(-1) == b'\x00\x00'


class TestPackInt16:
    def test_pos(self):        assert _pack_int16(100) == b'\x00\x64'
    def test_neg(self):        assert _pack_int16(-100) == b'\xff\x9c'
    def test_zero(self):       assert _pack_int16(0) == b'\x00\x00'
    def test_max_pos(self):    assert _pack_int16(32767) == b'\x7f\xff'
    def test_max_neg(self):    assert _pack_int16(-32768) == b'\x80\x00'
    def test_overflow_pos(self): assert _pack_int16(99999) == b'\x7f\xff'
    def test_overflow_neg(self): assert _pack_int16(-99999) == b'\x80\x00'


class TestPackUint8:
    def test_normal(self):     assert _pack_uint8(128) == b'\x80'
    def test_zero(self):       assert _pack_uint8(0) == b'\x00'
    def test_max(self):        assert _pack_uint8(255) == b'\xff'
    def test_overflow(self):   assert _pack_uint8(999) == b'\xff'
    def test_negative(self):   assert _pack_uint8(-1) == b'\x00'


# ---- CANFrame ----
class TestCANFrame:
    def test_default(self):
        f = CANFrame(0x0C0)
        assert f.can_id == 0x0C0 and f.dlc == 8 and len(f.data) == 8
        assert f.data == bytearray(8) and f.channel == 1 and f.direction == "Rx"

    def test_id_masking(self):
        assert CANFrame(0xFFF).can_id == 0x7FF

    def test_data_padding(self):
        f = CANFrame(0x0C0, bytearray([0x12, 0x34]))
        assert len(f.data) == 8 and f.data[0] == 0x12 and f.data[2] == 0x00

    def test_truncation(self):
        f = CANFrame(0x0C0, bytearray(range(20)))
        assert len(f.data) == 8 and list(f.data) == list(range(8))

    def test_custom_timestamp(self):
        assert CANFrame(0x0C0, timestamp=123.456).timestamp == 123.456

    def test_timestamp_recent(self):
        now = time.time()
        assert abs(CANFrame(0x0C0).timestamp - now) < 1.0


# ---- CANFrame.to_asc() ----
class TestCANFrameToAsc:
    def test_absolute(self):
        assert "5.123456" in CANFrame(0x0C0, timestamp=5.123456).to_asc()

    def test_relative(self):
        assert "5.000000" in CANFrame(0x0C0, timestamp=10.0).to_asc(base_time=5.0)

    def test_negative_clamped(self):
        assert "0.000000" in CANFrame(0x0C0, timestamp=5.0).to_asc(base_time=10.0)

    def test_hex_data(self):
        data = bytearray([0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0])
        asc = CANFrame(0x0C0, data, timestamp=0.0).to_asc(base_time=0.0)
        assert "12 34 56 78 9A BC DE F0" in asc and "d 8" in asc


# ---- CANFrame.encode() ----
class TestCANFrameEncode:
    def test_length(self):
        assert len(CANFrame(0x0C0).encode()) == 12

    def test_round_trip(self):
        f = CANFrame(0x1A0, bytearray([0x01, 0xC2]))
        raw = f.encode()
        assert struct.unpack(">I", raw[:4])[0] == 0x1A0
        assert raw[4] == 0x01 and raw[5] == 0xC2


# ---- Frame builders ----
class TestVehicleSpeed:
    def test_encoding(self):
        f = build_vehicle_speed_frame(85.3, timestamp=1.0)
        assert f.can_id == CAN_ID_VEHICLE_SPEED and f.timestamp == 1.0
        assert struct.unpack(">H", f.data[:2])[0] == 8530

    def test_zero(self):
        assert struct.unpack(">H", build_vehicle_speed_frame(0.0).data[:2])[0] == 0

    def test_reserved_zero(self):
        assert build_vehicle_speed_frame(60.0).data[2:] == b'\x00' * 6


class TestSteering:
    def test_positive(self):
        f = build_steering_frame(12.5, 3.2, timestamp=2.0)
        assert f.can_id == CAN_ID_STEERING
        assert struct.unpack(">h", f.data[:2])[0] == 125
        assert struct.unpack(">h", f.data[2:4])[0] == 32

    def test_negative(self):
        assert struct.unpack(">h", build_steering_frame(-45.0).data[:2])[0] == -450


class TestBrakePressure:
    def test_encoding(self):
        f = build_brake_pressure_frame(150.8)
        assert f.can_id == CAN_ID_BRAKE_PRESSURE
        assert struct.unpack(">H", f.data[:2])[0] == 1508

    def test_max_clamped(self):
        assert struct.unpack(">H", build_brake_pressure_frame(99999).data[:2])[0] == 65535


class TestThrottle:
    def test_full(self):   assert build_throttle_frame(1.0).data[0] == 255
    def test_closed(self): assert build_throttle_frame(0.0).data[0] == 0
    def test_half(self):   assert build_throttle_frame(0.5).data[0] == 128  # round(127.5)=128


class TestYawRate:
    def test_encoding(self):
        assert struct.unpack(">h", build_yaw_rate_frame(-5.7).data[:2])[0] == -57


class TestLongAccel:
    def test_encoding(self):
        assert struct.unpack(">h", build_longitudinal_accel_frame(-3.2).data[:2])[0] == -320


class TestRadarTarget:
    def test_encoding(self):
        f = build_radar_target_frame(42.5, -8.3)
        assert f.can_id == CAN_ID_RADAR_TARGET
        assert struct.unpack(">H", f.data[:2])[0] == 425
        assert struct.unpack(">h", f.data[2:4])[0] == -83


# ---- CANDatalogger ----
class TestCANDatalogger:
    def test_create_write_close(self):
        tmp = os.path.join(tempfile.gettempdir(), "test_can_logger.asc")
        try:
            logger = CANDatalogger(tmp)
            assert logger.get_message_count() == 0
            logger.log_frame(build_vehicle_speed_frame(60.0, timestamp=time.time()))
            logger.log_frame(build_brake_pressure_frame(30.0))
            logger.log_frame(build_throttle_frame(0.5))
            assert logger.get_message_count() == 3
            logger.close()
            with open(tmp, "r") as fh:
                content = fh.read()
            assert "date" in content and "0C0" in content
            assert "End TriggerBlock" in content
        finally:
            if os.path.exists(tmp):
                os.remove(tmp)

    def test_ten_frames(self):
        tmp = os.path.join(tempfile.gettempdir(), "test_can_10.asc")
        try:
            logger = CANDatalogger(tmp)
            for i in range(10):
                f = CANFrame(0x0C0, bytearray([i]), timestamp=float(i) * 0.05)
                logger.log_frame(f)
            assert logger.get_message_count() == 10
            logger.close()
        finally:
            if os.path.exists(tmp):
                os.remove(tmp)


# ---- Edge cases ----
class TestEdgeCases:
    def test_large_speed_clamped(self):
        assert struct.unpack(">H", build_vehicle_speed_frame(999.99).data[:2])[0] == 0xFFFF

    def test_float_rounding(self):
        assert struct.unpack(">H", build_vehicle_speed_frame(85.37).data[:2])[0] == 8537

    def test_repr(self):
        r = repr(CANFrame(0x0C0, bytearray([0x12, 0x34]), timestamp=1.0))
        assert "0x0C0" in r and "VehicleSpeed" in r and "1.000000" in r


# ---- All 7 builders smoke test ----
class TestAllBuilders:
    def test_all_valid_frames(self):
        builders = [
            (build_vehicle_speed_frame, [60.0], CAN_ID_VEHICLE_SPEED),
            (build_steering_frame, [10.0], CAN_ID_STEERING),
            (build_brake_pressure_frame, [50.0], CAN_ID_BRAKE_PRESSURE),
            (build_throttle_frame, [0.5], CAN_ID_THROTTLE),
            (build_yaw_rate_frame, [3.0], CAN_ID_YAW_RATE),
            (build_longitudinal_accel_frame, [-2.0], CAN_ID_LONGITUDINAL_ACCEL),
            (build_radar_target_frame, [30.0, -5.0], CAN_ID_RADAR_TARGET),
        ]
        for builder, args, expected_id in builders:
            f = builder(*args)
            assert isinstance(f, CANFrame)
            assert f.can_id == expected_id
            assert f.dlc == 8 and len(f.data) == 8
