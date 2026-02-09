import struct
import time
from typing import Optional
import serial

# import hexdump

port = "COM4"
baudrate = 115200

CONTROL_HEADER = 0x01
TX_PACKET_SIZE = 1 + 4 + 4 + 1 + 1  # 14 bytes
RX_PACKET_SIZE = 1 + 4 + 4 + 4 + 1 + 1 + 1  # 16 bytes


def xor_checksum(data: bytes) -> int:
    checksum = 0
    for b in data[1:]:
        checksum ^= b
    return checksum

ser = serial.Serial(port, baudrate=baudrate, timeout=1)

# def send_speeds(
#     left_motor_speed: float, right_motor_speed: float, gripper: bool
# ) -> Optional[tuple[float, float, float, bool, bool]]:
def send_speeds(left_motor_speed, right_motor_speed, gripper):
    ans: Optional[tuple[float, float, float, bool, bool]] = None

    args: list[float] = [
        CONTROL_HEADER,
        int(left_motor_speed),
        int(right_motor_speed),
        int(gripper),
    ]

    checksum = xor_checksum(struct.pack("<BffB", *args))
    args.append(checksum)
    command = struct.pack("<BffBB", *args)

    try:
        time.sleep(0.1)

        ser.write(command)
        print(f"Sent: {command}")

        ser.timeout = 0.1
        header = ser.read(1)
        while len(header) == 1 and header[0] != 0x01:
            header = ser.read(1)
            
        response = ser.read(RX_PACKET_SIZE-1)
        if len(response) != RX_PACKET_SIZE-1:
            print("Invalid response size:", len(response))
            return None
        
        print(f"Response: {response}")

        checksum = 0
        for byte in response:
            checksum ^= byte

        if checksum != 0:
            print("Invalid checksum:", checksum)
            return None
        
        # response = bytearray(response)
        # response.insert(0, 0x01)
        print(f"Resp: {response}, resplen: {len(response)}")

        ans = struct.unpack("<fffBBB", response)

        # print("Received:")
        # print("  x =", ans[0])
        # print("  y =", ans[1])
        # print("  theta =", ans[2])
        # print("  left_usik =", bool(ans[3]))
        # print("  right_usik =", bool(ans[4]))

    except serial.SerialException as e:
        print("Serial error:", e)

    # finally:
    #     if ser is not None and ser.is_open:
    #         ser.close()

    return ans


if __name__ == "__main__":
    
    k = 50 / 0.25
    
    time.sleep(5)
    
    print(send_speeds(0.1*k, 0.1*k, True))
    
    time.sleep(2)
    
    print(send_speeds(0, 0, False))
    
    time.sleep(5)

    # data_to_write = struct.pack(
    #     "<BfffBBB",
    #     0x01,
    #     0.5,
    #     4.125,
    #     4.25,
    #     int(True),
    #     int(False),
    #     0x32,
    # )

    # with open("output.bin", "wb") as binary_file:
    #     binary_file.write(data_to_write)

    # with open("output.bin", "rb") as binary_file:
    #     response = binary_file.read(RX_PACKET_SIZE)
    #     hexdump.hexdump(response)

    #     if xor_checksum(response) != 0:
    #         print("Invalid checksum")

    #     ans = struct.unpack("<BfffBBB", response)

    #     print("Received:")
    #     print("  x =", ans[1])
    #     print("  y =", ans[2])
    #     print("  theta =", ans[3])
    #     print("  left_usik =", bool(ans[4]))
    #     print("  right_usik =", bool(ans[5]))
