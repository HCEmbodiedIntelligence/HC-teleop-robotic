import socket
import struct
import time

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5005
DISCOVERY_PORT = 5006
POSE_TIMEOUT_SECONDS = 0.2
PRINT_INTERVAL_SECONDS = 0.1

DISCOVERY_REQUEST = b"PICO_DISCOVER_V1"
DISCOVERY_RESPONSE = f"PICO_RECEIVER_V1|{LISTEN_PORT}".encode("ascii")

# v2: header + 3 poses + left input + right input.
# Each controller input is: held/pressed/released uint16 + 6 float values.
PACKET_FORMAT = "<4sBIdB21f3H6f3H6f"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
MAGIC = b"PICO"
PROTOCOL_VERSION = 2

HEAD_TRACKED_FLAG = 1
LEFT_TRACKED_FLAG = 2
RIGHT_TRACKED_FLAG = 4

BUTTON_NAMES = {
    1 << 0: "primary",             # X / A
    1 << 1: "secondary",           # Y / B
    1 << 2: "grip_button",
    1 << 3: "trigger_button",
    1 << 4: "menu",
    1 << 5: "primary_axis_click",
    1 << 6: "primary_axis_touch",
    1 << 7: "secondary_axis_click",
    1 << 8: "secondary_axis_touch",
    1 << 9: "primary_touch",
    1 << 10: "secondary_touch",
}


def read_pose(values, offset):
    return {
        "position": tuple(values[offset:offset + 3]),
        "quaternion": tuple(values[offset + 3:offset + 7]),
    }


def decode_buttons(mask):
    return [name for bit, name in BUTTON_NAMES.items() if mask & bit]


def read_controller_input(values, offset):
    held, pressed, released = values[offset:offset + 3]
    return {
        "held_mask": held,
        "pressed_mask": pressed,
        "released_mask": released,
        "held": decode_buttons(held),
        "pressed": decode_buttons(pressed),
        "released": decode_buttons(released),
        "trigger": values[offset + 3],
        "grip": values[offset + 4],
        "primary_axis": tuple(values[offset + 5:offset + 7]),
        "secondary_axis": tuple(values[offset + 7:offset + 9]),
    }


def is_sequence_newer(sequence, last_sequence):
    if last_sequence is None:
        return True
    difference = (sequence - last_sequence) & 0xFFFFFFFF
    return 0 < difference < 0x80000000


def packet_loss_count(sequence, last_sequence):
    if last_sequence is None:
        return 0
    difference = (sequence - last_sequence) & 0xFFFFFFFF
    return difference - 1 if 1 < difference < 0x80000000 else 0


def stop_robot(reason):
    print(f"[SAFETY] STOP ROBOT: {reason}")
    # 在这里调用机器人的停止接口。


def stop_left_arm(reason):
    print(f"[SAFETY] STOP LEFT ARM: {reason}")


def stop_right_arm(reason):
    print(f"[SAFETY] STOP RIGHT ARM: {reason}")


def main():
    pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pose_sock.bind((LISTEN_IP, LISTEN_PORT))
    pose_sock.settimeout(0.05)

    discovery_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    discovery_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    discovery_sock.bind((LISTEN_IP, DISCOVERY_PORT))
    discovery_sock.setblocking(False)

    print(f"Pose receiver: UDP {LISTEN_IP}:{LISTEN_PORT}")
    print(f"Discovery service: UDP {LISTEN_IP}:{DISCOVERY_PORT}")
    print(f"Expected pose packet: {PACKET_SIZE} bytes")

    last_sequence = None
    last_sender = None
    last_packet_time = time.monotonic()
    last_print_time = 0.0
    timeout_active = False
    total_received = total_lost = total_old = total_invalid = 0

    stop_robot("receiver startup")

    try:
        while True:
            while True:
                try:
                    data, sender = discovery_sock.recvfrom(512)
                except BlockingIOError:
                    break

                if data.strip() == DISCOVERY_REQUEST:
                    discovery_sock.sendto(DISCOVERY_RESPONSE, sender)
                    print(f"[DISCOVERY] replied to PICO {sender[0]}:{sender[1]}")

            try:
                packet, sender = pose_sock.recvfrom(2048)
            except socket.timeout:
                elapsed = time.monotonic() - last_packet_time
                if elapsed > POSE_TIMEOUT_SECONDS and not timeout_active:
                    timeout_active = True
                    stop_robot(f"no pose data for {elapsed * 1000:.0f} ms")
                continue

            now = time.monotonic()

            if len(packet) != PACKET_SIZE:
                total_invalid += 1
                continue

            try:
                unpacked = struct.unpack(PACKET_FORMAT, packet)
            except struct.error:
                total_invalid += 1
                continue

            magic, version, sequence, pico_timestamp, flags = unpacked[:5]
            values = unpacked[5:]

            if magic != MAGIC or version != PROTOCOL_VERSION:
                total_invalid += 1
                continue

            if last_sender is not None and sender != last_sender:
                last_sequence = None
            last_sender = sender

            if not is_sequence_newer(sequence, last_sequence):
                total_old += 1
                continue

            total_lost += packet_loss_count(sequence, last_sequence)
            last_sequence = sequence
            last_packet_time = now
            total_received += 1

            if timeout_active:
                timeout_active = False
                print("[SAFETY] stream recovered; robot remains stopped")

            head_valid = bool(flags & HEAD_TRACKED_FLAG)
            left_valid = bool(flags & LEFT_TRACKED_FLAG)
            right_valid = bool(flags & RIGHT_TRACKED_FLAG)
            left_input = read_controller_input(values, 21)
            right_input = read_controller_input(values, 30)

            if not head_valid:
                stop_robot("head tracking invalid")
            if not left_valid:
                stop_left_arm("left controller tracking invalid")
            if not right_valid:
                stop_right_arm("right controller tracking invalid")

            if left_input["pressed"] or left_input["released"]:
                print(
                    f"Left events: pressed={left_input['pressed']}, "
                    f"released={left_input['released']}"
                )
            if right_input["pressed"] or right_input["released"]:
                print(
                    f"Right events: pressed={right_input['pressed']}, "
                    f"released={right_input['released']}"
                )

            if now - last_print_time >= PRINT_INTERVAL_SECONDS:
                last_print_time = now
                print(
                    f"seq={sequence}, pico_time={pico_timestamp:.3f}, from={sender}, "
                    f"tracking=({head_valid}, {left_valid}, {right_valid})"
                )
                if head_valid:
                    print("Head :", read_pose(values, 0))
                if left_valid:
                    print("Left :", read_pose(values, 7))
                if right_valid:
                    print("Right:", read_pose(values, 14))
                print(
                    "Left input : "
                    f"held={left_input['held']}, "
                    f"trigger={left_input['trigger']:.3f}, "
                    f"grip={left_input['grip']:.3f}, "
                    f"axis={left_input['primary_axis']}"
                )
                print(
                    "Right input: "
                    f"held={right_input['held']}, "
                    f"trigger={right_input['trigger']:.3f}, "
                    f"grip={right_input['grip']:.3f}, "
                    f"axis={right_input['primary_axis']}"
                )
                print(
                    f"stats: received={total_received}, lost={total_lost}, "
                    f"old={total_old}, invalid={total_invalid}"
                )

    except KeyboardInterrupt:
        print("Receiver stopped by user")
    finally:
        stop_robot("receiver shutdown")
        pose_sock.close()
        discovery_sock.close()


if __name__ == "__main__":
    main()
