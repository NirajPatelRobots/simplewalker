""" Print the state of simplewalker on desktop base computer. Receives from simplewalker over wifi.
June 2022
TODO:
    BUG: suddenly stopped returning when timeout, just blocks. Why?
    set from log?
    way to automatically deal with arrays
    clearly show when message is old
    better message struct input:
        parse field_names and struct_format from struct definition string
        parse messages.h?
"""

import socket
from xml.etree import ElementTree
import dataclasses
from typing import List, Dict, Any, Tuple
import struct
import sys
from time import sleep, time
try:
    from rich.live import Live
    from rich.table import Table
    from rich.console import Group
    from rich.panel import Panel
    from rich.align import Align
    from rich.text import Text
except ImportError:
    RICH_DISPLAY = False
    class Live:  # dummy context
        def __init__(self, *args):
            pass
        def __enter__(self):
            return self
        def __exit__(self, exc_type, exc_value, exc_traceback):
            pass
else:
    RICH_DISPLAY = True


@dataclasses.dataclass
class RemoteRobotState:
    """ State of simplewalker received by the base station. """
    msgID: int = 0
    errcode: int = 0
    timestamp_us: int = 0
    pos: List[float] = dataclasses.field(default_factory=lambda: [0., 0., 0.])
    axis: List[float] = dataclasses.field(default_factory=lambda: [0., 0., 0.])
    vel: List[float] = dataclasses.field(default_factory=lambda: [0., 0., 0.])
    angvel: List[float] = dataclasses.field(default_factory=lambda: [0., 0., 0.])
    right_leg_pos: List[float] = dataclasses.field(default_factory=lambda: [0., 0., 0.])
    left_leg_pos: List[float] = dataclasses.field(default_factory=lambda: [0., 0., 0.])
    right_leg_vel: List[float] = dataclasses.field(default_factory=lambda: [0., 0., 0.])
    left_leg_vel: List[float] = dataclasses.field(default_factory=lambda: [0., 0., 0.])
    elementnames = ["pos", "axis", "vel", "angvel",
                    "right_leg_pos", "left_leg_pos", "right_leg_vel", "left_leg_vel"]
    vector_length = 3 * len(elementnames)
    num_bytes = 8 + 4 * vector_length
    struct_format = "hhl" + "f" * vector_length

    def set_from_msg(self, msg: bytes):
        if len(msg) < self.num_bytes:
            print(len(msg), "Byte message not long enough for state decode", end='\r')
            return False
        unpacked_state = struct.unpack(self.struct_format, msg)
        self.msgID = unpacked_state[0]
        self.errcode = unpacked_state[1]
        self.timestamp_us = unpacked_state[2]
        index = 3
        for elementname in self.elementnames:
            element = [0] * 3
            for j in range(3):
                element[j] = unpacked_state[index]
                index += 1
            setattr(self, elementname, element)
        return True

    def set_from_log(self):
        pass # TODO

    def __repr__(self) -> str:
        return "pos:" + str(self.pos) + " axis:" + str(self.axis) \
               + " vel:" + str(self.vel) + " angvel:" + str(self.angvel)


@dataclasses.dataclass
class MessageInfo:
    name: str
    format: str
    length: int
    field_names: List[str]


class BaseReceiver:
    """ Connects to simplewalker and receives information"""
    def __init__(self):
        if "--local" in sys.argv:
            self.target_host = socket.gethostbyname("localhost")
        else:
            self.target_host = socket.gethostbyname("raspberrypi")
        self.client_socket = None
        self.num_bytes_per_read = RemoteRobotState.num_bytes
        self.msg_info: Dict[bytes, MessageInfo] = {}
        self.messages = {}
        self.bytes_in = bytes()
        self.connected_but_timeout = False
        self.last_rx_time = time()
        self.rx_loop_time = 0

    def get_state_msg(self) -> bytes:
        return self.client_socket.recv(RemoteRobotState.num_bytes)

    def register_message(self, name: str, msg_id: int, struct_format: str, field_names: List[str]):
        packed_id = struct.pack("<H", msg_id)
        length = struct.calcsize(struct_format)
        self.msg_info[packed_id] = MessageInfo(name=name, format=struct_format, length=length, field_names=field_names)
        if length < self.num_bytes_per_read:  # read minimum number of bytes each time
            self.num_bytes_per_read = length

    def read_message(self) -> Dict[str, Any]:
        try:
            self.bytes_in += self.client_socket.recv(self.num_bytes_per_read)
            while True:
                if len(self.bytes_in) >= 2:
                    packed_id = self.bytes_in[:2]
                    if packed_id in self.msg_info:
                        info = self.msg_info[packed_id]
                        if len(self.bytes_in) < info.length:
                            self.bytes_in += self.client_socket.recv(info.length - len(self.bytes_in))
                        else:
                            fields = struct.unpack(info.format, self.bytes_in[:info.length])
                            self.bytes_in = self.bytes_in[info.length:]
                            if len(fields) != len(info.field_names):
                                print("ERROR, field name mismatch", fields, info.field_names)
                                continue
                            return self.create_received_message(fields, info)
                    else:
                        print("Dropping unexpected byte not part of ID:", hex(self.bytes_in[0]))
                        self.bytes_in = self.bytes_in[1:] + self.client_socket.recv(self.num_bytes_per_read)
                else:  # if not received msg ID
                    self.bytes_in += self.client_socket.recv(self.num_bytes_per_read)
        except KeyboardInterrupt:
            sys.exit(0)
        except (TimeoutError, socket.timeout):
            # BUG: this doesn't happen
            self.connected_but_timeout = True
        else:
            self.connected_but_timeout = False

    def create_received_message(self, fields: Tuple[Any, ...], info: MessageInfo) -> Dict[str, Any]:
        out = {}
        for i in range(len(fields)):
            out[info.field_names[i]] = fields[i]
        self.messages[info.name] = out
        self.connected_but_timeout = False
        new_time = time()
        self.rx_loop_time = new_time - self.last_rx_time
        self.last_rx_time = new_time
        return out

    def blocking_connect(self, target_port: int):
        print("Connecting on", self.target_host, "::", target_port)
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(5)
        while True:
            try:
                client.connect((self.target_host, target_port))
            except (TimeoutError, socket.timeout):
                print("\rConnection attempt timed out...")
            except (ConnectionRefusedError, ConnectionAbortedError):
                print("Connection refused or aborted, is the server running?", end="\r")
                sleep(1)
            else:
                break
        self.client_socket = client


def generate_tables(receiver: BaseReceiver):
    def generate_table(name: str, message: Dict[str, Any]):
        table = Table(title=name)
        for key in message:
            table.add_column(key)
        table.add_row(*[str(value) for value in message.values()])
        return Align.center(table)
    text = (Text(f"Timeout {time() - receiver.last_rx_time:.6f} s", style="red") if receiver.connected_but_timeout
            else Text(f"Connected @ {receiver.rx_loop_time:.6f} s", style="green"))
    return Group(*([generate_table(n, m) for n, m in receiver.messages.items()] + [Align.center(text)]))


def main():
    settingsTree = ElementTree.parse("settings/settings.xml")
    settings = settingsTree.getroot()
    # state = RemoteRobotState()

    state_port = int(settings.find("General").find("state_send_port").text)
    receiver = BaseReceiver()
    receiver.register_message("IMU Info", 0x0C00, "<HHIIIIff",
                              ["id", 'errcode', 'timestamp_us', 'run_time_us', 'info_run_time_us',
                               'debug_int', 'debug_float', 'IMU_temp'])
    receiver.register_message("IMU Data", 0x0C02, "<HHIffffff",
                              ["id", 'errcode', 'timestamp_us', 'accel[0]', 'accel[1]', 'accel[2]',
                               'gyro[0]', 'gyro[1]', 'gyro[2]'])
    receiver.register_message("Controller Info", 0x0C22, "<IIIf",  # struct packing promotes ID to 4 bytes
                              ["id", 'timestamp_us', 'free_heap_bytes', 'processor_temp'])
    receiver.blocking_connect(state_port)

    with Live(Table()) as live:
        while True:
            message = receiver.read_message()
            if message:
                if RICH_DISPLAY:
                    live.update(generate_tables(receiver))
                else:
                    print(message, end='\r')
            elif receiver.connected_but_timeout:
                print("Timed out waiting for messages...", end='\r' if RICH_DISPLAY else '\n', flush=True)
            # if state.set_from_msg(receiver.get_state_msg()):
            #     print(state, end='\r')


if __name__ == '__main__':
    main()
