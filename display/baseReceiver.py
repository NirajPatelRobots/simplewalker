""" Print the state of simplewalker on desktop base computer. Receives from simplewalker over wifi.
June 2022
TODO:
    set from log?
    way to automatically deal with arrays
    better message struct input:
        parse field_names and struct_format from struct definition string
        parse messages.h?
"""

import socket
from xml.etree import ElementTree
import dataclasses
from typing import List, Dict, Any
import struct
import sys
from time import sleep
try:
    from rich.live import Live
    from rich.table import Table
    from rich.console import Group
    from rich.panel import Panel
    from rich.align import Align
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


class BaseReceiver:
    """ Connects to simplewalker and receives information"""
    def __init__(self):
        if "--local" in sys.argv:
            self.target_host = socket.gethostbyname("localhost")
        else:
            self.target_host = socket.gethostbyname("raspberrypi")
        self.state_client_socket = None
        self.num_bytes_per_read = RemoteRobotState.num_bytes
        self.formats = {}
        self.msg_lengths = {}
        self.field_names = {}
        self.msg_names = {}
        self.messages = {}

    def get_state_msg(self) -> bytes:
        return self.state_client_socket.recv(RemoteRobotState.num_bytes)

    def register_message(self, name: str, msg_id: int, struct_format: str, field_names: List[str]):
        packed_id = struct.pack("<H", msg_id)
        self.msg_names[packed_id] = name
        self.formats[packed_id] = struct_format
        msg_length = struct.calcsize(struct_format)
        self.msg_lengths[packed_id] = msg_length
        if msg_length < self.num_bytes_per_read:  # read minimum number of bytes each time
            self.num_bytes_per_read = msg_length
        self.field_names[packed_id] = field_names

    def read_message(self) -> Dict[str, Any]:
        bytes_in = self.state_client_socket.recv(self.num_bytes_per_read)
        while True:
            if len(bytes_in) >= 2:
                packed_id = bytes_in[:2]
                if packed_id in self.msg_lengths:
                    if len(bytes_in) < self.msg_lengths[packed_id]:
                        bytes_in += self.state_client_socket.recv(size=self.msg_lengths[packed_id] - len(bytes_in))
                    else:
                        fields = struct.unpack(self.formats[packed_id], bytes_in[:self.msg_lengths[packed_id] + 1])
                        if len(fields) != len(self.field_names[packed_id]):
                            print("ERROR, field name mismatch", fields, self.field_names[packed_id])
                            continue
                        out = {}
                        for i in range(len(fields)):
                            out[self.field_names[packed_id][i]] = fields[i]
                        self.messages[self.msg_names[packed_id]] = out
                        return out
                else:
                    print("Dropping unexpected byte not part of ID:", hex(bytes_in[0]))
                    bytes_in = bytes_in[1:] + self.state_client_socket.recv(self.num_bytes_per_read)
            else:  # if not received msg ID
                bytes_in += self.state_client_socket.recv(self.num_bytes_per_read)

    def blocking_connect(self, target_port: int):
        print("Connecting on", self.target_host, "::", target_port)
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(5)
        while True:
            try:
                client.connect((self.target_host, target_port))
            except TimeoutError:
                print("\rConnection attempt timed out...", end="")
            except ConnectionRefusedError:
                print("Connection refused by host, is Server running?", end="\r")
                sleep(1)
            except ConnectionAbortedError:
                print("Connection aborted, is Server running?         ", end="\r")
                sleep(1)
            else:
                break
        self.state_client_socket = client


def generate_tables(messages: Dict[str, Dict[str, Any]]):
    def generate_table(name: str, message: Dict[str, Any]):
        table = Table(title=name)
        for key in message:
            table.add_column(key)
        table.add_row(*[str(value) for value in message.values()])
        return Align.center(table)
    return Group(*[generate_table(n, m) for n, m in messages.items()])


def main():
    settingsTree = ElementTree.parse("settings/settings.xml")
    settings = settingsTree.getroot()
    state = RemoteRobotState()

    state_port = int(settings.find("General").find("state_send_port").text)
    receiver = BaseReceiver()
    receiver.register_message("IMU Info", 0x0C00, "<HHIIIIIff",
                              ["id", 'errcode', 'timestamp_us', 'free_heap_bytes', 'run_time_us', 'info_run_time_us',
                               'debug_int', 'IMU_temp', 'debug_float'])
    receiver.register_message("IMU Data", 0x0C02, "<HHIffffff",
                              ["id", 'errcode', 'timestamp_us', 'accel[0]', 'accel[1]', 'accel[2]',
                               'gyro[0]', 'gyro[1]', 'gyro[2]'])
    receiver.blocking_connect(state_port)

    with Live(Table()) as live:
        while True:
            try:
                message = receiver.read_message()
            except KeyboardInterrupt:
                sys.exit(0)
            if message:
                if RICH_DISPLAY:
                    live.update(generate_tables(receiver.messages))
                else:
                    print(message, end='\r')
            # if state.set_from_msg(receiver.get_state_msg()):
            #     print(state, end='\r')


if __name__ == '__main__':
    main()
