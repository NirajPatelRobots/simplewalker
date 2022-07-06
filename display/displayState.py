""" Display the state of simplewalker on desktop base computer. Receives from simplewalker over wifi.
June 2022
TODO:
"""

import socket
from xml.etree import ElementTree
import dataclasses
from typing import List
import struct


@dataclasses.dataclass
class RobotState:
    msgID: int = 0
    errcode: int = 0
    timestamp_us: int = 0
    pos: List[float] = dataclasses.field(default_factory = lambda: [0., 0., 0.])
    axis: List[float] = dataclasses.field(default_factory = lambda: [0., 0., 0.])
    vel: List[float] = dataclasses.field(default_factory = lambda: [0., 0., 0.])
    angvel: List[float] = dataclasses.field(default_factory = lambda: [0., 0., 0.])
    right_leg_pos: List[float] = dataclasses.field(default_factory = lambda: [0., 0., 0.])
    left_leg_pos: List[float] = dataclasses.field(default_factory = lambda: [0., 0., 0.])
    right_leg_vel: List[float] = dataclasses.field(default_factory = lambda: [0., 0., 0.])
    left_leg_vel: List[float] = dataclasses.field(default_factory = lambda: [0., 0., 0.])
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
                index +=1
            setattr(self, elementname, element)
        return True
    
    def __repr__(self) -> str:
        return "pos:" + str(self.pos) + " axis:" + str(self.axis) \
                + " vel:" + str(self.vel) + " angvel:" + str(self.angvel)


def try_connect(target_host: str, target_port:int) -> socket.socket:
    print("Connecting on", target_host, "::", target_port)
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.settimeout(5)
    while True:
        try:
            client.connect((target_host, target_port))
        except TimeoutError:
            print("\rConnection attempt timed out...", end="")
        except ConnectionRefusedError:
            print("\rConnection refused by host...", end="")
        else:
            break
    return client

        
def main():
    settingsTree = ElementTree.parse("settings/settings.xml")
    settings = settingsTree.getroot()
    state = RobotState()

    target_host = socket.gethostbyname("raspberrypi")
    target_port = int(settings.find("General").find("state_send_port").text)
    client = try_connect(target_host, target_port)

    while True:
        state_msg_bytes = client.recv(state.num_bytes)
        if state.set_from_msg(state_msg_bytes):
            print(state, end='\r')


if __name__ == '__main__':
    main()
