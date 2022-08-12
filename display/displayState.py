""" Display the state of simplewalker on desktop base computer
July 2022
TODO:
"""
import numpy as np
from xml.etree import ElementTree
import vpython as v
import remoteState
from typing import List


class DisplayRobot:
    def __init__(self, state: remoteState.RemoteRobotState):
        self.state = state
        self.x_axis = v.vector(1, 0, 0)
        self.y_axis = v.vector(0, 1, 0)
        self.z_axis = v.vector(0, 0, 1)
        self.x = v.arrow(axis=self.x_axis)
        self.y = v.arrow(axis=self.y_axis)
        self.z = v.arrow(axis=self.z_axis)

    def rotate(self, body_vector: v.vector):
        angle = np.linalg.norm(self.state.axis)
        if angle < 0.01:
            unit_axis = v.vector(1, 0, 0)
        else:
            unit_axis = v.vector(self.state.axis[0], self.state.axis[1], self.state.axis[2]) / angle
        return v.cos(angle) * body_vector + v.sin(angle) * v.cross(unit_axis, body_vector) \
               + (1 - v.cos(angle)) * v.dot(unit_axis, body_vector) * unit_axis

    def update(self):
        # self.x.pos = v.vector(state.pos)
        # self.y.pos = v.vector(state.pos)
        # self.z.pos = v.vector(state.pos)
        self.x.axis = v.vector(self.rotate(self.x_axis))
        self.y.axis = v.vector(self.rotate(self.y_axis))
        self.z.axis = v.vector(self.rotate(self.z_axis))


def main():
    settingsTree = ElementTree.parse("settings/settings.xml")
    settings = settingsTree.getroot()
    state = remoteState.RemoteRobotState()
    receiver = remoteState.BaseReceiver()
    receiver.connect_state(int(settings.find("General").find("state_send_port").text))
    displayRobot = DisplayRobot(state)

    while True:
        if state.set_from_msg(receiver.get_state_msg()):
            displayRobot.update()


if __name__ == '__main__':
    main()
