#!/usr/bin/env python3
"""Omni FALSO para trabajar sin el dispositivo.

Publica los mismos topics que sensable_phantom:
  /arm/measured_cp (PoseStamped), /arm/front (Joy), /arm/back (Joy)
y se suscribe a /arm/servo_cf (WrenchStamped) para mostrar la fuerza háptica.

NO lo lances a la vez que el sensable_phantom real.
"""
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import Joy
from PyQt5 import QtCore, QtWidgets


class MockOmni(Node):
    def __init__(self):
        super().__init__('mock_omni')
        self.pos = [0.0, 0.0, 0.0]
        self.front = 0
        self.back = 0
        self.force = [0.0, 0.0, 0.0]
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_p = self.create_publisher(PoseStamped, '/arm/measured_cp', qos)
        self.pub_f = self.create_publisher(Joy, '/arm/front', qos)
        self.pub_b = self.create_publisher(Joy, '/arm/back', qos)
        self.create_subscription(WrenchStamped, '/arm/servo_cf', self.on_force, 10)
        self.create_timer(0.01, self.tick)

    def on_force(self, m):
        f = m.wrench.force
        self.force = [f.x, f.y, f.z]

    def tick(self):
        now = self.get_clock().now().to_msg()
        p = PoseStamped()
        p.header.stamp = now
        p.pose.position.x, p.pose.position.y, p.pose.position.z = self.pos
        p.pose.orientation.w = 1.0
        self.pub_p.publish(p)
        for pub, val in ((self.pub_f, self.front), (self.pub_b, self.back)):
            j = Joy()
            j.header.stamp = now
            j.buttons = [int(val)]
            pub.publish(j)


class Panel(QtWidgets.QWidget):
    RANGE_MM = 100  # el stylus virtual se mueve +-100 mm

    def __init__(self, node):
        super().__init__()
        self.n = node
        self.setWindowTitle('Omni simulado')
        lay = QtWidgets.QVBoxLayout(self)
        self.sl = []
        for i, name in enumerate('XYZ'):
            row = QtWidgets.QHBoxLayout()
            s = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            s.setRange(-self.RANGE_MM, self.RANGE_MM)
            s.valueChanged.connect(lambda v, i=i: self.n.pos.__setitem__(i, v / 1000.0))
            row.addWidget(QtWidgets.QLabel(name))
            row.addWidget(s)
            lay.addLayout(row)
            self.sl.append(s)
        btn0 = QtWidgets.QPushButton('Centrar stylus')
        btn0.clicked.connect(lambda: [s.setValue(0) for s in self.sl])
        lay.addWidget(btn0)

        self.chk = QtWidgets.QCheckBox('Botón 1: EMBRAGUE (mantener activado)')
        self.chk.toggled.connect(lambda v: setattr(self.n, 'front', int(v)))
        lay.addWidget(self.chk)
        self.bb = QtWidgets.QPushButton('Botón 2: pinza (pulsar)')
        self.bb.pressed.connect(lambda: setattr(self.n, 'back', 1))
        self.bb.released.connect(lambda: setattr(self.n, 'back', 0))
        lay.addWidget(self.bb)

        self.lb = QtWidgets.QLabel('Fuerza háptica: 0 0 0')
        lay.addWidget(self.lb)
        t = QtCore.QTimer(self)
        t.timeout.connect(lambda: self.lb.setText(
            'Fuerza háptica (N): %+.2f  %+.2f  %+.2f' % tuple(self.n.force)))
        t.start(100)
        self._t = t


def main():
    rclpy.init()
    node = MockOmni()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    app = QtWidgets.QApplication(sys.argv)
    w = Panel(node)
    w.show()
    code = app.exec_()
    ex.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
