#!/usr/bin/env python3
"""Interfaz de teleoperación (PyQt5) - Práctica 4.

Uso (dentro del contenedor, con Gazebo y el Omni/mock_omni en marcha):
    python3 teleop_gui.py
"""
import sys
import time
import threading
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from PyQt5 import QtCore, QtGui, QtWidgets

from teleop_node import TeleopNode


# ---------------------------------------------------------------- gráfica
class LivePlot(QtWidgets.QWidget):
    COLORS = [QtGui.QColor(230, 60, 60), QtGui.QColor(60, 190, 80), QtGui.QColor(70, 120, 240)]

    def __init__(self, title, labels=('x', 'y', 'z'), unit='N', maxlen=300, min_range=1.0):
        super().__init__()
        self.title, self.labels, self.unit = title, labels, unit
        self.min_range = min_range
        self.bufs = [deque(maxlen=maxlen) for _ in labels]
        self.setMinimumHeight(150)

    def push(self, vals):
        for b, v in zip(self.bufs, vals):
            b.append(float(v))
        self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        r = self.rect().adjusted(40, 22, -8, -8)
        p.fillRect(self.rect(), QtGui.QColor(30, 30, 34))
        p.setPen(QtGui.QColor(220, 220, 220))
        p.drawText(8, 15, f'{self.title} [{self.unit}]')
        peak = max([self.min_range] + [max(map(abs, b)) for b in self.bufs if b])
        peak *= 1.1
        # ejes
        p.setPen(QtGui.QPen(QtGui.QColor(80, 80, 90), 1))
        p.drawRect(r)
        yz = r.center().y()
        p.drawLine(r.left(), yz, r.right(), yz)
        p.setPen(QtGui.QColor(170, 170, 170))
        p.drawText(2, r.top() + 10, f'{peak:+.1f}')
        p.drawText(2, r.bottom(), f'{-peak:+.1f}')
        # curvas
        for i, b in enumerate(self.bufs):
            if len(b) < 2:
                continue
            p.setPen(QtGui.QPen(self.COLORS[i % 3], 1.6))
            n = b.maxlen
            pts = []
            off = n - len(b)
            for k, v in enumerate(b):
                x = r.left() + r.width() * (k + off) / (n - 1)
                y = yz - (v / peak) * (r.height() / 2)
                pts.append(QtCore.QPointF(x, y))
            p.drawPolyline(QtGui.QPolygonF(pts))
        # leyenda
        x = r.right() - 120
        for i, l in enumerate(self.labels):
            p.setPen(self.COLORS[i % 3])
            p.drawText(x + 40 * i, 15, l)


class Led(QtWidgets.QLabel):
    def set(self, ok, text):
        c = '#2ecc71' if ok else '#e74c3c'
        self.setText(f'<span style="color:{c}">●</span> {text}')


# ---------------------------------------------------------------- ventana
class Window(QtWidgets.QWidget):
    def __init__(self, node):
        super().__init__()
        self.n = node
        self.setWindowTitle('Teleoperación Kinova Gen3 + Phantom Omni')
        self.resize(1250, 780)
        root = QtWidgets.QHBoxLayout(self)

        # ---- columna izquierda: estado y posiciones
        left = QtWidgets.QVBoxLayout()
        root.addLayout(left, 3)

        box = QtWidgets.QGroupBox('Conexión')
        l = QtWidgets.QVBoxLayout(box)
        self.led_omni = Led(); self.led_ft = Led(); self.led_js = Led()
        for w in (self.led_omni, self.led_ft, self.led_js):
            l.addWidget(w)
        left.addWidget(box)

        box = QtWidgets.QGroupBox('Posición (m)')
        g = QtWidgets.QFormLayout(box)
        self.lb_omni = QtWidgets.QLabel('-')
        self.lb_ee = QtWidgets.QLabel('-')
        g.addRow('Omni (local):', self.lb_omni)
        g.addRow('Extremo robot (remoto):', self.lb_ee)
        left.addWidget(box)

        box = QtWidgets.QGroupBox('Articulaciones del robot (grados)')
        g = QtWidgets.QGridLayout(box)
        self.jbars, self.jlabels = [], []
        for i in range(7):
            lab = QtWidgets.QLabel(f'J{i + 1}')
            bar = QtWidgets.QProgressBar()
            bar.setRange(-180, 180)
            bar.setFormat('%v°')
            self.jbars.append(bar)
            g.addWidget(lab, i, 0)
            g.addWidget(bar, i, 1)
        left.addWidget(box)

        box = QtWidgets.QGroupBox('Pinza')
        l = QtWidgets.QVBoxLayout(box)
        self.grip_bar = QtWidgets.QProgressBar()
        self.grip_bar.setRange(0, 100)
        self.grip_bar.setFormat('cerrada %p %')
        l.addWidget(self.grip_bar)
        left.addWidget(box)
        left.addStretch(1)

        # ---- columna central: gráficas
        mid = QtWidgets.QVBoxLayout()
        root.addLayout(mid, 5)
        self.plot_ft = LivePlot('Fuerza en la muñeca (filtrada, tarada)', unit='N', min_range=1.0)
        self.plot_hap = LivePlot('Fuerza enviada al Omni', unit='N', min_range=1.0)
        self.plot_vel = LivePlot('Velocidad enviada al robot', unit='m/s', min_range=0.05)
        for p in (self.plot_ft, self.plot_hap, self.plot_vel):
            mid.addWidget(p)
        self.lb_clutch = QtWidgets.QLabel()
        self.lb_clutch.setAlignment(QtCore.Qt.AlignCenter)
        self.lb_clutch.setStyleSheet('font-size:18px; font-weight:bold; padding:6px;')
        mid.addWidget(self.lb_clutch)

        # ---- columna derecha: controles
        right = QtWidgets.QVBoxLayout()
        root.addLayout(right, 3)

        self.btn_stop = QtWidgets.QPushButton('PARADA DE EMERGENCIA')
        self.btn_stop.setCheckable(True)
        self.btn_stop.setMinimumHeight(48)
        self.btn_stop.setStyleSheet('QPushButton{background:#c0392b;color:white;font-weight:bold;}'
                                    'QPushButton:checked{background:#7f8c8d;}')
        self.btn_stop.toggled.connect(self.on_stop)
        right.addWidget(self.btn_stop)

        box = QtWidgets.QGroupBox('Movimiento')
        g = QtWidgets.QFormLayout(box)
        self.s_vmax = self.slider(1, 20, int(node.v_max * 100), lambda v: setattr(node, 'v_max', v / 100))
        self.s_k = self.slider(5, 100, int(node.k_lin * 10), lambda v: setattr(node, 'k_lin', v / 10))
        g.addRow('Vel. máx (cm/s):', self.s_vmax[0])
        g.addRow('Ganancia (1/s):', self.s_k[0])
        right.addWidget(box)

        box = QtWidgets.QGroupBox('Háptica')
        g = QtWidgets.QFormLayout(box)
        self.chk_h = QtWidgets.QCheckBox('Activar retroalimentación')
        self.chk_h.setChecked(node.haptics_on)
        self.chk_h.toggled.connect(lambda v: setattr(node, 'haptics_on', v))
        self.s_kf = self.slider(0, 300, int(node.k_force * 100), lambda v: setattr(node, 'k_force', v / 100))
        self.s_fs = self.slider(5, 33, int(node.f_sat * 10), lambda v: setattr(node, 'f_sat', v / 10))
        self.s_lp = self.slider(2, 100, int(node.lp_alpha * 100), lambda v: setattr(node, 'lp_alpha', v / 100))
        btn_tare = QtWidgets.QPushButton('Tarar sensor (quitar sesgo)')
        btn_tare.clicked.connect(lambda: setattr(node, 'tare_req', True))
        g.addRow(self.chk_h)
        g.addRow('Ganancia fuerza:', self.s_kf[0])
        g.addRow('Saturación (N):', self.s_fs[0])
        g.addRow('Suavizado filtro:', self.s_lp[0])
        g.addRow(btn_tare)
        right.addWidget(box)

        box = QtWidgets.QGroupBox('Retardo de comunicación')
        g = QtWidgets.QFormLayout(box)
        self.s_dc = self.slider(0, 1000, 0, lambda v: setattr(node, 'delay_cmd', v / 1000))
        self.s_df = self.slider(0, 1000, 0, lambda v: setattr(node, 'delay_force', v / 1000))
        self.chk_c = QtWidgets.QCheckBox('Compensar retardo (reduce ganancias)')
        self.chk_c.toggled.connect(lambda v: setattr(node, 'comp', v))
        g.addRow('Omni → robot (ms):', self.s_dc[0])
        g.addRow('Robot → Omni (ms):', self.s_df[0])
        g.addRow(self.chk_c)
        right.addWidget(box)

        box = QtWidgets.QGroupBox('Ayuda')
        l = QtWidgets.QVBoxLayout(box)
        h = QtWidgets.QLabel('Botón 1 del stylus (mantener): mueve el robot.\n'
                             'Botón 2: abre/cierra la pinza.\n'
                             'Al soltar el botón 1 el robot se para y se\n'
                             'reancla la referencia.')
        l.addWidget(h)
        right.addWidget(box)
        right.addStretch(1)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(33)

    def slider(self, lo, hi, val, cb):
        s = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        s.setRange(lo, hi)
        s.setValue(val)
        lab = QtWidgets.QLabel(str(val))
        lab.setMinimumWidth(40)
        s.valueChanged.connect(lambda v: (lab.setText(str(v)), cb(v)))
        row = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(row)
        h.setContentsMargins(0, 0, 0, 0)
        h.addWidget(s)
        h.addWidget(lab)
        return row, s

    def on_stop(self, on):
        self.n.estop = on
        self.btn_stop.setText('EMERGENCIA ACTIVA (pulsa para rearmar)' if on else 'PARADA DE EMERGENCIA')

    def refresh(self):
        n, now = self.n, time.monotonic()
        self.led_omni.set(now - n.omni_t < 0.5, 'Omni')
        self.led_ft.set(now - n.ft_t < 0.5, 'Sensor de fuerza (/wrist_ft)')
        self.led_js.set(now - n.joint_t < 0.5, 'Articulaciones (/joint_states)')

        fmt = lambda v: '-' if v is None else f'{v[0]:+.3f}  {v[1]:+.3f}  {v[2]:+.3f}'
        self.lb_omni.setText(fmt(n.omni_pos))
        self.lb_ee.setText(fmt(n.ee_pos))

        for i, b in enumerate(self.jbars):
            if i < len(n.joint_pos):
                deg = int(np.degrees(n.joint_pos[i]))
                b.setValue(max(-180, min(180, deg)))
        self.grip_bar.setValue(int(n.grip * 100))

        self.plot_ft.push(n.ft_filt)
        self.plot_hap.push(n.force_sent)
        self.plot_vel.push(n.twist_sent)

        if n.estop:
            txt, col = 'PARADA DE EMERGENCIA', '#7f8c8d'
        elif n.clutch:
            txt, col = 'EMBRAGUE PULSADO: MOVIENDO', '#27ae60'
        else:
            txt, col = 'Embrague suelto (robot parado)', '#c0392b'
        self.lb_clutch.setText(txt)
        self.lb_clutch.setStyleSheet(f'font-size:18px;font-weight:bold;padding:6px;color:white;background:{col};')


def main():
    rclpy.init()
    node = TeleopNode()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    app = QtWidgets.QApplication(sys.argv)
    w = Window(node)
    w.show()
    code = app.exec_()

    node.stop_all()
    ex.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
