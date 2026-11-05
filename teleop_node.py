#!/usr/bin/env python3
"""Nodo de teleoperación: Omni (CRTK /arm/*) -> Kinova Gen3 (Gazebo) + háptica.

Entradas:
  /arm/measured_cp (PoseStamped)  pose del stylus
  /arm/front (Joy)                botón 1: EMBRAGUE (mantener para mover)
  /arm/back  (Joy)                botón 2: alternar pinza abrir/cerrar
  /wrist_ft  (Wrench)             fuerza en la muñeca del robot
  /joint_states (JointState)      articulaciones del robot
Salidas:
  /tool_twist_vel (Twist)         velocidad del extremo (frame de la herramienta)
  /gripper_cmd (Float64)          0 = abierta, 1 = cerrada
  /arm/servo_cf (WrenchStamped)   fuerza háptica hacia el Omni
"""
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, Wrench, WrenchStamped, PoseStamped
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64
import tf2_ros

LIMIT_V = 0.2  # m/s, límite del controlador (cartesian_key_teleop.py)

BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)


class DelayLine:
    """Cola con marca de tiempo: devuelve los elementos tras 'delay' segundos."""

    def __init__(self):
        self.delay = 0.0
        self.buf = deque()

    def push(self, item):
        self.buf.append((time.monotonic(), item))

    def pop_latest_ready(self):
        """Devuelve el último elemento ya 'maduro' (o None si no hay)."""
        out = None
        now = time.monotonic()
        while self.buf and now - self.buf[0][0] >= self.delay:
            out = self.buf.popleft()[1]
        return out


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_gui')

        # ---------- Parámetros de topics (cambiables al lanzar) ----------
        self.declare_parameter('pose_topic', '/arm/measured_cp')
        self.declare_parameter('front_topic', '/arm/front')
        self.declare_parameter('back_topic', '/arm/back')
        self.declare_parameter('force_out_topic', '/arm/servo_cf')
        self.declare_parameter('ft_topic', '/wrist_ft')
        self.declare_parameter('twist_topic', '/tool_twist_vel')
        self.declare_parameter('gripper_topic', '/gripper_cmd')
        self.declare_parameter('joint_topic', '/joint_states')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'tool_frame')
        # Matriz 3x3 (por filas) Omni -> frame herramienta. AJUSTAR con el Omni real.
        self.declare_parameter('axis_map', [1.0, 0.0, 0.0,
                                            0.0, 1.0, 0.0,
                                            0.0, 0.0, 1.0])
        # +1 / -1: invertir si la fuerza "tira" en vez de "empujar"
        self.declare_parameter('force_sign', 1.0)

        g = lambda n: self.get_parameter(n).value
        self.axis_map = np.array(g('axis_map'), dtype=float).reshape(3, 3)
        self.force_sign = float(g('force_sign'))
        self.base_frame = g('base_frame')
        self.ee_frame = g('ee_frame')

        # ---------- Parámetros ajustables desde la GUI ----------
        self.v_max = 0.08          # m/s
        self.k_lin = 2.0           # (m/s)/m  ganancia desplazamiento->velocidad
        self.deadzone = 0.004      # m, zona muerta del stylus
        self.delay_cmd = 0.0       # s, retardo Omni -> robot
        self.delay_force = 0.0     # s, retardo robot -> Omni
        self.comp = False          # compensación básica de retardo
        self.haptics_on = True
        self.k_force = 1.0         # ganancia de fuerza
        self.f_sat = 2.5           # N, saturación máxima hacia el Omni
        self.f_dead = 0.3          # N, zona muerta de fuerza (tras tara)
        self.lp_alpha = 0.15       # filtro paso bajo (0..1, menor = más suave)
        self.estop = False

        # ---------- Estado ----------
        self.omni_pos = None
        self.omni_ref = None
        self.omni_t = 0.0
        self.clutch = False
        self.back_prev = False
        self.grip_target = 0.0
        self.grip = 0.0
        self.ft_raw = np.zeros(3)
        self.ft_bias = np.zeros(3)
        self.ft_filt = np.zeros(3)
        self.ft_t = 0.0
        self.tare_req = False
        self.joint_names = []
        self.joint_pos = []
        self.joint_t = 0.0
        self.ee_pos = None
        self.twist_sent = np.zeros(3)      # lo último publicado (tras retardo)
        self.force_sent = np.zeros(3)      # lo último enviado al Omni (tras retardo)
        self.force_cmd = np.zeros(3)       # fuerza calculada (antes del retardo)

        self.cmd_line = DelayLine()
        self.force_line = DelayLine()
        self._last_twist = np.zeros(3)
        self._last_force = np.zeros(3)

        # ---------- ROS ----------
        self.create_subscription(PoseStamped, g('pose_topic'), self.on_pose, BEST_EFFORT)
        self.create_subscription(Joy, g('front_topic'), self.on_front, BEST_EFFORT)
        self.create_subscription(Joy, g('back_topic'), self.on_back, BEST_EFFORT)
        self.create_subscription(Wrench, g('ft_topic'), self.on_ft, BEST_EFFORT)
        self.create_subscription(JointState, g('joint_topic'), self.on_js, BEST_EFFORT)

        self.pub_twist = self.create_publisher(Twist, g('twist_topic'), 10)
        self.pub_grip = self.create_publisher(Float64, g('gripper_topic'), 10)
        self.pub_force = self.create_publisher(WrenchStamped, g('force_out_topic'), 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(1 / 20.0, self.control_step)
        self.create_timer(1 / 100.0, self.haptic_step)
        self.create_timer(0.2, self.update_ee)

    # ------------------------------------------------------------ callbacks
    def on_pose(self, m):
        p = m.pose.position
        self.omni_pos = np.array([p.x, p.y, p.z])
        self.omni_t = time.monotonic()

    def on_front(self, m):
        pressed = bool(m.buttons and m.buttons[0])
        if pressed and not self.clutch:
            self.omni_ref = None if self.omni_pos is None else self.omni_pos.copy()
        if not pressed:
            self.omni_ref = None
        self.clutch = pressed

    def on_back(self, m):
        pressed = bool(m.buttons and m.buttons[0])
        if pressed and not self.back_prev:
            self.grip_target = 0.0 if self.grip_target > 0.5 else 1.0
        self.back_prev = pressed

    def on_ft(self, m):
        self.ft_raw = np.array([m.force.x, m.force.y, m.force.z])
        if self.tare_req:
            self.ft_bias = self.ft_raw.copy()
            self.ft_filt[:] = 0.0
            self.tare_req = False
        x = self.ft_raw - self.ft_bias
        self.ft_filt = (1 - self.lp_alpha) * self.ft_filt + self.lp_alpha * x
        self.ft_t = time.monotonic()

    def on_js(self, m):
        if m.position:  # algunos mensajes llegan vacíos
            self.joint_names = list(m.name)
            self.joint_pos = list(m.position)
            self.joint_t = time.monotonic()

    def update_ee(self):
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame,
                                                rclpy.time.Time())
            v = t.transform.translation
            self.ee_pos = np.array([v.x, v.y, v.z])
        except Exception:
            self.ee_pos = None

    # ------------------------------------------------------- lazo de control
    def total_delay(self):
        return self.delay_cmd + self.delay_force

    def control_step(self):
        # 1) velocidad deseada (sin retardo)
        v = np.zeros(3)
        if (not self.estop and self.clutch and self.omni_pos is not None
                and self.omni_ref is not None):
            d = self.omni_pos - self.omni_ref
            d[np.abs(d) < self.deadzone] = 0.0
            v = self.axis_map @ (self.k_lin * d)
            if self.comp:  # compensación básica: menos ganancia con más retardo
                v = v / (1.0 + 2.0 * self.total_delay())
            n = np.linalg.norm(v)
            vmax = min(self.v_max, LIMIT_V)
            if n > vmax:
                v *= vmax / n

        # 2) retardo en el canal de comandos
        self.cmd_line.delay = self.delay_cmd
        self.cmd_line.push(v)
        ready = self.cmd_line.pop_latest_ready()
        if ready is not None:
            self._last_twist = ready
        out = np.zeros(3) if self.estop else self._last_twist
        self.twist_sent = out

        tw = Twist()
        tw.linear.x, tw.linear.y, tw.linear.z = map(float, out)
        self.pub_twist.publish(tw)

        # 3) pinza (rampa suave)
        dt = 1 / 20.0
        step = dt / 1.0
        if self.grip < self.grip_target:
            self.grip = min(self.grip_target, self.grip + step)
        elif self.grip > self.grip_target:
            self.grip = max(self.grip_target, self.grip - step)
        g = Float64()
        g.data = float(self.grip)
        self.pub_grip.publish(g)

    # ------------------------------------------------------ lazo de háptica
    def haptic_step(self):
        f = np.zeros(3)
        fresh = (time.monotonic() - self.ft_t) < 0.5
        if self.haptics_on and fresh and not self.estop:
            raw = self.ft_filt.copy()
            if np.linalg.norm(raw) < self.f_dead:
                raw[:] = 0.0
            k = self.k_force
            if self.comp:
                k = k / (1.0 + 3.0 * self.total_delay())
            f = self.force_sign * k * (self.axis_map.T @ raw)
            n = np.linalg.norm(f)
            if n > self.f_sat:
                f *= self.f_sat / n
        self.force_cmd = f

        self.force_line.delay = self.delay_force
        self.force_line.push(f)
        ready = self.force_line.pop_latest_ready()
        if ready is not None:
            self._last_force = ready
        out = self._last_force if (self.haptics_on and not self.estop) else np.zeros(3)
        if not fresh:
            out = np.zeros(3)
        self.force_sent = out

        w = WrenchStamped()
        w.header.stamp = self.get_clock().now().to_msg()
        w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = map(float, out)
        self.pub_force.publish(w)

    def stop_all(self):
        """Envía ceros antes de salir."""
        try:
            self.pub_twist.publish(Twist())
            w = WrenchStamped()
            self.pub_force.publish(w)
        except Exception:
            pass
