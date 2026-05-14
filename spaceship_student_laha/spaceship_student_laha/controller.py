#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from rclpy.node import Node

from spaceship_msgs.msg import ShipState
from spaceship_msgs.srv import SetMotorPower


class State(Enum):
    IDLE = 'IDLE'
    ORIENT = 'ORIENT'
    THRUST = 'THRUST'
    BRAKE = 'BRAKE'
    HOVER = 'HOVER'   # aproximación suave final
    ARRIVED = 'ARRIVED'


# Constantes físicas del simulador (para calcular distancia de frenado)
LINEAR_DRAG = 0.5
MAX_THRUST = 3.0


def angle_diff(a, b):
    """Diferencia angular normalizada en [-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class ShipController(Node):

    def __init__(self):
        super().__init__('ship_controller')

        # ── Parámetros configurables ──────────────────────────────────
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('max_power', 80)
        self.declare_parameter('turn_power', 40)

        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.max_power = int(self.get_parameter('max_power').value)
        self.turn_power = int(self.get_parameter('turn_power').value)

        # ── Estado interno ────────────────────────────────────────────
        self.state = State.IDLE
        self.ship: ShipState | None = None
        self._turn_dir = 0   # +1 izquierda, -1 derecha, 0 sin decidir

        # Umbral de orientación antes de empujar (radianes)
        self.ORIENT_THRESH = 0.25

        # ── Cliente del servicio (R2) ─────────────────────────────────
        self.motor_client = self.create_client(SetMotorPower, '/set_motor_power')

        # ── Suscripción al estado (R3) ────────────────────────────────
        self.create_subscription(ShipState, '/ship_state', self._on_ship_state, 10)

        # Bucle de control a 20 Hz (mismo ritmo que el simulador publica)
        self.create_timer(0.05, self._control_loop)
        # Log de estado cada 2 segundos
        self.create_timer(2.0, self._log_status)

        # Si los parámetros tienen un target real, arrancamos directo
        if self.target_x != 0.0 or self.target_y != 0.0:
            self.state = State.ORIENT
            self.get_logger().info(
                f'Target desde parámetros: ({self.target_x:.1f}, {self.target_y:.1f})')

    # ── Callback del topic /ship_state ────────────────────────────────

    def _on_ship_state(self, msg: ShipState):
        self.ship = msg

        # Si llegamos, paramos todo
        if msg.arrived and self.state not in (State.ARRIVED, State.IDLE):
            self.state = State.ARRIVED
            self._set_motor(0, 0)
            self.get_logger().info(
                f'¡LLEGADO! Tiempo: {msg.elapsed_time:.2f}s')
            return

        # Si no tenemos target y el simulador ya tiene uno, lo tomamos
        if self.state == State.IDLE and (msg.target_x != 0.0 or msg.target_y != 0.0):
            self.target_x = msg.target_x
            self.target_y = msg.target_y
            self._turn_dir = 0
            self.state = State.ORIENT
            self.get_logger().info(
                f'Target desde ship_state: ({self.target_x:.1f}, {self.target_y:.1f})')

    # ── Bucle de control ──────────────────────────────────────────────

    def _control_loop(self):
        if self.ship is None or self.state in (State.IDLE, State.ARRIVED):
            return

        s = self.ship
        dx = self.target_x - s.x
        dy = self.target_y - s.y
        dist = math.sqrt(dx * dx + dy * dy)
        speed = math.sqrt(s.vx ** 2 + s.vy ** 2)

        desired_heading = math.atan2(dy, dx)
        heading_error = angle_diff(desired_heading, s.heading)

        eff_decel = max(LINEAR_DRAG * speed + MAX_THRUST * 0.8, 1.0)
        braking_dist = (speed ** 2) / (2.0 * eff_decel) + speed * 3.0 + 5.0

        if self.state == State.ORIENT:
            self._do_orient(heading_error, dist)
        elif self.state == State.THRUST:
            self._do_thrust(heading_error, dist, braking_dist)
        elif self.state == State.BRAKE:
            self._do_brake(s, dist, speed)
        elif self.state == State.HOVER:
            self._do_hover(s, heading_error, dist, speed)

    # ── Estados ───────────────────────────────────────────────────────

    def _do_orient(self, heading_error: float, dist: float):
        """Gira la nave para apuntar al objetivo (control proporcional)."""
        if abs(heading_error) < self.ORIENT_THRESH:
            self._set_motor(0, 0)
            self._turn_dir = 0
            self.state = State.THRUST
            self.get_logger().info('ORIENT → THRUST')
            return

        # Potencia proporcional al error: más lejos del ángulo correcto → giro más fuerte
        # Mínimo 20 para garantizar movimiento, máximo turn_power
        turn = int(max(20, min(self.turn_power, abs(heading_error) * 30)))

        # Bloqueamos dirección solo cerca de ±180° (donde angle_diff oscila de signo).
        # En cualquier otro caso, seguimos el signo del error para frenar el overshoot.
        if self._turn_dir == 0 or abs(heading_error) < math.pi * 0.8:
            self._turn_dir = 1 if heading_error > 0 else -1

        if self._turn_dir > 0:   # girar izquierda (M1 > M2)
            self._set_motor(1, turn)
            self._set_motor(2, 0)
        else:                    # girar derecha (M2 > M1)
            self._set_motor(1, 0)
            self._set_motor(2, turn)

    def _do_thrust(self, heading_error: float, dist: float, braking_dist: float):
        """Acelera hacia el objetivo con corrección de heading."""
        if dist < braking_dist:
            self.state = State.BRAKE
            self.get_logger().info(f'THRUST → BRAKE  dist={dist:.1f}  brake_dist={braking_dist:.1f}')
            return

        # Si nos hemos desviado mucho, volvemos a orientar antes de empujar
        if abs(heading_error) > 0.6:
            self.state = State.ORIENT
            return

        # Corrección proporcional de heading mientras empujamos
        correction = int(min(self.turn_power, abs(heading_error) * 40))
        if heading_error > 0:  # girar izquierda: M1 > M2
            self._set_motor(1, self.max_power)
            self._set_motor(2, max(0, self.max_power - correction))
        else:                  # girar derecha: M2 > M1
            self._set_motor(1, max(0, self.max_power - correction))
            self._set_motor(2, self.max_power)

    def _do_brake(self, s: ShipState, dist: float, speed: float):
        """Frena apuntando en sentido contrario al movimiento y empujando."""
        if speed < 0.8:
            # Velocidad suficientemente baja: pasar a aproximación suave
            self._set_motor(0, 0)
            self._turn_dir = 0
            self.state = State.HOVER
            self.get_logger().info(f'BRAKE → HOVER  dist={dist:.1f}m')
            return

        # Dirección opuesta a la velocidad actual = dirección de frenado
        vel_dir = math.atan2(s.vy, s.vx)
        brake_heading = vel_dir + math.pi
        while brake_heading > math.pi:
            brake_heading -= 2 * math.pi

        brake_error = angle_diff(brake_heading, s.heading)

        if abs(brake_error) < 0.5:
            # Alineado a retrogrado: empujar + amortiguar omega residual
            omega_correction = int(min(20, abs(s.omega) * 25))
            if s.omega > 0:  # heading sube → contrarresta con M2 > M1
                self._set_motor(1, max(0, self.max_power - omega_correction))
                self._set_motor(2, self.max_power)
            else:
                self._set_motor(1, self.max_power)
                self._set_motor(2, max(0, self.max_power - omega_correction))
        else:
            # Girando a retrogrado: potencia baja para no acumular omega
            turn = int(max(10, min(20, abs(brake_error) * 12)))
            if brake_error > 0:  # girar izquierda: M1 > M2
                self._set_motor(1, turn)
                self._set_motor(2, 0)
            else:                # girar derecha: M2 > M1
                self._set_motor(1, 0)
                self._set_motor(2, turn)

    def _do_hover(self, s: ShipState, heading_error: float, dist: float, speed: float):
        """Aproximación final con control de velocidad proporcional (resistente a viento)."""
        if dist > 15.0:
            self._turn_dir = 0
            self.state = State.ORIENT
            self.get_logger().info('HOVER → ORIENT (demasiado lejos)')
            return

        dx = self.target_x - s.x
        dy = self.target_y - s.y

        if dist < 7.0:
            # Control de velocidad: desired_v apunta al target a velocidad segura.
            # El error de velocidad actúa de freno (si vamos rápido) y de empuje
            # (si el viento nos aleja), compensando perturbaciones automáticamente.
            approach_speed = min(0.18, dist * 0.06)
            desired_vx = approach_speed * dx / max(dist, 0.1)
            desired_vy = approach_speed * dy / max(dist, 0.1)

            err_vx = desired_vx - s.vx
            err_vy = desired_vy - s.vy
            err_mag = math.sqrt(err_vx ** 2 + err_vy ** 2)

            thrust_dir = math.atan2(err_vy, err_vx)
            thrust_error = angle_diff(thrust_dir, s.heading)

            power = int(min(70, max(15, err_mag * 50)))

            if abs(thrust_error) < 0.5:
                self._set_motor(0, power)
                self._turn_dir = 0
            else:
                if self._turn_dir == 0 or abs(thrust_error) < math.pi * 0.8:
                    self._turn_dir = 1 if thrust_error > 0 else -1
                turn = int(max(15, min(30, abs(thrust_error) * 20)))
                if self._turn_dir > 0:
                    self._set_motor(1, turn)
                    self._set_motor(2, 0)
                else:
                    self._set_motor(1, 0)
                    self._set_motor(2, turn)
        else:
            # Lejos: empuje directo hacia el target con potencia suficiente para el viento
            if abs(heading_error) > self.ORIENT_THRESH:
                if self._turn_dir == 0 or abs(heading_error) < math.pi * 0.8:
                    self._turn_dir = 1 if heading_error > 0 else -1
                turn = int(max(15, min(self.turn_power, abs(heading_error) * 25)))
                if self._turn_dir > 0:
                    self._set_motor(1, turn)
                    self._set_motor(2, 0)
                else:
                    self._set_motor(1, 0)
                    self._set_motor(2, turn)
            else:
                hover_power = int(max(40, min(70, dist * 8.0)))
                self._turn_dir = 0
                self._set_motor(0, hover_power)

    # ── Log periódico ─────────────────────────────────────────────────

    def _log_status(self):
        if self.ship is None:
            self.get_logger().warn('Esperando /ship_state... ¿está el simulador corriendo?')
            return
        s = self.ship
        dx = self.target_x - s.x
        dy = self.target_y - s.y
        dist = math.sqrt(dx * dx + dy * dy)
        speed = math.sqrt(s.vx ** 2 + s.vy ** 2)
        desired = math.atan2(self.target_y - s.y, self.target_x - s.x)
        herr = angle_diff(desired, s.heading)
        self.get_logger().info(
            f'[{self.state.value}]  '
            f'pos=({s.x:.1f},{s.y:.1f})  '
            f'dist={dist:.1f}m  spd={speed:.2f}m/s  '
            f'hdg={math.degrees(s.heading):.0f}°  '
            f'herr={math.degrees(herr):.0f}°  '
            f't={s.elapsed_time:.1f}s')

    # ── Llamada al servicio (R2) ──────────────────────────────────────

    def _set_motor(self, motor_id: int, power: int):
        """Envía un comando de motor al simulador vía servicio (asíncrono)."""
        if not self.motor_client.service_is_ready():
            return
        req = SetMotorPower.Request()
        req.motor_id = motor_id
        req.power = power
        self.motor_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = ShipController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
