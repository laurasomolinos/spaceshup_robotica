#!/usr/bin/env python3
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import math
from enum import Enum

import rclpy
from rclpy.node import Node

from spaceship_msgs.msg import ShipState
from spaceship_msgs.srv import SetMotorPower
from spaceship_student_laha_msgs.msg import ControlDebug


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
        
        # ── Publisher de depuración del controlador (R4) ───────────────────
        self.debug_pub = self.create_publisher(ControlDebug, '/control_debug', 10)
        #-
        self.marker_pub = self.create_publisher(MarkerArray, '/controller_markers', 10)

        # Últimos valores calculados por el controlador
        self._last_desired_heading = 0.0
        self._last_heading_error = 0.0
        self._last_acc_mag = 0.0
        self._last_power = 0
        self._last_m1 = 0
        self._last_m2 = 0
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

        # Tomamos el target del simulador siempre que exista.
        # Si cambia por un nuevo click en RViz, actualizamos el objetivo.
        if msg.target_x != 0.0 or msg.target_y != 0.0:
            target_changed = (
                abs(msg.target_x - self.target_x) > 0.01 or
                abs(msg.target_y - self.target_y) > 0.01
            )

            if self.state == State.IDLE or target_changed:
                self.target_x = msg.target_x
                self.target_y = msg.target_y
                self._turn_dir = 0
                self.state = State.THRUST
                self.get_logger().info(
                    f'Target actualizado desde ship_state: ({self.target_x:.1f}, {self.target_y:.1f})'
                )
    # ── Bucle de control ──────────────────────────────────────────────

    def _control_loop(self):
        if self.ship is None or self.state in (State.IDLE, State.ARRIVED):
            return

        s = self.ship

        if s.arrived:
            self._set_motor(0, 0)
            self.state = State.ARRIVED
            self.get_logger().info(f'¡LLEGADO! Tiempo: {s.elapsed_time:.2f}s')
            return

        self._simple_control(s)

    # ── Estados ───────────────────────────────────────────────────────


    def _simple_control(self, s: ShipState):
        """
        Control PD en posición + velocidad.

        En vez de apuntar siempre al target, calcula una aceleración deseada:
            a_cmd = Kp * error_posicion - Kd * velocidad

        Así, si la nave va muy rápido hacia el target, la aceleración deseada
        apunta hacia atrás y la nave frena antes de pasarse.
        """

        dx = self.target_x - s.x
        dy = self.target_y - s.y

        dist = math.sqrt(dx * dx + dy * dy)
        speed = math.sqrt(s.vx * s.vx + s.vy * s.vy)

        # Condición de llegada del simulador: dist < 2.0 y speed < 0.1.
        # Si estamos casi ahí, paramos motores y dejamos que el simulador marque arrived.
        if dist < 1.9 and speed < 0.10:
            self._set_motor(0, 0)
            self.state = State.HOVER
            return

        # Ganancias simples. Si oscila mucho, baja KP. Si se pasa de largo, sube KD.
        KP = 0.3
        KD = 1.45

        # Aceleración deseada en coordenadas mundo.
        ax_cmd = KP * dx - KD * s.vx
        ay_cmd = KP * dy - KD * s.vy

        acc_mag = math.sqrt(ax_cmd * ax_cmd + ay_cmd * ay_cmd)

        # Si la corrección es muy pequeña, no empujamos.
        if acc_mag < 0.08:
            self._set_motor(0, 0)
            self.state = State.HOVER
            return

        desired_heading = math.atan2(ay_cmd, ax_cmd)
        self._last_desired_heading = desired_heading
        self._last_acc_mag = acc_mag

        # Convertimos aceleración deseada a potencia.
        # MAX_THRUST = 3.0, así que acc/MAX_THRUST * 100 da potencia aproximada.
        power = int((acc_mag / MAX_THRUST) * 100.0)

        # Limitamos potencia para que no vaya como un misil.
        power = max(12, min(self.max_power, power))
        self._last_power = power

        # Fase solo para logs.
        if dist < 5.0 and speed > 0.20:
            self.state = State.BRAKE
        elif dist < 3.0:
            self.state = State.HOVER
        else:
            self.state = State.THRUST

        self._drive_towards_angle(
            current_heading=s.heading,
            desired_heading=desired_heading,
            power=power,
            turn_power=self.turn_power
        )


    def _drive_towards_angle(self, current_heading: float, desired_heading: float,
                            power: int, turn_power: int):
        """
        Apunta la nave hacia desired_heading y aplica potencia cuando está alineada.

        Según la física:
            torque = (F1 - F2) * ARM_LENGTH

        Por tanto:
            M1 > M2 aumenta heading
            M2 > M1 disminuye heading
        """

        error = angle_diff(desired_heading, current_heading)

        # Si está muy desalineada, giramos casi en el sitio con potencia moderada.
        # No metemos mucho giro para no añadir demasiada velocidad lineal accidental.
        if abs(error) > 0.45:
            turn = int(max(10, min(turn_power, abs(error) * 26)))
            if error > 0:
                self._last_m1 = int(turn)
                self._last_m2 = 0
                self._set_motor(1, turn)
                self._set_motor(2, 0)
            else:
                self._last_m1 = 0
                self._last_m2 = int(turn)
                self._set_motor(1, 0)
                self._set_motor(2, turn)

            self._last_heading_error = error
            self._publish_debug()
            self._publish_markers()
            return

        # Si está alineada, empujamos con corrección diferencial.
        correction = int(min(turn_power, abs(error) * 35))

        if error > 0:
            # Girar suavemente a izquierda: M1 algo mayor
            m1 = power
            m2 = max(0, power - correction)
        else:
            # Girar suavemente a derecha: M2 algo mayor
            m1 = max(0, power - correction)
            m2 = power

        self._last_m1 = int(m1)
        self._last_m2 = int(m2)
        self._last_heading_error = error

        self._set_motor(1, int(m1))
        self._set_motor(2, int(m2))

        self._publish_debug()
        self._publish_markers()

    def _publish_debug(self):
        """Publica información interna del controlador en /control_debug."""
        if self.ship is None:
            return

        s = self.ship

        dx = self.target_x - s.x
        dy = self.target_y - s.y

        dist = math.sqrt(dx * dx + dy * dy)
        speed = math.sqrt(s.vx * s.vx + s.vy * s.vy)

        msg = ControlDebug()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        msg.phase = self.state.value

        msg.target_x = float(self.target_x)
        msg.target_y = float(self.target_y)

        msg.distance_to_target = float(dist)
        msg.speed = float(speed)

        msg.desired_heading = float(self._last_desired_heading)
        msg.heading_error = float(self._last_heading_error)

        msg.commanded_acceleration = float(self._last_acc_mag)

        msg.power_m1 = int(max(0, min(100, self._last_m1)))
        msg.power_m2 = int(max(0, min(100, self._last_m2)))

        self.debug_pub.publish(msg)
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
        
    def _publish_markers(self):
        """Publica MarkerArray con 3 elementos visuales en RViz."""
        if self.ship is None:
            return

        s = self.ship
        now = self.get_clock().now().to_msg()

        state_colors = {
            'IDLE':    (0.5, 0.5, 0.5),
            'ORIENT':  (1.0, 0.8, 0.0),
            'THRUST':  (0.0, 0.8, 1.0),
            'BRAKE':   (1.0, 0.3, 0.0),
            'HOVER':   (0.4, 1.0, 0.4),
            'ARRIVED': (0.0, 1.0, 0.0),
        }
        r, g, b = state_colors.get(self.state.value, (1.0, 1.0, 1.0))

        markers = MarkerArray()

        # ── Marcador 1: Texto con fase, distancia y tiempo ────────────
        dx = self.target_x - s.x
        dy = self.target_y - s.y
        dist = math.sqrt(dx * dx + dy * dy)

        m_text = Marker()
        m_text.header.stamp = now
        m_text.header.frame_id = 'world'
        m_text.ns = 'controller'
        m_text.id = 0
        m_text.type = Marker.TEXT_VIEW_FACING
        m_text.action = Marker.ADD
        m_text.pose.position.x = s.x
        m_text.pose.position.y = s.y + 2.5
        m_text.pose.position.z = 0.0
        m_text.scale.z = 1.2
        m_text.color.r = r
        m_text.color.g = g
        m_text.color.b = b
        m_text.color.a = 1.0
        m_text.text = (
            f'[{self.state.value}]\n'
            f'dist: {dist:.1f} m\n'
            f't: {s.elapsed_time:.1f} s'
        )
        markers.markers.append(m_text)

        # ── Marcador 2: Flecha heading deseado ────────────────────────
        arrow_len = 3.5
        m_arrow = Marker()
        m_arrow.header.stamp = now
        m_arrow.header.frame_id = 'world'
        m_arrow.ns = 'controller'
        m_arrow.id = 1
        m_arrow.type = Marker.ARROW
        m_arrow.action = Marker.ADD
        m_arrow.scale.x = 0.15
        m_arrow.scale.y = 0.35
        m_arrow.scale.z = 0.35
        m_arrow.color.r = 1.0
        m_arrow.color.g = 1.0
        m_arrow.color.b = 0.0
        m_arrow.color.a = 1.0

        hdg = self._last_desired_heading
        p_start = Point()
        p_start.x = s.x
        p_start.y = s.y
        p_end = Point()
        p_end.x = s.x + arrow_len * math.cos(hdg)
        p_end.y = s.y + arrow_len * math.sin(hdg)
        m_arrow.points = [p_start, p_end]
        markers.markers.append(m_arrow)

        # ── Marcador 3: Flecha hacia el target ────────────────────────
        m_line = Marker()
        m_line.header.stamp = now
        m_line.header.frame_id = 'world'
        m_line.ns = 'controller'
        m_line.id = 2
        m_line.type = Marker.ARROW
        m_line.action = Marker.ADD
        m_line.scale.x = 0.10
        m_line.scale.y = 0.25
        m_line.scale.z = 0.25
        m_line.color.r = r
        m_line.color.g = g
        m_line.color.b = b
        m_line.color.a = 0.8

        p2_start = Point()
        p2_start.x = s.x
        p2_start.y = s.y
        p2_end = Point()
        p2_end.x = self.target_x
        p2_end.y = self.target_y
        m_line.points = [p2_start, p2_end]
        markers.markers.append(m_line)

        self.marker_pub.publish(markers)


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
