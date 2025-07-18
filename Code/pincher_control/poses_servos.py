import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import time

# Direcciones de los registros del AX-12A
ADDR_TORQUE_ENABLE    = 24
ADDR_GOAL_POSITION    = 30
ADDR_MOVING_SPEED     = 32
ADDR_TORQUE_LIMIT     = 34
ADDR_PRESENT_POSITION = 36

# 🧠 Función para convertir grados a ticks (0–1023)
def grados_a_ticks(angulos_grados):
    ticks = []
    for angulo in angulos_grados:
        tick = int(512 + angulo * (1023 / 300))  # Conversión
        tick = max(0, min(1023, tick))           # Limitar dentro de rango
        ticks.append(tick)
    return ticks

# 🤖 Clase principal
class PincherController(Node):
    def __init__(self):
        super().__init__('pincher_controller')

        # Parámetros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('dxl_ids', [1, 2, 3, 4, 5])
        self.declare_parameter('moving_speed', 100)
        self.declare_parameter('torque_limit', 1000)
        self.declare_parameter('delay', 1.5)

        port_name     = self.get_parameter('port').value
        baudrate      = self.get_parameter('baudrate').value
        dxl_ids       = self.get_parameter('dxl_ids').value
        moving_speed  = self.get_parameter('moving_speed').value
        torque_limit  = self.get_parameter('torque_limit').value
        delay_seconds = self.get_parameter('delay').value

        # 🟦 Definición de las 5 poses (en grados)
        poses_grados = [
            [0, 0, 0, 0, 0],
            [25, 25, 20, -20, 0],
            [-35, 35, -30, 30, 0],
            [85, -20, 55, 25, 0],
            [80, -35, 55, -45, 0]
        ]

        # Inicializar comunicación
        port = PortHandler(port_name)
        if not port.openPort():
            self.get_logger().error(" No se pudo abrir el puerto.")
            rclpy.shutdown()
            return
        port.setBaudRate(baudrate)
        packet = PacketHandler(1.0)

        # Habilitar torque, setear velocidad y torque límite
        for dxl_id in dxl_ids:
            packet.write2ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, moving_speed)
            packet.write2ByteTxRx(port, dxl_id, ADDR_TORQUE_LIMIT, torque_limit)
            packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)

        # Recorrer cada pose
        for i, pose in enumerate(poses_grados):
            self.get_logger().info(f' Enviando pose {i + 1}: {pose}')
            ticks = grados_a_ticks(pose)

            # Mover articulaciones una a una
            for j, dxl_id in enumerate(dxl_ids):
                packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, ticks[j])
                self.get_logger().info(f'   [ID {dxl_id}] → {pose[j]}° = {ticks[j]} ticks')
                time.sleep(delay_seconds)

            self.get_logger().info(f' Pose {i + 1} completada.\n')
            time.sleep(delay_seconds)

        # Leer posición final
        for dxl_id in dxl_ids:
            pos, _, _ = packet.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
            self.get_logger().info(f'[ID {dxl_id}] posición final = {pos} ticks')

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    PincherController()

if __name__ == '__main__':
    main()
