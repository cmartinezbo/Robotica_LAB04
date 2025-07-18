# pincher_control/control_servo.py
import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import time

ADDR_TORQUE_ENABLE    = 24
ADDR_GOAL_POSITION    = 30
ADDR_MOVING_SPEED     = 32
ADDR_TORQUE_LIMIT     = 34
ADDR_PRESENT_POSITION = 36

class PincherController(Node):
    def __init__(self):
        super().__init__('pincher_controller')

        # Parámetros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('dxl_ids', [1, 2, 3, 4, 5])
        self.declare_parameter('home_position', 512)
        self.declare_parameter('goal_positions', [256, 640, 640, 640, 512])
        self.declare_parameter('moving_speed', 100)
        self.declare_parameter('torque_limit', 1000)
        self.declare_parameter('delay', 2.0)

        port_name      = self.get_parameter('port').value
        baudrate       = self.get_parameter('baudrate').value
        dxl_ids        = self.get_parameter('dxl_ids').value
        home_pos       = self.get_parameter('home_position').value
        goal_positions = self.get_parameter('goal_positions').value
        moving_speed   = self.get_parameter('moving_speed').value
        torque_limit   = self.get_parameter('torque_limit').value
        delay_seconds  = self.get_parameter('delay').value

        if len(goal_positions) != len(dxl_ids):
            self.get_logger().error(
                f'La lista goal_positions ({len(goal_positions)}) '
                f'debe tener la misma longitud que dxl_ids ({len(dxl_ids)})'
            )
            rclpy.shutdown()
            return

        # Inicializar comunicación
        port   = PortHandler(port_name)
        port.openPort()
        port.setBaudRate(baudrate)
        packet = PacketHandler(1.0)

        # Inicializar todos los servos en posición "home"
        for dxl_id in dxl_ids:
            packet.write2ByteTxRx(port, dxl_id, ADDR_TORQUE_LIMIT, torque_limit)
            packet.write2ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, moving_speed)
            packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)
            packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, home_pos)
            self.get_logger().info(f'[ID {dxl_id}] inicializado en posición HOME={home_pos}')

        self.get_logger().info('Esperando para que el robot alcance la posición HOME...')
        time.sleep(delay_seconds)

        # Mover articulaciones una por una
        for i, dxl_id in enumerate(dxl_ids):
            goal = goal_positions[i]
            packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, goal)
            self.get_logger().info(f'[ID {dxl_id}] → Moviendo a {goal}')
            time.sleep(delay_seconds)

        # Mostrar posición final
        for dxl_id in dxl_ids:
            pos, _, _ = packet.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
            self.get_logger().info(f'[ID {dxl_id}] posición actual = {pos}')

        # Inicializar todos los servos en posición "home"
        for dxl_id in dxl_ids:
            packet.write2ByteTxRx(port, dxl_id, ADDR_TORQUE_LIMIT, torque_limit)
            packet.write2ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, moving_speed)
            packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)
            packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, home_pos)
            self.get_logger().info(f'[ID {dxl_id}] inicializado en posición HOME={home_pos}')

        self.get_logger().info('Esperando para que el robot alcance la posición HOME...')
        time.sleep(delay_seconds)

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    PincherController()

if __name__ == '__main__':
    main()