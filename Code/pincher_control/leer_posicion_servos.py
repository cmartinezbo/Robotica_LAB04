# pincher_control/leer_posicion_servos.py
import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import time

# Direcciones del AX-12
ADDR_PRESENT_POSITION = 36

def ticks_a_grados(ticks):
    """
    Convierte ticks (0 a 1023) a grados, tomando 512 como el punto 0°.
    """
    return round((ticks - 512) * (300 / 1023), 2)

class LeerPosicionServos(Node):
    def __init__(self):
        super().__init__('leer_posicion_servos')

        # Parámetros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('dxl_ids', [1, 2, 3, 4, 5])

        port_name = self.get_parameter('port').value
        baudrate  = self.get_parameter('baudrate').value
        dxl_ids   = self.get_parameter('dxl_ids').value

        # Inicializar comunicación
        port = PortHandler(port_name)
        if not port.openPort():
            self.get_logger().error("No se pudo abrir el puerto.")
            rclpy.shutdown()
            return
        port.setBaudRate(baudrate)
        packet = PacketHandler(1.0)

        self.get_logger().info("Leyendo posiciones actuales...\n")

        # Leer e imprimir posiciones
        for dxl_id in dxl_ids:
            pos, dxl_comm_result, dxl_error = packet.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
            if dxl_comm_result != 0 or dxl_error != 0:
                self.get_logger().warn(f'[ID {dxl_id}] Error de comunicación.')
            else:
                grados = ticks_a_grados(pos)
                self.get_logger().info(f'[ID {dxl_id}] → {pos} ticks = {grados}°')

        port.closePort()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    LeerPosicionServos()

if __name__ == '__main__':
    main()
