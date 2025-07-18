import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QWidget, QLabel, QVBoxLayout, QPushButton, QLineEdit, QFormLayout)
from PyQt5.QtCore import QTimer
from dynamixel_sdk import PortHandler, PacketHandler
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

ADDR_PRESENT_POSITION = 36
ADDR_GOAL_POSITION = 30
ADDR_TORQUE_ENABLE = 24
ADDR_MOVING_SPEED = 32
ADDR_TORQUE_LIMIT = 34

# Conversión grados a posición: 0° = 512, -90° = 256, 90° = 768

def degrees_to_position(deg):
    return int(512 + (deg / 300.0) * 1023)

def position_to_degrees(pos):
    return (pos - 512) * 300.0 / 1023

class HMIWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        self.setWindowTitle('HMI PhantomX Pincher X100')

        layout = QVBoxLayout()

        self.label_title = QLabel('Juan José Díaz, Cristian Martinez')
        layout.addWidget(self.label_title)

        self.form = QFormLayout()
        self.joint_inputs = []
        for i in range(5):
            input_field = QLineEdit()
            self.form.addRow(f'Articulación {i+1} (°)', input_field)
            self.joint_inputs.append(input_field)
        layout.addLayout(self.form)

        self.send_button = QPushButton('Enviar posiciones')
        self.send_button.clicked.connect(self.send_positions)
        layout.addWidget(self.send_button)

        self.positions_label = QLabel('Posiciones actuales:')
        layout.addWidget(self.positions_label)

        self.positions_display = QLabel('')
        layout.addWidget(self.positions_display)

        self.figure = Figure(figsize=(5, 5))
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_positions)
        self.timer.start(1000)

    def send_positions(self):
        goals = []
        for field in self.joint_inputs:
            try:
                deg = float(field.text())
                if -150 <= deg <= 150:
                    val = degrees_to_position(deg)
                    goals.append(val)
                else:
                    self.node.get_logger().warn('Ángulo fuera de rango [-150, 150]')
                    return
            except ValueError:
                self.node.get_logger().warn('Entrada inválida')
                return

        for i, dxl_id in enumerate(self.node.dxl_ids):
            self.node.packet_handler.write2ByteTxRx(
                self.node.port_handler, dxl_id, ADDR_GOAL_POSITION, goals[i]
            )
            self.node.get_logger().info(f'[ID {dxl_id}] Posición enviada: {goals[i]}')

        self.plot_3d(goals)

    def update_positions(self):
        positions = []
        raw_positions = []
        for dxl_id in self.node.dxl_ids:
            pos, _, _ = self.node.packet_handler.read2ByteTxRx(
                self.node.port_handler, dxl_id, ADDR_PRESENT_POSITION
            )
            raw_positions.append(pos)
            deg = position_to_degrees(pos)
            positions.append(f'{deg:.1f}°')
        self.positions_display.setText(' | '.join(positions))

        self.plot_3d(raw_positions)

    def plot_3d(self, pos_list):
        angles_deg = [position_to_degrees(val) for val in pos_list]
        angles_rad = np.radians(angles_deg)

        l1, l2, l3, l4 = 137, 105, 105, 110

        theta1 = angles_rad[0]
        theta2 = angles_rad[1]
        theta3 = angles_rad[2]
        theta4 = angles_rad[3]

        x = [0]
        y = [0]
        z = [0]

        x1 = 0
        y1 = 0
        z1 = l1
        x.append(x1)
        y.append(y1)
        z.append(z1)

        z2 = z1 + l2 * np.cos(theta2)
        r2 = l2 * np.sin(theta2)
        x2 = r2 * np.cos(theta1)
        y2 = r2 * np.sin(theta1)
        x.append(x2)
        y.append(y2)
        z.append(z2)

        z3 = z2 + l3 * np.cos(theta2 + theta3)
        r3 = r2 + l3 * np.sin(theta2 + theta3)
        x3 = r3 * np.cos(theta1)
        y3 = r3 * np.sin(theta1)
        x.append(x3)
        y.append(y3)
        z.append(z3)

        z4 = z3 + l4 * np.cos(theta2 + theta3 + theta4)
        r4 = r3 + l4 * np.sin(theta2 + theta3 + theta4)
        x4 = r4 * np.cos(theta1)
        y4 = r4 * np.sin(theta1)
        x.append(x4)
        y.append(y4)
        z.append(z4)

        self.figure.clear()
        ax = self.figure.add_subplot(111, projection='3d')
        ax.plot(x, y, z, marker='o')
        ax.set_xlim(-300, 300)
        ax.set_ylim(-300, 300)
        ax.set_zlim(0, 400)
        ax.set_title('Vista 3D manipulador')
        ax.grid(True)
        self.canvas.draw()

class PincherNode(Node):
    def __init__(self):
        super().__init__('hmi_pincher')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('dxl_ids', [1, 2, 3, 4, 5])

        port_name = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.dxl_ids = self.get_parameter('dxl_ids').value

        self.port_handler = PortHandler(port_name)
        self.port_handler.openPort()
        self.port_handler.setBaudRate(baudrate)
        self.packet_handler = PacketHandler(1.0)

        for dxl_id in self.dxl_ids:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, 1
            )
            self.packet_handler.write2ByteTxRx(
                self.port_handler, dxl_id, ADDR_MOVING_SPEED, 50
            )
            self.packet_handler.write2ByteTxRx(
                self.port_handler, dxl_id, ADDR_TORQUE_LIMIT, 800
            )

        self.get_logger().info('Conexión establecida con servos Dynamixel.')

def main(args=None):
    rclpy.init(args=args)
    node = PincherNode()

    app = QApplication(sys.argv)
    hmi = HMIWindow(node)
    hmi.show()
    app.exec_()

    rclpy.shutdown()

if __name__ == '__main__':
    main()