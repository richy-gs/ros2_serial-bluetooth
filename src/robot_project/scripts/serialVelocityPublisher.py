import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist


class SerialVelocityPublisher(Node):
    def __init__(self):
        super().__init__('serial_velocity_publisher')

        # Declarar parámetros con valores predeterminados
        self.declare_parameters(
            namespace='',
            parameters=[
                ('device', '/dev/ttyUSB0'),  # Dispositivo serial
                ('baudrate', 115200),       # Baudrate de la conexión serial
                ('topic_name', '/cmd_vel'), # Tópico donde se publicarán las velocidades
            ]
        )

        # Obtener parámetros
        self.device_name = self.get_parameter('device').value
        self.baudrate = self.get_parameter('baudrate').value
        self.topic_name = self.get_parameter('topic_name').value

        # Configurar la conexión serial
        try:
            self.ser = serial.Serial(self.device_name, self.baudrate, timeout=0.1)
            self.ser.reset_input_buffer()
            self.get_logger().info(f"Conexión serial establecida en {self.device_name} a {self.baudrate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"No se pudo abrir el puerto serial: {e}")
            rclpy.shutdown()

        # Configurar publicador para el tópico de velocidades
        self.publisher = self.create_publisher(Twist, self.topic_name, 10)

        # Configurar un temporizador para leer datos seriales
        self.timer = self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        """Leer datos de la conexión serial y publicarlos en un tópico."""
        try:
            # Leer una línea del puerto serial
            line = self.ser.readline().decode('utf-8').strip()

            if line:
                # Supongamos que el formato de los datos es "linear:0.5,angular:1.0"
                self.get_logger().info(f"Datos recibidos: {line}")
                data = line.split(',')

                # Extraer velocidades
                linear = float(data[0].split(':')[1])
                angular = float(data[1].split(':')[1])

                # Crear mensaje Twist y publicar
                twist_msg = Twist()
                twist_msg.linear.x = linear
                twist_msg.angular.z = angular
                self.publisher.publish(twist_msg)

                self.get_logger().info(f"Publicado - Velocidad lineal: {linear} m/s, angular: {angular} rad/s")
        except (ValueError, IndexError):
            self.get_logger().warn(f"Formato incorrecto en los datos recibidos: {line}")
        except serial.SerialException as e:
            self.get_logger().error(f"Error en la conexión serial: {e}")

    def destroy_node(self):
        """Cerrar el puerto serial al apagar el nodo."""
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Inicializar el nodo
    serial_velocity_publisher = SerialVelocityPublisher()

    try:
        rclpy.spin(serial_velocity_publisher)
    except KeyboardInterrupt:
        pass

    # Finalizar
    serial_velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
