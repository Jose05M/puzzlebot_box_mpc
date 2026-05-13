import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import tty
import termios

class TeleopColor(Node):
    def __init__(self):
        super().__init__('teleop_color')
        self.pub = self.create_publisher(String, '/box_color_command', 10)
        print("\n==========================================")
        print("CONTROL DE COLOR - Puzzlebot")
        print("==========================================")
        print("  g        = objetivo verde")
        print("  p        = objetivo rosa")
        print("  y        = objetivo amarillo")
        print("  h        = ir a home (0,0)")
        print("  w x y    = ir a waypoint  ej: w 1.5 2.0")
        print("  c        = cancelar")
        print("  q        = salir")
        print("==========================================\n")

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def publish(self, data):
        msg = String()
        msg.data = data
        self.pub.publish(msg)
        print(f"Enviado: {data}")

    def run(self):
        key_map = {
            'g': 'green',
            'p': 'pink',
            'y': 'yellow',
            'c': 'cancel',
        }
        while True:
            key = self.get_key()

            if key == 'q':
                break

            elif key == 'h':
                self.publish('home')

            elif key == 'w':
                # Necesita coordenadas: restaurar terminal para usar input()
                fd = sys.stdin.fileno()
                old = termios.tcgetattr(fd)
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
                try:
                    coords = input(" x y: ").strip()
                    parts = coords.split()
                    if len(parts) == 2:
                        float(parts[0])  # valida que sean números
                        float(parts[1])
                        self.publish(f"waypoint {parts[0]} {parts[1]}")
                    else:
                        print("Formato inválido. Usa: w  ej: w 1.5 2.0")
                except ValueError:
                    print("Coordenadas inválidas. Deben ser números.")

            elif key in key_map:
                self.publish(key_map[key])

def main(args=None):
    rclpy.init(args=args)
    node = TeleopColor()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
