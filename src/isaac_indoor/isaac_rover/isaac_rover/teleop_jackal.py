import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, tty, termios

SPEED_STEP = 0.1
ANGULAR_STEP = 0.1

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.linear_speed = 0.5
        self.angular_speed = 0.5
        self.display_ui()

    def display_ui(self):
        # Limpiar terminal para que se vea estético
        print("\033[H\033[J", end="") 
        print("--- CONTROLES JACKAL (Isaac Sim) ---")
        print(f"Movimiento: WASD | Detener: Cualquier otra tecla")
        print(f"Lineal (q/e):  {self.linear_speed:.2f}")
        print(f"Angular (z/c): {self.angular_speed:.2f}")
        print("Salir: Presiona 'ESC' o 'x'")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while True:
                key = self.get_key()
                twist = Twist()
                
                # Lógica de velocidades
                if key == 'w': twist.linear.x = self.linear_speed
                elif key == 's': twist.linear.x = -self.linear_speed
                elif key == 'a': twist.angular.z = self.angular_speed
                elif key == 'd': twist.angular.z = -self.angular_speed
                
                # Ajustes (q/e lineal, z/c angular)
                elif key == 'q': 
                    self.linear_speed += SPEED_STEP
                    self.display_ui()
                elif key == 'e': 
                    self.linear_speed = max(0.0, self.linear_speed - SPEED_STEP)
                    self.display_ui()
                elif key == 'z': 
                    self.angular_speed += ANGULAR_STEP
                    self.display_ui()
                elif key == 'c': 
                    self.angular_speed = max(0.0, self.angular_speed - ANGULAR_STEP)
                    self.display_ui()
                
                elif key == 'x': break # Salida
                
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                
                self.publisher_.publish(twist)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    teleop = TeleopKeyboard()
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()