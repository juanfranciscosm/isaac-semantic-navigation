import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math



VELOCIDAD_GIRO   = 1.0
VELOCIDAD_LINEAL = 0.3
DISTANCIA_BRAZOS = 1.5  # DISTANCIAS DE LA TRAYECTORIA EN U DE ESTE SCRIPT
DISTANCIA_BASE   = 6  # DISTANCIAS DE LA TRAYECTORIA EN U DE ESTE SCRIPT

# Función matemática para convertir la rotación 3D (cuaternión) a 2D (grados/radianes)
def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class OdomNavigator(Node):
    def __init__(self):
        super().__init__('odom_navigator')
        
        # Publicador y Suscriptor
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # --- PARÁMETROS DE TU TRAYECTORIA EN U ---
        self.m = DISTANCIA_BRAZOS  # Distancia de los brazos (metros)
        self.n = DISTANCIA_BASE  # Distancia de la base (metros)
        
        # Secuencia de acciones: (Tipo, Valor)
        # Tipos: 0 = Avanzar, 1 = Girar Izquierda
        self.rutina = [
            (0, self.m),  # Avanzar m
            (1, 90.0),    # Girar 90° izq
            (0, self.n),  # Avanzar n
            (1, 90.0),    # Girar 90° izq
            (0, self.m)   # Avanzar m
        ]
        
        # Variables de control
        self.paso_actual = 0
        self.origen_x = None
        self.origen_y = None
        self.origen_yaw = None
        
        self.get_logger().info('🚀 Navegador por Odometría iniciado.')

    def odom_callback(self, msg):
        # Si ya terminamos la rutina, no hacemos nada
        if self.paso_actual >= len(self.rutina):
            return

        # 1. Leer posición actual del rover
        x_actual = msg.pose.pose.position.x
        y_actual = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw_actual = euler_from_quaternion(q.x, q.y, q.z, q.w)

        # Guardar el punto de inicio de la maniobra actual
        if self.origen_x is None:
            self.origen_x = x_actual
            self.origen_y = y_actual
            self.origen_yaw = yaw_actual
            tipo_maniobra, valor = self.rutina[self.paso_actual]
            accion = "Avanzando" if tipo_maniobra == 0 else "Girando"
            self.get_logger().info(f'Paso {self.paso_actual + 1}: {accion} {valor} unidades...')

        cmd = Twist()
        tipo_maniobra, valor_objetivo = self.rutina[self.paso_actual]

        # --- MANIOBRA: AVANZAR ---
        if tipo_maniobra == 0:
            distancia_recorrida = math.sqrt((x_actual - self.origen_x)**2 + (y_actual - self.origen_y)**2)
            
            if distancia_recorrida < valor_objetivo:
                cmd.linear.x = VELOCIDAD_LINEAL  # Velocidad de avance (m/s)
            else:
                self.siguiente_paso()

        # --- MANIOBRA: GIRAR ---
        elif tipo_maniobra == 1:
            angulo_objetivo = self.origen_yaw + math.radians(valor_objetivo)
            
            # Normalizar ángulo entre -pi y pi
            angulo_objetivo = math.atan2(math.sin(angulo_objetivo), math.cos(angulo_objetivo))
            diferencia_angulo = math.atan2(math.sin(angulo_objetivo - yaw_actual), math.cos(angulo_objetivo - yaw_actual))
            
            if abs(diferencia_angulo) > 0.05: # Margen de error de ~3 grados
                cmd.angular.z = VELOCIDAD_GIRO if diferencia_angulo > 0 else -1*VELOCIDAD_GIRO # Velocidad de giro
            else:
                self.siguiente_paso()

        # Publicar el comando a los motores
        self.pub.publish(cmd)

    def siguiente_paso(self):
        # Frenar antes de pasar a la siguiente maniobra
        self.pub.publish(Twist()) 
        self.paso_actual += 1
        self.origen_x = None  # Resetear origen para la siguiente lectura
        
        if self.paso_actual >= len(self.rutina):
            self.get_logger().info('✅ Trayectoria en U completada con éxito.')

def main(args=None):
    rclpy.init(args=args)
    nodo = OdomNavigator()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()