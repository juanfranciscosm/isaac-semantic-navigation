import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import os

class PclCollector(Node):
    def __init__(self):
        super().__init__('pcl_collector')
        
        # 1. Configuración de carpetas
        self.save_dir = 'isaac_dataset'
        self.pcl_dir = os.path.join(self.save_dir, 'pcl')
        os.makedirs(self.pcl_dir, exist_ok=True)
        
        self.frame_count = 0
        self.last_save_time = self.get_clock().now()
        self.save_interval = 1.0 # Guardar 1 vez por segundo (ajustable)
        
        # 2. Suscriptor al tópico de la nube de puntos
        self.sub_pcl = self.create_subscription(
            PointCloud2, '/depth_pcl', self.pcl_callback, 10)
            
        self.get_logger().info("Recolector de Nube de Puntos (PCL) iniciado. Esperando datos...")
        self.get_logger().info(f"Guardando archivos .pcd en: {os.path.abspath(self.pcl_dir)}")

    def save_pcd_ascii(self, filename, points):
        """Genera un archivo .pcd estándar en formato ASCII sin dependencias externas"""
        num_points = points.shape[0]
        
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            f"WIDTH {num_points}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {num_points}\n"
            "DATA ascii\n"
        )
        
        with open(filename, 'w') as f:
            f.write(header)
            # Escribir las coordenadas x, y, z de cada punto
            np.savetxt(f, points, fmt='%.4f %.4f %.4f')

    def pcl_callback(self, msg_pcl):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_save_time).nanoseconds / 1e9
        
        # Control de frecuencia de guardado
        if time_diff < self.save_interval:
            return
            
        self.last_save_time = current_time
        base_name = f"pcl_frame_{self.frame_count:06d}"
        file_path = os.path.join(self.pcl_dir, f"{base_name}.pcd")

        try:
            # Leer los puntos (x, y, z) ignorando los valores NaNs (puntos infinitos en el vacío)
            points_gen = pc2.read_points(msg_pcl, field_names=("x", "y", "z"), skip_nans=True)
            points_list = list(points_gen)
            
            if len(points_list) > 0:
                # Extraemos explícitamente solo los números ignorando las etiquetas estructuradas
                clean_points = [[p[0], p[1], p[2]] for p in points_list]
                xyz = np.array(clean_points, dtype=np.float32)
                
                # Guardar en formato .pcd
                self.save_pcd_ascii(file_path, xyz)
                
                self.get_logger().info(f"✅ Nube de puntos guardada: {base_name}.pcd | Puntos: {xyz.shape[0]}")
                self.frame_count += 1
            else:
                self.get_logger().warn("Frame de nube de puntos recibido pero estaba vacío.")

        except Exception as e:
            self.get_logger().error(f"Error procesando el frame PCL {self.frame_count}: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = PclCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()