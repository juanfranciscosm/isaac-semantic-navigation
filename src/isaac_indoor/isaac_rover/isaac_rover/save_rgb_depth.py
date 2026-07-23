import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class SimpleDatasetCollector(Node):
    def __init__(self):
        super().__init__('simple_dataset_collector')
        self.bridge = CvBridge()
        
        # Configuración de carpetas
        self.save_dir = 'isaac_dataset'
        self.folders = ['rgb', 'depth']
        for folder in self.folders:
            os.makedirs(os.path.join(self.save_dir, folder), exist_ok=True)
            
        self.frame_count = 0
        self.last_save_time = self.get_clock().now()
        self.save_interval = 1.0 # Guardar 1 vez por segundo

        # Suscriptores a los tópicos de las cámaras
        self.sub_rgb = message_filters.Subscriber(self, Image, '/rgb')
        self.sub_depth = message_filters.Subscriber(self, Image, '/depth')

        # Sincronizador de tiempo para asegurar que RGB y Depth son del mismo instante
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)
        
        self.get_logger().info("Recolector RGB-Depth iniciado. Esperando datos...")

    def sync_callback(self, msg_rgb, msg_depth):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_save_time).nanoseconds / 1e9
        
        if time_diff < self.save_interval:
            return
            
        self.last_save_time = current_time
        base_name = f"frame_{self.frame_count:06d}"

        try:
            # 1. Procesar e imprimir imagen RGB
            cv_rgb = self.bridge.imgmsg_to_cv2(msg_rgb, desired_encoding='bgr8')
            cv2.imwrite(os.path.join(self.save_dir, 'rgb', f"{base_name}.png"), cv_rgb)

            # 2. Procesar Depth (Guardar como Numpy Float32 para mantener precisión nanométrica)
            cv_depth = self.bridge.imgmsg_to_cv2(msg_depth, desired_encoding='passthrough')
            np.save(os.path.join(self.save_dir, 'depth', f"{base_name}.npy"), cv_depth)

            self.get_logger().info(f"✅ Extraído: {base_name} (RGB + Depth)")
            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f"Error procesando frame {self.frame_count}: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleDatasetCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()