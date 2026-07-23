import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2
import message_filters
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import json
import time

class SingleFrameCollector(Node):
    def __init__(self):
        super().__init__('single_frame_collector')
        self.bridge = CvBridge()
        
        # 1. Configuración de carpetas principales
        self.save_dir = 'isaac_dataset/single_shots'
        self.folders = ['rgb', 'depth', 'seg_raw', 'seg_color', 'pcl']
        for folder in self.folders:
            os.makedirs(os.path.join(self.save_dir, folder), exist_ok=True)
            
        # Variables de estado
        self.labels_dict = {}
        self.labels_received = False
        self.snapshot_taken = False
        
        # 2. Suscriptor de etiquetas (Asíncrono)
        self.sub_labels = self.create_subscription(
            String, '/semantic_labels', self.labels_callback, 10)

        # 3. Suscriptores sincronizados (Imágenes y Nube de puntos)
        self.sub_rgb = message_filters.Subscriber(self, Image, '/rgb')
        self.sub_depth = message_filters.Subscriber(self, Image, '/depth')
        self.sub_seg = message_filters.Subscriber(self, Image, '/instance_segmentation')
        self.sub_pcl = message_filters.Subscriber(self, PointCloud2, '/depth_pcl')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth, self.sub_seg, self.sub_pcl],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)
        
        self.get_logger().info("📸 Esperando sincronización de sensores y etiquetas para capturar...")

    def labels_callback(self, msg):
        if not self.labels_received:
            try:
                self.labels_dict = json.loads(msg.data)
                self.labels_received = True
                self.get_logger().info("Etiquetas semánticas recibidas.")
            except Exception as e:
                self.get_logger().error(f"Error parseando etiquetas: {e}")

    def save_pcd_ascii(self, filename, points):
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
            np.savetxt(f, points, fmt='%.4f %.4f %.4f')

    def get_color(self, uid):
        np.random.seed(uid)
        return tuple(np.random.randint(0, 255, size=3).tolist())

    def sync_callback(self, msg_rgb, msg_depth, msg_seg, msg_pcl):
        # Asegurarnos de tener las etiquetas antes de guardar la imagen
        if not self.labels_received:
            return
            
        # Evitar capturas múltiples si la función es llamada rápidamente
        if self.snapshot_taken:
            return
            
        self.snapshot_taken = True
        
        # Generar un nombre único basado en el tiempo actual
        timestamp = int(time.time())
        base_name = f"shot_{timestamp}"
        
        self.get_logger().info(f"Procesando frame unificado: {base_name}...")

        try:
            # --- 1. RGB ---
            cv_rgb = self.bridge.imgmsg_to_cv2(msg_rgb, desired_encoding='bgr8')
            cv2.imwrite(os.path.join(self.save_dir, 'rgb', f"{base_name}.png"), cv_rgb)

            # --- 2. Depth ---
            cv_depth = self.bridge.imgmsg_to_cv2(msg_depth, desired_encoding='passthrough')
            np.save(os.path.join(self.save_dir, 'depth', f"{base_name}.npy"), cv_depth)

            # --- 3. Segmentación ---
            cv_seg = self.bridge.imgmsg_to_cv2(msg_seg, desired_encoding='passthrough')
            np.save(os.path.join(self.save_dir, 'seg_raw', f"{base_name}.npy"), cv_seg)
            
            color_img = np.zeros((*cv_seg.shape, 3), dtype=np.uint8)
            for uid in np.unique(cv_seg):
                color_img[cv_seg == uid] = self.get_color(int(uid))
            cv2.imwrite(os.path.join(self.save_dir, 'seg_color', f"{base_name}.png"), color_img)

            # --- 4. Point Cloud (Con la corrección de estructura) ---
            points_gen = pc2.read_points(msg_pcl, field_names=("x", "y", "z"), skip_nans=True)
            points_list = list(points_gen)
            if len(points_list) > 0:
                clean_points = [[p[0], p[1], p[2]] for p in points_list]
                xyz = np.array(clean_points, dtype=np.float32)
                self.save_pcd_ascii(os.path.join(self.save_dir, 'pcl', f"{base_name}.pcd"), xyz)

            # --- 5. Guardar Diccionario JSON ---
            with open(os.path.join(self.save_dir, f"{base_name}_labels.json"), 'w') as f:
                json.dump(self.labels_dict, f, indent=4)

            self.get_logger().info(f"✅ ¡Captura exitosa! Datos guardados en {self.save_dir}")
            
        except Exception as e:
            self.get_logger().error(f"Error procesando los datos: {str(e)}")

        # Salir limpiamente del nodo de ROS 2
        self.get_logger().info("Finalizando script...")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = SingleFrameCollector()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass # Capturamos el SystemExit para salir sin mostrar errores en la terminal
    finally:
        node.destroy_node()
        # Verificamos si el contexto sigue activo antes de apagarlo
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()