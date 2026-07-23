import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import json

class SegmentationCollector(Node):
    def __init__(self):
        super().__init__('segmentation_collector')
        self.bridge = CvBridge()
        
        # 1. Configuración de carpetas
        self.save_dir = 'isaac_dataset'
        self.raw_dir = os.path.join(self.save_dir, 'seg_raw')
        self.color_dir = os.path.join(self.save_dir, 'seg_color')
        os.makedirs(self.raw_dir, exist_ok=True)
        os.makedirs(self.color_dir, exist_ok=True)
        
        self.frame_count = 0
        self.last_save_time = self.get_clock().now()
        self.save_interval = 1.0 # Guardar 1 vez por segundo
        
        # 2. Configuración para el mapeo de etiquetas
        self.master_labels = {}
        self.labels_path = os.path.join(self.save_dir, 'semantic_labels.json')
        
        # Mapa de colores para humanos (0 suele ser fondo, 1 unlabelled)
        self.color_map = {0: (0, 0, 0), 1: (128, 128, 128)}
        
        # 3. Suscriptores (Sin message_filters porque String no tiene timestamp)
        self.sub_labels = self.create_subscription(
            String, '/semantic_labels', self.labels_callback, 10)
            
        self.sub_seg = self.create_subscription(
            Image, '/instance_segmentation', self.seg_callback, 10)
            
        self.get_logger().info("Recolector de Segmentación iniciado. Esperando datos...")

    def labels_callback(self, msg):
        try:
            # Isaac Sim envía un string con formato JSON: {"0":"BACKGROUND", "126":"..."}
            latest_labels = json.loads(msg.data)
            
            # Actualizamos el diccionario maestro si detectamos nuevos objetos
            updated = False
            for key, value in latest_labels.items():
                if key not in self.master_labels:
                    self.master_labels[key] = value
                    updated = True
                    
            # Si hay objetos nuevos, reescribimos el archivo JSON
            if updated:
                with open(self.labels_path, 'w') as f:
                    json.dump(self.master_labels, f, indent=4)
                    
        except Exception as e:
            self.get_logger().error(f"Error parseando las etiquetas: {e}")

    def get_color(self, uid):
        # Si el ID no tiene color asignado, le generamos uno aleatorio pero constante
        if uid not in self.color_map:
            np.random.seed(uid) # La semilla garantiza que el ID 126 siempre sea del mismo color
            self.color_map[uid] = tuple(np.random.randint(0, 255, size=3).tolist())
        return self.color_map[uid]

    def seg_callback(self, msg_seg):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_save_time).nanoseconds / 1e9
        
        if time_diff < self.save_interval:
            return
            
        self.last_save_time = current_time
        base_name = f"seg_frame_{self.frame_count:06d}"

        try:
            # A. Extraer imagen raw (Conserva la codificación 32SC1 original)
            cv_seg = self.bridge.imgmsg_to_cv2(msg_seg, desired_encoding='passthrough')
            
            # Guardar array int32 exacto para la red neuronal
            np.save(os.path.join(self.raw_dir, f"{base_name}.npy"), cv_seg)
            
            # B. Crear la imagen a color para inspección visual humana
            color_img = np.zeros((*cv_seg.shape, 3), dtype=np.uint8)
            unique_ids = np.unique(cv_seg)
            
            for uid in unique_ids:
                color = self.get_color(int(uid))
                # Aplicamos el color donde el array original coincida con el ID
                color_img[cv_seg == uid] = color
                
            # Guardar la imagen para humanos
            cv2.imwrite(os.path.join(self.color_dir, f"{base_name}.png"), color_img)

            self.get_logger().info(f"✅ Segmentación guardada: {base_name} | IDs detectados: {len(unique_ids)}")
            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f"Error procesando frame {self.frame_count}: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()