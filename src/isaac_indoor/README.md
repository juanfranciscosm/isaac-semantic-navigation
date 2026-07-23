# 🤖 Autonomous Rover - Isaac Sim Indoor Environment

Este repositorio contiene la arquitectura, modelos y scripts necesarios para simular un rover autónomo (Clearpath Jackal) en un entorno de interiores utilizando **NVIDIA Isaac Sim** y **ROS 2**. 

El objetivo principal de este entorno es facilitar la simulación cinemática y, primordialmente, **la recolección estructurada de datasets sintéticos** (RGB, Depth, Nubes de Puntos y Segmentación Semántica) para el entrenamiento de modelos de visión artificial.

---

## 📋 1. Requisitos Previos (Requirements)

Para que el proyecto funcione correctamente, necesitas el siguiente entorno de software:

- **OS:** Ubuntu 22.04 LTS
- **Simulador:** [NVIDIA Isaac Sim 5.1.0](https://developer.nvidia.com/isaac-sim)
- **Framework Robótico:** ROS 2 Humble

### Instalación rápida de ROS 2 Humble
Si aún no tienes ROS 2 instalado en tu sistema, puedes instalar la versión Desktop abriendo una terminal y siguiendo los pasos de  [Instalación de ROS2 Humble](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

```bash
pip install "setuptools==58.2.0"
```

## ⚙️ 2. Instalación y Compilación (Building)
Es recomendable utilizar un "espacio de trabajo" (workspace) de ROS 2 para mantener todo organizado.

1. Crea el workspace y clona el repositorio:

```bash
mkdir -p ~/rover_ws/src
cd ~/rover_ws/src
git clone [https://github.com/jumasaet/isaac_indoor.git](https://github.com/jumasaet/isaac_indoor.git)
```

2. Compila el proyecto:
Vuelve a la raíz del workspace y utiliza `colcon` para construir los paquetes.

```bash
cd ~/rover_ws
colcon build
```

3. Carga el entorno en tu terminal:
Nota: Debes hacer esto en cada nueva terminal que abras, o añadirlo a tu archivo `.bashrc`.

```bash
source ~/rover_ws/install/setup.bash
```

## 🚀 3. Lanzar la Simulación (Launching)
### 3.1 Abrir NVIDIA Isaac Sim
Asegúrate de tener Isaac Sim 5.1.0 configurado. Si es tu primera vez, puedes seguir este   .[Tutorial de Instalación Workstations](https://www.youtube.com/watch?v=Ej4zVyTCXkMpara)

Abre una terminal y ejecuta el simulador desde su directorio de instalación (usualmente en la carpeta de usuario):

```bash
./isaac-sim/isaac-sim.sh
```

### 3.2 Cargar el Mundo y el Robot
Una vez que la interfaz gráfica de Isaac Sim esté abierta, debes cargar el archivo del entorno (formato .usd) que ya incluye el modelo del Jackal y el escenario de almacén:

* Ve a `File > Open`

* Navega hasta la ruta de tu repositorio y abre el siguiente archivo:

```plaintext
<ruta_a_tu_repo>/isaac_indoor/isaac_description/isaac_description/warehouse_jackal.usd
```

* Presiona el botón Play (el triángulo en la barra de la izquierda) para iniciar la física y el puente con ROS 2.


## 🎮 4. Movimiento del Rover (Teleoperación)
Para mover el robot por el entorno, el repositorio incluye un script de teleoperación por teclado que se comunica mediante el tópico /cmd_vel.

En una nueva terminal (con el workspace cargado), ejecuta:

```bash
ros2 run isaac_rover teleop_jackal
```

Controles:

* Usa W, A, S, D para navegar.

* Usa Q / E para ajustar la velocidad lineal (+ / -).

* Usa Z / C para ajustar la velocidad angular (+ / -).

Presiona cualquier otra tecla para frenar en seco.

(*Nota: Si no deseas usar ROS 2, los scripts de la carpeta fuente también pueden ejecutarse de forma independiente como scripts nativos de Python: `python3 src/isaac_indoor/isaac_rover/isaac_rover/teleop_jackal.py`)*

Si se quiere realizar un movimiento automático puedes usar como base el ejecutable `movement.py`.

## 📷 5. Pipeline de Recolección de Datos (Data Collection)
Este proyecto prioriza la extracción de datos de alta calidad de los sensores del rover. Antes de ejecutar los scripts de captura, instala las dependencias de visión y procesamiento 3D.

*⚠️ Importante: Para evitar conflictos de memoria con las librerías de ROS 2 (`cv_bridge`), es mandatorio asegurar que `numpy` esté en una versión 1.x.*

```bash
pip install "numpy<2.0.0" open3d opencv-python
```

El repositorio incluye herramientas para tomar "fotografías integrales" de todos los sensores a la vez, garantizando que estén perfectamente sincronizadas en el tiempo. Para tomar una muestra (un solo frame sincronizado), ejecuta:

```bash
ros2 run isaac_rover single_frame_collector
```

Esto creará automáticamente una carpeta llamada `isaac_dataset/` guardando:

* Imágenes RGB (`.png`)
* Mapas de Profundidad - Depth (`.npy` exactos)
* Nubes de Puntos - Point Clouds (`.pcd`)
Segmentación de Instancias (Mascaras `.npy` y previsualizaciones `.png` a color)
* Un archivo `.json` con las etiquetas semánticas del entorno.


## 📡 6. Tópicos de ROS 2 Activos
Cuando la simulación está en "Play", el rover publica información en tiempo real. Puedes verificar el estado de los sensores haciendo `ros2 topic list`.

| Topic | Tipo de Mensaje | Descripción |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/Twist` | Comandos de velocidad para mover el chasis diferencial. |
| `/odom` | `nav_msgs/Odometry` | Odometría calculada del rover. |
| `/tf` | `tf2_msgs/TFMessage` | Transformadas espaciales (kinematics tree). |
| `/rgb` | `sensor_msgs/Image` | Imagen estándar a color de la cámara principal. |
| `/depth` | `sensor_msgs/Image` | Imagen de profundidad métrica. |
| `/depth_pcl` | `sensor_msgs/PointCloud2` | Nube de puntos 3D generada por los sensores. |
| `/instance_segmentation` | `sensor_msgs/Image` | Máscara de enteros (`32SC1`) con IDs únicos por objeto. |
| `/semantic_labels` | `std_msgs/String` | Diccionario JSON que mapea los IDs de segmentación con su clase u objeto. |
| `/camera_info` | `sensor_msgs/CameraInfo` | Parámetros intrínsecos de las cámaras simuladas. |

