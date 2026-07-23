# Navegación semántica en Isaac Sim con ROS 2

## Trabajo base y contribuciones realizadas

Este proyecto fue desarrollado tomando como base el repositorio
[`jumasaet/isaac_indoor`](https://github.com/jumasaet/isaac_indoor),
que proporciona el entorno de simulación en NVIDIA Isaac Sim, el modelo
del rover Clearpath Jackal, los archivos USD, herramientas de
teleoperación y la publicación de datos sensoriales mediante ROS 2.

### Componentes provenientes del proyecto base

- Entorno interior simulado en NVIDIA Isaac Sim.
- Modelo del robot Clearpath Jackal.
- Paquetes `isaac_description` e `isaac_rover`.
- Publicación de imágenes RGB.
- Publicación de imágenes de profundidad.
- Generación de nube de puntos.
- Publicación de odometría y transformaciones TF.
- Control del robot mediante `/cmd_vel`.

### Extensiones desarrolladas en este trabajo

- Adaptación de Isaac Sim 5.1.0 para ROS 2 Jazzy sobre Ubuntu 24.04.
- Configuración del ROS 2 Bridge con Fast DDS.
- Integración del modelo CLIPSeg para segmentación mediante texto.
- Creación del paquete `clipseg_ros`.
- Proyección de la máscara semántica sobre la imagen de profundidad.
- Cálculo de la posición tridimensional del objeto.
- Transformación del objeto desde el frame de la cámara hacia `odom`.
- Estabilización de la posición detectada.
- Generación automática de una meta frente al objeto.
- Creación del paquete `semantic_localizer`.
- Creación del paquete `semantic_navigator`.
- Integración y configuración de Nav2 sin mapa estático.
- Uso de costmaps rodantes basados en la nube `/depth_pcl`.
- Configuración de MPPI y Rotation Shim Controller.
- Navegación autónoma del Jackal hacia un objeto indicado mediante texto.