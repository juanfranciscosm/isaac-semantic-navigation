# Proyecto base

Este proyecto fue desarrollado a partir del repositorio:

- Repositorio: https://github.com/jumasaet/isaac_indoor
- Rama utilizada: `main`
- Commit base: `8adabf68b8fe01d1a563f4df478ebb8033cbaa66`

El repositorio original proporciona el entorno de NVIDIA Isaac Sim,
el robot Clearpath Jackal y la infraestructura inicial de comunicación
con ROS 2.

Sobre esa base se realizaron modificaciones en la escena
`warehouse_jackal.usd` y se añadieron los paquetes de segmentación,
localización tridimensional y navegación semántica.

La atribución completa y la separación entre el trabajo base y las
contribuciones de este proyecto se detallarán en el README principal.

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
