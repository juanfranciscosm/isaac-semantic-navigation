# Generación de los 60 mapas VLMaps


El objetivo de esta etapa es generar:

```text
10 objetos × 6 condiciones = 60 mapas VLMaps
```

Cada mapa será utilizado posteriormente para ejecutar 4 episodios de navegación:

```text
P1
P2
P3
P4
```

Por lo tanto, los 60 mapas permitirán ejecutar finalmente:

```text
60 mapas × 4 posiciones = 240 pruebas de navegación
```

---

# 1. Objetos y condiciones

Los 10 objetos son:

```text
boxes
cargo_vehicle
chair
computer
cylindrical_container
door
drinks_machine
electrical_panel
extinguisher
medical_bag
```

Las 6 condiciones son:

```text
I0
I25
I50
O20
O50
O80
```

Cada combinación representa un escenario independiente.

Ejemplos:

```text
boxes_I0
boxes_I25
cargo_vehicle_O50
chair_O80
medical_bag_I50
```

En total:

```text
10 × 6 = 60 escenarios
```

---

# 2. Estructura general del proceso

Para cada escenario se realizan dos etapas:

```text
ESCENA ISAAC SIM
       ↓
captura de 24 keyframes RGB-D
       ↓
dataset VLMaps
       ↓
construcción del mapa
       ↓
vlmaps_cam.h5df
```

El procedimiento completo para un escenario es:

```text
1. Cerrar Isaac Sim anterior
2. Iniciar una instancia nueva de Isaac Sim
3. Ejecutar capture_vlmaps_scenario.sh
4. Abrir el USD indicado
5. Presionar Play
6. Presionar Enter en la terminal
7. Esperar la captura automática de M001-M024
8. Verificar "DATASET VALIDO"
9. Construir el VLMap
10. Verificar vlmaps_cam.h5df
11. Pasar al siguiente escenario
```

---

# 3. Regla importante: reiniciar Isaac Sim entre escenarios

Cada escenario debe abrirse en una instancia nueva de Isaac Sim.

NO se recomienda hacer:

```text
warehouse_boxes_I0.usd
↓
cambiar de USD
↓
warehouse_boxes_I25.usd
```

dentro de la misma sesión.

Durante las pruebas se observó que al cambiar de USD en una sesión activa podían quedar diferentes referencias de tiempo entre:

```text
/clock
/rgb
/depth
/tf
```

Esto provoca errores como:

```text
Lookup would require extrapolation into the past
```

Por lo tanto, entre escenarios:

```text
cerrar Isaac Sim
↓
iniciar Isaac Sim nuevamente
↓
abrir el siguiente USD
```

---

# 4. Archivos y repositorios utilizados

Repositorio ROS / benchmark:

```text
~/indoor
```

Repositorio VLMaps:

```text
~/vlmaps_ws/vlmaps
```

Script de captura:

```text
~/indoor/tools/capture_vlmaps_scenario.sh
```

Script de construcción:

```text
~/vlmaps_ws/vlmaps/tools/build_vlmaps_scene_map.sh
```

Posiciones utilizadas para construir todos los mapas:

```text
~/indoor/evaluation/config/vlmaps_mapping_positions.yaml
```

El archivo contiene:

```text
M001
M002
...
M024
```

Todos los 60 mapas deben utilizar exactamente estas mismas 24 poses.

NO modificar estas posiciones durante la generación del benchmark.

---

# 5. Directorio donde se guardan los datasets

Todos los datasets se guardan en:

```text
~/vlmaps_data_benchmark/
```

Cada escenario genera su propio directorio.

Ejemplo:

```text
~/vlmaps_data_benchmark/boxes_I0/
```

Después de la captura tendrá aproximadamente:

```text
boxes_I0/
├── camera_info.txt
├── poses.txt
├── timestamps.txt
├── rgb/
│   ├── 000000.png
│   ├── 000001.png
│   ├── ...
│   └── 000023.png
└── depth/
    ├── 000000.npy
    ├── 000001.npy
    ├── ...
    └── 000023.npy
```

Debe haber exactamente:

```text
24 RGB
24 depth
24 poses
24 timestamps
```

---

# 6. Iniciar un escenario

## Paso 1 — Cerrar Isaac Sim anterior

Si Isaac Sim está abierto, cerrarlo completamente.

También se puede comprobar que no haya una instancia antigua con:

```bash
pgrep -af 'isaac-sim|kit'
```

Si es necesario:

```bash
pkill -f isaac-sim 2>/dev/null || true
pkill -f kit 2>/dev/null || true
```

---

## Paso 2 — Iniciar Isaac Sim

Abrir una terminal nueva:

```bash
conda deactivate 2>/dev/null || true

~/iniciar_isaac_jazzy.sh
```

Esperar a que Isaac Sim termine de cargar.

Todavía no es necesario buscar manualmente el archivo USD.

---

# 7. Iniciar la captura del escenario

Abrir otra terminal.

Ejemplo para:

```text
boxes_I0
```

ejecutar:

```bash
cd ~/indoor

./tools/capture_vlmaps_scenario.sh boxes I0
```

El formato siempre es:

```bash
./tools/capture_vlmaps_scenario.sh OBJETO CONDICION
```

Ejemplos:

```bash
./tools/capture_vlmaps_scenario.sh boxes I25
```

```bash
./tools/capture_vlmaps_scenario.sh cargo_vehicle O50
```

```bash
./tools/capture_vlmaps_scenario.sh medical_bag O80
```

---

# 8. Abrir el USD indicado

El script mostrará algo similar a:

```text
========================================
 VLMAPS MAPPING ACQUISITION
========================================
Scenario: boxes_I0

ABRE EN ISAAC SIM:
/home/robotica-cidis/indoor/evaluation/benchmark_clipseg_results/boxes/I0/scene/warehouse_boxes_I0.usd

Presiona Play y verifica que el Jackal esté cargado.
========================================

Presiona ENTER cuando Isaac esté listo...
```

Copiar la ruta mostrada por el script.

En Isaac Sim:

```text
File
→ Open
→ seleccionar el archivo USD indicado
```

La ubicación principal de los escenarios es:
```text
~/indoor/evaluation/benchmark_clipseg_results/<objeto>/<condicion>/scene/
```

Por ejemplo:
```text
~/indoor/evaluation/benchmark_clipseg_results/boxes/I0/scene/warehouse_boxes_I0.usd
```

Si el escenario no aparece en esa carpeta

Algunos escenarios también pueden encontrarse dentro de los archivos de descripción de Isaac.

Revisar:
```text
~/indoor/src/isaac_indoor/isaac_description/isaac_description/
```

Por ejemplo:
```text
~/indoor/src/isaac_indoor/isaac_description/isaac_description/warehouse_boxes_I0.usd
```
---

# 9. Presionar Play

Una vez cargada la escena:

```text
1. Verificar visualmente que aparece el Jackal.
2. Presionar Play.
3. Esperar aproximadamente 3-5 segundos.
```

No mover manualmente el robot.

No utilizar Reset Simulation.

No teletransportar manualmente el robot.

---

# 10. Iniciar la adquisición de los 24 keyframes

Volver a la terminal donde está:

```text
Presiona ENTER cuando Isaac esté listo...
```

Presionar:

```text
Enter
```

El proceso es automático.

El robot recorrerá:

```text
M001
M002
M003
...
M024
```

Para cada pose el sistema realiza:

```text
Pause
↓
teleport
↓
Play
↓
settle 3 s
↓
verificación de pose
↓
captura RGB-D
↓
TF Camera → odom
↓
escritura a disco
↓
saved_frame_count ACK
↓
siguiente pose
```

Se verá una salida similar a:

```text
START M001

Teleported to M001
Settling for 3.0 s...

M001 settled
M001: Capture armed for frame 000000
M001: capture confirmed | saved_frame_count=1
```

Luego:

```text
START M002
...
```

hasta:

```text
M024: capture confirmed | saved_frame_count=24
```

No intervenir mientras se ejecuta esta secuencia.

---

# 11. Validación automática del dataset

Al finalizar deben aparecer:

```text
M024: capture confirmed | saved_frame_count=24
```

Se debe verificar manualmente el contenido del dataset antes de construir el mapa.

Primero definir el escenario:
```text
SCENARIO="boxes_I0"
DIR="$HOME/vlmaps_data_benchmark/$SCENARIO"
```

Cambiar boxes_I0 por el escenario que se esté procesando.

Después contar los archivos:

```text
echo "========== VALIDACION DATASET =========="

echo -n "RGB: "
find "$DIR/rgb" \
  -maxdepth 1 \
  -name '*.png' \
  | wc -l

echo -n "Depth: "
find "$DIR/depth" \
  -maxdepth 1 \
  -name '*.npy' \
  | wc -l

echo -n "Poses: "
grep -cv '^[[:space:]]*$' \
  "$DIR/poses.txt"

echo -n "Timestamps: "
grep -cv '^[[:space:]]*$' \
  "$DIR/timestamps.txt"
```

El resultado correcto debe ser:
```text
RGB: 24
Depth: 24
Poses: 24
Timestamps: 24
```

---

# 12. Ubicación del dataset generado

Por ejemplo, para:

```text
boxes_I0
```

el dataset estará en:

```text
/home/robotica-cidis/vlmaps_data_benchmark/boxes_I0/
```

Para:

```text
cargo_vehicle_O50
```

estará en:

```text
/home/robotica-cidis/vlmaps_data_benchmark/cargo_vehicle_O50/
```

La regla es siempre:

```text
~/vlmaps_data_benchmark/<objeto>_<condicion>/
```

---

# 13. Construir el VLMap

Una vez que el dataset indique:

```text
DATASET VALIDO
```

abrir una terminal y ejecutar:

```bash
cd ~/vlmaps_ws/vlmaps

source ~/anaconda3/etc/profile.d/conda.sh
conda activate vlmaps

export PYTHONPATH="$PWD:${PYTHONPATH:-}"
```

Después construir el mapa.

Para:

```text
boxes_I0
```

ejecutar:

```bash
./tools/build_vlmaps_scene_map.sh boxes_I0
```

Para:

```text
cargo_vehicle_O50
```

ejecutar:

```bash
./tools/build_vlmaps_scene_map.sh cargo_vehicle_O50
```

La forma general es:

```bash
./tools/build_vlmaps_scene_map.sh <objeto>_<condicion>
```

---

# 14. Resultado esperado durante la construcción

Debe aparecer algo similar a:

```text
========================================
 BUILDING VLMAP
========================================
boxes_I0

Creating map for scene at:
/home/robotica-cidis/vlmaps_build_root/scene

Get Global Map: 100%|██████████| 24/24
100%|██████████| 24/24
```

Después:

```text
========================================
 MAPA GENERADO
========================================
```

y aparecerá el archivo:

```text
vlmaps_cam.h5df
```

---

# 15. Ubicación del mapa generado

El mapa NO se guarda dentro del repositorio VLMaps.

Se guarda dentro del dataset correspondiente.

Ejemplo:

```text
~/vlmaps_data_benchmark/boxes_I0/vlmap_cam/vlmaps_cam.h5df
```

Estructura final:

```text
boxes_I0/
├── camera_info.txt
├── poses.txt
├── timestamps.txt
├── rgb/
├── depth/
└── vlmap_cam/
    └── vlmaps_cam.h5df
```

El mapa `boxes_I0` generado durante la validación tuvo un tamaño aproximado de:

```text
237 MB
```

El tamaño de otros mapas puede variar.

---

# 16. Verificar que el mapa existe

Ejemplo:

```bash
ls -lh \
~/vlmaps_data_benchmark/boxes_I0/vlmap_cam/vlmaps_cam.h5df
```

Debe aparecer un archivo `.h5df` con tamaño mayor que cero.

También puede comprobarse con:

```bash
test -s \
~/vlmaps_data_benchmark/boxes_I0/vlmap_cam/vlmaps_cam.h5df \
&& echo "MAPA OK"
```

Resultado esperado:

```text
MAPA OK
```

---

# 17. Finalizar un escenario

Cuando exista correctamente:

```text
vlmap_cam/vlmaps_cam.h5df
```

ese escenario está terminado.

Ejemplo:

```text
boxes_I0 = COMPLETADO
```

Después:

```text
1. Cerrar Isaac Sim.
2. Iniciar una instancia nueva.
3. Ejecutar el siguiente escenario.
```

---

# 18. Qué hacer si falla una captura

El log del recorder se guarda temporalmente como:

```text
/tmp/vlmaps_recorder_<scenario>.log
```

Ejemplo:

```text
/tmp/vlmaps_recorder_boxes_I0.log
```

Para verlo:

```bash
cat /tmp/vlmaps_recorder_boxes_I0.log
```

---

## Error de sincronización temporal

Si aparece:

```text
Lookup would require extrapolation into the past
```

o existen tiempos muy diferentes entre RGB y TF, cerrar Isaac completamente y abrir una instancia nueva.

NO continuar con ese dataset.

---

# 19. Eliminar únicamente un dataset fallido

Si una captura falla, el script puede haber creado el directorio del escenario.

Antes de repetirlo hay que eliminar exclusivamente ese directorio incompleto.

Ejemplo:

```bash
rm -rf \
~/vlmaps_data_benchmark/boxes_I25
```

Después iniciar nuevamente la captura:

```bash
cd ~/indoor

./tools/capture_vlmaps_scenario.sh boxes I25
```

IMPORTANTE:

Nunca ejecutar:

```bash
rm -rf ~/vlmaps_data_benchmark
```

porque eliminaría los otros mapas ya terminados.

---

# Checklist de los 60 mapas

## boxes

```text
[x] boxes_I0
[x] boxes_I25
[x] boxes_I50
[x] boxes_O20
[x] boxes_O50
[x] boxes_O80
```

## cargo_vehicle

```text
[x] cargo_vehicle_I0
[x] cargo_vehicle_I25
[x] cargo_vehicle_I50
[x] cargo_vehicle_O20
[x] cargo_vehicle_O50
[x] cargo_vehicle_O80
```

## chair

```text
[x] chair_I0
[x] chair_I25
[x] chair_I50
[X] chair_O20
[x] chair_O50
[x] chair_O80
```

## computer

```text
[x] computer_I0
[x] computer_I25
[x] computer_I50
[x] computer_O20
[x] computer_O50
[ ] computer_O80
```

## cylindrical_container

```text
[x] cylindrical_container_I0
[x] cylindrical_container_I25
[x] cylindrical_container_I50
[ ] cylindrical_container_O20
[ ] cylindrical_container_O50
[ ] cylindrical_container_O80
```

## door

```text
[x] door_I0
[x] door_I25
[x] door_I50
[ ] door_O20
[ ] door_O50
[ ] door_O80
```

## drinks_machine

```text
[x] drinks_machine_I0
[x] drinks_machine_I25
[x] drinks_machine_I50
[ ] drinks_machine_O20
[ ] drinks_machine_O50
[ ] drinks_machine_O80
```

## electrical_panel

```text
[x] electrical_panel_I0
[x] electrical_panel_I25
[x] electrical_panel_I50
[ ] electrical_panel_O20
[ ] electrical_panel_O50
[ ] electrical_panel_O80
```

## extinguisher

```text
[x] extinguisher_I0
[x] extinguisher_I25
[x] extinguisher_I50
[ ] extinguisher_O20
[ ] extinguisher_O50
[ ] extinguisher_O80
```

## medical_bag

```text
[x] medical_bag_I0
[x] medical_bag_I25
[x] medical_bag_I50
[ ] medical_bag_O20
[ ] medical_bag_O50
[ ] medical_bag_O80
```

---
