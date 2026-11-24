# Resultados del benchmark de navegación semántica

Este documento resume los resultados del benchmark experimental del sistema de navegación semántica desarrollado con **Isaac Sim, ROS 2, CLIPSeg, localización 3D y Nav2**.

El objetivo del benchmark fue evaluar el comportamiento del sistema completo frente a variaciones de **iluminación** y **oclusión geométrica**, midiendo no solo si el objeto era detectado, sino también si la detección correspondía a la instancia correcta y si el robot lograba llegar a la región objetivo.

---

## 1. Diseño del benchmark

El benchmark está compuesto por:

- **10 objetos**
- **6 condiciones experimentales**
- **4 posiciones iniciales por condición**
- **240 episodios en total**

Las condiciones evaluadas fueron:

| Condición | Descripción |
|---|---|
| `I0` | Iluminación nominal |
| `I25` | Reducción moderada de iluminación |
| `I50` | Reducción mayor de iluminación |
| `O20` | Oclusión geométrica nominal del 20 % |
| `O50` | Oclusión geométrica nominal del 50 % |
| `O80` | Oclusión geométrica nominal del 80 % |

Las condiciones de oclusión se implementaron de forma **bottom-up**, mediante oclusores visuales sin física, manteniendo la iluminación nominal.

Los objetos evaluados fueron:

- `boxes`
- `cargo_vehicle`
- `chair`
- `computer`
- `cylindrical_container`
- `door`
- `drinks_machine`
- `electrical_panel`
- `extinguisher`
- `medical_bag`

---

## 2. Criterios de evaluación

Para evitar considerar como éxito una navegación hacia un objetivo incorrecto, se utilizaron tres niveles de evaluación.

### 2.1 Detección

Un episodio cuenta como detección cuando:

```text
detection_success = 1
```

Esto indica que el sistema produjo una detección utilizable dentro del tiempo permitido.

### 2.2 Percepción semánticamente correcta

Una percepción se considera correcta cuando:

```text
detection_success = 1
AND
target_correct = 1
```

Este criterio evita contabilizar como correcta una detección que corresponda a otra instancia u otra región del escenario.

### 2.3 Éxito end-to-end estricto

La métrica principal del benchmark se define como:

```text
detection_success = 1
AND
target_correct = 1
AND
goal_region_reached = 1
```

Por lo tanto, el robot debe:

1. detectar un objetivo;
2. seleccionar/localizar la instancia correcta;
3. navegar hasta la región objetivo establecida.

Este criterio es más estricto que utilizar únicamente `navigation_status = SUCCEEDED`, ya que Nav2 puede finalizar una acción aunque el objetivo semántico haya sido incorrecto o el robot haya quedado fuera de la tolerancia final de **0.25 m**.

---

## 3. Resultado global

De los **240 episodios**:

| Métrica | Episodios | Tasa |
|---|---:|---:|
| Detección | 200 / 240 | **83.3 %** |
| Percepción semánticamente correcta | 159 / 240 | **66.3 %** |
| Éxito end-to-end estricto | 128 / 240 | **53.3 %** |
| `NO_DETECTION` | 40 / 240 | **16.7 %** |
| `WRONG_TARGET_INSTANCE` | 41 / 240 | **17.1 %** |
| `SUCCEEDED_OUTSIDE_GOAL_REGION` | 28 / 240 | **11.7 %** |
| `TIMEOUT` después de percepción correcta | 3 / 240 | **1.3 %** |

El desglose por etapa queda así:

| Resultado final | Episodios | Porcentaje |
|---|---:|---:|
| Éxito end-to-end | 128 | **53.3 %** |
| Fallo de percepción | 81 | **33.8 %** |
| Fallo de navegación | 31 | **12.9 %** |

Entre los **159 episodios con percepción correcta**, el robot llegó correctamente a la región objetivo en:

```text
128 / 159 = 80.5 %
```

Esto indica que, una vez obtenido un objetivo semánticamente correcto, el subsistema de navegación completó correctamente la mayor parte de los episodios.

---

## 4. Resultados por condición

| Condición | Detección | Percepción correcta | Éxito E2E | Error 3D medio* |
|---|---:|---:|---:|---:|
| `I0` | 95.0 % | 85.0 % | **70.0 %** | 0.450 m |
| `I25` | 95.0 % | 87.5 % | **72.5 %** | 0.438 m |
| `I50` | 92.5 % | 77.5 % | **62.5 %** | 0.479 m |
| `O20` | 90.0 % | 77.5 % | **60.0 %** | 0.444 m |
| `O50` | 72.5 % | 45.0 % | **37.5 %** | 0.573 m |
| `O80` | 55.0 % | 25.0 % | **17.5 %** | 0.713 m |

\* El error 3D medio corresponde a `localization_error_3d_center_m` calculado únicamente sobre detecciones semánticamente correctas.

### Interpretación

Las tres condiciones de iluminación mantuvieron una tasa de detección superior al **92 %**. La reducción de iluminación de `I0` a `I50` produjo una degradación moderada del desempeño end-to-end.

La oclusión tuvo un efecto considerablemente mayor. Al aumentar de `O20` a `O80`:

- la detección cayó de **90.0 %** a **55.0 %**;
- la percepción correcta cayó de **77.5 %** a **25.0 %**;
- el éxito end-to-end cayó de **60.0 %** a **17.5 %**;
- el error 3D medio aumentó de aproximadamente **0.44 m** a **0.71 m**.

Por lo tanto, dentro de este benchmark, el sistema mostró mayor estabilidad frente a cambios de iluminación que frente a niveles crecientes de oclusión.

---

## 5. Iluminación vs. oclusión

Agrupando las condiciones:

| Grupo | Episodios | Detección | Percepción correcta | Éxito E2E |
|---|---:|---:|---:|---:|
| Iluminación (`I0`, `I25`, `I50`) | 120 | **94.2 %** | **83.3 %** | **68.3 %** |
| Oclusión (`O20`, `O50`, `O80`) | 120 | **72.5 %** | **49.2 %** | **38.3 %** |

La diferencia entre ambos grupos muestra que la oclusión afecta especialmente a la etapa de percepción semántica.

En condiciones de oclusión alta, el sistema presenta dos comportamientos principales:

1. deja de generar una detección válida (`NO_DETECTION`);
2. genera una detección, pero selecciona/localiza una instancia incorrecta (`WRONG_TARGET_INSTANCE`).

---

## 6. Resultados por objeto

| Objeto | Detección | Percepción correcta | Éxito E2E |
|---|---:|---:|---:|
| `medical_bag` | 22/24 (91.7 %) | 22/24 (91.7 %) | **22/24 (91.7 %)** |
| `electrical_panel` | 22/24 (91.7 %) | 18/24 (75.0 %) | **18/24 (75.0 %)** |
| `computer` | 18/24 (75.0 %) | 18/24 (75.0 %) | **16/24 (66.7 %)** |
| `boxes` | 24/24 (100 %) | 16/24 (66.7 %) | **15/24 (62.5 %)** |
| `cargo_vehicle` | 24/24 (100 %) | 15/24 (62.5 %) | **15/24 (62.5 %)** |
| `door` | 22/24 (91.7 %) | 16/24 (66.7 %) | **14/24 (58.3 %)** |
| `chair` | 16/24 (66.7 %) | 13/24 (54.2 %) | **9/24 (37.5 %)** |
| `extinguisher` | 8/24 (33.3 %) | 8/24 (33.3 %) | **7/24 (29.2 %)** |
| `cylindrical_container` | 20/24 (83.3 %) | 9/24 (37.5 %) | **6/24 (25.0 %)** |
| `drinks_machine` | 24/24 (100 %) | 24/24 (100 %) | **6/24 (25.0 %)** |

Los objetos presentan comportamientos claramente diferentes, lo que permite distinguir varios tipos de fallo.

---

## 7. Patrones observados por objeto

### `medical_bag`

Fue el objeto con el comportamiento más estable del benchmark.

- 22 detecciones correctas de 24 episodios.
- 22 éxitos end-to-end.
- Los dos fallos corresponden a ausencia de detección bajo `O80`.

El resultado sugiere una representación semántica consistente en la mayoría de puntos de vista y condiciones evaluadas.

### `computer`

Las 18 detecciones obtenidas fueron semánticamente correctas.

El patrón dominante fue una pérdida de detección al aumentar la oclusión:

- funcionamiento estable en iluminación;
- cuatro detecciones correctas en `O20`;
- solo dos detecciones en `O50`;
- ninguna detección en `O80`.

En este caso, la degradación está asociada principalmente al recall bajo oclusión y no a confusión de instancia.

### `cargo_vehicle`

El sistema generó detección en los **24 episodios**, pero solo 15 fueron semánticamente correctas.

Esto muestra un patrón diferente al de `computer`: el problema principal no fue la ausencia de detección, sino la selección/localización de una región o instancia incorrecta, especialmente bajo oclusión severa.

### `extinguisher`

Solo se obtuvieron detecciones en **8 de 24 episodios**. Sin embargo, todas las detecciones obtenidas correspondieron al objetivo correcto.

Por ello, el principal problema de este objeto fue el **recall**, más que la confusión semántica.

### `cylindrical_container`

Se obtuvieron 20 detecciones, pero solo 9 correspondieron correctamente a la instancia esperada.

El comportamiento indica una alta sensibilidad a la selección de instancia y a regiones visualmente similares del entorno.

### `drinks_machine`

Este objeto presenta uno de los resultados más relevantes para separar percepción y navegación:

- 24/24 detecciones;
- 24/24 percepciones semánticamente correctas;
- solo 6/24 éxitos end-to-end.

Los fallos restantes corresponden principalmente a casos donde Nav2 reportó `SUCCEEDED`, pero el robot terminó fuera de la región objetivo de 0.25 m.

Por lo tanto, para este objeto el cuello de botella no está en CLIPSeg, sino en etapas posteriores relacionadas con la generación del objetivo y/o el criterio geométrico de llegada.

### `chair`

El desempeño es aceptable bajo iluminación nominal y reducida, pero disminuye rápidamente al incrementar la oclusión.

- `O20`: 3 detecciones de 4.
- `O50`: 1 detección de 4.
- `O80`: 0 detecciones de 4.

Este objeto muestra una relación clara entre porcentaje visible y probabilidad de detección.

---

## 8. Distribución de fallos

Los 112 episodios que no cumplieron el criterio end-to-end se distribuyen de la siguiente forma:

| Tipo de fallo | Cantidad |
|---|---:|
| `WRONG_TARGET_INSTANCE` | **41** |
| `NO_DETECTION` | **40** |
| `SUCCEEDED_OUTSIDE_GOAL_REGION` | **28** |
| `TIMEOUT` | **3** |
| **Total** | **112** |

Esto permite identificar dos niveles principales de mejora.

### Percepción

Representa **81 de los 112 fallos**:

```text
40 NO_DETECTION
+ 41 WRONG_TARGET_INSTANCE
= 81 fallos de percepción
```

Las líneas futuras más importantes en esta etapa son:

- mejorar la selección de la instancia correcta;
- aumentar la robustez ante oclusiones parciales;
- estudiar prompts más discriminativos;
- incorporar información temporal o multi-view;
- filtrar detecciones inconsistentes antes de generar el goal.

### Navegación

Existen **31 fallos posteriores a una percepción correcta**:

```text
28 SUCCEEDED_OUTSIDE_GOAL_REGION
+ 3 TIMEOUT
= 31 fallos de navegación
```

Estos casos justifican estudiar:

- cómo se calcula el goal a partir del centroide 3D;
- la distancia deseada respecto al objeto;
- la geometría y dimensiones del objetivo;
- el criterio de tolerancia final;
- la diferencia entre el criterio interno de éxito de Nav2 y el criterio del benchmark.

---

## 9. Sobre `mask_iou`

La métrica `mask_iou` no está disponible para todo el benchmark.

Solo **90 de los 240 episodios** contienen un valor de IoU. En particular, no existen valores registrados para:

- `chair`
- `computer`
- `door`
- `drinks_machine`
- `medical_bag`

Por esta razón, `mask_iou` **no debe utilizarse como métrica global principal** para comparar los 10 objetos.

Puede utilizarse como análisis complementario únicamente en el subconjunto de episodios donde fue registrado.

---

## 10. Conclusiones principales

Los resultados del benchmark permiten extraer las siguientes conclusiones dentro del escenario experimental evaluado:

1. **El sistema completó exitosamente 128 de 240 episodios (53.3 %) bajo un criterio end-to-end estricto.**
2. **La percepción fue el principal origen de fallo**, acumulando 81 episodios frente a 31 fallos posteriores de navegación.
3. **La iluminación tuvo un impacto moderado**: el éxito end-to-end fue de 70.0 %, 72.5 % y 62.5 % para `I0`, `I25` e `I50`, respectivamente.
4. **La oclusión produjo una degradación mucho mayor**, reduciendo el éxito end-to-end desde 60.0 % en `O20` hasta 17.5 % en `O80`.
5. El comportamiento depende fuertemente del objeto. Algunos fallan principalmente por **recall**, otros por **confusión de instancia**, y otros por la **navegación posterior a una percepción correcta**.
6. Cuando la percepción fue correcta, el robot alcanzó la región objetivo en **128 de 159 casos (80.5 %)**.
7. Utilizar únicamente `navigation_status = SUCCEEDED` sobreestimaría el desempeño del sistema, por lo que el criterio end-to-end utilizado resulta más representativo.

En conjunto, el benchmark muestra que la arquitectura es capaz de realizar navegación semántica completa en una proporción significativa de los escenarios, pero también identifica claramente la **robustez frente a oclusión y la correcta selección de instancia** como las principales áreas de mejora.

---

## 11. Dataset de resultados

El archivo maestro contiene una fila por episodio e incluye, entre otras, las siguientes variables:

```text
episode_id
position_id
object
condition
detection_success
target_error_m
target_correct
localization_error_3d_center_m
localization_error_3d_bbox_m
mask_iou
navigation_status
navigation_time_sim_s
navigation_time_real_s
distance_traveled_m
final_goal_error_m
goal_region_reached
navigation_timeout
failure_stage
failure_reason
```

La combinación de estas métricas permite estudiar por separado:

- percepción semántica;
- localización 3D;
- generación de objetivos;
- navegación;
- desempeño end-to-end.

---

## 12. Estado del benchmark

```text
Objetos:      10 / 10
Condiciones:   6 / 6
Episodios:   240 / 240
Progreso:    100 %
```

El benchmark experimental queda **cerrado** y los resultados pueden utilizarse para la etapa de análisis, generación de figuras y redacción del paper.
