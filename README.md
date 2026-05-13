````md id="g2g5vx"
# Visual Servoing para Puzzlebot con Sampled MPC

Este proyecto implementa un sistema de navegación autónoma basada en visión para la plataforma Puzzlebot utilizando un controlador predictivo Sampled Model Predictive Control (MPC) calibrado experimentalmente.

El robot detecta cajas de colores utilizando una cámara RGB monocular, estima la distancia relativa y el error angular directamente desde características visuales de la imagen, y se aproxima autónomamente al objetivo mediante visual servoing.

Después de alcanzar la distancia deseada, el robot navega hacia un waypoint de entrega y finalmente regresa a su posición inicial utilizando control basado en odometría.

---

# Descripción General del Sistema

El sistema integra:

- Percepción visual usando OpenCV
- Estimación relativa del estado $(\rho,\alpha)$
- Sampled Model Predictive Control (MPC)
- Odometría diferencial
- Máquina de estados finitos (FSM)
- Navegación hacia waypoints y regreso a home

Flujo principal de la misión:

```text
WAIT_COMMAND
    ↓
TRACK_TARGET (Visual Servoing + MPC)
    ↓
COLLECTING
    ↓
GO_TO_WAYPOINT
    ↓
RETURN_HOME
    ↓
WAIT_COMMAND
````

---

# Estructura del Repositorio

```text
.
├── teleop.py
├── puzzlebot_odometry.py
├── mpc_hw.py
├── calibration_node.py
├── analyze_calibration.py
├── mpc_results.csv
└── README.md
```

## Descripción de Archivos

### `teleop.py`

Interfaz de teclado utilizada para enviar comandos al robot:

* seleccionar color objetivo,
* enviar waypoints,
* regresar a home,
* cancelar misiones.

---

### `puzzlebot_odometry.py`

Nodo ROS2 encargado de calcular la odometría diferencial utilizando las velocidades de las ruedas medidas por encoders.

Publica:

```text
/odom
```

---

### `mpc_hw.py`

Nodo principal del proyecto.

Implementa:

* detección de color,
* selección de objetivo,
* estimación de distancia y ángulo,
* Sampled MPC,
* máquina de estados finitos,
* navegación hacia waypoint,
* controlador de regreso,
* almacenamiento de datos en CSV.

---

### `calibration_node.py`

Nodo utilizado para generar datos experimentales para la calibración del modelo predictivo.

Realiza:

* pruebas lineales,
* pruebas angulares,
* calibración de distancia focal,
* registro de datos visuales.

---

### `analyze_calibration.py`

Script offline utilizado para estimar:

* $K_\rho$
* $K_\alpha$

del modelo predictivo utilizado por el MPC.

---

# Dependencias

## ROS2

Probado en:

```text
ROS2 Humble
```

---

## Librerías de Python

Instalar dependencias:

```bash
pip install opencv-python numpy pandas transforms3d
```

---

# Topics ROS2

## Topics Suscritos

| Topic                | Tipo                | Descripción          |
| -------------------- | ------------------- | -------------------- |
| `/video_source/raw`  | `sensor_msgs/Image` | Stream de cámara RGB |
| `/odom`              | `nav_msgs/Odometry` | Odometría del robot  |
| `/box_color_command` | `std_msgs/String`   | Comandos del usuario |

---

## Topics Publicados

| Topic      | Tipo                  | Descripción             |
| ---------- | --------------------- | ----------------------- |
| `/cmd_vel` | `geometry_msgs/Twist` | Comandos de velocidad   |
| `/odom`    | `nav_msgs/Odometry`   | Pose estimada del robot |

---

# Cómo Ejecutar

---

## 1. Ejecutar Nodo de Odometría

```bash
python3 puzzlebot_odometry.py
```

Este nodo estima la pose del robot utilizando las velocidades de las ruedas.

---

## 2. Ejecutar Nodo Principal MPC

```bash
python3 mpc_hw.py
```

Este nodo ejecuta:

* percepción visual,
* controlador MPC,
* FSM,
* navegación,
* sistema de logging.

---

## 3. Ejecutar Interfaz de Comandos

```bash
python3 teleop.py
```

Esto abre la interfaz de teclado para enviar comandos al robot.

---

# Comandos de Teleoperación

| Tecla   | Acción               |
| ------- | -------------------- |
| `g`     | Seguir caja verde    |
| `p`     | Seguir caja rosa     |
| `y`     | Seguir caja amarilla |
| `h`     | Regresar a home      |
| `w x y` | Ir a waypoint        |
| `c`     | Cancelar misión      |
| `q`     | Salir                |

Ejemplo:

```text
w 1.5 2.0
```

envía al robot al waypoint:

```text
(1.5 , 2.0)
```

---

# Modelo Predictivo MPC

El controlador opera sobre el estado visual relativo:

$$
x = [\rho,\alpha]^T
$$

donde:

* $\rho$ = distancia estimada al objetivo,
* $\alpha$ = error angular relativo.

El modelo predictivo calibrado es:

$$
\rho_{k+1} =
\rho_k -
K_{\rho} v_k \cos(\alpha_k)\Delta t
$$

$$
\alpha_{k+1} =
\alpha_k -
K_{\alpha}\omega_k\Delta t
$$

Las entradas de control son:

$$
u = [v,\omega]^T
$$

El controlador evalúa múltiples acciones candidatas dentro de un horizonte finito de predicción y selecciona la acción con menor costo.

---

# Procedimiento de Calibración

Los parámetros del modelo MPC fueron calibrados experimentalmente mediante pruebas controladas de movimiento.

---

## 1. Ejecutar Nodo de Calibración

```bash
python3 calibration_node.py
```

---

## 2. Ejecutar Pruebas de Calibración

Comandos disponibles:

| Tecla | Acción                      |
| ----- | --------------------------- |
| `k`   | Calibrar distancia focal    |
| `a`   | Movimiento angular positivo |
| `d`   | Movimiento angular negativo |
| `v`   | Movimiento lineal           |
| `x`   | Detener robot               |

---

## 3. Generar Datos de Calibración

El nodo genera:

```text
calibration_data.csv
```

---

## 4. Analizar Datos de Calibración

```bash
python3 analyze_calibration.py
```

Este script estima:

* $K_\rho$
* $K_\alpha$

utilizados por el modelo predictivo.

---

# Datos Experimentales Registrados

Durante la ejecución, el nodo MPC almacena datos experimentales en:

```text
mpc_results.csv
```

El archivo incluye:

| Variable          | Descripción            |
| ----------------- | ---------------------- |
| `rho`             | Distancia estimada     |
| `alpha`           | Error angular          |
| `v_cmd`           | Velocidad lineal       |
| `w_cmd`           | Velocidad angular      |
| `cost`            | Costo MPC              |
| `x,y,theta`       | Pose odométrica        |
| `state`           | Estado de la FSM       |
| `target_detected` | Indicador de detección |

---

# Resultados

El sistema fue validado experimentalmente sobre un Puzzlebot real.

El robot logró realizar exitosamente:

* seguimiento visual,
* aproximación autónoma,
* navegación hacia waypoint,
* regreso a home,
* múltiples ciclos completos de misión.

La implementación también incluye:

* acciones de control suaves,
* velocidades acotadas,
* coordinación mediante FSM,
* almacenamiento de datos experimentales.

---

# Ejemplo de Trayectoria

Agregar aquí imagen de trayectoria:

```md
![Trayectoria](01_trajectory_xy.png)
```

---

# Mejoras Futuras

Posibles extensiones futuras:

* evasión de obstáculos,
* generación dinámica de waypoints,
* implementación completa de IBVS,
* integración con SLAM,
* integración con manipulador,
* MPC no lineal,
* planeación multiobjetivo.

---

# Reporte

Este repositorio contiene únicamente la implementación y el pipeline de ejecución.

Para detalles sobre:

* formulación matemática,
* diseño del controlador,
* análisis experimental,
* modelado del sistema,

consultar el reporte completo del proyecto.

---

# Autor

José Eduardo Sánchez Martínez

```
```
