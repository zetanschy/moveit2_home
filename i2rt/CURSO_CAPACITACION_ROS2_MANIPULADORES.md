# Curso de Capacitación: Manipuladores Robóticos con ROS 2

**Duración:** 3 horas  
**Plataforma:** ROS 2 Humble  
**Robots:** YAM Arm (single y bimanual RTOP)  
**Framework:** MoveIt 2

---

## 📋 Tabla de Contenidos

1. [Objetivos del Curso](#objetivos-del-curso)
2. [Requisitos Previos](#requisitos-previos)
3. [Estructura del Curso (3 horas)](#estructura-del-curso-3-horas)
4. [Módulo 1: Introducción](#módulo-1-introducción)
5. [Módulo 2: Control y Teleoperación](#módulo-2-control-y-teleoperación)
6. [Ejercicios Prácticos](#ejercicios-prácticos)
7. [Recursos y Referencias](#recursos-y-referencias)

---

## 🎯 Objetivos del Curso

Al finalizar este curso, los participantes serán capaces de:

- Comprender la estructura y componentes de modelos URDF/Xacro para brazos robóticos
- Entender el sistema de transformaciones TF2 en ROS 2
- Implementar diferentes tipos de control (posición, velocidad, esfuerzo)
- Desarrollar sistemas de teleoperación con ROS 2
- Configurar y utilizar controladores de `ros2_control` y `joint_trajectory_controller`
- Integrar MoveIt 2 para planificación de trayectorias

---

## 📚 Requisitos Previos

- Conocimientos básicos de Linux
- Familiaridad con terminal y comandos básicos
- Conocimientos básicos de Python
- ROS 2 Humble instalado
- Workspace configurado: `/home/zetans/Desktop/moveit_isaac_ws`

### Verificación del Entorno

```bash
# Verificar ROS 2
source /opt/ros/humble/setup.bash
ros2 --version

# Verificar workspace
cd /home/zetans/Desktop/moveit_isaac_ws
colcon build --symlink-install
source install/setup.bash

# Verificar paquetes
ros2 pkg list | grep yam_arm
ros2 pkg list | grep rtop
```

---

## ⏱️ Estructura del Curso (3 horas)

| Tiempo | Módulo | Contenido |
|--------|--------|-----------|
| **0:00 - 0:15** | Introducción | Presentación y configuración del entorno |
| **0:15 - 1:00** | Módulo 1.1-1.3 | URDF/Xacro y TF2 (45 min) |
| **1:00 - 1:15** | Descanso | Pausa (15 min) |
| **1:15 - 2:30** | Módulo 2.1-2.3 | Control y Teleoperación (75 min) |
| **2:30 - 3:00** | Práctica Final | Ejercicios integrados (30 min) |

---

## 📖 Módulo 1: Introducción

### 1.1. Presentación del Curso (5 minutos)

**Objetivos:**
- Introducción a ROS 2 y manipulación robótica
- Presentación de los robots YAM (single y bimanual)
- Estructura del workspace

**Contenido Teórico:**
- ¿Qué es ROS 2?
- Arquitectura de nodos, topics, servicios y acciones
- Conceptos de manipulación robótica
- Introducción a MoveIt 2

**Práctica:**
```bash
# Explorar el workspace
cd /home/zetans/Desktop/moveit_isaac_ws/src/i2rt
tree -L 2

# Ver estructura de paquetes
ls single/
ls bimanual/
```

---

### 1.2. Descripción de Modelos URDF/Xacro de Brazos Robóticos (25 minutos)

#### Contenido Teórico (10 min)

**URDF (Unified Robot Description Format):**
- Formato XML para describir robots
- Componentes principales:
  - `<robot>`: Elemento raíz
  - `<link>`: Partes físicas del robot (eslabones)
  - `<joint>`: Conexiones entre links (articulaciones)
  - `<visual>`: Geometría visual (meshes, primitivos)
  - `<collision>`: Geometría de colisión
  - `<inertial>`: Propiedades físicas (masa, inercia)

**Xacro (XML Macros):**
- Extensión de URDF que permite:
  - Macros reutilizables (`<xacro:macro>`)
  - Variables y parámetros (`${variable}`, `$(arg param)`)
  - Inclusión de archivos (`<xacro:include>`)
  - Condicionales (`<xacro:if>`, `<xacro:unless>`)
- Ventajas: modularidad, reutilización, mantenibilidad

**Estructura del YAM Arm:**
```
yam_arm_description/
├── urdf/
│   ├── yam_arm_macro.xacro      # Macro reutilizable del brazo
│   ├── yam.urdf.xacro           # Brazo sin cámara
│   ├── yam_cam.urdf.xacro       # Brazo con cámara
│   └── yam_arm_gazebo.urdf.xacro # Configuración Gazebo
└── meshes/                       # Archivos de malla 3D
```

#### Contenido Práctico (15 min)

**Ejercicio 1.2.1: Explorar el URDF del YAM Arm**

```bash
# Ver el macro principal
cat src/i2rt/single/yam_arm_description/urdf/yam_arm_macro.xacro | head -50

# Procesar el URDF completo
ros2 run xacro xacro src/i2rt/single/yam_arm_description/urdf/yam.urdf.xacro > /tmp/yam_processed.urdf

# Ver links del robot
grep '<link name=' /tmp/yam_processed.urdf

# Ver joints del robot
grep '<joint name=' /tmp/yam_processed.urdf

# Contar DOF (grados de libertad)
grep -c '<joint name=' /tmp/yam_processed.urdf
```

**Ejercicio 1.2.2: Analizar la Estructura del Macro**

```bash
# Abrir el macro en un editor
code src/i2rt/single/yam_arm_description/urdf/yam_arm_macro.xacro

# Identificar:
# 1. Parámetros del macro (prefix, parent_link, xyz, rpy)
# 2. Links principales (arm, link_1 a link_6, gripper)
# 3. Joints (joint1 a joint6, gripper joints)
# 4. Propiedades inerciales
# 5. Geometrías visuales y de colisión
```

**Ejercicio 1.2.3: Comparar Single vs Bimanual**

```bash
# Ver macro single
cat src/i2rt/single/yam_arm_description/urdf/yam_arm_macro.xacro | grep 'xacro:macro'

# Ver uso en bimanual
cat src/i2rt/bimanual/rtop_description/urdf/rtop.urdf.xacro

# Identificar cómo se reutiliza el macro con prefijos
grep -A 5 'xacro:yam_arm' src/i2rt/bimanual/rtop_description/urdf/rtop.urdf.xacro
```

**Conceptos Clave:**
- **Links:** Representan partes físicas (eslabones)
- **Joints:** Definen cómo se mueven los links (revolute, prismatic, fixed)
- **Visual:** Cómo se ve el robot (meshes `.obj`, `.stl`)
- **Collision:** Geometría simplificada para detección de colisiones
- **Inertial:** Propiedades físicas necesarias para simulación

---

### 1.3. Representación en Coordenadas: TF2 (15 minutos)

#### Contenido Teórico (7 min)

**TF2 (Transform Library):**
- Sistema de transformaciones de coordenadas en ROS 2
- Permite relacionar diferentes frames de referencia
- Ejemplo: `base_link` → `link_1` → `link_2` → ... → `end_effector`

**Conceptos:**
- **Frame:** Sistema de coordenadas (ej: `base_link`, `link_1`)
- **Transform:** Relación entre dos frames (traslación + rotación)
- **Tree:** Árbol de transformaciones (cadena cinemática)
- **Broadcaster:** Publica transformaciones
- **Listener:** Escucha transformaciones

**TF2 en el YAM Arm:**
```
world (fixed)
  └── arm (base)
      └── link_1
          └── joint1_link
              └── link_2
                  └── ...
                      └── link_6
                          └── end_effector
```

**Nodos Clave:**
- `robot_state_publisher`: Publica TF2 desde el URDF
- `joint_state_publisher`: Publica estados de las articulaciones

#### Contenido Práctico (8 min)

**Ejercicio 1.3.1: Visualizar TF2 Tree**

```bash
# Terminal 1: Lanzar robot state publisher
ros2 launch yam_arm_description robot_state_publisher.launch.py

# Terminal 2: Ver el árbol TF2
ros2 run tf2_tools view_frames
evince frames.pdf  # Ver el árbol generado

# Ver frames disponibles
ros2 run tf2_ros tf2_echo link_1 link_2

# Monitorear transformaciones en tiempo real
ros2 run tf2_ros tf2_monitor
```

**Ejercicio 1.3.2: Inspeccionar Transformaciones**

```bash
# Ver transformación entre dos links
ros2 run tf2_ros tf2_echo arm link_6

# Ver todas las transformaciones
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once

# Ver frecuencia de publicación
ros2 topic hz /tf
ros2 topic hz /joint_states
```

**Ejercicio 1.3.3: Usar RViz para Visualizar TF2**

```bash
# Lanzar RViz con el robot
rviz2 rviz2

# En RViz:
# 1. Add → TF (ver frames)
# 2. Add → RobotModel (ver modelo)
# 3. Observar cómo se actualizan las transformaciones
```

**Comandos Útiles:**
```bash
# Listar todos los frames
ros2 run tf2_ros tf2_monitor

# Ver transformación específica
ros2 run tf2_ros tf2_echo <source_frame> <target_frame>

# Verificar si existe una transformación
ros2 run tf2_ros tf2_monitor <source_frame> <target_frame>
```

---

## 🤖 Módulo 2: Control y Teleoperación de Brazos Robóticos

### 2.1. Tipos de Control: Posición, Velocidad y Esfuerzo (25 minutos)

#### Contenido Teórico (10 min)

**Introducción a ros2_control:**

`ros2_control` es un framework para el control en tiempo real de robots utilizando ROS 2. Su objetivo principal es simplificar la integración de nuevo hardware y proporcionar una arquitectura modular y reutilizable. ([Documentación oficial](https://control.ros.org/humble/doc/ros2_control/doc/index.html))

**Componentes principales del ecosistema ros2_control:**
- **`ros2_control`**: Interfaces y componentes principales del framework
- **`ros2_controllers`**: Controladores ampliamente utilizados (joint_trajectory_controller, diff_drive_controller, etc.)
- **`control_toolbox`**: Implementaciones de teoría de control (controladores PID)
- **`realtime_tools`**: Herramientas para soporte en tiempo real (buffers, publicadores)
- **`control_msgs`**: Mensajes comunes utilizados en control

**Tipos de Control en Robótica (Hardware Interface Types):**

1. **Control de Posición (`position`):**
   - Especifica el ángulo/posición deseada de cada articulación
   - El controlador mueve la articulación a la posición objetivo
   - Uso: Movimientos precisos, trayectorias predefinidas, posicionamiento exacto
   - Command Interface: `command_interface: position`
   - State Interface: `state_interface: position` (y opcionalmente `velocity`, `effort`)

2. **Control de Velocidad (`velocity`):**
   - Especifica la velocidad angular/lineal deseada
   - El controlador ajusta continuamente la velocidad
   - Uso: Movimientos suaves, seguimiento de trayectorias, control continuo
   - Command Interface: `command_interface: velocity`
   - State Interface: `state_interface: velocity` (y opcionalmente `position`, `effort`)

3. **Control de Esfuerzo/Torque (`effort`):**
   - Especifica el torque/force deseado
   - El controlador aplica el esfuerzo directamente
   - Uso: Manipulación de objetos, control de fuerza, interacción con el entorno
   - Command Interface: `command_interface: effort`
   - State Interface: `state_interface: effort` (y opcionalmente `position`, `velocity`)

**Arquitectura de ros2_control:**

```
┌─────────────────────────────────────────┐
│         Controller Manager               │
│  (Gestiona controladores activos)        │
└──────────────┬──────────────────────────┘
               │
       ┌────────┴────────┐
       │                 │
┌──────▼──────┐  ┌───────▼────────┐
│ Controllers │  │ Hardware       │
│ (Software)  │  │ Components      │
│             │  │ (Hardware)     │
└─────────────┘  └────────────────┘
```

**Estructura de ros2_control en URDF/Xacro:**

```xml
<ros2_control name="IgnitionSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>
  <joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

**Conceptos Clave:**

- **Hardware Components**: Definen las interfaces para interactuar con el hardware del robot (real o simulado)
- **Command Interfaces**: Cómo se envía el comando al hardware (`position`, `velocity`, `effort`)
- **State Interfaces**: Qué información se lee del hardware (`position`, `velocity`, `effort`)
- **Controller Manager**: Gestiona la carga, descarga y ejecución de controladores en el sistema
- **Separation of Concerns**: Separación clara entre hardware y software permite portabilidad

#### Contenido Práctico (15 min)

**Ejercicio 2.1.1: Examinar Configuración de Control**

```bash
# Ver configuración de controladores
cat src/i2rt/single/yam_arm_moveit_config/config/ros2_controllers.yaml

# Ver interfaces de control en el URDF
grep -A 10 'ros2_control' src/i2rt/single/yam_arm_description/urdf/yam_arm_gazebo.urdf.xacro
```

**Ejercicio 2.1.2: Lanzar Simulación y Ver Controladores**

```bash
# Terminal 1: Lanzar Gazebo con controladores
ros2 launch yam_arm_gazebo yam_ctrl.gazebo.launch.py

# Terminal 2: Listar controladores disponibles
ros2 control list_controllers

# Ver información de un controlador
ros2 control list_controllers -v

# Ver interfaces de control
ros2 topic list | grep controller
ros2 topic echo /arm_controller/joint_trajectory --once
```

**Ejercicio 2.1.3: Enviar Comando de Posición Directo**

```bash
# Ver estado actual de las articulaciones
ros2 topic echo /joint_states --once

# Publicar comando de posición (ejemplo: mover joint1 a 0.5 rad)
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
{
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
  points: [
    {
      positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}"
```

**Ejercicio 2.1.4: Controlar el Gripper**

```bash
# Ver estado actual del gripper
ros2 topic echo /joint_states | grep finger

# Controlar el gripper usando la acción GripperCommand (recomendado)
# Abrir el gripper (posición 0.04 m)
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand "
{
  command: {
    position: 0.04,
    max_effort: 50.0
  }
}"

# Cerrar el gripper (posición 0.0 m)
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand "
{
  command: {
    position: 0.0,
    max_effort: 50.0
  }
}"

# Controlar el dedo derecho directamente por topic (control de posición)
# Nota: Para un gripper paralelo, el dedo derecho debe moverse en dirección opuesta al izquierdo
# Abrir: mover right_finger en dirección negativa
ros2 topic pub /right_finger_controller/commands std_msgs/msg/Float64MultiArray "
{
  data: [-0.02]
}" --once

# Cerrar: mover right_finger hacia 0
ros2 topic pub /right_finger_controller/commands std_msgs/msg/Float64MultiArray "
{
  data: [0.0]
}" --once

# Ver información de la acción del gripper
ros2 action info /gripper_controller/gripper_cmd
ros2 action list | grep gripper
```

**Ejercicio 2.1.5: Controlar Grippers Bimanuales (RTOP)**

```bash
# Terminal 1: Lanzar simulación bimanual
ros2 launch rtop_gazebo rtop_ctrl.gazebo.launch.py

# Terminal 2: Ver estado de todos los grippers
ros2 topic echo /joint_states | grep finger

# Controlar gripper izquierdo usando acción
ros2 action send_goal /left_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand "
{
  command: {
    position: 0.04,
    max_effort: 50.0
  }
}"

# Controlar gripper derecho usando acción
ros2 action send_goal /right_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand "
{
  command: {
    position: 0.04,
    max_effort: 50.0
  }
}"

# Controlar dedos derechos directamente por topic
# Nota: Para grippers paralelos, los dedos derechos se mueven en dirección opuesta
# Gripper izquierdo - dedo derecho
ros2 topic pub /left_right_finger_controller/commands std_msgs/msg/Float64MultiArray "
{
  data: [-0.02]
}" --once

# Gripper derecho - dedo derecho
ros2 topic pub /right_right_finger_controller/commands std_msgs/msg/Float64MultiArray "
{
  data: [-0.02]
}" --once

# Ver todas las acciones disponibles
ros2 action list
```

**Conceptos Clave:**
- **Command Interface:** Cómo se envía el comando (position/velocity/effort)
- **State Interface:** Qué información se lee (position/velocity/effort)
- **Controller Manager:** Gestiona la carga, descarga y ejecución de controladores
- **Hardware Component:** Conecta con el hardware real o simulado (Gazebo, hardware real, mock)
- **Update Rate:** Frecuencia a la que se actualiza el control (típicamente 100-1000 Hz)

---

### 2.2. Implementación de Teleoperación con ROS 2 y cmd_vel (25 minutos)

#### Contenido Teórico (10 min)

**Teleoperación:**
- Control remoto de un robot
- El operador envía comandos desde una interfaz
- El robot ejecuta los comandos en tiempo real

**cmd_vel (Command Velocity):**
- Topic estándar en ROS para comandos de velocidad
- Originalmente para robots móviles, adaptable a brazos
- Mensaje: `geometry_msgs/msg/Twist`

**Arquitectura de Teleoperación:**
```
Operador → Nodo Teleop → Topic cmd_vel → Nodo Controlador → Robot
```

**Alternativas para Brazos:**
- `joint_trajectory`: Comandos de trayectoria de articulaciones
- `FollowJointTrajectory` (Action): Trayectorias con feedback
- Topics personalizados: Comandos específicos del robot

#### Contenido Práctico (15 min)

**Ejercicio 2.2.1: Crear Nodo de Teleoperación Básico**

Crear archivo: `~/teleop_arm.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.current_positions = [0.0] * 6
        
        self.get_logger().info('Arm Teleop Node Started')
        self.get_logger().info('Use: w/s (joint1), a/d (joint2), q/e (joint3)')
        
    def move_joint(self, joint_idx, delta):
        """Mover una articulación por incremento"""
        self.current_positions[joint_idx] += delta
        self.send_trajectory()
        
    def send_trajectory(self):
        """Enviar trayectoria actual"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.current_positions.copy()
        point.time_from_start = Duration(sec=1, nanosec=0)
        
        msg.points = [point]
        self.publisher.publish(msg)
        self.get_logger().info(f'Positions: {self.current_positions}')

def main():
    rclpy.init()
    node = ArmTeleop()
    
    # Ejemplo interactivo (mejorar con keyboard input)
    import time
    try:
        while rclpy.ok():
            # Ejemplo: mover joint1
            node.move_joint(0, 0.1)
            time.sleep(2)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Ejercicio 2.2.2: Usar Script de Control Existente**

```bash
# El proyecto ya incluye un script de control CLI
python3 src/i2rt/single/yam_arm_gazebo/scripts/control_arm_cli.py

# Ver opciones disponibles
python3 src/i2rt/single/yam_arm_gazebo/scripts/control_arm_cli.py --help

# Enviar pose predefinida
python3 src/i2rt/single/yam_arm_gazebo/scripts/control_arm_cli.py home
python3 src/i2rt/single/yam_arm_gazebo/scripts/control_arm_cli.py ready
```

**Ejercicio 2.2.3: Teleoperación con Joystick (Opcional)**

```bash
# Instalar paquete de joystick (si está disponible)
sudo apt install ros-humble-joy

# Verificar joystick conectado
ls /dev/input/js*

# Lanzar nodo de joystick
ros2 run joy joy_node

# Ver mensajes del joystick
ros2 topic echo /joy
```

---

### 2.3. Uso de Controladores de ros2_control y joint_trajectory_controller (25 minutos)

#### Contenido Teórico (10 min)

**Controller Manager - Concepto Central:**

El **Controller Manager** es el componente central de `ros2_control` que gestiona la carga, descarga y ejecución de controladores en el sistema. ([Documentación](https://control.ros.org/humble/doc/ros2_control/doc/concepts/controller_manager.html))

**Funciones del Controller Manager:**
- Cargar controladores desde archivos de configuración
- Activar/desactivar controladores dinámicamente
- Gestionar recursos compartidos (joints, interfaces)
- Coordinar múltiples controladores
- Proporcionar interfaz CLI y programática

**Arquitectura del Controller Manager:**
```
Controller Manager
├── Hardware Interface (Gazebo/Real Hardware)
│   ├── Command Interfaces (position/velocity/effort)
│   └── State Interfaces (position/velocity/effort)
└── Controllers
    ├── JointTrajectoryController
    ├── JointStateBroadcaster
    └── GripperActionController
```

**Tipos de Controladores en ros2_controllers:**

1. **JointTrajectoryController** (`joint_trajectory_controller`):
   - Control de posición de múltiples articulaciones
   - Recibe trayectorias de articulaciones
   - Interpola entre puntos de la trayectoria suavemente
   - Soporta acciones (`FollowJointTrajectory`) y topics (`joint_trajectory`)
   - Ideal para brazos robóticos y manipuladores
   - Permite configurar constraints, gains (PID), y tolerancias

2. **GripperActionController** (`position_controllers/GripperActionController`):
   - Control de gripper/garra
   - Acción: `GripperCommand`
   - Maneja apertura/cierre sincronizado de dedos
   - Útil para manipulación de objetos

3. **JointStateBroadcaster** (`joint_state_broadcaster`):
   - Publica estado de todas las articulaciones
   - Topic: `/joint_states`
   - Necesario para que otros componentes conozcan el estado del robot
   - Se ejecuta siempre (no requiere activación explícita)

**Configuración Completa del Controlador:**

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz - Frecuencia de actualización del control

arm_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 50.0  # Hz - Frecuencia de publicación de estado
    action_monitor_rate: 20.0  # Hz - Frecuencia de monitoreo de acciones
    allow_partial_joints_goal: false  # Requiere todas las articulaciones
    constraints:
      stopped_velocity_tolerance: 0.01  # Tolerancia de velocidad para "detenido"
      goal_time: 0.0  # Tiempo objetivo (0 = usar tiempo de la trayectoria)
      joint1:
        trajectory: 0.05  # Tolerancia durante la trayectoria (rad)
        goal: 0.02        # Tolerancia en el objetivo (rad)
    # Opcional: Gains PID por articulación
    gains:
      joint1:
        p: 100.0
        i: 10.0
        d: 1.0
```

**Controller Chaining / Cascade Control:**

`ros2_control` soporta encadenamiento de controladores, permitiendo conectar múltiples controladores en cascada para lograr comportamientos complejos. Por ejemplo:
- Un controlador de velocidad que recibe comandos de un controlador de posición
- Controladores de bajo nivel que ejecutan comandos de controladores de alto nivel

**Comandos CLI del Controller Manager:**

```bash
# Listar controladores disponibles
ros2 control list_controllers

# Listar interfaces de hardware
ros2 control list_hardware_interfaces

# Activar un controlador
ros2 control set_controller_state <controller_name> start

# Desactivar un controlador
ros2 control set_controller_state <controller_name> stop

# Cargar un controlador
ros2 control load_controller <controller_name>

# Descargar un controlador
ros2 control unload_controller <controller_name>
```

#### Contenido Práctico (15 min)


**Ejercicio 2.3.1b: Interactuar con Controller Manager**

```bash
# Terminal 1: Lanzar simulación
ros2 launch yam_arm_gazebo yam_ctrl.gazebo.launch.py

# Terminal 2: Listar controladores y su estado
ros2 control list_controllers

# Ver información detallada
ros2 control list_controllers -v

# Listar interfaces de hardware disponibles
ros2 control list_hardware_interfaces

# Ver parámetros del controller manager
ros2 param list /controller_manager
ros2 param get /controller_manager update_rate
```

**Ejercicio 2.3.2: Usar Action Interface**

```bash
# Terminal 1: Lanzar simulación
ros2 launch yam_arm_gazebo yam_ctrl.gazebo.launch.py

# Terminal 2: Ver acción disponible
ros2 action list
ros2 action info /arm_controller/follow_joint_trajectory

# Enviar goal usando acción para el brazo
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
{
  trajectory: {
    joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
    points: [
      {
        positions: [0.5, -0.5, 0.5, 0.0, 0.0, 0.0],
        time_from_start: {sec: 3, nanosec: 0}
      }
    ]
  }
}"

# Info action
ros2 action info /gripper_controller/gripper_cmd

# Enviar comando el gripper
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand "
{
  command: {
    position: 0.5,
    max_effort: 50.0
  }
}"

**Ejercicio 2.3.3: Usar Script de Prueba**

```bash
# Ejecutar secuencia de movimientos predefinida
python3 src/i2rt/single/yam_arm_gazebo/scripts/test_arm_movement.py

# Ver código del script
cat src/i2rt/single/yam_arm_gazebo/scripts/test_arm_movement.py
```

**Conceptos Clave:**
- **Action vs Topic:** 
  - Actions proporcionan feedback (goal, result, feedback) y cancelación
  - Topics son unidireccionales y más simples
  - Para trayectorias, Actions son preferibles por el feedback
- **Trayectoria:** Secuencia de puntos (posición, velocidad, aceleración, tiempo)
- **Interpolación:** El controlador calcula posiciones intermedias entre puntos
- **Timing:** `time_from_start` especifica cuándo alcanzar cada punto
- **Constraints:** Límites de tolerancia para validar el éxito de la trayectoria
- **Update Rate:** Frecuencia a la que el controlador actualiza los comandos (100-1000 Hz típicamente)

---

## 🎓 Ejercicios Prácticos

### Ejercicio Integrado 1: Control Manual de Articulaciones

**Objetivo:** Crear un script que permita controlar cada articulación individualmente.

**Tareas:**
1. Crear un script Python que lea entrada del teclado
2. Mapear teclas a articulaciones (ej: 1-6 para joint1-6)
3. Permitir incremento/decremento con +/- o flechas
4. Enviar comandos al controlador

**Solución Base:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys, select, termios, tty

class ManualControl(Node):
    def __init__(self):
        super().__init__('manual_control')
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.positions = [0.0] * 6
        self.settings = termios.tcgetattr(sys.stdin)
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def send_command(self):
        msg = JointTrajectory()
        msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.positions.copy()
        point.time_from_start = Duration(sec=1, nanosec=0)
        msg.points = [point]
        self.pub.publish(msg)
        self.get_logger().info(f'Joints: {[f"{p:.2f}" for p in self.positions]}')
        
    def run(self):
        print("Manual Control - Press 1-6 to select joint, +/- to move, q to quit")
        while rclpy.ok():
            key = self.get_key()
            if key == 'q':
                break
            elif key in '123456':
                idx = int(key) - 1
                print(f"Selected joint{idx+1}")
            elif key == '+':
                # Incrementar último joint seleccionado
                pass
            elif key == '-':
                # Decrementar
                pass
            self.send_command()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    rclpy.init()
    node = ManualControl()
    node.run()
    rclpy.shutdown()
```

---

### Ejercicio Integrado 2: Secuencia de Movimientos Predefinidos

**Objetivo:** Crear un programa que ejecute una secuencia de poses predefinidas.

**Tareas:**
1. Definir 5-10 poses del robot (home, ready, pick, place, etc.)
2. Crear función que mueva el robot a cada pose
3. Ejecutar secuencia con pausas entre movimientos
4. Agregar verificación de éxito

**Referencia:** Ver `test_arm_movement.py` existente

---

### Ejercicio Integrado 3: Integración MoveIt + Control Manual

**Objetivo:** Combinar planificación de MoveIt con control directo.

**Tareas:**
1. Lanzar MoveIt con Gazebo
2. Usar MoveIt para planificar una trayectoria
3. Verificar que se ejecuta correctamente
4. Comparar con control directo del ejercicio 1
