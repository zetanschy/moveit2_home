# Homework 1: Control de Robot Bimanual y Configuración MoveIt

## Objetivos

1. Desarrollar un script de control CLI para el robot bimanual RTOP que permita ejecutar movimientos coordinados de ambos brazos.
2. Configurar MoveIt para el robot RTOP usando MoveIt Setup Assistant.

---

## Ejercicio 1: Script de Control CLI para Robot Bimanual

### Descripción

Crear un script similar a `control_arm_cli.py` (ubicado en `bimanual/rtop_gazebo/scripts/`) que permita controlar ambos brazos del robot RTOP de manera coordinada. El script debe poder ejecutar los siguientes comandos/movimientos:

### Movimientos Requeridos

#### 1. Ambos Brazos en Home
Ambos brazos deben regresar a su posición inicial (home), donde todas las articulaciones están en 0 radianes.

**Ilustración:**
```
        ┌─────┐
        │  📷 │  (top camera)
        └─────┘
           │
    ┌──────┴──────┐
    │             │
    │    BASE     │
    │             │
    └───┬─────┬───┘
        │     │
    ┌───┘     └───┐
    │             │
  LEFT           RIGHT
  ARM            ARM
    │             │
    │             │
    │             │
    │             │
    └─────────────┘
  (ambos brazos extendidos hacia abajo)
```

#### 2. Levantar los Dos Brazos
Ambos brazos deben levantarse simultáneamente, extendiéndose hacia arriba.

**Ilustración:**
```
        ┌─────┐
        │  📷 │
        └─────┘
           │
    ┌──────┴──────┐
    │             │
    │    BASE     │
    │             │
    └───┬─────┬───┘
        │     │
    ┌───┘     └───┐
    │             │
  LEFT           RIGHT
    │             │
    │             │
    │             │
    │             │
    │             │
    └─────────────┘
  (ambos brazos apuntando hacia arriba)
```

#### 3. Brazo Izquierdo Levantado y Brazo Derecho al Frente
El brazo izquierdo debe estar levantado (extendido hacia arriba) mientras que el brazo derecho debe estar extendido hacia el frente.

**Ilustración:**
```
        ┌─────┐
        │  📷 │
        └─────┘
           │
    ┌──────┴──────┐
    │             │
    │    BASE     │
    │             │
    └───┬─────┬───┘
        │     │
    ┌───┘     └───┐
    │             │
  LEFT           RIGHT
    │             │
    │             │
    │             │
    │             │
    │             │
    └─────────────┘
      ↑           →
  (izquierdo      (derecho
   hacia arriba)   hacia el frente)
```

#### 4. Brazo Derecho Levantado y Brazo Izquierdo al Frente
El brazo derecho debe estar levantado (extendido hacia arriba) mientras que el brazo izquierdo debe estar extendido hacia el frente.

**Ilustración:**
```
        ┌─────┐
        │  📷 │
        └─────┘
           │
    ┌──────┴──────┐
    │             │
    │    BASE     │
    │             │
    └───┬─────┬───┘
        │     │
    ┌───┘     └───┐
    │             │
  LEFT           RIGHT
    │             │
    │             │
    │             │
    │             │
    │             │
    └─────────────┘
      →           ↑
  (izquierdo      (derecho
   hacia el        hacia arriba)
   frente)
```

### Especificaciones Técnicas

- El script debe basarse en la estructura de `control_arm_cli.py` existente.
- Debe controlar ambos brazos simultáneamente usando los controladores `left_arm_controller` y `right_arm_controller`.
- Cada brazo tiene 6 articulaciones: `{left/right}_joint1` a `{left/right}_joint6`.
- El script debe tener una interfaz CLI que permita seleccionar entre los movimientos predefinidos.
- Los valores de las articulaciones deben estar en radianes.
- El tiempo de ejecución de cada movimiento debe ser configurable.

### Estructura de Joints del Robot RTOP

Cada brazo tiene las siguientes articulaciones:
- `{left/right}_joint1`: Rotación base
- `{left/right}_joint2`: Hombro (elevación)
- `{left/right}_joint3`: Codo
- `{left/right}_joint4`: Rotación del antebrazo
- `{left/right}_joint5`: Muñeca (pitch)
- `{left/right}_joint6`: Muñeca (roll)

### Sugerencias para los Valores de Articulaciones

**Home (ambos brazos):**
- Todos los joints en `0.0` radianes

**Ambos brazos levantados:**
- `joint1`: `0.0`
- `joint2`: `π/2` (90 grados)
- `joint3`: `π` (180 grados)
- `joint4`: `0.0`
- `joint5`: `0.0`
- `joint6`: `0.0`

**Brazo levantado:**
- Mismo que "ambos brazos levantados"

**Brazo al frente:**
- `joint1`: `0.0`
- `joint2`: `π/2` (90 grados)
- `joint3`: `π/2` (90 grados)
- `joint4`: `-π/2` (-90 grados)
- `joint5`: `0.0`
- `joint6`: `0.0`

*Nota: Estos valores son sugerencias. Puedes ajustarlos según sea necesario para lograr los movimientos deseados.*

---

## Ejercicio 2: Configuración MoveIt con Setup Assistant

### Descripción

Usar MoveIt Setup Assistant para crear la configuración de MoveIt para el robot RTOP.

### Pasos a Seguir

1. **Preparar el URDF del robot:**
   - Asegúrate de tener el URDF del robot RTOP disponible.
   - El URDF debe estar en `bimanual/rtop_description/urdf/rtop.urdf` o `rtop.urdf.xacro`.

2. **Iniciar MoveIt Setup Assistant:**
   ```bash
   ros2 launch moveit_setup_assistant setup_assistant.launch.py
   ```

3. **Configurar el robot:**
   - Cargar el URDF del robot RTOP.
   - Configurar los grupos de planificación:
     - `left_arm`: desde `left_arm` hasta `left_link_6`
     - `right_arm`: desde `right_arm` hasta `right_link_6`
     - `bimanual`: grupo que incluye ambos brazos
   - Configurar los límites de las articulaciones.
   - Configurar la cinemática (puedes usar KDL o IKFast si está disponible).
   - Configurar los controladores ROS 2.
   - Configurar los estados predefinidos (opcional pero recomendado).
   - Configurar el archivo de configuración de RViz.

4. **Generar el paquete de configuración:**
   - Guardar la configuración en un directorio apropiado (por ejemplo, `rtop_moveit_config`).
   - Asegúrate de que todos los archivos de configuración se generen correctamente.

5. **Verificar la configuración:**
   - Probar que el paquete se puede compilar.
   - Verificar que los archivos de configuración están correctos.

### Archivos de Configuración Esperados

El paquete generado debe incluir:
- `config/rtop.srdf`: Archivo de descripción semántica del robot
- `config/joint_limits.yaml`: Límites de las articulaciones
- `config/kinematics.yaml`: Configuración de cinemática inversa
- `config/moveit_controllers.yaml`: Configuración de controladores
- `config/ros2_controllers.yaml`: Configuración de controladores ROS 2
- `launch/demo.launch.py`: Launch file para demostración
- Otros archivos de configuración necesarios

---

## Entregables

### Para el Ejercicio 1:

1. **Script Python:** 
   - El archivo del script de control CLI para el robot bimanual.
   - Debe estar ubicado en `bimanual/rtop_gazebo/scripts/` o en un directorio apropiado.
   - El script debe ser ejecutable y tener los comentarios necesarios.

2. **Video de demostración:**
   - Un video corto (máximo 2-3 minutos) mostrando el funcionamiento del script.
   - El video debe mostrar:
     - La ejecución de cada uno de los 4 movimientos requeridos.
     - El robot en Gazebo ejecutando los movimientos.
     - La interfaz CLI en funcionamiento.

### Para el Ejercicio 2:

1. **Video de demostración:**
   - Un video corto (máximo 2-3 minutos) mostrando:
     - Movimientos planificados y ejecutados en MoveIt (usando RViz o la interfaz de MoveIt).
     - El robot en Gazebo siguiendo los movimientos planificados por MoveIt.
     - Al menos 2-3 movimientos diferentes (por ejemplo, mover el brazo izquierdo, mover el brazo derecho, mover ambos brazos).

### Formato de Entrega

- Los scripts deben estar en un repositorio Git o en un archivo comprimido.
- Los videos deben estar en formato MP4 o similar, y deben ser accesibles (subidos a YouTube, Google Drive, o similar).
- Incluir un archivo README con instrucciones de cómo ejecutar el script y cómo reproducir los resultados.

---

## Criterios de Evaluación

### Ejercicio 1 (50 puntos):

- **Funcionalidad (30 puntos):**
  - El script ejecuta correctamente los 4 movimientos requeridos (7.5 puntos cada uno).
  
- **Código (15 puntos):**
  - Código bien estructurado y comentado (5 puntos).
  - Manejo adecuado de errores (5 puntos).
  - Interfaz CLI clara y fácil de usar (5 puntos).

- **Video (5 puntos):**
  - Video claro que muestra todos los movimientos funcionando correctamente.

### Ejercicio 2 (50 puntos):

- **Configuración MoveIt (30 puntos):**
  - Configuración completa y funcional de MoveIt (15 puntos).
  - Grupos de planificación correctamente configurados (10 puntos).
  - Controladores correctamente configurados (5 puntos).

- **Integración con Gazebo (15 puntos):**
  - MoveIt puede comunicarse con Gazebo (10 puntos).
  - Los movimientos planificados se ejecutan correctamente en Gazebo (5 puntos).

- **Video (5 puntos):**
  - Video claro que muestra MoveIt planificando y Gazebo ejecutando los movimientos.

---

## Fecha de Entrega

**Fecha límite:** [A definir por el instructor]

---

## Recursos y Referencias

- Documentación de MoveIt: https://moveit.picknik.ai/
- Documentación de ROS 2 Control: https://control.ros.org/
- Script de referencia: `bimanual/rtop_gazebo/scripts/control_arm_cli.py`
- URDF del robot: `bimanual/rtop_description/urdf/rtop.urdf.xacro`
- Configuración MoveIt existente (referencia): `bimanual/rtop_moveit_config/`

---

## Notas Adicionales

- Asegúrate de que Gazebo esté corriendo antes de ejecutar el script de control.
- Verifica que los controladores de los brazos estén activos antes de enviar comandos.
- Para el Ejercicio 2, puedes usar la configuración existente en `rtop_moveit_config_assistant` como referencia, pero debes crear tu propia configuración desde cero usando Setup Assistant.
- Si encuentras problemas con las colisiones o la cinemática, ajusta los parámetros en la configuración de MoveIt.

---

¡Buena suerte con la tarea!

