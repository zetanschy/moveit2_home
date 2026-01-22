# Homework 1: Control de Robot Bimanual y Configuración MoveIt

## Objetivos

1. Desarrollar un script de control CLI para el robot bimanual que permita ejecutar movimientos coordinados de ambos brazos.
2. Configurar MoveIt para el robot usando MoveIt Setup Assistant.

---

## Ejercicio 1: Script de Control CLI para Robot Bimanual

### Descripción

Crear un script similar a `control_arm_cli.py` (ubicado en `single/yam_arm_gazebo/scripts/`) que permita controlar ambos brazos del robot de manera coordinada. El script debe poder ejecutar los siguientes comandos/movimientos según el comando ingresado por teclado.:

### Movimientos Requeridos

#### 1. Ambos Brazos en Home
Ambos brazos deben regresar a su posición inicial (home), donde todas las articulaciones están en 0 radianes.

#### 2. Levantar los Dos Brazos
Ambos brazos deben levantarse simultáneamente, extendiéndose hacia arriba.

#### 3. Brazo Izquierdo Levantado y Brazo Derecho al Frente
El brazo izquierdo debe estar levantado (extendido hacia arriba) mientras que el brazo derecho debe estar extendido hacia el frente.

---

## Ejercicio 2: Configuración MoveIt con Setup Assistant

### Descripción

Usar MoveIt Setup Assistant para crear la configuración de MoveIt para el robot bimanual.

---

## Entregables

### Para el Ejercicio 1:

1. **Script Python:** 
   - El archivo del script de control CLI para el robot bimanual.

2. **Video de demostración:**
   - Un video corto mostrando el funcionamiento del script.
   - El video debe mostrar:
     - La ejecución de cada uno de los 3 movimientos requeridos en Gazebo.

### Para el Ejercicio 2:

1. **Video de demostración:**
   - Un video corto mostrando:
     - Movimientos planificados y ejecutados en MoveIt (usando la interfaz de MoveIt).
     - El robot en Gazebo siguiendo los movimientos planificados por MoveIt.
     - Al menos 2-3 movimientos diferentes (por ejemplo, mover el brazo izquierdo, mover el brazo derecho, mover ambos brazos). Uno de esos movimientos debe ser abrir/cerrar algún gripper.

### Formato de Entrega

- Los dos videos cortos y el script deben estar comprimidos en un archivo zip con su respectivo nombre y apellido.