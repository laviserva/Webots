# README
Este proyecto fue realizado en noviembre 2021. Se hizo un simil a un robot que *resuelve laberintos* utilizando una librería básica sin ningun tipo de aplicación de teoría de control, una ventana para seleccionar punto de partida y punto final, se implementaron algoritmos basados en grafos para obtener el camino mas corto y optimo del punto de inicio al punto de final, el algoritmo seleccionado puede ser grassfire o A*. tiempo de entrega 3 semanas.

## controllers
Dentro de esta carpeta están los siguientes archivos con su propia funcionalidad.

    *librería-py*: Se encuentran todas las funcionalidades utiles para el desempeño propio del robot.
    *Roberto_Robato_Laberinto.py*: Es el controlador propio del robot, la parte main que ejecuta el resto de funciones
    *mapa/antiguo.txt*: mapas de prueba donde se testearon algunos comportamientos del robot a nivel de algoritmos, como grassfire o A*.
    *ventana.py*: GUI: Ventana para seleccionar punto de inicio y punto final del cual se calcularía algoritmo A*

# Funciones
### librería
**Traslación**

 * *desplazamiento*: desplaza el robot
    * *detener_base*: detiene la rotación de la base
    * *detener_eslabon_1*: detiene la rotación del eslabon 1
    * *detener_eslabon_2*: detiene la rotación del eslabon 2
    * *detener_motores*: detiene los motores
    * *Mapear_Terreno*: Mapea el terreno
    * *base_rotacion*: Rota la base
    * *hacia_atras*: desplaza el robot hacia atras
    * *hacia_delante*: desplaza el robot hacia delante
    * *hacia_derecha*: desplaza el robot hacia la derecha
    * *hacia_izquierda*: Desplaza el robot hacia la izquierda
    * *mov_eslabon_1*: ejecuta el movimiento del eslabon 1
    * *mov_eslabon_2*: ejecuta el movimiento del eslabon 1
    * *mover_eslabon_1_atras*: mueve el eslabon 1 hacia atras
    * *mover_eslabon_1_delante*: mueve el eslabon 1 hacia delante
    * *mover_eslabon_1_derecha*: mueve el eslabon 1 hacia la derecha
    * *mover_eslabon_1_izquierda*: mueve el eslabon 1 hacia la izquierda
    * *mover_eslabon_2_atras*: mueve el eslabon 2 hacia atras
    * *mover_eslabon_2_delante*: mueve el eslabon 2 hacia delante
    * *mover_eslabon_2_derecha*: mueve el eslabon 2 hacia la derecha
    * *mover_eslabon_2_izquierda*: mueve el eslabon 2 hacia la izquierda

**Rotación**
    * *girar_base_derecha*: gira base en sentido horario
    * *girar_base_izquierda*: gira la base en sentido antihorario
    * *girar_derecha*: gira el robot en sentido horario
    * *girar_izquierda*: gira el robot en sentido antihorario
    * *giros_a_radianes*: conversion de giros a radianes
    * *grados_a_radianes*: conversión de grados a radianes
    * *rotar_robot*: Rota el robot horario o de forma antihoraria

**Funionalidades de apoyo**
    * *explorar*: Explora el mapa
    * *limite_sensores*: Verifica sensores para evitar colision
    * *orden_a_robot*: ejecuta el comando de desplazamiento del robot
    * *path_a_orden*: convierte el path de A* a ordenes a ejecutar para el desplazamiento del robot
    * *reset_encoder*: Resetea el encoder para contar los angulos de giro de las ruedas
    * *revisar_variables*: revisa que no hayan problemas de inicialización
    * *set_velocity_base*: Orden para rotar la base
    * *set_velocity_eslabon_1*: Orden para rotar el eslabon 1
    * *set_velocity_eslabon_2*: Orden para rotar el eslabon 2
    * *set_velocity_ruedas*: Orden para rotar las ruedas y generar un desplazamiento
