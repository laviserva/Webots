# README
Este proyecto fue realizado en octubre 2021. Se hizo un simil a un brazo robot utilizando una librería básica sin ningun tipo de aplicación de teoría de control, tiempo de entrega 2 semanas.

## controllers/Controlador_Movimiento_Basico.py
Funcion principal, donde al robot se le proporciona las acciones a realizar, acciones las cuales están dentro del archivo librería. Modifico los valores iniciales de por ejemplo; velocidad. Además de establecer la rutina principal del robot, de forma no inteligente, sino siguiendo ordenes de script.

## libraries/librería.py
Librería donde están todas las funciones utilizadas divididas en.

**Traslación:**
    - desplazamiento
    - detener_motores
    - detener_base
    - detener_eslabon_1
    - detener_eslabon_2
    - hacia_atras
    - hacia_delante
    - hacia_derecha
    - hacia_izquierda
    - mov_eslabon_1
    - mov_eslabon_2
    - mover_eslabon_1_atras
    - mover_eslabon_1_delante
    - mover_eslabon_1_derecha
    - mover_eslabon_1_izquierda
    - mover_eslabon_2_atras
    - mover_eslabon_2_delante
    - mover_eslabon_2_derecha
    - mover_eslabon_2_izquierda

**Rotación**
    - girar_derecha
    - girar_izquierda
    - base_rotacion
    - girar_base_derecha
    - girar_base_izquierda
    - giros_a_radianes
    - grados_a_radiantes

**Soporte para otras funciones**
    - reset_encoder
    - revisar_variables
    - rotar_robot
    - set_velocity_base
    - set_velocity_eslabon_1
    - set_velocity_eslabon_2
    - set_velocity_ruedas