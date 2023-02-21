from controller import Robot
from Libreria import *
from ventana import *
import numpy as np
import os, pickle
        
if __name__ == "__main__":
    Roboto = Robot_de_laberintos()
    time_init = Roboto.getTime()
    
    """
    Roboto.block_speed = 0.1                  # Controla la velocidad de los eslabones
    Roboto.speed = 0.5                        # Controla la velocidad general del robot
    Roboto.left_speed_1 = 0.0                 # Controla rueda izquierda superior
    Roboto.right_speed_1 = 0.0                # Controla rueda derecha superior
    Roboto.left_speed_2 = 0.0                 # Controla rueda izquierda inferior
    Roboto.right_speed_2 = 0.0                # Controla rueda derecha inferior
    Roboto.base_speed = 0.0                   # Controla la velocidad de la base
    Roboto.eslabon_1_x_speed_block = 0.0      # Controla la velocidad del eslabon 1 en x
    Roboto.eslabon_1_z_speed_block = 0.0      # Controla la velocidad del eslabon 1 en z
    Roboto.eslabon_2_x_speed_block = 0.0      # Controla la velocidad del eslabon 2 en x
    Roboto.eslabon_2_z_speed_block = 0.0      # Controla la velocidad del eslabon 2 en z
    
    
    giros = 1
    Roboto.block_speed = 0.1

    Roboto.hacia_derecha() #Establece los valores iniciales para que el robot se dirija hacial a izquierda
    Roboto.desplazamiento(giros=Roboto.giros_a_radianes(giros)) #Activa los motores para que se mueva x giros y luego para
    Roboto.hacia_izquierda()
    Roboto.desplazamiento(giros=Roboto.giros_a_radianes(giros))
    Roboto.hacia_delante()
    Roboto.desplazamiento(giros=Roboto.giros_a_radianes(giros))
    Roboto.hacia_atras()
    Roboto.desplazamiento(giros=Roboto.giros_a_radianes(giros))
    Roboto.girar_base_derecha()
    Roboto.base_rotacion(rotacion=np.pi)
    Roboto.girar_base_izquierda()
    Roboto.base_rotacion(rotacion=np.pi)
    Roboto.mover_eslabon_1_derecha()
    #se mueve a 1 pi radian, Si es motor_x False, se moverá a traves el eje z
    Roboto.mov_eslabon_1(rotacion = 1, motor_x = False)
    Roboto.mover_eslabon_2_atras()
    Roboto.mov_eslabon_2(rotacion = 1) #1 pi radian
    Roboto.hacia_derecha()
    Roboto.desplazamiento(por_siempre = True) # Se desplaza "indefinidamente" pero se detiene instantaneamente por siguiente orden
    Roboto.detener_motores() #Se detinen los motores por 0.05 segundos
    Roboto.hacia_derecha()
    Roboto.desplazamiento(por_siempre = True)
    #Usar la siguiente funcion cuando se quiera cambiar de movimiento en el eje del robot de x a z o viceversa
    Roboto.detener_motores(time = 1) 
    Roboto.girar_derecha()
    Roboto.desplazamiento(giros = Roboto.vueltas_rueda_a_giro_robot*2*np.pi*10)
    """
    
    if not os.path.isfile("mapa.txt"): path = Roboto.explorar()
    #path = Roboto.explorar()
    else:
        path = run_a_star(70, 10) 
    Roboto.orden_a_robot(path)
    


    """
    while Roboto.step(Roboto.timestep) != -1:
        Roboto.limite_sensores([sensor.getValue() for sensor in Roboto.sensores_distancia])
    """
    