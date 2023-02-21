from controller import Robot
import numpy as np
from ventana import *

class Robot_de_laberintos(Robot):
    def __init__(self):  
        super().__init__()
        
        self.parar_programa = False
        
        self.max_speed = 10
        self.radio_de_llanta = 0.025#0.025
        self.scale =  (1,1,1)
        self.speed = 1
        self.time_init = self.getTime() #posible cambio
        self.timestep = int(self.getBasicTimeStep())
        
        #Distancias geometricas del robot a partir del centro (0,0,0)
        self.coordenadas_rueda_centro = (-0.07, -0.022, -0.070)
        """se calcula la distancia desde el centro del robot
        hasta el punto de giro de la rueda
        Las ruedas están a una distancia (0.075, 0.013, 0.075)
        Si las distancias del robot son (0.14,0.05,0.014) entonces cualquier esquina será (0.07,0.05,0.07) con una distancia de
        0.05 en el eje x y eje z
        """
        
        #Ajustes por escala
        # 2 veces Radio del robot a partir de puntos de pivote para
        # rotarlo alrededor de las ruedas
        self.distancia_entre_ruedas_en_x = 2 * self.coordenadas_rueda_centro[0] * self.scale[0]
        self.distancia_entre_ruedas_en_z = 2 * self.coordenadas_rueda_centro[1] * self.scale[2]
        
        self.espacio_a_recorrer = 0.7 + 0.05 #0.70 del sensor 0.07 del robot
        
        
        #Rotaciones del robot        
        self.radio_robot = np.linalg.norm(self.coordenadas_rueda_centro)
        self.perimetro_robot = 2 * np.pi * self.radio_robot
        
        self.perimetro_de_llanta = 2 * np.pi * self.radio_de_llanta
        
        self.vueltas_rueda_a_giro_robot = self.perimetro_robot / self.perimetro_de_llanta
        
        self.distancia_a_recorrer = self.espacio_a_recorrer / (self.perimetro_de_llanta * np.sin(np.pi/4))
        
        #Seguridad de movimiento
        self.velocidad_seguridad = 10
        self.base_seguridad = 10
        self.block_speed_seguridad = 0.3
        self.eslabon_1_seguridad = 3
        self.eslabon_2_seguridad = 3
        self.valor_limite = 400
        
        # path
        self.celda_mapeo_robot = np.pi*2
        self.y_deseado = None
        self.x_deseado = None
        
        #Monitorea el numero de giros para mapear el terreno
        self.mapear_terreno = True
        self.contador_giros = 0
        self.val_prev_num_vueltas_ea = 0
        self.encoder_absoluto = 0
        
        
        #motores
        self.motores_names = ["Motor_1", "Motor_2", "Motor_3","Motor_4",
                              "Motor_Base",
                              "Motor_Eslabon_1_x", "Motor_Eslabon_1_z",
                              "Motor_Eslabon_2_x","Motor_Eslabon_2_z"]
        
        self.motores = [self.getDevice(motor) for motor in self.motores_names]
        [motor.setPosition(float("inf")) for motor in self.motores]
        [motor.setVelocity(0.0) for motor in self.motores]
        
        #Encoders
        self.encoders_names = ["Encoder_1", "Encoder_2", "Encoder_3","Encoder_4",
                              "Encoder_Base",
                              "Encoder_Eslabon_1_x", "Encoder_Eslabon_1_z",
                              "Encoder_Eslabon_2_x","Encoder_Eslabon_2_z"]
        
        self.encoders = [self.getDevice(encoder) for encoder in self.encoders_names]
        [encoder.enable(self.timestep) for encoder in self.encoders]
        self.encoder_contador = 0
        
        # Sensores de distancia
        sensors_names = ["Sensor_Delantero", "Sensor_Trasero", "Sensor_Izquierdo", "Sensor_Derecho"]
        self.sensores_distancia = [self.getDevice(sensor) for sensor in sensors_names]
        [sensor.enable(self.timestep) for sensor in self.sensores_distancia]
        
        #Definir velocidad inicial
        self.left_speed_1 = 0.0
        self.right_speed_1 = 0.0
        self.left_speed_2 = 0.0
        self.right_speed_2 = 0.0
        self.base_speed = 0.0
        self.block_speed = 0.02
        self.eslabon_1_x_speed_block = 0.0
        self.eslabon_1_z_speed_block = 0.0
        self.eslabon_2_x_speed_block = 0.0
        self.eslabon_2_z_speed_block = 0.0
        
        #Previniendo posibles problemas
        if self.timestep == 0: self.timestep = 1
        if self.speed <= 0 or self.speed > 1: self.speed = 1
        if self.max_speed == 0 or self.max_speed > 10 or self.max_speed < 0:
            self.max_speed = 6
        
    def revisar_variables(self):
        """
        Esta función revisa cada una de las variables involucradas en el movimiento y establece medidas de seguridad
        basicas para evitar que el robot pueda dañarse.
        """
        print("Iniciando revisión.")
        
        if self.block_speed > self.block_speed_seguridad:
            print("Estableciendo la velocidad de los eslabones a 0.3", self.block_speed_seguridad)
            self.block_speed = self.block_speed_seguridad
        if self.timestep == 0:
            self.timestep = 1
        if self.speed < 0 or self.speed > 1:
            print("Estableciendo la velocidad del robot a 1")
            self.speed = 1
        if self.max_speed == 0 or self.max_speed > self.velocidad_seguridad  or self.max_speed < 0:
            print("Estableciendo la velocidad maxima del robot a 10")
            self.max_speed = 10
        if self.left_speed_1 > self.velocidad_seguridad:
            print("Estableciendo la velocidad maxima de la llanta izquierda superior en ",self.velocidad_seguridad)
            self.left_speed_1 = self.velocidad_seguridad
        if self.right_speed_1 > self.velocidad_seguridad:
            print("Estableciendo la velocidad maxima de la llanta derecha superior en ", self.velocidad_seguridad)
            self.right_speed_1 = self.velocidad_seguridad
        if self.left_speed_2 > self.velocidad_seguridad:
            print("Estableciendo la velocidad maxima de la llanta izquierda inferior en ", self.velocidad_seguridad)
            self.left_speed_2 = self.velocidad_seguridad
        if self.right_speed_2 > self.velocidad_seguridad:
            print("Estableciendo la velocidad maxima de la llanta derecha superior en ", self.velocidad_seguridad)
            self.right_speed_2 = self.velocidad_seguridad
        if self.base_speed > self.base_seguridad:
            print("Estableciendo la velocidad maxima de la base del brazo robot en ", self.base_seguridad)
            self.base_speed = self.base_seguridad
        if self.eslabon_1_x_speed_block > self.eslabon_1_seguridad:
            print("Estableciendo la velocidad maxima del eslabon 1 eje x del brazo robot en ", self.eslabon_1_seguridad)
            self.eslabon_1_x_speed_block = self.eslabon_1_seguridad
        if self.eslabon_1_z_speed_block > self.eslabon_1_seguridad:
            print("Estableciendo la velocidad maxima del eslabon 1 eje z del brazo robot en ", self.eslabon_1_seguridad)
            self.eslabon_1_z_speed_block = self.eslabon_1_seguridad
        if self.eslabon_2_x_speed_block > self.eslabon_2_seguridad:
            print("Estableciendo la velocidad maxima del eslabon 2 eje x del brazo robot en ", self.eslabon_2_seguridad)
            self.eslabon_2_x_speed_block = self.eslabon_2_seguridad
        if self.eslabon_2_z_speed_block > self.eslabon_2_seguridad:
            print("Estableciendo la velocidad maxima del eslabon 2 eje z del brazo robot en ", self.eslabon_2_seguridad)
            self.eslabon_2_z_speed_block = self.eslabon_2_seguridad
    
    def reset_encoder(self, encoder_value, limit_reset =2*np.pi):
        """
        Se resetea el encoder cada giro de llanta.
        Cada giro es actualizado en la variable 'encoder_contador'
        El valor de 0 a 2pi se almacena en la variable valor

        Parameters
        ----------
        encoder_value : float
            encoder_value = x | 2pi > x > -2pi.
            Como va desde 0 hasta infinito, esta funcion resetea el valor del encoder cuando llega a 2pi y posteriormente
            almacena el numero de veces que se ha reseteado, es decir, el numero de giros de la llanta.
            
            Todo esto es basado en el encoder.

        Returns
        -------
        valor : float
            Mapea el encoder de valor = {Θ | 0 < Θ <  2pi}
        """
        
        giro_completo = False
        
        print("-------------- Encoder --------------")
        print("encoder_value: ",encoder_value)
        grados = abs(encoder_value) - limit_reset *self.encoder_contador
        
        print("self.encoder_contador", self.encoder_contador)
        
        if grados > limit_reset and grados > 0:
            print("Se ha dado un giro completo")
            grados = abs(encoder_value) - limit_reset *self.encoder_contador
            self.encoder_contador += 1
            giro_completo = True
            
        if grados < limit_reset and grados < 0:
            print("Se ha dado un giro completo")
            grados = abs(encoder_value) - limit_reset *self.encoder_contador
            self.encoder_contador -= 1
            giro_completo = True
        
        print("grados: ", grados)
        return encoder_value, giro_completo
            
    def desplazamiento(self, giros = 2*np.pi, por_siempre = False, text = True):
        """
        Desplaza en un ciclo while infinito hasta que se cumpla un numero determinado de giros
        """
        self.revisar_variables()
        if por_siempre:self.set_velocity_ruedas()
            
        else:
            first_run = True
            
            self.set_velocity_ruedas()
            
            while self.step(self.timestep) != -1:                
                if first_run:
                    initial_value = self.encoders[0].getValue()
                    first_run = False
                
                encoder_value = self.encoders[0].getValue()
                
                error = encoder_value - initial_value
                if text:
                    print("-------------- Desplazamiento --------------")
                    print("initial_value: ", initial_value)
                    print("encoder_value: ", encoder_value)
                    print("error: ", abs(error))
                    print("limite: ", giros)
                
                if abs(error) >= abs(giros):
                    self.detener_motores(time=0.5)
                    #self.set_velocity_ruedas()
                    return True
    
    def detener_motores(self,time=0.05):
        """
        Detiene los motores. Necesita la función set_velocity_ruedas para que se efectue
        """
        init_time = self.getTime()
        self.left_speed_1 = 0.0
        self.right_speed_1 = 0.0
        self.left_speed_2 = 0.0
        self.right_speed_2 = 0.0
        
        self.set_velocity_ruedas()
        
        while self.step(self.timestep) != -1 and (self.getTime() - init_time) >= time:
            continue
        
    def rotar_robot(self, grados_radianes = np.pi/2):
        
        coordenada_x_circ_robot = self.distancia_rueda_centro_x/2
        coordenada_z_circ_robot = self.distancia_rueda_centro_z/2
        
        radio_robot = np.sqrt(np.square(coordenada_x_circ_robot) + np.square(coordenada_z_circ_robot))
        perimetro_robot = 2 * np.pi * radio_robot
        
        perimetro_de_llanta = 2 * np.pi * self.radio_de_llanta
        
        vueltas_rueda_a_giro_robot = perimetro_robot / perimetro_de_llanta
        return vueltas_rueda_a_giro_robot
        
    def girar_izquierda(self):
        """
        Gira hacia la izquierda. Necesita la función set_velocity_ruedas para que se efectue
        """
        self.left_speed_1  = -self.speed*self.max_speed
        self.right_speed_1 =  self.speed*self.max_speed
        self.left_speed_2  = -self.speed*self.max_speed
        self.right_speed_2 =  self.speed*self.max_speed      
    
    def girar_derecha(self):
        """
        Gira hacia la derecha. Necesita la función set_velocity_ruedas para que se efectue
        """
        self.left_speed_1  =  self.speed*self.max_speed
        self.right_speed_1 = -self.speed*self.max_speed
        self.left_speed_2  =  self.speed*self.max_speed
        self.right_speed_2 = -self.speed*self.max_speed
        
    def hacia_delante(self):
        """
        Establece la velocidad a la cual debe de moverse el robot y lo mueve hacia delante
        Necesita la función set_velocity_ruedas para que se efectue
        """
        self.left_speed_1  = self.speed*self.max_speed
        self.right_speed_1 = self.speed*self.max_speed
        self.left_speed_2  = self.speed*self.max_speed
        self.right_speed_2 = self.speed*self.max_speed     
        
    def hacia_atras(self):
        """
        Establece la velocidad a la cual debe de moverse el robot y lo mueve hacia atras
        Necesita la función set_velocity_ruedas para que se efectue
        """
        self.left_speed_1  = -self.speed*self.max_speed
        self.right_speed_1 = -self.speed*self.max_speed
        self.left_speed_2  = -self.speed*self.max_speed
        self.right_speed_2 = -self.speed*self.max_speed
        
    def hacia_izquierda(self):
        """
        Establece la velocidad a la cual debe de moverse el robot y lo mueve hacia izquierda
        Necesita la función set_velocity_ruedas para que se efectue
        """
        self.left_speed_1  = -self.speed*self.max_speed
        self.right_speed_1 =  self.speed*self.max_speed
        self.left_speed_2  =  self.speed*self.max_speed
        self.right_speed_2 = -self.speed*self.max_speed
    
    def hacia_derecha(self):
        """
        Establece la velocidad a la cual debe de moverse el robot y lo mueve hacia derecha
        Necesita la función set_velocity_ruedas para que se efectue
        """
        self.left_speed_1  =  self.speed*self.max_speed
        self.right_speed_1 = -self.speed*self.max_speed
        self.left_speed_2  = -self.speed*self.max_speed
        self.right_speed_2 =  self.speed*self.max_speed
        
    def set_velocity_ruedas(self):
        """
        Ejecuta las velocidades previamente establecidas, en los motores de las ruedas
        """
        
        self.motores[0].setVelocity(self.left_speed_1)
        self.motores[1].setVelocity(self.right_speed_1)
        self.motores[2].setVelocity(self.left_speed_2)
        self.motores[3].setVelocity(self.right_speed_2)
        
    def girar_base_izquierda(self):
        """
        Gira la base en sentido contrario a las manecillas del reloj. Necesita función set_velocity_base
        """
        self.base_speed = self.speed*self.max_speed
        
    def girar_base_derecha(self):
        """
        Gira la base en sentido  a las manecillas del reloj. Necesita función set_velocity_base
        """
        self.base_speed = -self.speed*self.max_speed
        
    def detener_base(self):
        """
        Detiene el giro de la base. Necesita función set_velocity_base
        """
        self.base_speed = 0
    
    def set_velocity_base(self):
        self.motores[4].setVelocity(self.base_speed)
        
    def base_rotacion(self, rotacion = 2*np.pi, por_siempre = False):
        """
        Desplaza en un ciclo while infinito hasta que se cumpla un numero determinado de giros
        """
        self.revisar_variables()
        if por_siempre:
            self.set_velocity_base()
        
        else:
            
            first_run = True
            self.set_velocity_base()
            
            while self.step(self.timestep) != -1:
                
                if first_run:
                    initial_value = self.encoders[4].getValue()
                    first_run = False
                
                encoder_value = self.encoders[4].getValue()
                
                error = encoder_value - initial_value
                print("-------------- Base --------------")
                print("initial_value: ", initial_value)
                print("encoder_value: ", encoder_value)
                print("error: ", abs(error))
                print("limite: ", rotacion)
                
                
                
                if abs(error) >= abs(rotacion):
                    self.detener_base()
                    self.set_velocity_base()
                    return True
    
    def mover_eslabon_1_delante(self):
        """
        Rota  el eslabon 1 alrededor eje x, sus valores positivos harán que el eslabon apunte
        hacia la izquierda mientras que los valores negativos hacia atrás
        """
        self.eslabon_1_x_speed_block = self.block_speed*self.speed*self.max_speed
        
    def mover_eslabon_1_atras(self):
        """
        Rota  el eslabon 1 alrededor eje x, sus valores positivos harán que el eslabon apunte
        hacia la izquierda mientras que los valores negativos hacia atrás
        """
        self.eslabon_1_x_speed_block = -self.block_speed*self.speed*self.max_speed
        
    def mover_eslabon_1_izquierda(self):
        """
        Rota  el eslabon 1 alrededor eje x, sus valores positivos harán que el eslabon apunte
        hacia la izquierda mientras que los valores negativos hacia atrás
        """
        self.eslabon_1_z_speed_block = -self.block_speed*self.speed*self.max_speed
        
    def mover_eslabon_1_derecha(self):
        """
        Rota  el eslabon 1 alrededor eje x, sus valores positivos harán que el eslabon apunte
        hacia la izquierda mientras que los valores negativos hacia atrás
        """
        self.eslabon_1_z_speed_block = self.block_speed*self.speed*self.max_speed
        
    def detener_eslabon_1(self):
        """
        Detiene el eslabon 1
        """
        self.eslabon_1_x_speed_block = 0
        self.eslabon_1_z_speed_block = 0
        
    def set_velocity_eslabon_1(self):
        """
        Envia las velocidades previamente establecidas al motor
        """
        
        self.motores[5].setVelocity(self.eslabon_1_x_speed_block)
        self.motores[6].setVelocity(self.eslabon_1_z_speed_block)
        
    def mov_eslabon_1(self, rotacion = 1, motor_x = True, por_siempre = False):
        """
        Desplaza en un ciclo while infinito hasta que se cumpla un numero determinado de giros
        """
        self.revisar_variables()
        if por_siempre: self.set_velocity_eslabon_1()
        
        else:
            first_run = True
            
            self.set_velocity_eslabon_1()
            
            while self.step(self.timestep) != -1:
                
                if first_run == True and motor_x == True:
                    initial_value = self.encoders[5].getValue()
                    first_run = False
                if first_run == True and motor_x == False:
                    initial_value = self.encoders[6].getValue()
                    first_run = False
                
                if motor_x == True:  encoder_value = self.encoders[5].getValue()
                if motor_x == False: encoder_value = self.encoders[6].getValue()
                
                error = encoder_value - initial_value
                print("-------------- Eslabon 1 --------------")
                print("initial_value: ", initial_value)
                print("encoder_value: ", encoder_value)
                print("error: ", abs(error))
                print("limite: ", rotacion)
                
                if abs(error) >= abs(rotacion):
                    self.detener_eslabon_1()
                    self.set_velocity_eslabon_1()
                    return True
            
        
    def mover_eslabon_2_delante(self):
        """
        Rota  el eslabon 2 alrededor eje x, sus valores positivos harán que el eslabon apunte
        hacia la izquierda mientras que los valores negativos hacia atrás
        """
        self.eslabon_2_x_speed_block = -self.block_speed*self.speed*self.max_speed
        
    def mover_eslabon_2_atras(self):
        """
        Rota  el eslabon 2 alrededor eje x, sus valores positivos harán que el eslabon apunte
        hacia la izquierda mientras que los valores negativos hacia atrás
        """
        self.eslabon_2_x_speed_block = self.block_speed*self.speed*self.max_speed
        
    def mover_eslabon_2_izquierda(self):
        """
        Rota  el eslabon 2 alrededor eje x, sus valores positivos harán que el eslabon apunte
        hacia la izquierda mientras que los valores negativos hacia atrás
        """
        self.eslabon_2_z_speed_block = self.block_speed*self.speed*self.max_speed
        
    def mover_eslabon_2_derecha(self):
        """
        Rota  el eslabon 2 alrededor eje x, sus valores positivos harán que el eslabon apunte
        hacia la izquierda mientras que los valores negativos hacia atrás
        """
        self.eslabon_2_z_speed_block = -self.block_speed*self.speed*self.max_speed
        
    def detener_eslabon_2(self):
        """
        Establece las velocidades del eslabon 2 en cero, para detenerse
        """
        self.eslabon_2_x_speed_block = 0
        self.eslabon_2_z_speed_block = 0
        
    def set_velocity_eslabon_2(self):
        """
        Actua las velocidades del eslabon 2 en los motores
        """
        self.motores[7].setVelocity(self.eslabon_2_x_speed_block)
        self.motores[8].setVelocity(self.eslabon_2_z_speed_block)
        
    def mov_eslabon_2(self, rotacion = 1, motor_x = True, por_siempre = False):
        """
        Desplaza en un ciclo while infinito hasta que se cumpla un numero determinado de giros
        """
        self.revisar_variables()
        if por_siempre: self.set_velocity_eslabon_2()
        
        else:
            first_run = True
            
            self.set_velocity_eslabon_2()
            
            while self.step(self.timestep) != -1:
                
                if first_run == True and motor_x == True:
                    initial_value = self.encoders[7].getValue()
                    first_run = False
                if first_run == True and motor_x == False:
                    initial_value = self.encoders[8].getValue()
                    first_run = False
                
                if motor_x == True:  encoder_value = self.encoders[7].getValue()
                if motor_x == False: encoder_value = self.encoders[8].getValue()
                
                error = encoder_value - initial_value
                print("-------------- Eslabon 2 --------------")
                print("initial_value: ", initial_value)
                print("encoder_value: ", encoder_value)
                print("error: ", abs(error))
                print("limite: ", rotacion)
                
                if abs(error) >= abs(rotacion):
                    self.detener_eslabon_2()
                    self.set_velocity_eslabon_2()
                    return True
    
    def grados_a_radianes(self, grados):
        """
        Hace intercambio de grados a radianes
        """
        return 2*np.pi / 360 * grados
    def giros_a_radianes(self, grados):
        """
        convierte grados a radianes
        """
        return grados * 2 * np.pi
    
    def limite_sensores(self, *sensor, medir_sensor=3, first_time = False):
        """
        Verifica los sensores para evitar una colisión
        """
        valor_limite = self.valor_limite
        #espacio_a_recorrer = 0.05 + 0.07 #50% distancia maxima sensor + distancia sensor, extremo.
                    
        salir = False
        while self.step(self.timestep) != -1:
            sensor = [sensor.getValue() for sensor in self.sensores_distancia]
            print("Sensores: [  ", end=(""))
            [print("{:.3f}".format(valor), end="  ")for valor in sensor]
            print("]")
            
            
            if valor_limite <= 300: valor_limite = 300
            print("")
            
            for i,valor in enumerate(sensor):
                #print("Sensor_{}: ".format(i), end = "")
                #print("{:>7.2f}".format(valor), end="   ")
                if valor <= valor_limite and medir_sensor == i:
                    salir = True
                    print("\n{:^80}".format("----- Se ha detectado una colisión -----"))
            if salir:
                self.detener_motores()
                return True

        print("\n")
        
    def Mapear_Terreno(self):
        """
        Se usan los encoders para mapear el terreno.
        La matriz que retorna devuelve el terreno mapeado
        0 para espacios navegables
        1 para pareces
        2 para inicio
        """
        
        # Midiendo al robot            
        if self.mapear_terreno:
            self.valores_iniciales = np.array([encoder.getValue() for encoder in self.encoders][:4])
            self.error_previo = 0
            self.giros_a_arriba = 0
            self.giros_a_abajo = 0
            self.giros_a_izquierda = 0
            self.giros_a_derecha = 0
            self.lowest_x = 0
            self.lowest_y = 0
            self.highest_x = 0
            self.highest_y = 0
            self.mapear_terreno = False
            self.habilitar_mapeado = True
            self.terreno_mapeado = np.zeros((2,3),dtype=np.int)
        
        error = np.array([encoder.getValue() for encoder in self.encoders][:4])
        self.error_actual = abs(error) - abs(self.error_previo)
        errores = self.valores_iniciales - error
        self.error_previo = error
        
        self.encoder_absoluto += abs(self.error_actual[0])
        self.num_vueltas_ea = self.encoder_absoluto / self.celda_mapeo_robot #/(2*np.pi) # /2
        giro_completo = int(self.num_vueltas_ea) - int(self.val_prev_num_vueltas_ea) # 6 2/3 hacen una vuelta completa
        print("giro_completo: ", giro_completo)
        self.val_prev_num_vueltas_ea = self.num_vueltas_ea
        
        sensor = np.array([sensor.getValue() for sensor in self.sensores_distancia])
        self.y = self.giros_a_arriba - self.giros_a_abajo - self.highest_y
        self.x = self.giros_a_derecha - self.giros_a_izquierda - self.lowest_x
        #self.y = self.highest_y - self.lowest_y
        #self.x = self.highest_x - self.lowest_x
        
        # Estableciendo matriz local
        matriz_local_x = np.zeros([1,3])[0]
        matriz_local_y = np.zeros([3,1])
        
        matriz_paredes = np.where(sensor < self.valor_limite, 1, 0)
        matriz_local_y[0] = matriz_paredes[0]
        matriz_local_y[2] = matriz_paredes[1] #Si hay paredes en matriz local
        matriz_local_x[0] = matriz_paredes[2]
        matriz_local_x[2] = matriz_paredes[3]
        
        if giro_completo:
            
            if self.sensor_previo_pared == 0: self.giros_a_izquierda += 1
            if self.sensor_previo_pared == 1: self.giros_a_derecha += 1
            if self.sensor_previo_pared == 2: self.giros_a_abajo += 1
            if self.sensor_previo_pared == 3: self.giros_a_arriba += 1
                
            if giro_completo and self.giros_a_abajo == 2 and self.habilitar_mapeado:
                self.terreno_mapeado = np.zeros((2,3),dtype=np.int)
                self.giros_a_arriba = 0
                self.giros_a_abajo = 0
                self.giros_a_izquierda = 0
                self.giros_a_derecha = 0
                self.habilitar_mapeado = False
            if self.giros_a_abajo <= 2 and self.habilitar_mapeado: return
                
            if  self.y < 0 and abs(self.y) >= self.lowest_y and self.sensor_previo_pared == 2:
                self.lowest_y +=1
                self.terreno_mapeado = np.insert(self.terreno_mapeado, self.terreno_mapeado.shape[0], 0, axis=0)
                
            if  self.x < 0 and abs(self.x) >= self.lowest_x and self.sensor_previo_pared == 1:
                self.lowest_x 
                self.terreno_mapeado = np.insert(self.terreno_mapeado, self.terreno_mapeado.shape[1], 0, axis=1)
                
            if  self.y > 0 and self.y >= self.highest_y and self.sensor_previo_pared == 3:
                self.highest_y +=1
                self.terreno_mapeado = np.insert(self.terreno_mapeado, 0, 0, axis=0)
                
            if self.x > 0 and abs(self.x) >= self.highest_x and self.sensor_previo_pared == 1:
                self.highest_x +=1
                self.terreno_mapeado = np.insert(self.terreno_mapeado, self.terreno_mapeado.shape[1], 0, axis=1)
                       
            if self.sensor_previo_pared == 0 and self.y > 0: self.terreno_mapeado[self.y][self.x] = matriz_local_y[0]
            if self.sensor_previo_pared == 0 and self.y < 0: self.terreno_mapeado[abs(self.y)][self.x] = matriz_local_y[0]
            if self.sensor_previo_pared == 1 and self.y < 0: self.terreno_mapeado[abs(self.y)][self.x] = matriz_local_y[2]            
            if self.sensor_previo_pared == 2 and (self.x > 0 or self.y <0): self.terreno_mapeado[abs(self.y)][self.x] = matriz_local_x[0]
            if self.sensor_previo_pared == 3 and self.x > 0: self.terreno_mapeado[abs(self.y)][self.x] = matriz_local_x[2]
            if self.sensor_previo_pared == 3 and self.y > 0 and self.highest_x == self.y:
                self.terreno_mapeado[self.y][self.x] = matriz_local_x[2]

        if (np.sum(errores) / 8) < 1 and self.terreno_mapeado.shape >= (5,5) and self.sensor_previo_pared == 2 and self.x <8 and self.y < 8 and self.habilitar_mapeado == False:
            """
            Regresando a punto inicial.
            """
            print("Se está cerca del punto final. parando motores y explotando")
            self.coord_final_y = self.y + 1
            self.coord_final_x = self.x + 2
            print("self.coord_final_y: ", self.coord_final_y)
            print("self.coord_final_x: ", self.coord_final_x)
            self.parar_programa = True
        
    def path_a_orden(self, path):
        """
        path -> (y,x)
        de un valor previo del path, saber hacia donde se mueve. si hacia arriba o hacia abajo
        empezar for en [1,0] hasta un maximo de (max_y, max_x)
        usar ifs para comprar valor previo con valor actual y de esta forma decidir hacia donde se moverá el robot
        
        """
        order_path = []
        row_prev = path[0]
        for row in path[1:]:
            if row_prev[0] > row[0]:
                order_path.append(0) # arriba
            if row_prev[0] < row[0]:
                order_path.append(1) # abajo
            if row_prev[1] > row[1]:
                order_path.append(2) # izquierda
            if row_prev[1] < row[1]:
                order_path.append(3) # dereecha
            row_prev = row
        
        orders = []
        string_prev = order_path[0]
        counter_string = 1
        for string in order_path[1:]:
            if string_prev == string:
                counter_string += 1
            else:
                orders.append([string_prev, counter_string])
                counter_string = 1
            string_prev = string
        orders.append([string_prev, counter_string])
        
        return orders
    
    def orden_a_robot(self, path):
        orders = self.path_a_orden(path)
        
        for string, giro in orders:
            print(string)
            if string == 0:self.hacia_delante()
            if string == 1:self.hacia_atras()
            if string == 2:self.hacia_izquierda()
            if string == 3:self.hacia_derecha()
            
            self.desplazamiento(giros=giro*self.celda_mapeo_robot*1.5, text= False) # valor optimo utilizado en mapeo del mapa
            print("giros: giro*np.pi*2", giro*np.pi*4)
            print("string: ", string)
            self.detener_motores(time=1)
    
    def explorar(self, path = [None, None]):
        """
        Explora el terreno
        """
        print("---------- Iniciando exploración ----------")
        
        self.programa_finalizado = False
        self.y_deseado = path[0]
        self.x_deseado = path[1]
        
        self.hacia_izquierda()
        self.desplazamiento(por_siempre = True)
        self.limite_sensores(medir_sensor=2, first_time = True)
        
        self.hacia_atras()
        self.desplazamiento(por_siempre = True)
        #self.limite_sensores(medir_sensor=1, first_time = True)
        self.valor_limite = valor_limite = 450
        tiempo_espera = 1
        self.sensor_previo_pared = 2
        
        self.mapear_terreno = True
        
        while self.step(self.timestep) != -1:
            sensor = [sensor.getValue() for sensor in self.sensores_distancia]
            print("Sensores: [  ", end=(""))
            [print("{:.3f}".format(valor), end="  ")for valor in sensor]
            print("]" , self.sensor_previo_pared)
            
            self.Mapear_Terreno()
            
            if len(set(sensor)) == 1 and self.sensor_previo_pared == 0: # Pared arriba movimiento arriba
                
                self.hacia_izquierda()
                self.desplazamiento(giros = self.distancia_a_recorrer)
                self.detener_motores(time=tiempo_espera)
                self.hacia_delante()
                self.desplazamiento(giros = self.distancia_a_recorrer)
                self.detener_motores(time=tiempo_espera)
                self.hacia_derecha()
                self.desplazamiento(giros = np.pi/40)
                self.detener_motores(time=tiempo_espera)
                self.hacia_delante()
                self.set_velocity_ruedas()
                self.sensor_previo_pared = 3
                continue
            
            if len(set(sensor)) == 1 and self.sensor_previo_pared == 1: # Pared abajo movimiento abajo
                self.hacia_derecha()
                self.desplazamiento(giros = self.distancia_a_recorrer)
                
                self.detener_motores(time=tiempo_espera)
                self.hacia_atras()
                self.desplazamiento(giros = self.distancia_a_recorrer)
                self.detener_motores(time=tiempo_espera)
                self.hacia_izquierda()
                self.desplazamiento(giros = np.pi/40)
                self.detener_motores(time=tiempo_espera)
                self.hacia_atras()
                self.set_velocity_ruedas()
                self.sensor_previo_pared = 2
                continue
            
            if len(set(sensor)) == 1 and self.sensor_previo_pared == 2: # Pared izquierda movimiento izquierda
                self.hacia_atras()
                self.desplazamiento(giros = self.distancia_a_recorrer)
                
                self.detener_motores(time=tiempo_espera)
                self.hacia_izquierda()
                self.desplazamiento(giros = self.distancia_a_recorrer)
                self.detener_motores(time=tiempo_espera)
                self.hacia_delante()
                self.desplazamiento(giros = np.pi/40)
                self.detener_motores(time=tiempo_espera)
                self.hacia_izquierda()
                self.set_velocity_ruedas()
                self.sensor_previo_pared = 0
                continue
            
            if len(set(sensor)) == 1 and self.sensor_previo_pared == 3: # Pared derecha movimiento derecha
                
                self.hacia_delante()
                self.desplazamiento(giros = self.distancia_a_recorrer)
                self.detener_motores(time=tiempo_espera)
                self.hacia_derecha()
                self.desplazamiento(giros = self.distancia_a_recorrer)
                self.detener_motores(time=tiempo_espera)
                self.hacia_atras()
                self.desplazamiento(giros = np.pi/40)
                self.detener_motores(time=tiempo_espera)
                self.hacia_derecha()
                self.set_velocity_ruedas()
                self.sensor_previo_pared = 1
                continue
                
            
            if sensor[0] <= valor_limite and sensor[3] <= valor_limite:
                self.detener_motores(time=tiempo_espera)
                self.hacia_izquierda()
                self.set_velocity_ruedas()
                self.sensor_previo_pared = 0
                continue
                
            if sensor[1] <= valor_limite and sensor[2] < valor_limite: # Robot esquina inferior izquierda
                self.detener_motores(time=tiempo_espera)
                self.hacia_derecha()
                self.set_velocity_ruedas()
                self.sensor_previo_pared = 1
                continue
                
            if sensor[2] <= valor_limite and sensor[0] <= valor_limite:
                self.detener_motores(time=tiempo_espera)
                self.hacia_atras()
                self.set_velocity_ruedas()
                self.sensor_previo_pared = 2
                continue
                
            if sensor[3] <= valor_limite and sensor[1] <= valor_limite:
                self.detener_motores(time=tiempo_espera)
                self.hacia_delante()
                self.set_velocity_ruedas()
                self.sensor_previo_pared = 3
                continue
            
            if self.parar_programa:
                self.detener_motores()
                
                import pickle
                import os
                with open('mapa.txt', 'wb') as file:
                    pickle.dump(self.terreno_mapeado, file)
                print("Archivo guardado en:", os.getcwd())
                
                path = run_a_star(self.coord_final_y, self.coord_final_x)
                return path
            
            """
            self.hacia_delante()
            self.desplazamiento(por_siempre = True)
            self.limite_sensores(medir_sensor=0)

            self.hacia_izquierda()
            self.set_velocity_ruedas()
            self.limite_sensores(medir_sensor=2)
            
            self.hacia_atras()
            self.set_velocity_ruedas()
            self.limite_sensores(medir_sensor=1)
            
            self.hacia_derecha()
            self.set_velocity_ruedas()
            self.limite_sensores(medir_sensor=3)
            """