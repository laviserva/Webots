from controller import Robot
import numpy as np

class Robot_de_laberintos(Robot):
    def __init__(self):  
        super().__init__()
        
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
        
        #Rotaciones del robot        
        self.radio_robot = np.linalg.norm(self.coordenadas_rueda_centro)
        self.perimetro_robot = 2 * np.pi * self.radio_robot
        
        self.perimetro_de_llanta = 2 * np.pi * self.radio_de_llanta
        
        self.vueltas_rueda_a_giro_robot = self.perimetro_robot / self.perimetro_de_llanta
        
        #Seguridad de movimiento
        self.velocidad_seguridad = 10
        self.base_seguridad = 10
        self.block_speed_seguridad = 0.3
        self.eslabon_1_seguridad = 3
        self.eslabon_2_seguridad = 3
        
        #motores
        self.left_motor_1  = self.getDevice("Motor_1")
        self.right_motor_1 = self.getDevice("Motor_2")
        self.left_motor_2  = self.getDevice("Motor_3")
        self.right_motor_2 = self.getDevice("Motor_4")
        self.base_motor = self.getDevice("Motor_Base")
        self.eslabon_1_x_motor = self.getDevice("Motor_Eslabon_1_x")
        self.eslabon_1_z_motor = self.getDevice("Motor_Eslabon_1_z")
        self.eslabon_2_x_motor = self.getDevice("Motor_Eslabon_2_x")
        self.eslabon_2_z_motor = self.getDevice("Motor_Eslabon_2_z")
        
        #Encoders
        self.left_encoder_1  = self.getDevice("Encoder_1")
        self.right_encoder_1 = self.getDevice("Encoder_2")
        self.left_encoder_2  = self.getDevice("Encoder_3")
        self.right_encoder_2 = self.getDevice("Encoder_4")
        self.base_encoder    = self.getDevice("Encoder_Base")
        self.eslabon_1_x_encoder=self.getDevice("Encoder_Eslabon_1_x")
        self.eslabon_1_z_encoder=self.getDevice("Encoder_Eslabon_1_z")
        self.eslabon_2_x_encoder=self.getDevice("Encoder_Eslabon_2_x")
        self.eslabon_2_z_encoder=self.getDevice("Encoder_Eslabon_2_z")
        
        #Activar encoders
        self.left_encoder_1.enable(self.timestep)
        self.right_encoder_1.enable(self.timestep)
        self.left_encoder_2.enable(self.timestep)
        self.right_encoder_2.enable(self.timestep)
        self.base_encoder.enable(self.timestep)
        self.eslabon_1_x_encoder.enable(self.timestep)
        self.eslabon_1_z_encoder.enable(self.timestep)
        self.eslabon_2_x_encoder.enable(self.timestep)
        self.eslabon_2_z_encoder.enable(self.timestep)
        
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
            
        #Estableciendo condiciones iniciales de los motores
        self.left_motor_1.setPosition(float("inf"))
        self.right_motor_1.setPosition(float("inf"))
        self.left_motor_2.setPosition(float("inf"))
        self.right_motor_2.setPosition(float("inf"))
        self.base_motor.setPosition(float("inf"))
        self.eslabon_1_x_motor.setPosition(float("inf"))
        self.eslabon_1_z_motor.setPosition(float("inf"))
        self.eslabon_2_x_motor.setPosition(float("inf"))
        self.eslabon_2_z_motor.setPosition(float("inf"))
        
        self.left_motor_1.setVelocity(self.left_speed_1)
        self.right_motor_1.setVelocity(self.right_speed_1)
        self.left_motor_2.setVelocity(self.left_speed_2)
        self.right_motor_2.setVelocity(self.right_speed_2)
        self.base_motor.setVelocity(self.base_speed)
        self.eslabon_1_x_motor.setVelocity(self.eslabon_1_x_speed_block)
        self.eslabon_1_z_motor.setVelocity(self.eslabon_1_z_speed_block)
        self.eslabon_2_x_motor.setVelocity(self.eslabon_2_x_speed_block)
        self.eslabon_2_z_motor.setVelocity(self.eslabon_2_z_speed_block)
        
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
            
    def desplazamiento(self, giros = 2*np.pi, por_siempre = False):
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
                    initial_value = self.left_encoder_1.getValue()
                    first_run = False
                
                encoder_value = self.left_encoder_1.getValue()
                
                error = encoder_value - initial_value
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
        
        
        
        pass
        
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
        self.left_motor_1.setVelocity(self.left_speed_1)
        self.right_motor_1.setVelocity(self.right_speed_1)
        self.left_motor_2.setVelocity(self.left_speed_2)
        self.right_motor_2.setVelocity(self.right_speed_2)
        
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
        self.base_motor.setVelocity(self.base_speed)
        
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
                    initial_value = self.base_encoder.getValue()
                    first_run = False
                
                encoder_value = self.base_encoder.getValue()
                
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
        
        self.eslabon_1_x_motor.setVelocity(self.eslabon_1_x_speed_block)
        self.eslabon_1_z_motor.setVelocity(self.eslabon_1_z_speed_block)
        
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
                    initial_value = self.eslabon_1_x_encoder.getValue()
                    first_run = False
                if first_run == True and motor_x == False:
                    initial_value = self.eslabon_1_z_encoder.getValue()
                    first_run = False
                
                if motor_x == True:  encoder_value = self.eslabon_1_x_encoder.getValue()
                if motor_x == False: encoder_value = self.eslabon_1_z_encoder.getValue()
                
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
        self.eslabon_2_x_motor.setVelocity(self.eslabon_2_x_speed_block)
        self.eslabon_2_z_motor.setVelocity(self.eslabon_2_z_speed_block)
        
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
                    initial_value = self.eslabon_2_x_encoder.getValue()
                    first_run = False
                if first_run == True and motor_x == False:
                    initial_value = self.eslabon_2_z_encoder.getValue()
                    first_run = False
                
                if motor_x == True:  encoder_value = self.eslabon_2_x_encoder.getValue()
                if motor_x == False: encoder_value = self.eslabon_2_z_encoder.getValue()
                
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
    
    def grados_a_radiantes(self, grados):
        """
        Hace intercambio de grados a radianes
        """
        return 2*np.pi / 360 * grados
    def giros_a_radianes(self, grados):
        return grados * 2 * np.pi
    
    def limite_sensores(self, *sensor, valor_limite = 500):
        if valor_limite <= 300: valor_limite = 300
        
        for valor in sensor:
            if valor <= valor_limite:
                print(" - - Se ha detectado una colisión - - ")
                
                