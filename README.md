Autor: REDA KASTITE
version: 1.1.0
Python_version: 2.7.17

Coleccion de funciones. 
    
    Contiene:

    + Funcion lectura de archivo de parametros con formato object y publicacion en param server. [leer_Config()]

    + Funcion lectura de archivo con formato. [leer_Archivo()]

    + Coleccion de metodos para calculo de velocidades en Navegacion Robot. [Nav_utils().[]]

        TODO: DEJAR DE PASAR VELOCIDADES COMO ARGS A LAS FUNCIONES Y COGERLAS DE LOS PARAMETROS

        @ Funciones -------------------------------------------------------------------------------------------------------------

        + cuadrante -> Eleccion de cuadrante segun angulo

        + mixto_diferencial_sin -> Calculo de velocidades angulares de ruedas general. Para robots diferenciales y curvatura de la trayectoria segun angulo usando sin.

        + recto_diferencial -> Calculo de velocidades angulares de ruedas especifico para ir en linea recta (dir=1) o en marcha atras (dir=-1)

        + giro -> Calculo de velocidades angulares de ruedas especifico para giros sobre el propio eje del robot.

        + w_wheels_Twist_diferencial -> Calculo de Velocidad Lineal X y Velocidad Angular Z a partir de las velocidades angulares de las ruedas.

        + vels_a_w -> Calculo de las velocidades angulares de las ruedas a partir  de Velocidad Lineal X y Velocidad Angular Z.
