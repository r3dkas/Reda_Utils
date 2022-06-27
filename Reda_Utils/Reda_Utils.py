#! /usr/bin/python

"""
Autor: REDA KASTITE
version: 1.02
Python_version: 2.7.17
"""

import os,rospy
import requests
import math
import geometry_msgs.msg 
import std_msgs.msg  
import ast

#class DictObj:
#    def __init__(self, in_dict):
#        assert isinstance(in_dict,dict)
#        for key, val in in_dict.items():
#            if isinstance(val, (list, tuple)):
#                setattr(self, key, [DictObj(x) if isinstance(x, dict) else x for x in val])
#            else:
#                setattr(self, key, DictObj(val) if isinstance(val, dict) else val)

class General_clase(object):
    Activar_mensajes=bool
    Diametro_ruedas=float
    Distancia_entre_ruedas=float
    Mapa=str
    Robot=str
    Posicion_cargador={}
    
class Navegacion_clase(object):

    Velocidad_lineal_max= float
    Velocidad_lineal_max_pto_final= float
    Velocidad_lineal_min= float

    Velocidad_angular_max=float
    Velocidad_angular_max_pto_final= float
    Velocidad_angular_min= float

    Margen_desplazamiento_intermedio= float
    Margen_desplazamiento_final= float

    Margen_giro_intermedio= float
    Margen_giro_final= float

class Obstaculos_clase(object):

    Deteccion=str
    Radio_deteccion= float 
    Largo=float
    Ancho=float

    Limite_puntos= int

class Config_clase(object):

    General=General_clase()
    Navegacion=Navegacion_clase()
    Obstaculos=Obstaculos_clase()


class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__

def leer_Config(path=str):
    """
    Lee el archivo yaml de Configuracion, lo publica en ROS param server y lo devuelve como objeto con los parametros como atributos.

    @parametros -------------------------------------------------------------------------------------------------------------------------------------------------

    + path -> Directorio COMPLETO hasta el archivo yaml de configuracion. Ej.: '/home/{nombre_pc}/.../{Configuracion}.yaml' 
    """

    cmd='rosparam load '+path

    os.system('rosparam load /home/agv/catkin_ws/src/navegacion/config/General.yaml')

    #os.system(cmd)

    parametros=rospy.get_param('/Configuracion')
    global configuracion
    configuracion={}
    configuracion=dotdict(configuracion)
    configuracion = dotdict(parametros)

    configuracion.Obstaculos=dotdict(parametros['Obstaculos'])

    configuracion.General=dotdict(parametros['General'])

    configuracion.Navegacion=dotdict(parametros['Navegacion'])


    config=Config_clase()

    config.Obstaculos.Ancho=configuracion.Obstaculos['Ancho']
    config.Obstaculos.Deteccion=configuracion.Obstaculos['Deteccion']
    config.Obstaculos.Largo=configuracion.Obstaculos['Largo']
    config.Obstaculos.Limite_puntos=configuracion.Obstaculos['Limite puntos']
    config.Obstaculos.Radio_deteccion=configuracion.Obstaculos['Radio deteccion']


    config.General.Activar_mensajes=configuracion.General['Activar mensajes']
    config.General.Diametro_ruedas=configuracion.General['Diametro ruedas']
    config.General.Distancia_entre_ruedas=configuracion.General['Distancia entre ruedas']
    config.General.Mapa=configuracion.General['Mapa']
    config.General.Robot=configuracion.General['Robot']
    config.General.Posicion_cargador=configuracion.General['Posicion cargador']



    config.Navegacion.Velocidad_lineal_max= configuracion.Navegacion['Velocidad lineal max']
    config.Navegacion.Velocidad_lineal_max_pto_final= configuracion.Navegacion['Velocidad lineal max pto_final']
    config.Navegacion.Velocidad_lineal_min= configuracion.Navegacion['Velocidad lineal min']
    
    config.Navegacion.Velocidad_angular_max=configuracion.Navegacion['Velocidad angular max']
    config.Navegacion.Velocidad_angular_max_pto_final= configuracion.Navegacion['Velocidad angular max pto_final']
    config.Navegacion.Velocidad_angular_min= configuracion.Navegacion['Velocidad angular min']
    
    config.Navegacion.Margen_desplazamiento_intermedio= configuracion.Navegacion['Margen desplazamiento intermedio']
    config.Navegacion.Margen_desplazamiento_final= configuracion.Navegacion['Margen desplazamiento final']
    config.Navegacion.Margen_giro_intermedio= configuracion.Navegacion['Margen giro intermedio']
    config.Navegacion.Margen_giro_final= configuracion.Navegacion['Margen giro final']

    #res_obj = DictObj(parametros)

    #print res_obj.Obstaculos.Largo

    return config #,res_obj
    
def leer_Archivo(ficheroPath=str):
        """

        Lectura de archivo por lineas y devolucion lista de dicts.

        @parametros -------------------------------------------------------------------------------------------------------------------------------------------------
        
        + ficheroPath -> directorio completo hasta el fichero a leer.

        """
        i=0
        archivo_raw=str
        #ficheroTrayectoria=self.path+'trayectorias/definitivas/'+msg
        print('Directorio = '+ficheroPath)
        #abrir archivo
        try:
            f=open(ficheroPath,"r")
        except IOError as e:
            print("Archivo no encontrado/no existe, o : ",e)
            pass
        #leer el archivo y guardarlo
        archivo_raw=f.readlines()
        archivo=[]
        for line in archivo_raw:
            archivo.append(ast.literal_eval(archivo_raw[i]))
            i+=1
        f.close()
        return archivo

class Nav_utils(object):
    """
    Coleccion de metodos para calculo de velocidades en Navegacion Robot.

    TODO: DEJAR DE PASAR VELOCIDADES COMO ARGS A LAS FUNCIONES Y COGERLAS DE LOS PARAMETROS

    @ Funciones -------------------------------------------------------------------------------------------------------------

    + cuadrante -> Eleccion de cuadrante segun angulo

    + mixto_diferencial_sin -> Calculo de velocidades angulares de ruedas general. Para robots diferenciales y curvatura de la trayectoria segun angulo usando sin.

    + recto_diferencial -> Calculo de velocidades angulares de ruedas especifico para ir en linea recta (dir=1) o en marcha atras (dir=-1)

    + giro -> Calculo de velocidades angulares de ruedas especifico para giros sobre el propio eje del robot.

    + w_wheels_Twist_diferencial -> Calculo de Velocidad Lineal X y Velocidad Angular Z a partir de las velocidades angulares de las ruedas.

    """
    config=leer_Config('')

    vel_lineal_max=config.Navegacion.Velocidad_lineal_max
    vel_lineal=vel_lineal_max
    vel_lineal_min=config.Navegacion.Velocidad_lineal_min

    
    vel_angular_max=config.Navegacion.Velocidad_angular_max
    vel_giro=vel_angular_max
    vel_angular_min=config.Navegacion.Velocidad_angular_min

    diametro_rueda=config.General.Diametro_ruedas
    dist_entre_ruedas=config.General.Distancia_entre_ruedas


    def cuadrante(self,angulo=float):
        """
        Eleccion de cuadrante segun angulo.

        @parametros -------------------------------------------------------------------------------------------------------------------------------------------------

        + angulo -> Angulo en GRADOS.

        """

        if angulo>=0 and angulo<90:
            cuadrante=1  
            # 1+sin
        elif angulo>=90 and angulo<180:
            cuadrante=2
            # 1+sin
        elif angulo>=180 and angulo<=270:
            cuadrante=3
            # -1+sin    
        elif angulo>270 and angulo<360:
            cuadrante=4
            # -1+sin

        print('ELECCION CUADRANTE: Angulo = '+str(angulo)+' -> CUADRANTE = '+str(cuadrante))
        return cuadrante

    def mixto_diferencial_sin(self,angulo=float,vel_lineal=vel_lineal,diametro_rueda= dist_entre_ruedas):
        """

        Calculo de velocidades angulares de ruedas general. Para robots diferenciales y curvatura de la trayectoria segun angulo usando sin.

        @parametros -------------------------------------------------------------------------------------------------------------------------------------------------
        
        + angulo -> Angulo de direccion de movimiento en RADIANES.

        + vel_lineal -> Velocidad lineal deseada. Por defecto el valor en param server.

        + diametro_rueda -> Diametro de las ruedas. Por defecto el valor en param server.
        
        """
        print("CALCULANDO VELOCIDADES MIXTAS")

        wd=(vel_lineal/diametro_rueda/2)*(1+math.sin(angulo))*2
        wi=(vel_lineal/diametro_rueda/2)*(1-math.sin(angulo))*2

        return wd,wi

    def recto_diferencial(self,vel_lineal =  vel_lineal,dir=1,diametro_rueda = diametro_rueda):
        """
        Calculo de velocidades angulares de ruedas especifico para ir en linea recta (dir=1) o en marcha atras (dir=-1)
        
        @parametros -------------------------------------------------------------------------------------------------------------------------------------------------
        
        + vel_lineal -> Velocidad lineal deseada. Por defecto el valor en param server.

        + dir -> Sentido de la direccion de movimiento.

        + diametro_rueda -> Diametro de las ruedas. Por defecto el valor en param server.
        
        """
        if dir==1:
            print("CALCULANDO VELOCIDADES RECTO")
        elif dir==-1:
            print("CALCULANDO VELOCIDADES MARCHA ATRAS")
        else:
            print("ERROR : VALOR DE VARIABLE 'dir' NO VALIDO")
            return 0,0

        wd=dir*(vel_lineal/diametro_rueda/2)*2
        wi=dir*(vel_lineal/diametro_rueda/2)*2

        return wd,wi

    def giro(self,ang_giro=float,vel_giro =  vel_giro,diametro_rueda =  diametro_rueda,dist_entre_ruedas =  dist_entre_ruedas,vel_angular_min=vel_angular_min) :  ## problemas en +-90/180/+-270 grados (parcheados con el if==+-180 y con los cuadrantes, pero ?suficiente?) 

        """
        Calculo de velocidades angulares de ruedas especifico para giros sobre el propio eje del robot.

        Division de cuadrantes para giro mas corto.

        NOTAS: problemas en +-90/180/+-270 grados (parcheados con el if==+-180 y con los cuadrantes, pero ?suficiente?)

        @parametros -------------------------------------------------------------------------------------------------------------------------------------------------

        + ang_giro -> Angulo de giro deseado en RADIANES.

        + vel_giro -> Velocidad de giro deseada. Por defecto el valor en param server.

        + diametro_rueda, dist_entre_ruedas -> Diametro de ruedas y distancia entre ruedas. Por defecto el valor en param server.

        + vel_angular_min -> Velocidad angular deseada minima. Por defecto el valor en param server.

        """

        print("CALCULANDO VELOCIDAD GIRO")
        cuad=self.cuadrante(math.degrees(ang_giro)%360)
        #rospy.logwarn("cuadrante: %s; angulo %s",cuad,angg)
        ## si quiero poner macha atras, tendria q poner un bucle aqui q cnd ang==180, se mueva recto -1 hasta q modV sea menor q margen. Como saldria de ang=180 entonces?
        if math.degrees(ang_giro)==180 or math.degrees(ang_giro)==-180:
            wd=(vel_giro*math.cos(ang_giro)/diametro_rueda/2)
        else:
            if cuad==1:
                wd=(vel_giro*(1+math.sin(ang_giro))/diametro_rueda/2)
            elif cuad==2:
                wd=(vel_giro*(1+math.sin(ang_giro))/diametro_rueda/2)
            elif cuad==3:
                wd=(vel_giro*(-1+math.sin(ang_giro))/diametro_rueda/2)
            elif cuad==4:
                wd=(vel_giro*(-1+math.sin(ang_giro))/diametro_rueda/2)
            if abs(wd)<(vel_angular_min*dist_entre_ruedas/diametro_rueda):
                wd=wd/abs(wd)*(vel_angular_min*dist_entre_ruedas/diametro_rueda)
        wi=-wd

        return wd,wi

    def w_wheels_Twist_diferencial(self,wder=float,wizq=float,diametro_rueda =  diametro_rueda,dist_entre_ruedas =  dist_entre_ruedas,vel_lineal_max =  vel_lineal_max,vel_lineal_min =  vel_lineal_min,vel_angular_max =  vel_angular_max,vel_angular_min =  vel_angular_min):

        """
        Calculo de Velocidad Lineal X y Velocidad Angular Z a partir de las velocidades angulares de las ruedas.

        @parametros -------------------------------------------------------------------------------------------------------------------------------------------------

        + wizq,wder -> Velocidades diferenciales de las ruedas.

        + diametro_rueda,dist_entre_ruedas -> Diametro de ruedas y distancia entre ruedas. Por defecto el valor en param server.

        + vel_lineal -> Velocidades lineales. Por defecto el valor en param server.

        + vel_angular -> Velocidades angulares. Por defecto el valor en param server.

        """
        print('CALCULO DE VELS LINEAL Y ANGULAR')
        vel_msg_Twist=geometry_msgs.msg.Twist()
        vel_linearX=(wizq+wder)*(diametro_rueda/2)                     
        velTh=(wder-wizq)*(diametro_rueda/2)/dist_entre_ruedas 

        # VELOCIDAD LINEAL
        vel_msg_Twist.linear.x = round(vel_linearX,2)   ### de ya en m/min
        vel_msg_Twist.linear.y = 0
        vel_msg_Twist.linear.z = 0
        # VELOCIDAD ANGULAR
        vel_msg_Twist.angular.x = 0
        vel_msg_Twist.angular.y = 0
        vel_msg_Twist.angular.z =round(velTh,2)   ### ya en grados/min

        #### velocidades maximas y minimas
        if vel_msg_Twist.linear.x>0:
            vel_msg_Twist.linear.x=min(vel_msg_Twist.linear.x,vel_lineal_max)
            vel_msg_Twist.linear.x=max(vel_msg_Twist.linear.x,vel_lineal_min)
        elif vel_msg_Twist.linear.x<0:
            vel_msg_Twist.linear.x=max(vel_msg_Twist.linear.x,-vel_lineal_max)
            vel_msg_Twist.linear.x=min(vel_msg_Twist.linear.x,-vel_lineal_min)
        if vel_msg_Twist.angular.z>0:
            vel_msg_Twist.angular.z=min(vel_msg_Twist.angular.z,vel_angular_max)
            vel_msg_Twist.angular.z=max(vel_msg_Twist.angular.z,vel_angular_min)
        elif vel_msg_Twist.angular.z<0:
            vel_msg_Twist.angular.z=max(vel_msg_Twist.angular.z,-vel_angular_max)
            vel_msg_Twist.angular.z=min(vel_msg_Twist.angular.z,-vel_angular_min)

        return vel_msg_Twist

    