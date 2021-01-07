# rover-visual
Implementación de un Rover que usa las herramientas de Visión de CoppeliaSim para las diferentes tareas de navegación y manipulación. 

Este sería capaz de: 
-	Detectar y seguir las líneas usando BlobDetection y Color filtering (Binary, o colorSegmentation)
-	Detectar cajas usando herramientas similares, centrarse en ellas, usar sensores de proximidad para recoger. 
-	Buscar la pista en un giro de 180 grados tras asegurar la caja.
-	Implementar una máquina de estados para decidir qué hacer según las situaciones, documentar y mostrar los estados, un diagrama del razonamiento, resultados.
-	Distribuir el trabajo de cada subsistema en scripts separados (en los joints, por ejemplo), y usar señales para comunicación de lo que se requiere en cada estado. 

## Detalles

Estarán usando el robot Valkyria para la implementación. Este ya se encuentra en una pista de Rover en la escena de coppeliaSim del archivo _Rover-Visual.**ttt**_. Esta escena cuenta con una interfaz para posicionar el vehículo en los diferentes puntos de inicio de cada color de la pista, que se puede modificar en el script de la Mesa.

El modelo del robot contiene un script cuya única función es llamar al archivo "_rover_.lua", que debería ubicarse en el mismo directorio que la escena. 

El manipulador de Valkyria funciona con uno de los grippers que tiene disponible coppeliaSim en su lista de modelos. Este gripper tiene su propio script de control y funciona por medio de señales para abrirlo y cerrarlo. Ver las funciones _abrirDedos_ y _cerrarDedos_.

Es *necesario* modificar (o reemplazar) los sensores de visión que tiene el modelo del robot por otros que estén ubicados y configurados de manera que permitan lograr lo esperado.
   - Por ende, es **primordial** adaptar el vehículo antes de empezar a hacer los trabajos.
   - Dejar los sensores anteriores no es buena idea pues consumirían recursos de simulación.

El archivo _rover.lua_ contiene las funciones básicas para obtener las informaciones de los sensores del robot y generar los movimientos de sus partes.

Las funciones a usar para el procesamiento de imagen son parte del Plugin de Visión (`simVision`) para [Procesamiento Simple de Imágenes](https://www.coppeliarobotics.com/helpFiles/en/visionPlugin-imageProcessing.htm) que trae coppeliaSim (y se basa en OpenCV). Otras funciones están disponible en el Plugin de [Imágenes](https://www.coppeliarobotics.com/helpFiles/en/imageApi.htm) (`simIM`) que tiene funciones básicas para transformar imágenes.

## Sobre el procesamiento de imágenes en CoppeliaSim

Las tareas que se ejecutan en imágenes se hacen en un buffer que contienen la información de la imagen extraída de un sensor de visión. Para más información ver la descripción de:

  - Visión: 
    -  `simVision.sensorImgToWorkImg`
    -  `simVision.workImgToSensorImg`
    -  `simVision.workImgToBuffer1`

  - Imágenes: 
    - `simIM.readFromVisionSensor`
    - `simIM.writeToVisionSensor`

En el código base, a este proceso se le denomina **extraer** una imagen (ver la función `extraerImagen(sensor)`).

El API regular de CoppeliaSim también permite leer la imagen de un sensor de visión usando una manera similar a leer los datos de colores del mismo. La [documentación](https://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetVisionSensorImage.htm) de esta función **sí** devuelve una tabla de valores RGB con cada pixel.

En el código base, a este proceso se le denomina **leer** una imagen.(ver la función `leerImagen(sensor)`).

Es **muy importante** llevar control de cuánta información está almacenando el programa, pues esto ralentiza la simulación, así que es necesario tomar en cuenta cómo y cuándo se leen y almacenan estas imágenes.