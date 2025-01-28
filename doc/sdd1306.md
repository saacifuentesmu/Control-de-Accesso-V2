# Using SSD1306 to STM32 Project

## Driver de SSD1306 para STM32

Descargar desde [afiskon/stm32-ssd1306](https://github.com/afiskon/stm32-ssd1306)

## Agregar SSD1306 al proyecto
8
1. Copie la carpeta */ssd1306* desde la descargar al directorio *4100901_Access_Control_System/External/ssd1306*
    * Debe crear */External* en el directorio raiz del proyecto
2. Agregue en el *CMakelists.txt* los archivos *External/ssd1306/ssd1306.c* y *External/ssd1306/ssd1306_fonts.c*
```CMake
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/button.c
    Core/Src/state_machine.c
    External/ssd1306/ssd1306.c
    External/ssd1306/ssd1306_fonts.c
    # Add user sources here
)
```
3. Agregue al *CMakeLists.txt* la carpeta */External/ssd1306* para incluir los encabezados en el proyecto.
```CMake
# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    External/ssd1306
    # Add user defined include paths
)
```

## Configurar la librería SDD1306
1. En la carpeta de ssd1306, renombrar el archivo ***ssd1306_config_template.h*** a ***ssd1306_config.h***
2. En el archivo ***ssd1306_config.h*** comentar la linea 10 (*#define STM32F0*) y descomentar la linea 15 (*#define STM32L4*) para compilar para el **STM32L476RG**
3. Asegurese de que el driver esté configurado con I2C (en lugar de SPI).

## Agregar el driver de I2C1 en STM32CubeMX
1. Abra STM32CubeMX -> Connectivity -> I2C1 y seleccione la opcion I2C
2. Con la tecla CTRL presionada, arrastre los pines del I2C1 a PB8 y PB9. Vea [Nucleo-CAD](https://www.st.com/en/evaluation-tools/nucleo-l476rg.html#cad-resources)
3. Genere el proyecto y compile

## Usar las funciones de la librería en el programa.

### Inicializar el display
1. Vaya al archivo *ssd1306.h* donde puede encontrar las funciones disponibles de la librería (debe incluir este header en la seccion correcta de *main.c*)
2. Llame la función de inicialización para configurar el display
3. Llame la función de llenado para llenar la pantalla de un color determinado
4. Llame la function actualizar para refrescar la pantalla
```C
  ssd1306_Init();
  ssd1306_Fill(White);
  ssd1306_UpdateScreen();
```

### Poner un texto en pantalla
1. Vaya al archivo *ssd1306_fonts.h* donde puede encontrar los tipos de fuente disponibles de la librería (debe incluir este header en la seccion correcta de *main.c*)
2. Llame la función para fijar el cursor, por ejemplo en el pixel x=20, y=20
3. Llame la función de escritura de string para agregar el string al display
4. Llame la function actualizar para refrescar la pantalla
```C
  ssd1306_SetCursor(20, 20);
  ssd1306_WriteString("Hello, World!", Font_7x10, Black);
  ssd1306_UpdateScreen();
```

### Muestre una imagen en pantalla
