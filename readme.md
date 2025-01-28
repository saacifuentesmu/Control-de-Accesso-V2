# Guía de Implementación: Sistema de Control de Acceso usando STM32CubeMX

## Introducción

Esta guía te ayudará a implementar un sistema de control de acceso utilizando un microcontrolador STM32L476RG. La configuración inicial se realizará usando STM32CubeMX, lo que simplificará la inicialización de periféricos. El entorno de desarrollo será STM32CubeIDE, que incluye tanto STM32CubeMX como las herramientas de desarrollo necesarias.

El proyecto incluye:
* Una máquina de estados para controlar la puerta
* Comunicación UART (USART2)
* Control de LEDs y detección de eventos de botón
* Temporización usando SysTick

## Sección 0: Instala STM32CubeMX

* Descarga [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html#st-get-software)
* Instala el software. Puedes seguir este [tutorial en español.](https://www.youtube.com/watch?v=vK0mvabEwDM)
* Añade a STM32 VS Code Extension: Extensions -> STM32 VS Code Extension -> Settings.

![STM32 VS Code Settings Path](doc/assets/stm32_vs_code_settings_path.png)
![STM32 VS Code Settings](doc/assets/stm32_vs_code_settings.png)

## Sección 1: Creación del Proyecto

### 1.1 Configuración Inicial en STM32CubeMX

1. Abre STM32CubeMX
2. Selecciona "New Project"
3. En la pestaña "Board Selector", busca y selecciona "NUCLEO-L476RG"
4. Haz clic en "Start Project"
5. Cuando pregunta si inicializar periféricos por defecto, selecciona "Yes"

![STM32 VS Code start](doc/assets/stm32cubemx_start.png)

### 1.2 Configuración de Periféricos

#### Configuración del Reloj
1. Ve a la pestaña "Clock Configuration"
2. Configura HCLK (MHz) a 80MHz para máximo rendimiento
3. Asegúrate que el USART2 esté usando PCLK1 como fuente de reloj

![STM32 VS Code clock](doc/assets/stm32cubemx_clock.png)

#### Configuración de GPIO
1. En la pestaña "Pinout & Configuration":
   * PA5 (LED): Configura como "GPIO_Output" (LD2)
   * PC13 (Button): Configura como "GPIO_EXTI13" (B1)
   * PA4: Configura como "GPIO_Output" (LED estado puerta)

![STM32 VS Code gpio](doc/assets/stm32cubemx_gpio.png)

#### Configuración USART2
1. En la sección "Connectivity":
   * Habilita USART2
   * Mode: Asynchronous
   * Basic Parameters:
     * Baud Rate: 115200
     * Word Length: 8 Bits
     * Parity: None
     * Stop Bits: 1
   * En NVIC Settings:
     * Habilita "USART2 global interrupt"

![STM32 VS Code uart](doc/assets/stm32cubemx_uart.png)

#### Configuración EXTI
1. Para PC13 (ya configurado como EXTI13):
   * GPIO mode: External Interrupt Mode with Falling edge trigger detection
   * En NVIC Settings:
     * Habilita "EXTI line[15:10] interrupts"

![STM32 VS Code nvic](doc/assets/stm32cubemx_nvic.png)

### 1.3 Generación del Código

1. Ve a "Project Manager":
   * Project Name: `4100901-Access_Control_System`
   * Project Location: Selecciona tu directorio
   * Application Structure: Advanced
   * Toolchain/IDE: CMake
2. Click en "Generate Code"
3. Cuando pregunte si abrir carpeta ("Open Folder"), selecciona "Yes"

![STM32 VS Code Project](doc/assets/stm32cubemx_project.png)

### 1.4 Importación del Proyecto en VS Code

1. Abre VS Code y selecciona STM32 VS Code Extension -> Import CMAKE Project
2. Navega hasta la carpeta del proyecto generado y selecciónala
4. Cuando VS Code detecte el proyecto CMake, selecciona "Import Project"
5. Presiona el boton de Build y selecciona la opción "Debug"

![STM32 VS Code Import](doc/assets/stm32cubemx_import.png)

## Sección 2: Implementación del Código

### 2.1 Estructura del Proyecto
El proyecto generado tendrá esta estructura:
```
proyecto/
├── Core/
│   ├── Inc/         #Archivos de cabecera (.h)
│   │   ├── main.h               # Autogenerado por STM32CubeMX
│   │   ├── stm32l4xx_hal_conf.h # Autogenerado por STM32CubeMX
│   │   ├── stm32l4xx_it.h       # Autogenerado por STM32CubeMX
│   │   └── button.h
│   └── Src/         # Archivos fuente (.c)
│       ├── main.c               # Autogenerado por STM32CubeMX
│       ├── stm32l4xx_hal_msp.c  # Autogenerado por STM32CubeMX
│       ├── stm32l4xx_it.c       # Autogenerado por STM32CubeMX
│       └── button.c
├── Drivers/                 # Drivers HAL y CMSIS
├── CMakeLists.txt           # To include sources and header directories
└── .project                 # Archivos de configuración IDE
```

### 2.2 Implementación del driver de boton
Crea `button.c` en `Core/Src/` con la ***definición*** de la funcion de antirebote y detección de soble presión:
```c
#include "button.h"
#include "main.h"

uint32_t b1_tick = 0; // para detectar antirebote y doble presion basado en el ultimo evento

uint8_t detect_button_press(void) {
    uint8_t button_pressed = 0;
    if (HAL_GetTick() - b1_tick < 50) {
        // Ignore bounces less than 50ms
    } else if (HAL_GetTick() - b1_tick > 500) {
        button_pressed = 1; // single press
    } else {
        button_pressed = 2; // double press
    }
    b1_tick = HAL_GetTick();
    return button_pressed;
}

```

Crea `button.h` en `Core/Inc/` con la ***declaración*** de la función:
```c
#ifndef __BUTTON_H_
#define __BUTTON_H_

#include <stdint.h>

uint8_t detect_button_press(void);

#endif // __BUTTON_H_

```

### 2.3 Modificaciones del Main

En `main.c`, ahora agregamos algunas líneas de código necesarias. asegurese de agregar las lineas en los lugares correctos usando los tags de los comentarios.

```c
/* USER CODE BEGIN Includes */
#include "button.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
uint8_t rx_byte = 0;
uint8_t b1_byte = 0;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == B1_Pin) {
        b1_byte = detect_button_press();
    }
}

void heartbeat_handler(void)
{
  static uint32_t heartbeat_tick = 0;
  if (HAL_GetTick() - heartbeat_tick >= 500) {
    heartbeat_tick = HAL_GetTick();
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  HAL_UART_Transmit(&huart2, (uint8_t *)"Hello, World!\r\n", 15, 1000);
  while (1) {
    if (b1_byte != 0) {
      if (b1_byte == 1) {
        HAL_UART_Transmit(&huart2, "1\r\n", 3, 10); // echo received byte
      } else if (b1_byte == 2) {
        AL_UART_Transmit(&huart2, "2\r\n", 3, 10); // echo received byte
      }
      b1_byte = 0;
    }

    if (rx_byte != 0) {
        HAL_UART_Transmit(&huart2, &rx_byte, 1, 10); // echo received byte
        rx_byte = 0;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

```


### 2.4 Manejadores de Interrupción
Asegúrate de que estos manejadores estén definidos en el archivo `stm32l4xx_it.c`:

```c
/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(B1_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}
```

Estos son los puntos de entrada de las interrupciones de los periféricos. Luego el HAL llamará los callback que vamos a implementar en la aplicación.

## Sección 3: Diferencias Clave con la Implementación Manual

1. **Inicialización de Periféricos**
   * STM32CubeMX genera automáticamente el código de inicialización
   * Usa la HAL (Hardware Abstraction Layer) en lugar de acceso directo a registros
   * La configuración se realiza mediante interfaz gráfica

2. **Manejo de Interrupciones**
   * Los callbacks de interrupción son manejados por la HAL
   * No es necesario configurar manualmente los registros NVIC
   * Los nombres de las funciones callback son estandarizados

3. **Funciones HAL vs Registro**
   * `HAL_GPIO_WritePin()` vs acceso directo a ODR
   * `HAL_GetTick()` vs variable global `ms_counter`
   * `HAL_UART_Transmit()` vs escribir al registro TDR

4. **Ventajas del Enfoque CubeMX**
   * Configuración más rápida y menos propensa a errores
   * Código más portable entre diferentes MCUs STM32
   * Mejor documentación y soporte
   * Actualizaciones y correcciones de bugs automáticas
