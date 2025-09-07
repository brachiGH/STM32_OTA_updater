/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"
#include "stm32f4xx_hal_flash_ex.h"

#include "uartCommands.h"
#include "bootGPIO.h"
#include "uart6.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*firmware_main_func_ptr)(void);

typedef struct
{
  unsigned long int firmwareVersionId;
  unsigned long int deviceModelId;
  unsigned long int deviceUniqueId;
} FIRMWARE_INFO;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRMWARE_ID_ADDRESS 0x0807FFF8UL

#define EMPTY_MEM 0xFFFFFFFFUL
#define FIRMWARE_EMPTY_MEM 0xFFFFFF00UL

#define MAX_ACK_CHECK 30
#define MAX_VERSION_CHECK 3
#define MAX_UPDATE_TRIES 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RETURN_ON_FALSE(expr) \
  do                          \
  {                           \
    if ((expr) == false)      \
      return false;           \
  } while (0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t application_start __attribute__((section(".application")));

const FIRMWARE_INFO _deviceInfo_Factory = {
    .firmwareVersionId = 1,
    .deviceModelId = 1,
    .deviceUniqueId = 1,
};
uint32_t _firmwareVersionId;

bool downloadRecoverFirmware = false;

/* Flash Related variables*/
uint32_t SectorError;
FLASH_EraseInitTypeDef FlashErase;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
static void jump_to_app();

bool firmwareInfoCheck();
void RestratDeviceIn(uint32_t timeoutSec);

bool sendCMDRecvData(const uint8_t cmd, const uint8_t *data, size_t datalen, void *recvBuff, uint16_t *recvBufflen);
bool sendCMD(const uint8_t cmd, const uint8_t *data, size_t datalen);

uint32_t CRC_Calculate(uint8_t *data, size_t len);
uint32_t clearFlash();
void saveFirmwareInfo();
uint32_t firmwareFlashWrite(uint8_t *data, uint32_t writeIndex, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  uart6_write(ch);
  return ch;
}

void println(const char *format, ...)
{
  va_list arg;
  char data[UART_MAX_SEND_BUFF_SIZE];
  data[0] = __SYN;
  data[1] = _CMD_LOG;

  va_start(arg, format);
  size_t len = vsnprintf(data + 2, UART_MAX_SEND_BUFF_SIZE, format, arg);
  va_end(arg);
  for (size_t i = 0; i < len + 2; i++)
  {
    uart6_write(data[i]);
  }
  for (int i = 0; i < 50000; i++)
  {
  }
}

static void printDeviceInfo()
{
  println("***Device Info :*******************");
  println("Firmware Version ID: %lu", _firmwareVersionId);
  println("Device Model: %lu", _deviceInfo_Factory.deviceModelId);
  println("Current device ID: %lu", _deviceInfo_Factory.deviceUniqueId);
  println("***********************************");
}

static void LOG_pingFailed()
{
  println("------------------------------------------------------");
  println("Check if device connected properly.\n\r");

  println("Restart device while holding blue button to try again.");
}

static void LOG_getLastestFailed()
{
  println("------------------------------------------------------");
  println("Check if device connected properly.");
  println("Problem with checking latest version, current app maybe corrupted\n\r");

  println("Restart device while holding blue button to try again.");
}

static void LOG_clearFlashFailed()
{
  println("------------------------------------------------------");
  println("Clear Failed!");

  println("Restart device while holding blue button to try again.");
}

static void LOG_updateFailed()
{
  println("------------------------------------------------------");
  println("Updating Faild, check internet connection.\n\r");

  println("Restart device while holding blue button to try again.");
}

static void LOG_updateWriteFailed()
{
  println("------------------------------------------------------");
  println("Updating Failed, Error While Writing to flash.\n\r");

  println("Restart device while holding blue button to try again.");
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  _firmwareVersionId = *((uint32_t *)FIRMWARE_ID_ADDRESS);
  BOOT_GPIO_init();
  uart6_rxtx_init();

  println("Bootloader Started...");
  printDeviceInfo();

  if (!firmwareInfoCheck())
  {
    println("Can't Start, Firmware corrupted!");
    println("Downloading Factory Firmware...");
  }
  else
  {
    if (GPIOC->IDR & BTN_PIN)
    {
      /** Rest GPIO & uart6 */
      RCC->AHB1ENR = 0;
      GPIOA->MODER = 0xA800 0000;
      GPIOC->MODER = 0;
      GPIOC->AFR[0] = 0;
      RCC->APB2ENR = 0;
      USART6->CR1 = 0;
      USART6->BRR = 0;

      jump_to_app();

      // If jump failed downlaod factory firmware. (if jump_to_app fails it returns)
      downloadRecoverFirmware = true;
      println("Begin Downloading Factory Firmware...");
    }
  }

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  SystemCoreClockUpdate();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  if (!downloadRecoverFirmware)
  {
    println("Keep pressing the blue button untill the led start linking to downlaod factory firmware...");
    GPIOA->BSRR = LED_PIN; // Turn on led
    // wait for user to hold btn, to download factory firmware
    HAL_Delay(4000);
    GPIOA->BSRR = (LED_PIN << 16); // Turn off led
    if (GPIOC->IDR & BTN_PIN)
    {
      println("Begin Downloading Lastest Firmware...");
    }
    else
    { // btn pressed
      downloadRecoverFirmware = true;
      println("Begin Downloading Factory Firmware...");
    }
  }

  tim2_20hzLedBinkBegin();

  /* Checking the updater is connected*/
  println("Wait for Ping from updater device....");
  uint16_t ackCount = 0;
  while (1)
  {
    if (sendCMD(_CMD_PING, NULL, 0))
    {
      break;
    }
    HAL_Delay(10);
    println("Attempt %u: FAILED.", ++ackCount);
    if (ackCount == MAX_ACK_CHECK)
    {
      LOG_pingFailed();
      while (1)
      {
        /* do nothing */
      }
    }
  }

  if (!downloadRecoverFirmware)
  {
    /* Checking for new Version  */
    uint8_t lastestVersionID[4];
    uint16_t lastestVersionLen = 4;
    uint16_t versionCheckCount = 0;
    while (!sendCMDRecvData(_CMD_GET_LASTEST_VERSION_ID, (uint8_t *)(&_deviceInfo_Factory.deviceModelId),
                            4, lastestVersionID, &lastestVersionLen))
    {
      HAL_Delay(1000);
      versionCheckCount++;
      if (versionCheckCount == MAX_VERSION_CHECK)
      {
        LOG_getLastestFailed();
        while (1)
        {
          /* do nothing */
        }
      }
    }

    if (*(uint32_t *)(lastestVersionID) == _firmwareVersionId)
    {
      println("Device is UpTodate.");
      RestratDeviceIn(3);
      /** end of update checking **/
    }

    _firmwareVersionId = *(uint32_t *)lastestVersionID;
  }
  else
  {
    // Setting factory version ID as the requested download firmware
    _firmwareVersionId = _deviceInfo_Factory.firmwareVersionId;
  }

  if (clearFlash() != 0)
  {
    LOG_clearFlashFailed();

    while (1)
    {
    };
  }

  uint8_t firmwareData[UART_MAX_RECV_BUFF_SIZE] = {0};
  uint16_t firmwareDataLen = UART_MAX_RECV_BUFF_SIZE;
  uint16_t chunkLen = UART_MAX_RECV_BUFF_SIZE - 4;
  uint8_t updateInfo[10] = {0};
  *(uint32_t *)updateInfo = _deviceInfo_Factory.deviceModelId;
  *(uint32_t *)(updateInfo + 4) = _firmwareVersionId;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint16_t retryCount = 0;
    while (!sendCMDRecvData(_CMD_GET_UPDATE, updateInfo,
                            10, firmwareData, &firmwareDataLen))
    {
      HAL_Delay(1000);
      retryCount++;
      if (retryCount > MAX_UPDATE_TRIES)
      {
        LOG_updateFailed();

        while (1)
        {
          /* do nothing */
        }
      }
    }

    uint32_t writeIndex = *(uint16_t *)(updateInfo + 8) * chunkLen;
    if (firmwareFlashWrite(firmwareData + 4, writeIndex, chunkLen) != 0)
    {
      LOG_updateWriteFailed();
      while (1)
      {
        /* do nothing */
      }
    }

    if (*(uint32_t *)firmwareData == EMPTY_MEM)
    {
      (*(uint16_t *)(updateInfo + 8))++; // length is in 4092bytes
      println("%lu Bytes Writen", writeIndex + chunkLen);
      continue;
    }
    else
    {
      saveFirmwareInfo();
      println("---------------\n\r");
      println("Updating Done!.");

      RestratDeviceIn(3);
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_9B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_EVEN;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void jump_to_app()
{
  uint32_t app_start_address;
  firmware_main_func_ptr jump_to_app;
  uint32_t firmwareMSP = *(uint32_t *)(&application_start);

  if (firmwareMSP != FIRMWARE_EMPTY_MEM)
  {
    println("Starting application.....");
    app_start_address = *(uint32_t *)(&application_start + 4);

    jump_to_app = (firmware_main_func_ptr)app_start_address;

    /*Initialialize main stack pointer*/
    __set_MSP(firmwareMSP);

    /*jump*/
    jump_to_app();
  }
  else
  {
    println("No application found at location....");
  }
}

bool firmwareInfoCheck()
{
  if (_firmwareVersionId == EMPTY_MEM)
  {
    return false;
  }

  uint32_t firmwareIDCopy = *(uint32_t *)(FIRMWARE_ID_ADDRESS + 4);
  uint32_t crc = _firmwareVersionId ^ firmwareIDCopy;
  return crc == 0UL;
}

void RestratDeviceIn(uint32_t timeoutSec)
{
  println("Device restarting in %luSecond(s).", timeoutSec);
  for (; timeoutSec > 0; timeoutSec--)
  {
    HAL_Delay(1000);
    println("%lu...", timeoutSec);
  }
  // rest Intr
  NVIC_SystemReset();
}

uint32_t CRC_Calculate(uint8_t *data, size_t len)
{
  uint32_t crc = 0;
  size_t templen = len - ((len % 4 == 0) ? 0 : 1);
  size_t i = 0;
  for (; i < templen; i += 4)
  {
    crc ^= *(uint32_t *)(data + i);
  }
  if (len % 4 != 0)
  {
    uint8_t temp[4];
    for (; i < templen + 4; i++)
    {
      if (i < len)
      {
        temp[i % 4] = data[i];
      }
      else
      {
        temp[i % 4] = 0x00;
      }
    }

    crc ^= *(uint32_t *)temp;
  }

  return crc;
}

bool sendCMD(const uint8_t cmd, const uint8_t *data, size_t datalen)
{
  return sendCMDRecvData(cmd, data, datalen, NULL, NULL);
}

bool sendCMDRecvData(const uint8_t cmd, const uint8_t *data, size_t datalen, void *retBuff, uint16_t *recvBufflen)
{
  RETURN_ON_FALSE(UART_MAX_SEND_BUFF_SIZE >= datalen);

  uint16_t bufferLen = datalen * 2 + 2;
  uint8_t buffer[bufferLen];
  uint8_t recvBuff[UART_MAX_CMD_BUFF_SIZE] = {0};
  buffer[0] = __SYN;
  buffer[1] = cmd;

  if (data != NULL && datalen > 0)
  {
    memcpy(buffer + 2, data, datalen);
    memcpy(buffer + datalen + 2, data, datalen); // copy for checking data integrity
  }

  __HAL_UART_FLUSH_DRREGISTER(&huart6);
  RETURN_ON_FALSE(HAL_UART_Transmit(&huart6, buffer, bufferLen, 1000) == HAL_OK);
  HAL_UART_Receive(&huart6, recvBuff, UART_MAX_CMD_BUFF_SIZE, 100);
  RETURN_ON_FALSE(recvBuff[0] == __ACK);

  uint16_t len = *(uint16_t *)(recvBuff + 1);
  if (len == 0 || retBuff == NULL || recvBufflen == NULL)
  {
    return true;
  }

  if (len > *recvBufflen)
  {
    len = *recvBufflen;
  }
  else
  {
    *recvBufflen = len;
  }

  uint8_t temp[len];
  __HAL_UART_FLUSH_DRREGISTER(&huart6);
  HAL_UART_Receive(&huart6, temp, len + 4, 20000);
  uint32_t crc_result = CRC_Calculate((temp + 4), len);
  if (*(uint32_t *)temp != crc_result)
  {
    return false;
  }
  memcpy(retBuff, temp + 4, len);
  return true;
}

uint32_t clearFlash()
{
  println("Deleting Flash...\r\n");

  HAL_FLASH_Unlock(); // unlock the flash

  FlashErase.Sector = FLASH_SECTOR_2;
  FlashErase.NbSectors = 6;
  FlashErase.TypeErase = FLASH_TYPEERASE_SECTORS;
  FlashErase.VoltageRange = VOLTAGE_RANGE_3;

  // The erase function
  if (HAL_FLASHEx_Erase(&FlashErase, &SectorError) != HAL_OK)
  {
    println("Flash Delete status : %lx \r\n", HAL_FLASH_GetError());
    return HAL_FLASH_GetError();
  }

  println("Flash Delete status : %lx \r\n", 0);

  HAL_FLASH_Lock(); // lock the flash

  return 0;
}

void saveFirmwareInfo()
{
  HAL_FLASH_Unlock();                                       // unlock the flash
  uint32_t FlashMemAddress = (uint32_t)FIRMWARE_ID_ADDRESS; // Start writing from the 1st address
  for (size_t i = 0; i < 2; i++)
  {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FlashMemAddress, _firmwareVersionId);
    FlashMemAddress += 4; // 4 byte increment
  }
  HAL_FLASH_Lock(); // lock the flash
}

uint32_t firmwareFlashWrite(uint8_t *data, uint32_t writeIndex, uint16_t len)
{
  HAL_FLASH_Unlock(); // unlock the flash

  uint32_t FlashMemAddress = (uint32_t)(&application_start) + writeIndex;
  for (size_t i = 0; i < len; i++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FlashMemAddress, data[i]) != HAL_OK)
    {
      return HAL_FLASH_GetError();
    }
    FlashMemAddress += 1; // 1 byte increment
  }
  HAL_FLASH_Lock(); // lock the flash

  return 0;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
