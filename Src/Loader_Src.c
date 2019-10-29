/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "quadspi.h"
#include "gpio.h"
#include "Loader_Src.h"
#include "Dev_Inf.h"
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static QSPI_CommandTypeDef sCommand;
__IO uint8_t CmdCplt, RxCplt, TxCplt, StatusMatch, TimeOut;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static int QSPI_WritePage(unsigned long adr, unsigned long sz, unsigned char *buf);
static int QSPI_WriteEnable(QSPI_HandleTypeDef *qspiHandle);
static int QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *qspiHandle);
static int QUADSPI_MappedMode(QSPI_HandleTypeDef *qspiHandle);
static int EraseSector(unsigned long adr);
static void ResetMemory(QSPI_HandleTypeDef *qspiHandle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int  QSPI_WritePage(unsigned long adr, unsigned long sz, unsigned char *buf)
{
  QSPI_WriteEnable(&hqspi);

  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.AlternateBytesSize = 0;
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = 0x32;
  sCommand.AddressMode = QSPI_ADDRESS_4_LINES;
  sCommand.DataMode    = QSPI_DATA_4_LINES;
  sCommand.NbData      = sz;
  sCommand.DummyCycles = 0;
  sCommand.Address = adr & 0x7FFFFF;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

//  TxCplt = 0;
  if (HAL_QSPI_Transmit(&hqspi, buf, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

//  while(TxCplt == 0);

  HAL_Delay(2);
  return 1;
}

static int QSPI_WriteEnable(QSPI_HandleTypeDef *qspiHandle)
{
  QSPI_CommandTypeDef     tCommand;
  QSPI_AutoPollingTypeDef sConfig;

  /* Enable write operations ------------------------------------------ */
  tCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  tCommand.Instruction       = WRITE_ENABLE_CMD;
  tCommand.AddressMode       = QSPI_ADDRESS_NONE;
  tCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  tCommand.DataMode          = QSPI_DATA_NONE;
  tCommand.DummyCycles       = 0;
  tCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  tCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  tCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(qspiHandle, &tCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure automatic polling mode to wait for write enabling ---- */
  sConfig.Match           = 0x02;
  sConfig.Mask            = 0x02;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x20;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  tCommand.Instruction    = READ_STATUS_REG_CMD;
  tCommand.DataMode       = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(qspiHandle, &tCommand, &sConfig, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  return 1;
}

static int QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *qspiHandle)
{
  QSPI_CommandTypeDef     tCommand;
  QSPI_AutoPollingTypeDef sConfig;

  /* Configure automatic polling mode to wait for memory ready ------ */
  tCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  tCommand.Instruction       = READ_STATUS_REG_CMD;
  tCommand.AddressMode       = QSPI_ADDRESS_NONE;
  tCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  tCommand.DataMode          = QSPI_DATA_1_LINE;
  tCommand.DummyCycles       = 0;
  tCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  tCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  tCommand.SIOOMode         = QSPI_SIOO_INST_EVERY_CMD;

  sConfig.Match           = 0x00;
  sConfig.Mask            = 0x01;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x20;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling(qspiHandle, &tCommand, &sConfig, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  return 1;
}

static int QUADSPI_MappedMode(QSPI_HandleTypeDef *qspiHandle)
{
  char MemoryMappedMode = 0;

  if (!MemoryMappedMode)
  {
    QSPI_MemoryMappedTypeDef sMemMappedCfg;

    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.AlternateBytesSize = 0;
    sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction = 0xEB;
    sCommand.AddressMode = QSPI_ADDRESS_4_LINES;
    sCommand.DataMode    = QSPI_DATA_4_LINES;
    sCommand.DummyCycles = 6; //no * 2

    sMemMappedCfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;

    if (HAL_QSPI_MemoryMapped(qspiHandle, &sCommand, &sMemMappedCfg) != HAL_OK)
    {
      Error_Handler();
    }
    MemoryMappedMode = 1;
  }
  return 1;
}

static int EraseSector (unsigned long adr)
{
  QSPI_WriteEnable(&hqspi);

  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.AlternateBytesSize = 0;
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = 0x20;
  sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
  sCommand.Address     = adr;
  sCommand.DataMode    = QSPI_DATA_NONE;
  sCommand.NbData      = 0;
  sCommand.DummyCycles = 0;

//  CmdCplt = 0;
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

//  while(CmdCplt == 0);

  HAL_Delay(50);

  return 1;
}

static int QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *qspiHandle)
{
  return 1;
}

static void ResetMemory(QSPI_HandleTypeDef *qspiHandle)
{
}
/*********************************************************************************/
//KeepInCompilation
/*********************************************************************************/

KeepInCompilation int Write(uint32_t Address, uint32_t Size, uint8_t* buffer)
{
  uint32_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;
  uint32_t   QSPI_DataNum = 0;

  Addr = Address % QSPI_PAGE_SIZE;
  count = QSPI_PAGE_SIZE - Addr;
  NumOfPage =  Size / QSPI_PAGE_SIZE;
  NumOfSingle = Size % QSPI_PAGE_SIZE;

  if (Addr == 0) /*!< Address is QSPI_PAGESIZE aligned  */
  {
    if (NumOfPage == 0) /*!< NumByteToWrite < QSPI_PAGESIZE */
    {
      QSPI_DataNum = Size;
      QSPI_WritePage(Address, QSPI_DataNum, buffer);
    }
    else /*!< Size > QSPI_PAGESIZE */
    {
      while (NumOfPage--)
      {
        QSPI_DataNum = QSPI_PAGE_SIZE;
        QSPI_WritePage(Address, QSPI_DataNum, buffer);
        Address +=  QSPI_PAGE_SIZE;
        buffer += QSPI_PAGE_SIZE;
      }

      QSPI_DataNum = NumOfSingle;
      if(QSPI_DataNum > 0)
        QSPI_WritePage(Address, QSPI_DataNum, buffer);
    }
  }
  else /*!< Address is not QSPI_PAGESIZE aligned  */
  {
    if (NumOfPage == 0) /*!< Size < QSPI_PAGESIZE */
    {
      if (NumOfSingle > count) /*!< (Size + Address) > QSPI_PAGESIZE */
      {
        temp = NumOfSingle - count;
        QSPI_DataNum = count;
        QSPI_WritePage(Address, QSPI_DataNum, buffer);
        Address +=  count;
        buffer += count;
        QSPI_DataNum = temp;
        QSPI_WritePage(Address, QSPI_DataNum, buffer);
      }
      else
      {
        QSPI_DataNum = Size;
        QSPI_WritePage(Address, QSPI_DataNum, buffer);
      }
    }
    else /*!< Size > QSPI_PAGESIZE */
    {
      Size -= count;
      NumOfPage =  Size / QSPI_PAGE_SIZE;
      NumOfSingle = Size % QSPI_PAGE_SIZE;
      QSPI_DataNum = count;
      QSPI_WritePage(Address, QSPI_DataNum, buffer);
      Address +=  count;
      buffer += count;

      while (NumOfPage--)
      {
        QSPI_DataNum = QSPI_PAGE_SIZE;
        QSPI_WritePage(Address, QSPI_DataNum, buffer);
        Address +=  QSPI_PAGE_SIZE;
        buffer += QSPI_PAGE_SIZE;
      }

      if (NumOfSingle != 0)
      {
        QSPI_DataNum = NumOfSingle;
        QSPI_WritePage(Address, QSPI_DataNum, buffer);
      }
    }
  }

  return 1;
}
KeepInCompilation int SectorErase (uint32_t EraseStartAddress ,uint32_t EraseEndAddress)
{
  uint32_t BlockAddr;
  EraseStartAddress = EraseStartAddress - (EraseStartAddress % 0x1000);

  while (EraseEndAddress >= EraseStartAddress)
  {
    BlockAddr = EraseStartAddress & 0x7FFFFF;
    if(EraseSector(BlockAddr) != 1)
    {
      return 0;
    }
    EraseStartAddress += 0x1000;
  }

  return 1;
}
uint32_t CheckSum(uint32_t StartAddress, uint32_t Size, uint32_t InitVal)
{
  uint8_t missalignementAddress = StartAddress%4;
  uint8_t missalignementSize = Size;
  int cnt;
  uint32_t Val;

  StartAddress-=StartAddress%4;
  Size += (Size%4==0)?0:4-(Size%4);

  for(cnt=0; cnt<Size ; cnt+=4)
  {
    Val = *(uint32_t*)StartAddress;
    if(missalignementAddress)
    {
      switch (missalignementAddress)
      {
        case 1:
          InitVal += (uint8_t) (Val>>8 & 0xff);
          InitVal += (uint8_t) (Val>>16 & 0xff);
          InitVal += (uint8_t) (Val>>24 & 0xff);
          missalignementAddress-=1;
          break;
        case 2:
          InitVal += (uint8_t) (Val>>16 & 0xff);
          InitVal += (uint8_t) (Val>>24 & 0xff);
          missalignementAddress-=2;
          break;
        case 3:
          InitVal += (uint8_t) (Val>>24 & 0xff);
          missalignementAddress-=3;
          break;
      }
    }
    else if((Size-missalignementSize)%4 && (Size-cnt) <=4)
    {
      switch (Size-missalignementSize)
      {
        case 1:
          InitVal += (uint8_t) Val;
          InitVal += (uint8_t) (Val>>8 & 0xff);
          InitVal += (uint8_t) (Val>>16 & 0xff);
          missalignementSize-=1;
          break;
        case 2:
          InitVal += (uint8_t) Val;
          InitVal += (uint8_t) (Val>>8 & 0xff);
          missalignementSize-=2;
          break;
        case 3:
          InitVal += (uint8_t) Val;
          missalignementSize-=3;
          break;
      }
    }
    else
    {
      InitVal += (uint8_t) Val;
      InitVal += (uint8_t) (Val>>8 & 0xff);
      InitVal += (uint8_t) (Val>>16 & 0xff);
      InitVal += (uint8_t) (Val>>24 & 0xff);
    }
    StartAddress+=4;
  }

  return (InitVal);
}
KeepInCompilation uint64_t Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size, uint32_t missalignement)
{
  uint32_t VerifiedData = 0, InitVal = 0;
  uint64_t checksum;
  Size*=4;

  if(QUADSPI_MappedMode(&hqspi)!=1)
    return 0;

  checksum = CheckSum((uint32_t)MemoryAddr + (missalignement & 0xF), Size - ((missalignement >> 16) & 0xF), InitVal);
  while (Size>VerifiedData)
  {
    if ( *(uint8_t*)MemoryAddr++ != *((uint8_t*)RAMBufferAddr + VerifiedData))
      return ((checksum<<32) + (MemoryAddr + VerifiedData));

    VerifiedData++;
  }

  return (checksum<<32);
}
#if 0
KeepInCompilation int Read (uint32_t Address, uint32_t Size, uint16_t* buffer)
{
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.AlternateBytesSize = 0;
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = 0xEB;
  sCommand.AddressMode = QSPI_ADDRESS_4_LINES;
  sCommand.Address     = Address;
  sCommand.DataMode    = QSPI_DATA_4_LINES;
  sCommand.NbData      = Size;
  sCommand.DummyCycles = 6; //no * 2

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_QSPI_Receive(&hqspi, (uint8_t *)buffer, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  return 1;
}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int Init(uint8_t configureMemoryMappedMode)
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
  MX_QUADSPI_Init();
  /* USER CODE BEGIN 2 */

  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.AlternateBytesSize = 0;
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;


  QSPI_WriteEnable(&hqspi);

  /* write configuration register SPI Quad mode write*/
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.AlternateBytesSize = 0;
  sCommand.Instruction = WRITE_STATUS_REG_CMD;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.DataMode    = QSPI_DATA_1_LINE;
  sCommand.DummyCycles = 0;
  sCommand.NbData      = 2;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  uint8_t tx[2] = {0x00, 0x0A};

  if (HAL_QSPI_Transmit(&hqspi, tx, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  /* read configuration register SPI Quad mode read */
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.AlternateBytesSize = 0;
  sCommand.Instruction = ENTER_QUAD_CMD;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.NbData      = 1;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  uint8_t reg;

  if (HAL_QSPI_Receive(&hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  if(reg != 0x0A)
  {
    Error_Handler();
  }

  /* Global Block Protection Unlock */
  QSPI_WriteEnable(&hqspi);

  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.AlternateBytesSize = 0;
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = 0x98;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.Address     = 0x000000;
  sCommand.DataMode    = QSPI_DATA_NONE;
  sCommand.NbData      = 0;
  sCommand.DummyCycles = 0;

//  CmdCplt = 0;
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

//  while(CmdCplt == 0);

  QSPI_AutoPollingMemReady(&hqspi);

  HAL_Delay(1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#if 0
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
#endif

  if(!configureMemoryMappedMode)
  {
    if(QUADSPI_MappedMode(&hqspi) !=1)
      return 0; //fail
  }

  return 1;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Command completed callbacks.
  * @param  hqspi: QSPI handle
  * @retval None
  */
void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *qspiHandle)
{
  CmdCplt++;
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  hqspi: QSPI handle
  * @retval None
  */
void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *qspiHandle)
{
  RxCplt++;
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hqspi: QSPI handle
  * @retval None
  */
void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *qspiHandle)
{
  TxCplt++;
}

/**
  * @brief  Status Match callbacks
  * @param  hqspi: QSPI handle
  * @retval None
  */
void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *qspiHandle)
{
  StatusMatch++;
}

void HAL_Delay(uint32_t Delay)
{
  __IO uint32_t i,j;

  for(j = 0; j<Delay; j++)
  {
    for(i=0; i<50000; i++)
    {
      __asm("nop");
    }
  }
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
  while(1);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
