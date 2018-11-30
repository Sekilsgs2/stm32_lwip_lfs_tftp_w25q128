/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "w25qxx.h"
#include "lfs.h"
#include "lwip/apps/httpd.h"
#include "lwip/apps/tftp_server.h"
#include "lwip/apps/fs.h"
#include "fsdata.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

int block_device_read(const struct lfs_config *cfg, lfs_block_t block,
        lfs_off_t off, void *buffer, lfs_size_t size)
{
	W25qxx_ReadBytes((uint8_t*)buffer, (block * cfg->block_size) + off, size);
	return 0;
}

int block_device_prog(const struct lfs_config *c, lfs_block_t block,
	lfs_off_t off, const void *buffer, lfs_size_t size)
{
	W25qxx_WriteNoCheck((uint8_t*)buffer, (block * c->block_size + off), size);
	
	return 0;
}

int block_device_erase(const struct lfs_config *c, lfs_block_t block)
{
	W25qxx_EraseSector(block * c->block_size);
	
	return 0;
}

int block_device_sync(const struct lfs_config *c)
{
	return 0;
}

lfs_t lfs;
lfs_file_t file;
struct lfs_config cfg;
struct lfs_info info;

uint8_t lfs_read_buf[2048] = {0};
uint8_t lfs_prog_buf[2048]= {0};
uint8_t lfs_lookahead_buf[2048]= {0};	// 128/8=16
uint8_t lfs_file_buf[2048]= {0};

void LFS_Config(void)
{
	// block device operations
	cfg.read  = &block_device_read;
	cfg.prog  = &block_device_prog;
	cfg.erase = &block_device_erase;
	cfg.sync  = &block_device_sync;

	// block device configuration
	cfg.read_size = 2048;
	cfg.prog_size = 2048;
	cfg.block_size = 4096;
	cfg.block_count = 4096;
	cfg.lookahead = 1024;
	
	cfg.read_buffer = lfs_read_buf;
	cfg.prog_buffer = lfs_prog_buf;
	cfg.lookahead_buffer = lfs_lookahead_buf;
	cfg.file_buffer = lfs_file_buf;
}

struct tftp_context tftpctx;

void* tftp_open(const char* fname, const char* mode, u8_t write)
{
	lfs_file_t *file = malloc(sizeof(lfs_file_t));
	memset(file,0,sizeof(lfs_file_t));
	
	printf("tftp open file name = %s\n\r",fname);
	
	int resopen = lfs_file_open(&lfs, file, fname, write==0 ? LFS_O_RDONLY : LFS_O_RDWR|LFS_O_CREAT);
	printf("tftp open file resopn = %d\n\r",resopen);
	if (resopen >= 0) return (void*)file;
	else return NULL;
}

void tftp_close(void* handle)
{
	lfs_file_t *file = (lfs_file_t*)handle;
  lfs_file_close(&lfs, file);
	free(file);

}

int tftp_read(void* handle, void* buf, int bytes)
{
	lfs_file_t *file = (lfs_file_t*)handle;
	
  if(file->pos == file->size) {
    return FS_READ_EOF;
  }
	else
	{
		return lfs_file_read(&lfs, file, buf, bytes);
	}
}

int tftp_write(void* handle, struct pbuf* p)
{
	lfs_file_t *file = (lfs_file_t*)handle;
	return lfs_file_write(&lfs, file, p->payload, p->len);
}



void TFTP_Config(void)
{
	tftpctx.open = tftp_open;
	tftpctx.close = tftp_close;
	tftpctx.read = tftp_read;
	tftpctx.write = tftp_write;

}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */
	
		uint8_t pbuf[256];
	
	//HAL_Delay(4000);
	
	uint32_t	id;
	
	while(HAL_GetTick()<100)
		HAL_Delay(1);
	
	id = W25qxx_ReadID();
	
	W25qxx_Init();
	
	//W25qxx_ReadPage(pbuf,0,0,0);
	
	
	W25qxx_ReadPage(pbuf,0,0,0);
	
	
  printf("\n\r welcome to www.waveshere.com !!!\n\r");
	printf("\n\r 1 FALSH ID =  %x", id);
	printf("\n\r 2 FALSH ID =  %x", W25qxx_ReadID());
	printf("\n\r 3 FALSH ID =  %x", W25qxx_ReadID());
	
	
	printf("\n\r From flash init id = %x \n\r", w25qxx.ID);
	printf("\n\r Read byte from flash = %x \n\r", pbuf[56]);
	
	LFS_Config();
	
	// mount the filesystem
    int err = lfs_mount(&lfs, &cfg);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
			  printf("Error - try format spi flash \n\r");
        lfs_format(&lfs, &cfg);
			  printf("Format ended - try mount flash \n\r"); 
        err = lfs_mount(&lfs, &cfg);
			  if (err) {
					printf("Mount error after format - go loop \n\r");
				}
				printf("Format ended - mount good \n\r");
			  
    }
		
		// read current count
    uint32_t boot_count = 0;
    lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));

    // update boot count
		
		//char *bufa;
    boot_count += 1;
    lfs_file_rewind(&lfs, &file);
    lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));

    // remember the storage is not updated until the file is closed successfully
    lfs_file_close(&lfs, &file);
		
    // release any resources we were using
		
    // print the boot count
    printf("boot_count: %d\n\r", boot_count);
		
		httpd_init();
		
		TFTP_Config();
		
		tftp_init(&tftpctx);
		

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		MX_LWIP_Process();
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
  /**Configure the Systick interrupt time 
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
