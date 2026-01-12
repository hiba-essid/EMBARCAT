/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Utilisation d'un buffer circulaire pour éviter les valeurs intermédiaires
#define ADC_BUFFER_SIZE 4
volatile uint16_t adc_buffer[ADC_BUFFER_SIZE] = {0};
volatile uint8_t adc_index = 0;
float voltage = 0.0f;
char msg[64];  // Buffer global réutilisable

// Définition des seuils de tension pour le comportement LED
#define VOLTAGE_THRESHOLD_LOW   1.0f    // Tension basse
#define VOLTAGE_THRESHOLD_MID   2.0f    // Tension moyenne
#define VOLTAGE_THRESHOLD_HIGH  2.5f    // Tension élevée

// Variables pour le contrôle de la LED
uint32_t last_led_toggle = 0;
uint32_t led_blink_interval = 1000;  // Par défaut 1 seconde
uint8_t led_state = 0;

// Configuration des intervalles de temps (en ms)
#define DISPLAY_INTERVAL_MS     2000    // 2 secondes entre les affichages
#define LED_BLINK_SLOW_MS       1000    // 1 seconde - clignotement lent
#define LED_BLINK_MEDIUM_MS     500     // 500 ms - clignotement moyen
#define LED_BLINK_FAST_MS       250     // 250 ms - clignotement rapide
#define LED_BLINK_VERY_FAST_MS  100     // 100 ms - clignotement très rapide
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void UpdateLEDBehavior(float voltage);  // Nouvelle fonction

/* USER CODE BEGIN 0 */
// Fonction pour calculer la moyenne des échantillons ADC
static uint16_t ADC_GetAverageValue(void)
{
    uint32_t sum = 0;
    for(int i = 0; i < ADC_BUFFER_SIZE; i++) {
        sum += adc_buffer[i];
    }
    return (uint16_t)(sum / ADC_BUFFER_SIZE);
}

// Fonction pour mettre à jour le comportement de la LED selon la tension
static void UpdateLEDBehavior(float voltage)
{
    if (voltage < VOLTAGE_THRESHOLD_LOW) {
        // Tension très basse : LED éteinte
        led_blink_interval = 0;  // Éteinte
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    }
    else if (voltage < 1.5f) {
        // Tension basse : clignotement très lent (2s)
        led_blink_interval = LED_BLINK_SLOW_MS * 2;  // 2 secondes
    }
    else if (voltage < VOLTAGE_THRESHOLD_MID) {
        // Tension basse-moyenne : clignotement lent (1s)
        led_blink_interval = LED_BLINK_SLOW_MS;  // 1 seconde
    }
    else if (voltage < 2.25f) {
        // Tension moyenne : clignotement moyen (500ms)
        led_blink_interval = LED_BLINK_MEDIUM_MS;  // 500 ms
    }
    else if (voltage < VOLTAGE_THRESHOLD_HIGH) {
        // Tension moyenne-élevée : clignotement rapide (250ms)
        led_blink_interval = LED_BLINK_FAST_MS;  // 250 ms
    }
    else {
        // Tension élevée : clignotement très rapide (100ms)
        led_blink_interval = LED_BLINK_VERY_FAST_MS;  // 100 ms
    }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  /* Initialisation des périphériques */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();

  /* Message de démarrage */
  const char* startup_msg =
      "\r\nADC Demo - PA0\r\n"
      "3.3V ref | 12-bit\r\n"
      "Display: every 2s | LED: speed = voltage\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)startup_msg, strlen(startup_msg), HAL_MAX_DELAY);

  /* Démarrage de l'ADC en mode DMA circulaire avec buffer */
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Boucle principale */
  uint32_t last_display_tick = 0;
  uint32_t last_adc_sample_tick = 0;
  const uint32_t adc_sample_interval = 100;  // Échantillonnage ADC toutes les 100ms

  while (1)
  {
    uint32_t current_tick = HAL_GetTick();

    /* Acquisition ADC à intervalle régulier */
    if ((current_tick - last_adc_sample_tick) >= adc_sample_interval)
    {
      last_adc_sample_tick = current_tick;

      /* Calcul de la moyenne pour lisser les mesures */
      uint16_t avg_adc_value = ADC_GetAverageValue();

      /* Calcul de la tension (ADC 12 bits) */
      voltage = (avg_adc_value * 3.3f) / 4095.0f;

      /* Mise à jour du comportement de la LED selon la tension */
      UpdateLEDBehavior(voltage);
    }

    /* Affichage des valeurs toutes les 2 secondes */
    if ((current_tick - last_display_tick) >= DISPLAY_INTERVAL_MS)
    {
      last_display_tick = current_tick;

      /* Pré-calcul pour éviter les calculs redondants */
      float percentage = (ADC_GetAverageValue() * 100.0f) / 4095.0f;

      /* Affichage UART optimisé */
      int len = snprintf(msg, sizeof(msg),
                        "ADC: %4u | %.3f V | %.1f%% | LED: %lums\r\n",
                        ADC_GetAverageValue(), voltage, percentage, led_blink_interval);

      if (len > 0 && len < sizeof(msg)) {
          HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
      }
    }

    /* Contrôle de la LED selon l'intervalle défini */
    if (led_blink_interval > 0) {
        if ((current_tick - last_led_toggle) >= led_blink_interval) {
            last_led_toggle = current_tick;
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        }
    } else {
        // LED éteinte (led_blink_interval = 0)
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    }

    /* Mode basse consommation */
    __WFI();
  }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
}

/* USART2 init function - optimisée pour une transmission fiable */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;  // Réduit pour économiser de l'énergie

  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

/* DMA init function */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  hdma_adc1.Instance = DMA2_Stream0;
  hdma_adc1.Init.Channel = DMA_CHANNEL_0;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;  // Changé pour incrémenter l'adresse mémoire
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
  hdma_adc1.Init.Priority = DMA_PRIORITY_MEDIUM;  // Priorité augmentée
  hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;  // FIFO activé pour stabilité
  hdma_adc1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;

  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/* GPIO init function */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable GPIO clocks - optimisé */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Configure LD2 pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

/* Error handler optimisé */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(200);  // Ralenti pour économie d'énergie
  }
}

/* Callback de conversion ADC complète */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1) {
    adc_index = (adc_index + 1) % ADC_BUFFER_SIZE;
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
