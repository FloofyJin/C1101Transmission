/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cc1101.h"
#include "animation.h"
#include <stdio.h>

/*
 * ===================================================================
 * WHERE THE FRAMES COME FROM -- set exactly ONE of these to 1.
 * ===================================================================
 *
 * All three feed animation.c, so the cadence is identical in every mode: one
 * frame per period, wall-clock, regardless of how many spans it holds. Only
 * the source of the points differs.
 *
 *   ANIM_FROM_UART       Streamed from the PC by tools/animstream.py.
 *                        Unlimited clip length -- this is the only mode that
 *                        can play all 4381 frames of Bad Apple. Needs a PC
 *                        attached and ~2.5 KB of the 32 KB RAM.
 *
 *   COMPILED_ANIMATION   Baked into flash by tools/anim2c.py. Runs standalone
 *                        with no PC and costs no RAM, but 128 KB of flash is
 *                        only ~300 frames (15 s at 20 fps) before .rodata
 *                        overflows the region.
 *                        REQUIRES animation_data.c/.h, which are generated:
 *                            python tools/anim2c.py <clip>.json --fps 20 \
 *                                                   --kbps 250 --preamble 16
 *                        They are deleted from the tree right now, so this
 *                        mode will not link until you regenerate them.
 *
 *   TRIANGLE_ANIMATION   A filled triangle rotating on the spot, generated on
 *                        the board. No PC, no JSON, no flash tables -- the
 *                        bring-up pattern. When the scope is dark, this is the
 *                        mode that says whether the fault is upstream of the
 *                        transmitter at all.
 */
#define ANIM_FROM_UART      1
#define COMPILED_ANIMATION  0
#define TRIANGLE_ANIMATION  0

#define ANIMATION_FPS       20

/*
 * Catch a mis-set mode at COMPILE time. The #if/#elif chains below would
 * otherwise just silently pick the first one that is true, or fall through to
 * nothing and build a firmware that transmits an uninitialised buffer -- which
 * on a scope looks like a wiring fault, not a configuration mistake.
 */
#if (ANIM_FROM_UART + COMPILED_ANIMATION + TRIANGLE_ANIMATION) != 1
#error "Set exactly one of ANIM_FROM_UART, COMPILED_ANIMATION, TRIANGLE_ANIMATION"
#endif

#if ANIM_FROM_UART
#include "anim_stream.h"
#elif COMPILED_ANIMATION
#include "animation_data.h"
#else
#include "triangle.h"
#endif
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Playback state. One per clip; the mode block below binds it to a source. */
static animation_t anim;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {            /* retarget printf to the ST-Link VCP */
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/*
 * Bind the playback engine to whichever source the mode selects, and say on
 * the console which one it is -- the single most useful line in the log, since
 * "nothing is moving" means something different in each mode.
 */
static void animation_setup(void)
{
#if ANIM_FROM_UART
    /* Arm the DMA receiver BEFORE announcing anything, so the hello message
       and the first window update leave with the receiver already listening. */
    anim_stream_init(&huart2);
    animation_init_source(&anim, anim_stream_next, anim_stream_release,
                          NULL, ANIMATION_FPS);
    printf("animation: UART stream @ %d fps (%lu ms/frame), "
           "%d slots x %d points\r\n",
           ANIMATION_FPS, (unsigned long)anim.period_ms,
           ANIM_STREAM_SLOTS, ANIM_STREAM_MAX_POINTS);
    printf("  run: python tools/animstream.py <clip>.json --baud %lu\r\n",
           (unsigned long)huart2.Init.BaudRate);

#elif COMPILED_ANIMATION
    animation_init(&anim, &animation_clip, ANIMATION_FPS);
    printf("animation: %u frames from flash @ %d fps (%lu ms/frame)\r\n",
           (unsigned)animation_clip.n_frames, ANIMATION_FPS,
           (unsigned long)anim.period_ms);

#else /* TRIANGLE_ANIMATION */
    triangle_init();
    animation_init_source(&anim, triangle_next, triangle_done,
                          NULL, ANIMATION_FPS);
    printf("animation: rotating triangle @ %d fps (%lu ms/frame)\r\n",
           ANIMATION_FPS, (unsigned long)anim.period_ms);
    printf("  %d rows, %d points, %d packets/frame, %d angles/rev "
           "-> %.1f s per turn\r\n",
           TRI_ROWS, TRI_POINTS,
           (TRI_POINTS + CC_MAX_POINTS - 1) / CC_MAX_POINTS, ROT_STEPS,
           (double)ROT_STEPS / ROT_ADVANCE / ANIMATION_FPS);
#endif
}

/*
 * Work that must happen every pass, not once per frame. Empty in the modes
 * that generate their own points.
 */
static inline void animation_service(void)
{
#if ANIM_FROM_UART
    /* Drain the DMA ring into decoded frames. Must run far more often than
       once per frame period: nothing polls while cc1101_send_frame() is
       blocking, so this call is the only thing keeping the 1 KB ring from
       lapping itself. It is cheap and bounded -- see anim_stream.c. */
    anim_stream_poll();
#endif
}

/* Per-mode detail for the once-a-second report. The common counters are
   printed by the caller. */
static void animation_report(void)
{
#if ANIM_FROM_UART
    /*
     * Two different "the animation stutters" causes, and this line tells them
     * apart:
     *   late    -> the RADIO cannot deliver a frame in one period
     *   starved -> the UART/PC cannot deliver a frame in one period
     * They need opposite fixes, so never guess between them.
     */
    const anim_stream_stats_t *s = anim_stream_get_stats();
    printf("  anim: late=%lu starved=%lu | rx: frames=%lu depth=%u "
           "crc=%lu len=%lu ovf=%lu resync=%lu bytes=%lu\r\n",
           (unsigned long)anim.frames_late,
           (unsigned long)anim.frames_starved,
           (unsigned long)s->frames_ok, (unsigned)s->depth,
           (unsigned long)s->crc_errors, (unsigned long)s->len_errors,
           (unsigned long)s->overflows, (unsigned long)s->resyncs,
           (unsigned long)s->bytes);
    /*
     * bytes==0 means the DMA never moved anything, which is a different fault
     * from "frames are arriving but failing". Three suspects, in the order
     * they are worth checking:
     *   1. animstream.py is not running, or is on the wrong COM port
     *   2. --baud does not match huart2.Init.BaudRate
     *   3. USART2_RX is not on DMA1 stream 5 channel 4 on this part
     *      (RM0401 table 27) -- the one thing in anim_stream_init() that could
     *      not be checked from the HAL headers
     */
    if (s->bytes == 0)
        printf("  !! no UART bytes at all -- check animstream.py is running, "
               "--baud matches, and DMA1_S5C4 is USART2_RX\r\n");

#elif COMPILED_ANIMATION
    printf("  anim: frame %u/%u  loops=%lu late=%lu\r\n",
           (unsigned)anim.index, (unsigned)animation_clip.n_frames,
           (unsigned long)anim.loops, (unsigned long)anim.frames_late);

#else /* TRIANGLE_ANIMATION */
    printf("  anim: angle %u/%d  late=%lu\r\n",
           (unsigned)triangle_angle(), ROT_STEPS,
           (unsigned long)anim.frames_late);
#endif
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  printf("\r\n=== PacketSender: CC1101 transmit ===\r\n");

  bool cfg_ok = cc1101_init(&hspi1);

  /* MUST come after cc1101_init -- that call is what hands the driver the SPI
     handle. Read the chip before it and every transfer is a no-op against a
     NULL handle, which reads back as a very convincing 00/00. */
  uint8_t partnum, version;
  cc1101_selftest(&partnum, &version);

  printf("PARTNUM=0x%02X VERSION=0x%02X  config=%s\r\n",
         partnum, version, cfg_ok ? "OK" : "FAILED");

  /* PARTNUM = 0x00 is the CORRECT value for a CC1101 -- it is not a failure.
     VERSION is the liveness check: non-zero with mixed 1s and 0s proves MISO
     can drive both rails. 0x14 is typical but varies by silicon batch, so do
     not hard-fail on a specific value. Both reading 0x00 means no SPI traffic
     reached the chip at all. */
  if (version == 0x00)
      printf("  !! VERSION=0 -- chip not responding. Check wiring/CSn/power.\r\n");

  /* PARTNUM is 0x00 and VERSION is not, so equal values mean the reads are not
     returning register data at all -- a broken MISO reads as plausible-looking
     garbage in every register otherwise. */
  if (partnum == version)
      printf("  !! PARTNUM==VERSION==0x%02X -- read path broken, not the radio.\r\n",
             partnum);

  /* Paced playback. The rate is wall-clock, not data-driven: every frame
     occupies the same period however many spans it holds, so playback speed
     no longer tracks picture complexity. See animation.h. */
  animation_setup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /*
   * The loop is the same in all three modes: service the source, offer it to
   * the pacer, report once a second. Everything mode-specific was resolved at
   * compile time in the helpers above.
   *
   * If nothing appears on the scope, narrow it with the mode switch rather
   * than by probing: TRIANGLE_ANIMATION removes the PC, the JSON and the
   * inbound UART path from the picture entirely, so if the triangle draws and
   * a streamed clip does not, the radio and the FPGA are both fine.
   */

  uint32_t sent = 0, failed = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Mode-specific upkeep -- draining the UART ring, in the streaming mode.
       Nothing in the other two. */
    animation_service();

    /*
     * Non-blocking: returns immediately until the next frame is due, then
     * transmits exactly one frame. The wait is what fixes the cadence -- see
     * the pacing discussion in animation.h.
     *
     * send_frame splits the frame into ceil(n/28) packets and marks the LAST
     * one end-of-frame, which is what makes the receiver swap banks -- so the
     * display only ever shows a complete frame, never half of one and half of
     * the next. A dropped packet costs a few spans that are one frame stale
     * rather than permanently wrong: the next pass rewrites every index.
     */
    (void)animation_tick(&anim);
    sent   = anim.frames_sent;
    failed = anim.frames_failed;

    /*
     * Report once a second, never from inside the transmit path.
     * __io_putchar blocks on the UART, so a printf per packet would stall the
     * CPU for milliseconds at a time and throttle the send rate.
     */
    static uint32_t t_report = 0;
    if (HAL_GetTick() - t_report >= 1000) {
        t_report = HAL_GetTick();
        const cc1101_tx_diag_t *d = cc1101_last_diag();
        /* frames, not packets -- each pass is several transmits here, and
           the diag describes only the last of them. */
        printf("frames=%lu failed=%lu | txbytes=0x%02X marc=0x%02X "
               "sync=%d done=%d drained=%d timeout=%d\r\n",
               sent, failed, d->txbytes, d->marcstate,
               d->sync_seen, d->sent_ok, d->drained, d->timed_out);

        /* late climbing steadily means the clip is too dense for the frame
           rate: the radio cannot deliver a frame inside its period. Lower
           ANIMATION_FPS, raise the link rate, or simplify the frames.
           tools/anim2c.py predicts this at conversion time. */
        animation_report();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  /* HSI with no PLL -> PCLK2 = 16 MHz. Prescaler 2 would give 8 MHz, which is
     over the CC1101's 6.5 MHz BURST limit (single access allows 9 MHz). The TX
     FIFO write is a 63-byte burst -- far longer than anything the RFreceiver
     ever clocked -- so take the /4 and run at 4 MHz. SPI was never the
     bottleneck here anyway; airtime is. */
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 15;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  /* 230400, not 115200: a streamed frame is 180 points x 2 B + 8 B of
     framing = 368 B, and 20 fps of that is 7360 B/s. 115200 delivers
     11520 B/s, so the link would sit at 64% utilisation with no room
     for the retransmit-free protocol to absorb a stall. Mirrored in
     PacketSender.ioc so CubeMX regeneration keeps it. */
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
