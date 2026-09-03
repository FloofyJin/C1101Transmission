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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * ---- filled triangle, as scanline spans ----------------------------------
 *
 * NOT a 3-point outline. ScanoutEngine derives blanking from segment PARITY --
 * even segments are spans, odd are connectors -- so a 3-corner polygon gets
 * two of its three sides blanked. That is why the seeded triangle outline was
 * retired on the FPGA side. The pipeline draws scanline fill, so the triangle
 * has to arrive as fill: one span per row, two points each.
 *
 *          apex (CX, Y_TOP)
 *               /\
 *              /  \          row i:   xL(i) ......... xR(i)   all at y(i)
 *             /    \
 *            /______\        base 2*HALF_BASE wide, at Y_BOT
 *
 * TRI_ROWS sets the row pitch: TRI_H / (TRI_ROWS - 1). Keep it near the FPGA's
 * SPACING (4 coordinate units) -- the design rule is
 * SPACING ~= row pitch ~= beam spot, and a finer pitch than SPACING just
 * spends DAC time without making the fill look any more solid.
 *
 *   64 rows over 173 units -> 2.7 units/row, 128 points, 5 packets/frame
 *
 * ---- rotation ------------------------------------------------------------
 *
 * The shape is built ONCE about its own centroid and rotated into place each
 * frame. Rotating the span endpoints is legitimate here because blanking is
 * derived from segment PARITY, not from y being constant: a rotated span is
 * still segment 2i (even -> drawn) and its connector still 2i+1 (odd ->
 * blanked). The fill lines rotate with the shape, which is how a rotating
 * solid should look, and rigid rotation preserves path length so the DAC
 * budget does not change with angle.
 *
 * The size is set by the ROTATION CIRCLE, not by the screen. Every point must
 * stay inside 0..255 at EVERY angle or the coordinate wraps and draws a line
 * across the image. With the centroid at (128,128) the budget is a radius of
 * 128; B=100/H=173 puts the base corners at 115.4 and the apex at 115.3 --
 * near-equal, which is the best aspect ratio for filling a circle. Swept over
 * all angles the extremes are 12.4 and 243.6.
 */
#define TRI_ROWS       64            /* spans; 2 points each                  */
#define TRI_B          100           /* half base width, from the centroid    */
#define TRI_H          173           /* apex-to-base height                   */

#define TRI_POINTS     (2 * TRI_ROWS)   /* must be EVEN and <= CC_FRAME_POINTS */

#define ROT_STEPS      64            /* angles per revolution; power of two   */
#define ROT_CX         128           /* rotation centre                       */
#define ROT_CY         128
#define ROT_ADVANCE    1             /* angle steps per frame; negative = CW  */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* The shape about its own centroid, built once. Signed, and NOT clamped to
   8 bits -- these are offsets, not coordinates. */
static int16_t base_x[TRI_POINTS], base_y[TRI_POINTS];

/* One frame's worth of screen coordinates, rewritten each rotation step. */
static cc1101_point_t tri[TRI_POINTS];

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
 * Fill `tri` with one span per row, bottom to top.
 *
 * Rows alternate direction (SERPENTINE) so the row-to-row connector is a short
 * hop along the shape's edge instead of a jump back across the figure. Every
 * connector is blanked either way, but a short one costs far less DAC time --
 * the beam still has to physically travel it.
 *
 * Point ORDER carries the blanking, so it is not cosmetic:
 *
 *   points 2i, 2i+1   -> segment 2i   EVEN -> span      -> drawn
 *   points 2i+1, 2i+2 -> segment 2i+1 ODD  -> connector -> blanked
 *
 * TRI_POINTS is even, so the wrap segment (last point back to point 0) is odd
 * and gets blanked too. That matters: the wrap runs from the apex all the way
 * down to the bottom-left corner.
 */
static void build_triangle(void)
{
    const int span = TRI_ROWS - 1;    /* so row `span` lands exactly on the apex */

    for (int i = 0; i < TRI_ROWS; i++) {
        /* Centroid-relative: the base sits at -H/3 and the apex at +2H/3, which
           is what puts the rotation centre at the centroid rather than at the
           base. Rotating about anything else makes the triangle wobble. */
        int y    = -(TRI_H / 3) + (i * TRI_H) / span;
        int half = (TRI_B * (span - i)) / span;      /* full at base, 0 at apex */
        int xl   = -half;
        int xr   =  half;

        if (i & 1) { int t = xl; xl = xr; xr = t; }   /* odd rows run right-to-left */

        base_x[2*i    ] = (int16_t)xl;  base_y[2*i    ] = (int16_t)y;
        base_x[2*i + 1] = (int16_t)xr;  base_y[2*i + 1] = (int16_t)y;
    }
}

/*
 * Q15 sine, one revolution in ROT_STEPS steps. A table rather than sinf() so
 * this pulls in no libm and the arithmetic is exactly reproducible.
 * cos(i) = sin(i + ROT_STEPS/4).
 */
static const int16_t sin_q15[ROT_STEPS] = {
         0,   3212,   6393,   9512,  12539,  15446,  18204,  20787,
     23170,  25329,  27245,  28898,  30273,  31356,  32137,  32609,
     32767,  32609,  32137,  31356,  30273,  28898,  27245,  25329,
     23170,  20787,  18204,  15446,  12539,   9512,   6393,   3212,
         0,  -3212,  -6393,  -9512, -12539, -15446, -18204, -20787,
    -23170, -25329, -27245, -28898, -30273, -31356, -32137, -32609,
    -32767, -32609, -32137, -31356, -30273, -28898, -27245, -25329,
    -23170, -20787, -18204, -15446, -12539,  -9512,  -6393,  -3212,
};

/*
 * Rotate the base shape by `angle` steps and drop it on the screen centre.
 *
 * Point ORDER is untouched, which is the whole reason this is safe: the
 * span/connector parity that drives Z blanking survives rotation unchanged.
 *
 * Products peak at 116 * 32767, well inside int32. The clamp should never
 * fire -- the geometry is sized to the rotation circle -- but a coordinate
 * that wrapped past 255 would draw a bright line clean across the image, so
 * it is cheap insurance against a future resize that forgets the constraint.
 */
static void rotate_into(cc1101_point_t *out, uint8_t angle)
{
    const int32_t c = sin_q15[(angle + ROT_STEPS/4) & (ROT_STEPS - 1)];
    const int32_t s = sin_q15[ angle                & (ROT_STEPS - 1)];

    for (int i = 0; i < TRI_POINTS; i++) {
        int32_t bx = base_x[i], by = base_y[i];
        int32_t x = ((bx * c - by * s) >> 15) + ROT_CX;
        int32_t y = ((bx * s + by * c) >> 15) + ROT_CY;

        if (x <   0) x =   0;
        if (x > 255) x = 255;
        if (y <   0) y =   0;
        if (y > 255) y = 255;

        out[i].x = (uint8_t)x;
        out[i].y = (uint8_t)y;
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

  build_triangle();
  printf("triangle: %d rows, %d points, %d packets/frame, %d angles/rev\r\n",
         TRI_ROWS, TRI_POINTS,
         (TRI_POINTS + CC_MAX_POINTS - 1) / CC_MAX_POINTS, ROT_STEPS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /*
   * ---- M13 stage A: prove the transmit path against the EXISTING Zybo ----
   *
   * Sends { 0xAA, 0x55, seq } -- byte-identical to TxSeq.sv -- so the Zybo's
   * current unmodified bitstream receives it and lights LD3. This isolates
   * "does the STM32 transmit" from "does the new packet format parse".
   *
   * On the Zybo: turn SW1 (sw_onlySend) OFF so radio B is configured, and
   * trigger the ILA on b_rx_done. Expect b_rxbytes=0x06, b_len_byte=0x03,
   * b_b0=0xAA, b_b1=0x55, b_crc_ok=1, b_pkt_ok=1, and b_b2 incrementing.
   *
   * Once that passes, switch SEND_POINTS to 1 for stage B. RxSeq.sv will NOT
   * parse point packets until the M14 RTL work lands -- expect silence, not a
   * wrong answer.
   */

  uint32_t sent = 0, failed = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    bool ok;
    /* One rotation step per frame. send_frame splits TRI_POINTS into
       ceil(n/28) packets and marks the LAST one end-of-frame, which is what
       makes the receiver swap banks -- so the display only ever shows a
       complete angle, never half of one and half of the next.

       A dropped packet now costs a few spans that are one FRAME stale rather
       than permanently wrong: the next pass rewrites every index. That is the
       self-healing property doing real work once the image moves. */
    static uint8_t angle = 0;

    rotate_into(tri, angle);
    angle = (uint8_t)((angle + ROT_ADVANCE) & (ROT_STEPS - 1));

    ok = cc1101_send_frame(tri, TRI_POINTS);

    if (ok) sent++; else failed++;

    /*
     * Report once a second, never from inside the transmit path.
     * __io_putchar blocks on the UART, so a printf per packet would stall the
     * CPU for milliseconds at a time and throttle the send rate.
     */
    static uint32_t t_report = 0;
    if (HAL_GetTick() - t_report >= 1000) {
        t_report = HAL_GetTick();
        const cc1101_tx_diag_t *d = cc1101_last_diag();
        /* frames, not packets -- each pass is 5 transmits here, and the diag
           describes only the last of them. */
        printf("frames=%lu failed=%lu | txbytes=0x%02X marc=0x%02X "
               "sync=%d done=%d drained=%d timeout=%d\r\n",
               sent, failed, d->txbytes, d->marcstate,
               d->sync_seen, d->sent_ok, d->drained, d->timed_out);
    }

    /*
     * Between FRAMES, not between packets -- send_frame already sent all five
     * back to back. This is now the ANIMATION rate, so it is no longer free:
     * every millisecond here is a millisecond of rotation.
     *
     * At the config table's 38.4 kbps a 59-byte packet is ~14 ms of airtime,
     * so a frame is already ~75 ms and a revolution takes ROT_STEPS * (75 +
     * this). At 0 that is ~5 s per turn. Raise it to slow the spin down;
     * the DISPLAY refresh is unaffected either way -- the FPGA redraws
     * PointRam continuously no matter how often the contents change, which is
     * the entire point of decoupling the two rates.
     */
    HAL_Delay(0);
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
  huart2.Init.BaudRate = 115200;
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
