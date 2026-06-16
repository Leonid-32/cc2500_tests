/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CC2500 Full TX/RX Driver
  *                   Pins: CS=PB11, SCK=PA1, MISO=PA6, MOSI=PA7
  *                         GDO1=PB1, GDO2=PB2, LED=PA4
  *                   Debug: SWO ITM + USART1 @ 115200
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>



/* LED = PA4 */
#define LED_PORT    GPIOA
#define LED_PIN     GPIO_PIN_4
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CC2500_READ   0x80  /* bit7=1 → read  */
#define CC2500_BURST  0x40  /* bit6=1 → burst */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ═══════════════════════════════════════════════════════
 * GPIO MAPPING (CORRECTED)
 * ═══════════════════════════════════════════════════════ */

#define CC2500_CS_PORT    GPIOB
#define CC2500_CS_PIN     GPIO_PIN_11
#define CC2500_CS_LOW()   HAL_GPIO_WritePin(CC2500_CS_PORT, CC2500_CS_PIN, GPIO_PIN_RESET)
#define CC2500_CS_HIGH()  HAL_GPIO_WritePin(CC2500_CS_PORT, CC2500_CS_PIN, GPIO_PIN_SET)

/* GDO pins (ONLY THESE ARE REAL SIGNALS) */
#define GDO0_PORT   GPIOB   // PB1
#define GDO0_PIN    GPIO_PIN_1
#define GDO2_PORT   GPIOB   // PB2
#define GDO2_PIN    GPIO_PIN_2

#define GDO0_READ() HAL_GPIO_ReadPin(GDO0_PORT, GDO0_PIN)
#define GDO2_READ() HAL_GPIO_ReadPin(GDO2_PORT, GDO2_PIN)

#define LED_PORT    GPIOA
#define LED_PIN     GPIO_PIN_4

/* ═══════════════════════════════════════════════════════
 * CC2500 SPI BITS
 * ═══════════════════════════════════════════════════════ */

#define CC2500_READ   0x80
#define CC2500_BURST  0x40

/* ═══════════════════════════════════════════════════════
 * REGISTERS
 * ═══════════════════════════════════════════════════════ */

#define REG_PARTNUM   0x30
#define REG_VERSION   0x31
#define REG_LQI       0x33
#define REG_RSSI      0x34
#define REG_MARCSTATE 0x35
#define REG_TXBYTES   0x3A
#define REG_RXBYTES   0x3B

#define REG_IOCFG0    0x02
#define REG_IOCFG1    0x01
#define REG_IOCFG2    0x00   // FIXED (was missing)

#define REG_PKTLEN    0x06
#define REG_PKTCTRL0  0x08
#define REG_PKTCTRL1  0x07
#define REG_CHANNR    0x0A

#define REG_FREQ2     0x0D
#define REG_FREQ1     0x0E
#define REG_FREQ0     0x0F

#define REG_MDMCFG4   0x10
#define REG_MDMCFG3   0x11
#define REG_MDMCFG2   0x12
#define REG_MDMCFG1   0x13
#define REG_DEVIATN   0x15

#define REG_MCSM1     0x17
#define REG_MCSM0     0x18

#define REG_FOCCFG    0x19
#define REG_AGCCTRL2  0x1B

#define REG_FREND0    0x22
#define REG_FSCAL1    0x25
#define REG_FSCAL0    0x26

#define REG_PATABLE   0x3E
#define REG_TXFIFO    0x3F
#define REG_RXFIFO    0x3F

/* ═══════════════════════════════════════════════════════
 * STROBES
 * ═══════════════════════════════════════════════════════ */

#define STROBE_SRES   0x30
#define STROBE_SFSTXON 0x31
#define STROBE_SXOFF  0x32
#define STROBE_SCAL   0x33
#define STROBE_SRX    0x34
#define STROBE_STX    0x35
#define STROBE_SIDLE  0x36
#define STROBE_SFRX   0x3A
#define STROBE_SFTX   0x3B
/* ── MARCSTATE values ───────────────────────────────────────────────────────── */
#define MARCSTATE_IDLE 0x01
#define MARCSTATE_RX   0x0D
#define MARCSTATE_TX   0x13

/* ── Crystal and frequency ──────────────────────────────────────────────────── */
#define FXOSC        26000000.0    /* 26 MHz */
#define CENTRE_FREQ  2433000000.0  /* 2.433 GHz */

/* ── RX buffer size ─────────────────────────────────────────────────────────── */
#define RX_BUF_SIZE  64

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* Route printf → SWO ITM port 0 AND USART1 simultaneously */
int __io_putchar(int ch)
{
    ITM_SendChar(ch);
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
    return ch;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Low-level SPI */
void    CC2500_WriteReg(uint8_t addr, uint8_t value);
uint8_t CC2500_ReadReg(uint8_t addr);
uint8_t CC2500_Strobe(uint8_t cmd);
void    CC2500_WriteBurst(uint8_t addr, const uint8_t *data, uint8_t len);
void    CC2500_ReadBurst(uint8_t addr, uint8_t *data, uint8_t len);
/* Mid-level */
void    CC2500_Reset(void);
uint8_t CC2500_WaitIdle(uint32_t timeout_ms);
/* High-level */
void    CC2500_RFConfig(void);
void    CC2500_SetFrequency(double frequency);
double  CC2500_GetFrequency(void);
void    CC2500_TransmitPacket(const uint8_t *data, uint8_t length);
void    CC2500_ReceiveLoop(void);
/* Test/bring-up */
void    CC2500_Test(void);
void    CC2500_TxLoopbackTest(void);

static const char *marcstate_str(uint8_t s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ════════════════════════════════════════════════════════════════════════════
 *  LOW-LEVEL SPI
 * ════════════════════════════════════════════════════════════════════════════ */

void CC2500_WriteReg(uint8_t addr, uint8_t val)
{
    uint8_t buf[2] = { addr & 0x3F, val };
    CC2500_CS_LOW();
    HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
    CC2500_CS_HIGH();
}
uint8_t CC2500_ReadReg(uint8_t addr)
{
    uint8_t tx = (addr >= 0x30) ? (addr | CC2500_READ | CC2500_BURST)
                                : (addr | CC2500_READ);
    uint8_t rx = 0;

    CC2500_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &rx, 1, HAL_MAX_DELAY);
    CC2500_CS_HIGH();

    return rx;
}

uint8_t CC2500_Strobe(uint8_t cmd)
{
    uint8_t status = 0;
    CC2500_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, &cmd, &status, 1, HAL_MAX_DELAY);
    CC2500_CS_HIGH();
    return status;
}

void CC2500_WriteBurst(uint8_t addr, const uint8_t *data, uint8_t len)
{
    uint8_t hdr = (addr & 0x3F) | CC2500_BURST;
    CC2500_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &hdr, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)data, len, HAL_MAX_DELAY);
    CC2500_CS_HIGH();
}

void CC2500_ReadBurst(uint8_t addr, uint8_t *data, uint8_t len)
{
    uint8_t hdr = addr | CC2500_READ | CC2500_BURST;
    uint8_t dummy[64];
    memset(dummy, 0xFF, len);

    CC2500_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &hdr, 1, HAL_MAX_DELAY);
    HAL_SPI_TransmitReceive(&hspi1, dummy, data, len, HAL_MAX_DELAY);
    CC2500_CS_HIGH();
}

/* ════════════════════════════════════════════════════════════════════════════
 *  MID-LEVEL HELPERS
 * ════════════════════════════════════════════════════════════════════════════ */

void CC2500_Reset(void)
{
    CC2500_CS_HIGH(); HAL_Delay(2);
    CC2500_CS_LOW();  HAL_Delay(2);
    CC2500_CS_HIGH(); HAL_Delay(2);

    CC2500_Strobe(STROBE_SRES);
    HAL_Delay(10);
}

uint8_t CC2500_WaitIdle(uint32_t timeout_ms)
{
    CC2500_Strobe(STROBE_SIDLE);
    uint32_t deadline = HAL_GetTick() + timeout_ms;
    while ((CC2500_ReadReg(REG_MARCSTATE) & 0x1F) != MARCSTATE_IDLE) {
        if (HAL_GetTick() > deadline) return 0;
    }
    return 1;
}

static const char *marcstate_str(uint8_t s)
{
    switch (s) {
        case 0x00: return "SLEEP";
        case 0x01: return "IDLE";
        case 0x0D: return "RX";
        case 0x13: return "TX";
        case 0x15: return "TX_UNDERFLOW";
        case 0x17: return "RX_OVERFLOW";
        default:   return "UNKNOWN";
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 *  RF CONFIGURATION  –  2.433 GHz, 250 kbps GFSK
 * ════════════════════════════════════════════════════════════════════════════ */

void CC2500_RFConfig(void)
{
    CC2500_WriteReg(REG_FREQ2, 0x5D);
    CC2500_WriteReg(REG_FREQ1, 0x93);
    CC2500_WriteReg(REG_FREQ0, 0xB1);

    CC2500_WriteReg(REG_MDMCFG4, 0x2D);
    CC2500_WriteReg(REG_MDMCFG3, 0x3B);
    CC2500_WriteReg(REG_MDMCFG2, 0x73);
    CC2500_WriteReg(REG_MDMCFG1, 0x22);
    CC2500_WriteReg(REG_DEVIATN, 0x34);

    CC2500_WriteReg(REG_PKTCTRL0, 0x05);
    CC2500_WriteReg(REG_PKTCTRL1, 0x04);
    CC2500_WriteReg(REG_PKTLEN, 60);

    uint8_t pa[8] = {0x00,0xFE,0,0,0,0,0,0};
    CC2500_WriteBurst(REG_PATABLE, pa, 8);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  FREQUENCY CONTROL
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * Set carrier frequency in Hz.
 * Valid range: 2400e6 – 2483.5e6 Hz.
 * Chip is forced to IDLE before writing registers.
 */
void CC2500_SetFrequency(double frequency)
{
    if (frequency < 2400e6 || frequency > 2483.5e6) {
        printf("Frequency %.0f Hz out of 2.4GHz band range\r\n", frequency);
        return;
    }
    if (!CC2500_WaitIdle(100)) {
        printf("SetFrequency: failed to reach IDLE\r\n");
        return;
    }
    uint32_t word = (uint32_t)round(frequency * (65536.0 / FXOSC));
    CC2500_WriteReg(REG_FREQ2, (uint8_t)((word >> 16) & 0xFF));
    CC2500_WriteReg(REG_FREQ1, (uint8_t)((word >>  8) & 0xFF));
    CC2500_WriteReg(REG_FREQ0, (uint8_t)((word      ) & 0xFF));
    printf("Frequency set to %.3f MHz\r\n", frequency / 1e6);
}

/**
 * Read back the configured carrier frequency in Hz.
 */
double CC2500_GetFrequency(void)
{
    uint32_t word = ((uint32_t)CC2500_ReadReg(REG_FREQ2) << 16)
                  | ((uint32_t)CC2500_ReadReg(REG_FREQ1) <<  8)
                  |  (uint32_t)CC2500_ReadReg(REG_FREQ0);
    return (FXOSC * (double)word) / 65536.0;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  TRANSMIT
 *
 *  Packet format (variable length mode, PKTCTRL0=0x05):
 *    data[0]        = length byte  (number of payload bytes that follow)
 *    data[1..len-1] = payload bytes
 *
 *  GDO1 (PB1 = CC2500_STATUS1) is wired to GDO0 of CC2500.
 *  With IOCFG1=0x06 it asserts when sync is sent, deasserts at EOP.
 *  MCSM1=0x3C returns chip to IDLE automatically after TX.
 * ════════════════════════════════════════════════════════════════════════════ */

void CC2500_TransmitPacket(const uint8_t *data, uint8_t len)
{
    CC2500_Strobe(STROBE_SIDLE);
    CC2500_Strobe(STROBE_SFTX);

    CC2500_WriteBurst(REG_TXFIFO, data, len);
    CC2500_Strobe(STROBE_STX);

    HAL_Delay(10);

    printf("TX OK\r\n");
}
/* ════════════════════════════════════════════════════════════════════════════
 *  RECEIVE LOOP  (blocking)
 *
 *  Received packet layout in RX FIFO (PKTCTRL1=0x04, append status on):
 *    buf[0]             = length byte
 *    buf[1 .. len]      = payload
 *    buf[len+1]         = RSSI raw
 *    buf[len+2]         = LQI[6:0] | CRC_OK[7]
 *
 *  Call from main() when you want this board to be the receiver.
 *  To use as transmitter instead, comment this out and use
 *  CC2500_TransmitPacket() in a loop.
 * ════════════════════════════════════════════════════════════════════════════ */

void CC2500_ReceiveLoop(void)
{
    printf("\r\nRX MODE START\r\n");

    while (1)
    {
        CC2500_Strobe(STROBE_SIDLE);
        CC2500_Strobe(STROBE_SFRX);
        CC2500_Strobe(STROBE_SRX);

        uint8_t rxbytes = CC2500_ReadReg(REG_RXBYTES) & 0x7F;

        if (rxbytes == 0 || rxbytes > 64)
        {
            HAL_Delay(5);
            continue;
        }

        uint8_t buf[64];
        CC2500_ReadBurst(REG_RXFIFO, buf, rxbytes);

        uint8_t len = buf[0];

        if (len == 0 || len > 60 || len + 3 > rxbytes)
        {
            printf("[RX] bad len %d\r\n", len);
            CC2500_Strobe(STROBE_SFRX);
            continue;
        }

        uint8_t *payload = &buf[1];
        uint8_t rssi_raw = buf[len + 1];
        uint8_t lqi_crc  = buf[len + 2];

        uint8_t lqi = lqi_crc & 0x7F;
        uint8_t crc = lqi_crc >> 7;

        int8_t rssi = (int8_t)(rssi_raw / 2) - 72;

        printf("[RX] \"%.*s\" RSSI=%d LQI=%d CRC=%d\r\n",
               len, payload, rssi, lqi, crc);

        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

        CC2500_Strobe(STROBE_SFRX);
        HAL_Delay(1);
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SELF-TEST FUNCTIONS  (from bring-up, kept for diagnostics)
 * ════════════════════════════════════════════════════════════════════════════ */

void CC2500_Test(void)
{
    printf("\r\n=============================\r\n");
    printf("   CC2500 SPI TEST\r\n");
    printf("   CS=PB11 SCK=PA1\r\n");
    printf("   MISO=PA6 MOSI=PA7\r\n");
    printf("=============================\r\n\r\n");

    printf("[1] Resetting CC2500...\r\n");
    CC2500_Reset();

    printf("[2] Chip identification...\r\n");
    uint8_t partnum = CC2500_ReadReg(REG_PARTNUM);
    uint8_t version = CC2500_ReadReg(REG_VERSION);
    printf("    PARTNUM = 0x%02X  (expect 0x80) %s\r\n",
           partnum, partnum == 0x80 ? "OK" : "FAIL");
    printf("    VERSION = 0x%02X  (expect 0x03) %s\r\n",
           version, version == 0x03 ? "OK" : "FAIL");

    if (partnum != 0x80 || version != 0x03) {
        printf("\r\nCOMMS FAILED — halting\r\n");
        while (1) {
            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
            HAL_Delay(100);
        }
    }

    printf("\r\n[3] Register write/readback...\r\n");
    uint8_t vals[] = { 0xAA, 0x55, 0x12, 0xA5 };
    uint8_t all_ok = 1;
    for (int i = 0; i < 4; i++) {
        CC2500_WriteReg(REG_CHANNR, vals[i]);
        uint8_t rb = CC2500_ReadReg(REG_CHANNR);
        printf("    Wrote 0x%02X  Read 0x%02X  %s\r\n",
               vals[i], rb, rb == vals[i] ? "OK" : "MISMATCH");
        if (rb != vals[i]) all_ok = 0;
    }

    printf("\r\n[4] State machine (SIDLE strobe)...\r\n");
    CC2500_Strobe(STROBE_SIDLE);
    HAL_Delay(5);
    uint8_t state = CC2500_ReadReg(REG_MARCSTATE) & 0x1F;
    printf("    MARCSTATE = 0x%02X (%s) %s\r\n",
           state, marcstate_str(state),
           state == MARCSTATE_IDLE ? "OK" : "FAIL");

    printf("\r\n=============================\r\n");
    if (all_ok && state == MARCSTATE_IDLE) {
        printf("   ALL TESTS PASSED\r\n");
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    } else {
        printf("   SOME TESTS FAILED\r\n");
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    }
    printf("=============================\r\n");
}

void CC2500_TxLoopbackTest(void)
{
    printf("\r\n[5] TX loopback test...\r\n");

    CC2500_WaitIdle(50);
    CC2500_Strobe(STROBE_SIDLE);
    CC2500_Strobe(STROBE_SFTX);

    uint8_t payload[5] = {4, 0xDE, 0xAD, 0xBE, 0xEF};

    CC2500_WriteBurst(REG_TXFIFO, payload, 5);

    uint8_t txbytes = CC2500_ReadReg(REG_TXBYTES) & 0x7F;

    printf("TXBYTES=%d (expect 5)\r\n", txbytes);

    CC2500_Strobe(STROBE_STX);

    HAL_Delay(20);

    uint8_t state = CC2500_ReadReg(REG_MARCSTATE) & 0x1F;

    printf("Post-TX state=0x%02X (%s)\r\n", state, marcstate_str(state));
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

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_I2C1_Init();
    MX_IWDG_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */
    HAL_Delay(200);   /* Let SWO viewer connect */

    /* ── Step 1: SPI self-test ── */
    CC2500_Test();

    /* ── Step 2: Apply RF configuration ── */
    printf("\r\n[6] Applying RF configuration...\r\n");
    CC2500_RFConfig();
    printf("    Frequency : %.3f MHz\r\n", CC2500_GetFrequency() / 1e6);
    printf("    LQI       : 0x%02X\r\n", CC2500_ReadReg(REG_LQI));
    printf("    RSSI      : 0x%02X\r\n", CC2500_ReadReg(REG_RSSI));
    uint8_t st = CC2500_ReadReg(REG_MARCSTATE) & 0x1F;
    printf("    MARCSTATE : 0x%02X (%s)\r\n", st, marcstate_str(st));

    /* ── Step 3: Raw FIFO loopback ── */
    CC2500_TxLoopbackTest();

    /* ── Step 4: Formatted packet TX ── */
    printf("\r\n[7] Sending formatted packet...\r\n");
    uint8_t pkt[] = "Hello from PCB!!";
    uint8_t len = strlen((char*)pkt);
    uint8_t tx[len + 1];
    tx[0] = len;
    memcpy(&tx[1], pkt, len);
    CC2500_TransmitPacket(tx, len + 1);

    /* ════════════════════════════════════════════════════════════
     *  CHOOSE MODE — comment/uncomment one of the two blocks below
     * ════════════════════════════════════════════════════════════ */

    /* ── RECEIVER MODE (default) ── */
//    CC2500_ReceiveLoop();   /* blocking — never returns */

    // ── TRANSMITTER MODE ──
    printf("\r\n[TX] Continuous transmit @ 200 ms...\r\n");
    uint8_t tx_pkt[17];
    tx_pkt[0] = 16;
    uint32_t count = 0;
    while (1) {
        snprintf((char *)&tx_pkt[1], 16, "Pkt %010lu", count++);
        CC2500_TransmitPacket(tx_pkt, 17);
        HAL_Delay(200);
    }


    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN 3 */
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(1000);
        /* USER CODE END 3 */
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 *  PERIPHERAL INITIALISATION
 * ════════════════════════════════════════════════════════════════════════════ */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
        Error_Handler();

    /* HSE=12 MHz, PLL: PLLM=1, PLLN=8, PLLR=2 → SYSCLK=48 MHz */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.LSIState       = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 1;
    RCC_OscInitStruct.PLL.PLLN       = 8;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ       = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();

    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/**
  * @brief CAN1 Initialization Function
  */
static void MX_CAN1_Init(void)
{
    hcan1.Instance                  = CAN1;
    hcan1.Init.Prescaler            = 16;
    hcan1.Init.Mode                 = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1             = CAN_BS1_1TQ;
    hcan1.Init.TimeSeg2             = CAN_BS2_1TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = DISABLE;
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = DISABLE;
    hcan1.Init.ReceiveFifoLocked    = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
        Error_Handler();
}

/**
  * @brief I2C1 Initialization Function
  */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x10805D88;
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
        Error_Handler();

    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
        Error_Handler();

    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
        Error_Handler();
}

/**
  * @brief IWDG Initialization Function
  */
static void MX_IWDG_Init(void)
{
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
    hiwdg.Init.Window    = 4095;
    hiwdg.Init.Reload    = 4095;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
        Error_Handler();
}

/**
  * @brief SPI1 Initialization Function
  * @note  8-bit data, Mode 0 (CPOL=0/CPHA=0), prescaler /8 → 6 MHz @ 48 MHz SYSCLK
  *        (within CC2500's 10 MHz max).  NSS pulse disabled because CS is driven
  *        manually via CC2500_CS_LOW/HIGH.
  */
static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;       /* FIX: was 4BIT  */
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;        /* CPOL=0         */
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;         /* CPHA=0 → Mode 0*/
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; /* FIX: was /2 (too fast) */
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 7;
    hspi1.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;   /* FIX: was ENABLE */
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
        Error_Handler();
}

/**
  * @brief USART1 Initialization Function
  * @note  Hardware flow control disabled — RTS/CTS pins are not connected.
  */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance                    = USART1;
    huart1.Init.BaudRate               = 115200;
    huart1.Init.WordLength             = UART_WORDLENGTH_8B;
    huart1.Init.StopBits               = UART_STOPBITS_1;
    huart1.Init.Parity                 = UART_PARITY_NONE;
    huart1.Init.Mode                   = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl              = UART_HWCONTROL_NONE; /* FIX: was RTS_CTS */
    huart1.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
        Error_Handler();
}

/**
  * @brief  Enable DMA controller clock and configure IRQ priorities.
  * @note   Priorities set to 6 — must be numerically > configMAX_SYSCALL_INTERRUPT_PRIORITY (5)
  *         so FreeRTOS can safely mask them during critical sections.
  */
static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* FIX: priorities were 0 (above SYSCALL limit); raised to 6 */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* ── Output initial levels ── */
    HAL_GPIO_WritePin(GPIOA,
        CAN_LOOPBACK_EN_Pin | CAN_STANDBY_Pin | CC2500_SS_Pin | ADF4371_SS_Pin,
        GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB,
        ADF4371_EN_Pin | REG_3V3_EN_Pin | REG_2V0_EN_Pin | ADF4371_OSC_EN_Pin
        | REG_6V5_EN_Pin | REG_5V0_EN_Pin,
        GPIO_PIN_RESET);

    /* LED = PA4, default OFF */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = LED_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    /* CC2500_SS = PA (mapped via CubeMX pin) — set HIGH before init to avoid glitch */
    HAL_GPIO_WritePin(CC2500_CS_PORT, CC2500_CS_PIN, GPIO_PIN_SET);

    /* REG_3V3_AO_PG — falling edge EXTI */
    GPIO_InitStruct.Pin  = REG_3V3_AO_PG_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(REG_3V3_AO_PG_GPIO_Port, &GPIO_InitStruct);

    /* TPS_STATUS — plain input */
    GPIO_InitStruct.Pin  = TPS_STATUS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(TPS_STATUS_GPIO_Port, &GPIO_InitStruct);

    /* TEMP_ALARM, POWER_ALARM — falling edge EXTI */
    GPIO_InitStruct.Pin  = TEMP_ALARM_Pin | POWER_ALARM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN_LOOPBACK_EN, CAN_STANDBY, CC2500_SS, ADF4371_SS — outputs */
    GPIO_InitStruct.Pin   = CAN_LOOPBACK_EN_Pin | CAN_STANDBY_Pin
                          | CC2500_SS_Pin | ADF4371_SS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADF4371_EN, REG_3V3_EN, REG_2V0_EN, ADF4371_OSC_EN,
       REG_6V5_EN, REG_5V0_EN — outputs */
    GPIO_InitStruct.Pin   = ADF4371_EN_Pin | REG_3V3_EN_Pin | REG_2V0_EN_Pin
                          | ADF4371_OSC_EN_Pin | REG_6V5_EN_Pin | REG_5V0_EN_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CC2500_STATUS1 (GDO1=PB1), CC2500_STATUS2 (GDO2=PB2) — plain inputs
     * FIX: were IT_RISING; GDO polling is done in software, no EXTI needed */
    GPIO_InitStruct.Pin  = CC2500_STATUS1_Pin | CC2500_STATUS2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* REG_2V0_PG, REG_6V5_PG, REG_5V0_PG — falling edge EXTI */
    GPIO_InitStruct.Pin  = REG_2V0_PG_Pin | REG_6V5_PG_Pin | REG_5V0_PG_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PA8 — MCO output */
    GPIO_InitStruct.Pin       = GPIO_PIN_8;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
