/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CC2500 Full TX/RX Driver
  * Pins: CS=PB11, SCK=PA1, MISO=PA6, MOSI=PA7
  * GDO1=PB1, GDO2=PB2, LED=PA4
  * Debug: SWO ITM + USART1 @ 115200
  * @note           : Heavily commented to explain SPI logic, state machine
  * transitions, and RF configurations.
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

/* Debug Indicator LED = PA4 */
#define LED_PORT    GPIOA
#define LED_PIN     GPIO_PIN_4
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* CC2500 SPI Command Header Bits */
#define CC2500_READ   0x80  /* Bit 7 = 1 indicates a Read operation, 0 is Write */
#define CC2500_BURST  0x40  /* Bit 6 = 1 indicates a Burst (multi-byte) access */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ═══════════════════════════════════════════════════════
 * GPIO MAPPING (CORRECTED)
 * ═══════════════════════════════════════════════════════ */

/* Chip Select (CS) Control Macros - Manually driven for SPI */
#define CC2500_CS_PORT    GPIOB
#define CC2500_CS_PIN     GPIO_PIN_11
#define CC2500_CS_LOW()   HAL_GPIO_WritePin(CC2500_CS_PORT, CC2500_CS_PIN, GPIO_PIN_RESET)
#define CC2500_CS_HIGH()  HAL_GPIO_WritePin(CC2500_CS_PORT, CC2500_CS_PIN, GPIO_PIN_SET)

/* General Digital Output (GDO) pins from CC2500 to STM32 */
#define GDO0_PORT   GPIOB   // Mapped to PB1
#define GDO0_PIN    GPIO_PIN_1
#define GDO2_PORT   GPIOB   // Mapped to PB2
#define GDO2_PIN    GPIO_PIN_2

/* Macros to read the hardware state of the GDO pins */
#define GDO0_READ() HAL_GPIO_ReadPin(GDO0_PORT, GDO0_PIN)
#define GDO2_READ() HAL_GPIO_ReadPin(GDO2_PORT, GDO2_PIN)

/* ═══════════════════════════════════════════════════════
 * REGISTERS - See CC2500 Datasheet for full details
 * ═══════════════════════════════════════════════════════ */

/* Status Registers (Read-Only) */
#define REG_PARTNUM   0x30  // Chip part number
#define REG_VERSION   0x31  // Chip version
#define REG_LQI       0x33  // Link Quality Indicator
#define REG_RSSI      0x34  // Received Signal Strength Indicator
#define REG_MARCSTATE 0x35  // Main Radio Control State Machine state
#define REG_TXBYTES   0x3A  // Bytes in TX FIFO
#define REG_RXBYTES   0x3B  // Bytes in RX FIFO

/* Configuration Registers */
#define REG_IOCFG0    0x02  // GDO0 output pin configuration
#define REG_IOCFG1    0x01  // GDO1 output pin configuration
#define REG_IOCFG2    0x00  // GDO2 output pin configuration

#define REG_PKTLEN    0x06  // Packet length
#define REG_PKTCTRL0  0x08  // Packet automation control
#define REG_PKTCTRL1  0x07  // Packet automation control
#define REG_CHANNR    0x0A  // Channel number

/* Frequency control words */
#define REG_FREQ2     0x0D  // Frequency control word, high byte
#define REG_FREQ1     0x0E  // Frequency control word, middle byte
#define REG_FREQ0     0x0F  // Frequency control word, low byte

/* Modem configuration */
#define REG_MDMCFG4   0x10  // Modem configuration (data rate exponent, channel BW)
#define REG_MDMCFG3   0x11  // Modem configuration (data rate mantissa)
#define REG_MDMCFG2   0x12  // Modem configuration (modulation format, sync mode)
#define REG_MDMCFG1   0x13  // Modem configuration (FEC, preamble length)
#define REG_DEVIATN   0x15  // Modem deviation setting

/* Main Radio Control State Machine configuration */
#define REG_MCSM1     0x17
#define REG_MCSM0     0x18

#define REG_FOCCFG    0x19  // Frequency Offset Compensation config
#define REG_AGCCTRL2  0x1B  // AGC control

#define REG_FREND0    0x22  // Front end TX configuration
#define REG_FSCAL1    0x25  // Frequency synthesizer calibration
#define REG_FSCAL0    0x26  // Frequency synthesizer calibration

/* FIFO and Power Access */
#define REG_PATABLE   0x3E  // Power amplifier output power table
#define REG_TXFIFO    0x3F  // TX FIFO (Write)
#define REG_RXFIFO    0x3F  // RX FIFO (Read)

/* ═══════════════════════════════════════════════════════
 * COMMAND STROBES (Single-byte commands)
 * ═══════════════════════════════════════════════════════ */

#define STROBE_SRES   0x30  // Reset chip
#define STROBE_SFSTXON 0x31 // Enable and calibrate frequency synthesizer
#define STROBE_SXOFF  0x32  // Turn off crystal oscillator
#define STROBE_SCAL   0x33  // Calibrate frequency synthesizer and turn it off
#define STROBE_SRX    0x34  // Enable RX
#define STROBE_STX    0x35  // Enable TX
#define STROBE_SIDLE  0x36  // Exit RX / TX, turn off frequency synthesizer
#define STROBE_SFRX   0x3A  // Flush the RX FIFO
#define STROBE_SFTX   0x3B  // Flush the TX FIFO

/* ── MARCSTATE values (State Machine Status) ────────────────────────────────── */
#define MARCSTATE_IDLE 0x01
#define MARCSTATE_RX   0x0D
#define MARCSTATE_TX   0x13

/* ── Crystal and frequency constraints ──────────────────────────────────────── */
#define FXOSC        26000000.0    /* CC2500 requires a 26 MHz crystal */
#define CENTRE_FREQ  2433000000.0  /* Targeted base frequency: 2.433 GHz */

/* ── RX buffer size ─────────────────────────────────────────────────────────── */
#define RX_BUF_SIZE  64  /* CC2500 hardware FIFO is exactly 64 bytes */

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

/**
 * @brief Overrides standard printf to output to both SWO ITM and USART1.
 * Useful for redundant debugging depending on what cables are plugged in.
 */
int __io_putchar(int ch)
{
    ITM_SendChar(ch); // Send to SWO Viewer
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000); // Send to Serial Terminal
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
 * LOW-LEVEL SPI FUNCTIONS
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Writes a single byte to a specific CC2500 register.
 */
void CC2500_WriteReg(uint8_t addr, uint8_t val)
{
    // Mask with 0x3F ensures bits 6 and 7 are 0 (Write mode, Single byte)
    uint8_t tx_buf[2] = { addr & 0x3F, val };
    uint8_t rx_buf[2]; // Captures and flushes the incoming bytes automatically

    CC2500_CS_LOW(); // Initiate SPI transaction
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, HAL_MAX_DELAY);
    CC2500_CS_HIGH(); // Terminate SPI transaction
}

/**
 * @brief Reads a single byte from a specific CC2500 register.
 */
uint8_t CC2500_ReadReg(uint8_t addr)
{
    // Status registers (>= 0x30) require the BURST bit to be set during reads,
    // otherwise the chip expects a Strobe command. Configuration registers do not.
    uint8_t tx = (addr >= 0x30) ? (addr | CC2500_READ | CC2500_BURST)
                                : (addr | CC2500_READ);
    uint8_t tx_buf[2] = { tx, 0x00 }; // Byte 1: Header, Byte 2: Dummy clocking byte
    uint8_t rx_buf[2] = { 0, 0 };

    CC2500_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, HAL_MAX_DELAY);
    CC2500_CS_HIGH();

    // The first byte returned is the status byte; the second is the actual register data
    return rx_buf[1];
}

/**
 * @brief Sends a 1-byte instruction (Strobe) to the CC2500.
 */
uint8_t CC2500_Strobe(uint8_t cmd)
{
    uint8_t status = 0;
    CC2500_CS_LOW();
    // Sending a single byte. The CC2500 immediately replies with its global status byte
    HAL_SPI_TransmitReceive(&hspi1, &cmd, &status, 1, HAL_MAX_DELAY);
    CC2500_CS_HIGH();
    return status;
}

/**
 * @brief Writes multiple bytes consecutively (Burst Write).
 */
void CC2500_WriteBurst(uint8_t addr, const uint8_t *data, uint8_t len)
{
    uint8_t tx_buf[65];
    uint8_t rx_buf[65];

    // Hardware limit: FIFO cannot hold more than 64 bytes
    if (len > 64) len = 64;

    // Set Burst bit (0x40). Write bit (0x80) is naturally 0.
    tx_buf[0] = (addr & 0x3F) | CC2500_BURST;
    memcpy(&tx_buf[1], data, len); // Append payload after header byte

    CC2500_CS_LOW();
    // Transmit and receive simultaneously to prevent internal MCU FIFO overflow
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, len + 1, HAL_MAX_DELAY);
    CC2500_CS_HIGH();
}

/**
 * @brief Reads multiple bytes consecutively (Burst Read).
 */
void CC2500_ReadBurst(uint8_t addr, uint8_t *data, uint8_t len)
{
    uint8_t tx_buf[65];
    uint8_t rx_buf[65];
    if (len > 64) len = 64;

    memset(tx_buf, 0x00, len + 1); // Fill with dummy bytes to keep clock running
    // Set both Read (0x80) and Burst (0x40) bits
    tx_buf[0] = addr | CC2500_READ | CC2500_BURST;

    CC2500_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, len + 1, HAL_MAX_DELAY);
    CC2500_CS_HIGH();

    // Copy received bytes (offset by 1 to skip the initial status byte) into user buffer
    memcpy(data, &rx_buf[1], len);
}

/* ════════════════════════════════════════════════════════════════════════════
 * MID-LEVEL HELPERS
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Hardware-level reset sequence specified by the CC2500 datasheet.
 */
void CC2500_Reset(void)
{
    // Toggle CS high-low-high to trigger internal reset circuitry
    CC2500_CS_HIGH(); HAL_Delay(2);
    CC2500_CS_LOW();  HAL_Delay(2);
    CC2500_CS_HIGH(); HAL_Delay(2);

    CC2500_Strobe(STROBE_SRES); // Send Software Reset Strobe
    HAL_Delay(10);              // Allow oscillators to stabilize
}

/**
 * @brief Blocks execution until the CC2500 state machine enters the IDLE state.
 */
uint8_t CC2500_WaitIdle(uint32_t timeout_ms)
{
    CC2500_Strobe(STROBE_SIDLE); // Command chip to go to IDLE
    uint32_t deadline = HAL_GetTick() + timeout_ms;

    // Mask with 0x1F removes reserved upper bits from MARCSTATE
    while ((CC2500_ReadReg(REG_MARCSTATE) & 0x1F) != MARCSTATE_IDLE) {
        if (HAL_GetTick() > deadline) return 0; // Timeout failure
    }
    return 1; // Success
}

/**
 * @brief Converts hexadecimal MARCSTATE value into a human-readable string.
 */
static const char *marcstate_str(uint8_t s)
{
    switch (s) {
    		case 0x00: return "SLEEP";
            case 0x01: return "IDLE";
            case 0x0D: return "RX";
            case 0x11: return "RX_OVERFLOW";
            case 0x13: return "TX";
            case 0x16: return "TX_UNDERFLOW";
            default:   return "UNKNOWN";
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * RF CONFIGURATION  –  2.433 GHz, 250 kbps GFSK
 * ════════════════════════════════════════════════════════════════════════════ */

void CC2500_RFConfig(void)
{
    // FIX 2: Configure GDO pins explicitly to link hardware pins to the RF engine
    // REG_IOCFG0 (Controls PB1/GDO0): 0x06 asserts on sync word match, deasserts at End-of-Packet
    CC2500_WriteReg(REG_IOCFG0, 0x06);

    // REG_IOCFG2 (Controls PB2/GDO2): 0x29 outputs CHIP_RDYn (low when crystal oscillator is stable)
    CC2500_WriteReg(REG_IOCFG2, 0x29);

    // Set Base Frequency to 2.433 GHz
    CC2500_WriteReg(REG_FREQ2, 0x5D);
    CC2500_WriteReg(REG_FREQ1, 0x93);
    CC2500_WriteReg(REG_FREQ0, 0xB1);

    // Modem Configurations (Baud rate, modulation type, sync words)
    CC2500_WriteReg(REG_MDMCFG4, 0x2D); // Channel bandwidth
    CC2500_WriteReg(REG_MDMCFG3, 0x3B); // Data rate
    CC2500_WriteReg(REG_MDMCFG2, 0x73); // Modulation format (GFSK), 30/32 sync bits detected
    CC2500_WriteReg(REG_MDMCFG1, 0x22); // FEC disabled, 4 preamble bytes
    CC2500_WriteReg(REG_DEVIATN, 0x34); // FSK deviation

    // Packet Automation Controls
    CC2500_WriteReg(REG_PKTCTRL0, 0x05); // Variable length packets, CRC enabled
    CC2500_WriteReg(REG_PKTCTRL1, 0x04); // Append two status bytes (RSSI, LQI) to RX payload
    CC2500_WriteReg(REG_PKTLEN, 60);     // Max payload length allowed

    // Power Amplifier settings (0xFE is roughly +1 dBm output power)
    uint8_t pa[8] = {0x00, 0xFE, 0, 0, 0, 0, 0, 0};
    CC2500_WriteBurst(REG_PATABLE, pa, 8);
}

/* ════════════════════════════════════════════════════════════════════════════
 * FREQUENCY CONTROL
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
    if (!CC2500_WaitIdle(100)) { // Chip must be IDLE to safely rewrite FREQ registers
        printf("SetFrequency: failed to reach IDLE\r\n");
        return;
    }

    // Formula derived from CC2500 datasheet: FREQ = (f * 2^16) / f_osc
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
 * TRANSMIT
 *
 * Packet format (variable length mode, PKTCTRL0=0x05):
 * data[0]        = length byte  (number of payload bytes that follow)
 * data[1..len-1] = payload bytes
 *
 * GDO1 (PB1 = CC2500_STATUS1) is wired to GDO0 of CC2500.
 * With IOCFG1=0x06 it asserts when sync is sent, deasserts at EOP.
 * MCSM1=0x3C returns chip to IDLE automatically after TX.
 * ════════════════════════════════════════════════════════════════════════════ */

void CC2500_TransmitPacket(const uint8_t *data, uint8_t len)
{
    // Force IDLE, flush the TX FIFO to clear old data
    CC2500_Strobe(STROBE_SIDLE);
    CC2500_Strobe(STROBE_SFTX);

    // Push the new payload into the hardware FIFO
    CC2500_WriteBurst(REG_TXFIFO, data, len);

    // Command the chip to begin Transmitting
    CC2500_Strobe(STROBE_STX);

    // FIX 3: Dynamic hardware state tracking instead of a blind 10ms guess
    // Block until the state machine leaves the TX state (packet is fully sent)
    uint32_t timeout = HAL_GetTick() + 50; // 50ms absolute safety ceiling
    while ((CC2500_ReadReg(REG_MARCSTATE) & 0x1F) == MARCSTATE_TX) {
        if (HAL_GetTick() > timeout) {
            printf("TX Error: State machine timeout!\r\n");
            break;
        }
    }

    // Post-transmit validation to handle unexpected RF locks
    // Verify success. 0x16 indicates the TX FIFO ran empty before packet completion
    uint8_t post_state = CC2500_ReadReg(REG_MARCSTATE) & 0x1F;
    if (post_state == 0x16) { // FIXED: 0x16 = TX_UNDERFLOW
        printf("TX Error: UNDERFLOW! Resetting TX FIFO...\r\n");
        CC2500_Strobe(STROBE_SIDLE);
        CC2500_Strobe(STROBE_SFTX);
    } else {
        printf("TX OK\r\n");
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * RECEIVE LOOP  (blocking)
 *
 * Received packet layout in RX FIFO (PKTCTRL1=0x04, append status on):
 * buf[0]             = length byte
 * buf[1 .. len]      = payload
 * buf[len+1]         = RSSI raw
 * buf[len+2]         = LQI[6:0] | CRC_OK[7]
 *
 * Call from main() when you want this board to be the receiver.
 * To use as transmitter instead, comment this out and use
 * CC2500_TransmitPacket() in a loop.
 * ════════════════════════════════════════════════════════════════════════════ */

void CC2500_ReceiveLoop(void)
{
    printf("\r\nRX MODE START\r\n");

    // FIX 1: Move initialization strobes OUTSIDE the infinite loop
    // Clear state and enter continuous Receive Mode
    CC2500_Strobe(STROBE_SIDLE);
    CC2500_Strobe(STROBE_SFRX); // Flush old junk in RX FIFO
    CC2500_Strobe(STROBE_SRX);  // Enter RX state

    while (1)
    {
        uint8_t rxbytes1 = 0, rxbytes2 = 0;

        // FIX 4: Asynchronous clock domain double-read protection
        // HARDWARE ERRATA WORKAROUND:
        // The RXBYTES register can glitch if read exactly while a byte is written to the FIFO
        // by the internal RF MAC. Double-reading ensures structural integrity of the count.
        do {
            rxbytes1 = CC2500_ReadReg(REG_RXBYTES) & 0x7F;
            rxbytes2 = CC2500_ReadReg(REG_RXBYTES) & 0x7F;
        } while (rxbytes1 != rxbytes2);

        uint8_t rxbytes = rxbytes1;

        // Handle hardware overflow conditions safely
        // Check if the FIFO received more than 64 bytes without being read (Overflow)
        uint8_t state = CC2500_ReadReg(REG_MARCSTATE) & 0x1F;
        if (state == 0x11) // 0x11 = RX_OVERFLOW
        {
            printf("[RX] Overflow encountered! Purging...\r\n");
            CC2500_Strobe(STROBE_SIDLE);
            CC2500_Strobe(STROBE_SFRX);
            CC2500_Strobe(STROBE_SRX);
            continue;
        }

        if (rxbytes == 0)
        {
            HAL_Delay(1); // Yield execution slightly to avoid thrashing the SPI bus
            continue;     // Spinlock until data arrives
        }

        // Pull the entire pending buffer
        uint8_t buf[64];
        CC2500_ReadBurst(REG_RXFIFO, buf, rxbytes);

        // First byte is the packet length (PKTCTRL0=0x05 variable length mode)
        uint8_t len = buf[0];

        // Validate structure integrity of the dynamic buffer length byte
        // Sanity check packet bounds
        if (len == 0 || len > 60 || len + 3 > rxbytes)
        {
            printf("[RX] Corrupted frame parsed: len=%d, bytes=%d\r\n", len, rxbytes);
            CC2500_Strobe(STROBE_SIDLE);
            CC2500_Strobe(STROBE_SFRX);
            CC2500_Strobe(STROBE_SRX);
            continue;
        }

        // Extract Payload
        uint8_t *payload = &buf[1];

        // Because PKTCTRL1=0x04, the chip automatically appends RSSI and LQI/CRC bits
        uint8_t rssi_raw = buf[len + 1];
        uint8_t lqi_crc  = buf[len + 2];

        uint8_t lqi = lqi_crc & 0x7F;      // Bottom 7 bits are Link Quality
        uint8_t crc = lqi_crc >> 7;        // Top bit indicates if hardware CRC passed

        // RSSI calculation formula from CC2500 datasheet
        int8_t rssi = (int8_t)(rssi_raw / 2) - 72;

        if (crc) {
            printf("[RX] \"%.*s\" RSSI=%d dBm LQI=%d CRC=OK\r\n", len, payload, rssi, lqi);
            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        } else {
            printf("[RX] Bad CRC frame dropped\r\n");
        }

        // Cleanly return to RX mode for next inbound packet
        CC2500_Strobe(STROBE_SIDLE);
        CC2500_Strobe(STROBE_SRX);
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * SELF-TEST FUNCTIONS  (from bring-up, kept for diagnostics)
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

    // Verify SPI communication by asking for hardcoded silicon ID numbers
    uint8_t partnum = CC2500_ReadReg(REG_PARTNUM);
    uint8_t version = CC2500_ReadReg(REG_VERSION);

    printf("    PARTNUM = 0x%02X  (expect 0x80) %s\r\n",
           partnum, partnum == 0x80 ? "OK" : "FAIL");
    printf("    VERSION = 0x%02X  (expect 0x03) %s\r\n",
           version, version == 0x03 ? "OK" : "FAIL");

    // 0x80 and 0x03 are constants from Texas Instruments for this silicon revision
    if (partnum != 0x80 || version != 0x03) {
        printf("\r\nCOMMS FAILED — halting\r\n");
        while (1) {
            HAL_GPIO_TogglePin(LED_PORT, LED_PIN); // Blink death loop
            HAL_Delay(100);
        }
    }

    printf("\r\n[3] Register write/readback...\r\n");

    // Write a series of bytes to a harmless register and read them back to ensure MISO/MOSI health
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

    // Dummy Payload: Size (4) + Data (DE AD BE EF)
    uint8_t payload[5] = {4, 0xDE, 0xAD, 0xBE, 0xEF};

    // Push to TX FIFO
    CC2500_WriteBurst(REG_TXFIFO, payload, 5);

    // Read back the internal register that counts bytes waiting in the TX FIFO
    uint8_t txbytes = CC2500_ReadReg(REG_TXBYTES) & 0x7F;

    printf("TXBYTES=%d (expect 5)\r\n", txbytes);

    // Transmit!
    CC2500_Strobe(STROBE_STX);

    HAL_Delay(20); // Wait for physical air propagation

    // Post-TX state check. Should fall back to IDLE (0x01)
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

    /* ── Initialize Peripherals ── */
    MX_GPIO_Init();

    // Overriding CubeMX's CS Pin config to enforce VERY_HIGH speed (sharp edges for SPI reliability)
    GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
    GPIO_InitStructPrivate.Pin   = CC2500_CS_PIN;
    GPIO_InitStructPrivate.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructPrivate.Pull  = GPIO_NOPULL;
    GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // Fast edges for clean SPI framing
    HAL_GPIO_Init(CC2500_CS_PORT, &GPIO_InitStructPrivate);

    // Immediately pull it high to finish the boot-glitch fix
    // Force CS HIGH immediately so the CC2500 doesn't think an SPI transaction is starting
    CC2500_CS_HIGH();

    MX_DMA_Init();
    MX_CAN1_Init();
    MX_I2C1_Init();
//    MX_IWDG_Init(); // Watchdog disabled for debugging
    MX_SPI1_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */
    HAL_Delay(200);   /* Let SWO viewer or terminal application connect */

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
    tx[0] = len; // Prepend dynamic length byte
    memcpy(&tx[1], pkt, len);
    CC2500_TransmitPacket(tx, len + 1);

    /* ════════════════════════════════════════════════════════════
     * CHOOSE MODE — comment/uncomment one of the two blocks below
     * ════════════════════════════════════════════════════════════ */

    /* ── RECEIVER MODE (default) ── */
//    CC2500_ReceiveLoop();   /* blocking — never returns */

    // ── TRANSMITTER MODE ──
    printf("\r\n[TX] Continuous transmit @ 200 ms...\r\n");

        uint8_t tx_pkt[17];
        uint32_t count = 0;

        while (1)
        {
            memset(tx_pkt, 0, sizeof(tx_pkt));

            // Define exactly how many bytes of actual payload we want to send
            uint8_t payload_len = 16;
            tx_pkt[0] = payload_len; // Byte 0 is length marker

            // Use payload_len + 1 (or sizeof(tx_pkt) - 1) to allow snprintf to use all 16 slots for data
            snprintf((char *)&tx_pkt[1], payload_len + 1, "Pkt %010lu", count++);

            // Transmit total size: 1 length byte + 16 payload bytes = 17 bytes
            CC2500_TransmitPacket(tx_pkt, payload_len + 1);

            HAL_GPIO_TogglePin(LED_PORT, LED_PIN); // Blink to show life
            HAL_Delay(200); // Wait 200ms before next send
        }


    /* USER CODE END 2 */

    // Fallback loop if the application drops out of TX/RX logic
    while (1)
    {
        /* USER CODE BEGIN 3 */
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(1000);
        /* USER CODE END 3 */
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * PERIPHERAL INITIALISATION
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
//    hiwdg.Instance       = IWDG;
//    hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
//    hiwdg.Init.Window    = 4095;
//    hiwdg.Init.Reload    = 4095;
//    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
//        Error_Handler();
}

/**
  * @brief SPI1 Initialization Function
  * @note  8-bit data, Mode 0 (CPOL=0/CPHA=0), prescaler /8 → 6 MHz @ 48 MHz SYSCLK
  * (within CC2500's 10 MHz max).  NSS pulse disabled because CS is driven
  * manually via CC2500_CS_LOW/HIGH.
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
  * so FreeRTOS can safely mask them during critical sections.
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
