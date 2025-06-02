#include "peripheral.h"

void SPI_MODE_MT(SPI_HandleTypeDef *hspi) {
  HAL_SPI_DeInit(hspi);
  hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
  HAL_SPI_Init(hspi);
}

void SPI_MODE_DRV(SPI_HandleTypeDef *hspi) {
  HAL_SPI_DeInit(hspi);
  hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
  HAL_SPI_Init(hspi);
}

#define PERIOD (float)0xFFF7
void DUTY_CYCLE(HRTIM_HandleTypeDef *hrrtim, uint32_t timind, float duty) {
  if (duty < 0.001f) {
    duty = 0.001f;
  } else if (duty > 1.0f) {
    duty = 1.0f;
  }
  __HAL_HRTIM_SetCompare(&hrrtim, timind, HRTIM_COMPAREUNIT_1,
                         (uint32_t)(PERIOD * (1.0f - duty)) / 2);
  __HAL_HRTIM_SetCompare(&hrrtim, timind, HRTIM_COMPAREUNIT_2,
                         (uint32_t)(PERIOD * (1.0f + duty)) / 2);
}

float MT_READ(SPI_HandleTypeDef *hspi) {
  uint8_t tx_buf[2] = {
      0x00, 0x00};      // Transmit dummy data (MT6701 ignores MOSI during read)
  uint8_t rx_buf[2];    // Buffer to receive 2 bytes (16 bits)
  uint16_t angle_raw;   // To store the 14-bit raw angle value
  float angle_degrees;  // To store the final angle in degrees

  HAL_GPIO_WritePin(MT_CS_GPIO_Port, MT_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(MT_CS_GPIO_Port, MT_CS_Pin, GPIO_PIN_SET);

  if (status != HAL_OK) {
    return -1.0f;  // Return -1.0f to indicate an error
  }

  // Why is [0] the LSB and [1] the MSB? Its set to MSB first...
  uint16_t received_16bit_word = ((uint16_t)rx_buf[1] << 8) | rx_buf[0];

  // Mask out first bit, which is dummy bit
  uint16_t data_shifted_for_dummy = received_16bit_word & 0x7FFF;

  // Extract first 14 bits, ignoring Mg bits
  angle_raw =
      (data_shifted_for_dummy >> 1) & 0x3FFF;  // 0x3FFF is a 14-bit mask

  angle_degrees = ((float)angle_raw) / 16384.0f * 360.0f;

  return angle_degrees;
}

uint16_t DRV_Read(SPI_HandleTypeDef *hspi, uint8_t address) {
  uint16_t command_word;   // Use uint16_t for the 16-bit command/response
  uint16_t received_word;  // To store the 16-bit data received over MISO
  uint16_t data_value;     // To store the extracted 11-bit data

  // --- Step 1: Construct the 16-bit READ command ---
  // The DRV8305 expects a 16-bit frame:
  // Bit 15: R/W bit (1 for READ, 0 for WRITE)
  // Bits 14-11: 4-bit Register Address
  // Bits 10-0: 11-bit "Don't Care" data (for read operations)

  // Set Bit 15 to 1 for a READ operation (1U << 15)
  // Mask the address to ensure it's only 4 bits, then shift to bits 14-11
  // The lower 11 bits (0-10) are '0' as "Don't Care" for a read
  command_word = (1U << 15) | ((address & 0x0F) << 11);

  HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_TransmitReceive(hspi, (uint8_t *)&command_word,
                              (uint8_t *)&received_word, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_SET);

  if (status != HAL_OK) {
    return 0xFFFF;
  }

  // The DRV8305's 16-bit response typically has this format:
  // Bit 15: Fault/Status bit (often indicates if a fault occurred during the
  // read) Bits 14-11: The original 4-bit Register Address that was read Bits
  // 10-0: The actual 11-bit Data value from the register

  // Extract the 11-bit data value (bits 10 to 0) from the received 16-bit word
  data_value = received_word & 0x07FF;  // 0x07FF is binary 0000 0111 1111 1111

  return data_value;
}

HAL_StatusTypeDef DRV_Write(SPI_HandleTypeDef *hspi, uint8_t address,
                            uint16_t data) {
  uint16_t command_word;  // Use uint16_t for the 16-bit command
  uint16_t tx_data;       // To hold the 16-bit data for transmission

  // --- Step 1: Clamp data to 11 bits ---
  // Ensure the input 'data' fits within the 11-bit data field [10:0].
  data &= 0x07FF;  // 0x07FF is binary 0000 0111 1111 1111

  // --- Step 2: Build the 16-bit WRITE command word ---
  // The DRV8305 expects a 16-bit frame:
  // Bit 15: R/W bit (0 for WRITE, 1 for READ)
  // Bits 14-11: 4-bit Register Address
  // Bits 10-0: 11-bit Data to write

  // Set Bit 15 to 0 for a WRITE operation (no need to explicitly 'OR' with 0 <<
  // 15) Mask the address to ensure it's only 4 bits, then shift to bits 14-11
  // 'OR' with the clamped 11-bit 'data'
  command_word = ((address & 0x0F) << 11) | data;  // Bit 15 remains 0
  tx_data = command_word;

  HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(hspi, (uint8_t *)&tx_data, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_SET);

  return status;
}