#pragma once
#include "SPI.h"

class HalfDuplexSPI : protected SPIClass {
  uint32_t ssel;

  public:
    HalfDuplexSPI(bool master, uint32_t data = MOSI, uint32_t sclk = SCK, uint32_t ssel = PNUM_NOT_DEFINED) : SPIClass(data, PNUM_NOT_DEFINED, sclk, ssel)
    {
      this->ssel = ssel;
      setDirection(SPI_DIRECTION_1LINE);
      setIsMaster(master);
    }

    bool begin()
    {
      return SPIClass::begin();
    }

    bool beginTransaction(SPISettings settings)
    {
      return SPIClass::beginTransaction(settings);
    }
 
    void endTransaction()
    {
      SPIClass::endTransaction();
    }

    void end()
    {
      SPIClass::end();
    }
    
    HAL_StatusTypeDef transmit(uint8_t *tx_buf, size_t count, uint32_t timeout = HAL_MAX_DELAY)
    {
      digitalWrite(ssel, LOW);
      auto error = HAL_SPI_Transmit(getHandle(), tx_buf, count, timeout);
      digitalWrite(ssel, HIGH);
      if (error != HAL_OK)
        return error;
      return HAL_OK;
    }

    HAL_SPI_StateTypeDef getState() {
      return HAL_SPI_GetState(getHandle());
    }

    SPI_HandleTypeDef* getHandle() {
      return SPIClass::getHandle();
    }

    bool isBusy() {
      auto state = getState();
      return state == HAL_SPI_STATE_BUSY || state == HAL_SPI_STATE_BUSY_TX || state == HAL_SPI_STATE_BUSY_RX || state == HAL_SPI_STATE_BUSY_TX_RX;
    }

    HAL_StatusTypeDef receiveAsync(uint8_t *rx_buf, size_t count)
    {
      Serial.printf("Async receive. Count: %d\n", count);
      digitalWrite(ssel, LOW);
      auto error = HAL_SPI_Receive_IT(getHandle(), rx_buf, count);
      if (error != HAL_OK)
        return error;

      return HAL_OK;
    }

    HAL_StatusTypeDef abortTransfer() {
      digitalWrite(ssel, HIGH);
      return HAL_SPI_Abort_IT(getHandle());
    }
};