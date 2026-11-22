#include "headfile.h"

static uint8_t spi_transfer_byte(SPI_Regs *spi, uint8_t data)
{
    uint8_t rx_data;

    while (DL_SPI_isBusy(spi)) {
    }

    while (!DL_SPI_isRXFIFOEmpty(spi)) {
        (void)DL_SPI_receiveData8(spi);
    }

    DL_SPI_transmitDataBlocking8(spi, data);
    rx_data = DL_SPI_receiveDataBlocking8(spi);

    while (DL_SPI_isBusy(spi)) {
    }

    return rx_data;
}

uint8_t spi0_transfer_byte(uint8_t data)
{
    return spi_transfer_byte(SPI_0_INST, data);
}

void spi0_wait_idle(void)
{
    while (DL_SPI_isBusy(SPI_0_INST)) {
    }
}

uint8_t spi1_transfer_byte(uint8_t data)
{
    return spi_transfer_byte(SPI_1_INST, data);
}

void spi1_wait_idle(void)
{
    while (DL_SPI_isBusy(SPI_1_INST)) {
    }
}
