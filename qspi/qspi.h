#ifndef QSPI_H
#define QSPI_H

#include <xqspips.h>

typedef struct qspi {
    XQspiPs* 		instance;
    XQspiPs_Config* config;
    u16 			device_id;
} qspi_t;

qspi_t qspi_handler(XQspiPs*, uint16_t);
int qspi_initialize(qspi_t* qspi);
int qspi_get_flash_id(qspi_t* qspi);
int qspi_write(qspi_t* qspi, uint32_t address, uint8_t* buffer, uint32_t length);
int qspi_read(qspi_t* qspi, uint32_t address, uint8_t* buffer, uint32_t length);
int qspi_erase(qspi_t* qspi, uint32_t address, uint32_t byte_count);

#endif
