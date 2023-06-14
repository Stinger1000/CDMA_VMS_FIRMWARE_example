#include "qspi.h"

#include <string.h>

#define WRITE_STATUS_CMD 0x01 /*Flash commands*/
#define WRITE_CMD 0x02
#define READ_CMD 0x03
#define WRITE_DISABLE_CMD 0x04
#define READ_STATUS_CMD 0x05
#define WRITE_ENABLE_CMD 0x06
#define FAST_READ_CMD 0x0B
#define DUAL_READ_CMD 0x3B
#define QUAD_READ_CMD 0x6B
#define BULK_ERASE_CMD 0xC7
#define SEC_ERASE_CMD 0xD8
#define READ_ID 0x9F

#define COMMAND_OFFSET 0 /* FLASH instruction */
#define ADDRESS_1_OFFSET 1 /* MSB byte of address to read or write */
#define ADDRESS_2_OFFSET 2 /* Middle byte of address to read or write */
#define ADDRESS_3_OFFSET 3 /* LSB byte of address to read or write */
#define DATA_OFFSET 4 /* Start of Data for Read/Write */
#define DUMMY_OFFSET 4 /* Dummy byte offset for fast, dual and quad reads */
#define DUMMY_SIZE 1 /* Number of dummy bytes for fast, dual and quad reads */
#define RD_ID_SIZE 4 /* Read ID command + 3 bytes ID response */
#define BULK_ERASE_SIZE 1 /* Bulk Erase command size */
#define SEC_ERASE_SIZE 4 /* Sector Erase command + Sector address */
#define OVERHEAD_SIZE 4 // Command + 3 bytes address

#define SECTOR_SIZE 65536 // Sector size flash
#define PAGE_SIZE 256 // Page size flash

static int qspi_enable_quad(qspi_t* qspi);


qspi_t qspi_handler(XQspiPs* xqspi, uint16_t dev_id) {
	qspi_t qspi = {
		.instance  = xqspi,
		.device_id = dev_id,
	};

	return qspi;
}


int qspi_initialize(qspi_t* qspi) {
    int status;

    qspi->config = XQspiPs_LookupConfig(qspi->device_id);
    if (!qspi->config)
        return XST_NO_DATA;

    status = XQspiPs_CfgInitialize(qspi->instance, qspi->config, qspi->config->BaseAddress);
    if (status != XST_SUCCESS)
        return status;

    XQspiPs_Reset(qspi->instance);

    status = XQspiPs_SelfTest(qspi->instance);
    if (status != XST_SUCCESS)
        return status;

    status = XQspiPs_SetClkPrescaler(qspi->instance, XQSPIPS_CLK_PRESCALE_8);
    if (status != XST_SUCCESS)
        return status;

    status = XQspiPs_SetOptions(qspi->instance, XQSPIPS_FORCE_SSELECT_OPTION | XQSPIPS_MANUAL_START_OPTION | XQSPIPS_HOLD_B_DRIVE_OPTION);
    if (status != XST_SUCCESS)
        return status;

    status = XQspiPs_SetSlaveSelect(qspi->instance);
    if (status != XST_SUCCESS)
        return status;

    qspi_get_flash_id(qspi);

    status = qspi_enable_quad(qspi);
    if (status != XST_SUCCESS)
        return status;

    return XST_SUCCESS;
}


int qspi_get_flash_id(qspi_t* qspi) {
    uint8_t tx_buffer[4] = { [COMMAND_OFFSET] = READ_ID };

    uint8_t rx_buffer[RD_ID_SIZE];
    memset(rx_buffer, 0, sizeof(rx_buffer));

    int status;

    status = XQspiPs_PolledTransfer(qspi->instance, tx_buffer, rx_buffer, RD_ID_SIZE);
    if (status != XST_SUCCESS)
        return status;

    return XST_SUCCESS;
}


int qspi_write(qspi_t* qspi, uint32_t address, uint8_t* buffer, uint32_t length)
{
    uint8_t write_enable_cmd = WRITE_ENABLE_CMD;
    uint8_t read_status_cmd[] = { READ_STATUS_CMD, 0 };
    uint8_t flash_status[2];

    int status = 0;
    int chunks_cnt = length / PAGE_SIZE;
    int tail_size = length % PAGE_SIZE;

    if (tail_size)
        chunks_cnt++;

    for (int i = 0; i < chunks_cnt; i++) {

        status = XQspiPs_PolledTransfer(qspi->instance, &write_enable_cmd, NULL, 1);
        if (status != XST_SUCCESS)
            return status;

        uint8_t tx_buffer[PAGE_SIZE + OVERHEAD_SIZE] = {
            [COMMAND_OFFSET] = WRITE_CMD,
            [ADDRESS_1_OFFSET] = (uint8_t)((address & 0xFF0000) >> 16),
            [ADDRESS_2_OFFSET] = (uint8_t)((address & 0xFF00) >> 8),
            [ADDRESS_3_OFFSET] = (uint8_t)(address & 0xFF),
        };

        int chunk_length = (tail_size && i == chunks_cnt - 1) ? tail_size : PAGE_SIZE;

        memcpy(tx_buffer + OVERHEAD_SIZE, buffer + i * PAGE_SIZE, chunk_length);

        status = XQspiPs_PolledTransfer(qspi->instance, tx_buffer, NULL, chunk_length + OVERHEAD_SIZE);
        if (status != XST_SUCCESS)
            return status;

        while (1) {

            status = XQspiPs_PolledTransfer(qspi->instance, read_status_cmd, flash_status, sizeof(read_status_cmd));
            if (status != XST_SUCCESS)
                return status;

            flash_status[1] |= flash_status[0];
            if ((flash_status[1] & 0x01) == 0)
                break;
        }

        address += chunk_length;
    }

    return XST_SUCCESS;
}

int qspi_read(qspi_t* qspi, uint32_t address, uint8_t* buffer, uint32_t length) {
    uint8_t read_status_cmd[] = { READ_STATUS_CMD, 0 };
    uint8_t flash_status[2];

    int status;

    uint8_t tx_buffer[length + OVERHEAD_SIZE];
    uint8_t rx_buffer[length + OVERHEAD_SIZE];
    memset(tx_buffer, 0, length + OVERHEAD_SIZE);

    tx_buffer[COMMAND_OFFSET] = READ_CMD;
    tx_buffer[ADDRESS_1_OFFSET] = (uint8_t)((address & 0xFF0000) >> 16);
    tx_buffer[ADDRESS_2_OFFSET] = (uint8_t)((address & 0xFF00) >> 8);
    tx_buffer[ADDRESS_3_OFFSET] = (uint8_t)(address & 0xFF);

    status = XQspiPs_PolledTransfer(qspi->instance, tx_buffer, rx_buffer,
    								length + OVERHEAD_SIZE);
    if (status != XST_SUCCESS)
        return status;

    while (1) {

        status = XQspiPs_PolledTransfer(qspi->instance, read_status_cmd, flash_status, sizeof(read_status_cmd));
        if (status != XST_SUCCESS)
            return status;

        flash_status[1] |= flash_status[0];
        if ((flash_status[1] & 0x01) == 0)
            break;
    }

    memcpy(buffer, rx_buffer + OVERHEAD_SIZE, length);

    return XST_SUCCESS;
}


int qspi_enable_quad(qspi_t* qspi) {
    uint8_t write_enable_cmd = WRITE_ENABLE_CMD;
    uint8_t read_status_cmd[] = { READ_STATUS_CMD, 0 };
    uint8_t quad_enable_cmd[] = { WRITE_STATUS_CMD, 0 };
    uint8_t flash_status[2];

    int status;

    status = XQspiPs_PolledTransfer(qspi->instance, read_status_cmd, flash_status, sizeof(read_status_cmd));
    if (status != XST_SUCCESS)
        return status;

    quad_enable_cmd[1] = flash_status[1] | 1 << 6;

    status = XQspiPs_PolledTransfer(qspi->instance, &write_enable_cmd, NULL, 1);
    if (status != XST_SUCCESS)
        return status;

    status = XQspiPs_PolledTransfer(qspi->instance, quad_enable_cmd, NULL, sizeof(quad_enable_cmd));
    if (status != XST_SUCCESS)
        return status;

    while (1) {

        status = XQspiPs_PolledTransfer(qspi->instance, read_status_cmd, flash_status, sizeof(read_status_cmd));
        if (status != XST_SUCCESS)
            return status;

        if ((flash_status[0] == 0x40) && (flash_status[1] == 0x40))
            break;
    }

    return XST_SUCCESS;
}


int qspi_erase(qspi_t* qspi, uint32_t address, uint32_t sectors)
{
    uint8_t write_enable_cmd = WRITE_ENABLE_CMD;
    uint8_t read_status_cmd[] = { READ_STATUS_CMD, 0 };
    uint8_t flash_status[2];

    int status = XST_SUCCESS;

    uint8_t tx_buffer[SEC_ERASE_SIZE];

    for (int i = 0; i < sectors; i++) {
        status = XQspiPs_PolledTransfer(qspi->instance, &write_enable_cmd, NULL, 1);
        if (status != XST_SUCCESS)
            return status;

        tx_buffer[COMMAND_OFFSET] = SEC_ERASE_CMD;
        tx_buffer[ADDRESS_1_OFFSET] = (uint8_t)((address & 0xFF0000) >> 16);
        tx_buffer[ADDRESS_2_OFFSET] = (uint8_t)((address & 0xFF00) >> 8);
        tx_buffer[ADDRESS_3_OFFSET] = (uint8_t)(address & 0xFF);

        status = XQspiPs_PolledTransfer(qspi->instance, tx_buffer, NULL, SEC_ERASE_SIZE);
        if (status != XST_SUCCESS)
            return status;

        while (1) {
            status = XQspiPs_PolledTransfer(qspi->instance, read_status_cmd, flash_status, sizeof(read_status_cmd));
            if (status != XST_SUCCESS)
                return status;

            flash_status[1] |= flash_status[0];
            if ((flash_status[1] & 0x01) == 0)
                break;
        }

        address += SECTOR_SIZE;
    }

    return status;
}
