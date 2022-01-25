/**
 * Copyright (C) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "bma4_defs.h"
#include "common.h"

/******************************************************************************/
/*!                       Macro definitions                                   */

/*! BMA4xy shuttle board ID */
#define BMA4XY_SHUTTLE_ID    UINT16_C(0x141)

/*! Read write length varies based on user requirement */
#define BMA4_READ_WRITE_LEN  UINT8_C(46)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES platform
 */
BMA4_INTF_RET_TYPE bma4_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return coines_read_i2c(dev_addr, reg_addr, reg_data, (uint16_t)len);
}

/*!
 * I2C write function map to COINES platform
 */
BMA4_INTF_RET_TYPE bma4_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return coines_write_i2c(dev_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
}

/*!
 * SPI read function map to COINES platform
 */
BMA4_INTF_RET_TYPE bma4_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return coines_read_spi(dev_addr, reg_addr, reg_data, (uint16_t)len);
}

/*!
 * SPI write function map to COINES platform
 */
BMA4_INTF_RET_TYPE bma4_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return coines_write_spi(dev_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
}

/*!
 * Delay function map to COINES platform
 */
void bma4_delay_us(uint32_t period, void *intf_ptr)
{
    coines_delay_usec(period);
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *  Also to initialize coines platform
 */
int8_t bma4_interface_init(struct bma4_dev *bma, uint8_t intf, uint8_t variant)
{
    int8_t rslt = BMA4_OK;

    if (bma != NULL)
    {
        int16_t result = coines_open_comm_intf(COINES_COMM_INTF_USB);
        struct coines_board_info board_info;
        if (result < COINES_SUCCESS)
        {
            printf(
                "\n Unable to connect with Application Board ! \n" " 1. Check if the board is connected and powered on. \n" " 2. Check if Application Board USB driver is installed. \n"
                " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
            exit(result);
        }

        rslt = coines_get_board_info(&board_info);

    #if defined(PC)
        setbuf(stdout, NULL);
    #endif

        if (rslt == COINES_SUCCESS)
        {
            if ((board_info.shuttle_id != BMA4XY_SHUTTLE_ID))
            {
                printf("! Warning invalid sensor shuttle \n ," "This application will not support this sensor \n");
                exit(COINES_E_FAILURE);
            }
        }

        coines_set_shuttleboard_vdd_vddio_config(0, 0);
        coines_delay_usec(10000);

        /* Bus configuration : I2C */
        if (intf == BMA4_I2C_INTF)
        {
            printf("I2C Interface \n");

            /* To initialize the user I2C function */
            dev_addr = BMA4_I2C_ADDR_PRIMARY;
            bma->intf = BMA4_I2C_INTF;
            bma->bus_read = bma4_i2c_read;
            bma->bus_write = bma4_i2c_write;

            /* SDO to Ground */
            coines_set_pin_config(COINES_SHUTTLE_PIN_22, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);

            /* Make CSB pin HIGH */
            coines_set_pin_config(COINES_SHUTTLE_PIN_21, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
            coines_delay_msec(100);

            coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
        }
        /* Bus configuration : SPI */
        else if (intf == BMA4_SPI_INTF)
        {
            printf("SPI Interface \n");

            /* To initialize the user SPI function */
            dev_addr = COINES_SHUTTLE_PIN_7;
            bma->intf = BMA4_SPI_INTF;
            bma->bus_read = bma4_spi_read;
            bma->bus_write = bma4_spi_write;
            coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_7_5_MHZ, COINES_SPI_MODE0);
        }

        /* Assign variant information */
        bma->variant = variant;

        /* Assign device address to interface pointer */
        bma->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        bma->delay_us = bma4_delay_us;

        /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
        bma->read_write_len = BMA4_READ_WRITE_LEN;

        coines_delay_msec(100);

        coines_set_shuttleboard_vdd_vddio_config(3300, 3300);

        coines_delay_msec(200);
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;

}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bma4_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMA4_OK)
    {
        printf("%s\t", api_name);
        if (rslt == BMA4_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer\r\n", rslt);
        }
        else if (rslt == BMA4_E_COM_FAIL)
        {
            printf("Error [%d] : Communication failure\r\n", rslt);
        }
        else if (rslt == BMA4_E_CONFIG_STREAM_ERROR)
        {
            printf("Error [%d] : Invalid configuration stream\r\n", rslt);
        }
        else if (rslt == BMA4_E_SELF_TEST_FAIL)
        {
            printf("Error [%d] : Self test failed\r\n", rslt);
        }
        else if (rslt == BMA4_E_INVALID_SENSOR)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else if (rslt == BMA4_E_OUT_OF_RANGE)
        {
        	printf("Error [%d] : Out of Range\r\n", rslt);
        }
        else if (rslt == BMA4_E_AVG_MODE_INVALID_CONF)
        {
        	printf("Error [%d] : Invalid bandwidth and ODR combination in Accel Averaging mode\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}

/*!
 *  @brief Deinitializes coines platform
 *
 *  @return void.
 */
void bma4_coines_deinit(void)
{
    fflush(stdout);

    coines_set_shuttleboard_vdd_vddio_config(0, 0);
    coines_delay_msec(1000);

    /* Coines interface reset */
    coines_soft_reset();
    coines_delay_msec(1000);
    coines_close_comm_intf(COINES_COMM_INTF_USB);
}
