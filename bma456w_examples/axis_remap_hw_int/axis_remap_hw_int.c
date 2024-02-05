/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "coines.h"
#include "bma456w.h"
#include "common.h"

/******************************************************************************/
/*!           Global variable Declaration                                     */

volatile uint8_t drdy_int_status = 0;

/******************************************************************************/
/*!                Macro definition                                           */

/*! APP20 Board number */
#define BOARD_MCU_APP20  UINT8_C(0x03)

/*! APP30 Board number */
#define BOARD_MCU_APP30  UINT8_C(0x05)

/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 * @brief This function gets board information
 *
 * @param[out] board       : Board value to determine as APP2.0 or APP3.0
 */
static void get_board_info(uint8_t *board);

/*!
 * @brief This internal API is used to set the interrupt status
 */
static void interrupt_callback(uint32_t param1, uint32_t param2)
{
    (void)param1;
    (void)param2;
    drdy_int_status = 1;
}

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    int8_t rslt;
    struct bma4_dev bma4;
    struct bma4_remap remap_data = { 0 };
    struct bma4_accel accel = { 0 };
    struct bma4_accel_config accel_conf = { 0 };
    uint16_t int_status = 0;
    struct bma4_int_pin_config pin_config = { 0 };

    /* Variable for interrupt line selection*/
    uint8_t int_line;

    uint8_t intf;
    uint8_t board = 0;

    char data_array[13][11] =
    { { 0 }, { "BMA4_X" }, { "BMA4_Y" }, { 0 }, { "BMA4_Z" }, { 0 }, { 0 }, { 0 }, { 0 }, { "BMA4_NEG_X" },
      { "BMA4_NEG_Y" }, { 0 }, { "BMA4_NEG_Z" } };

    /* intf: Interface reference is given as a parameter
     *         For I2C : BMA4_I2C_INTF
     *         For SPI : BMA4_SPI_INTF
     * Variant information given as parameter - BMA45X_VARIANT
     */

    intf = BMA4_I2C_INTF;
    rslt = bma4_interface_init(&bma4, intf, BMA45X_VARIANT);
    bma4_error_codes_print_result("bma4_interface_init", rslt);

    /* Sensor initialization */
    rslt = bma456w_init(&bma4);
    bma4_error_codes_print_result("bma456w_init", rslt);

    /* Upload the configuration file to enable the features of the sensor. */
    rslt = bma456w_write_config_file(&bma4);
    bma4_error_codes_print_result("bma456w_write_config_file", rslt);

    /* Accelerometer configuration Setting */
    /* Output data Rate */
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_50HZ;

    /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G) */
    accel_conf.range = BMA4_ACCEL_RANGE_2G;

    /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
     * if it is set to 2, then 2^(bandwidth parameter) samples
     * are averaged, resulting in 4 averaged samples
     * Note1 : For more information, refer the datasheet.
     * Note2 : A higher number of averaged samples will result in a less noisier signal, but
     * this has an adverse effect on the power consumed.
     */
    accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;

    /* Enable the filter performance mode where averaging of samples
     * will be done based on above set bandwidth and ODR.
     * There are two modes
     *  0 -> Averaging samples (Default)
     *  1 -> No averaging
     * For more info on No Averaging mode refer datasheet.
     */
    accel_conf.perf_mode = BMA4_CIC_AVG_MODE;

    /* Set the accel configurations */
    rslt = bma4_set_accel_config(&accel_conf, &bma4);
    bma4_error_codes_print_result("bma4_set_accel_config status", rslt);

    /* Hardware interrupt configuration */
    int_line = BMA4_INTR2_MAP;

    rslt = bma456w_map_interrupt(int_line, BMA4_DATA_RDY_INT, BMA4_ENABLE, &bma4);
    bma4_error_codes_print_result("bma456w_map_interrupt status", rslt);

    /* Get board information */
    get_board_info(&board);

    /*
     * Attach interrupt based on board
     */
    if (board == BOARD_MCU_APP20)
    {
        switch (int_line)
        {
            case BMA4_INTR1_MAP:
                coines_attach_interrupt(COINES_SHUTTLE_PIN_20, interrupt_callback, COINES_PIN_INTERRUPT_RISING_EDGE);
                break;

            case BMA4_INTR2_MAP:
                coines_attach_interrupt(COINES_SHUTTLE_PIN_21, interrupt_callback, COINES_PIN_INTERRUPT_RISING_EDGE);
                break;
            default:
                break;
        }
    }

#if !defined(MCU_APP20)
    else if (board == BOARD_MCU_APP30)
    {
        switch (int_line)
        {
            case BMA4_INTR1_MAP:
                coines_attach_interrupt(COINES_MINI_SHUTTLE_PIN_1_6,
                                        interrupt_callback,
                                        COINES_PIN_INTERRUPT_RISING_EDGE);
                break;

            case BMA4_INTR2_MAP:
                coines_attach_interrupt(COINES_MINI_SHUTTLE_PIN_1_7,
                                        interrupt_callback,
                                        COINES_PIN_INTERRUPT_RISING_EDGE);
                break;

            default:
                break;
        }
    }
#endif

    /* Latch mode to be set for I2C to read Drdy interrupt with 100kHz Speed */
    switch (intf)
    {
        case BMA4_I2C_INTF:
            rslt = bma4_set_interrupt_mode(BMA4_LATCH_MODE, &bma4);
            bma4_error_codes_print_result("bma4_set_interrupt_mode status", rslt);
            break;
        default:
            break;
    }

    rslt = bma4_get_int_pin_config(&pin_config, int_line, &bma4);
    bma4_error_codes_print_result("bma4_get_int_pin_config status", rslt);

    pin_config.edge_ctrl = BMA4_EDGE_TRIGGER;
    pin_config.output_en = BMA4_OUTPUT_ENABLE;
    pin_config.lvl = BMA4_ACTIVE_HIGH;
    pin_config.od = BMA4_PUSH_PULL;
    pin_config.input_en = BMA4_INPUT_DISABLE;

    rslt = bma4_set_int_pin_config(&pin_config, int_line, &bma4);
    bma4_error_codes_print_result("bma4_set_int_pin_config status", rslt);

    /* NOTE : Enable accel after set of configurations */
    rslt = bma4_set_accel_enable(1, &bma4);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    printf("\nAXIS_REMAP_FUNC_TEST 1\n");
    printf("Get sensor data of re-mapped axes\n");

    rslt = bma456w_get_remap_axes(&remap_data, &bma4);
    bma4_error_codes_print_result("bma456w_get_remap_axes", rslt);

    printf("Re-mapped x value = %s\n", data_array[remap_data.x]);
    printf("Re-mapped y value = %s\n", data_array[remap_data.y]);
    printf("Re-mapped z value = %s\n", data_array[remap_data.z]);

    printf("Expected Re-mapped x value = BMA4_X\n");
    printf("Expected Re-mapped y value = BMA4_Y\n");
    printf("Expected Re-mapped z value = BMA4_Z\n");

    if ((remap_data.x == BMA4_X) && (remap_data.y == BMA4_Y) && (remap_data.z == BMA4_Z))
    {
        printf(">> PASS\n");
    }
    else
    {
        printf(">> FAIL\n");
    }

    printf("Print mapped data\n");

    for (;;)
    {
        if (drdy_int_status == 1)
        {
            drdy_int_status = 0;

            /* Read interrupt status */
            rslt = bma456w_read_int_status(&int_status, &bma4);
            bma4_error_codes_print_result("bma456w_read_int_status", rslt);

            /* Filtering only the accel data ready interrupt */
            if ((rslt == BMA4_OK) && (int_status & BMA4_ACCEL_DATA_RDY_INT))
            {
                rslt = bma4_read_accel_xyz(&accel, &bma4);
                bma4_error_codes_print_result("bma4_read_accel_xyz", rslt);

                printf("Accel :: X = %d Y = %d Z = %d\n", accel.x, accel.y, accel.z);

                break;
            }
        }
    }

    printf("\nAXIS_REMAP_FUNC_TEST 2\n");
    printf("Get sensor data of re-mapped axes\n");

    remap_data.x = BMA4_NEG_Y;
    remap_data.y = BMA4_Z;
    remap_data.z = BMA4_NEG_X;

    rslt = bma456w_set_remap_axes(&remap_data, &bma4);
    bma4_error_codes_print_result("bma456w_set_remap_axes", rslt);

    if (rslt == BMA4_OK)
    {
        rslt = bma456w_get_remap_axes(&remap_data, &bma4);
        bma4_error_codes_print_result("bma456w_get_remap_axes", rslt);

        if (rslt == BMA4_OK)
        {
            printf("Re-mapped x value = %s\n", data_array[remap_data.x]);
            printf("Re-mapped y value = %s\n", data_array[remap_data.y]);
            printf("Re-mapped z value = %s\n", data_array[remap_data.z]);
        }

        printf("Expected Re-mapped x value = BMA4_NEG_Y\n");
        printf("Expected Re-mapped y value = BMA4_Z\n");
        printf("Expected Re-mapped z value = BMA4_NEG_X\n");

        if ((remap_data.x == BMA4_NEG_Y) && (remap_data.y == BMA4_Z) && (remap_data.z == BMA4_NEG_X))
        {
            printf(">> PASS\n");
        }
        else
        {
            printf(">> FAIL\n");
        }
    }

    printf("Print mapped data\n");

    for (;;)
    {
        if (drdy_int_status == 1)
        {
            drdy_int_status = 0;

            /* Read interrupt status */
            rslt = bma456w_read_int_status(&int_status, &bma4);
            bma4_error_codes_print_result("bma456w_read_int_status", rslt);

            /* Filtering only the accel data ready interrupt */
            if ((rslt == BMA4_OK) && (int_status & BMA4_ACCEL_DATA_RDY_INT))
            {
                rslt = bma4_read_accel_xyz(&accel, &bma4);
                bma4_error_codes_print_result("bma4_read_accel_xyz", rslt);

                printf("Accel :: X = %d Y = %d Z = %d\n", accel.x, accel.y, accel.z);

                break;
            }
        }
    }

    printf("\nAXIS_REMAP_FUNC_TEST 3\n");
    printf("Get sensor data of re-mapped axes - 2nd combination\n");

    remap_data.x = BMA4_NEG_Z;
    remap_data.y = BMA4_NEG_X;
    remap_data.z = BMA4_Y;

    rslt = bma456w_set_remap_axes(&remap_data, &bma4);
    bma4_error_codes_print_result("bma456w_set_remap_axes", rslt);

    if (rslt == BMA4_OK)
    {
        rslt = bma456w_get_remap_axes(&remap_data, &bma4);
        bma4_error_codes_print_result("bma456w_get_remap_axes", rslt);

        if (rslt == BMA4_OK)
        {
            printf("Re-mapped x value = %s\n", data_array[remap_data.x]);
            printf("Re-mapped y value = %s\n", data_array[remap_data.y]);
            printf("Re-mapped z value = %s\n", data_array[remap_data.z]);
        }

        printf("Expected Re-mapped x value = BMA4_NEG_Z\n");
        printf("Expected Re-mapped y value = BMA4_NEG_X\n");
        printf("Expected Re-mapped z value = BMA4_Y\n");

        if ((remap_data.x == BMA4_NEG_Z) && (remap_data.y == BMA4_NEG_X) && (remap_data.z == BMA4_Y))
        {
            printf(">> PASS\n");
        }
        else
        {
            printf(">> FAIL\n");
        }
    }

    printf("Print mapped data\n");

    for (;;)
    {
        if (drdy_int_status == 1)
        {
            drdy_int_status = 0;

            /* Read interrupt status */
            rslt = bma456w_read_int_status(&int_status, &bma4);
            bma4_error_codes_print_result("bma456w_read_int_status", rslt);

            /* Filtering only the accel data ready interrupt */
            if ((rslt == BMA4_OK) && (int_status & BMA4_ACCEL_DATA_RDY_INT))
            {
                rslt = bma4_read_accel_xyz(&accel, &bma4);
                bma4_error_codes_print_result("bma4_read_accel_xyz", rslt);

                printf("Accel :: X = %d Y = %d Z = %d\n", accel.x, accel.y, accel.z);

                break;
            }
        }
    }

    printf("\nAXIS_REMAP_FUNC_TEST 4\n");
    printf("Get sensor data of re-mapped axes - 3rd combination\n");

    remap_data.x = BMA4_Y;
    remap_data.y = BMA4_Z;
    remap_data.z = BMA4_X;

    rslt = bma456w_set_remap_axes(&remap_data, &bma4);
    bma4_error_codes_print_result("bma456w_set_remap_axes", rslt);

    if (rslt == BMA4_OK)
    {
        rslt = bma456w_get_remap_axes(&remap_data, &bma4);
        bma4_error_codes_print_result("bma456w_get_remap_axes", rslt);

        if (rslt == BMA4_OK)
        {
            printf("Re-mapped x value = %s\n", data_array[remap_data.x]);
            printf("Re-mapped y value = %s\n", data_array[remap_data.y]);
            printf("Re-mapped z value = %s\n", data_array[remap_data.z]);
        }

        printf("Expected Re-mapped x value = BMA4_Y\n");
        printf("Expected Re-mapped y value = BMA4_Z\n");
        printf("Expected Re-mapped z value = BMA4_X\n");

        if ((remap_data.x == BMA4_Y) && (remap_data.y == BMA4_Z) && (remap_data.z == BMA4_X))
        {
            printf(">> PASS\n");
        }
        else
        {
            printf(">> FAIL\n");
        }
    }

    printf("Print mapped data\n");

    for (;;)
    {
        if (drdy_int_status == 1)
        {
            drdy_int_status = 0;

            /* Read interrupt status */
            rslt = bma456w_read_int_status(&int_status, &bma4);
            bma4_error_codes_print_result("bma456w_read_int_status", rslt);

            /* Filtering only the accel data ready interrupt */
            if ((rslt == BMA4_OK) && (int_status & BMA4_ACCEL_DATA_RDY_INT))
            {
                rslt = bma4_read_accel_xyz(&accel, &bma4);
                bma4_error_codes_print_result("bma4_read_accel_xyz", rslt);

                printf("Accel :: X = %d Y = %d Z = %d\n", accel.x, accel.y, accel.z);

                break;
            }
        }
    }

    printf("\nAXIS_REMAP_FUNC_TEST 5\n");
    printf("Get sensor data of re-mapped axes - 4th combination\n");

    remap_data.x = BMA4_NEG_X;
    remap_data.y = BMA4_NEG_Y;
    remap_data.z = BMA4_NEG_Z;

    rslt = bma456w_set_remap_axes(&remap_data, &bma4);
    bma4_error_codes_print_result("bma456w_set_remap_axes", rslt);

    if (rslt == BMA4_OK)
    {
        rslt = bma456w_get_remap_axes(&remap_data, &bma4);
        bma4_error_codes_print_result("bma456w_get_remap_axes", rslt);

        if (rslt == BMA4_OK)
        {
            printf("Re-mapped x value = %s\n", data_array[remap_data.x]);
            printf("Re-mapped y value = %s\n", data_array[remap_data.y]);
            printf("Re-mapped z value = %s\n", data_array[remap_data.z]);
        }

        printf("Expected Re-mapped x value = BMA4_NEG_X\n");
        printf("Expected Re-mapped y value = BMA4_NEG_Y\n");
        printf("Expected Re-mapped z value = BMA4_NEG_Z\n");

        if ((remap_data.x == BMA4_NEG_X) && (remap_data.y == BMA4_NEG_Y) && (remap_data.z == BMA4_NEG_Z))
        {
            printf(">> PASS\n");
        }
        else
        {
            printf(">> FAIL\n");
        }
    }

    printf("Print mapped data\n");

    for (;;)
    {
        if (drdy_int_status == 1)
        {
            drdy_int_status = 0;

            /* Read interrupt status */
            rslt = bma456w_read_int_status(&int_status, &bma4);
            bma4_error_codes_print_result("bma456w_read_int_status", rslt);

            /* Filtering only the accel data ready interrupt */
            if ((rslt == BMA4_OK) && (int_status & BMA4_ACCEL_DATA_RDY_INT))
            {
                rslt = bma4_read_accel_xyz(&accel, &bma4);
                bma4_error_codes_print_result("bma4_read_accel_xyz", rslt);

                printf("Accel :: X = %d Y = %d Z = %d\n", accel.x, accel.y, accel.z);

                break;
            }
        }
    }

    bma4_coines_deinit();

    return rslt;
}

/*!
 * @brief This function gets board information
 */
static void get_board_info(uint8_t *board)
{
    struct coines_board_info board_info;
    int16_t result;

    result = coines_get_board_info(&board_info);

    if (result == COINES_SUCCESS)
    {
        (*board) = board_info.board;
    }
}
