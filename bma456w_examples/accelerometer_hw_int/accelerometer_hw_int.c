/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "bma456w.h"
#include "common.h"
#include "coines.h"

/******************************************************************************/
/*!           Global variable Declaration                                     */

volatile uint8_t drdy_int_status = 0;

/******************************************************************************/
/*!                Macro definition                                           */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH       (9.80665f)

/*! Macro that holds the total number of accel x,y and z axes sample counts to be printed */
#define ACCEL_SAMPLE_COUNT  UINT8_C(100)

/*! APP20 Board number */
#define BOARD_MCU_APP20     UINT8_C(0x03)

/*! APP30 Board number */
#define BOARD_MCU_APP30     UINT8_C(0x05)

/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 *  @brief This internal API converts raw sensor values(LSB) to meters per seconds square.
 *
 *  @param[in] val       : Raw sensor value.
 *  @param[in] g_range   : Accel Range selected (2G, 4G, 8G, 16G).
 *  @param[in] bit_width : Resolution of the sensor.
 *
 *  @return Accel values in meters per second square.
 *
 */
static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width);

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
    /* Variable to store the status of API */
    int8_t rslt;

    /* Sensor initialization configuration */
    struct bma4_dev bma = { 0 };

    /* Variable to store accel data ready interrupt status */
    uint16_t int_status = 0;

    /* Variable that holds the accelerometer sample count */
    uint8_t n_data = 1;

    struct bma4_accel sens_data = { 0 };
    float x = 0, y = 0, z = 0;
    struct bma4_accel_config accel_conf = { 0 };

    struct bma4_int_pin_config pin_config = { 0 };

    /* Variable for interrupt line selection */
    uint8_t int_line;

    uint8_t intf;
    uint8_t board = 0;

    /* intf: Interface reference is given as a parameter
     *         For I2C : BMA4_I2C_INTF
     *         For SPI : BMA4_SPI_INTF
     * Variant information given as parameter - BMA45X_VARIANT
     */

    intf = BMA4_I2C_INTF;
    rslt = bma4_interface_init(&bma, intf, BMA45X_VARIANT);
    bma4_error_codes_print_result("bma4_interface_init", rslt);

    /* Sensor initialization */
    rslt = bma456w_init(&bma);
    bma4_error_codes_print_result("bma456w_init status", rslt);

    /* Accelerometer configuration settings */
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
    rslt = bma4_set_accel_config(&accel_conf, &bma);
    bma4_error_codes_print_result("bma4_set_accel_config status", rslt);

    /* Hardware interrupt configuration */
    int_line = BMA4_INTR1_MAP;

    rslt = bma456w_map_interrupt(int_line, BMA4_DATA_RDY_INT, BMA4_ENABLE, &bma);
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
            rslt = bma4_set_interrupt_mode(BMA4_LATCH_MODE, &bma);
            bma4_error_codes_print_result("bma4_set_interrupt_mode status", rslt);
            break;
        default:
            break;
    }

    rslt = bma4_get_int_pin_config(&pin_config, int_line, &bma);
    bma4_error_codes_print_result("bma4_get_int_pin_config status", rslt);

    pin_config.edge_ctrl = BMA4_EDGE_TRIGGER;
    pin_config.output_en = BMA4_OUTPUT_ENABLE;
    pin_config.lvl = BMA4_ACTIVE_HIGH;
    pin_config.od = BMA4_PUSH_PULL;
    pin_config.input_en = BMA4_INPUT_DISABLE;

    rslt = bma4_set_int_pin_config(&pin_config, int_line, &bma);
    bma4_error_codes_print_result("bma4_set_int_pin_config status", rslt);

    /* NOTE : Enable accel after set of configurations */
    rslt = bma4_set_accel_enable(1, &bma);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    printf("Data, Acc_Raw_X, Acc_Raw_Y, Acc_Raw_Z, Acc_ms2_X, Acc_ms2_Y, Acc_ms2_Z\n");

    for (;;)
    {
        if (drdy_int_status == 1)
        {
            drdy_int_status = 0;

            /* Read interrupt status */
            rslt = bma456w_read_int_status(&int_status, &bma);
            bma4_error_codes_print_result("bma456w_read_int_status", rslt);

            /* Filtering only the accel data ready interrupt */
            if ((rslt == BMA4_OK) && (int_status & BMA4_ACCEL_DATA_RDY_INT))
            {
                /* Read the accel x, y, z data */
                rslt = bma4_read_accel_xyz(&sens_data, &bma);
                bma4_error_codes_print_result("bma4_read_accel_xyz status", rslt);

                if (rslt == BMA4_OK)
                {

                    /* Converting lsb to meter per second squared for 16 bit resolution at 2G range */
                    x = lsb_to_ms2(sens_data.x, (float)2, bma.resolution);
                    y = lsb_to_ms2(sens_data.y, (float)2, bma.resolution);
                    z = lsb_to_ms2(sens_data.z, (float)2, bma.resolution);

                    /* Print the data in m/s2 */
                    printf("%d, %d, %d, %d, %4.2f, %4.2f, %4.2f\n",
                           n_data,
                           sens_data.x,
                           sens_data.y,
                           sens_data.z,
                           x,
                           y,
                           z);
                }

                /* Increment the count that determines the number of samples to be printed */
                n_data++;

                /* When the count reaches more than ACCEL_SAMPLE_COUNT, break and exit the loop */
                if (n_data > ACCEL_SAMPLE_COUNT)
                {
                    break;
                }
            }
        }
    }

    bma4_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API converts raw sensor values(LSB) to meters per seconds square.
 */
static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
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
