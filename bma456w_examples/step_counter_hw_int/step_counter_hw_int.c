/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include <math.h>
#include "bma456w.h"
#include "common.h"

/******************************************************************************/
/*!           Global variable Declaration                                     */

volatile uint8_t interrupt_status = 0;

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
    interrupt_status = 1;
}

/******************************************************************************/
/*!            Function                                                       */

/* This function starts the execution of program. */
int main(void)
{
    /* Variable to store the status of API */
    int8_t rslt;

    /* Sensor initialization configuration */
    struct bma4_dev bma = { 0 };

    /* Variable to store step counter interrupt status */
    uint16_t int_status = 0;

    /* Variable to store step counter status */
    uint32_t step_out = 0;

    struct bma4_int_pin_config pin_config = { 0 };
    uint8_t board = 0;

    /* Variable for interrupt line selection*/
    uint8_t int_line;

    /* Interface reference is given as a parameter
     *         For I2C : BMA4_I2C_INTF
     *         For SPI : BMA4_SPI_INTF
     * Variant information given as parameter - BMA45X_VARIANT
     */
    rslt = bma4_interface_init(&bma, BMA4_SPI_INTF, BMA45X_VARIANT);
    bma4_error_codes_print_result("bma4_interface_init", rslt);

    /* Sensor initialization */
    rslt = bma456w_init(&bma);
    bma4_error_codes_print_result("bma456w_init status", rslt);

    /* Upload the configuration file to enable the features of the sensor. */
    rslt = bma456w_write_config_file(&bma);
    bma4_error_codes_print_result("bma456w_write_config status", rslt);

    /* Enable accelerometer to perform feature testing */
    rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    rslt = bma456w_feature_enable(BMA456W_STEP_CNTR, BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma456w_feature_enable status", rslt);

    /* Setting watermark level 1, the output step resolution is 20 steps.
     * Eg: 1 means, 1 * 20 = 20. Every 20 steps once output triggers
     */
    rslt = bma456w_step_counter_set_watermark(1, &bma);
    bma4_error_codes_print_result("bma456w_step_counter_set_watermark status", rslt);

    /* Hardware interrupt configuration */
    int_line = BMA4_INTR2_MAP;

    rslt = bma4_get_int_pin_config(&pin_config, int_line, &bma);
    bma4_error_codes_print_result("bma4_get_int_pin_config status", rslt);

    rslt = bma456w_map_interrupt(int_line, BMA456W_STEP_CNTR_INT, BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma456w_map_interrupt status", rslt);

    pin_config.edge_ctrl = BMA4_EDGE_TRIGGER;
    pin_config.output_en = BMA4_OUTPUT_ENABLE;
    pin_config.lvl = BMA4_ACTIVE_HIGH;
    pin_config.od = BMA4_PUSH_PULL;
    pin_config.input_en = BMA4_INPUT_DISABLE;

    rslt = bma4_set_int_pin_config(&pin_config, int_line, &bma);
    bma4_error_codes_print_result("bma4_set_int_pin_config status", rslt);

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

    printf("Step counter feature is enabled\n");

    printf("Step counter watermark level is 1 (Output resolution is 20 steps)\n");

    printf("Move the board in steps for greater than 3 seconds\n");

    /* Loop over until step counter interrupt occurs */
    for (;;)
    {
        if (interrupt_status == 1)
        {
            interrupt_status = 0;

            /* Read the interrupt status (After 20 steps, generates interrupt) */
            rslt = bma456w_read_int_status(&int_status, &bma);
            bma4_error_codes_print_result("bma456w_read_int_status", rslt);

            if ((BMA4_OK == rslt) && (int_status & BMA456W_STEP_CNTR_INT))
            {
                printf("Step counter interrupt received when watermark level is reached (20 steps)\n");

                /* Get the step counter output after reset */
                rslt = bma456w_step_counter_output(&step_out, &bma);
                bma4_error_codes_print_result("bma456w_step_counter_output status", rslt);

                break;
            }
        }
    }

    printf("The step counter output is %lu \n", (long unsigned int)step_out);

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
