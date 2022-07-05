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

    /* Variable to store step activity interrupt status */
    uint16_t int_status = 0;

    /* Variable to hold iteration value */
    uint8_t loop = 1;

    /* Variable to store activity status */
    uint8_t activity_output = 0;

    struct bma4_int_pin_config pin_config = { 0 };
    uint8_t board = 0;

    /* Variable for interrupt line selection*/
    uint8_t int_line;

    /* Interface reference is given as a parameter
     *         For I2C : BMA4_I2C_INTF
     *         For SPI : BMA4_SPI_INTF
     * Variant information given as parameter - BMA45X_VARIANT
     */
    rslt = bma4_interface_init(&bma, BMA4_I2C_INTF, BMA45X_VARIANT);
    bma4_error_codes_print_result("bma4_interface_init", rslt);

    /* Sensor initialization */
    rslt = bma456w_init(&bma);
    bma4_error_codes_print_result("bma456w_init status", rslt);

    /* Upload the configuration file to enable the features of the sensor. */
    rslt = bma456w_write_config_file(&bma);
    bma4_error_codes_print_result("bma456w_write_config status", rslt);

    /* Enable the accelerometer */
    rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    if (rslt == BMA4_OK)
    {
        /* Setting watermark level 1, the output step resolution is 20 steps.
         * Eg: 1 means, 1 * 20 = 20. Every 20 steps once output triggers
         */
        rslt = bma456w_step_counter_set_watermark(1, &bma);
        bma4_error_codes_print_result("bma456w_step_counter_set_watermark status", rslt);

        if (rslt == BMA4_OK)
        {
            /* Enabling step detector feature */
            rslt = bma456w_feature_enable(BMA456W_STEP_ACT, BMA4_ENABLE, &bma);
            bma4_error_codes_print_result("bma456w_feature_enable status", rslt);
        }
    }

    if (rslt == BMA4_OK)
    {
        /* Hardware interrupt configuration */
        int_line = BMA4_INTR1_MAP;

        rslt = bma4_get_int_pin_config(&pin_config, int_line, &bma);
        bma4_error_codes_print_result("bma4_get_int_pin_config status", rslt);

        rslt = bma456w_map_interrupt(int_line, BMA456W_ACTIVITY_INT, BMA4_ENABLE, &bma);
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
                    coines_attach_interrupt(COINES_SHUTTLE_PIN_20, interrupt_callback,
                                            COINES_PIN_INTERRUPT_RISING_EDGE);
                    break;

                case BMA4_INTR2_MAP:
                    coines_attach_interrupt(COINES_SHUTTLE_PIN_21, interrupt_callback,
                                            COINES_PIN_INTERRUPT_RISING_EDGE);
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
    }

    if (rslt == BMA4_OK)
    {
        printf("Move the board in steps to perform step activity\n");

        while (loop <= 5)
        {
            if (interrupt_status == 1)
            {
                interrupt_status = 0;

                /* Read interrupt status */
                rslt = bma456w_read_int_status(&int_status, &bma);
                bma4_error_codes_print_result("bma456w_read_int_status", rslt);

                /* Filtering only the activity interrupt */
                if ((rslt == BMA4_OK) && (int_status & BMA456W_ACTIVITY_INT))
                {
                    printf("\nIteration : %d\n", loop);

                    /* Get step activity output */
                    rslt = bma456w_activity_output(&activity_output, &bma);
                    bma4_error_codes_print_result("bma456w_activity_output status", rslt);

                    if (rslt == BMA4_OK)
                    {
                        printf("The Activity output is %d\n", activity_output);

                        switch (activity_output)
                        {
                            case BMA456W_USER_STATIONARY:
                                printf("User state is stationary\n");
                                break;
                            case BMA456W_USER_WALKING:
                                printf("User state is walking\n");
                                break;
                            case BMA456W_USER_RUNNING:
                                printf("User state is running\n");
                                break;
                            case BMA456W_STATE_INVALID:
                                printf("User state is invalid state\n");
                                break;
                            default:
                                break;
                        }

                        loop++;
                    }
                }

                int_status = 0;
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
