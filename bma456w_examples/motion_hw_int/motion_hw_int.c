/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include <math.h>
#include "bma456w.h"
#include "common.h"

/*****************************************************************************/
/*!                Global variable                                           */

/*! Structure to define any/no-motion configurations */
struct bma456w_any_no_mot_config any_no_mot = { 0 };

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
 *  @brief This internal API is used to get any/no-motion configurations.
 *
 *  @param[in] bma       : Structure instance of bma4_dev.
 *
 *  @return Status of execution.
 */
static int8_t get_any_no_mot_config(struct bma4_dev *bma);

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
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    /* Variable to store the status of API */
    int8_t rslt;

    /* Sensor initialization configuration */
    struct bma4_dev bma = { 0 };

    /* Variable to store any/no-motion interrupt status */
    uint16_t int_status = 0;

    /* Loop variable */
    uint8_t iteration = 20;

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

    /* Get any-motion and no-motion configurations */
    rslt = get_any_no_mot_config(&bma);
    bma4_error_codes_print_result("get_any_no_mot_config status", rslt);

    /* Hardware interrupt configuration */
    int_line = BMA4_INTR2_MAP;

    rslt = bma4_get_int_pin_config(&pin_config, int_line, &bma);
    bma4_error_codes_print_result("bma4_get_int_pin_config status", rslt);

    rslt = bma456w_map_interrupt(int_line, (BMA456W_ANY_MOT_INT | BMA456W_NO_MOT_INT), BMA4_ENABLE, &bma);
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

    printf("Shake the board for any-motion interrupt whereas do not shake the board for no-motion interrupt\n");

    for (;;)
    {
        if (interrupt_status == 1)
        {
            interrupt_status = 0;

            /* Read interrupt status */
            rslt = bma456w_read_int_status(&int_status, &bma);
            bma4_error_codes_print_result("bma456w_read_int_status", rslt);

            if (rslt == BMA4_OK)
            {
                /* Enters only if the obtained interrupt is any-motion */
                if (int_status & BMA456W_ANY_MOT_INT)
                {
                    printf("Any-motion interrupt occurred\n");
                    iteration--;
                }
                /* Enters only if the obtained interrupt is no-motion */
                else if (int_status & BMA456W_NO_MOT_INT)
                {
                    printf("No-motion interrupt occurred\n");
                    iteration--;
                }

                int_status = 0;

                /* Break out of the loop when iteration has reached zero */
                if (iteration == 0)
                {
                    printf("Iterations are done. Exiting !");
                    break;
                }
            }
        }
    }

    bma4_coines_deinit();

    return rslt;
}

/*!
 *  @brief This internal API is used to get any/no-motion configurations.
 */
static int8_t get_any_no_mot_config(struct bma4_dev *bma)
{
    /* Variable to store the status of API */
    int8_t rslt;

    /* Getting any-motion configuration to get default configuration */
    rslt = bma456w_get_any_mot_config(&any_no_mot, bma);
    bma4_error_codes_print_result("bma456w_get_any_mot_config status", rslt);

    if (rslt == BMA4_OK)
    {
        /*
         * Set the slope threshold:
         *  Interrupt will be generated if the slope of all the axis exceeds the threshold (1 bit = 0.48mG)
         */
        any_no_mot.threshold = 10;

        /*
         * Set the duration for any-motion interrupt:
         *  Duration defines the number of consecutive data points for which threshold condition must be true(1
         * bit =
         * 20ms)
         */
        any_no_mot.duration = 4;

        /* Enabling X, Y, and Z axis for Any-motion feature */
        any_no_mot.axes_en = BMA456W_EN_ALL_AXIS;

        /* Like threshold and duration, we can also change the config of int_bhvr and slope */

        /* Set the threshold and duration configuration */
        rslt = bma456w_set_any_mot_config(&any_no_mot, bma);
        bma4_error_codes_print_result("bma456w_set_any_mot_config status", rslt);

        if (rslt == BMA4_OK)
        {
            /* Getting no-motion configuration to get default configuration */
            rslt = bma456w_get_no_mot_config(&any_no_mot, bma);
            bma4_error_codes_print_result("bma456w_get_no_mot_config status", rslt);

            if (rslt == BMA4_OK)
            {
                /*
                 * Set the slope threshold:
                 *  Interrupt will be generated if the slope of all the axis exceeds the threshold (1 bit = 0.48mG)
                 */
                any_no_mot.threshold = 10;

                /*
                 * Set the duration for no-motion interrupt:
                 * Duration defines the number of consecutive data points for which threshold condition must be
                 * true(1 bit = 20ms)
                 */
                any_no_mot.duration = 4;

                /* Enabling X, Y, and Z axis for no-motion feature */
                any_no_mot.axes_en = BMA456W_EN_ALL_AXIS;

                /* Like threshold and duration, we can also change the config of int_bhvr */

                /* Set the threshold and duration configuration */
                rslt = bma456w_set_no_mot_config(&any_no_mot, bma);
                bma4_error_codes_print_result("bma456w_set_no_mot_config status", rslt);
            }
        }
    }

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
