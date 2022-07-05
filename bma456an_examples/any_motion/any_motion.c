/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include "bma456_an.h"
#include "common.h"
#include "coines.h"

/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 *  @brief This internal API is used to get any-motion configurations.
 *
 *  @param[in] bma       : Structure instance of bma4_dev.
 *
 *  @return Status of execution.
 */
static int8_t get_any_no_mot_config(struct bma4_dev *bma);

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

    /* Interface reference is given as a parameter
     *         For I2C : BMA4_I2C_INTF
     *         For SPI : BMA4_SPI_INTF
     * Variant information given as parameter - BMA45X_VARIANT
     */
    rslt = bma4_interface_init(&bma, BMA4_I2C_INTF, BMA45X_VARIANT);
    bma4_error_codes_print_result("bma4_interface_init", rslt);

    /* Sensor initialization */
    rslt = bma456_an_init(&bma);
    bma4_error_codes_print_result("bma456_an_init status", rslt);

    /* Upload the configuration file to enable the features of the sensor. */
    rslt = bma456_an_write_config_file(&bma);
    bma4_error_codes_print_result("bma456_an_write_config status", rslt);

    /* Enable the accelerometer */
    rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    /* Map the interrupt 1 for any-motion */
    rslt = bma456_an_map_interrupt(BMA4_INTR1_MAP, BMA456_AN_ANY_MOT_INT, BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma456_an_map_interrupt status", rslt);

    /* Get any-motion configurations */
    rslt = get_any_no_mot_config(&bma);
    bma4_error_codes_print_result("get_any_no_mot_config status", rslt);

    printf("Shake the board for any-motion interrupt\n");

    for (;;)
    {
        /* Read interrupt status */
        rslt = bma456_an_read_int_status(&int_status, &bma);
        bma4_error_codes_print_result("bma456_an_read_int_status", rslt);

        if (rslt == BMA4_OK)
        {
            /* Enters only if the obtained interrupt is any-motion */
            if (int_status & BMA456_AN_ANY_MOT_INT)
            {
                printf("Any-motion interrupt occurred\n");
                break;
            }
        }
    }

    bma4_coines_deinit();

    return rslt;
}

/*!
 *  @brief This internal API is used to get any-motion configurations.
 */
static int8_t get_any_no_mot_config(struct bma4_dev *bma)
{
    /* Variable to store the status of API */
    int8_t rslt;

    /*! Structure to define any/no-motion configurations */
    struct bma456_an_any_no_mot_config any_no_mot = { 0 };

    /* Getting any-motion configuration to get default configuration */
    rslt = bma456_an_get_any_mot_config(&any_no_mot, bma);
    bma4_error_codes_print_result("bma456_an_get_any_mot_config status", rslt);

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
        any_no_mot.axes_en = BMA456_AN_EN_ALL_AXIS;

        /* Like threshold and duration, we can also change the config of int_bhvr and slope */

        /* Set the threshold and duration configuration */
        rslt = bma456_an_set_any_mot_config(&any_no_mot, bma);
        bma4_error_codes_print_result("bma456_an_set_any_mot_config status", rslt);
    }

    return rslt;
}
