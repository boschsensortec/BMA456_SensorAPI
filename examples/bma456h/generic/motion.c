/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include "bma456h.h"
#include "bma4_common.h"

/*****************************************************************************/
/*!                Global variable                                           */

/*! Structure to define any/no-motion configurations */
struct bma456h_any_no_mot_config any_no_mot = { 0 };

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

    uint8_t iteration = 20;

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated */
    rslt = bma4_interface_selection(&bma);
    bma4_error_codes_print_result("bma4_interface_selection status", rslt);

    /* Sensor initialization */
    rslt = bma456h_init(&bma);
    bma4_error_codes_print_result("bma456h_init status", rslt);

    /* Upload the configuration file to enable the features of the sensor. */
    rslt = bma456h_write_config_file(&bma);
    bma4_error_codes_print_result("bma456h_write_config status", rslt);

    /* Enable the accelerometer */
    rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    /* Map the interrupt pin 1 for any/no-motion */
    rslt = bma456h_map_interrupt(BMA4_INTR1_MAP, (BMA456H_ANY_MOT_INT | BMA456H_NO_MOT_INT), BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma456h_map_interrupt status", rslt);

    /* Enabling X, Y, and Z axis for Any and No-motion feature */
    rslt = bma456h_feature_enable((BMA456H_ANY_MOTION_ALL_AXIS_EN | BMA456H_NO_MOTION_ALL_AXIS_EN), BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma456h_feature_enable status", rslt);

    /* Get any-motion and no-motion configurations */
    rslt = get_any_no_mot_config(&bma);
    bma4_error_codes_print_result("get_any_no_mot_config status", rslt);

    printf("Shake the board for any-motion interrupt whereas do not shake the board for no-motion interrupt\n");

    if (rslt == BMA4_OK)
    {
        while (1)
        {
            /* Read interrupt status */
            rslt = bma456h_read_int_status(&int_status, &bma);
            bma4_error_codes_print_result("bma456h_read_int_status", rslt);

            if (rslt == BMA4_OK)
            {
                /* Enters only if the obtained interrupt is any-motion */
                if (int_status & BMA456H_ANY_MOT_INT)
                {
                    printf("Any-motion interrupt occurred\n");
                    iteration--;
                }

                /* Enters only if the obtained interrupt is no-motion */
                else if (int_status & BMA456H_NO_MOT_INT)
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
    rslt = bma456h_get_any_mot_config(&any_no_mot, bma);
    bma4_error_codes_print_result("bma456h_get_any_mot_config status", rslt);

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

        /* Like threshold and duration, we can also change the config of int_bhvr and slope */

        /* Set the threshold and duration configuration */
        rslt = bma456h_set_any_mot_config(&any_no_mot, bma);
        bma4_error_codes_print_result("bma456h_set_any_mot_config status", rslt);

        if (rslt == BMA4_OK)
        {
            /* Getting no-motion configuration to get default configuration */
            rslt = bma456h_get_no_mot_config(&any_no_mot, bma);
            bma4_error_codes_print_result("bma456h_get_no_mot_config status", rslt);

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

                /* Like threshold and duration, we can also change the config of int_bhvr */

                /* Set the threshold and duration configuration */
                rslt = bma456h_set_no_mot_config(&any_no_mot, bma);
                bma4_error_codes_print_result("bma456h_set_no_mot_config status", rslt);
            }
        }
    }

    return rslt;
}
