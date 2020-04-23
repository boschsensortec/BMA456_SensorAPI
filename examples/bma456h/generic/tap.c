/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include "bma456h.h"
#include "bma4_common.h"

/******************************************************************************/
/*!            Function                                                       */

/* This function starts the execution of program. */
int main(void)
{
    /* Variable to store the status of API */
    int8_t rslt;

    /* Sensor initialization configuration */
    struct bma4_dev bma = { 0 };

    /* Variable to store tap interrupt status */
    uint16_t int_status = 0;

    /* Loop variable */
    uint8_t loop = 10;

    struct bma456h_out_state tap_out = { 0 };
    struct bma456h_multitap_settings settings = { 0 };

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

    if (rslt == BMA4_OK)
    {
        /* Map the interrupt pin 1 for tap detection */
        rslt = bma456h_map_interrupt(BMA4_INTR1_MAP, BMA456H_TAP_OUT_INT, BMA4_ENABLE, &bma);
        bma4_error_codes_print_result("bma456h_map_interrupt status", rslt);

        if (rslt == BMA4_OK)
        {
            /* Enabling the single, double and triple tap features */
            rslt = bma456h_feature_enable((BMA456H_SINGLE_TAP_EN | BMA456H_DOUBLE_TAP_EN | BMA456H_TRIPLE_TAP_EN),
                                          BMA4_ENABLE,
                                          &bma);
            bma4_error_codes_print_result("bma456h_feature_enable status", rslt);

            if (rslt == BMA4_OK)
            {
                /* Getting tap parameters settings */
                rslt = bma456h_tap_get_parameter(&settings, &bma);
                bma4_error_codes_print_result("bma456h_tap_get_parameter status", rslt);
            }
        }

        if (rslt == BMA4_OK)
        {
            printf("Tap the board either single, double or triple tap\n");

            while (loop > 0)
            {
                /* Read interrupt status */
                rslt = bma456h_read_int_status(&int_status, &bma);
                bma4_error_codes_print_result("bma456h_read_int_status", rslt);

                /* Filtering only the tap interrupt */
                if ((rslt == BMA4_OK) && (int_status & BMA456H_TAP_OUT_INT))
                {
                    rslt = bma456h_output_state(&tap_out, &bma);

                    if (BMA4_OK == rslt)
                    {
                        /* Enters only if the obtained interrupt is single-tap */
                        if (tap_out.single_tap)
                        {
                            printf("Single Tap interrupt occurred\n");
                        }
                        /* Enters only if the obtained interrupt is double-tap */
                        else if (tap_out.double_tap)
                        {
                            printf("Double Tap interrupt occurred\n");
                        }
                        /* Enters only if the obtained interrupt is triple-tap */
                        else if (tap_out.triple_tap)
                        {
                            printf("Triple Tap interrupt occurred\n");
                        }

                        loop--;
                    }

                    int_status = 0;
                }
            }
        }
    }

    return rslt;
}
