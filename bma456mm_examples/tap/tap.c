/**\
 * Copyright (c) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include "bma456mm.h"
#include "common.h"

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

    struct bma456mm_out_state tap_out = { 0 };
    struct bma456mm_multitap_settings settings = { 0 };

    /* Interface reference is given as a parameter
     *         For I2C : BMA4_I2C_INTF
     *         For SPI : BMA4_SPI_INTF
     * Variant information given as parameter - BMA45X_VARIANT
     */
    rslt = bma4_interface_init(&bma, BMA4_I2C_INTF, BMA45X_VARIANT);
    bma4_error_codes_print_result("bma4_interface_init", rslt);

    /* Sensor initialization */
    rslt = bma456mm_init(&bma);
    bma4_error_codes_print_result("bma456mm_init status", rslt);

    /* Upload the configuration file to enable the features of the sensor. */
    rslt = bma456mm_write_config_file(&bma);
    bma4_error_codes_print_result("bma456mm_write_config status", rslt);

    /* Enable the accelerometer */
    rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    if (rslt == BMA4_OK)
    {
        /* Map the interrupt pin 1 for tap detection */
        rslt = bma456mm_map_interrupt(BMA4_INTR1_MAP, BMA456MM_TAP_OUT_INT, BMA4_ENABLE, &bma);
        bma4_error_codes_print_result("bma456mm_map_interrupt status", rslt);

        if (rslt == BMA4_OK)
        {
            /* Enabling the single, double and triple tap features */
            rslt = bma456mm_feature_enable((BMA456MM_SINGLE_TAP | BMA456MM_DOUBLE_TAP | BMA456MM_TRIPLE_TAP),
                                           BMA4_ENABLE,
                                           &bma);
            bma4_error_codes_print_result("bma456mm_feature_enable status", rslt);

            if (rslt == BMA4_OK)
            {
                /* Getting tap parameters settings */
                rslt = bma456mm_tap_get_parameter(&settings, &bma);
                bma4_error_codes_print_result("bma456mm_tap_get_parameter status", rslt);
            }
        }

        if (rslt == BMA4_OK)
        {
            printf("Tap the board either single, double or triple tap\n");

            while (loop > 0)
            {
                /* Read interrupt status */
                rslt = bma456mm_read_int_status(&int_status, &bma);
                bma4_error_codes_print_result("bma456mm_read_int_status", rslt);

                /* Filtering only the tap interrupt */
                if ((rslt == BMA4_OK) && (int_status & BMA456MM_TAP_OUT_INT))
                {
                    rslt = bma456mm_output_state(&tap_out, &bma);

                    if (BMA4_OK == rslt)
                    {
                        /* Enters only if the obtained interrupt is single-tap */
                        if (tap_out.s_tap)
                        {
                            printf("Single Tap interrupt occurred\n");
                        }
                        /* Enters only if the obtained interrupt is double-tap */
                        else if (tap_out.d_tap)
                        {
                            printf("Double Tap interrupt occurred\n");
                        }
                        /* Enters only if the obtained interrupt is triple-tap */
                        else if (tap_out.t_tap)
                        {
                            printf("Triple Tap interrupt occurred\n");
                        }

                        loop--;
                    }

                    int_status = 0;
                }
            }

            /* Break out of the loop when iteration has reached zero */
            if (loop == 0)
            {
                printf("Iterations are done. Exiting !\n\n");
            }
        }
    }

    bma4_coines_deinit();

    return rslt;
}
