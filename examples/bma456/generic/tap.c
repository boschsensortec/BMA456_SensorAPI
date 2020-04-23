/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include "bma456.h"
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
    uint8_t iteration = 10;

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated */
    rslt = bma4_interface_selection(&bma);
    bma4_error_codes_print_result("bma4_interface_selection status", rslt);

    /* Sensor initialization */
    rslt = bma456_init(&bma);
    bma4_error_codes_print_result("bma456_init status", rslt);

    /* Upload the configuration file to enable the features of the sensor. */
    rslt = bma456_write_config_file(&bma);
    bma4_error_codes_print_result("bma456_write_config status", rslt);

    /* Enable the accelerometer */
    rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    if (rslt == BMA4_OK)
    {
        /* Map the interrupt 1 for tap detection */
        rslt = bma456_map_interrupt(BMA4_INTR1_MAP, (BMA456_SINGLE_TAP_INT | BMA456_DOUBLE_TAP_INT), BMA4_ENABLE, &bma);
        bma4_error_codes_print_result("bma456_map_interrupt status", rslt);

        if (rslt == BMA4_OK)
        {
            /* Enabling the single and double tap features */
            rslt = bma456_feature_enable((BMA456_SINGLE_TAP | BMA456_DOUBLE_TAP), BMA4_ENABLE, &bma);
            bma4_error_codes_print_result("bma456_feature_enable status", rslt);
        }

        if (rslt == BMA4_OK)
        {
            printf("Do Single or Double Tap the board\n");

            while (1)
            {
                /* Read interrupt status */
                rslt = bma456_read_int_status(&int_status, &bma);
                bma4_error_codes_print_result("bma456_read_int_status", rslt);

                /* Enters only if the obtained interrupt is single-tap */
                if ((rslt == BMA4_OK) && (int_status & BMA456_SINGLE_TAP_INT))
                {
                    printf("Single tap interrupt occurred\n");
                    iteration--;
                }

                /* Enters only if the obtained interrupt is double-tap */
                else if ((rslt == BMA4_OK) && (int_status & BMA456_DOUBLE_TAP_INT))
                {
                    printf("Double tap interrupt occurred\n");
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
