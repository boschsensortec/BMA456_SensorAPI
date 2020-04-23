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

    /* Variable to store step counter interrupt status */
    uint16_t int_status = 0;

    /* Variable to store step counter output */
    uint32_t step_out = 0;

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
        /* Map the interrupt pin 1 for step counter */
        rslt = bma456h_map_interrupt(BMA4_INTR1_MAP, BMA456H_STEP_CNTR_INT, BMA4_ENABLE, &bma);
        bma4_error_codes_print_result("bma456h_map_interrupt status", rslt);

        if (rslt == BMA4_OK)
        {
            /* Setting watermark level 1, the output step resolution is 20 steps.
             * Eg: 1 means, 1 * 20 = 20. Every 20 steps once output triggers
             */
            rslt = bma456h_step_counter_set_watermark(1, &bma);
            bma4_error_codes_print_result("bma456h_step_counter_set_watermark status", rslt);

            if (rslt == BMA4_OK)
            {
                /* Enabling step counter feature */
                rslt = bma456h_feature_enable(BMA456H_STEP_COUNTER_EN, BMA4_ENABLE, &bma);
                bma4_error_codes_print_result("bma456h_feature_enable status", rslt);
            }
        }

        if (rslt == BMA4_OK)
        {
            printf("Move the board in steps to perform step counter\n");

            while (1)
            {
                /* Read interrupt status */
                rslt = bma456h_read_int_status(&int_status, &bma);
                bma4_error_codes_print_result("bma456h_read_int_status", rslt);

                /* Enters only if the obtained interrupt is step counter */
                if ((rslt == BMA4_OK) && (int_status & BMA456H_STEP_CNTR_INT))
                {
                    rslt = bma456h_step_counter_output(&step_out, &bma);
                    printf("The step counter output is %lu\n", step_out);
                    break;
                }

                int_status = 0;
            }
        }
    }

    return rslt;
}
