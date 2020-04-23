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

    /* Variable to store step activity interrupt status */
    uint16_t int_status = 0;

    /* Variable to store activity status */
    uint8_t activity_output = 0;

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
        /* Map the interrupt 1 for step activity */
        rslt = bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_ACTIVITY_INT, BMA4_ENABLE, &bma);
        bma4_error_codes_print_result("bma456_map_interrupt status", rslt);

        if (rslt == BMA4_OK)
        {
            /* Setting watermark level 1, the output step resolution is 20 steps.
             * Eg: 1 means, 1 * 20 = 20. Every 20 steps once output triggers
             */
            rslt = bma456_step_counter_set_watermark(1, &bma);
            bma4_error_codes_print_result("bma456_step_counter_set_watermark status", rslt);

            if (rslt == BMA4_OK)
            {
                /* Enabling step detector feature */
                rslt = bma456_feature_enable(BMA456_STEP_ACT, BMA4_ENABLE, &bma);
                bma4_error_codes_print_result("bma456_feature_enable status", rslt);
            }
        }

        if (rslt == BMA4_OK)
        {
            printf("Move the board in steps to perform step activity\n");

            while (1)
            {
                /* Read interrupt status */
                rslt = bma456_read_int_status(&int_status, &bma);
                bma4_error_codes_print_result("bma456_read_int_status", rslt);

                /* Filtering only the activity interrupt */
                if ((rslt == BMA4_OK) && (int_status & BMA456_ACTIVITY_INT))
                {
                    printf("Activity interrupt occurred\n");

                    /* Get step activity output */
                    rslt = bma456_activity_output(&activity_output, &bma);
                    bma4_error_codes_print_result("bma456_activity_output status", rslt);

                    if (rslt == BMA4_OK)
                    {
                        printf("The Activity output is %d\n", activity_output);

                        switch (activity_output)
                        {
                            case BMA456_USER_STATIONARY:
                                printf("User state is stationary\n");
                                break;
                            case BMA456_USER_WALKING:
                                printf("User state is walking\n");
                                break;
                            case BMA456_USER_RUNNING:
                                printf("User state is running\n");
                                break;
                            case BMA456_STATE_INVALID:
                                printf("User state is invalid state\n");
                                break;
                            default:
                                break;
                        }

                        /* Break from while loop only if the activity interrupt occurs */
                        break;
                    }
                }

                int_status = 0;
            }
        }
    }

    return rslt;
}
