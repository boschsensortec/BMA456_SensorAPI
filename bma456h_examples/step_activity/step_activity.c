/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include "bma456h.h"
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

    /* Variable to hold iteration value */
    uint8_t loop = 1;

    /* Variable to store step activity interrupt status */
    uint16_t int_status = 0;

    struct bma456h_out_state activity_output = { 0 };

    /* Interface reference is given as a parameter
     *         For I2C : BMA4_I2C_INTF
     *         For SPI : BMA4_SPI_INTF
     * Variant information given as parameter - BMA45X_VARIANT
     */
    rslt = bma4_interface_init(&bma, BMA4_I2C_INTF, BMA45X_VARIANT);
    bma4_error_codes_print_result("bma4_interface_init", rslt);

    /* Sensor initialization */
    rslt = bma456h_init(&bma);
    bma4_error_codes_print_result("bma456h_init status", rslt);

    /* Upload the configuration file to enable the features of the sensor */
    rslt = bma456h_write_config_file(&bma);
    bma4_error_codes_print_result("bma456h_write_config status", rslt);

    /* Enable the accelerometer */
    rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    if (rslt == BMA4_OK)
    {
        /* Enabling step activity feature */
        rslt = bma456h_feature_enable(BMA456H_STEP_ACTIVITY_EN, BMA4_ENABLE, &bma);
        bma4_error_codes_print_result("bma456h_feature_enable status", rslt);

        if (rslt == BMA4_OK)
        {
            /* Map the interrupt pin 1 for step counter */
            rslt = bma456h_map_interrupt(BMA4_INTR1_MAP, BMA456H_ACTIVITY_INT, BMA4_ENABLE, &bma);
            bma4_error_codes_print_result("bma456h_map_interrupt status", rslt);
        }

        if (rslt == BMA4_OK)
        {
            printf("Move the board in steps to perform step activity\n");

            while (loop <= 5)
            {
                /* Read interrupt status */
                rslt = bma456h_read_int_status(&int_status, &bma);
                bma4_error_codes_print_result("bma456h_read_int_status", rslt);

                /* Filtering only the activity interrupt */
                if ((rslt == BMA4_OK) && (int_status & BMA456H_ACTIVITY_INT))
                {
                    printf("\nIteration : %d\n", loop);

                    /* Get step activity output */
                    rslt = bma456h_output_state(&activity_output, &bma);

                    if (rslt == BMA4_OK)
                    {
                        printf("The Activity output is %d\n", activity_output.activity_type);

                        switch (activity_output.activity_type)
                        {
                            case BMA456H_USER_STATIONARY:
                                printf("User state is stationary\n");
                                break;
                            case BMA456H_USER_WALKING:
                                printf("User state is walking\n");
                                break;
                            case BMA456H_USER_RUNNING:
                                printf("User state is running\n");
                                break;
                            case BMA456H_UNKNOWN_ACTVTY:
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
