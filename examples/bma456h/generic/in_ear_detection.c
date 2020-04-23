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

    /* Variable to store in-ear status */
    uint16_t int_status = 0;

    struct bma456h_out_state ear_detect_out = { 0 };

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

    /* Map in-ear interrupt to interrupt pin 1 */
    rslt = bma456h_map_interrupt(BMA4_INTR1_MAP, BMA456H_IN_EAR_INT, BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma456h_map_interrupt status", rslt);

    if (rslt == BMA4_OK)
    {
        /* Enable in-ear feature */
        rslt = bma456h_feature_enable(BMA456H_IN_EAR_DETECTION_EN, BMA4_ENABLE, &bma);
        bma4_error_codes_print_result("bma456h_feature_enable status", rslt);
    }

    if (rslt == BMA4_OK)
    {
        printf("Do In-ear movement\n");

        while (1)
        {
            /* Read interrupt status */
            rslt = bma456h_read_int_status(&int_status, &bma);
            bma4_error_codes_print_result("bma456h_read_int_status", rslt);

            /* Enters only if the obtained interrupt is in-ear */
            if ((rslt == BMA4_OK) && (int_status & BMA456H_IN_EAR_INT))
            {
                printf("In-ear interrupt occurred\n");

                /* Read the status of the hearing device */
                rslt = bma456h_output_state(&ear_detect_out, &bma);

                if (rslt == BMA4_OK)
                {
                    printf("The In-ear detection output is %d\n", ear_detect_out.ear_detection_out);

                    switch (ear_detect_out.ear_detection_out)
                    {
                        case BMA456H_DEVICE_UNKNOWN:
                            printf("Device is in unknown state\n");
                            break;
                        case BMA456H_DEVICE_NOT_IN_EAR:
                            printf("Device is not in-ear\n");
                            break;
                        case BMA456H_DEVICE_WEARING:
                            printf("Device is wearing \n");
                            break;
                        case BMA456H_DEVICE_REMOVING:
                            printf("Device is removing\n");
                            break;
                        case BMA456H_DEVICE_IN_EAR:
                            printf("Device is in ear\n");
                            break;
                        default:
                            break;
                    }
                }

                /* To break from while loop once the interrupt occurred */
                break;
            }
        }
    }

    return rslt;
}
