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

    /* Variable to store step detector interrupt status */
    uint16_t int_status = 0;

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

    /* Upload the configuration file to enable the features of the sensor. */
    rslt = bma456h_write_config_file(&bma);
    bma4_error_codes_print_result("bma456h_write_config status", rslt);

    /* Enable the accelerometer */
    rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    if (rslt == BMA4_OK)
    {
        /* Map the interrupt pin 1 for step detector */
        rslt = bma456h_map_interrupt(BMA4_INTR1_MAP, BMA456H_STEP_CNTR_INT, BMA4_ENABLE, &bma);
        bma4_error_codes_print_result("bma456h_map_interrupt status", rslt);

        if (rslt == BMA4_OK)
        {
            /* Enabling step detector feature */
            rslt = bma456h_feature_enable(BMA456H_STEP_DETECTOR_EN, BMA4_ENABLE, &bma);
            bma4_error_codes_print_result("bma456h_feature_enable status", rslt);

            if (rslt == BMA4_OK)
            {
                /* By setting watermark level 1, the output step resolution is 20 steps.
                 * By setting watermark level 0, it provides output for each steps for step detection.
                 */
                rslt = bma456h_step_counter_set_watermark(0, &bma);
                bma4_error_codes_print_result("bma456h_step_counter_set_watermark status", rslt);
            }
        }

        if (rslt == BMA4_OK)
        {
            printf("Move the board in steps to get interrupt\n");

            for (;;)
            {
                /* Read interrupt status */
                rslt = bma456h_read_int_status(&int_status, &bma);
                bma4_error_codes_print_result("bma456h_read_int_status", rslt);

                /* Filtering only the step detector interrupt */
                if ((rslt == BMA4_OK) && (int_status & BMA456H_STEP_CNTR_INT))
                {
                    printf("The step detector interrupt is occurred\n");
                    break;
                }

                int_status = 0;
            }
        }
    }

    bma4_coines_deinit();

    return rslt;
}
