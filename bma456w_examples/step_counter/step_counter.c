/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include "bma456w.h"
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

    /* Variable to store step counter interrupt status */
    uint16_t int_status = 0;

    /* Variable to store step counter status */
    uint32_t step_out = 0;

    /* Interface reference is given as a parameter
     *         For I2C : BMA4_I2C_INTF
     *         For SPI : BMA4_SPI_INTF
     * Variant information given as parameter - BMA45X_VARIANT
     */
    rslt = bma4_interface_init(&bma, BMA4_I2C_INTF, BMA45X_VARIANT);
    bma4_error_codes_print_result("bma4_interface_init", rslt);

    /* Sensor initialization */
    rslt = bma456w_init(&bma);
    bma4_error_codes_print_result("bma456w_init status", rslt);

    /* Upload the configuration file to enable the features of the sensor. */
    rslt = bma456w_write_config_file(&bma);
    bma4_error_codes_print_result("bma456w_write_config status", rslt);

    /* Enable accelerometer to perform feature testing */
    rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    rslt = bma456w_feature_enable(BMA456W_STEP_CNTR, BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma456w_feature_enable status", rslt);

    rslt = bma456w_map_interrupt(BMA4_INTR1_MAP, BMA456W_STEP_CNTR_INT, BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma456w_map_interrupt status", rslt);

    /* Setting watermark level 1, the output step resolution is 20 steps.
     * Eg: 1 means, 1 * 20 = 20. Every 20 steps once output triggers
     */
    rslt = bma456w_step_counter_set_watermark(1, &bma);
    bma4_error_codes_print_result("bma456w_step_counter_set_watermark status", rslt);

    printf("Step counter feature is enabled\n");

    printf("Step counter watermark level is 1 (Output resolution is 20 steps)\n");

    printf("Move the board in steps for greater than 3 seconds\n");

    /* Loop over until step counter interrupt occurs */
    for (;;)
    {
        /* Read the interrupt status (After 20 steps, generates interrupt) */
        rslt = bma456w_read_int_status(&int_status, &bma);
        bma4_error_codes_print_result("bma456w_read_int_status", rslt);

        if ((BMA4_OK == rslt) && (int_status & BMA456W_STEP_CNTR_INT))
        {
            printf("Step counter interrupt received when watermark level is reached (20 steps)\n");

            /* Get the step counter output after reset */
            rslt = bma456w_step_counter_output(&step_out, &bma);
            bma4_error_codes_print_result("bma456w_step_counter_output status", rslt);

            break;
        }
    }

    printf("The step counter output is %lu \n", (long unsigned int)step_out);

    bma4_coines_deinit();

    return rslt;
}
