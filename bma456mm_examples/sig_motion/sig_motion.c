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

    /* Variable to store sig-motion interrupt status */
    uint16_t int_status = 0;

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
        /* Map the interrupt pin 1 for sig-motion */
        rslt = bma456mm_map_interrupt(BMA4_INTR1_MAP, BMA456MM_SIG_MOT_INT, BMA4_ENABLE, &bma);
        bma4_error_codes_print_result("bma456mm_map_interrupt status", rslt);

        if (rslt == BMA4_OK)
        {
            /* Enabling sig-motion feature */
            rslt = bma456mm_feature_enable(BMA456MM_SIG_MOTION, BMA4_ENABLE, &bma);
            bma4_error_codes_print_result("bma456mm_feature_enable status", rslt);
        }

        if (rslt == BMA4_OK)
        {
            printf("Shake the sensor for greater than 3 sec to detect sig-motion interrupt\n");

            while (1)
            {
                /* Read the interrupt status */
                rslt = bma456mm_read_int_status(&int_status, &bma);
                bma4_error_codes_print_result("bma456mm_read_int_status", rslt);

                /* Check if sig-motion interrupt is received */
                if ((rslt == BMA4_OK) && (int_status & BMA456MM_SIG_MOT_INT))
                {
                    printf("\nReceived Sig-motion interrupt\n");
                    break;
                }

                int_status = 0;
            }
        }
    }

    bma4_coines_deinit();

    return rslt;
}
