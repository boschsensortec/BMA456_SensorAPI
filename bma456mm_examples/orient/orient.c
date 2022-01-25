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

    /* Variable to store step activity interrupt status */
    uint16_t int_status = 0;

    /* Structure to hold orientation configuration */
    struct bma456mm_orientation_config orient;
    struct bma456mm_out_state out_state;

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

    /* Enable orientation feature */
    rslt = bma456mm_feature_enable(BMA456MM_ORIENT, 1, &bma);
    bma4_error_codes_print_result("bma456mm_feature_enable status", rslt);

    orient.upside_down = 0x01;
    orient.mode = 0x02;
    orient.blocking = 0x01;
    orient.theta = 0x33;
    orient.hysteresis = 0x80;

    rslt = bma456mm_set_orientation_config(&orient, &bma);
    bma4_error_codes_print_result("bma456mm_set_orientation_config status", rslt);

    if (rslt == BMA4_OK)
    {
        /* Map the interrupt 1 for tap detection */
        rslt = bma456mm_map_interrupt(BMA4_INTR1_MAP, BMA456MM_ORIENT_INT, BMA4_ENABLE, &bma);
        bma4_error_codes_print_result("bma456mm_map_interrupt status", rslt);

        if (rslt == BMA4_OK)
        {
            printf("Move the board to detect orientation\n");

            while (1)
            {
                /* Read interrupt status */
                rslt = bma456mm_read_int_status(&int_status, &bma);
                bma4_error_codes_print_result("bma456mm_read_int_status", rslt);

                /* Filtering only the activity interrupt */
                if ((rslt == BMA4_OK) && (int_status & BMA456MM_ORIENT_INT))
                {
                    printf("\nOrientation interrupt occurred\n");

                    rslt = bma456mm_output_state(&out_state, &bma);
                    bma4_error_codes_print_result("bma456mm_orientation_output", rslt);

                    printf("The Orientation output is %d\n", out_state.orientation_out);
                    printf("The Orientation faceup/down output is %d\n", out_state.orientation_faceup_down);

                    switch (out_state.orientation_faceup_down)
                    {
                        case BMA456MM_FACE_UP:
                            printf("\nOrientation state is face up\n");
                            break;
                        case BMA456MM_FACE_DOWN:
                            printf("\nOrientation state is face down\n");
                            break;
                    }

                    switch (out_state.orientation_out)
                    {
                        case BMA456MM_LANDSCAPE_LEFT:
                            printf("\nOrientation state is landscape left\n\n");
                            break;
                        case BMA456MM_LANDSCAPE_RIGHT:
                            printf("\nOrientation state is landscape right\n\n");
                            break;
                        case BMA456MM_PORTRAIT_UP_DOWN:
                            printf("\nOrientation state is portrait upside down\n\n");
                            break;
                        case BMA456MM_PORTRAIT_UP_RIGHT:
                            printf("\nOrientation state is portrait upright\n\n");
                            break;
                    }

                    break;
                }

                int_status = 0;
            }
        }
    }

    bma4_coines_deinit();

    return rslt;
}
