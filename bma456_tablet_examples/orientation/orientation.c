/**\
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include "bma456_tablet.h"
#include "common.h"

/******************************************************************************/
/*!                          Macros                                           */

#define         COUNT  UINT8_C(5)

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

    /* Variable to hold iteration value */
    uint8_t loop = 1;

    /* Structure to hold orientation configuration */
    struct bma456_tablet_orientation_config orient;
    uint8_t orientation_out = 0;
    uint8_t orientation_faceup_down = 0;

    /* Interface reference is given as a parameter
     *         For I2C : BMA4_I2C_INTF
     *         For SPI : BMA4_SPI_INTF
     * Variant information given as parameter - BMA45X_VARIANT
     */
    rslt = bma4_interface_init(&bma, BMA4_I2C_INTF, BMA45X_VARIANT);
    bma4_error_codes_print_result("bma4_interface_init", rslt);

    /* Sensor initialization */
    rslt = bma456_tablet_init(&bma);
    bma4_error_codes_print_result("bma456_tablet_init status", rslt);

    /* Upload the configuration file to enable the features of the sensor. */
    rslt = bma456_tablet_write_config_file(&bma);
    bma4_error_codes_print_result("bma456_tablet_write_config status", rslt);

    /* Enable the accelerometer */
    rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    /* Enable orientation feature */
    rslt = bma456_tablet_feature_enable(BMA456_TABLET_ORIENTATION, 1, &bma);
    bma4_error_codes_print_result("bma456_tablet_feature_enable status", rslt);

    orient.upside_down = 0x01;
    orient.mode = 0x02;
    orient.blocking = 0x01;
    orient.hold_time = 0x14;
    orient.slope_thres = 0xA4;
    orient.hysteresis = 0x80;
    orient.theta = 0x33;

    rslt = bma456_tablet_set_orientation_config(&orient, &bma);
    bma4_error_codes_print_result("bma456_tablet_set_orientation_config status", rslt);

    if (rslt == BMA4_OK)
    {
        /* Map the interrupt 1 for orientation detection */
        rslt = bma456_tablet_map_interrupt(BMA4_INTR1_MAP, BMA456_TABLET_ORIENTATION_INT, BMA4_ENABLE, &bma);
        bma4_error_codes_print_result("bma456_tablet_map_interrupt status", rslt);

        if (rslt == BMA4_OK)
        {
            printf("Move the board to detect orientation\n");

            while (loop <= COUNT)
            {
                int_status = 0;

                /* Read interrupt status */
                rslt = bma456_tablet_read_int_status(&int_status, &bma);
                bma4_error_codes_print_result("bma456_tablet_read_int_status", rslt);

                /* Filtering only the orientation interrupt */
                if ((rslt == BMA4_OK) && (int_status & BMA456_TABLET_ORIENTATION_INT))
                {
                    printf("\nIteration : %d\n", loop);

                    rslt = bma456_tablet_orientation_output(&orientation_out, &orientation_faceup_down, &bma);
                    bma4_error_codes_print_result("bma456_tablet_orientation_output", rslt);

                    printf("The Orientation output is %d\n", orientation_out);
                    printf("The Orientation faceup/down output is %d\n", orientation_faceup_down);

                    switch (orientation_faceup_down)
                    {
                        case BMA456_TABLET_FACE_UP:
                            printf("\nOrientation state is face up\n");
                            break;
                        case BMA456_TABLET_FACE_DOWN:
                            printf("\nOrientation state is face down\n");
                            break;
                        default:
                            break;
                    }

                    switch (orientation_out)
                    {
                        case BMA456_TABLET_LANDSCAPE_LEFT:
                            printf("\nOrientation state is landscape left\n\n");
                            break;
                        case BMA456_TABLET_LANDSCAPE_RIGHT:
                            printf("\nOrientation state is landscape right\n\n");
                            break;
                        case BMA456_TABLET_PORTRAIT_UP_DOWN:
                            printf("\nOrientation state is portrait upside down\n\n");
                            break;
                        case BMA456_TABLET_PORTRAIT_UP_RIGHT:
                            printf("\nOrientation state is portrait upright\n\n");
                            break;
                        default:
                            break;
                    }

                    loop++;

                    if (loop > COUNT)
                    {
                        break;
                    }
                }
            }
        }
    }

    bma4_coines_deinit();

    return rslt;
}
