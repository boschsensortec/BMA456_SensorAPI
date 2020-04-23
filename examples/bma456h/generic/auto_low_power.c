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

    /* Variable to store auto low power state */
    uint8_t auto_low_power_state = 0;

    struct bma4_accel_config accel_conf = { 0 };
    struct bma456h_auto_low_power auto_low_power = { 0 };

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

    /* Get the accel configurations */
    rslt = bma4_get_accel_config(&accel_conf, &bma);
    bma4_error_codes_print_result("bma4_get_accel_config status", rslt);

    if (rslt == BMA4_OK)
    {
        /* Accelerometer configuration settings */
        /* Output data Rate */
        accel_conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G) */
        accel_conf.range = BMA4_ACCEL_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a less noisier signal, but
         * this has an adverse effect on the power consumed.
         */
        accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Averaging samples (Default)
         *  1 -> No averaging
         * For more info on No Averaging mode refer datasheet.
         */
        accel_conf.perf_mode = BMA4_CIC_AVG_MODE;

        /* Set the accel configurations */
        rslt = bma4_set_accel_config(&accel_conf, &bma);
        bma4_error_codes_print_result("bma4_set_accel_config status", rslt);

        /* Enable auto low power feature */
        rslt = bma456h_feature_enable(BMA456H_AUTO_LOW_POWER_EN, BMA4_ENABLE, &bma);
        bma4_error_codes_print_result("bma456h_feature_enable status", rslt);

        if (rslt == BMA4_OK)
        {
            /* Enable auto low power state through no-motion interrupt */
            auto_low_power.no_motion = 0;

            /* Enable auto low power state through time-out duration */
            auto_low_power.time_out = 1;

            /* Time-out duration is (value * resolution) eg: (10 * 20ms) */
            auto_low_power.time_out_dur = 10;

            /* These bits used to set odr at auto low power mode. default
             * value is '0'(i.e) 25/16 HZ */
            auto_low_power.lp_odr = 0;

            /* Set this bit to set default odr of titan core in auto low power mode */
            auto_low_power.pwr_mgt = 1;

            /* Configure Auto low power settings */
            rslt = bma456h_set_auto_low_power_config(&auto_low_power, &bma);
            bma4_error_codes_print_result("bma456h_set_auto_low_power_config status", rslt);

            if (rslt == BMA4_OK)
            {
                /* Get output of auto low power state */
                rslt = bma456h_get_auto_low_power_state(&auto_low_power_state, &bma);
                bma4_error_codes_print_result("bma456h_get_auto_low_power_state status", rslt);
            }
        }
    }

    if (rslt == BMA4_OK)
    {
        /* Read and check the ODR change after changing the auto low power settings */
        rslt = bma4_get_accel_config(&accel_conf, &bma);
        bma4_error_codes_print_result("bma4_get_accel_config status", rslt);

        if (rslt == BMA4_OK)
        {
            printf("ODR : %X\n", accel_conf.odr);

            if (auto_low_power_state)
            {
                printf("Auto low power state : Disabled(Auto wake state)");
            }
            else
            {
                printf("Auto low power state : Enabled(Auto sleep state)");
            }
        }
    }

    return rslt;
}
