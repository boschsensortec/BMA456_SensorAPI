/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include "bma456w.h"
#include "common.h"

int main(void)
{
    struct bma4_dev bma;
    struct bma4_accel_config accel_conf;
    struct bma456w_wrist_wear_wakeup_params setting;
    uint8_t try = 10;
    int8_t rslt;

    /* Variable to get the interrupt status */
    uint16_t int_status = 0;

    /* Interface reference is given as a parameter
     *         For I2C : BMA4_I2C_INTF
     *         For SPI : BMA4_SPI_INTF
     * Variant information given as parameter - BMA45X_VARIANT
     */
    rslt = bma4_interface_init(&bma, BMA4_I2C_INTF, BMA45X_VARIANT);
    bma4_error_codes_print_result("bma4_interface_init", rslt);

    /* Sensor initialization */
    rslt = bma456w_init(&bma);
    bma4_error_codes_print_result("bma456w_init", rslt);

    /* Upload the configuration file to enable the features of the sensor. */
    rslt = bma456w_write_config_file(&bma);
    bma4_error_codes_print_result("bma456w_write_config", rslt);

    /* Accelerometer Configuration Setting */
    /* Output data Rate */
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;

    /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G) */
    accel_conf.range = BMA4_ACCEL_RANGE_2G;

    /* Bandwidth configure number of sensor samples required to average
     * if value = 2, then 4 samples are averaged
     * averaged samples = 2^(val(accel bandwidth))
     * Note1 : More info refer datasheets
     * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but since the
     * performance power mode phase is increased, the power consumption will also rise.
     */
    accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;

    /* Enable the filter performance mode where averaging of samples
     * will be done based on above set bandwidth and ODR.
     * There are two modes
     *  0 -> Averaging samples (Default)
     *  1 -> No averaging
     * For more info on No Averaging mode refer datasheets.
     */
    accel_conf.perf_mode = BMA4_CIC_AVG_MODE;

    /* Set the accel configurations */
    rslt = bma4_set_accel_config(&accel_conf, &bma);
    bma4_error_codes_print_result("bma4_set_accel_config status", rslt);

    /* NOTE : Enable accel after set of configurations */
    rslt = bma4_set_accel_enable(1, &bma);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    printf("\nEnable wear feature\n");

    rslt = bma456w_feature_enable(BMA456W_WRIST_WEAR_WAKEUP, 1, &bma);
    bma4_error_codes_print_result("bma456w_feature_enable status", rslt);

    rslt = bma456w_get_wrist_wear_wakeup_param_config(&setting, &bma);
    bma4_error_codes_print_result("bma456w_get_wrist_wear_wakeup_param_config status", rslt);

    rslt = bma456w_map_interrupt(BMA4_INTR1_MAP, BMA456W_WRIST_WEAR_WAKEUP_INT, BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma456w_map_interrupt status", rslt);

    printf("Do wear movement\n");

    for (;;)
    {
        /* Read the interrupt register to check whether wrist wear interrupt is received. */
        rslt = bma456w_read_int_status(&int_status, &bma);
        bma4_error_codes_print_result("bma456w_read_int_status status", rslt);

        /* Check if step counter interrupt is triggered */
        if ((BMA4_OK == rslt) && (int_status & BMA456W_WRIST_WEAR_WAKEUP_INT))
        {
            printf("\nWear interrupt is received\n\n");

            break;
        }
    }

    printf("\nDisable wear feature\n");
    rslt = bma456w_feature_enable(BMA456W_WRIST_WEAR_WAKEUP, BMA4_DISABLE, &bma);
    bma4_error_codes_print_result("bma456w_feature_enable status", rslt);

    rslt = bma456w_map_interrupt(BMA4_INTR1_MAP, BMA456W_WRIST_WEAR_WAKEUP_INT, BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma456w_map_interrupt status", rslt);

    printf("Do wear movement\n\n");

    while (try--)
    {
        rslt = bma456w_read_int_status(&int_status, &bma);
        bma4_error_codes_print_result("bma456w_read_int_status status", rslt);

        if (int_status & BMA456W_WRIST_WEAR_WAKEUP_INT)
        {
            printf("Wear interrupt is received, Test is failed\n");
            break;
        }
        else
        {
            printf("Wear feature disable and wear interrupt not received\n");
        }
    }

    bma4_coines_deinit();

    return rslt;
}
