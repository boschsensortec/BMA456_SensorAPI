/**\
 * Copyright (c) 2021 Bosch Sensortec GmbH. All rights reserved.
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

    /* Variable to store result of self-test */
    int8_t selftest_rslt = 0;

    /* Interface reference is given as a parameter
     *         For I2C : BMA4_I2C_INTF
     *         For SPI : BMA4_SPI_INTF
     * Variant information given as parameter - BMA45X_VARIANT
     */
    rslt = bma4_interface_init(&bma, BMA4_SPI_INTF, BMA45X_VARIANT);
    bma4_error_codes_print_result("bma4_interface_init", rslt);

    /* Sensor initialization */
    rslt = bma456h_init(&bma);
    bma4_error_codes_print_result("bma456h_init status", rslt);

    printf("Perform accel self-test\n");

    rslt = bma4_perform_accel_selftest(&selftest_rslt, &bma);
    bma4_error_codes_print_result("bma4_perform_accel_selftest", rslt);
    printf("Self-test result is %d\n", selftest_rslt);

    bma4_coines_deinit();

    return rslt;
}
