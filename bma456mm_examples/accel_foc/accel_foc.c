/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include <string.h>

#include "bma456mm.h"
#include "common.h"

/******************************************************************************/
/*!                   Macro Definitions                                       */

#define ACCEL_SAMPLE_COUNT  UINT8_C(100)

/******************************************************************************/
/*!         Global Variable Declaration                                       */

/* Structure to store temporary axes data values */
struct temp_axes_val
{
    /* X data */
    int32_t x;

    /* Y data */
    int32_t y;

    /* Z data */
    int32_t z;
};

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used perform accel foc and determine limits based on range
 *
 *  @param[in] range              : Range of Accel
 *  @param[in] input_axis         : Axis selected for Accel FOC
 *  @param[in,out] dev            : Structure instance of bma4_dev.
 *
 *  @return Status of execution.
 */
static int8_t perform_foc_range_test(uint8_t range, uint8_t input_axis, struct bma4_dev *dev);

/*!
 *  @brief This internal API is to determine if average accel FOC data is within limits
 *
 *  @param[in] range                   : Value of Accel range
 *  @param[in] avg_accel_foc_data      : Average Accel FOC value
 *  @param[in] reference               : Reference LSB based on Accel Range
 *  @param[in] foc_sign                : Input sign of performed Accel FOC
 *  @param[in] min_val                 : Minimum acceptable LSB limit
 *  @param[in] max_val                 : Maximum acceptable LSB limit
 *
 *  @return Status of execution.
 */
static int8_t accel_foc_report(uint8_t range,
                               int16_t avg_accel_foc_data,
                               int16_t reference,
                               uint8_t foc_sign,
                               int16_t min_val,
                               int16_t max_val);

/*!
 *  @brief This internal API is to collect and verify accel sensor data
 *
 *  @param[in] range                   : Value of Accel range
 *  @param[in] reference               : Reference LSB based on Accel Range
 *  @param[in] matched_axis            : Input Axis to perform Accel FOC
 *  @param[in] foc_sign                : Input sign to perform Accel FOC
 *  @param[in,out] dev                : Structure instance of bma4_dev.
 *
 *  @return Status of execution.
 */
static int8_t verify_accel_foc_data(uint8_t range,
                                    int16_t reference,
                                    int8_t matched_axis,
                                    uint8_t foc_sign,
                                    struct bma4_dev *dev);

/*!
 *  @brief This internal API is to calculate noise level for Accel FOC data
 *
 *  @param[in] matched_axis            : Input Axis to perform accel FOC
 *  @param[in] accel_foc_data          : Array of Accel FOC data
 *  @param[in] avg_accel_foc_data      : Average Accel FOC data
 *
 *  @return Status of execution.
 */
static void calculate_noise(int8_t matched_axis, struct bma4_accel *accel_foc_data,
                            struct bma4_accel avg_accel_foc_data);

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    /* Sensor initialization configuration. */
    struct bma4_dev dev;

    uint8_t try = 0, j = 0;
    int8_t rslt;
    struct bma4_accel_config accel_conf = { 0 };
    uint8_t data = 0, range, input_axis = 0;

    /* Interface reference is given as a parameter
     *         For I2C : BMA4_I2C_INTF
     *         For SPI : BMA4_SPI_INTF
     * Variant information given as parameter - BMA45X_VARIANT
     */
    rslt = bma4_interface_init(&dev, BMA4_I2C_INTF, BMA45X_VARIANT);
    bma4_error_codes_print_result("bma4_interface_init", rslt);

    printf("Functional test for accel foc start..\n\n");

    printf("Choose the axis for accel FOC to be done\n");
    printf("Press '1' to choose X axis\n");
    printf("Press '2' to choose Y axis\n");
    printf("Press '3' to choose Z axis\n");

    printf("Press '4' to choose -X axis\n");
    printf("Press '5' to choose -Y axis\n");
    printf("Press '6' to choose -Z axis\n");

    while (1)
    {
        scanf("%hu", (short unsigned int *)&input_axis);
        if (input_axis > 0 && input_axis < 7)
        {
            break;
        }
    }

    if (input_axis == 1)
    {
        printf("The choosen input axis for FOC is : X\n");
    }
    else if (input_axis == 2)
    {
        printf("The choosen input axis for FOC is : Y\n");
    }
    else if (input_axis == 3)
    {
        printf("The choosen input axis for FOC is : Z\n");
    }
    else if (input_axis == 4)
    {
        printf("The choosen input axis for FOC is : -X\n");
    }
    else if (input_axis == 5)
    {
        printf("The choosen input axis for FOC is : -Y\n");
    }
    else if (input_axis == 6)
    {
        printf("The choosen input axis for FOC is : -Z\n");
    }

    printf("Confirm your chosen axis and the sensor keeping position are same before doing FOC\n");

    for (j = 0; j < 2; j++)
    {
        try = 0;

        if (j == 1)
        {
            printf("Keep sensor in wrong position and press 5\n");
        }
        else if (j == 0)
        {
            printf("Keep sensor in right position and press 5\n");
        }

        while (1)
        {
            scanf("%hu", (short unsigned int *)&try);
            if (try == 5)
            {
                break;
            }
        }

        for (range = BMA4_ACCEL_RANGE_2G; range <= BMA4_ACCEL_RANGE_16G; range++)
        {
            /****************************************************************/
            /* Initialize by enabling configuration load */
            printf("#########################################################\n\n");

            rslt = bma456mm_init(&dev);
            bma4_error_codes_print_result("bma4_init", rslt);

            /* Upload the configuration file to enable the features of the sensor. */
            rslt = bma456mm_write_config_file(&dev);
            bma4_error_codes_print_result("bma4_write_config", rslt);

            /* Enable the accelerometer */
            rslt = bma4_set_accel_enable(BMA4_ENABLE, &dev);
            bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

            /* Accelerometer Configuration Settings */
            /* Output Data Rate */
            accel_conf.odr = BMA4_OUTPUT_DATA_RATE_50HZ;

            /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G) */
            accel_conf.range = range;

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
            rslt = bma4_set_accel_config(&accel_conf, &dev);
            bma4_error_codes_print_result("bma4_set_accel_config status", rslt);

            /* Delay to set accel sensor configurations (20ms for 50HZ) */
            dev.delay_us(20000, dev.intf_ptr);

            /* Mapping data ready interrupt with interrupt1 to get interrupt status once getting new accel data */
            rslt = bma456mm_map_interrupt(BMA4_INTR1_MAP, BMA4_DATA_RDY_INT, BMA4_ENABLE, &dev);
            bma4_error_codes_print_result("bma4_map_interrupt status", rslt);

            printf("ODR = %d, RANGE = %d, BANDWIDTH = %d\n", accel_conf.odr, accel_conf.range, accel_conf.bandwidth);

            /* Perform FOC for different ranges */
            rslt = perform_foc_range_test(range, input_axis, &dev);

            if ((j == 1) && (rslt == BMA4_E_OUT_OF_RANGE))
            {
                printf("\n#########   Valid input - Wrong position   #########\n\n");
                bma4_error_codes_print_result("perform_foc_range_test", rslt);
            }
            else if ((j == 0) && (rslt == BMA4_OK))
            {
                printf("\n#########   Valid input - Right position   #########\n\n");
                bma4_error_codes_print_result("perform_foc_range_test", rslt);
            }
            else if ((j == 1) && (rslt == BMA4_OK))
            {
                printf("\n#########   Invalid input - Right position   #########\n\n");
                bma4_error_codes_print_result("perform_foc_range_test", rslt);
            }
            else if ((j == 0) && (rslt == BMA4_E_OUT_OF_RANGE))
            {
                printf("\n#########   Invalid input - Wrong position   #########\n\n");
                bma4_error_codes_print_result("perform_foc_range_test", rslt);
            }
            else if ((j == 0) && (rslt == BMA4_E_OUT_OF_RANGE))
            {
                printf("\n#########   Valid input - Right position   #########\n\n");
                printf("\n#########   Before FOC is better than after FOC   #########\n\n");
                bma4_error_codes_print_result("perform_foc_range_test", rslt);
            }
            else if ((j == 1) && (rslt == BMA4_E_OUT_OF_RANGE))
            {
                printf("\n#########   Invalid input - Right position   #########\n\n");
                printf("\n#########   Before FOC is better than after FOC   #########\n\n");
                bma4_error_codes_print_result("perform_foc_range_test", rslt);
            }
        }

        /* Disable offset compensation */
        rslt = bma4_read_regs(BMA4_NV_CONFIG_ADDR, &data, 1, &dev);
        bma4_error_codes_print_result("bma4_read_regs", rslt);

        data = BMA4_SET_BIT_VAL_0(data, BMA4_NV_ACCEL_OFFSET);

        rslt = bma4_write_regs(BMA4_NV_CONFIG_ADDR, &data, 1, &dev);
        bma4_error_codes_print_result("bma4_write_regs", rslt);
    }

    bma4_coines_deinit();

    return rslt;
}

static int8_t accel_foc_report(uint8_t range,
                               int16_t avg_accel_foc_data,
                               int16_t reference,
                               uint8_t foc_sign,
                               int16_t min_val,
                               int16_t max_val)
{
    int8_t rslt = BMA4_OK;
    int16_t diff_after = 0;

    if (foc_sign == 0)
    {
        if ((avg_accel_foc_data >= (min_val)) && (avg_accel_foc_data <= (max_val)))
        {
            if (avg_accel_foc_data >= reference)
            {
                diff_after = avg_accel_foc_data - reference;
            }
            else
            {
                diff_after = reference - avg_accel_foc_data;
            }

            printf("\n# ********** PASS | Difference = %d **********\n", diff_after);
            printf("\n# Avg_FOC %d in range\n", avg_accel_foc_data);
            rslt = BMA4_OK;
        }
        else
        {
            if (avg_accel_foc_data >= reference)
            {
                diff_after = avg_accel_foc_data - reference;
            }
            else
            {
                diff_after = reference - avg_accel_foc_data;
            }

            printf("\n# ********** FAIL | Difference = %d **********\n", diff_after);
            printf("\n# Avg_FOC %d not in range\n", avg_accel_foc_data);
            rslt = BMA4_E_OUT_OF_RANGE;
        }
    }

    if (foc_sign == 1)
    {
        if ((avg_accel_foc_data <= (min_val)) && (avg_accel_foc_data >= (max_val)))
        {
            if (avg_accel_foc_data <= reference)
            {
                diff_after = avg_accel_foc_data - reference;
            }
            else
            {
                diff_after = reference - avg_accel_foc_data;
            }

            printf("\n# ********** PASS | Difference = %d **********\n", diff_after);
            printf("\n# Avg_FOC %d in range\n", avg_accel_foc_data);
            rslt = BMA4_OK;
        }
        else
        {
            if (avg_accel_foc_data <= reference)
            {
                diff_after = avg_accel_foc_data - reference;
            }
            else
            {
                diff_after = reference - avg_accel_foc_data;
            }

            printf("\n# ********** FAIL | Difference = %d **********\n", diff_after);
            printf("\n# Avg_FOC %d not in range\n", avg_accel_foc_data);
            rslt = BMA4_E_OUT_OF_RANGE;
        }
    }

    return rslt;
}

static void calculate_noise(int8_t matched_axis, struct bma4_accel *accel_foc_data,
                            struct bma4_accel avg_accel_foc_data)
{
    uint16_t variance = 0;
    uint16_t noise_level = 0;
    uint16_t index = 0;

    if (matched_axis == 'X')
    {
        for (index = 0; index < ACCEL_SAMPLE_COUNT; index++)
        {
            variance +=
                ((accel_foc_data[index].x - avg_accel_foc_data.x) * (accel_foc_data[index].x - avg_accel_foc_data.x));
        }
    }
    else if (matched_axis == 'Y')
    {
        for (index = 0; index < ACCEL_SAMPLE_COUNT; index++)
        {
            variance +=
                ((accel_foc_data[index].y - avg_accel_foc_data.y) * (accel_foc_data[index].y - avg_accel_foc_data.y));
        }
    }
    else if (matched_axis == 'Z')
    {
        for (index = 0; index < ACCEL_SAMPLE_COUNT; index++)
        {
            variance +=
                ((accel_foc_data[index].z - avg_accel_foc_data.z) * (accel_foc_data[index].z - avg_accel_foc_data.z));
        }
    }

    noise_level = sqrt(variance);

    printf("\n# ********** NOISE LEVEL = %d **********\n", noise_level);
}

static int8_t verify_accel_foc_data(uint8_t range,
                                    int16_t reference,
                                    int8_t matched_axis,
                                    uint8_t foc_sign,
                                    struct bma4_dev *dev)
{
    int8_t rslt = BMA4_E_INVALID_STATUS;
    uint8_t i;
    uint16_t reg_status = 0;
    int16_t xl, yl, zl;
    int16_t xh, yh, zh;
    uint16_t min_val = 0;
    uint16_t max_val = 0;
    struct bma4_accel accel_foc_data[ACCEL_SAMPLE_COUNT] = { { 0 } };
    struct temp_axes_val temp_foc_data = { 0 };
    struct bma4_accel avg_accel_foc_data = { 0 };
    struct bma4_accel sensor_data = { 0 };

    /* Setting initial values */
    xl = yl = zl = 32767;
    xh = yh = zh = -32768;

    /* Read accelerometer values before/after FOC */
    for (i = 0; i < ACCEL_SAMPLE_COUNT; i++)
    {
        while (1)
        {
            /* To get the data ready interrupt status */
            rslt = bma4_read_int_status(&reg_status, dev);
            bma4_error_codes_print_result("bma4_read_int_status", rslt);

            /* Read accelerometer data based on data ready interrupt */
            if ((rslt == BMA4_OK) && (reg_status & BMA4_ACCEL_DATA_RDY_INT))
            {
                rslt = bma4_read_accel_xyz(&sensor_data, dev);
                bma4_error_codes_print_result("bma4_read_accel_xyz", rslt);

                memcpy(&accel_foc_data[i], &sensor_data, sizeof(struct bma4_accel));

                printf("X[%d] = %5d   Y[%d] = %5d   Z[%d] = %5d\n",
                       i,
                       accel_foc_data[i].x,
                       i,
                       accel_foc_data[i].y,
                       i,
                       accel_foc_data[i].z);

                if (xl > accel_foc_data[i].x)
                {
                    xl = accel_foc_data[i].x;
                }

                if (xh < accel_foc_data[i].x)
                {
                    xh = accel_foc_data[i].x;
                }

                if (yl > accel_foc_data[i].y)
                {
                    yl = accel_foc_data[i].y;
                }

                if (yh < accel_foc_data[i].y)
                {
                    yh = accel_foc_data[i].y;
                }

                if (zl > accel_foc_data[i].z)
                {
                    zl = accel_foc_data[i].z;
                }

                if (zh < accel_foc_data[i].z)
                {
                    zh = accel_foc_data[i].z;
                }

                temp_foc_data.x += accel_foc_data[i].x;
                temp_foc_data.y += accel_foc_data[i].y;
                temp_foc_data.z += accel_foc_data[i].z;
                break;

            }
        }
    }

    /* Taking average values to calculate percentage deviation */
    avg_accel_foc_data.x = (int16_t)(temp_foc_data.x / ACCEL_SAMPLE_COUNT);
    avg_accel_foc_data.y = (int16_t)(temp_foc_data.y / ACCEL_SAMPLE_COUNT);
    avg_accel_foc_data.z = (int16_t)(temp_foc_data.z / ACCEL_SAMPLE_COUNT);

    printf("********* MIN & MAX VALUES ********\n");

    printf("XL = %5d  YL = %5d  ZL = %5d\n", xl, yl, zl);
    printf("XH = %5d  YH = %5d  ZH = %5d\n", xh, yh, zh);

    printf("***** AVERAGE AFTER FOC *****\n");
    printf("Avg-X = %d        Avg-Y = %d        Avg-Z = %d\n",
           avg_accel_foc_data.x,
           avg_accel_foc_data.y,
           avg_accel_foc_data.z);

    /* Calculate noise level */
    calculate_noise(matched_axis, accel_foc_data, avg_accel_foc_data);

    /* "zero-g offset" of accel is +/- 20 mg for all ranges as per datasheet (for 16-bit resolution) */
    if (range == 0)
    {
        /* Min and Max limits for Range 2G */
        min_val = BMA4_16BIT_ACC_2G_MIN_NOISE_LIMIT;
        max_val = BMA4_16BIT_ACC_2G_MAX_NOISE_LIMIT;
    }
    else if (range == 1)
    {
        /* Min and Max limits for Range 4G */
        min_val = BMA4_16BIT_ACC_4G_MIN_NOISE_LIMIT;
        max_val = BMA4_16BIT_ACC_4G_MAX_NOISE_LIMIT;
    }
    else if (range == 2)
    {
        /* Min and Max limits for Range 8G */
        min_val = BMA4_16BIT_ACC_8G_MIN_NOISE_LIMIT;
        max_val = BMA4_16BIT_ACC_8G_MAX_NOISE_LIMIT;
    }
    else if (range == 3)
    {
        /* Min and Max limits for Range 16G */
        min_val = BMA4_16BIT_ACC_16G_MIN_NOISE_LIMIT;
        max_val = BMA4_16BIT_ACC_16G_MAX_NOISE_LIMIT;
    }

    if ((matched_axis == 'X') && (foc_sign == 0))
    {
        rslt = accel_foc_report(range, avg_accel_foc_data.x, reference, foc_sign, min_val, max_val);
        printf("Range : %d  Avg_FOC-X : %d   Reference : %d   Min_Value : %u  Max_Value : %u\n",
               range,
               avg_accel_foc_data.x,
               reference,
               min_val,
               max_val);
    }
    else if ((matched_axis == 'Y') && (foc_sign == 0))
    {
        rslt = accel_foc_report(range, avg_accel_foc_data.y, reference, foc_sign, min_val, max_val);
        printf("Range : %d  Avg_FOC-X : %d   Reference : %d   Min_Value : %u  Max_Value : %u\n",
               range,
               avg_accel_foc_data.y,
               reference,
               min_val,
               max_val);
    }
    else if ((matched_axis == 'Z') && (foc_sign == 0))
    {
        rslt = accel_foc_report(range, avg_accel_foc_data.z, reference, foc_sign, min_val, max_val);
        printf("Range : %d  Avg_FOC-X : %d   Reference : %d   Min_Value : %u  Max_Value : %u\n",
               range,
               avg_accel_foc_data.z,
               reference,
               min_val,
               max_val);
    }
    else if ((matched_axis == 'X') && (foc_sign == 1))
    {
        rslt =
            accel_foc_report(range, avg_accel_foc_data.x, (reference * (-1)), foc_sign, (min_val * (-1)),
                             (max_val * (-1)));
        printf("Range : %d  Avg_FOC-X : %d   Reference : %d   Min_Value : %d  Max_Value : %d\n",
               range,
               avg_accel_foc_data.x,
               (reference * (-1)),
               (min_val * (-1)),
               (max_val * (-1)));
    }
    else if ((matched_axis == 'Y') && (foc_sign == 1))
    {
        rslt =
            accel_foc_report(range, avg_accel_foc_data.y, (reference * (-1)), foc_sign, (min_val * (-1)),
                             (max_val * (-1)));
        printf("Range : %d  Avg_FOC-X : %d   Reference : %d   Min_Value : %d  Max_Value : %d\n",
               range,
               avg_accel_foc_data.y,
               (reference * (-1)),
               (min_val * (-1)),
               (max_val * (-1)));
    }
    else if ((matched_axis == 'Z') && (foc_sign == 1))
    {
        rslt =
            accel_foc_report(range, avg_accel_foc_data.z, (reference * (-1)), foc_sign, (min_val * (-1)),
                             (max_val * (-1)));
        printf("Range : %d  Avg_FOC-X : %d   Reference : %d   Min_Value : %d  Max_Value : %d\n",
               range,
               avg_accel_foc_data.z,
               (reference * (-1)),
               (min_val * (-1)),
               (max_val * (-1)));
    }

    return rslt;
}

/* Perform FOC for different range and resolutions */
static int8_t perform_foc_range_test(uint8_t range, uint8_t input_axis, struct bma4_dev *dev)
{
    int8_t rslt;
    int8_t matched_axis = 0;
    int16_t reference = 0;

    /* Set accel foc axis and it's sign (x, y, z, sign)*/
    struct bma4_accel_foc_g_value g_value_foc = { 0, 0, 0, 0 };

    if (input_axis == 1)
    {
        g_value_foc.x = 1;
        g_value_foc.y = 0;
        g_value_foc.z = 0;
        g_value_foc.sign = 0;
    }
    else if (input_axis == 2)
    {
        g_value_foc.x = 0;
        g_value_foc.y = 1;
        g_value_foc.z = 0;
        g_value_foc.sign = 0;
    }
    else if (input_axis == 3)
    {
        g_value_foc.x = 0;
        g_value_foc.y = 0;
        g_value_foc.z = 1;
        g_value_foc.sign = 0;
    }
    else if (input_axis == 4)
    {
        g_value_foc.x = 1;
        g_value_foc.y = 0;
        g_value_foc.z = 0;
        g_value_foc.sign = 1;
    }
    else if (input_axis == 5)
    {
        g_value_foc.x = 0;
        g_value_foc.y = 1;
        g_value_foc.z = 0;
        g_value_foc.sign = 1;
    }
    else if (input_axis == 6)
    {
        g_value_foc.x = 0;
        g_value_foc.y = 0;
        g_value_foc.z = 1;
        g_value_foc.sign = 1;
    }

    switch (range)
    {
        /* Reference LSB value of 2G */
        case 0:
            reference = BMA4_16BIT_ACC_FOC_2G_REF;
            break;

        /* Reference LSB value of 4G */
        case 1:
            reference = BMA4_16BIT_ACC_FOC_4G_REF;
            break;

        /* Reference LSB value of 8G */
        case 2:
            reference = BMA4_16BIT_ACC_FOC_8G_REF;
            break;

        /* Reference LSB value of 16G */
        case 3:
            reference = BMA4_16BIT_ACC_FOC_16G_REF;
            break;
    }

    if (g_value_foc.x == 1)
    {
        matched_axis = 'X';
    }
    else if (g_value_foc.y == 1)
    {
        matched_axis = 'Y';
    }
    else if (g_value_foc.z == 1)
    {
        matched_axis = 'Z';
    }

    if (g_value_foc.sign == 1)
    {
        printf("MATCHED AXIS is = -%c\n", matched_axis);
    }
    else
    {
        printf("MATCHED AXIS is = %c\n", matched_axis);
    }

    printf("\n\n# Before FOC\n");
    rslt = verify_accel_foc_data(range, reference, matched_axis, g_value_foc.sign, dev);

    printf("\n\n######### Perform Accel FOC #########\n\n");

    /* Perform accelerometer FOC */
    rslt = bma4_perform_accel_foc(&g_value_foc, dev);
    bma4_error_codes_print_result("bma4_perform_accel_foc", rslt);

    /* Delay after performing Accel FOC */
    dev->delay_us(30000, dev->intf_ptr);

    printf("\n\n# After FOC\n");
    rslt = verify_accel_foc_data(range, reference, matched_axis, g_value_foc.sign, dev);

    return rslt;
}
