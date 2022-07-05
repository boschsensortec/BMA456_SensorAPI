/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include <math.h>
#include "bma456w.h"
#include "common.h"

/******************************************************************************/
/*!           Global variable Declaration                                     */

volatile uint8_t interrupt_status = 0;

/******************************************************************************/
/*!                Macro definition                                           */

/*! APP20 Board number */
#define BOARD_MCU_APP20  UINT8_C(0x03)

/*! APP30 Board number */
#define BOARD_MCU_APP30  UINT8_C(0x05)

/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 * @brief This function gets board information
 *
 * @param[out] board       : Board value to determine as APP2.0 or APP3.0
 */
static void get_board_info(uint8_t *board);

/*!
 * @brief This internal API is used to set the interrupt status
 */
static void interrupt_callback(uint32_t param1, uint32_t param2)
{
    (void)param1;
    (void)param2;
    interrupt_status = 1;
}

/******************************************************************************/
/*!            Function                                                       */

int main(void)
{
    struct bma4_dev bma;
    struct bma4_accel_config accel_conf;
    struct bma456w_wrist_wear_wakeup_params setting;
    uint8_t try = 10;
    int8_t rslt;

    uint16_t int_status = 0;
    uint8_t board = 0;
    struct bma4_int_pin_config pin_config = { 0 };

    /* Variable for interrupt line selection*/
    uint8_t int_line;

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

    /* Hardware interrupt configuration */
    int_line = BMA4_INTR1_MAP;

    rslt = bma4_get_int_pin_config(&pin_config, int_line, &bma);
    bma4_error_codes_print_result("bma4_get_int_pin_config status", rslt);

    rslt = bma456w_map_interrupt(int_line, BMA456W_WRIST_WEAR_WAKEUP_INT, BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma456w_map_interrupt status", rslt);

    pin_config.edge_ctrl = BMA4_EDGE_TRIGGER;
    pin_config.output_en = BMA4_OUTPUT_ENABLE;
    pin_config.lvl = BMA4_ACTIVE_HIGH;
    pin_config.od = BMA4_PUSH_PULL;
    pin_config.input_en = BMA4_INPUT_DISABLE;

    rslt = bma4_set_int_pin_config(&pin_config, int_line, &bma);
    bma4_error_codes_print_result("bma4_set_int_pin_config status", rslt);

    /* Get board information */
    get_board_info(&board);

    /*
     * Attach interrupt based on board
     */
    if (board == BOARD_MCU_APP20)
    {
        switch (int_line)
        {
            case BMA4_INTR1_MAP:
                coines_attach_interrupt(COINES_SHUTTLE_PIN_20, interrupt_callback, COINES_PIN_INTERRUPT_RISING_EDGE);
                break;

            case BMA4_INTR2_MAP:
                coines_attach_interrupt(COINES_SHUTTLE_PIN_21, interrupt_callback, COINES_PIN_INTERRUPT_RISING_EDGE);
                break;

            default:
                break;
        }
    }

#if !defined(MCU_APP20)
    else if (board == BOARD_MCU_APP30)
    {
        switch (int_line)
        {
            case BMA4_INTR1_MAP:
                coines_attach_interrupt(COINES_MINI_SHUTTLE_PIN_1_6,
                                        interrupt_callback,
                                        COINES_PIN_INTERRUPT_RISING_EDGE);
                break;

            case BMA4_INTR2_MAP:
                coines_attach_interrupt(COINES_MINI_SHUTTLE_PIN_1_7,
                                        interrupt_callback,
                                        COINES_PIN_INTERRUPT_RISING_EDGE);
                break;

            default:
                break;
        }
    }
#endif

    printf("Do wear movement\n");

    for (;;)
    {
        if (interrupt_status == 1)
        {
            interrupt_status = 0;

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

/*!
 * @brief This function gets board information
 */
static void get_board_info(uint8_t *board)
{
    struct coines_board_info board_info;
    int16_t result;

    result = coines_get_board_info(&board_info);

    if (result == COINES_SUCCESS)
    {
        (*board) = board_info.board;
    }
}
