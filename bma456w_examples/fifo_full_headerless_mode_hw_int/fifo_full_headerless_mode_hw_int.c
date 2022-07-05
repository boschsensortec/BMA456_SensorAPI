/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include <math.h>
#include "bma456w.h"
#include "common.h"
#include "coines.h"

/******************************************************************************/
/*!           Global variable Declaration                                     */

volatile uint8_t interrupt_status = 0;

/******************************************************************************/
/*!                Macro definition                                           */

/*! Buffer size allocated to store raw FIFO data */
#define BMA456W_FIFO_RAW_DATA_BUFFER_SIZE  UINT16_C(1024)

/*! Length of data to be read from FIFO */
#define BMA456W_FIFO_RAW_DATA_USER_LENGTH  UINT16_C(1024)

/*! Number of accel frames to be extracted from FIFO
 * Calculation:
 * fifo_buffer = 1024, accel_frame_len = 6.
 * fifo_accel_frame_count = (1024 / 6) = 170 frames
 */
#define BMA456W_FIFO_ACCEL_FRAME_COUNT     UINT8_C(170)

/*! APP20 Board number */
#define BOARD_MCU_APP20                    UINT8_C(0x03)

/*! APP30 Board number */
#define BOARD_MCU_APP30                    UINT8_C(0x05)

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

/* This function starts the execution of program */
int main(void)
{
    /* Status of API are returned to this variable */
    int8_t rslt;

    /* Accelerometer configuration structure */
    struct bma4_accel_config acc_conf = { 0 };

    /* Sensor initialization configuration */
    struct bma4_dev dev = { 0 };

    /* Number of accelerometer frames */
    uint16_t accel_length;

    /* Variable to idx bytes */
    uint16_t idx = 0;

    /* Variable to maintain count of loop to run FIFO read */
    uint8_t loop = 1;

    /* Number of bytes of FIFO data
     * NOTE : Dummy byte (for SPI Interface) required for FIFO data read must be given as part of array size
     */
    uint8_t fifo_data[BMA456W_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };

    /* Array of accelerometer frames -> Total bytes =
     * 170 * (6 axes bytes(+/- x,y,z)) = 1020 bytes */
    struct bma4_accel fifo_accel_data[BMA456W_FIFO_ACCEL_FRAME_COUNT] = { { 0 } };

    /* Initialize FIFO frame structure */
    struct bma4_fifo_frame fifoframe = { 0 };

    /* Variable that contains interrupt status value */
    uint16_t int_status = 0;

    /* Variable to hold the length of FIFO data */
    uint16_t fifo_length = 0;

    struct bma4_int_pin_config pin_config = { 0 };
    uint8_t board = 0;

    /* Variable for interrupt line selection*/
    uint8_t int_line;

    /* Interface reference is given as a parameter
     *         For I2C : BMA4_I2C_INTF
     *         For SPI : BMA4_SPI_INTF
     * Variant information given as parameter - BMA45X_VARIANT
     */
    rslt = bma4_interface_init(&dev, BMA4_I2C_INTF, BMA45X_VARIANT);
    bma4_error_codes_print_result("bma4_interface_init", rslt);

    /* Initialize BMA456W */
    rslt = bma456w_init(&dev);
    bma4_error_codes_print_result("bma456w_init status", rslt);

    /* Accelerometer configuration settings */
    acc_conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
    acc_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
    acc_conf.range = BMA4_ACCEL_RANGE_2G;
    acc_conf.perf_mode = BMA4_CIC_AVG_MODE;

    /* Set the accel configurations */
    rslt = bma4_set_accel_config(&acc_conf, &dev);
    bma4_error_codes_print_result("bma4_set_accel_config status", rslt);

    /* NOTE : Enable accel after set of configurations */
    rslt = bma4_set_accel_enable(1, &dev);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    /* Disabling advance power save mode as FIFO data is not accessible in advance low power mode */
    rslt = bma4_set_advance_power_save(BMA4_DISABLE, &dev);
    bma4_error_codes_print_result("bma4_set_advance_power_save status", rslt);

    /* Clear FIFO configuration register */
    rslt = bma4_set_fifo_config(BMA4_FIFO_ALL, BMA4_DISABLE, &dev);
    bma4_error_codes_print_result("bma4_set_fifo_config disable status", rslt);

    /* Set FIFO configuration by enabling accel.
     * NOTE 1: The header mode is enabled by default.
     * NOTE 2: By default the FIFO operating mode is FIFO mode. */
    rslt = bma4_set_fifo_config(BMA4_FIFO_ACCEL, BMA4_ENABLE, &dev);
    bma4_error_codes_print_result("bma4_set_fifo_config enable status", rslt);

    /* Update FIFO structure */
    fifoframe.data = fifo_data;
    fifoframe.length = BMA456W_FIFO_RAW_DATA_USER_LENGTH;

    /* To enable headerless mode, disable the header. */
    rslt = bma4_set_fifo_config(BMA4_FIFO_HEADER, BMA4_DISABLE, &dev);
    bma4_error_codes_print_result("bma4_set_fifo_config status", rslt);

    printf("FIFO is configured in headerless mode\n");

    /* Hardware interrupt configuration */
    int_line = BMA4_INTR2_MAP;

    rslt = bma456w_map_interrupt(int_line, BMA4_FIFO_FULL_INT, BMA4_ENABLE, &dev);
    bma4_error_codes_print_result("bma456w_map_interrupt status", rslt);

    pin_config.edge_ctrl = BMA4_EDGE_TRIGGER;
    pin_config.output_en = BMA4_OUTPUT_ENABLE;
    pin_config.lvl = BMA4_ACTIVE_HIGH;
    pin_config.od = BMA4_PUSH_PULL;
    pin_config.input_en = BMA4_INPUT_DISABLE;

    rslt = bma4_set_int_pin_config(&pin_config, int_line, &dev);
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

    while (loop <= 10)
    {
        if (interrupt_status == 1)
        {
            interrupt_status = 0;

            rslt = bma456w_read_int_status(&int_status, &dev);
            bma4_error_codes_print_result("bma4_read_int_status", rslt);

            if ((rslt == BMA4_OK) && (int_status & BMA4_FIFO_FULL_INT))
            {
                printf("\nIteration : %d\n", loop);

                rslt = bma4_get_fifo_length(&fifo_length, &dev);
                bma4_error_codes_print_result("bma4_get_fifo_length status", rslt);

                printf("FIFO data bytes available : %d\n", fifo_length);
                printf("FIFO data bytes requested : %d\n", fifoframe.length);

                /* Read FIFO data */
                rslt = bma4_read_fifo_data(&fifoframe, &dev);
                bma4_error_codes_print_result("bma4_read_fifo_data status", rslt);

                accel_length = BMA456W_FIFO_ACCEL_FRAME_COUNT;

                if (rslt == BMA4_OK)
                {
                    printf("Requested data frames before parsing: %d\n", accel_length);

                    /* Parse the FIFO data to extract accelerometer data from the FIFO buffer */
                    rslt = bma4_extract_accel(fifo_accel_data, &accel_length, &fifoframe, &dev);
                    printf("Parsed accelerometer data frames: %d\n", accel_length);

                    printf("ACCEL, X, Y, Z\n");

                    /* Print the parsed accelerometer data from the FIFO buffer */
                    for (idx = 0; idx < accel_length; idx++)
                    {
                        printf("%d, %d, %d, %d\n", idx, fifo_accel_data[idx].x, fifo_accel_data[idx].y,
                               fifo_accel_data[idx].z);
                    }
                }

                loop++;
            }
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
