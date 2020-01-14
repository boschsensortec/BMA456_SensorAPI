# BMA456 Sensor API

## Introduction
This package contains Bosch Sensortec's BMA456 sensor API

## Sensor API integration

Inside your project, include files *bma4_defs.h*, *bma4.h*, *bma4.c*, *bma456.c* and *bma456.h*.

If you are using an auxiliary magnetometer, either the BMM150 or the AKM9916, include to respective source and include, *aux_bmm150.c* and *aux_bmm150.h*, or, *aux_akm9916.c* and *aux_akm9916.h*.

In your code, include header *bma456.h*

``` c
#include "bma456.h"
```

## User guide

### Initializing sequence
First application setup examples algorithms:
After correct power up by setting the correct voltage to the appropriate external pins, the
BMA456 enters automatically into the Power On Reset (POR) sequence. In order to properly
make use of the BMA456, certain steps from host processor side are needed. 
a. Reading the chip id.
b. Performing initialization sequence.
c. Checking the correct status of the interrupt feature engine.

``` c
uint16_t main(void) {
    uint16_t rslt = BMA4_OK;
    uint8_t init_seq_status = 0;
    
    /* Declare an instance of the BMA456 device */
    struct bma4_dev dev;
    
    /* Modify the parameters */
    dev.dev_addr        = BMA4_I2C_ADDR_PRIMARY;
    dev.interface       = BMA4_I2C_INTERFACE;
    dev.bus_read        = USER_i2c_read;
    dev.bus_write       = USER_i2c_write;
    dev.delay           = USER_delay_ms;
    dev.read_write_len  = 8;
    dev.resolution      = 12;
    dev.feature_len     = BMA456_FEATURE_SIZE;
    
    /* a. Reading the chip id. */
    rslt |= bma456_init(&dev);
    
    /* b. Performing initialization sequence. 
       c. Checking the correct status of the initialization sequence.
    */
    rslt |= bma456_write_config_file(&dev);
    
    return rslt;
}
```

### Sensor API initialization for the I2C protocol

``` c
uint16_t main(void) {
    uint16_t rslt = BMA4_OK;
    
    /* Declare an instance of the BMA456 device */
    struct bma4_dev dev;
    
    /* Modify the parameters */
    dev.dev_addr = BMA4_I2C_ADDR_PRIMARY;
    dev.interface = BMA4_I2C_INTERFACE;
    dev.bus_read = USER_i2c_read;
    dev.bus_write = USER_i2c_write;
    dev.delay = USER_delay_ms;
    dev.read_write_len = 8;
    
    /* Initialize the instance */
    rslt |= bma456_init(&dev);
    
    return rslt;
}
```

### Configuring the accelerometer

``` c
uint16_t main(void) {
    uint16_t rslt = BMA4_OK;

    /* Initialize the device instance as per the initialization example */
    
    /* Enable the accelerometer */
    bma4_set_accel_enable(ACCEL_ENABLE, &dev);
    
    /* Declare an accelerometer configuration structure */
    struct bma4_accel_config accel_conf;
    
    /* Assign the desired settings */
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
    accel_conf.range = BMA4_ACCEL_RANGE_2G;
    accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
    
    /* Set the configuration */
    rslt |= bma4_set_accel_config(&accel_conf, &dev);
    
    return rslt;
}
```

### Reading out the accelerometer data

``` c
uint16_t main(void) {
    uint16_t rslt = BMA4_OK;

    /* Initialize the device instance as per the initialization example */
    
    /* Configure the accelerometer */
    
    /* Declare an instance of the sensor data structure */
    struct bma4_accel sens_data;
    
    /* Loop forever */
    while (1) {
        /* Read the sensor data into the sensor data instance */
        rslt |= bma4_read_accel_xyz(&sens_data, &dev);
        
        /* Exit the program in case of a failure */
        if (rslt != BMA4_OK)
            return rslt;
        
        /* Use the data */
        printf("X: %d, Y: %d, Z: %d\n", sens_data.x, sens_data.y, sens_data.z);
        
        /* Pause for 10ms, 100Hz output data rate */
        USER_delay_ms(10);
    }
    
    return rslt;
}
```

### Configuring reading out sensor data from the FIFO buffer 

``` c
uint16_t main(void) {
    uint16_t rslt = BMA4_OK;

    /* Initialize the device instance as per the initialization example */
    
    /* Configure the accelerometer */
    
    /* Setup and configure the FIFO buffer */
    /* Declare memory to store the raw FIFO buffer information */   
    uint8_t fifo_buff[255];
    struct bma4_fifo_frame fifo_frame;
    
    /* Modify the FIFO buffer instance and link to the device instance */
    fifo_frame.data = fifo_buff;
    fifo_frame.length = 255;
    fifo_frame.fifo_data_enable = BMA4_ENABLE;
    fifo_frame.fifo_header_enable = BMA4_ENABLE;
    dev.fifo = &fifo_frame;
    
    /* Disable the advanced power save mode to configure the FIFO buffer */
    rslt |= bma4_set_advance_power_save(BMA4_DISABLE, &dev);
    
    /* Configure the FIFO buffer */
    rslt |= bma4_set_fifo_config((BMA4_FIFO_ACCEL | BMA4_FIFO_HEADER), BMA4_ENABLE, &dev);
    
    /* Declare instances of the sensor data structure */
    struct bma4_accel sens_data[36]; // 255 bytes / ~7bytes per frame = 36 instances
    uint16_t n_instances, i;
    
    /* Loop forever */
    while (1) {
        /* Read data from the sensor FIFO buffer */
        rslt |= bma4_read_fifo_data(&dev); // Read FIFO data
        
        /* Exit the program in case of a failure */
        if (rslt != BMA4_OK)
            return rslt;
        
        /* Reset the maximum number of required sensor data instances */
        n_instances = 36;
        
        /* Parse the FIFO until there are less frames than requested */
        while (n_instances == 36) {
            /* Parse the FIFO buffer and extract required number of accelerometer data frames */
            rslt |= bma4_extract_accel(sens_data, &n_instances, &dev);
            
            /* Exit the program in case of a failure */
            if (rslt != BMA4_OK)
                return rslt;
            
            /* Use the accelerometer data frames */
            for (i = 0; i < n_instances; i++)
                printf("X:%d, Y:%d, Z:%d\n", sens_data[i].x, sens_data[i].y, sens_data[i].z);
        }
        
        // At 100Hz, The FIFO buffer will have 36 frames ready in (1 / 100) * 36 = ~0.36s
        USER_delay_ms(360); 
    }
    
    return rslt;
}
```

### Enabling and mapping line interrupt to that of BMA456 sensor interrupt

This example shows mapping of a line interrupt with two feature interrupts simultaneously in
variant BMA456.
Note: There are two interrupts - feature interrupt and hardware interrupts. You can map more 
than one interrupt with a single line interrupt. If a feature interrupt is mapped with a line 
interrupt, one can  map the other line interrupt with that of hardware interrupt. This example
can be done for other variants as well.

``` c 
uint16_t main(void)
{
    uint16_t result = BMA4_OK;
     /* Variable to define two interrupt lines */
    uint8_t  int_line[2] = {BMA4_INTR1_MAP, BMA4_INTR2_MAP};
    /* Variable to define feature interrupts to be mapped*/
    uint16_t int_map  = (BMA456_STEP_CNTR_INT | BMA456_SINGLE_TAP_INT);    
    
    /* Initialize the device instance as per the initialization example */
    
    /* Configure the accelerometer as per the example */
    
    /* Mapping line interrupt 1 with that of two sensor feature interrupts -
     * Step counter and single tap interrupt */
    result =  bma456_map_interrupt(int_line[0], int_map, BMA4_ENABLE, &dev);
    
    if(result == BMA4_OK) {
       printf("Interrupt mapping successful\r\n");
    }
    else {
        printf("Interrupt mapping failed\r\n");
        printf("Error code = %d\r\n", result);
    }
    
    return result;
}
```

### Reading interrupt status register
This example is in continuation of the  previous example: Enabling and mapping line interrupt.
After the interrupts are mapped, interrupt status register is read in an interrupt service 
routine to perform the corresponding tasks on an interrupt. 

``` c
void interrupt_handler(void)
{ 
    uint16_t result = BMA4_OK;
    /* Define a variable to get the status */
    uint16_t int_status = 0;    
   
    /* Read the interrupt status register */
    result =  bma456_read_int_status(&int_status, &dev)

    if (result == BMA4_OK) {
  
        if (int_status & BMA456_STEP_CNTR_INT) {
   
            /* Call the function to be performed on step counter interrupt */
        
        } 
        
        if (int_status & BMA456_DOUBLE_TAP_INT) {
    
            /* Call the function to be performed on double tap interrupt */ 
        }       
    }
}
```
### Configuring the auxiliary sensor BMM150

### Initialization of auxiliary interface to access BMM150

```
/* Structure declaration */
struct bma4_dev *dev;

/* Switch on the the auxiliary interface mode */
dev->aux_config.if_mode = BMA4_ENABLE_AUX_IF_MODE;
/* Set the I2C device address of auxiliary device */
/* Device address of BMM150 */
dev->aux_config.aux_dev_addr = BMA4_I2C_BMM150_ADDR;
/* Set aux interface to manual mode */
dev->aux_config.manual_enable = BMA4_MANUAL_ENABLE;
/* Set the number of bytes for burst read */
dev->aux_config.burst_read_length = BMA4_AUX_READ_LEN_0;

/* Initialize the auxiliary interface */
bma4_aux_interface_init(dev);
```
### Reading of data from auxiliary sensor BMM150  
Initialize the auxiliary interface as shown above. Set the power mode to sleep mode and the operation mode 
to forced mode. Then validate the sensor by reading the chip id.

```  
uint8_t aux_data[5] = {0};
uint8_t aux_chip_id = 0;
uint8_t aux_write;

/* Enable the power control bit for sleep mode in 0x4B register of BMM150 */
aux_write = 0x01;
result |= bma4_aux_write(0x4B, &aux_write, 1, dev);

/* Enable forced mode in 0x4C register of BMM150 */
aux_write = 0x02;
result |= bma4_aux_write(0x4c, &aux_write, 1, dev);

/* Read the chip-id of the auxiliary sensor */    
result = bma4_aux_read(BMA4_AUX_CHIP_ID_ADDR, &aux_chip_id, 1, dev);
if(aux_chip_id == BMM150_CHIP_ID) {
    result = bma4_aux_read(aux_read_addr, aux_data, 1, dev);
}    
```

### Writing of data from auxiliary sensor BMM150 
Initialize the auxiliary interface as shown above. Set the power mode to sleep mode and the operation mode 
to forced mode. Then validate the sensor by reading the chip id before writing..

```    
uint8_t aux_write_data[5] = {0xFF, 0xAA, 0xFD, 0x78, 0x99};
uint8_t aux_chip_id = 0;
uint8_t aux_write;

/* Enable the power control bit for sleep mode in 0x4B register of BMM150 */
aux_write = 0x01;
result |= bma4_aux_write(0x4B, &aux_write, 1, dev);

/* Enable forced mode in 0x4C register of BMM150 */
aux_write = 0x02;
result |= bma4_aux_write(0x4c, &aux_write, 1, dev);
        
/* Read the chip-id of the auxiliary sensor */    
result = bma4_aux_read(BMA4_AUX_CHIP_ID_ADDR, &aux_chip_id, 1, dev);
if(aux_chip_id == BMM150_CHIP_ID) {
    result = bma4_aux_write(0x50, aux_write_data, 4, dev);
}
    
```

### Accessing of auxiliary sensor BMM150 with the help of BMM150 APIs via BMA4 secondary interface.
User has to create a  wrapper function over bma4_aux_read and bma4_aux_write in order to map with 
bmi150 read and write

```
/* Structure declaration */
struct bmm150_dev bmm150;

/* function declaration */
int8_t bma4_user_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);
int8_t bma4_user_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);

/* Update bmm150 structure */
bmm150.read = bma4_user_aux_read;
bmm150.write = bma4_user_aux_write;
bmm150.id = BMM150_DEFAULT_I2C_ADDRESS;
bmm150.delay_ms = delay_ms;
bmm150.interface = BMM150_I2C_INTF;

/* Initialize bmm150 sensor */
bmm150_init(&bmm150);
```

### Wrapper functions for auxiliary read and write
    
```
/* Wrapper function to map bmm150.read */
int8_t bma4_user_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len)
{
       int8_t result;

       if (dev->aux_config.aux_dev_addr == id)
              result = bma4_aux_read(reg_addr, aux_data, len, dev);

       return result;
}

/* Wrapper function to map bmm150.write */
int8_t bma4_user_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len)
{
       int8_t result;

       if (dev->aux_config.aux_dev_addr == id)
              result = bma4_aux_write(reg_addr, aux_data, len, dev);

       return result;
}
```

### Get temperature from BMA4 sensor.
A scaling factor of 1000 has to be used to convert the read temperature back to its 
float value.

```
void main()
{
    int8_t rslt;
    int32_t get_temp_C = 0;
    int32_t get_temp_F = 0;
    int32_t get_temp_K = 0;
    float actual_temp = 0;
    
    /* Initialize the device instance as per the initialization example */
    
    /* Initialize bma4 sensor to get the chip id */
    rslt = bma4_init(&dev);
    
    if(rslt == BMA4_OK) {
        if(dev.chip_id == BMA456_CHIP_ID) {
            rslt = bma456_init(&dev);
            
            if(rslt == BMA4_OK) {
                /* Reset the accelerometer */
                bma4_set_command_register(0xB6, &dev);
                dev.delay(1);
            
                /* Enable the accelerometer */
                rslt = bma4_set_accel_enable(1, &dev);
                dev.delay(10);
                
                /* Get temperature in degree C */
                rslt = bma4_get_temperature(&get_temp_C, BMA4_DEG, &dev);
                /* Get temperature in degree F */
                rslt = bma4_get_temperature(&get_temp_F, BMA4_FAHREN, &dev);
                /* Get temperature in degree K */
                rslt = bma4_get_temperature(&get_temp_K, BMA4_KELVIN, &dev);
            
                /* Divide the temperature read with the scaling factor to get 
                the actual temperature */
                if(rslt == BMA4_OK) {
                    actual_temp = (float)get_temp_C / (float)BMA4_SCALE_TEMP;
                    printf("Actual temperature in degree celsius is %10.2f degrees C\r\n", actual_temp);
                    
                    actual_temp = (float)get_temp_F / (float)BMA4_SCALE_TEMP;
                    printf("Actual temperature in degree fahranheit is %10.2f degrees F\r\n", actual_temp);
                    
                    actual_temp = (float)get_temp_K / (float)BMA4_SCALE_TEMP;
                    printf("Actual temperature in degree kelvin is %10.2f degrees K\r\n", actual_temp);
                    
                    /* 0x80 - temp read from the register and 23 is the ambient temp added.
                     * If the temp read from register is 0x80, it means no valid
                     * information is available */
                    if(((get_temp_C - 23) / BMA4_SCALE_TEMP) == 0x80) {
                        printf("No valid temperature information available\r\n");
                    }
               }
            }                
        }
    }
}
```
### Activity(still/walking/running/invalid) recognition.

``` 
/* Enable the activity feature */
rslt = bma456_feature_enable(BMA456_STEP_ACT, 1, &dev);

/* Map the activity out interupt to INT pin1 */
bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_ACTIVITY_INT, BMA4_ENABLE, &dev);
    
```
### ISR for activity interrupt recognition

``` c
uint16_t int_status = 0;
uint8_t activity_out = 0;
 
/* Read the Interrupt status reg. */
bma456_read_int_status(&int_status, &dev);

if(int_status & BMA456_ACTIVITY_INT) {  
    /* Read the activity out register for user's activity*/
    bma456_activity_output(&activity_out, &dev);
    
    /* compare the output of activity register and perform the action as per requirement */
    switch (activity_out) {
    case BMA456_USER_STATIONARY:
         /* User state is stationary */
         /* Perform the respective action according to this state */
         break;
    case BMA456_USER_WALKING:
         /* User state is walking */
         /* Perform the respective action according to this state */
         break;
    case BMA456_USER_RUNNING:
         /* User state is running */
         /* Perform the respective action according to this state */
         break;
    case BMA456_STATE_INVALID:
        /* User state is invalid */
        /* Perform the respective action according to this state */
         break;
    }
}
```

### Get any-motion interrupt

``` c
/* Variable to get error results */
uint16_t rslt = 0;
/* Variable to define dummy byte */
uint8_t dummy_read = 0;
/* Structure to define any-motion */
struct bma456_any_no_mot_config any_mot = {0};
/* Random value for while loop to get the any motion interrupt */
uint8_t try = 50;
/* Variable to store the interrupt status of any motion */
uint16_t int_status = 0;

/* Soft-reset */
rslt |= bma4_set_command_register(0xB6, &dev);
dev.delay(10);

/* Perform a dummy read in order to switch to SPI. Kindly see the data
sheet */
if (dev.interface == BMA4_SPI_INTERFACE) {
    rslt |= bma4_read_regs(0, &dummy_read, 1, &dev);
}

/* Initialize BMA456 */
rslt |= bma456_init(&dev);
if (rslt != BMA4_OK) {
    printf("Initilalization fail\n");
    return rslt;
} else {
    printf("Initilalization success\r\n");
}

/* Enable accelerometer */
rslt = bma4_set_accel_enable(1, &dev);
if (rslt != BMA4_OK) {
    printf("Accelerometer not enabled\r\n");
    return rslt;
} else {
    printf("Accelerometer enabled\r\n");
}

/* Load configuration file */
rslt = bma456_write_config_file(&dev);
if (rslt != BMA4_OK) {
    printf("Load configuration fail\r\n");
    return rslt;
} else {
    printf("Load configuration successful\r\n");
}

/* Get any-motion configuration to get the default values */
rslt = bma456_get_any_mot_config(&any_mot, &dev);
if (rslt != BMA4_OK) {
    printf("Get any-motion configuration fail\r\n");
    return rslt;
} else {
    printf("Get any-motion configuration successful\r\n");
    printf("axis_en = %x\n", any_mot.axes_en);
    printf("duration = %x\n", any_mot.duration);
    printf("threshold = %x\r\n", any_mot.threshold);
}

/* Enable any-motion for all axes */
any_mot.axes_en = BMA456_EN_ALL_AXIS;

/* Set the new configuration */
rslt = bma456_set_any_mot_config(&any_mot, &dev);
if (rslt != BMA4_OK) {
    printf("Set any-motion configuration fail\r\n");
    return rslt;
} else {
    printf("Set any-motion configuration successful\r\n");
}

/* Get any-motion configuration to validate */
rslt = bma456_get_any_mot_config(&any_mot, &dev);
if (rslt != BMA4_OK) {
    printf("Get any-motion configuration fail\r\n");
    return rslt;
} else {
    printf("Get any-motion configuration successful\r\n");

    printf("axis_en = %x\n", any_mot.axes_en);
    printf("duration = %x\n", any_mot.duration);
    printf("threshold = %x\r\n", any_mot.threshold);
}

/* Map interrupt for any-motion */
rslt = bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_ANY_MOT_INT, BMA4_ENABLE, &dev);
if (rslt != BMA4_OK) {
    printf("Map interrupt fail\r\n");
    return rslt;
} else {
    printf("Map interrupt successful\r\n");
}

/* Loop over to get the interrupt status of any-motion */
printf("Shake the sensor in any direction\n");
while (try--) {
    /* Read the interrupt status */
    rslt = bma456_read_int_status(&int_status, &dev);
    if (rslt == BMA4_OK) {
        /* check if any/no motion interrupt is triggered */
        if (int_status & BMA456_ANY_MOT_INT) {
            printf("Any-Motion interrupt received\n");
            break;
        }
    }

    int_status = 0;

    dev.delay(200);
}

return rslt;
```
> _Note_:The above example can be referred for getting no-motion interrupt.

### Get step counter output

``` c
/* Variable to define errors */
uint16_t rslt = 0;
/* Variable to get the step counter output */
uint32_t step_out = 0;
/* Variable to get the interrupt status */
uint16_t int_status = 0;
/* Variable to loop */
uint8_t try = 50;
/* Variable to define dummy byte */
uint8_t dummy_read = 0;

/* Soft-reset */
rslt |= bma4_set_command_register(0xB6, &dev);
dev.delay(10);

/* Perform a dummy read in order to switch to SPI. Kindly see the data
sheet */
if (dev.interface == BMA4_SPI_INTERFACE) {
    rslt |= bma4_read_regs(0, &dummy_read, 1, &dev);
}

/* Initialize BMA456 */
rslt |= bma456_init(&dev);
if (rslt != BMA4_OK) {
    printf("Initialization fail\n");
    return rslt;
} else {
    printf("Initialization success\r\n");
}

/* Enable accelerometer */
rslt = bma4_set_accel_enable(1, &dev);
if (rslt != BMA4_OK) {
    printf("Accelerometer not enabled\r\n");
    return rslt;
} else {
    printf("Accelerometer enabled\r\n");
}

/* Load configuration file */
rslt = bma456_write_config_file(&dev);
if (rslt != BMA4_OK) {
    printf("Load configuration fail\r\n");
    return rslt;
} else {
    printf("Load configuration successful\r\n");
}

/* Enable step counter */
rslt = bma456_feature_enable(BMA456_STEP_CNTR, 1, &dev);
if (rslt != BMA4_OK) {
    printf("Step counter not enabled\r\n");
    return rslt;
} else {
    printf("Step counter enabled\r\n");
}

/* Map step counter interrupt */
rslt = bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_STEP_CNTR_INT, 1, &dev);
if (rslt != BMA4_OK) {
    printf("Error code: %d\n", rslt);
    return rslt;
}

/* Set water-mark level 1 */
rslt = bma456_step_counter_set_watermark(1, &dev);
if (rslt != BMA4_OK) {
    printf("Error code: %d\n", rslt);
    return rslt;
}

printf("Move the board\n");
while (try) {
    /* Get the interrupt status */
    rslt = bma456_read_int_status(&int_status, &dev);
    if (rslt == BMA4_OK) {
        /* check if step counter interrupt is triggered */
        if (int_status & BMA456_STEP_CNTR_INT) {
            printf("\nStep counter interrupt received\n");

            /* On interrupt, Get step counter output */
            rslt = bma456_step_counter_output(&step_out, &dev);
            if (rslt == BMA4_OK) {
                printf("\nThe step counter output is %u\r\n", step_out);                
            } else {
                break;
            }
        }
    } else {
        break;
    }

    try--;

    dev.delay(1000);
}

printf("Status return: %d\n", rslt);
```



     