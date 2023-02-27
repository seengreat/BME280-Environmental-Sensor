/***************************************************************************************
 * Experimental Platform :STM32F103C8T6 + BME280 Environmental Sensor
 * Hardware Connection :BME280 Environmental Sensor -> STM32F103C8T6
 * In hardware SPI mode:
 *			3.3V ->	3.3V					
 *			GND	 ->	GND		
 *      SCK  -> PB13
 *			MOSI -> PB15
 *			MISO -> PB14
 *      CS   -> PB12

 *  In soft IIC mode:
 *			3.3V ->	3.3V					
 *			GND	 ->	GND	
 *      SCK  -> PB10
 *      MOSI -> PB11
 *			MISO -> NC
 *      CS   -> NC 
 *			
 * Library Version :ST_V3.5
 * Author		   : Andy Li
 * Web Site		   :www.seengreat.com
***************************************************************************************/
#include "usart.h"
#include "gpio.h"
#include "delay.h"
#include "spi.h"
#include "bme280.h"
#include "bme280_defs.h"
#include "timer.h"
#include "soft_iic.h"


#ifdef __KERNEL__
#include <sys/ioctl.h>
#include <dev/iicbus/iic.h>
#endif

/******************************************************************************/
/*!                         System header files                               */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "sys.h"

/******************************************************************************
 *                        Own definition                                    */
 //If USEIIC is 1 means use I2C interface, If it is 0,use SPI interface
#define USE_IIC  1

/******************************************************************************/
/*!                               Structures                                  */

/* Structure that contains identifier details used in example */
struct identifier
{
    /* Variable to hold device address */
    uint8_t dev_addr;

    /* Variable that contains file descriptor */
    int8_t fd;
};

/******************************************************************************/
/*!                           Functions                                       */

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr       : Register address.
 *  @param[out] data          : Pointer to the data buffer to store the read data.
 *  @param[in] len            : No of bytes to read.
 *  @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs.
 *
 *  @return Status of execution
 *
 *  @retval 0 -> Success
 *  @retval > 0 -> Failure Info
 *
 */
//int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs.
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *  @return void.
 *
 */
void user_delay_us(uint32_t period, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr       : Register address.
 *  @param[in] data           : Pointer to the data buffer whose value is to be written.
 *  @param[in] len            : No of bytes to write.
 *  @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 *  @return Status of execution
 *
 *  @retval BME280_OK -> Success
 *  @retval BME280_E_COMM_FAIL -> Communication failure.
 *
 */
//int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);

/*!
 * @brief Function for print the temperature, humidity and pressure data.
 *
 * @param[out] comp_data    :   Structure instance of bme280_data
 *
 * @note Sensor data whose can be read
 *
 * sens_list
 * --------------
 * Pressure
 * Temperature
 * Humidity
 *
 */
static void print_sensor_data(struct bme280_data *comp_data);

/*!
 * @brief Function reads temperature, humidity and pressure data in forced mode.
 *
 * @param[in] dev   :   Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 *
 * @retval BME280_OK - Success.
 * @retval BME280_E_NULL_PTR - Error: Null pointer error
 * @retval BME280_E_COMM_FAIL - Error: Communication fail error
 * @retval BME280_E_NVM_COPY_FAILED - Error: NVM copy failed
 *
 */
static int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev);

/*!
 * @brief This function reading the sensor's registers through I2C bus.
 */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
	  int i = 0;
    struct identifier id;
    id = *((struct identifier *)intf_ptr);
		IIC_Start();
		IIC_Send_Byte(id.dev_addr<<1); 
		if(IIC_Wait_Ack())
		{
			IIC_Stop();
			return 1;
		}
		IIC_Send_Byte(reg_addr); 
		IIC_Wait_Ack();
		
		IIC_Start();
		IIC_Send_Byte(id.dev_addr<<1 | 0x01); 
		IIC_Wait_Ack();
		for(i=0;i<len-1;i++)
		{
		   *data++ = IIC_Read_Byte(1);
		}
		*data = IIC_Read_Byte(0);
		IIC_Stop();	
		return 0;

}

int8_t user_spi_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    struct identifier id;
	  int i = 0;
    id = *((struct identifier *)intf_ptr);
	
	  NSS = 1;	 
    NSS = 0;
	  SPI2_ReadWriteByte(reg_addr);
	  for(i=0;i<len;i++)
		{
		    *data++ = SPI2_ReadWriteByte(0XFF);
		}
		NSS = 1;
    return BME280_OK;
}

/*!
 * @brief This function provides the delay for required time (Microseconds) as per the input provided in some of the
 * APIs
 */
void user_delay_us(uint32_t period, void *intf_ptr)
{
    usr_delay_us(period);
}

/*!
 * @brief This function for writing the sensor's registers through I2C bus.
 */
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
	  int i = 0;
	  struct identifier id;
    id = *((struct identifier *)intf_ptr);
		IIC_Start();
		IIC_Send_Byte(id.dev_addr<<1); 
		if(IIC_Wait_Ack())
		{
			IIC_Stop();
			return 1;
		}
		IIC_Send_Byte(reg_addr); 
		IIC_Wait_Ack();
		for(i=0;i<len;i++)
		{
			IIC_Send_Byte(*data++); 
			IIC_Wait_Ack();
		}
		IIC_Stop();
		return 0;

}

/*!
 * @brief This function for writing the sensor's registers through I2C bus.
 */
int8_t user_spi_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    struct identifier id;
    uint8_t i = 0;
		NSS = 1;	
	  NSS = 0;
		SPI2_ReadWriteByte(reg_addr);
		for(i = 0; i < len; i++)
		{
			SPI2_ReadWriteByte(*data++);
		}		
		NSS = 1;

    return BME280_OK;
}

/*!
 * @brief This API used to print the sensor temperature, pressure and humidity data.
 */
void print_sensor_data(struct bme280_data *comp_data)
{
    float temp, press, hum;

#ifdef BME280_FLOAT_ENABLE
    temp = comp_data->temperature;
    press = 0.01 * comp_data->pressure;
    hum = comp_data->humidity;
#else
#ifdef BME280_64BIT_ENABLE
    temp = 0.01f * comp_data->temperature;
    press = 0.0001f * comp_data->pressure;
    hum = 1.0f / 1024.0f * comp_data->humidity;
#else
    temp = 0.01f * comp_data->temperature;
    press = 0.01f * comp_data->pressure;
    hum = 1.0f / 1024.0f * comp_data->humidity;
#endif
#endif
    printf("%0.2lf deg C, %0.2lf hPa, %0.2lf%%\n", temp, press, hum);
}

/*!
 * @brief This API reads the sensor temperature, pressure and humidity data in forced mode.
 */
int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t settings_sel;
    struct bme280_data comp_data;

    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, dev);
    if (rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to set sensor settings (code %+d).", rslt);

        return rslt;
    }

    printf("Temperature, Pressure, Humidity\n");

    /* Continuously stream sensor data */
    while (1)
    {
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
        if (rslt != BME280_OK)
        {
            fprintf(stderr, "Failed to set sensor mode (code %+d).", rslt);
            break;
        }

        /* Wait for the measurement to complete and print data @25Hz */
        dev->delay_us(40000, dev->intf_ptr);
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
        if (rslt != BME280_OK)
        {
            fprintf(stderr, "Failed to get sensor data (code %+d).", rslt);
            break;
        }

        print_sensor_data(&comp_data);
        dev->delay_us(400000, dev->intf_ptr);
    }

    return rslt;
}

/*!
 * @brief This function starts execution of the program.
 */
int main(int argc, char* argv[])
{
    struct bme280_dev dev;
    int8_t rslt = BME280_OK;

    struct identifier id;
	
		SystemInit();
		Uart1Init(115200,0,0);
		printf("-- bme280 demo --\r\n");
		printf("STM32F103C8T6\r\n");
		usr_delay_init();
#if (USE_IIC)
	  printf("use IIC\r\n");
	  IIC_Init();
    /*
     * make sure to select BME280_I2C_ADDR_PRIM
     * or BME280_I2C_ADDR_SEC as needed
     */
    id.dev_addr = BME280_I2C_ADDR_SEC;//0x77

    dev.intf = BME280_I2C_INTF; //use iic bus
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_us = user_delay_us;
#else
	  IO_Init();
	  printf("use SPI\r\n");
    id.dev_addr = 0;
		id.fd = 0;

    dev.intf = BME280_SPI_INTF; //use spi bus
    dev.read = user_spi_read;
    dev.write = user_spi_write;
    dev.delay_us = user_delay_us;
#endif

    /* Update interface pointer with the structure that contains both device address and file descriptor */
    dev.intf_ptr = &id;

    rslt = bme280_init(&dev);
    if (rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to initialize the device (code %+d).\n", rslt);
    }

    rslt = stream_sensor_data_forced_mode(&dev);
    if (rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to stream sensor data (code %+d).\n", rslt);
    }

    return 0;
}
