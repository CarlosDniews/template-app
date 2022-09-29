#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "bmp280.h"
#include "bmp280_defs.h"
#include "bmp280.c"

//semaphores
SemaphoreHandle_t semaphore = NULL;

//define pins to I2C setup
#define SDA_GPIO 21
#define SCL_GPIO 22
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY //frequency to the master 

//global
struct bmp280_uncomp_data ucomp_data;
int32_t temp32;
double temp; 
uint32_t pres32, pres64;
double pres;
double reading;
struct bmp280_dev bmp;
struct bmp280_config config;
int8_t bmpstatus;

//initializing master using I2C
void i2c_master_init() {
    i2c_config_t i2c_config = {
            .mode = I2C_MODE_MASTER, //master mod
            .sda_io_num = SDA_GPIO, //config GPIO to SDA
            .scl_io_num = SCL_GPIO, //config GPIO to CLK
            .sda_pullup_en = GPIO_PULLUP_ENABLE, //pull up SDA
            .scl_pullup_en = GPIO_PULLUP_ENABLE, //pull up CLK
            .master.clk_speed = 100000 //clock's frequency to the project
 
     };


i2c_param_config(I2C_NUM_0, &i2c_config); //initializes the driver configuration for an I2C port
i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0); //install driver by calling the function

//Verify that the I2C slave is working properly
   esp_err_t esp_retval;  //return value variable
 
   i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //create a command link 
    i2c_master_start(cmd); //start master with command
    i2c_master_write_byte(cmd, (BMP280_I2C_ADDR_PRIM << 1) | I2C_MASTER_WRITE, true); // byte address is provided as an argument of this function call
    i2c_master_stop(cmd); //Stop 
    esp_retval = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS); //Trigger the execution of the command link by I2C controller
    if (esp_retval != ESP_OK) {
        printf("I2C slave NOT working or wrong I2C slave address - error (%i)", esp_retval);
        // LABEL
 
    }
i2c_cmd_link_delete(cmd);
 
 
}

 //Implement the I2C read routine according to the target machine.
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
 
    int8_t iError; //return error variable 
    esp_err_t esp_err; 

    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();  //create a command link 
    i2c_master_start(cmd_handle); //start master with command
    i2c_master_write_byte(cmd_handle, (i2c_addr << 1) | I2C_MASTER_WRITE, true); 
    i2c_master_write_byte(cmd_handle, reg_addr, true);
    i2c_master_start(cmd_handle); //start master with command
    i2c_master_write_byte(cmd_handle, (i2c_addr << 1) | I2C_MASTER_READ, true);
    if (length > 1) {
        i2c_master_read(cmd_handle, reg_data, length - 1, I2C_MASTER_ACK); // read command to the commands list. I2C ack for each byte read
    }
    i2c_master_read_byte(cmd_handle, reg_data + length - 1, I2C_MASTER_NACK); // read command to the commands list. I2C nack for each byte read
    i2c_master_stop(cmd_handle); //Stop 
    esp_err = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 1000 / portTICK_PERIOD_MS);
 
    if (esp_err == ESP_OK) {
        iError = 0;
    } else {
        iError = -1; //return error 
    }
 
    i2c_cmd_link_delete(cmd_handle); //release the resources used by the command link
 
   
    return iError; //return error variable 
}

 //Implement the I2C write routine according to the target machine.
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
   
    int8_t iError; //return error variable 

    esp_err_t esp_err;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, reg_addr, true);
    i2c_master_write(cmd_handle, reg_data, length,true);
    i2c_master_stop(cmd_handle);
    esp_err = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 1000 / portTICK_PERIOD_MS);
    if (esp_err == ESP_OK) {
        iError = 0;
    } else {
        iError = -1;
    }
    i2c_cmd_link_delete(cmd_handle);
    return iError; //return error variable 
}

//Implement the delay routine according to the target machine 
void delay_ms(uint32_t period_ms) {
         ets_delay_us(period_ms * 1000);
}

//print errors if occur  
void print_rslt(const char api_name[], int8_t rslt) {
 
 
    if (rslt != BMP280_OK) {
        printf("%s\t", api_name);
        if (rslt == BMP280_E_NULL_PTR) {
            printf("Error [%d] : Null pointer error\r\n", rslt);
        } else if (rslt == BMP280_E_COMM_FAIL) {
            printf("Error [%d] : Bus communication failed\r\n", rslt);
        } else if (rslt == BMP280_E_IMPLAUS_TEMP) {
            printf("Error [%d] : Invalid Temperature\r\n", rslt);
        } else if (rslt == BMP280_E_DEV_NOT_FOUND) {
            printf("Error [%d] : Device not found\r\n", rslt);
        } else {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }else{
        printf("%s\t", api_name);
        printf(" BMP280 status [%d]\n ",rslt);
    }
}

//bmp initialization
void bmp280_inicializar()
{
    

    /* Map the delay function pointer with the function responsible for implementing the delay */
    bmp.delay_ms = delay_ms;

    /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
    bmp.dev_id = BMP280_I2C_ADDR_PRIM;

    /* Select the interface mode as I2C */
    bmp.intf = BMP280_I2C_INTF;

    /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
    bmp.read = i2c_reg_read;
    bmp.write = i2c_reg_write;

    bmpstatus = bmp280_init(&bmp);
    print_rslt("bmp280_init status", bmpstatus);
    bmpstatus = bmp280_get_config(&config, &bmp);
    print_rslt("bmp280_get_config status", bmpstatus);

    /* configuring the temperature oversampling, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    config.filter = BMP280_FILTER_COEFF_2;


     //below part should be checked according to datasheet 
    /* Temperature oversampling set at 1x */
    config.os_temp = BMP280_OS_1X;

    /* Pressure over sampling  (pressure measurement) */
    config.os_pres = BMP280_OS_8X;
    
    
    /* Setting the output data rate as 1HZ(1000ms) */
    config.odr = BMP280_ODR_1000_MS;
    bmpstatus = bmp280_set_config(&config, &bmp);
    print_rslt(" bmp280_set_config status", bmpstatus);
    bmpstatus = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);

    print_rslt(" bmp280_set_power_mode status", bmpstatus);
}

//bmp temperature
void bmp280_temperature(void *pvParamters) {

    for (;;)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        /* Reading the raw data from sensor */
        bmpstatus = bmp280_get_uncomp_data(&ucomp_data, &bmp);
        /* Getting the 32 bit compensated temperature */
        //bmpstatus = bmp280_get_comp_temp_32bit(&temp32, ucomp_data.uncomp_temp, &bmp);

        /* Getting the compensated temperature as floating point value */
        if(xSemaphoreTake(semaphore,portMAX_DELAY) == pdTRUE)
        bmpstatus = bmp280_get_comp_temp_double(&reading, ucomp_data.uncomp_temp, &bmp);  
        
        /* Sleep time between measurements = BMP280_ODR_1000_MS */
        bmp.delay_ms(1);
        
        // add a delay of reasonable  ms within the while loop
       vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void bmp280_pressure(void *pvParamters) {
    for (;;)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        /* Reading the raw data from sensor */
        bmpstatus = bmp280_get_uncomp_data(&ucomp_data, &bmp);
        /* Getting the compensated pressure using 32 bit precision */
        //bmpstatus = bmp280_get_comp_pres_32bit(&pres32, ucomp_data.uncomp_press, &bmp);
       
        /* Getting the compensated pressure using 64 bit precision */
        //bmpstatus = bmp280_get_comp_pres_64bit(&pres64, ucomp_data.uncomp_press, &bmp);

        /* Getting the compensated pressure as floating point value */
        if(xSemaphoreTake(semaphore,portMAX_DELAY) == pdTRUE)
        bmpstatus = bmp280_get_comp_pres_double(&reading, ucomp_data.uncomp_press, &bmp);
        
        /* Sleep time between measurements = BMP280_ODR_1000_MS */
        bmp.delay_ms(1);
        
        // add a delay of reasonable  ms within the while loop
       vTaskDelay(4000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

}


void bmp280_print(void *parametros)
{
    
    for (;;)
    {
      vTaskDelay(50 / portTICK_PERIOD_MS);
      //printf("T32: %d, T: %f \r\n", temp32, temp);
      /*printf("P32: %d, P64: %d, P64N: %d, P: %f\r\n",
               pres32,
               pres64,
               pres64 / 256,
               pres);*/
    if(reading < 100) printf("T: ");
    else printf("P: ");
    xSemaphoreGive(semaphore);
    printf("%f\n",reading);
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
        
    }
}


void app_main(void)
{
     ESP_ERROR_CHECK(nvs_flash_init());
   
   
    i2c_master_init();

    bmp280_inicializar();
    semaphore = xSemaphoreCreateBinary();
    xTaskCreate(&bmp280_temperature, "bmp280_temp", 2048, NULL, 6, NULL);
    xTaskCreate(&bmp280_pressure, "bmp280_pres", 2048, NULL, 6, NULL);
    xTaskCreate(&bmp280_print, "bmp280_print", 2048, NULL, 6, NULL);

}