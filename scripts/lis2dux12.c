/* Library required:
    1. wiring Pi
    2. STM library for lis2dux12
        https://github.com/STMicroelectronics/lis2dux12-pid?tab=readme-ov-file
 */


 #include <string.h>
 #include <stdio.h>
 #include <stdint.h>
 #include <wiringPiSPI.h>
 #include "lis2dux12_reg.h"
 
 #define SPI_CHANNEL 0    // SPI0 (CS0)
 #define SPI_SPEED 1000000 // 1 MHz SPI speed
 
 static stmdev_ctx_t dev_ctx;
 
 static uint8_t id;
 static lis2dux12_status_t status;
 static lis2dux12_xl_data_t data_xl;
 static lis2dux12_md_t md;
 
 
 // Initialize SPI for Raspberry Pi
 void platform_init(void) {
     if (wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED) < 0) {
         perror("SPI Setup Failed");
     }
 }
 
 // Delay function (usleep for microseconds)
 void platform_delay(uint32_t ms) {
     usleep(ms * 1000);
 }
 
 // Send buffer to console
 void tx_com(uint8_t *tx_buffer, uint16_t len) {
     write(STDOUT_FILENO, tx_buffer, len);
 }
 
 int main() {
     // Initialize SPI
     platform_init();
 
     // Initialize LIS2DUX12 driver context
     dev_ctx.write_reg = platform_write;
     dev_ctx.read_reg = platform_read;
     dev_ctx.mdelay = platform_delay;
     dev_ctx.handle = NULL; // Not needed for WiringPi
 
     // Wait for sensor boot
     platform_delay(10);
 
     // Wake up the sensor
     lis2dux12_exit_deep_power_down(&dev_ctx);
 
     // Read device ID
     lis2dux12_device_id_get(&dev_ctx, &id);
     printf("LIS2DUX12 ID: 0x%X\n", id);
 
     if (id != LIS2DUX12_ID) {
         printf("Error: Device not found!\n");
         return -1;
     }
 
     // Reset sensor
     lis2dux12_init_set(&dev_ctx, LIS2DUX12_RESET);
     do {
         lis2dux12_status_get(&dev_ctx, &status);
     } while (status.sw_reset);
 
     // Configure sensor (set BDU, IF_INC, enable accelerometer)
     lis2dux12_init_set(&dev_ctx, LIS2DUX12_SENSOR_ONLY_ON);
 
     // Set Output Data Rate (ODR), Full Scale (FS)
     md.fs = LIS2DUX12_4g;
     md.bw = LIS2DUX12_ODR_div_4;
     md.odr = LIS2DUX12_25Hz_LP;
     lis2dux12_mode_set(&dev_ctx, &md);
 
     // Read data in a loop
     while (1) {
         lis2dux12_status_get(&dev_ctx, &status);
         if (status.drdy) {
             lis2dux12_xl_data_get(&dev_ctx, &md, &data_xl);
 
             printf("Acceleration [mg]: X=%4.2f Y=%4.2f Z=%4.2f\n",
                    data_xl.mg[0], data_xl.mg[1], data_xl.mg[2]);
 
             platform_delay(100); // Wait 100ms
         }
     }
 
     return 0;
 }
 