#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include <esp_err.h>
#include <esp_log.h>
#include <string.h>
static const char *TAG = "[DHT11_SSD]";

#include <dht_espidf.h>
#include "ssd1306.h"
//#include "font8x8_basic.h"

/*
 You have to set this config value with menuconfig
 CONFIG_INTERFACE

 for i2c
 CONFIG_MODEL
 CONFIG_SDA_GPIO
 CONFIG_SCL_GPIO
 CONFIG_RESET_GPIO

 for SPI
 CONFIG_CS_GPIO
 CONFIG_DC_GPIO
 CONFIG_RESET_GPIO
*/

#define BLINK_GPIO 2 //CONFIG_BLINK_GPIO
#define DHT_IO 16 //27

//void app_main();

struct _dataQ {
  double fahrenheit;
  double humidity;
};

struct _dataQ data;

QueueHandle_t dataQueue;

SSD1306_t dev;
char buf[128];

void DHT_task(void *pvParameter) {
  struct dht_reading dht_data;
  
  while (1) {
    dht_result_t res = read_dht_sensor_data((gpio_num_t)DHT_IO, DHT11, &dht_data);

    if (res != DHT_OK) {
      ESP_LOGW(TAG, "DHT sensor reading failed");
    } else {
      //double fahrenheit = (dht_data.temperature * 1.8f) + 32.0f;
      //double humidity = dht_data.humidity;
      data.fahrenheit = (dht_data.temperature * 1.8f) + 32.0f;
      data.humidity = dht_data.humidity;

      ESP_LOGI(TAG, "DHT sensor reading: %2.2f° / %f", data.fahrenheit, data.humidity);

      if (xQueueSend(dataQueue, (void *)&data, (TickType_t)10) == pdPASS) ESP_LOGI(TAG, "Sent dht_data to Queue.");
      else ESP_LOGI(TAG, "Failed to send dht_data to Queue!!");


    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

static uint8_t s_led_state = 0;

// CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

void blink_led_task(void *pvParameter) {
    /* Configure the peripheral according to the LED type */
    configure_led();

    while (1) {
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        /* Toggle the LED state */
        s_led_state = !s_led_state;
//        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

int app_main() {

#if CONFIG_I2C_INTERFACE
  ESP_LOGI(TAG, "INTERFACE is i2c");
  ESP_LOGI(TAG, "CONFIG_SDA_GPIO=%d", CONFIG_SDA_GPIO);
  ESP_LOGI(TAG, "CONFIG_SCL_GPIO=%d", CONFIG_SCL_GPIO);
  ESP_LOGI(TAG, "CONFIG_RESET_GPIO=%d", CONFIG_RESET_GPIO);
  i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
#endif // CONFIG_I2C_INTERFACE

#if CONFIG_SPI_INTERFACE
  ESP_LOGI(TAG, "INTERFACE is SPI");
  ESP_LOGI(TAG, "CONFIG_MOSI_GPIO=%d", CONFIG_MOSI_GPIO);
  ESP_LOGI(TAG, "CONFIG_SCLK_GPIO=%d", CONFIG_SCLK_GPIO);
  ESP_LOGI(TAG, "CONFIG_CS_GPIO=%d", CONFIG_CS_GPIO);
  ESP_LOGI(TAG, "CONFIG_DC_GPIO=%d", CONFIG_DC_GPIO);
  ESP_LOGI(TAG, "CONFIG_RESET_GPIO=%d", CONFIG_RESET_GPIO);
  spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO);
#endif // CONFIG_SPI_INTERFACE

#if CONFIG_FLIP
  dev._flip = true;
  ESP_LOGW(TAG, "Flip upside down");
#endif

#if CONFIG_SSD1306_128x64
  ESP_LOGI(TAG, "Panel is 128x64");
  ssd1306_init(&dev, 128, 64);
#endif // CONFIG_SSD1306_128x64
#if CONFIG_SSD1306_128x32
  ESP_LOGI(TAG, "Panel is 128x32");
  ssd1306_init(&dev, 128, 32);
#endif // CONFIG_SSD1306_128x32

  dataQueue = xQueueCreate(1, sizeof(&data));
  xTaskCreate(&DHT_task, "DHT_task", 2048, NULL, 5, NULL);
  xTaskCreate(&blink_led_task, "DHT_task", 2048, NULL, 5, NULL);

  ssd1306_clear_screen(&dev, false);
  ssd1306_contrast(&dev, 0xff);
  ssd1306_display_text_x3(&dev, 0, "Hello", 5, false);
  
 // 		ssd1306_display_text(dev, page, space, sizeof(space), true);

  
  ssd1306_display_text(&dev, 7, "Hello World!!", 12, true);

  vTaskDelay(3000 / portTICK_PERIOD_MS);
  ssd1306_clear_screen(&dev, false);

  while (1) {

    if (xQueueReceive(dataQueue, &data, portMAX_DELAY) == pdTRUE) {
      ESP_LOGI(TAG, "Received dht_data from Queue.");
      ESP_LOGI(TAG, "!!DHT sensor returned: %2.2f° / %f", data.fahrenheit, data.humidity);

      ssd1306_clear_screen(&dev, false);
      sprintf(buf, "Temp.= %2.2f F", data.fahrenheit);
      ssd1306_display_text(&dev, 0, buf, strlen(buf), false);
      sprintf(buf, "Humidity= %2.0f %%", data.humidity);
      ssd1306_display_text(&dev, 2, buf, strlen(buf), false);
    } else 
      ESP_LOGI(TAG, "Failed to get dht_data from Queue!!");

    vTaskDelay((3000 / portTICK_PERIOD_MS) * 3);
  }
  return 0;
}
