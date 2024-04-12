#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"

#include "driver/spi.h"
#include "driver/gpio.h"

#include "wifi.h"

#include <stdio.h>
#include <string.h>

extern const unsigned char fontdata_mini_4x6[];

typedef struct {
	unsigned int w, h;
	unsigned char *data;
} bitmap_t;

#define get_bw(w) \
	(((w) + 8 * sizeof(char) - 1) / (8 * sizeof(char)))

int bitmap_bit_blt8(bitmap_t *dest, int x, int y,
	unsigned int w, unsigned int h, const bitmap_t *src)
{
	unsigned int i;
	unsigned char mask_in, mask_out;

	unsigned int dest_bw = get_bw(dest->w);

	if (x >= dest->w)
		return 0;

	mask_in = ((unsigned char)~0) >> (x % 8);
	if (x % 8 + w < 8)
		mask_in &= ~((1 << (8 - (x % 8 + w))) - 1);
	mask_out = ~mask_in;

	for (i = 0; i < h; i++) {
		unsigned int line_index = y + i;

		unsigned char *line_start = &dest->data[line_index * dest_bw];
		unsigned char *line_end = line_start + dest_bw;
		unsigned char *out = line_start + x / 8;
		const unsigned char *in = src->data + i * ((src->w + 8 - 1) / 8);

		int j;

		unsigned char val = (*in >> x % 8) & mask_in;

		if (line_index >= dest->h)
			break;

		*out = (*out & mask_out) | val;

		out++;

		if (!(x % 8))
			in++;

		for (j = ((int)w - (int)(8 - x % 8)) / (int)8; j > 0 && out < line_end; j--) {
			unsigned char val = *in++;
			if (x % 8)
				val = (val << (8 - x % 8)) | (*in >> (x % 8));

			*out = val;

			out++;
		}

		if ((x + w) % 8 && (x / 8) < ((x + w) / 8) && out < line_end) {
			unsigned char mask_out = ((unsigned char)~0) >> ((x + w) % 8);
			unsigned char mask_in = ~mask_out;

			unsigned char val = *in++;
			if (x % 8)
				val = val << (8 - x % 8) | (*in >> (x % 8));
			val &= mask_in;

			*out = (*out & mask_out) | val;
		}
	}

	return 0;
}

static void draw_char(bitmap_t *fb, int x, int y, uint8_t c)
{
	bitmap_t charbmp;

	charbmp.w = 4;
	charbmp.h = 6;
	charbmp.data = (uint8_t *)fontdata_mini_4x6 + c * 6;

	bitmap_bit_blt8(fb, x, y, 4, 6, &charbmp);
}

void spi_event_cb(int event, void *arg)
{
	if (event == SPI_TRANS_DONE_EVENT) {
		/* latch data */
		gpio_set_level(GPIO_NUM_15, 0);
		gpio_set_level(GPIO_NUM_15, 1);
	}
}

static uint8_t rev8(uint8_t x)
{
	uint32_t u = x * 0x00020202;
	uint32_t m = 0x01044010;
	uint32_t s = u & m;
	uint32_t t = (u << 2) & (m << 1);
	return (0x01001001 * (s + t)) >> 24;
}

static void update_display(bitmap_t *fb)
{
	spi_trans_t trans;
	size_t fb_size = fb->h * ((fb->w + 7) >> 3);
	uint32_t buffer[(fb_size + 3) >> 2];
	int j;

	/* map frame buffer bits to display bits */
	for (j = 0; j < fb_size >> 3; j++) {
		int offset = j << 3;
		((uint8_t *)buffer)[offset + 2] = rev8(fb->data[offset + 0]);
		((uint8_t *)buffer)[offset + 0] = rev8(fb->data[offset + 1]);
		((uint8_t *)buffer)[offset + 3] = fb->data[offset + 2];
		((uint8_t *)buffer)[offset + 1] = fb->data[offset + 3];
		((uint8_t *)buffer)[offset + 4] = rev8(fb->data[offset + 4]);
		((uint8_t *)buffer)[offset + 6] = rev8(fb->data[offset + 5]);
		((uint8_t *)buffer)[offset + 5] = fb->data[offset + 6];
		((uint8_t *)buffer)[offset + 7] = fb->data[offset + 7];
	}

	trans.bits.cmd = 0;
	trans.bits.addr = 0;
	trans.bits.mosi = 32 * ((fb_size + 3) >> 2);
	trans.bits.miso = 0;
	trans.cmd = NULL;
	trans.addr = NULL;
	trans.mosi = (uint32_t *)buffer;
	trans.miso = NULL;
	spi_trans(HSPI_HOST, &trans);
}

static void gpio_irq_handler_wps_pb(void *arg)
{
	SemaphoreHandle_t *wps_pb_sema = arg;
	xSemaphoreGiveFromISR(wps_pb_sema, pdFALSE);
}

static uint8_t fb_data[32];
static bitmap_t fb = {
		.w = 16, .h = 16,
		.data = fb_data,
};

static void got_ip_event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data)
{
	ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
	uint32_t ip = ntohl(event->ip_info.ip.addr) & 0xff;
	printf("got ip: %s", ip4addr_ntoa(&event->ip_info.ip));

	draw_char(&fb, 0, 0, '0' + ((ip / 100) % 10));
	draw_char(&fb, 4, 0, '0' + ((ip / 10) % 10));
	draw_char(&fb, 8, 0, '0' + ((ip / 1) % 10));
}

void app_main()
{
	SemaphoreHandle_t *wps_pb_sema;
	int i = 0;

	spi_config_t spi_config = {
			.interface.val = SPI_DEFAULT_INTERFACE,
			.intr_enable.val = SPI_MASTER_DEFAULT_INTR_ENABLE,
			.event_cb = spi_event_cb,
			.mode = SPI_MASTER_MODE,
			.clk_div = SPI_2MHz_DIV,
	};

	gpio_config_t gpio_output_config = {
			.pin_bit_mask = GPIO_Pin_12 | GPIO_Pin_15,
			.mode = GPIO_MODE_OUTPUT,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_DISABLE,
	};

	gpio_config_t gpio_irq_config = {
			.pin_bit_mask = GPIO_Pin_2,
			.mode = GPIO_MODE_INPUT,
			.pull_up_en = GPIO_PULLUP_ENABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_NEGEDGE,
	};

	/* set initial level of GPIOs */
	gpio_set_level(GPIO_NUM_12, 0);	/* output enable */
	gpio_set_level(GPIO_NUM_15, 1); /* latch */

	gpio_config(&gpio_output_config);
	gpio_config(&gpio_irq_config);

	wps_pb_sema = xSemaphoreCreateBinary();
	gpio_install_isr_service(0);
	gpio_isr_handler_add(GPIO_NUM_2, gpio_irq_handler_wps_pb, wps_pb_sema);

	spi_config.interface.miso_en = 0;
	spi_config.interface.cs_en = 0;
	spi_init(HSPI_HOST, &spi_config);

	nvs_flash_init();

	tcpip_adapter_init();
	esp_event_loop_create_default();
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &got_ip_event_handler, NULL));

	wifi_init();

	memset(fb_data, 0, sizeof(fb_data));

	/* first test: 'Hello world', short form */
	draw_char(&fb, 0, 0, 'H');
	draw_char(&fb, 4, 0, 'e');
	draw_char(&fb, 8, 0, 'l');
	draw_char(&fb, 12, 0, 'l');
	draw_char(&fb, 0, 5, 'w');
	draw_char(&fb, 4, 5, 'o');
	draw_char(&fb, 8, 5, 'r');
	draw_char(&fb, 12, 5, 'l');

	for (;;) {
		if (xSemaphoreTake(wps_pb_sema, 0) == pdTRUE) {
			printf("WPS push button pressed\n");
			wifi_start_wps();
		}

		/* second test: show a counter */
		draw_char(&fb, 0, 11, '0' + (i / 1000) % 10);
		draw_char(&fb, 4, 11, '0' + ((i / 100) % 10));
		draw_char(&fb, 8, 11, '0' + ((i / 10) % 10));
		draw_char(&fb, 12, 11, '0' + ((i / 1) % 10));

		i++;

		update_display(&fb);

		vTaskDelay(200 / portTICK_PERIOD_MS);
	}
}
