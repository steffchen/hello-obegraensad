/*
 * wifi.c
 *
 *  Created on: Apr 12, 2024
 *      Author: steff
 */

#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_wps.h"

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data)
{
	wifi_config_t wifi_config;

	switch (event_id) {
	case WIFI_EVENT_STA_START:
		printf("WIFI station started\n");
		break;
	case WIFI_EVENT_STA_DISCONNECTED:
		printf("WIFI station disconnected\n");
		break;
	case WIFI_EVENT_STA_WPS_ER_SUCCESS:
		printf("WIFI station WPS successful");

		/* save acquired configuration to flash */
		ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &wifi_config));
		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

		ESP_ERROR_CHECK(esp_wifi_wps_disable());
		ESP_ERROR_CHECK(esp_wifi_connect());
		break;
	case WIFI_EVENT_STA_WPS_ER_FAILED:
		printf("WIFI station WPS failed\n");

		ESP_ERROR_CHECK(esp_wifi_wps_disable());
		break;
	case WIFI_EVENT_STA_WPS_ER_TIMEOUT:
		printf("WIFI station WPS timeout\n");

		ESP_ERROR_CHECK(esp_wifi_wps_disable());
		break;
	}
}

void wifi_init(void)
{
	wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();

	ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_start());

	/* try to connect with the configuration from flash */
	ESP_ERROR_CHECK(esp_wifi_connect());
}

void wifi_start_wps(void)
{
	esp_wps_config_t config = WPS_CONFIG_INIT_DEFAULT(WPS_TYPE_PBC);

	ESP_ERROR_CHECK(esp_wifi_wps_enable(&config));
	ESP_ERROR_CHECK(esp_wifi_wps_start(0));
}
