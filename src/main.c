/********************************************************************************
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 ********************************************************************************/
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <modem/lte_lc.h>
/* STEP 2.3 - Include the header file for the MQTT Library*/
#include <zephyr/net/mqtt.h>

#include "mqtt_connection.h"

/********************************************************************************
 *
 ********************************************************************************/
#define OPEN_VALVE 		1
#define CLOSE_VALVE 	0

#define HIGH 			1
#define LOW 			0

#define ON 				1
#define OFF 			0

#define MOTION_DETECTOR 13 /*  */

#define WATER_VALVE 	16 /*  */
#define BUZZER 			28	   /* sig pin of the buzzer */
#define LIGHTWELL_RED 	29
#define LIGHTWELL_GREEN 30
#define LIGHTWELL_BLUE 	31

#define MAX_OUTPUTS 	5
#define MAX_INPUTS 		1

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 	1000

/* Option 1: by node label */
#define MY_GPIO0 DT_NODELABEL(gpio0)

/********************************************************************************
 *
 ********************************************************************************/
// const struct device *gpio_dev;
const struct device *gpio_dev = DEVICE_DT_GET(MY_GPIO0);
//
struct k_timer my_timer;
// extern void my_expiry_function(struct k_timer *timer_id);
static struct gpio_callback motion_cb_data;

/********************************************************************************
 *
 ********************************************************************************/
enum led_id_t
{
	LIGHTWELL_RED_LED,
	LIGHTWELL_GREEN_LED,
	LIGHTWELL_BLUE_LED
};

/********************************************************************************
 *
 ********************************************************************************/
static uint8_t buzzer_state = OFF;
static uint32_t output_gpio[MAX_OUTPUTS] = {WATER_VALVE, BUZZER, LIGHTWELL_RED,
											LIGHTWELL_GREEN, LIGHTWELL_BLUE};
static uint32_t input_gpio[MAX_INPUTS] = {MOTION_DETECTOR};

/********************************************************************************
 *
 ********************************************************************************/
/* The mqtt client struct */
static struct mqtt_client client;
/* File descriptor */
static struct pollfd fds;

static K_SEM_DEFINE(lte_connected, 0, 1);

LOG_MODULE_REGISTER(a_smart_water_tap_v1, LOG_LEVEL_INF);

/********************************************************************************
 * Play tone
 ********************************************************************************/
void beep_buzzer(int tone, int duration);
void motion_detected(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void my_expiry_function(struct k_timer *timer_id);
void blink(uint32_t sleep_ms, enum led_id_t id);
void blink0(void);
void blink1(void);
void blink2(void);
void configuer_all_input(void);
void configuer_all_output(void);

/********************************************************************************
 *
 ********************************************************************************/
void blink(uint32_t sleep_ms, enum led_id_t id)
{
	int ret;

	switch (id)
	{
	case LIGHTWELL_RED_LED:
		ret = gpio_pin_toggle(gpio_dev, LIGHTWELL_RED);
		if (ret < 0)
		{
			return;
		}
		k_msleep(sleep_ms);
		break;

	case LIGHTWELL_GREEN_LED:
		ret = gpio_pin_toggle(gpio_dev, LIGHTWELL_GREEN);
		if (ret < 0)
		{
			return;
		}
		k_msleep(sleep_ms);
		break;

	case LIGHTWELL_BLUE_LED:
		ret = gpio_pin_toggle(gpio_dev, LIGHTWELL_BLUE);
		if (ret < 0)
		{
			return;
		}
		k_msleep(sleep_ms);
		break;

	default:
		break;
	}
}

/********************************************************************************
 * Play tone
 ********************************************************************************/
void beep_buzzer(int tone, int duration)
{
	for (long i = 0; i < duration * 1000L; i += tone * 2)
	{
		gpio_pin_set(gpio_dev, BUZZER, HIGH);
		k_msleep(tone);
		gpio_pin_set(gpio_dev, BUZZER, LOW);
		k_msleep(tone);
	}
}

/********************************************************************************
 *
 ********************************************************************************/
void blink0(void)
{
		blink(100, LIGHTWELL_RED_LED);
}

/********************************************************************************
 *
 ********************************************************************************/
void blink1(void)
{
	blink(1000, LIGHTWELL_GREEN_LED);
}

/********************************************************************************
 *
 ********************************************************************************/
void blink2(void)
{
	blink(5000, LIGHTWELL_BLUE_LED);
}

/********************************************************************************
 *
 ********************************************************************************/
void configuer_all_output(void)
{
	int err;
	for (uint32_t i = 0; i < MAX_OUTPUTS; i++)
	{
		if (!device_is_ready(gpio_dev))
		{
			return;
		}

		err = gpio_pin_configure(gpio_dev, output_gpio[i], GPIO_OUTPUT_INACTIVE);
		if (err < 0)
		{
			return;
		}
	}
}

/********************************************************************************
 *
 ********************************************************************************/
void configuer_all_input(void)
{
	int err;
	for (uint32_t i = 0; i < MAX_INPUTS; i++)
	{
		if (!device_is_ready(gpio_dev))
		{
			return;
		}

		err = gpio_pin_configure(gpio_dev, input_gpio[i], GPIO_INPUT | GPIO_PULL_DOWN);
		if (err < 0)
		{
			return;
		}
	}
}

/********************************************************************************
 * Define the callback function
 ********************************************************************************/
void motion_detected(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	buzzer_state = ON;
	gpio_pin_set(gpio_dev, WATER_VALVE, OPEN_VALVE);
	/* start periodic timer that expires once every second */
	k_timer_start(&my_timer, K_SECONDS(10), K_NO_WAIT);
}

/********************************************************************************
 * Define a variable of type static struct gpio_callback
 ********************************************************************************/
void my_expiry_function(struct k_timer *timer_id)
{
	buzzer_state = OFF;
	gpio_pin_set(gpio_dev, WATER_VALVE, CLOSE_VALVE);
}

/********************************************************************************
 *
 ********************************************************************************/
static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type)
	{
	case LTE_LC_EVT_NW_REG_STATUS:
		if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
			(evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING))
		{
			break;
		}
		LOG_INF("Network registration status: %s",
				evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ? "Connected - home network" : "Connected - roaming");
		k_sem_give(&lte_connected);
		break;
	case LTE_LC_EVT_RRC_UPDATE:
		LOG_INF("RRC mode: %s", evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ? "Connected" : "Idle");
		break;
	default:
		break;
	}
}

/********************************************************************************
 *
 ********************************************************************************/
static void modem_configure(void)
{
	LOG_INF("Connecting to LTE network");

	int err = lte_lc_init_and_connect_async(lte_handler);
	if (err)
	{
		LOG_INF("Modem could not be configured, error: %d", err);
		return;
	}
	k_sem_take(&lte_connected, K_FOREVER);
	LOG_INF("Connected to LTE network");
	dk_set_led_on(DK_LED2);
}

/********************************************************************************
 *
 ********************************************************************************/
static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	switch (has_changed)
	{
	case DK_BTN1_MSK:
		/* STEP 7.2 - When button 1 is pressed, call data_publish() to publish a message */
		if (button_state & DK_BTN1_MSK)
		{
			int err = data_publish(&client, MQTT_QOS_1_AT_LEAST_ONCE,
								   CONFIG_BUTTON_EVENT_PUBLISH_MSG, sizeof(CONFIG_BUTTON_EVENT_PUBLISH_MSG) - 1);
			if (err)
			{
				LOG_INF("Failed to send message, %d", err);
				return;
			}
		}
		break;
	}
}

/********************************************************************************
 *
 ********************************************************************************/
void main(void)
{
	int err;
	uint32_t connect_attempt = 0;
	k_msleep(SLEEP_TIME_MS * 10);
	LOG_INF("A Smart Water Tap Leakage Controller IoT Project/n/r");

	configuer_all_output();
	configuer_all_input();

	/* Configure the interrupt on the button's pin */
	err = gpio_pin_interrupt_configure(gpio_dev, MOTION_DETECTOR, GPIO_INT_EDGE_TO_ACTIVE);
	if (err < 0)
	{
		return;
	}

	/* Initialize the static struct gpio_callback variable */
	gpio_init_callback(&motion_cb_data, motion_detected, BIT(13));
	/* Add the callback function by calling gpio_add_callback() */
	gpio_add_callback(gpio_dev, &motion_cb_data);

	//
	k_timer_init(&my_timer, my_expiry_function, NULL);

	//
	while (1)
	{
		LOG_INF("Beep Buzzer!");
		beep_buzzer(3, 1);
		k_msleep(SLEEP_TIME_MS);
	}

	
	if (dk_leds_init() != 0)
	{
		LOG_ERR("Failed to initialize the LED library");
	}

	modem_configure();

	if (dk_buttons_init(button_handler) != 0)
	{
		LOG_ERR("Failed to initialize the buttons library");
	}

	err = client_init(&client);
	if (err)
	{
		LOG_ERR("Failed to initialize MQTT client: %d", err);
		return;
	}

do_connect:
	if (connect_attempt++ > 0)
	{
		LOG_INF("Reconnecting in %d seconds...",
				CONFIG_MQTT_RECONNECT_DELAY_S);
		k_sleep(K_SECONDS(CONFIG_MQTT_RECONNECT_DELAY_S));
	}
	err = mqtt_connect(&client);
	if (err)
	{
		LOG_ERR("Error in mqtt_connect: %d", err);
		goto do_connect;
	}

	err = fds_init(&client, &fds);
	if (err)
	{
		LOG_ERR("Error in fds_init: %d", err);
		return;
	}

	while (1)
	{
		err = poll(&fds, 1, mqtt_keepalive_time_left(&client));
		if (err < 0)
		{
			LOG_ERR("Error in poll(): %d", errno);
			break;
		}

		err = mqtt_live(&client);
		if ((err != 0) && (err != -EAGAIN))
		{
			LOG_ERR("Error in mqtt_live: %d", err);
			break;
		}

		if ((fds.revents & POLLIN) == POLLIN)
		{
			err = mqtt_input(&client);
			if (err != 0)
			{
				LOG_ERR("Error in mqtt_input: %d", err);
				break;
			}
		}

		if ((fds.revents & POLLERR) == POLLERR)
		{
			LOG_ERR("POLLERR");
			break;
		}

		if ((fds.revents & POLLNVAL) == POLLNVAL)
		{
			LOG_ERR("POLLNVAL");
			break;
		}
	}

	LOG_INF("Disconnecting MQTT client");

	err = mqtt_disconnect(&client);
	if (err)
	{
		LOG_ERR("Could not disconnect MQTT client: %d", err);
	}
	goto do_connect;
}