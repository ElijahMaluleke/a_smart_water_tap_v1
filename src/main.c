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
#define OPEN_VALVE 			1
#define CLOSE_VALVE 		0
#define VALVE_OPENED		1
#define VALVE_CLOSED		0

#define HIGH 				1
#define LOW 				0

#define ON 					1
#define OFF 				0

#define MOTION_DETECTOR 13 /*  */

#define WATER_VALVE 		16 /*  */
#define BUZZER 				28	   /* sig pin of the buzzer */
#define LIGHTWELL_RED_LED 	29
#define LIGHTWELL_GREEN_LED 30
#define LIGHTWELL_BLUE_LED 	31

#define MAX_OUTPUTS 		5
#define MAX_INPUTS 			1

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 		1000
#define SLEEP_TIME_S		1000
#define SLEEP_TIME_HALF_S	500
#define SLEEP_TIME_QUOTA_S	250

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
	LIGHTWELL_RED = 0,
	LIGHTWELL_GREEN = 1,
	LIGHTWELL_BLUE = 2
};

/********************************************************************************
 *
 ********************************************************************************/
static uint8_t valve_status = VALVE_CLOSED;
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
void blink_leds(uint32_t sleep_ms, enum led_id_t id, uint32_t numOfBlinks);
void configuer_all_inputs(void);
void configuer_all_outputs(void);

/********************************************************************************
 *
 ********************************************************************************/
void blink_leds(uint32_t sleep_ms, enum led_id_t id, uint32_t numOfBlinks)
{
	int err;
	uint32_t i = 0;

	switch(id)
	{
		case LIGHTWELL_RED:
		{	
			for(i = 0; i < numOfBlinks; i++) 
			{	
				LOG_INF("LIGHTWELL RED LED");
				dk_set_led_on(DK_LED1);
				if (err < 0)
				{
					return;
				}
				k_msleep(sleep_ms);
				dk_set_led_off(DK_LED1);
				if (err < 0)
				{
					return;
				}
				k_msleep(sleep_ms);
			}
		}
		break;
		
		case LIGHTWELL_GREEN:
		{	
			for(i = 0; i < numOfBlinks; i++)
			{	
				LOG_INF("LIGHTWELL GREEN LED");
				dk_set_led_on(DK_LED2);
				if (err < 0)
				{
					return;
				}
				k_msleep(sleep_ms);
				dk_set_led_off(DK_LED2);
				if(err < 0)
				{
					return;
				}
				k_msleep(sleep_ms);
			}
		break;
		}
		case LIGHTWELL_BLUE:
		{
			for(i = 0; i < numOfBlinks; i++)
			{
				LOG_INF("LIGHTWELL BLUE LED");
				dk_set_led_on(DK_LED3);
				if (err < 0)
				{
					return;
				}
				k_msleep(sleep_ms);
				dk_set_led_off(DK_LED3);
				if(err < 0)
				{
					return;
				}
			}
		}
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
void configuer_all_outputs(void)
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
void configuer_all_inputs(void)
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
	valve_status = VALVE_OPENED;
	gpio_pin_set(gpio_dev, WATER_VALVE, OPEN_VALVE);
	/* start periodic timer that expires once every second */
	k_timer_start(&my_timer, K_SECONDS(10), K_NO_WAIT);
}

/********************************************************************************
 * Define a variable of type static struct gpio_callback
 ********************************************************************************/
void my_expiry_function(struct k_timer *timer_id)
{
	valve_status = VALVE_CLOSED;
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
	
	configuer_all_outputs();
	configuer_all_inputs();
	if (dk_leds_init() != 0)
	{
		LOG_ERR("Failed to initialize the LED library");
	}
	k_msleep(SLEEP_TIME_MS * 10);
	LOG_INF("A Smart Water Tap Leakage Controller IoT Project/n/r");
	blink_leds(150, LIGHTWELL_RED, 5);
	blink_leds(150, LIGHTWELL_GREEN, 5);
	blink_leds(150, LIGHTWELL_BLUE, 5);

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
	while(1)
	{
		if(valve_status == VALVE_CLOSED)
		{
			blink_leds(250, LIGHTWELL_GREEN, 2);	
		}
		else if (valve_status == VALVE_OPENED)
		{
			blink_leds(100, LIGHTWELL_RED, 1);
			beep_buzzer(1, 1);
		}
		else
		{}
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