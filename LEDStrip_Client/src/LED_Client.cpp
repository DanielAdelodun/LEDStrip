#include <stdio.h>
#include <stdlib.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include "LED_Client.h"

using namespace mavsdk;

uint32_t ArrayOfColours[] =
{
        RED, ORANGE,
        YELLOW, GREEN,
        LIGHTBLUE, BLUE,
        PURPLE, PINK
};

uint8_t NoOfColours = sizeof(ArrayOfColours) / sizeof(uint32_t);

mavlink_led_strip_config_t LEDStripConfig{
    .colors = {0, 0, 0, 0, 0, 0, 0, 0},
    .target_system = 1,
    .target_component = 134,
    .fill_mode = LED_FILL_MODE_ALL,
    .led_index = 0,
    .length = 8,
    .strip_id = UINT8_MAX,
};

void setFollowFlightMode(mavlink_led_strip_config_t &LEDStripConfig)
{
    LEDStripConfig.fill_mode = LED_FILL_MODE_FOLLOW_FLIGHT_MODE;
}

void setLEDFillColour(uint32_t newColour, mavlink_led_strip_config_t &LEDStripConfig)
{
    LEDStripConfig.colors[0] = newColour;
    LEDStripConfig.fill_mode = LED_FILL_MODE_ALL;
}

void sendLedStripConfig(MavlinkPassthrough &mavlink_passthrough, const mavlink_led_strip_config_t &LEDStripConfig)
{
    mavlink_message_t message;

    mavlink_msg_led_strip_config_encode(
        mavlink_passthrough.get_our_sysid(),
        mavlink_passthrough.get_our_compid(),
        &message,
        &LEDStripConfig);

    mavlink_passthrough.send_message(message);

    if (LEDStripConfig.fill_mode != LED_FILL_MODE_FOLLOW_FLIGHT_MODE)
        std::cout << "Attempted to send LEDStripConfig with colour: " << std::hex << LEDStripConfig.colors[0] << std::endl;
    else
        std::cout << "Attempted to send follow flight mode" << std::endl;
}

static uint32_t cycleLEDColour(void)
{
    static int ColourIndex;
    ColourIndex++;
    ColourIndex %= NoOfColours;
    
    return ArrayOfColours[ColourIndex];
}