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
    .led_index = UINT8_MAX,
    .length = UINT8_MAX,
    .strip_id = UINT8_MAX,
};


std::shared_ptr<System> getSystem(Mavsdk &mavsdk)
{
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    mavsdk.subscribe_on_new_system([&mavsdk, &prom]()
                                   {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        } });

    if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::timeout)
    {
        bool not_found = true;
        mavsdk.subscribe_on_new_system(nullptr);

        std::cerr << "No autopilot caught. Re-checking found systems once.\n";
        for (auto system : mavsdk.systems())
        {
            if (system->has_autopilot())
            {
                std::cout << "Discovered autopilot\n";
                prom.set_value(system);
                not_found = false;
                break;
            }
        }
        if (not_found)
            return 0;
    }

    return fut.get();
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

    std::cout << "Attempted to send LEDStripConfig with colour: " << std::hex << LEDStripConfig.colors[0] << std::endl;
}

static uint32_t cycleLEDColour(void)
{
    static int ColourIndex;
    ColourIndex++;
    ColourIndex %= NoOfColours;
    
    return ArrayOfColours[ColourIndex];
}

std::shared_ptr<mavsdk::System> configMavsdk(Mavsdk &mavsdk, const std::string &connection_url)
{
    mavsdk.set_configuration(Mavsdk::Configuration(1, 135, true));

    ConnectionResult connection_result = mavsdk.add_any_connection(connection_url);

    if (connection_result != ConnectionResult::Success)
    {
        std::cerr << "Connection failed: " << connection_result << '\n';
        exit(1);
    }

    auto system = getSystem(mavsdk);
    if (!system)
    {
        exit(1);
    }

    return system;
}