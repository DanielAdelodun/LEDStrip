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

// Colours
#define RED 0x00FF0000
#define ORANGE 0x00FF7F00
#define YELLOW 0x00FFFF00
#define GREEN 0x0000FF00
#define LIGHTBLUE 0x0000FFFF
#define BLUE 0x000000FF
#define PURPLE 0x00FF00FF
#define PINK 0x00FF007F
#define WHITE 0x00FFFFFF

extern uint32_t ArrayOfColours[];
extern uint8_t NoOfColours;
extern mavlink_led_strip_config_t LEDStripConfig;

void setFollowFlightMode(mavlink_led_strip_config_t &);
void setLEDFillColour(uint32_t, mavlink_led_strip_config_t &);
void cycleLEDColour(mavsdk::MavlinkPassthrough &, const mavlink_led_strip_config_t &);
void sendLedStripConfig(mavsdk::MavlinkPassthrough &, const mavlink_led_strip_config_t &);

std::shared_ptr<mavsdk::System> configMavsdk(mavsdk::Mavsdk &, const std::string &);
std::shared_ptr<mavsdk::System> getSystem(mavsdk::Mavsdk &);

