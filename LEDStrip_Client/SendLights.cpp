#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include <thread>

using namespace mavsdk;
using std::chrono::seconds;

static void usage(const std::string& binaryName);
static void send_led_string_set(
    MavlinkPassthrough&, 
    mavlink_led_strip_set_t
);
static void setLEDFillColour(u_int32_t, mavlink_led_strip_set_t);
static u_int32_t getNewLEDColour(void);
std::shared_ptr<System> get_system(Mavsdk&);

mavlink_led_strip_set_t LEDStripSet {
    .colors = {
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 
    0x00000000, 0x00000000, 0x00000000, 0x00000000
    },
    .target_system = 1,
    .target_component = 134,
    .fill_mode = LED_FILL_MODE_ALL,
    .led_index = UINT8_MAX,
    .length = UINT8_MAX,
    .strip_index = UINT8_MAX,
};

int main(int argc, char** argv) {
    if (argc != 2) {
        usage(argv[0]);
        exit(1);
    }

    Mavsdk mavsdk;
	mavsdk.set_configuration(Mavsdk::Configuration(1, 135, true));

    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        exit(1);
    }

    auto system = get_system(mavsdk);
    if (!system) {
        exit(1);
    }
    
    // Instantiate plugins.
    auto mavlink_passthrough = MavlinkPassthrough{system};

    while (true) {
        u_int32_t newColour = getNewLEDColour();
        setLEDFillColour(newColour, LEDStripSet);
        send_led_string_set(mavlink_passthrough, LEDStripSet);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    exit(0);
}

void usage(const std::string& bin_name) {
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

std::shared_ptr<System> get_system(Mavsdk& mavsdk) {
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
    });

    if (fut.wait_for(seconds(3)) == std::future_status::timeout) {
        bool not_found = true;
        mavsdk.subscribe_on_new_system(nullptr);

        std::cerr << "No autopilot caught. Re-checking found systems once.\n";
        for (auto system : mavsdk.systems()) {
            if (system->has_autopilot()) {
                std::cout << "Discovered autopilot\n";
                prom.set_value(system);
                not_found = false;
                break;
            }
        }
        if (not_found) return 0;    
    }

    return fut.get();
}

static void setLEDFillColour(u_int32_t newColour, mavlink_led_strip_set_t LEDStripSet) {
    LEDStripSet.colors[0] = newColour;
    LEDStripSet.fill_mode = LED_FILL_MODE_ALL;
}


void send_led_string_set(MavlinkPassthrough& mavlink_passthrough, mavlink_led_strip_set_t LEDStripSet)
{
    mavlink_message_t message;

    mavlink_msg_led_strip_set_encode(
        mavlink_passthrough.get_our_sysid(),
        mavlink_passthrough.get_our_compid(),
        &message,
        &LEDStripSet
    ); 

    mavlink_passthrough.send_message(message);
}
