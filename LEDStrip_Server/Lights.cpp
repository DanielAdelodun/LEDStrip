// Daemon that controls the colour of LEDs based on:
// - Current Mode
// - Arm State / Recent Arming changes
// - Incoming Mavlink Messsage targeted at this "LED" Component 
// TODO: Many Things - including Implementing Command-Line Options.

#include <cstdint>
#include <cstdlib>
#include <csignal>
#include <unistd.h>
#include <string.h>
#include <getopt.h>
#include <iostream>
#include <future>
#include <map>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <ws2811.h>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

// General Macros
#define ARRAY_SIZE(stuff)       (sizeof(stuff) / sizeof(stuff[0]))


// Cmdline Defaults
#define LEFT_ARM_CHANNEL        0
#define LEFT_ARM_GPIO           12
#define RIGHT_ARM_CHANNEL       1
#define RIGHT_ARM_GPIO          13
#define DMA                     10
#define ARM_LENGTH              5
#define ARM_COUNT               2
#define STRIP_TYPE              WS2811_STRIP_GRB		// WS2812/SK6812RGB integrated chip+leds
const char* ENDPOINT     =      "tcp://127.0.0.1:5760";

// Colours
#define RED						0x00FF0000
#define ORANGE					0x00FF7F00
#define YELLOW					0x00FFFF00
#define GREEN					0x0000FF00
#define LIGHTBLUE				0x0000FFFF
#define BLUE					0x000000FF
#define PURPLE					0x00FF00FF
#define PINK					0x00FF007F
#define WHITE					0x00FFFFFF

ws2811_led_t ArrayOfColours[] =
{
	RED,        ORANGE,
	YELLOW,     GREEN,
	LIGHTBLUE,  BLUE,
	PURPLE,     PINK
};


// Setup Signal Catchers. Set running = 0 on SIGINT/SIGTERM
static uint8_t clearOnExit = 0;
static uint8_t running = 1;
static void ctrl_c_handler(int signum)
{
    running = 0;
}
static void setup_handlers(void)
{
 	std::signal(SIGINT, ctrl_c_handler);
    std::signal(SIGTERM, ctrl_c_handler);
}


// Setup WS2811. Assumes ARM_COUNT == 2
// TODO: Dynamic Setup & multiple ws2811 objects based on ARM_COUNT
static ws2811_return_t DroneLightStatus;
static ws2811_t DroneLights =
{
    .freq = WS2811_TARGET_FREQ,
    .dmanum = DMA,
    .channel =
    {
        [LEFT_ARM_CHANNEL] =
        {
            .gpionum = LEFT_ARM_GPIO,
            .invert = 0,
            .count = ARM_LENGTH,
            .strip_type = STRIP_TYPE,
            .brightness = 255,
        },
        [RIGHT_ARM_CHANNEL] =
        {
            .gpionum = RIGHT_ARM_GPIO,
            .invert = 0,
            .count = ARM_LENGTH,
            .strip_type = STRIP_TYPE,
            .brightness = 255,
        },
    },
};


// Map FlightModes to LED Colours
std::map<Telemetry::FlightMode, ws2811_led_t> FlightMode2Colour {
	{Telemetry::FlightMode::Manual, RED},
	{Telemetry::FlightMode::Posctl, GREEN},
	{Telemetry::FlightMode::Altctl, BLUE},
	{Telemetry::FlightMode::Mission, LIGHTBLUE},
	{Telemetry::FlightMode::Hold, PINK},
	{Telemetry::FlightMode::Offboard, ORANGE},
	{Telemetry::FlightMode::Acro, PURPLE},
	{Telemetry::FlightMode::Stabilized, YELLOW},
	{Telemetry::FlightMode::FollowMe, WHITE},
	{Telemetry::FlightMode::Land, WHITE},
	{Telemetry::FlightMode::Rattitude, WHITE},
	{Telemetry::FlightMode::Ready, WHITE},
	{Telemetry::FlightMode::ReturnToLaunch, WHITE},
	{Telemetry::FlightMode::Takeoff, WHITE},
	{Telemetry::FlightMode::Unknown, WHITE},
};


std::string mavsdkEndpoint = ENDPOINT;
// Parse Cmdline Options using getopt
// TODO: Account for multiple ws2811 objects
void parseargs(int argc, char **argv, ws2811_t *ws2811)
{
	int index, opt;

	static struct option longopts[] =
	{
		{"help", no_argument, 0, 'h'},
		{"dma", required_argument, 0, 'd'},
		{"gpio", required_argument, 0, 'g'},
		{"clear", no_argument, 0, 'c'},
		{"strip", required_argument, 0, 's'},
		{"arms", required_argument, 0, 'a'},
		{"length", required_argument, 0, 'l'},
		{"endpoint", required_argument, 0, 'e'},
		{0, 0, 0, 0}
	};

	while (1)
	{

		index = 0;
		opt = getopt_long(argc, argv, "cd:g:hs:a:l:", longopts, &index);

		if (opt == -1)
			break;

		switch (opt)
		{		
		case 0:
		case 'a':
		case 'l':
		case 'g':
			break;

		case 'h':
			std::cerr << "Usage: " << argv[0] << "\n"
				<< "-h (--help)     - this information\n"
				<< "-s (--strip)    - strip type - rgb, grb, gbr, rgbw\n"
				<< "-d (--dma)      - dma channel to use (default 10)\n"
				<< "-g (--gpio)     - Comma seperated list of GPIO to use\n"
				<< "                  (default 12,13 (PWM0/1))\n"
				<< "-c (--clear)    - clear matrix on exit.\n"
				<< "-a (--arms)     - No. arms with LEDS attached.\n" 
				<< "                  i.e No. of LED strips (default 2)\n"
				<< "-l (--length)   - No. leds per arm (default 5)\n"
				<< "-e (--endpoint) - mavlink endpoint to connect to"
				<< "                  (default tcp://127.0.0.1:5760)";
			exit(-1);

		case 'c':
			clearOnExit=1;
			break;

		case 'd':
			if (optarg) {
				int dma = std::atoi(optarg);
				if (dma < 14) {
					ws2811->dmanum = dma;
				} else {
					std::cerr << "invalid dma " << dma << "\n";
					std::exit (-1);
				}
			}
			break;

		case 's':
			if (optarg) {
				if (!strncasecmp("rgb", optarg, 4)) {
					ws2811->channel[LEFT_ARM_CHANNEL].strip_type = WS2811_STRIP_RGB;
					ws2811->channel[RIGHT_ARM_CHANNEL].strip_type = WS2811_STRIP_RGB;
				}
				else if (!strncasecmp("rbg", optarg, 4)) {
					ws2811->channel[LEFT_ARM_CHANNEL].strip_type = WS2811_STRIP_RBG;
					ws2811->channel[RIGHT_ARM_CHANNEL].strip_type = WS2811_STRIP_RBG;
				}
				else if (!strncasecmp("grb", optarg, 4)) {
					ws2811->channel[LEFT_ARM_CHANNEL].strip_type = WS2811_STRIP_GRB;
					ws2811->channel[RIGHT_ARM_CHANNEL].strip_type = WS2811_STRIP_GRB;
				}
				else if (!strncasecmp("gbr", optarg, 4)) {
					ws2811->channel[LEFT_ARM_CHANNEL].strip_type = WS2811_STRIP_GBR;
					ws2811->channel[RIGHT_ARM_CHANNEL].strip_type = WS2811_STRIP_GBR;
				}
				else if (!strncasecmp("brg", optarg, 4)) {
					ws2811->channel[LEFT_ARM_CHANNEL].strip_type = WS2811_STRIP_BRG;
					ws2811->channel[RIGHT_ARM_CHANNEL].strip_type = WS2811_STRIP_BRG;
				}
				else if (!strncasecmp("bgr", optarg, 4)) {
					ws2811->channel[LEFT_ARM_CHANNEL].strip_type = WS2811_STRIP_BGR;
					ws2811->channel[RIGHT_ARM_CHANNEL].strip_type = WS2811_STRIP_BGR;
				}
				else if (!strncasecmp("rgbw", optarg, 4)) {
					ws2811->channel[LEFT_ARM_CHANNEL].strip_type = SK6812_STRIP_RGBW;
					ws2811->channel[RIGHT_ARM_CHANNEL].strip_type = SK6812_STRIP_RGBW;
				}
				else if (!strncasecmp("grbw", optarg, 4)) {
					ws2811->channel[LEFT_ARM_CHANNEL].strip_type = SK6812_STRIP_GRBW;
					ws2811->channel[RIGHT_ARM_CHANNEL].strip_type = SK6812_STRIP_GRBW;
				}
				else {
					std::cerr << "invalid strip " << optarg << "\n";
					std::exit (-1);
				}
			}
			break;

		case 'e':
			mavsdkEndpoint = optarg;

		case '?':
			/* getopt_long already reported error? */
			std::exit(-1);

		default:
			std::exit(-1);
		}
	}
}


inline void clearArms(void) {
	for(int Arm = 0; Arm < ARM_COUNT; Arm++)
		for(int Pos = 0; Pos < ARM_LENGTH; Pos++)
			DroneLights.channel[Arm].leds[Pos] = (ws2811_led_t)0;
}

inline void fillArms(ws2811_led_t Colour) {
	for(int Arm = 0; Arm < ARM_COUNT; Arm++)
		for(int Pos = 0; Pos < ARM_LENGTH; Pos++)
			DroneLights.channel[Arm].leds[Pos] = Colour;
}

inline void fillArm(ws2811_led_t Colour, int Arm) {
	for(int Pos; Pos < ARM_LENGTH; Pos++) {
		DroneLights.channel[Arm].leds[Pos] = Colour;
	}
}

inline void killLights() {
		fillArms(RED);
		ws2811_render(&DroneLights);
		ws2811_fini(&DroneLights);
}


void subscribe_led_string_set(MavlinkPassthrough& mavlink_passthrough){
	uint32_t *leds;
    mavlink_passthrough.subscribe_message_async(
		60200,
        [leds](const mavlink_message_t& msg) { 
			mavlink_msg_led_strip_set_get_colors(&msg, leds);
            std::cout <<  "LED Colour: " << std::hex << leds[0] << '\n'; 
			// std::cout << "Heartbeat Autopilot: " << (int)mavlink_msg_heartbeat_get_autopilot(&msg) << '\n';
        }
    );
}

void subscribe_flight_mode(Telemetry& telemetry){
    telemetry.subscribe_flight_mode([](Telemetry::FlightMode flight_mode) {
		auto Colour = FlightMode2Colour[flight_mode];

		fillArms(Colour);
        if ((DroneLightStatus = ws2811_render(&DroneLights)) != WS2811_SUCCESS)
        {
			std::cerr << "ws2811_render failed: " 
			          << ws2811_get_return_t_str(DroneLightStatus) << '\n';
        }

    });
}

void unsubscribe_flight_mode(Telemetry& telemetry){
    telemetry.subscribe_flight_mode(nullptr);
}

std::shared_ptr<System> get_system(Mavsdk& mavsdk)
{
    std::cout << "Waiting to discover system...\n";
    auto promise_system = std::promise<std::shared_ptr<System>>{};
    auto future_system = promise_system.get_future();

    mavsdk.subscribe_on_new_system([&mavsdk, &promise_system]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            mavsdk.subscribe_on_new_system(nullptr);
            promise_system.set_value(system);
        }
    });

    if (future_system.wait_for(seconds(3)) == std::future_status::timeout) {
        bool not_found;
        mavsdk.subscribe_on_new_system(nullptr);

        std::cerr << "No autopilot caught. Re-checking found systems once.\n";
        for (auto system : mavsdk.systems()) {
            if (system->has_autopilot()) {
                std::cout << "Discovered autopilot\n";
                promise_system.set_value(system);
                not_found = false;
                break;
            }
        }
        if (not_found) return 0;    
    }

    return future_system.get();
}

int main(int argc, char *argv[])
{
    parseargs(argc, argv, &DroneLights);

	ws2811_led_t Colour;
	int i = 0;


    if ((DroneLightStatus = ws2811_init(&DroneLights)) != WS2811_SUCCESS)
    {
        std::cerr << "ws2811_init failed: " 
		          << ws2811_get_return_t_str(DroneLightStatus) << "\n";
        return DroneLightStatus;
    }

	Mavsdk mavsdk;
	Mavsdk::Configuration config(1, 134, true);
	mavsdk.set_configuration(config);
    
    ConnectionResult connection_result = mavsdk.add_any_connection(mavsdkEndpoint);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
		killLights();
		return -1;
	}

	std::cout << "Mavlink Connection Established: " << mavsdkEndpoint << '\n';

    auto system = get_system(mavsdk);
    if (!system) {
        std::cerr << "No autopilot found\n";
		killLights();
		return -1;  
	}

    setup_handlers();
	clearArms();

    // Instantiate plugins.
    auto telemetry = Telemetry{system};
	auto mavlink_passthrough = MavlinkPassthrough{system};

	subscribe_led_string_set(mavlink_passthrough);
    subscribe_flight_mode(telemetry);

    while (running) {}

    if (clearOnExit) {
	clearArms();
	ws2811_render(&DroneLights);
    }

    ws2811_fini(&DroneLights);

    std::cout << "\n";
    return DroneLightStatus;
}

