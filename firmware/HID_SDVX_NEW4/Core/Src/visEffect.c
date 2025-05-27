/*

  WS2812B CPU and memory efficient library

  Date: 28.9.2016

  Author: Martin Hubacek
  	  	  http://www.martinhubacek.cz
  	  	  @hubmartin

  Licence: MIT License

*/

#include <stdint.h>
#include <math.h>   // For sinf, fabsf
#include <stdlib.h> // For abs
#include <string.h> // For memset

#include "stm32f4xx_hal.h"
#include "ws2812b/ws2812b.h"
#include "visEffect.h"
#include "main.h" // For HAL_GetTick

// External variables from main.c
extern volatile int16_t g_encoder_delta_x;
extern volatile int16_t g_encoder_delta_y;
extern USBD_JoystickReport_TypeDef joystickReport; // Read-only access assumed here

// Effect parameters - consider moving to defines if they don't change
#define NUM_BUTTONS				7 // Must match main.c

// Helper defines
#define newColor(r, g, b) (((uint32_t)(r) << 16) | ((uint32_t)(g) <<  8) | (b))
#define Red(c) ((uint8_t)((c >> 16) & 0xFF))
#define Green(c) ((uint8_t)((c >> 8) & 0xFF))
#define Blue(c) ((uint8_t)(c & 0xFF))

// Fog effect parameters
#define FOG_BASE_R		150.0f
#define FOG_BASE_G		0.0f
#define FOG_BASE_B		255.0f

// Noise parameters
#define NOISE_X_SCALE	0.5f
#define NOISE_Y_SCALE	0.6f
#define NOISE_XY_SCALE	0.4f
#define NOISE_TIME_SCALE_1 0.7f // Factor for time scale on second sinf
#define NOISE_TIME_SCALE_2 1.3f // Factor for time scale on third sinf
#define TIME_SCALE		0.0030f // Multiplier for HAL_GetTick()

// Brightness parameters
#define MIN_BRIGHTNESS	0.5f
#define MAX_BRIGHTNESS	0.8f
#define BRIGHTENING_THRESHOLD		0.65f
#define BRIGHTENING_MIX				0.5f

// Pulse effect parameters
#define NUM_LEDS				12
#define PULSE_DURATION_MS		250UL
#define PULSE_MAX_BRIGHTNESS	255.0f
#define PULSE_COLOR_R			255.0f // White pulse
#define PULSE_COLOR_G			255.0f
#define PULSE_COLOR_B			255.0f

// Encoder visualization parameters
#define ENCODER_VIS_MAX_DELTA		50.0f // Max delta value for full brightness scaling
#define ENCODER_VIS_MAX_BRIGHTNESS	200.0f // Maximum brightness contribution from encoder
#define ENCODER1_COLOR_R			0.0f
#define ENCODER1_COLOR_G			255.0f
#define ENCODER1_COLOR_B			255.0f
#define ENCODER2_COLOR_R			255.0f
#define ENCODER2_COLOR_G			0.0f
#define ENCODER2_COLOR_B			255.0f
#define ENCODER_ACTIVITY_DECAY		0.955f // Multiplier for decay per frame
#define ENCODER_ACTIVITY_THRESHOLD  0.01f  // Below this, activity snaps to 0
#define ENCODER_INTENSITY_SCALE     2.0f   // Multiplier for activity -> intensity

// Frame buffer for WS2812B data (GRB format typically)
uint8_t frameBuffer[3 * NUM_LEDS]; // 3 bytes per LED (G, R, B)

// Static LED coordinates (read-only)
const float led_coords[NUM_LEDS][2] = {
    {0.0f, 1.0f}, {0.0f, 2.0f}, {0.0f, 3.0f}, // LEDs 1, 2, 3
    {2.0f, 2.0f}, {2.0f, 1.0f}, {2.0f, 0.0f}, // LEDs 4, 5, 6
    {4.0f, 0.0f}, {4.0f, 1.0f}, {4.0f, 2.0f}, // LEDs 7, 8, 9
    {6.0f, 3.0f}, {6.0f, 2.0f}, {6.0f, 1.0f}  // LEDs A, B, C
};

// Maps buttons (index 0-6) to pairs of LED indices (-1 if no second LED)
const int button_to_led_map[NUM_BUTTONS][2] = {
		{0, 1},   // BTN1 -> LEDs 0, 1
		{0, 3},   // BTN2 -> LEDs 0, 3
		{8, 11},  // BTN3 -> LEDs 8, 11
		{11, 10}, // BTN4 -> LEDs 11, 10
		{1, 2},   // FXL  -> LEDs 1, 2
		{10, 9},  // FXR  -> LEDs 10, 9
		{5, 6}    // START-> LEDs 5, 6
};

// Lookup table: 0=None, 1=Encoder1 LED, 2=Encoder2 LED
static uint8_t led_to_encoder_map[NUM_LEDS];

// Timing for pulse effect per LED
static uint32_t pulse_start_time[NUM_LEDS] = {0};
// Previous button state for edge detection
static uint8_t prev_buttons_state = 0;

// Activity level for encoder visualization
static float encoder1_activity = 0.0f;
static float encoder2_activity = 0.0f;


// Helper function to clamp float to uint8_t range
static inline uint8_t clamp_u8(float val) {
    if (val <= 0.0f) return 0;
    if (val >= 255.0f) return 255;
    return (uint8_t)val;
}

// Precompute LED to Encoder mapping
static void init_led_encoder_map() {
    memset(led_to_encoder_map, 0, sizeof(led_to_encoder_map));

    // Define which LEDs belong to which encoder
    const int encoder1_led_indices[] = {3, 4, 5, 2};
	const int encoder2_led_indices[] = {6, 7, 8, 9};
	const int num_encoder1_leds = sizeof(encoder1_led_indices) / sizeof(encoder1_led_indices[0]);
	const int num_encoder2_leds = sizeof(encoder2_led_indices) / sizeof(encoder2_led_indices[0]);


    for (int j = 0; j < num_encoder1_leds; ++j) {
        int idx = encoder1_led_indices[j];
        if (idx >= 0 && idx < NUM_LEDS) {
            led_to_encoder_map[idx] = 1; // Mark as Encoder 1 LED
        }
    }
    for (int j = 0; j < num_encoder2_leds; ++j) {
         int idx = encoder2_led_indices[j];
        if (idx >= 0 && idx < NUM_LEDS) {
            // Avoid overwriting if somehow assigned to both (should not happen with current data)
            if (led_to_encoder_map[idx] == 0) {
                 led_to_encoder_map[idx] = 2; // Mark as Encoder 2 LED
            }
        }
    }
}


/**
 * @brief Main visualization script to calculate LED colors.
 * @param fBuf Pointer to the frame buffer (GRB format expected).
 * @param frameBufferSize Size of the frame buffer in bytes.
 */
void visScript(uint8_t *fBuf, uint32_t frameBufferSize) {
    if (fBuf == NULL || frameBufferSize != (3 * NUM_LEDS)) {
        // Handle error: invalid buffer
        return;
    }

	uint32_t current_tick = HAL_GetTick();
	float time_sec = (float)current_tick * TIME_SCALE;

	// --- Button Press Pulse Effect ---
	uint8_t current_buttons = joystickReport.buttons; // Read current button state
	uint8_t pressed_edge = current_buttons & ~prev_buttons_state; // Detect newly pressed buttons
	prev_buttons_state = current_buttons; // Store for next iteration

	for (int i = 0; i < NUM_BUTTONS; ++i) {
		if ((pressed_edge >> i) & 1) { // If button 'i' was just pressed
			int led_index1 = button_to_led_map[i][0];
			int led_index2 = button_to_led_map[i][1];

			// Trigger pulse on the first mapped LED
			if (led_index1 >= 0 && led_index1 < NUM_LEDS) {
				 // Use 1 if current_tick is 0 to ensure pulse starts
				 pulse_start_time[led_index1] = current_tick ? current_tick : 1;
			}
			// Trigger pulse on the second mapped LED (if different from the first)
			if (led_index2 >= 0 && led_index2 < NUM_LEDS && led_index2 != led_index1) {
				 pulse_start_time[led_index2] = current_tick ? current_tick : 1;
			}
		}
	}

	// --- Encoder Activity Update ---
    // Read volatile deltas once
	int16_t current_delta_x = g_encoder_delta_x;
	int16_t current_delta_y = g_encoder_delta_y;

    // Apply decay
    encoder1_activity *= ENCODER_ACTIVITY_DECAY;
    encoder2_activity *= ENCODER_ACTIVITY_DECAY;

    // Calculate new activity based on delta, ensuring it's non-negative
    float target_activity1 = fabsf((float)current_delta_x) / ENCODER_VIS_MAX_DELTA;
    target_activity1 = (target_activity1 > 1.0f) ? 1.0f : target_activity1; // Clamp to [0, 1]

    float target_activity2 = fabsf((float)current_delta_y) / ENCODER_VIS_MAX_DELTA;
    target_activity2 = (target_activity2 > 1.0f) ? 1.0f : target_activity2; // Clamp to [0, 1]

    // Update activity only if the new target is higher (instant attack, slow decay)
    if (target_activity1 > encoder1_activity) {
        encoder1_activity = target_activity1;
    } else if (encoder1_activity < ENCODER_ACTIVITY_THRESHOLD) {
         encoder1_activity = 0.0f; // Snap to zero if very low
    }

    if (target_activity2 > encoder2_activity) {
        encoder2_activity = target_activity2;
    } else if (encoder2_activity < ENCODER_ACTIVITY_THRESHOLD) {
         encoder2_activity = 0.0f; // Snap to zero if very low
    }


	// --- LED Color Calculation Loop ---
	for (int i = 0; i < NUM_LEDS; ++i) {
		float r_final = 0.0f, g_final = 0.0f, b_final = 0.0f;
		uint8_t pulse_active = 0;
		float pulse_intensity = 0.0f;

		// Check for active pulse
		if (pulse_start_time[i] != 0) {
			uint32_t elapsed = current_tick - pulse_start_time[i];
			if (elapsed < PULSE_DURATION_MS) {
				pulse_active = 1;
                // Linear fade out for pulse intensity
				pulse_intensity = 1.0f - ((float)elapsed / (float)PULSE_DURATION_MS);
			} else {
				pulse_start_time[i] = 0; // Pulse finished
			}
		}

		// Calculate background "fog" color based on noise
		float x = led_coords[i][0];
		float y = led_coords[i][1];
		// Sum of sines for noise pattern
		float noise_val = sinf(x * NOISE_X_SCALE + time_sec)
                        + sinf(y * NOISE_Y_SCALE + time_sec * NOISE_TIME_SCALE_1)
                        + sinf((x + y) * NOISE_XY_SCALE + time_sec * NOISE_TIME_SCALE_2);
		// Normalize noise value to [0, 1] approx range (-3 to 3 -> 0 to 6 -> 0 to 1)
		float noise_norm = (noise_val + 3.0f) * (1.0f / 6.0f);
		noise_norm = (noise_norm < 0.0f) ? 0.0f : (noise_norm > 1.0f) ? 1.0f : noise_norm; // Clamp strictly

		// Base brightness modulated by noise
		float base_brightness = MIN_BRIGHTNESS + noise_norm * (MAX_BRIGHTNESS - MIN_BRIGHTNESS);
		float r_fog = FOG_BASE_R * base_brightness;
		float g_fog = FOG_BASE_G * base_brightness;
		float b_fog = FOG_BASE_B * base_brightness;

        // Add white brightening for high noise values
		float white_mix = 0.0f;
		if (noise_norm > BRIGHTENING_THRESHOLD) {
            // Scale mix amount based on how far above threshold
			white_mix = (noise_norm - BRIGHTENING_THRESHOLD) / (1.0f - BRIGHTENING_THRESHOLD);
			white_mix *= BRIGHTENING_MIX; // Apply mix factor
			white_mix = (white_mix > 1.0f) ? 1.0f : white_mix; // Clamp mix factor
		}
        // Mix fog color with white
		float bg_r = r_fog * (1.0f - white_mix) + 255.0f * white_mix;
		float bg_g = g_fog * (1.0f - white_mix) + 255.0f * white_mix;
		float bg_b = b_fog * (1.0f - white_mix) + 255.0f * white_mix;


		// --- Combine Effects ---
		if (pulse_active) {
			// If pulse is active, it overrides the background
            float pulse_val = PULSE_MAX_BRIGHTNESS * pulse_intensity;
			r_final = PULSE_COLOR_R * pulse_intensity;
            g_final = PULSE_COLOR_G * pulse_intensity;
            b_final = PULSE_COLOR_B * pulse_intensity;
		} else {
			// Otherwise, start with the background color
			r_final = bg_r;
            g_final = bg_g;
            b_final = bg_b;
		}

		// Check if this LED is for encoder visualization using the map
        uint8_t encoder_map_type = led_to_encoder_map[i];

        if (encoder_map_type == 1 && encoder1_activity > 0.0f) {
            // Add Encoder 1 color based on activity
            float current_intensity = encoder1_activity * ENCODER_INTENSITY_SCALE;
            current_intensity = (current_intensity > 1.0f) ? 1.0f : current_intensity; // Clamp intensity contribution
			float brightness_scale = current_intensity * ENCODER_VIS_MAX_BRIGHTNESS;
            r_final += ENCODER1_COLOR_R * brightness_scale / 255.0f; // Scale color contribution
            g_final += ENCODER1_COLOR_G * brightness_scale / 255.0f;
            b_final += ENCODER1_COLOR_B * brightness_scale / 255.0f;
        } else if (encoder_map_type == 2 && encoder2_activity > 0.0f) {
            // Add Encoder 2 color based on activity
            float current_intensity = encoder2_activity * ENCODER_INTENSITY_SCALE;
            current_intensity = (current_intensity > 1.0f) ? 1.0f : current_intensity; // Clamp intensity contribution
			float brightness_scale = current_intensity * ENCODER_VIS_MAX_BRIGHTNESS;
            r_final += ENCODER2_COLOR_R * brightness_scale / 255.0f; // Scale color contribution
            g_final += ENCODER2_COLOR_G * brightness_scale / 255.0f;
            b_final += ENCODER2_COLOR_B * brightness_scale / 255.0f;
        }

        // --- Write final color to buffer (GRB order) ---
        fBuf[i * 3 + 0] = clamp_u8(r_final); // Green
        fBuf[i * 3 + 1] = clamp_u8(g_final); // Red
        fBuf[i * 3 + 2] = clamp_u8(b_final); // Blue
	}
}


/**
 * @brief Internal handler called periodically to update LED effects.
 */
void visHandle2()
{
	static uint32_t timestamp = 0;
    const uint32_t update_interval_ms = 20; // Target ~50 FPS

	// Check if enough time has passed since the last update
	if ((HAL_GetTick() - timestamp) >= update_interval_ms)
	{
		timestamp = HAL_GetTick();

		// Calculate the next frame of the LED visualization
		visScript(frameBuffer, sizeof(frameBuffer));
	}
}


/**
 * @brief Initializes the visualization system.
 */
void visInit()
{
    // Initialize LED state arrays
	memset(frameBuffer, 0, sizeof(frameBuffer));
	memset(pulse_start_time, 0, sizeof(pulse_start_time));

    // Initialize effect state variables
	prev_buttons_state = 0;
	encoder1_activity = 0.0f;
	encoder2_activity = 0.0f;

    // Precompute the LED-to-encoder mapping
    init_led_encoder_map();

    // Configure the WS2812B library instance (assuming only one strip)
	ws2812b.item[0].channel = 0; // Use channel 0 (adjust if using multiple channels/timers)
	ws2812b.item[0].frameBufferPointer = frameBuffer;
	ws2812b.item[0].frameBufferSize = sizeof(frameBuffer);

	// Initialize the WS2812B hardware interface (timers, DMA)
	ws2812b_init();
}


/**
 * @brief Main handler for WS2812B updates, called from the main loop.
 */
void visHandle()
{
    // Check if the previous DMA transfer for WS2812B is complete
	if(ws2812b.transferComplete)
	{
		// Update the LED effect frame buffer (call the script)
		// Note: visHandle2 includes its own timing, so script only runs ~50Hz
		visHandle2();

		// Signal the WS2812B library that a new frame is ready
		ws2812b.startTransfer = 1;
        // Start the DMA transfer to send the updated frameBuffer to the LEDs
		ws2812b_handle();
	}
}
