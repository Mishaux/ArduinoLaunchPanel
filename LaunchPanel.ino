#include "FastSPI_LED2.h"
#define NUM_RING_LEDS 16   // LED count in fuel pump ring
#define NUM_SHIP_LEDS 36  // LED count in rocket fuel tank strip
#define TOTAL_LEDS 52     // Total LEDs in strip

// Input pins
#define PANEL_POWER_SWITCH_PIN 11     // Input pin for toggle switch on control panel
#define FUEL_BUTTON_PIN 10            // Input pin for fuel button on control panel
#define LAUNCH_BUTTON_PIN 9           // Input pin for launch button on control panel
#define TANK_PRESSURE_SENSOR_PIN A5   // Input pin for tank pressure sensor
#define PRESSURE_SET_BUTTON_PIN 8     // Input pin for max pressure setting button

//Output pins
#define FUEL_VALVE_PIN 0          // Output pin for fuel valve
#define LAUNCH_VALVE_PIN 1        // Output pin for launch valve
#define REVOLVING_LIGHT_PIN 2     // Output pin for revolving cherry lights on launcher
#define LAUNCH_BUTTON_LED_PIN 3   // Output pin for red led built in to launch button
#define LED_STRIP_DATA_PIN 4      // Output pin for data bus to RGB LED strip
#define COUNTDOWN_TRIGGER_PIN 5   // Output pin for triggering countdown display on launcher

CRGB pixels[TOTAL_LEDS];                   // Data structure for LED array settings
int max_brightness = 255;                  // Sets overall max brightness for LEDs
float red, green, blue = 0;                // Buffers used while calculating color for a pixel
float tank_red, tank_green, tank_blue = 0; // Buffers used for the color of the fuel tank pixels
long blink_timer = 0;                      // Timer buffer for blinking lights
int chasing_animation_frame_delay = 40;    // Delay between frames for chasing LED animations
long chasing_animation_timer = 0;          // Timer Buffer for chasing LED animations
int chasing_animation_index = 0;           // Animation frame index buffer for various animations

int empty_pressure_reading = 70;    // Reading from pressure sensor on empty tank
int full_pressure_reading = 460;    // Must corrospond to 100PSI on pressure sensor

float absolute_max_pressure = 95;   // Absolute system max pressure setting in PSI
float absolute_min_pressure = 30;   // Lowest pressure setting from control panel
float max_pressure_setting = absolute_min_pressure;     // Starting 'Full' pressure setting in psi
float current_pressure = 0;         // Current pressure reading in psi
boolean panel_power_switch = false;      // Safety switch position
boolean fueling = true;             // True while all conditions for opening fill valve are met

boolean fuel_button_reading = false;          // Current detected fuel button state
boolean launch_button_reading = false;        // Current detected launch button state
int launch_duration = 2000;                   // Time the launch valve remains open on launch
long launch_buffer = 0;                       // Buffer for lanch valve open duration
boolean panel_power_switch_reading = false;        // Current detected panel power switch state
float pressure_sensor_reading = 0;            // Current raw pressure sensor reading
boolean pressure_set_button_reading = false;  // Current detected fuel button state
float tank_percent_full = 0;

// We need to smooth the readings from the pressure sensor so we use a ring buffer of readings and average them
static const int measurment_count = 200;  // measurements in ring. More to increase smoothing, less for faster response
static int measurments[measurment_count]; // Circular measurement buffer
static byte index = 0;                    // Index pointer for ring buffer
static long sum = 0;                      // Running sum (we don't re-sum the whole ring each sample, just subtract the one we removed and add the new)
static byte count = 0;                    // As the buffer fills, we need to know how many samples we have so far.

void setup() {
  // sanity check delay - allows reprogramming if accidently blowing power w/leds
  delay(2000);
  
  // Initialize ring buffer
  memset(measurments,0,sizeof(measurments));
  
  // Initialize blink timer buffer
  blink_timer = millis() + 500;
  chasing_animation_timer = millis() + chasing_animation_frame_delay;
  
  // Initialize LEDs
  LEDS.setBrightness(255);
  LEDS.addLeds<WS2811, LED_STRIP_DATA_PIN>(pixels, TOTAL_LEDS);
  
  // Initialize Input Pins
  pinMode(FUEL_BUTTON_PIN, INPUT);
  pinMode(LAUNCH_BUTTON_PIN, INPUT);
  pinMode(PANEL_POWER_SWITCH_PIN, INPUT);
  pinMode(TANK_PRESSURE_SENSOR_PIN, INPUT);
  pinMode(PRESSURE_SET_BUTTON_PIN, INPUT);
  
  // This enables the built in pull up resistor in these pins
  digitalWrite(FUEL_BUTTON_PIN, HIGH);
  digitalWrite(LAUNCH_BUTTON_PIN, HIGH);

  // Initialize Output Pins
  pinMode(LAUNCH_BUTTON_LED_PIN, OUTPUT);
  pinMode(FUEL_VALVE_PIN, OUTPUT);
  pinMode(LAUNCH_VALVE_PIN, OUTPUT);
  pinMode(REVOLVING_LIGHT_PIN, OUTPUT);
  pinMode(COUNTDOWN_TRIGGER_PIN, OUTPUT);
  
  digitalWrite(FUEL_VALVE_PIN, LOW);
  digitalWrite(LAUNCH_VALVE_PIN, LOW);
  digitalWrite(REVOLVING_LIGHT_PIN, HIGH);
}

void loop() {
  // Get readings
  fuel_button_reading = digitalRead(FUEL_BUTTON_PIN);
  launch_button_reading = digitalRead(LAUNCH_BUTTON_PIN);
  panel_power_switch_reading = digitalRead(PANEL_POWER_SWITCH_PIN);
  pressure_set_button_reading = digitalRead(PRESSURE_SET_BUTTON_PIN);
  tank_percent_full = tankPercentFull();
  panel_power_switch = (panel_power_switch_reading == HIGH);
  
  // Set max pressure if pressure setting key is turned
  if (pressure_set_button_reading == HIGH) {
    setMaxPressure ();
  } 
  
  // If the launch button is pressed and the panel power switch is on, energize the launch valve and turn off the button LED
  if (launch_button_reading == HIGH && panel_power_switch) {
    launch_buffer = millis() + launch_duration;
    while ( launch_buffer > millis() && panel_power_switch) {
      digitalWrite(LAUNCH_BUTTON_LED_PIN, LOW);
      digitalWrite(LAUNCH_VALVE_PIN, LOW);
    }
  } else if (launch_button_reading == LOW) {
    digitalWrite(LAUNCH_VALVE_PIN, HIGH);
  }

  // If the fuel button is pressed and the panel power switch is on and the tank is not yet full, fill it.
  if (fuel_button_reading == HIGH && panel_power_switch && tank_percent_full <= 1) {
    fueling = true;
    digitalWrite(FUEL_VALVE_PIN, LOW);
  } else {
    fueling = false;
    digitalWrite(FUEL_VALVE_PIN, HIGH);
  }
  
  // If panel power switch is off, do not light LEDs. Otherwise, calculate out the display
  if (panel_power_switch_reading == true) {
   setRocketTankLEDs();
   setFuelPumpLEDs();
  } else {
    for (int i=0; i<TOTAL_LEDS; i++) {
      pixels[i] = CRGB(0, 0, 0);
    }
    digitalWrite(LAUNCH_BUTTON_LED_PIN, LOW);
  }
  
  // Write pixel array to led strand
  FastLED.show();
  
  // If tank is full, blink launch button
  if (tank_percent_full >= 1 && panel_power_switch) {
    if (blink_timer > millis()) {
      digitalWrite(LAUNCH_BUTTON_LED_PIN, HIGH);
    } else {
      digitalWrite(LAUNCH_BUTTON_LED_PIN, LOW);
    }
  } else {
    digitalWrite(LAUNCH_BUTTON_LED_PIN, LOW);
  }
  
  // Decriment animation index used by the chasing leds
  if (chasing_animation_timer < millis()) {
    chasing_animation_index += 1;
    // Reset index occasionally to prevent overruns
    if (chasing_animation_index == 10000) {
      chasing_animation_index = 0;
    }
    chasing_animation_timer = millis() + chasing_animation_frame_delay;
  }
  
  // Reset blink timer
  if (blink_timer + 500 < millis()) {
    blink_timer = millis() + 500;
  }
} // End Main Loop

void setFuelPumpLEDs () {
  // Set pump ring LEDs, they need to chase if fueling
  for (int i=0; i<NUM_RING_LEDS; i++) {
    if (fueling) {
      int pump_angle = (chasing_animation_index + i) % 8;
      if (pump_angle == 0) {
        pixels[i] = CRGB(max_brightness / 3, max_brightness / 4, max_brightness);
      } else if (pump_angle == 1) {
        pixels[i] = CRGB(max_brightness / 6, max_brightness / 8, max_brightness);
      } else if (pump_angle == 2) {
        pixels[i] = CRGB(0, 0, max_brightness);
      } else if (pump_angle == 3) {
        pixels[i] = CRGB(0, 0, max_brightness / 3 );
      } else if (pump_angle == 4) {
        pixels[i] = CRGB(0, 0, max_brightness / 8 );
      } else if (pump_angle == 5) {
        pixels[i] = CRGB(0, 0, max_brightness / 10 );
      } else if (pump_angle == 5) {
        pixels[i] = CRGB(0, 0, 1 );
      } else {
        pixels[i] = CRGB(0, 0, 0);
      }
    } else {
      pixels[i] = CRGB(0, 0, 0);
    }
  }
}

void setRocketTankLEDs () {
  // Set fuel tank LEDs
    // First we set the color of the fuel tank leds
    // Set red to stay on for the 3/4 and fade through the last 1/4
    // This fades the red to yellow then to green
    if (tank_percent_full < 0.75) {
      tank_red = max_brightness;
    } else if (tank_percent_full >= 0.75 && tank_percent_full < 0.99) {
      tank_red = (1 - ((tank_percent_full) - 0.75) * 4 ) * max_brightness;
    } else if (tank_percent_full >= 0.97) {
      tank_red = 0;
    }
    
    // Now Set Green to fade in to create yellow and full on when full
    if (tank_percent_full < 0.5) {
      tank_green = 0;
    } else if (tank_percent_full >= 0.5 && tank_percent_full < 1) {
      tank_green = (((tank_percent_full) - 0.5) * 2 ) * max_brightness;
    } else if (tank_percent_full >= 1) {
      tank_green = max_brightness;
    }
    
    tank_blue = 0;
    
  for (int i=NUM_RING_LEDS; i<TOTAL_LEDS; i++) {
    
    // Now we set how many of the leds are actually on, more as the tank fills.
    if ((tank_percent_full) < (((float)i - NUM_RING_LEDS) / NUM_SHIP_LEDS)) {
      red = 0;
      green = 0;
      blue = 0;
    } else {
      blue = tank_blue;
      red = tank_red;
      green = tank_green;
    }
    
    // If tank is empty, blink first two LEDS
    if (tank_percent_full < .05 && (i == NUM_RING_LEDS || i == NUM_RING_LEDS + 1)) {
      if (blink_timer > millis()) {
        blue = 0;
        red = max_brightness;
        green = 0;
      } else {
        blue = 0;
        red = 0;
        green = 0;
      }
    }
    
    // Add chasing blue leds if fueling, but only on lit portion of the scale
    if (fueling && (red + green + blue != 0)) {
      if ((i - chasing_animation_index) % 12 == 0) {
        green = 0;
        red = 0;
        blue = max_brightness;
      } else if ((i - 1 - chasing_animation_index) % 12 == 0) {
        green = 0;
        red = 0;
        blue = max_brightness;
      } else if ((i - 2 - chasing_animation_index) % 12 == 0) {
        green = max_brightness / 3;
        red = max_brightness / 4;
        blue = max_brightness;
      } else {
        blue = tank_blue;
        red = tank_red;
        green = tank_green;
      }
    }
    
    // Write colors to pixel array
    pixels[i] = CRGB(green, red, blue);
  }
}

float tankPercentFull () {
  // This assumes that the sensor reading calibrations are set from 0 - 100 PSI
  pressure_sensor_reading = runningAveragePressure();
  current_pressure = ((pressure_sensor_reading - empty_pressure_reading) / 
    (full_pressure_reading - empty_pressure_reading)) * 100.0;
  return current_pressure / max_pressure_setting;
}

float runningAveragePressure () {
  // Take a sample from the pressure sensor, incorprate it in to the ring buffer, return the smoothed value.
  sum -= measurments[index];
  measurments[index] = analogRead(TANK_PRESSURE_SENSOR_PIN);
  sum += measurments[index];
  index++;
  index = index % measurment_count;
  return (float)sum / (float)measurment_count;
}

void setMaxPressure () {
  // Turn off all leds to prep to show white indicator dot on scale
  for (int i=0; i<TOTAL_LEDS; i++) {
    pixels[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
  
  // Loop until the button is released
  while (pressure_set_button_reading == HIGH) {
    // Find correct pixel to show indicator dot and light it
    for (int i=0; i<NUM_SHIP_LEDS; i++) {
      if (i == int((float(max_pressure_setting - absolute_min_pressure) / float(absolute_max_pressure - absolute_min_pressure)) * NUM_SHIP_LEDS - 1)) {
        pixels[i + NUM_RING_LEDS] = CRGB(max_brightness, max_brightness, max_brightness);
      } else {
        pixels[i + NUM_RING_LEDS] = CRGB(0, 0, 0);
      }
    }
    FastLED.show();
    
    // Run pressure setting up and down until button is released
    if (max_pressure_setting < absolute_max_pressure) {
      max_pressure_setting++;
    } else {
      // Max pressure reached, delay a bit to give the user a chance to release the key,
      //  then check again, and if they are still holding it, start back low 
      delay(1000);
      pressure_set_button_reading = digitalRead(PRESSURE_SET_BUTTON_PIN);
      if (pressure_set_button_reading == HIGH) {
        max_pressure_setting = absolute_min_pressure;
      }
    }
 
    delay(100);
    pressure_set_button_reading = digitalRead(PRESSURE_SET_BUTTON_PIN);
  }
}
