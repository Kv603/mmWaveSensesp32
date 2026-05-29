#ifndef LITELED
inline uint8_t DecayBodySign(boolean noop) {
  return (0);
}
#endif

#ifdef LITELED
// Required Libraries
#include <LiteLED.h>  // Assumes LiteLED library is installed (compatible with NeoPixel/FastLED)
#include <cmath>      // For logf, expf, and ceilf (floating point math)
#include <algorithm>  // For std::min and std::max

// --- Configuration ---
// Adjust these based on your actual hardware setup


// Global strip object (Required for Arduino structure)
// NOTE: Replace 'LiteLED' with the actual constructor if it differs (e.g., Adafruit_NeoPixel)
//LiteLED strip = LiteLED(LEDCount, LED_PIN);


/////////////////////////////////////////
inline uint8_t DecayBodySign(boolean noop) {
  LastLedUpdate = TimeNow;
  if (!UseLedStrip) return (0);
#ifdef LITELED
  //Brightness level (0-255, where 0=off, 255=full brightness)
  uint8_t current = strip.getBrightness();
  if (noop) return (current);
  if (current > 0) {
    current--;
    strip.brightness(current, true);
    //strip.show();
    return (current);
  }
#endif
  return (0);
}


// --- Helper Function for Color Mapping ---

/**
 * @brief Maps a calculated value (representing a point on the scale) to an RGB color,
 * and converts it to the 32-bit color integer.
 *
 * The colors follow the required transitions:
 * 1. < 10: Green fading into Blue
 * 2. 10 - 40: Blue fading into Red
 * 3. > 40: Solid Red
 *
 * @param value The calculated value the pixel represents on the scale.
 * @return A 32-bit color value (0xRRGGBB).
 */
uint32_t getColorForValue(float value) {
  uint8_t r = 0, g = 0, b = 0;

  // --- Color Mapping Logic ---
  if (value < 10.0f) {
    // Range 1-10: Green (V=1) fading into Blue (V=10)
    // Interpolation factor f: 0.0 at V=1, 1.0 at V=10
    float f = (value - 1.0f) / 9.0f;
    f = std::max(0.0f, std::min(1.0f, f));  // Clamp to 0.0 - 1.0

    g = (uint8_t)roundf(255.0f * (1.0f - f));  // Green fades out
    b = (uint8_t)roundf(255.0f * f);           // Blue fades in
  } else if (value <= 40.0f) {
    // Range 10-40: Blue (V=10) fading into Red (V=40)
    // Interpolation factor f: 0.0 at V=10, 1.0 at V=40
    float f = (value - 10.0f) / 30.0f;
    f = std::max(0.0f, std::min(1.0f, f));  // Clamp to 0.0 - 1.0

    b = (uint8_t)roundf(255.0f * (1.0f - f));  // Blue fades out
    r = (uint8_t)roundf(255.0f * f);           // Red fades in
  } else {
    // Range > 40: Solid Red
    r = 255;
  }

  // Manual RGB to 32-bit color conversion (0xRRGGBB)
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}



//////////////////////////
boolean startLedStrip() {

  if (!strip.isGpioAvailable(LED_PIN)) {
    log_e("Cannot start LED strip, GPIO %d is occupied.", LED_PIN);
    return (UseLedStrip = false);
  }
  esp_err_t result1 = strip.begin(LED_PIN, LEDCount);
  if (result1 != ESP_OK) {
    log_e("Failed to start LED strip!");
    return (UseLedStrip = false);
  }
  UseLedStrip = true;
  strip.brightness(StripBrightness / 4, true);
  strip.fillRandom(true);
  log_d("Setup LED strip on GPIO # %d with %d pixels", LED_PIN, LEDCount);
  do_delay(300);
  //strip.brightness(StripBrightness);
  strip.clear(true);
  return (UseLedStrip);
}

// --- Main Subroutine ---

/**
 * @brief Updates an LED strip to display an unsigned integer value on a logarithmic scale.
 *
 * @param strip Pointer to the LiteLED strip object.
 * @param currentValue The input value to display.
 * @param numPixels The total number of addressable pixels in the strip.
 */
void updateLedMeter(LiteLED* strip, unsigned int currentValue, unsigned int numPixels) {
  // Static state variable: stores the highest value seen so far.
  // This value defines the peak of the logarithmic scale (all pixels illuminated).
  static unsigned int highestValueSeen = BASE_LED_VALUE_HIGHWATER;

  if (!UseLedStrip) return;  // Exit if our LED strip isn't functional
  strip->brightness(StripBrightness);
  // --- 1. Handle Zero Value ---
  if (currentValue == 0) {
    for (unsigned int i = 0; i < numPixels; i++) {
      //strip->setPixelColor(i, 0);  // Turn all pixels off (color 0 is black/off)
      strip->setPixel(i, 0);  // Turn all pixels off (color 0 is black/off)
    }
    strip->show();
    // Do not update highestValueSeen
    return;
  }

  // --- 2. Update Highest Value Seen ---
  if (currentValue > highestValueSeen) {
    highestValueSeen = currentValue;
    Serial.printf("New Peak Value Observed: %u\n", highestValueSeen);
  }

  // Ensure V_max is at least 1 for logarithmic calculation
  const float V_max = (float)highestValueSeen;
  const float V = (float)currentValue;

  // --- 3. Logarithmic Scaling (Calculate Pixels To Light) ---
  // Formula: P_lit = P_max * log(V+1) / log(V_max+1)

  float logV = logf(V + 1.0f);
  float logVMax = logf(V_max + 1.0f);

  int pixelsToLight = 0;

  if (logVMax > 0.0f) {
    // Calculate the proportion and round up (ceilf) to ensure that a very small
    // non-zero value still lights up at least one pixel.
    float raw_pixels = ((float)numPixels * logV) / logVMax;
    pixelsToLight = (int)ceilf(raw_pixels);

    // Clamp the result to the total number of pixels
    pixelsToLight = std::min((int)numPixels, pixelsToLight);
  }

  // --- 4. Clear and Set Pixel Colors ---

  // Clear the entire strip (necessary because pixelsToLight might be smaller than last call)
  for (unsigned int i = 0; i < numPixels; i++) {
    strip->setPixel(i, 0);
  }

  // Iterate through the determined number of pixels to be lit
  for (int i = 0; i < pixelsToLight; i++) {
    // Inverse Logarithmic Mapping: Calculate the VALUE represented by pixel 'i'
    // This is necessary because the color mapping depends on V_pixel, not 'currentValue'
    // V_pixel = exp( log(V_max + 1) * (i + 1) / numPixels ) - 1

    float logFraction = logVMax * ((float)(i + 1) / (float)numPixels);
    float pixelValue = expf(logFraction) - 1.0f;

    // Ensure the color mapping doesn't use a value beyond the current peak
    pixelValue = std::min(V_max, pixelValue);

    // Get the color for the represented pixel value and set the pixel
    // NOTE: The signature of getColorForValue has been updated, no longer passing the strip pointer.
    uint32_t color = getColorForValue(pixelValue);
    strip->setPixel(i, color);
  }

  // --- 5. Commit Changes to Strip ---
  //uint8_t current = strip->getBrightness();
  strip->show();
}
#endif
