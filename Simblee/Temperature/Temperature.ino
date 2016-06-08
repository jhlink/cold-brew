#include <SimbleeForMobile.h>

#include "log_png.h"

void setup()
{
  SimbleeForMobile.advertisementData = "prisma";

  SimbleeForMobile.domain = "prisma.firstbuild.com";

  // establish a baseline to use the cache during development to bypass uploading
  // the image each time
  SimbleeForMobile.baseline = "Jun8";

  // start SimbleeForMobile
  SimbleeForMobile.begin();
}

bool first_sample;

float first_temp;
float min_temp;
float max_temp;
float temp_range;
int red;

uint8_t text;
uint8_t mercury;
uint8_t rtextfield;
uint8_t rslider;

// include newlib printf float support (%f used in sprintf below)
asm(".global _printf_float");

void loop()
{
  // sample once per second
  // todo: Simblee_ULPDelay( SECONDS(1) );

  if (SimbleeForMobile.updatable)
  {
    float temp = Simblee_temperature(CELSIUS);

  
    // requires newlib printf float support
    char buf[16];
    sprintf(buf, "%.02f", temp);
  
    // base everything on the first sample / ambient temperature
    if (first_sample)
    {
      first_temp = temp;
      
      // putting your finger on the Simblee shield raises the temp approx 2 degrees
      min_temp = first_temp - 0.25;
      max_temp = first_temp + 1.0;
      temp_range = max_temp - min_temp;
      
      first_sample = false;
    }

    // update the text first with the actual temp
    SimbleeForMobile.updateText(text, buf);

    // restrict temp to range
    if (temp < min_temp)
      temp = min_temp;
    if (temp > max_temp)
      temp = max_temp;

    int mercury_range = 262;
    int mercury_position = ((temp - min_temp) / temp_range) * mercury_range;

    // invert so the smallest value at the bottom
    mercury_position = mercury_range - (mercury_position);

    // update the mercury
    SimbleeForMobile.updateRect(mercury, 65, 136 + mercury_position, 33, 262 + 15 - mercury_position);
  }

  // process must be called in the loop for SimbleeForMobile
  SimbleeForMobile.process();
}

void SimbleeForMobile_onConnect()
{
  first_sample = true;
}

void ui()
{
  #define  IMAGE1  1

  color_t darkgray = rgb(85,85,85);
  SimbleeForMobile.beginScreen(WHITE);

  SimbleeForMobile.drawText(25, 71, "Pressure:", WHITE);
  rslider = SimbleeForMobile.drawSlider(55, 65, 175, 0, 15);
  rtextfield = SimbleeForMobile.drawTextField(245, 65, 50, 255, "", WHITE, darkgray);

  // usable area: 56, 136, 51, 262
  // mercury area: 65, 136, 33, 262 + 15
  mercury = SimbleeForMobile.drawRect(65, 136, 33, 262 + 15, rgb(160,0,0), rgb(204,0,0));

  // hide the mercury until the image is uploadeds
  SimbleeForMobile.setVisible(mercury, false);
  
//  SimbleeForMobile.imageSource(IMAGE1, PNG, logo_png, logo_png_len);  
//  SimbleeForMobile.drawImage(IMAGE1, 30, 100);
  
  SimbleeForMobile.setVisible(mercury, true);

  SimbleeForMobile.endScreen();

//  update();
}

void update() {
    
    
//  SimbleeForMobile.updateValue(rslider, red);
//  SimbleeForMobile.updateValue(rtextfield, red);
}

void ui_event(event_t &event)
{
    if (event.id == rslider || event.id == rtextfield) {
    red = event.value;
    }
//    update();
}
