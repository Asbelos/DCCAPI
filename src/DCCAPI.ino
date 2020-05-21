#include <Arduino.h>
#include "DCCApi.h"
void setup() {
  DCCApi::begin();
}

void loop() {
  DCCApi::loop();
}