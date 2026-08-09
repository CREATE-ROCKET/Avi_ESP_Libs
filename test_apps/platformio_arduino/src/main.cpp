#include <Arduino.h>

#include "../../api_smoke.h"
#include "../../as5047d_pipeline_api_smoke.h"

void setup() {
  aviApiSmoke();
  aviAs5047dPipelineApiSmoke();
}

void loop() {}
