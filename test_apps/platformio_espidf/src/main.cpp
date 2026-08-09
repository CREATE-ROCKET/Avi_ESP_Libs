#include "../../api_smoke.h"
#include "../../as5047d_pipeline_api_smoke.h"

extern "C" void app_main() {
  aviApiSmoke();
  aviAs5047dPipelineApiSmoke();
}
