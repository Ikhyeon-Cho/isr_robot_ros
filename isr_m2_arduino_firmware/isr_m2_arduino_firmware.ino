#include "Controller.h"

void setup(void) {
  controller.Initialize();
}

void loop(void) {
  controller.SpinOnce();
}

