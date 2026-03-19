#pragma once

void ForceFeedback_setup();
void ForceFeedback_loop(float normalizedTorque);

void ForceFeedback_stop();
void ForceFeedback_setEnabled(bool enabled);
int ForceFeedback_getAppliedPwm();