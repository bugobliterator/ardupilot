/* Copyright 2025 The MathWorks, Inc. */

#include "AC_Simulink_Normal.h"

volatile boolean_T stopRequested;
volatile boolean_T runModel;

AC_Simulink_Normal::AC_Simulink_Normal() {}

void AC_Simulink_Normal::init() {
    stopRequested = false;
    this->setErrorStatus(0);
    MW_StringifyDefineFunction(MODEL, _initialize)();
    runModel = this->getErrorStatus();
}

void AC_Simulink_Normal::update() {
    if (runModel) {
        MW_StringifyDefineFunction(MODEL, _step)();
    }
}

void AC_Simulink_Normal::reset() {
    MW_StringifyDefineFunction(MODEL, _terminate)();
}

void AC_Simulink_Normal::setErrorStatus(const char_T* status) {
#if defined(rtmSetErrorStatus)
    rtmSetErrorStatus(MW_StringifyDefineFunction(MODEL, _M), status);
#else
    MW_StringifyDefineFunction(MODEL, _M)->setErrorStatus(status);
#endif
}

bool AC_Simulink_Normal::getErrorStatus() {
#if defined(rtmGetErrorStatus)
    return rtmGetErrorStatus(MW_StringifyDefineFunction(MODEL, _M)) == (NULL);
#else
    return MW_StringifyDefineFunction(MODEL, _M)->getErrorStatus() == (NULL);
#endif
}