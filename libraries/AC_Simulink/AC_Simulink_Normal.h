/* Copyright 2025 The MathWorks, Inc. */

#pragma once

/// @file    AC_Simulink_Normal.h
/// @brief   Normal mode code-deployment of Simulink

#include "AC_Simulink_Base.h"
#include "AC_Simulink_ModelDefines.h"

class AC_Simulink_Normal : public AC_Simulink_Base {
public:
    AC_Simulink_Normal();
    ~AC_Simulink_Normal() override {}

    void init() override;
    void update() override;
    void reset() override;

    void setErrorStatus(const char_T*);
    bool getErrorStatus();

    CLASS_NO_COPY(AC_Simulink_Normal);
};