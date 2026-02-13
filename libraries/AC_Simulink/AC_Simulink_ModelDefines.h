/* Copyright 2025 The MathWorks, Inc. */

#pragma once

/// @file    AC_Simulink_ModelDefines.h
/// @brief   Simulink library

#define MW_StringifyDefine(x) MW_StringifyDefineExpanded(x)
#define MW_StringifyDefineExpanded(x) #x

#define MW_StringifyDefineFunction(x, y) MW_StringifyDefineExpandedFunction(x, y)
#define MW_StringifyDefineExpandedFunction(x, y) x##y

#define MW_StringifyDefineX(x) MW_StringifyDefineExpandedX(x)
#define MW_StringifyDefineExpandedX(x) x.##h

#define MW_StringifyDefineTypesX(x) MW_StringifyDefineExpandedTypesX(x)
#define MW_StringifyDefineExpandedTypesX(x) x##_types.h

#define MW_StringifyDefinePrivateX(x) MW_StringifyDefineExpandedPrivateX(x)
#define MW_StringifyDefineExpandedPrivateX(x) x##_private.h

#include MW_StringifyDefine(MODEL.h)
#include MW_StringifyDefine(MW_StringifyDefineTypesX(MODEL))
#include MW_StringifyDefine(MW_StringifyDefinePrivateX(MODEL))
