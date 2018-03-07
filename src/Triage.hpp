#ifndef TRIAGE_H
#define TRIAGE_H

#include "Topmain.h"
#include "Tools.hpp"

#pragma SDS data copy(InputDisps, TriageResult)
#pragma SDS data access_pattern(InputDisps:SEQUENTIAL)
#pragma SDS data access_pattern(TriageResult:SEQUENTIAL)
void triage (ap_int<16> InputDisps[rows*cols], ap_int<16> TriageResult[rows*cols]);

#endif
