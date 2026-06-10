// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// viewer/core — the /sinew channel -> SOMA joint name table.
#include "channel_map.h"

#include <string.h>

// /sinew OSC channel name (what the driver emits) -> SOMA joint name.
static const struct {
	const char *channel, *soma;
} kMap[] = {{"Hips", "Hips"},
            {"LeftUpperLeg", "LeftLeg"},
            {"RightUpperLeg", "RightLeg"},
            {"LeftLowerLeg", "LeftShin"},
            {"RightLowerLeg", "RightShin"},
            {"LeftFoot", "LeftFoot"},
            {"RightFoot", "RightFoot"},
            {"Chest", "Chest"},
            {"Head", "Head"},
            {"LeftUpperArm", "LeftArm"},
            {"RightUpperArm", "RightArm"},
            {"LeftLowerArm", "LeftForeArm"},
            {"RightLowerArm", "RightForeArm"},
            {"LeftHand", "LeftHand"},
            {"RightHand", "RightHand"}};

const char *soma_for_channel(const char *channel) {
	for (unsigned i = 0; i < sizeof(kMap) / sizeof(kMap[0]); i++) {
		if (strcmp(kMap[i].channel, channel) == 0) {
			return kMap[i].soma;
		}
	}
	return 0;
}

const char *channel_map_soma(int i) {
	if (i < 0 || i >= (int)(sizeof(kMap) / sizeof(kMap[0]))) {
		return 0;
	}
	return kMap[i].soma;
}
