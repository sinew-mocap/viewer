// SPDX-License-Identifier: MIT
// Unit tests for viewer/core (the neutral /sinew channel -> SOMA joint map).
#include "channel_map.h"

#include <cassert>
#include <cstdio>
#include <cstring>

int main() {
	// Lookup hits and the rig-name remap.
	assert(std::strcmp(soma_for_channel("Hips"), "Hips") == 0);
	assert(std::strcmp(soma_for_channel("LeftUpperLeg"), "LeftLeg") == 0);
	assert(std::strcmp(soma_for_channel("RightLowerArm"), "RightForeArm") == 0);
	// Miss.
	assert(soma_for_channel("Nope") == nullptr);
	// Ordered (NodeNumber) accessor + bounds.
	assert(std::strcmp(channel_map_soma(0), "Hips") == 0);
	assert(channel_map_soma(-1) == nullptr);
	assert(channel_map_soma(CHANNEL_MAP_COUNT) == nullptr);
	// Every NodeNumber maps to a non-empty SOMA name.
	for (int i = 0; i < CHANNEL_MAP_COUNT; i++) {
		const char *s = channel_map_soma(i);
		assert(s != nullptr && s[0] != '\0');
	}
	std::puts("viewer/channel_map: all unit tests passed");
	return 0;
}
