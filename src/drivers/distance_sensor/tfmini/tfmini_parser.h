/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file modified from sf0x_parser.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Chuong Nguyen <chnguye7@asu.edu>
 * @author Ayush Gaud <ayush.gaud@gmail.com>
 *
 * Declarations of parser for the Benewake TFmini laser rangefinder series
 */

#pragma once

#include <cstdint>

// Data Format for Benewake TFmini
// ===============================
// 9 bytes total per message:
// Byte0) 0x59
// Byte1) 0x59
// Byte2) Dist_L (low 8 bits)
// Byte3) Dist_H (high 8 bits)
// Byte4) Strength_L (low 8 bits)
// Byte5) Strength_H (high 8 bits)
// Byte6) Mode, distance mode, represented respectively by 02 (short distance) and 07 (long distance), automatically switchable by default.
// Byte7) Spare byte, 00 by default
// Byte8) CheckSum is the low 8 bits of the cumulative sum of the numbers of the first 8 bytes.

enum TFMINI_PARSE_STATE {
	TFMINI_PARSE_STATE0_UNSYNC = 0,
	TFMINI_PARSE_STATE1_SYNC_1,
	TFMINI_PARSE_STATE1_SYNC_2,
	TFMINI_PARSE_STATE2_GOT_DIST_L,
	TFMINI_PARSE_STATE2_GOT_DIST_H,
	TFMINI_PARSE_STATE3_GOT_STRENGTH_L,
	TFMINI_PARSE_STATE3_GOT_STRENGTH_H,
	TFMINI_PARSE_STATE4_GOT_RESERVED,
	TFMINI_PARSE_STATE5_GOT_QUALITY,
	TFMINI_PARSE_STATE6_GOT_CHECKSUM
};

int tfmini_parser(char c, char *parserbuf, unsigned *parserbuf_index, enum TFMINI_PARSE_STATE *state, float *dist,
		  int8_t *signal_quality);
