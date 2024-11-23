/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

// Newer simulator needs these
#define X_MIN_ENDSTOP_HIT_STATE DISABLED(X_MIN_ENDSTOP_INVERTING)
#define Y_MIN_ENDSTOP_HIT_STATE DISABLED(Y_MIN_ENDSTOP_INVERTING)
#define Z_MIN_ENDSTOP_HIT_STATE DISABLED(Z_MIN_ENDSTOP_INVERTING)
#define I_MIN_ENDSTOP_HIT_STATE DISABLED(I_MIN_ENDSTOP_INVERTING)
#define J_MIN_ENDSTOP_HIT_STATE DISABLED(J_MIN_ENDSTOP_INVERTING)
#define K_MIN_ENDSTOP_HIT_STATE DISABLED(K_MIN_ENDSTOP_INVERTING)
#define U_MIN_ENDSTOP_HIT_STATE DISABLED(U_MIN_ENDSTOP_INVERTING)
#define V_MIN_ENDSTOP_HIT_STATE DISABLED(V_MIN_ENDSTOP_INVERTING)
#define W_MIN_ENDSTOP_HIT_STATE DISABLED(W_MIN_ENDSTOP_INVERTING)
#define X_MAX_ENDSTOP_HIT_STATE DISABLED(X_MAX_ENDSTOP_INVERTING)
#define Y_MAX_ENDSTOP_HIT_STATE DISABLED(Y_MAX_ENDSTOP_INVERTING)
#define Z_MAX_ENDSTOP_HIT_STATE DISABLED(Z_MAX_ENDSTOP_INVERTING)
#define I_MAX_ENDSTOP_HIT_STATE DISABLED(I_MAX_ENDSTOP_INVERTING)
#define J_MAX_ENDSTOP_HIT_STATE DISABLED(J_MAX_ENDSTOP_INVERTING)
#define K_MAX_ENDSTOP_HIT_STATE DISABLED(K_MAX_ENDSTOP_INVERTING)
#define U_MAX_ENDSTOP_HIT_STATE DISABLED(U_MAX_ENDSTOP_INVERTING)
#define V_MAX_ENDSTOP_HIT_STATE DISABLED(V_MAX_ENDSTOP_INVERTING)
#define W_MAX_ENDSTOP_HIT_STATE DISABLED(W_MAX_ENDSTOP_INVERTING)
#define Z_MIN_PROBE_ENDSTOP_HIT_STATE DISABLED(Z_MIN_PROBE_ENDSTOP_INVERTING)
