// Copyright (C) 2023  Fernando García Liñán <fernandogarcialinan@gmail.com>
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU Library General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301, USA

#ifndef SIMGEAR_COLOR_SPACE_HXX
#define SIMGEAR_COLOR_SPACE_HXX

namespace simgear {

/// Transform a linear sRGB color to sRGB (gamma correction)
void eotf_sRGB(float in[3], float out[3]);
/// Transform an sRGB color to linear sRGB
void eotf_inverse_sRGB(float in[3], float out[3]);

} // namespace simgear

#endif /* SIMGEAR_COLOR_SPACE_HXX */
