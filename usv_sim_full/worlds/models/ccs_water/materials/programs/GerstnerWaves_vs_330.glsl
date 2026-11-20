// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright (c) 2019 Rhys Mainwaring.
// Modified for vector Gerstner parameters (Tessendorf's wave convention).
// CCS: use a graded mesh and enhance vertical displacement for rendering only.
// Current advection uses integrated simulation-time phase, not time * velocity.

#version 330

in vec4 vertex;
uniform mat4 worldviewproj_matrix;

uniform int Nwaves;
uniform vec3 camera_position_object_space;
uniform float rescale;
uniform vec2 bumpScale;
uniform vec2 bumpSpeed;
uniform int currentAdvectionEnabled;
uniform vec2 currentUvOffset;
uniform float t;
uniform vec3 amplitude;
uniform vec3 wavenumber;
uniform vec3 omega;
uniform vec3 phase;
uniform vec3 steepness;
uniform vec2 dir0;
uniform vec2 dir1;
uniform vec2 dir2;
uniform float tau;

out block
{
  mat3 rotMatrix;
  vec3 eyeVec;
  vec2 bumpCoord;
} outVs;

struct WaveParameters {
  float k;
  float a;
  float omega;
  float phase;
  vec2 d;
  float q;
};

out gl_PerVertex
{
  vec4 gl_Position;
};

// Fade resolution before each coarser band in generate_ccs_water_mesh.py.
// This prevents short waves from aliasing into large triangles in the distance.
float waveGridSpacing(float radius)
{
  return 3.0
      + 3.0 * smoothstep(72.0, 96.0, radius)
      + 6.0 * smoothstep(336.0, 384.0, radius)
      + 12.0 * smoothstep(480.0, 576.0, radius)
      + 24.0 * smoothstep(672.0, 864.0, radius);
}

void main()
{
  WaveParameters waves[3];

  waves[0] = WaveParameters(wavenumber.x, amplitude.x, omega.x, phase.x, dir0.xy, steepness.x);
  waves[1] = WaveParameters(wavenumber.y, amplitude.y, omega.y, phase.y, dir1.xy, steepness.y);
  waves[2] = WaveParameters(wavenumber.z, amplitude.z, omega.z, phase.z, dir2.xy, steepness.z);

  vec4 P = vertex;
  // The mesh is already in metres: phase, wavelength and slopes use the
  // same coordinates as the wavefield rather than a 6x-scaled visual.
  float radius = max(abs(vertex.x), abs(vertex.y));
  float gridSpacing = waveGridSpacing(radius);
  float waveEnvelope = (1.0 - exp(-t/tau)) * (1.0 - smoothstep(1152.0, 1440.0, radius));
  // Boost gentle waves most. The extra crest displacement is bounded by
  // 0.56 m even under very rough inputs, and a zero input stays flat.
  float verticalBoost = 1.0 + 0.56 / (0.4 + dot(abs(amplitude), vec3(1.0)));

  vec3 B = vec3(1.0, 0.0, 0.0);
  vec3 T = vec3(0.0, 1.0, 0.0);
  vec3 N = vec3(0.0, 0.0, 1.0);

  for(int i = 0; i < Nwaves; ++i)
  {
    float k = waves[i].k;
    float resolved = 1.0 - smoothstep(1.2, 2.3, k * gridSpacing);
    float a = waves[i].a * waveEnvelope * resolved;
    if (a == 0.0)
      continue;
    float az = a * verticalBoost;
    float q = waves[i].q;
    float dx = waves[i].d.x;
    float dy = waves[i].d.y;
    float theta = dot(waves[i].d, vertex.xy)*k - t*waves[i].omega + waves[i].phase;
    float c = cos(theta);
    float s = sin(theta);

    P.x -= q*a*dx*s;
    P.y -= q*a*dy*s;
    P.z += az*c;

    float ka = a*k;
    float qkac = q*ka*c;
    float kas = az*k*s;
    float dxy = dx*dy;

    B += vec3(-qkac*dx*dx, -qkac*dxy, -kas*dx);
    T += vec3(-qkac*dxy, -qkac*dy*dy, -kas*dy);
    N += vec3(dx*kas, dy*kas, -qkac);
  }

  B = normalize(B)*rescale;
  T = normalize(T)*rescale;
  N = normalize(N);
  outVs.rotMatrix = mat3(B, T, N);

  gl_Position = worldviewproj_matrix * P;
  // Sampling at (uv - offset) moves visible features WITH positive current.
  // The fallback retains legacy animation when the optional subscription is off.
  vec2 drift = currentAdvectionEnabled != 0 ? -currentUvOffset : t*bumpSpeed;
  // Preserve the legacy UV span and V direction without importing nearly
  // identical UVs, which the COLLADA loader can merge on the dense grid.
  vec2 uv = vec2(0.5) + vertex.xy * vec2(1.0, -1.0) / 36000.0;
  outVs.bumpCoord = uv*bumpScale + drift;
  outVs.eyeVec = P.xyz - camera_position_object_space;
}
