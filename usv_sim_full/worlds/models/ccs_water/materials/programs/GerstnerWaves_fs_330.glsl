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
// CCS modification, 2026-11-11: restore the existing VRX normal/cube textures
// instead of a constant upward normal and a constant blue environment.

#version 330

uniform sampler2D bumpMap;
uniform samplerCube cubeMap;
uniform vec4 deepColor;
uniform vec4 shallowColor;
uniform float fresnelPower;
uniform float hdrMultiplier;

in block
{
  mat3 rotMatrix;
  vec3 eyeVec;
  vec2 bumpCoord;
} inPs;

out vec4 fragColor;

void main()
{
  vec3 tangentNormal = texture(bumpMap, inPs.bumpCoord).xyz * 2.0 - 1.0;
  vec3 N = normalize(inPs.rotMatrix * tangentNormal);
  vec3 E = normalize(inPs.eyeVec);
  vec3 R = reflect(E, N);

  // The bundled gz-rendering skybox uses a flipped Z cubemap convention.
  R = vec3(R.x, R.y, -R.z);
  vec4 envColor = texture(cubeMap, R);
  envColor.rgb *= (envColor.r + envColor.g + envColor.b) * hdrMultiplier;

  float facing = clamp(1.0 - dot(-E, N), 0.0, 1.0);
  float waterEnvRatio = clamp(pow(facing, fresnelPower), 0.05, 1.0);
  // These are view-angle colours, not actual bathymetry or depth refraction.
  vec4 waterColor = mix(shallowColor, deepColor, facing);
  vec4 color = mix(waterColor, envColor, waterEnvRatio);
  fragColor = vec4(color.xyz, 0.9);
}
