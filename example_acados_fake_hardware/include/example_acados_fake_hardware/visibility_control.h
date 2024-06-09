// Copyright 2023, ICube Laboratory, University of Strasbourg
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
// Author: Maciej Bednarczyk (macbednarczyk@gmail.com)

#ifndef EXAMPLE_ACADOS_FAKE_HARDWARE__VISIBILITY_CONTROL_H_
#define EXAMPLE_ACADOS_FAKE_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define EXAMPLE_ACADOS_FAKE_HARDWARE_EXPORT __attribute__((dllexport))
#define EXAMPLE_ACADOS_FAKE_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define EXAMPLE_ACADOS_FAKE_HARDWARE_EXPORT __declspec(dllexport)
#define EXAMPLE_ACADOS_FAKE_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef EXAMPLE_ACADOS_FAKE_HARDWARE_BUILDING_DLL
#define EXAMPLE_ACADOS_FAKE_HARDWARE_PUBLIC EXAMPLE_ACADOS_FAKE_HARDWARE_EXPORT
#else
#define EXAMPLE_ACADOS_FAKE_HARDWARE_PUBLIC EXAMPLE_ACADOS_FAKE_HARDWARE_IMPORT
#endif
#define EXAMPLE_ACADOS_FAKE_HARDWARE_PUBLIC_TYPE EXAMPLE_ACADOS_FAKE_HARDWARE_PUBLIC
#define EXAMPLE_ACADOS_FAKE_HARDWARE_LOCAL
#else
#define EXAMPLE_ACADOS_FAKE_HARDWARE_EXPORT __attribute__((visibility("default")))
#define EXAMPLE_ACADOS_FAKE_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define EXAMPLE_ACADOS_FAKE_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define EXAMPLE_ACADOS_FAKE_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define EXAMPLE_ACADOS_FAKE_HARDWARE_PUBLIC
#define EXAMPLE_ACADOS_FAKE_HARDWARE_LOCAL
#endif
#define EXAMPLE_ACADOS_FAKE_HARDWARE_PUBLIC_TYPE
#endif

#endif  // EXAMPLE_ACADOS_FAKE_HARDWARE__VISIBILITY_CONTROL_H_
