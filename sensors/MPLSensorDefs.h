/*
 * Copyright (C) 2008 The Android Open Source Project
 * Copyright (C) 2011 Invensense, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * Common definitions for MPL sensor devices.
 */

#ifndef ANDROID_MPL_SENSOR_DEFS_H
#define ANDROID_MPL_SENSOR_DEFS_H

#define MPLROTATIONVECTOR_DEF {                         \
    .name = "MPL rotation vector",                      \
    .vendor = "Invensense",                             \
    .version = 1,                                       \
    .handle = ID_RV,                                    \
    .type = SENSOR_TYPE_ROTATION_VECTOR,                \
    .maxRange = 10240.0f,                               \
    .resolution = 1.0f,                                 \
    .power = 0.5f,                                      \
    .minDelay = 20000,                                  \
    .fifoReservedEventCount = 0,                        \
    .fifoMaxEventCount = 0 }

#define MPLLINEARACCEL_DEF {                            \
    .name = "MPL linear accel",                         \
    .vendor = "Invensense",                             \
    .version = 1,                                       \
    .handle = ID_LA,                                    \
    .type = SENSOR_TYPE_LINEAR_ACCELERATION,            \
    .maxRange = 10240.0f,                               \
    .resolution = 1.0f,                                 \
    .power = 0.5f,                                      \
    .minDelay = 20000,                                  \
    .fifoReservedEventCount = 0,                        \
    .fifoMaxEventCount = 0 }

#define MPLGRAVITY_DEF {                                \
    .name = "MPL gravity",                              \
    .vendor = "Invensense",                             \
    .version = 1,                                       \
    .handle = ID_GR,                                    \
    .type = SENSOR_TYPE_GRAVITY,                        \
    .maxRange = 10240.0f,                               \
    .resolution = 1.0f,                                 \
    .power = 0.5f,                                      \
    .minDelay = 20000,                                  \
    .fifoReservedEventCount = 0,                        \
    .fifoMaxEventCount = 0 }

#define MPLGYRO_DEF {                                   \
    .name = "MPL Gyro",                                 \
    .vendor = "Invensense",                             \
    .version = 1,                                       \
    .handle = ID_GY,                                    \
    .type = SENSOR_TYPE_GYROSCOPE,                      \
    .maxRange = 10240.0f,                               \
    .resolution = 1.0f,                                 \
    .power = 0.5f,                                      \
    .minDelay = 20000,                                  \
    .fifoReservedEventCount = 0,                        \
    .fifoMaxEventCount = 0 }

#define MPLACCEL_DEF {                                  \
    .name = "MPL accel",                                \
    .vendor = "Invensense",                             \
    .version = 1,                                       \
    .handle = ID_A,                                     \
    .type = SENSOR_TYPE_ACCELEROMETER,                  \
    .maxRange = 10240.0f,                               \
    .resolution = 1.0f,                                 \
    .power = 0.5f,                                      \
    .minDelay = 20000,                                  \
    .fifoReservedEventCount = 0,                        \
    .fifoMaxEventCount = 0 }

#define MPLMAGNETICFIELD_DEF {                          \
    .name = "MPL magnetic field",                       \
    .vendor = "Invensense",                             \
    .version = 1,                                       \
    .handle = ID_M,                                     \
    .type = SENSOR_TYPE_MAGNETIC_FIELD,                 \
    .maxRange = 10240.0f,                               \
    .resolution = 1.0f,                                 \
    .power = 0.5f,                                      \
    .minDelay = 20000,                                  \
    .fifoReservedEventCount = 0,                        \
    .fifoMaxEventCount = 0 }

#define MPLORIENTATION_DEF {                            \
    .name = "MPL Orientation",                          \
    .vendor = "Invensense",                             \
    .version = 1,                                       \
    .handle = ID_O,                                     \
    .type = SENSOR_TYPE_ORIENTATION,                    \
    .maxRange = 360.0f,                                 \
    .resolution = 1.0f,                                 \
    .power = 9.7f,                                      \
    .minDelay = 20000,                                  \
    .fifoReservedEventCount = 0,                        \
    .fifoMaxEventCount = 0 }

#endif
