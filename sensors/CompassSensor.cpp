/*
 * Copyright (c) 2011-2013, NVIDIA CORPORATION.  All rights reserved.
 * Copyright (C) 2012 The Android Open Source Project
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

#define LOG_NDEBUG 0

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <cutils/log.h>
#include <linux/input.h>
#include <string.h>

#include "CompassSensor.h"
#include "sensors.h"
#include "MPLSupport.h"
#include "sensor_params.h"
#include "ml_sysfs_helper.h"
#include "log.h"

#define COMPASS_MAX_SYSFS_ATTRB sizeof(compassSysFs) / sizeof(char*)


/*****************************************************************************/

CompassSensor::CompassSensor(const char *compassName)
    : SensorBase(NULL, NULL),
      mCompassTimestamp(0),
      mCompassInputReader(8),
      mCompassName(compassName),
      mEnable(0)
{
    VFUNC_LOG;

    if (mCompassName == NULL)
        mCompassName = "INV_COMPASS";

    if(inv_init_sysfs_attributes()) {
        ALOGE("Error Instantiating Compass\n");
        return;
    }

    compass_fd = openInput(mCompassName);
    if (compass_fd == -1) {
        ALOGE("HAL:could not open the compass device node");
    } else {
        ALOGV("HAL:Compass compass_fd %s opened : %d", mCompassName, compass_fd);
    }

    if (!strcmp(mCompassName, "INV_COMPASS")) {
        mI2CBus = COMPASS_BUS_SECONDARY;
    } else {
        mI2CBus = COMPASS_BUS_PRIMARY;
    }

    memset(mCachedCompassData, 0, sizeof(mCachedCompassData));

    ALOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:cat %s (%lld)",
            compassSysFs.compass_orient, getTimestamp());
    FILE *fptr;
    fptr = fopen(compassSysFs.compass_orient, "r");
    if (fptr != NULL) {
        int om[9];
        fscanf(fptr, "%d,%d,%d,%d,%d,%d,%d,%d,%d",
               &om[0], &om[1], &om[2], &om[3], &om[4], &om[5],
               &om[6], &om[7], &om[8]);
        fclose(fptr);

        ALOGV_IF(EXTRA_VERBOSE,
                "HAL:compass mounting matrix: "
                "%+d %+d %+d %+d %+d %+d %+d %+d %+d",
                om[0], om[1], om[2], om[3], om[4], om[5], om[6], om[7], om[8]);

        mCompassOrientation[0] = om[0];
        mCompassOrientation[1] = om[1];
        mCompassOrientation[2] = om[2];
        mCompassOrientation[3] = om[3];
        mCompassOrientation[4] = om[4];
        mCompassOrientation[5] = om[5];
        mCompassOrientation[6] = om[6];
        mCompassOrientation[7] = om[7];
        mCompassOrientation[8] = om[8];
    } else {
        ALOGE("HAL:Couldn't read compass mounting matrix");
    }

    enable(ID_M, 0);
}

CompassSensor::~CompassSensor()
{
    VFUNC_LOG;

    free(pathP);
    if( compass_fd > 0)
        close(compass_fd);
}

int CompassSensor::getFd() const
{
    VHANDLER_LOG;
    return compass_fd;
}

/**
 *  @brief        This function will enable/disable sensor.
 *  @param[in]    handle
 *                  which sensor to enable/disable.
 *  @param[in]    en
 *                  en=1, enable;
 *                  en=0, disable
 *  @return       if the operation is successful.
 */
int CompassSensor::enable(int32_t handle, int en)
{
    VFUNC_LOG;

    int tempFd;
    int res;

    ALOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
             en, compassSysFs.compass_enable, getTimestamp());
    tempFd = open(compassSysFs.compass_enable, O_RDWR);
    res = errno;
    if(tempFd < 0) {
        ALOGE("HAL:open of %s failed with '%s' (%d)",
              compassSysFs.compass_enable, strerror(res), res);
        return res;
    }
    res = enable_sysfs_sensor(tempFd, en);
    if (res < 0) {
        ALOGE("%s HAL:enable compass failed", __func__);
    } else {
        mEnable = en;
    }
    return res;
}

int CompassSensor::setDelay(int32_t handle, int64_t ns)
{
    VFUNC_LOG;
    int tempFd;
    int res;

    ALOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %.0f > %s (%lld)",
             (float)NS2HZ(ns), compassSysFs.compass_rate, getTimestamp());
    mDelay = ns;
    if (ns == 0)
        return -1;
    tempFd = open(compassSysFs.compass_rate, O_RDWR);
    res = write_attribute_sensor(tempFd, NS2HZ(ns));
    if(res < 0) {
        ALOGE("HAL:Compass update delay error");
    }
    return res;
}


/**
    @brief      This function will return the state of the sensor.
    @return     1=enabled; 0=disabled
**/
int CompassSensor::getEnable(int32_t handle)
{
    VFUNC_LOG;

    return mEnable;
}

/* use for Invensense compass calibration */
#define COMPASS_EVENT_DEBUG (0)
void CompassSensor::processCompassEvent(const input_event *event)
{
    VHANDLER_LOG;

    switch (event->code) {
    case EVENT_TYPE_ICOMPASS_X:
        ALOGV_IF(COMPASS_EVENT_DEBUG, "EVENT_TYPE_ICOMPASS_X\n");
        mCachedCompassData[0] = event->value;
        break;
    case EVENT_TYPE_ICOMPASS_Y:
        ALOGV_IF(COMPASS_EVENT_DEBUG, "EVENT_TYPE_ICOMPASS_Y\n");
        mCachedCompassData[1] = event->value;
        break;
    case EVENT_TYPE_ICOMPASS_Z:
        ALOGV_IF(COMPASS_EVENT_DEBUG, "EVENT_TYPE_ICOMPASS_Z\n");
        mCachedCompassData[2] = event->value;
        break;
    }

    mCompassTimestamp =
        (int64_t)event->time.tv_sec * 1000000000L + event->time.tv_usec * 1000L;
}

void CompassSensor::getOrientationMatrix(signed char *orient)
{
    VFUNC_LOG;

    memcpy(orient, mCompassOrientation, sizeof(mCompassOrientation));
}

long CompassSensor::getSensitivity()
{
    VFUNC_LOG;

    long sensitivity;
    ALOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:cat %s (%lld)",
            compassSysFs.compass_scale, getTimestamp());
    inv_read_data(compassSysFs.compass_scale, &sensitivity);
    return sensitivity;
}

int CompassSensor::fill()
{
    ssize_t n = mCompassInputReader.fill(compass_fd);
    if (n < 0)
        LOGE("HAL:no compass events read");
    return n;
}

/**
    @brief         This function is called by sensors_mpl.cpp
                   to read sensor data from the driver.
    @param[out]    data      sensor data is stored in this variable. Scaled such that
                             1 uT = 2^16
    @para[in]      timestamp data's timestamp
    @return        1, if 1   sample read, 0, if not, negative if error
 */
int CompassSensor::readSample(long *data, int64_t *timestamp)
{
    VHANDLER_LOG;

    int numEventReceived = 0, done = 0;
    input_event const* event;

    while (done == 0 && mCompassInputReader.readEvent(&event)) {
        int type = event->type;
        if (type == EV_REL) {
            processCompassEvent(event);
        } else if (type == EV_SYN) {
            *timestamp = mCompassTimestamp;
            memcpy(data, mCachedCompassData, sizeof(mCachedCompassData));
            done = 1;
        } else {
            ALOGE("HAL:Compass Sensor: unknown event (type=%d, code=%d)",
                 type, event->code);
        }
        mCompassInputReader.next();
    }

    return done;
}

/**
 *  @brief  This function will return the current delay for this sensor.
 *  @return delay in nanoseconds.
 */
int64_t CompassSensor::getDelay(int32_t handle)
{
    VFUNC_LOG;

    return mDelay;
}

int CompassSensor::inv_init_sysfs_attributes(void)
{
    VFUNC_LOG;

    unsigned char i = 0;
    char sysfs_path[MAX_SYSFS_NAME_LEN], tbuf[2];
    char *sptr;
    char **dptr;
    int num;

    pathP = (char*)malloc(
                    sizeof(char[COMPASS_MAX_SYSFS_ATTRB][MAX_SYSFS_NAME_LEN]));
    sptr = pathP;
    dptr = (char**)&compassSysFs;
    if (sptr == NULL)
        return -1;

    do {
        *dptr++ = sptr;
        sptr += sizeof(char[MAX_SYSFS_NAME_LEN]);
    } while (++i < COMPASS_MAX_SYSFS_ATTRB);

    inv_get_sysfs_abs_path(sysfs_path);

    if (!strcmp(mCompassName, "INV_COMPASS")) {
        // get sysfs absolute path
        inv_get_sysfs_abs_path(sysfs_path);
        sprintf(compassSysFs.compass_enable, "%s%s", sysfs_path, "/compass_enable");
        sprintf(compassSysFs.compass_rate, "%s%s", sysfs_path, "/fifo_rate");
        sprintf(compassSysFs.compass_scale, "%s%s", sysfs_path, "/compass_scale");
        sprintf(compassSysFs.compass_orient, "%s%s", sysfs_path, "/compass_matrix");
    } else {
        inv_get_input_number(mCompassName, &num);
        tbuf[0] = num + 0x30;
        tbuf[1] = 0;
        sprintf(sysfs_path, "%s%s", "sys/class/input/input", tbuf);
        strcat(sysfs_path, "/ak8975");
        sprintf(compassSysFs.compass_enable, "%s%s", sysfs_path, "/enable");
        sprintf(compassSysFs.compass_rate, "%s%s", sysfs_path, "/fifo_rate");
        sprintf(compassSysFs.compass_scale, "%s%s", sysfs_path, "/scale");
        sprintf(compassSysFs.compass_orient, "%s%s", sysfs_path, "/compass_matrix");
    }

#if 0
    // test print sysfs paths
    dptr = (char**)&compassSysFs;
    for (i = 0; i < COMPASS_MAX_SYSFS_ATTRB; i++) {
        ALOGE("HAL:sysfs path: %s", *dptr++);
    }
#endif
    return 0;
}
