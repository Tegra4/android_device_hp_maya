/* Copyright (C) 2012 The Android Open Source Project
 * Copyright (c) 2011-2012, NVIDIA CORPORATION.  All rights reserved.
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
//see also the EXTRA_VERBOSE define in the MPLSensor.h header file

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <float.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/select.h>
#include <sys/syscall.h>
#include <dlfcn.h>
#include <pthread.h>
#include <cutils/log.h>
#include <utils/KeyedVector.h>
#include <utils/String8.h>
#include <string.h>
#include <linux/input.h>

#include "MPLSensor.h"
#include "MPLSupport.h"
#include "sensor_params.h"

#include "invensense.h"
#include "invensense_adv.h"
#include "ml_stored_data.h"
#include "ml_load_dmp.h"
#include "ml_sysfs_helper.h"

#define MAX_SYSFS_ATTRB (sizeof(struct sysfs_attrbs) / sizeof(char*))

/******************************************************************************/
/*  MPL interface misc.                                                       */
/******************************************************************************/
static int hertz_request = 200;

#define DEFAULT_MPL_GYRO_RATE           (20000L)     //us
#define DEFAULT_MPL_COMPASS_RATE        (20000L)     //us

#define DEFAULT_HW_GYRO_RATE            (100)        //Hz
#define DEFAULT_HW_ACCEL_RATE           (20)         //ms
#define DEFAULT_HW_COMPASS_RATE         (20000000L)  //ns
#define DEFAULT_HW_AKMD_COMPASS_RATE    (200000000L) //ns

/* convert ns to hardware units */
#define HW_GYRO_RATE_NS                 (1000000000LL / rate_request) // to Hz
#define HW_ACCEL_RATE_NS                (rate_request / (1000000L))   // to ms
#define HW_COMPASS_RATE_NS              (rate_request)                // to ns

/* convert Hz to hardware units */
#define HW_GYRO_RATE_HZ                 (hertz_request)
#define HW_ACCEL_RATE_HZ                (1000 / hertz_request)
#define HW_COMPASS_RATE_HZ              (1000000000LL / hertz_request)

#define CAL_PART "/dev/block/mmcblk0p2"
#define CAL_DATA "/data/inv_cal_data.bin"

MPLSensor *MPLSensor::gMPLSensor = NULL;

extern "C" {
void procData_cb_wrapper()
{
    if(MPLSensor::gMPLSensor) {
        MPLSensor::gMPLSensor->cbProcData();
    }
}

void setCallbackObject(MPLSensor* gbpt)
{
    MPLSensor::gMPLSensor = gbpt;
}

MPLSensor* getCallbackObject() {
    return MPLSensor::gMPLSensor;
}

} //end of extern C

#ifdef INV_PLAYBACK_DBG
static FILE *logfile = NULL;
#endif

#if CAL_DATA_AUTO_LOAD == 1
#define cal_data_auto_load_path  "/cal_data_auto_load"

static int self_test_auto_load_state_read(void)
{
    int count = 0;
    char raw_buf[10];
    short raw = 0;
    char sysfs_path[MAX_SYSFS_NAME_LEN];
    int res = 0;
    int fd;

    // get sysfs absolute path
    inv_get_sysfs_abs_path(sysfs_path);
    strcat(sysfs_path, cal_data_auto_load_path);

    fd = open(sysfs_path, O_RDONLY);
    res = errno;
    if(fd < 0){
        ALOGE("HAL:Open of %s failed with '%s' (%d)",
             sysfs_path, strerror(res), res);
        return -1;
    }

    memset(raw_buf, 0, sizeof(raw_buf));
    count = read_attribute_sensor(fd, raw_buf, sizeof(raw_buf));
    if(count < 1) {
        ALOGE("HAL:error reading auto load state");
        close(fd);
        return -1;
    }
    count = sscanf(raw_buf, "%hd", &raw);
    if(count < 0) {
        ALOGE("HAL:auto load state data is invalid");
        close(fd);
        return -1;
    }
    ALOGV_IF(EXTRA_VERBOSE, "HAL:auto load state read = %d, count = %d", raw, count);
    close(fd);
    return (int)raw;
}

static void self_test_auto_load_state_write(int en)
{
    char sysfs_path[MAX_SYSFS_NAME_LEN];
    int res = 0;
    int fd;

    // get sysfs absolute path
    inv_get_sysfs_abs_path(sysfs_path);
    strcat(sysfs_path, cal_data_auto_load_path);

    fd = open(sysfs_path, O_WRONLY);
    res = errno;
    if(fd < 0){
        ALOGE("HAL:Open of %s failed with '%s' (%d)",
             sysfs_path, strerror(res), res);
        return;
    }

    if((res=enable_sysfs_sensor(fd, en)) < 0) {
        ALOGE("HAL:Couldn't write auto load state");
    } else {
        ALOGV_IF(EXTRA_VERBOSE, "HAL:auto load state write = %d", en);
    }
    close(fd);
}
#endif

static bool restore_calibration_data()
{
    int fd1, fd2, nread;
    char cal_buffer[92];

    fd1 = open(CAL_PART, O_RDONLY);
    if(fd1 < 0) {
	ALOGE("HAL:Open of %s failed", CAL_PART);
	close(fd1);
	return false;
    }

    fd2 = open(CAL_DATA, O_CREAT | O_WRONLY, 0666);
    if(fd2 < 0) {
	ALOGE("HAL:Open of %s failed", CAL_DATA);
	goto fail;
    }

    nread = read(fd1, cal_buffer, 92);

    if(nread != 92) {
	ALOGE("HAL:Read of %s failed", CAL_PART);
	goto fail;
    }

    if((write(fd2, cal_buffer, nread) != nread)) {
	ALOGE("HAL:Write of %s failed", CAL_DATA);
	goto fail;
    }

    close(fd1);
    close(fd2);
    return true;

fail:
    close(fd1);
    close(fd2);
    return false;
}
/*******************************************************************************
 * MPLSensor class implementation
 ******************************************************************************/

MPLSensor::MPLSensor(CompassSensor *compass)
                       : SensorBase(NULL, NULL),
                         mNewData(0),
                         mMasterSensorMask(INV_ALL_SENSORS),
                         mLocalSensorMask(0),
                         mPollTime(-1),
                         mHaveGoodMpuCal(0),
                         mGyroAccuracy(0),
                         mAccelAccuracy(0),
                         mSampleCount(0),
                         mEnabled(0),
                         mOldEnabledMask(0),
                         mAccelInputReader(4),
                         mGyroInputReader(32),
                         mTempScale(0),
                         mTempOffset(0),
                         mTempCurrentTime(0),
#if CAL_DATA_AUTO_LOAD == 1
                         mCalDataCurrentTime(0),
#endif
                         mAccelScale(2),
                         mPendingMask(0),
                         mSensorMask(0),
                         mGyroOrientation{0},
                         mAccelOrientation{0}
{
    VFUNC_LOG;

    inv_error_t rv;
    int i, fd;
    char *port = NULL;
    char *ver_str;
    unsigned long mSensorMask;
    int res;
    FILE *fptr;

    mCompassSensor = compass;

    ALOGV_IF(EXTRA_VERBOSE,
            "HAL:MPLSensor constructor : numSensors = %d", numSensors);

    pthread_mutex_init(&mMplMutex, NULL);

    mForceSleep = false;

    /* setup sysfs paths */
    inv_init_sysfs_attributes();

    /* get chip name */
    if (inv_get_chip_name(chip_ID) != INV_SUCCESS) {
        ALOGE("HAL:ERR- Failed to get chip ID\n");
    } else {
        ALOGV("HAL:Chip ID= %s\n", chip_ID);
    }

    /* retrieve input subsystem paths */
    mpu_int_fd = openInput(chip_ID);
    if (mpu_int_fd == -1) {
        ALOGE("HAL:could not open the gyro device node");
    } else {
        ALOGV("HAL:Gyro mpu_int_fd %s opened : %d", chip_ID, mpu_int_fd);
    }

    /*
    accel_fd = openInput(chip_ID);
    if (accel_fd == -1) {
        ALOGE("HAL:could not open the accel device node");
    } else {
        ALOGV("HAL:Accel accel_fd %s opened : %d", chip_ID, accel_fd);
    }
    */

    /* turn on power state */
    onPower(1);

    /* reset driver master enable */
    masterEnable(0);

    /*
    keep disabled because not needed and causes higher than normal I2C traffic
    loadDMPTap();
    */

    /* disable fifo output */
    ALOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:echo 0 > %s (%lld)",
            mpu.gyro_fifo_enable, getTimestamp());
    fd = open(mpu.gyro_fifo_enable, O_RDWR);
    if (fd < 0) {
        ALOGE("HAL:could not open gyro fifo enable");
    } else {
        if (enable_sysfs_sensor(fd, 0) < 0) {
            ALOGE("HAL:could not disable gyro fifo");
        }
    }

    ALOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:echo 0 > %s (%lld)",
            mpu.accel_fifo_enable, getTimestamp());
    fd = open(mpu.accel_fifo_enable, O_RDWR);
    if (fd < 0) {
        ALOGE("HAL:could not open accel enable");
    } else {
        if (enable_sysfs_sensor(fd, 0) < 0) {
            ALOGE("HAL:could not disable accel fifo");
        }
    }

    /* open temperature fd for temp comp */
    ALOGV_IF(EXTRA_VERBOSE, "HAL:gyro temperature path: %s", mpu.temperature);
    gyro_temperature_fd = open(mpu.temperature, O_RDONLY);
    if (gyro_temperature_fd == -1) {
        ALOGE("HAL:could not open temperature node");
    } else {
        ALOGV_IF(EXTRA_VERBOSE,
                "HAL:temperature_fd opened: %s", mpu.temperature);
    }

    /* read temperature scale */
    char buf[8];
    int count = 0;

    ALOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:cat %s (%lld)", mpu.temp_scale, getTimestamp());
    fd = open(mpu.temp_scale, O_RDONLY);
    if (fd < 0) {
        ALOGE("HAL:Error opening gyro temp scale");
    }
    memset(buf, 0, sizeof(buf));
    count = read_attribute_sensor(fd, buf, sizeof(buf));
    if (count < 1) {
        ALOGE("HAL:Error reading gyro temp scale");
    } else {
        count = sscanf(buf, "%hd", &mTempScale);
        if (count)
            ALOGV_IF(EXTRA_VERBOSE, "HAL:Temparature scale used %d", mTempScale);
    }
    close(fd);

    /* read temperature offset */
    ALOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:cat %s (%lld)", mpu.temp_offset, getTimestamp());
    fd = open(mpu.temp_offset, O_RDONLY);
    if (fd < 0) {
        ALOGE("HAL:error opening gyro temp offset");
    } else {
       memset(buf, 0, sizeof(buf));
       count = read_attribute_sensor(fd, buf, sizeof(buf));
       if (count < 0) {
           ALOGE("HAL:error reading gyro temp offset");
       } else {
          count = sscanf(buf, "%hd", &mTempOffset);
          if (count)
              ALOGV_IF(EXTRA_VERBOSE,
                      "HAL:Temperature offset used %d", mTempOffset);
       }
       close(fd);
    }

    /* read accel FSR to calcuate accel scale later */
    ALOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:cat %s (%lld)", mpu.accel_fsr, getTimestamp());

    fd = open(mpu.accel_fsr, O_RDONLY);
    if (fd < 0) {
        ALOGE("HAL:Error opening accel FSR");
    } else {
        char buf[3];
        int count = 0;
        memset(buf, 0, sizeof(buf));
        count = read_attribute_sensor(fd, buf, sizeof(buf));
        if (count < 1) {
            ALOGE("HAL:Error reading accel FSR");
        } else {
            count = sscanf(buf, "%d", &mAccelScale);
            if (count)
                ALOGV_IF(EXTRA_VERBOSE, "HAL:Accel FSR used %d", mAccelScale);
        }
        close(fd);
    }

    /* initialize sensor data */
    memset(mPendingEvents, 0, sizeof(mPendingEvents));

    mPendingEvents[RotationVector].version = sizeof(sensors_event_t);
    mPendingEvents[RotationVector].sensor = ID_RV;
    mPendingEvents[RotationVector].type = SENSOR_TYPE_ROTATION_VECTOR;
    mPendingEvents[RotationVector].acceleration.status
            = SENSOR_STATUS_ACCURACY_HIGH;

    mPendingEvents[LinearAccel].version = sizeof(sensors_event_t);
    mPendingEvents[LinearAccel].sensor = ID_LA;
    mPendingEvents[LinearAccel].type = SENSOR_TYPE_LINEAR_ACCELERATION;
    mPendingEvents[LinearAccel].acceleration.status
            = SENSOR_STATUS_ACCURACY_HIGH;

    mPendingEvents[Gravity].version = sizeof(sensors_event_t);
    mPendingEvents[Gravity].sensor = ID_GR;
    mPendingEvents[Gravity].type = SENSOR_TYPE_GRAVITY;
    mPendingEvents[Gravity].acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;

    mPendingEvents[Gyro].version = sizeof(sensors_event_t);
    mPendingEvents[Gyro].sensor = ID_GY;
    mPendingEvents[Gyro].type = SENSOR_TYPE_GYROSCOPE;
    mPendingEvents[Gyro].gyro.status = SENSOR_STATUS_ACCURACY_HIGH;

    mPendingEvents[Accelerometer].version = sizeof(sensors_event_t);
    mPendingEvents[Accelerometer].sensor = ID_A;
    mPendingEvents[Accelerometer].type = SENSOR_TYPE_ACCELEROMETER;
    mPendingEvents[Accelerometer].acceleration.status
            = SENSOR_STATUS_ACCURACY_HIGH;

    /* Invensense compass calibration */
    mPendingEvents[MagneticField].version = sizeof(sensors_event_t);
    mPendingEvents[MagneticField].sensor = ID_M;
    mPendingEvents[MagneticField].type = SENSOR_TYPE_MAGNETIC_FIELD;
    mPendingEvents[MagneticField].magnetic.status =
        SENSOR_STATUS_ACCURACY_HIGH;

    mPendingEvents[Orientation].version = sizeof(sensors_event_t);
    mPendingEvents[Orientation].sensor = ID_O;
    mPendingEvents[Orientation].type = SENSOR_TYPE_ORIENTATION;
    mPendingEvents[Orientation].orientation.status
            = SENSOR_STATUS_ACCURACY_HIGH;

    mHandlers[RotationVector] = &MPLSensor::rvHandler;
    mHandlers[LinearAccel] = &MPLSensor::laHandler;
    mHandlers[Gravity] = &MPLSensor::gravHandler;
    mHandlers[Gyro] = &MPLSensor::gyroHandler;
    mHandlers[Accelerometer] = &MPLSensor::accelHandler;
    mHandlers[MagneticField] = &MPLSensor::compassHandler;
    mHandlers[Orientation] = &MPLSensor::orienHandler;

    for (int i = 0; i < numSensors; i++) {
        mDelays[i] = 0;
    }

    (void)inv_get_version(&ver_str);
    ALOGI("%s\n", ver_str);

    /* setup MPL */
    inv_constructor_init();

    /* load calibration file from /data/inv_cal_data.bin */
    rv = inv_load_calibration();
    if (rv == INV_SUCCESS)
        ALOGV("HAL:Calibration file successfully loaded");
    else
    {
	res = restore_calibration_data();
	if(!res)
            ALOGE("HAL:Could not open or load MPL calibration file");
	else
	    ALOGV("HAL:Calibration file successfully loaded");
    }
#if CAL_DATA_AUTO_LOAD == 1
    //set to "0" after loading calibration
    self_test_auto_load_state_write(0);
#endif

    inv_set_device_properties();

    /* disable driver master enable the first sensor goes on */
    masterEnable(0);
    enableGyro(0);
    enableAccel(0);
    enableCompass(0);
    onPower(0);

#ifdef INV_PLAYBACK_DBG
    logfile = fopen("/data/playback.bin", "wb");
    if (logfile)
        inv_turn_on_data_logging(logfile);
#endif
}

int MPLSensor::inv_constructor_init()
{
    VFUNC_LOG;

    inv_error_t result = inv_init_mpl();
    if (result) {
        ALOGE("HAL:inv_init_mpl() failed");
        return result;
    }
    result = inv_constructor_default_enable();
    if (result) {
        ALOGE("HAL:inv_constructor_default_enable() failed");
        LOG_RESULT_LOCATION(result);
        return result;
    }
    result = inv_start_mpl();
    if (result) {
        ALOGE("HAL:inv_start_mpl() failed");
        LOG_RESULT_LOCATION(result);
        return result;
    }

    return result;
}

int MPLSensor::inv_constructor_default_enable()
{
    VFUNC_LOG;

    inv_error_t result;

    result = inv_enable_quaternion();
    if (result) {
        ALOGE("HAL:Cannot enable quaternion\n");
        return result;
    }

    ALOGV("HAL:in_use_auto_calibration, disable");
    mAccelAccuracy = 3;
    inv_set_accel_bias(NULL, 3);

    if (result) {
        return result;
    }
    result = inv_enable_fast_nomot();
    if (result) {
        return result;
    }
    result = inv_enable_gyro_tc();
    if (result) {
        return result;
    }
    result = inv_enable_hal_outputs();
    if (result) {
        return result;
    }

    if (mCompassSensor != NULL) {
        if (!mCompassSensor->providesCalibration()) {
            /* Invensense compass calibration */
            ALOGV("HAL:Invensense vector compass cal enabled");
            result = inv_enable_vector_compass_cal();
            if (result) {
                LOG_RESULT_LOCATION(result);
                return result;
            }

        ALOGV("inv_vector_compass_cal_sensitivity, 3");
        inv_vector_compass_cal_sensitivity(3);

            result = inv_enable_compass_bias_w_gyro();
            if (result) {
                LOG_RESULT_LOCATION(result);
                return result;
            }

            result = inv_enable_heading_from_gyro();
            if (result) {
                LOG_RESULT_LOCATION(result);
                return result;
            }

            result = inv_enable_magnetic_disturbance();
            if (result) {
                LOG_RESULT_LOCATION(result);
                return result;
            }
        }
    }
    result = inv_enable_9x_sensor_fusion();
    if (result) {
        LOG_RESULT_LOCATION(result);
        return result;
    }

    result = inv_enable_no_gyro_fusion();
    if (result) {
        LOG_RESULT_LOCATION(result);
        return result;
    }
    return result;
}

/* TODO: create function pointers to calculate scale */
void MPLSensor::inv_set_device_properties()
{
    VFUNC_LOG;

    unsigned short orient;

    inv_get_sensors_orientation();

    inv_set_gyro_sample_rate(DEFAULT_MPL_GYRO_RATE);
    inv_set_compass_sample_rate(DEFAULT_MPL_COMPASS_RATE);

    /* gyro setup */
    orient = inv_orientation_matrix_to_scalar(mGyroOrientation);
    inv_set_gyro_orientation_and_scale(orient, 2000L << 15);

    /* accel setup */
    orient = inv_orientation_matrix_to_scalar(mAccelOrientation);
    // BMA250
    //inv_set_accel_orientation_and_scale(orient, 1LL << 22);
    // MPU6050
    inv_set_accel_orientation_and_scale(orient, mAccelScale << 15);

    /* compass setup */
    if (mCompassSensor != NULL) {
        signed char orientMtx[9];

        mCompassSensor->getOrientationMatrix(orientMtx);
        orient = inv_orientation_matrix_to_scalar(orientMtx);
        long sensitivity;
        sensitivity = mCompassSensor->getSensitivity();
        inv_set_compass_orientation_and_scale(orient, sensitivity);
    }
}

void MPLSensor::loadDMPTap()
{
    int res, fd;
    FILE *fptr;

    /* load DMP firmware */
    ALOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:cat %s (%lld)", mpu.firmware_loaded, getTimestamp());
    fd = open(mpu.firmware_loaded, O_RDONLY);
    if (fd < 0) {
        ALOGE("HAL:could not open dmp state");
    } else {
        if (inv_read_dmp_state(fd) == 0) {
            ALOGV_IF(EXTRA_VERBOSE, "HAL:load dmp: %s", mpu.dmp_firmware);
            fptr = fopen(mpu.dmp_firmware, "w");
            if (!fptr) {
                ALOGE("HAL:could not write to dmp");
            } else {
                int res = inv_load_dmp(fptr);
                if (res < 0) {
                    ALOGE("HAL:load DMP failed");
                } else {
                    ALOGI("HAL:DMP loaded");
                }
                fclose(fptr);
            }
        } else {
            ALOGV("HAL:DMP is already loaded");
        }
    }

    /* turn on tap */
    ALOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:echo 1 > %s (%lld)", mpu.tap_on, getTimestamp());
    fd = open(mpu.tap_on, O_RDWR);
    if (fd < 0) {
        ALOGE("HAL:could not open enable dmp tap node");
    } else {
        if (enable_sysfs_sensor(fd, 1) < 0) {
            ALOGE("HAL:could not enable dmp tap features");
        }
    }
}

void MPLSensor::inv_get_sensors_orientation()
{
    FILE *fptr;

    // get gyro orientation
    ALOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:cat %s (%lld)", mpu.gyro_orient, getTimestamp());
    fptr = fopen(mpu.gyro_orient, "r");
    if (fptr != NULL) {
        int om[9];
        fscanf(fptr, "%d,%d,%d,%d,%d,%d,%d,%d,%d",
               &om[0], &om[1], &om[2], &om[3], &om[4], &om[5],
               &om[6], &om[7], &om[8]);
        fclose(fptr);

        ALOGV_IF(EXTRA_VERBOSE,
                "HAL:gyro mounting matrix: "
                "%+d %+d %+d %+d %+d %+d %+d %+d %+d",
                om[0], om[1], om[2], om[3], om[4], om[5], om[6], om[7], om[8]);

        mGyroOrientation[0] = om[0];
        mGyroOrientation[1] = om[1];
        mGyroOrientation[2] = om[2];
        mGyroOrientation[3] = om[3];
        mGyroOrientation[4] = om[4];
        mGyroOrientation[5] = om[5];
        mGyroOrientation[6] = om[6];
        mGyroOrientation[7] = om[7];
        mGyroOrientation[8] = om[8];
    } else {
        ALOGE("HAL:Couldn't read gyro mounting matrix");
    }

    // get accel orientation
    ALOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:cat %s (%lld)", mpu.accel_orient, getTimestamp());
    fptr = fopen(mpu.accel_orient, "r");
    if (fptr != NULL) {
        int om[9];
        fscanf(fptr, "%d,%d,%d,%d,%d,%d,%d,%d,%d",
               &om[0], &om[1], &om[2], &om[3], &om[4], &om[5],
               &om[6], &om[7], &om[8]);
        fclose(fptr);

        ALOGV_IF(EXTRA_VERBOSE,
                "HAL:accel mounting matrix: "
                "%+d %+d %+d %+d %+d %+d %+d %+d %+d",
                om[0], om[1], om[2], om[3], om[4], om[5], om[6], om[7], om[8]);

        mAccelOrientation[0] = om[0];
        mAccelOrientation[1] = om[1];
        mAccelOrientation[2] = om[2];
        mAccelOrientation[3] = om[3];
        mAccelOrientation[4] = om[4];
        mAccelOrientation[5] = om[5];
        mAccelOrientation[6] = om[6];
        mAccelOrientation[7] = om[7];
        mAccelOrientation[8] = om[8];
    } else {
        ALOGE("HAL:Couldn't read accel mounting matrix");
    }
}

MPLSensor::~MPLSensor()
{
    VFUNC_LOG;

#if 0 // mCompassSensor removed
    delete mCompassSensor;
#endif

    /* Close open fds */
    if (mpu_int_fd > 0)
        close(mpu_int_fd);
    if (accel_fd > 0)
        close(accel_fd);
    if (gyro_temperature_fd > 0)
        close(gyro_temperature_fd);
    if (sysfs_names_ptr)
        free(sysfs_names_ptr);

    /* Turn off Gyro master enable          */
    /* A workaround until driver handles it */
    /* TODO: Turn off and close all sensors */
    ALOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:echo 0 > %s (%lld)", mpu.chip_enable, getTimestamp());
    int fd = open(mpu.chip_enable, O_RDWR);
    if (fd < 0) {
        ALOGE("HAL:could not open gyro chip enable");
    } else {
        if (enable_sysfs_sensor(fd, 0) < 0) {
            ALOGE("HAL:could not disable gyro master enable");
        }
    }

#ifdef INV_PLAYBACK_DBG
    inv_turn_off_data_logging();
    fclose(logfile);
#endif
}

#define GY_ENABLED ((1 << Gyro) & enabled_sensors)
#define A_ENABLED  ((1 << Accelerometer) & enabled_sensors)
#ifdef INVENSENSE_COMPASS_CAL
#define M_ENABLED  ((1 << MagneticField) & enabled_sensors)
#else
// TODO: ID_M = 2 even for 3rd-party solution
#define M_ENABLED  ((1 << MagneticField) & enabled_sensors)
#endif
#define O_ENABLED  ((1 << Orientation) & enabled_sensors)
#define LA_ENABLED ((1 << LinearAccel) & enabled_sensors)
#define GR_ENABLED ((1 << Gravity) & enabled_sensors)
#define RV_ENABLED ((1 << RotationVector) & enabled_sensors)

/* TODO: this step is optional, remove?  */
int MPLSensor::setGyroInitialState()
{
    VFUNC_LOG;

    int res = 0;

    ALOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            HW_GYRO_RATE_HZ, mpu.gyro_fifo_rate, getTimestamp());
    int fd = open(mpu.gyro_fifo_rate, O_RDWR);
    res = errno;
    if (fd < 0) {
        ALOGE("HAL:open of %s failed with '%s' (%d)",
             mpu.gyro_fifo_rate, strerror(res), res);
        return res;
    }
    res = write_attribute_sensor(fd, HW_GYRO_RATE_HZ);
    if (res < 0) {
        ALOGE("HAL:write_attribute_sensor : error writing %s with %d",
             mpu.gyro_fifo_rate, HW_GYRO_RATE_HZ);
        return res;
    }

    // Setting LPF is deprecated

    return 0;
}

/* this applies to BMA250 only */
int MPLSensor::setAccelInitialState()
{
    VFUNC_LOG;

    struct input_absinfo absinfo_x;
    struct input_absinfo absinfo_y;
    struct input_absinfo absinfo_z;
    float value;
    if (!ioctl(accel_fd, EVIOCGABS(EVENT_TYPE_ACCEL_X), &absinfo_x) &&
        !ioctl(accel_fd, EVIOCGABS(EVENT_TYPE_ACCEL_Y), &absinfo_y) &&
        !ioctl(accel_fd, EVIOCGABS(EVENT_TYPE_ACCEL_Z), &absinfo_z)) {
        value = absinfo_x.value;
        mPendingEvents[Accelerometer].data[0] = value * CONVERT_A_X;
        value = absinfo_y.value;
        mPendingEvents[Accelerometer].data[1] = value * CONVERT_A_Y;
        value = absinfo_z.value;
        mPendingEvents[Accelerometer].data[2] = value * CONVERT_A_Z;
        //mHasPendingEvent = true;
    }
    return 0;
}

int MPLSensor::onPower(int en)
{
    VFUNC_LOG;

    int res = 0;
    char buf[sizeof(int)+1];
    int count, curr_power_state;

    ALOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.power_state, getTimestamp());
    int tempFd = open(mpu.power_state, O_RDWR);
    res = errno;
    if (tempFd < 0) {
        ALOGE("HAL:Open of %s failed with '%s' (%d)",
             mpu.power_state, strerror(res), res);
    } else {
        // check and set new power state
        count = read_attribute_sensor(tempFd, buf, sizeof(buf));
        if (count < 1) {
            ALOGE("HAL:Error reading power state");
            // will set power_state anyway
            curr_power_state= -1;
        } else {
            sscanf(buf, "%d", &curr_power_state);
        }

        if (en!=curr_power_state) {
            if ((res=enable_sysfs_sensor(tempFd, en)) < 0) {
                ALOGE("HAL:Couldn't write power state");
            }
        } else {
            ALOGV_IF(EXTRA_VERBOSE,
                    "HAL:Power state already enable/disable curr=%d new=%d",
                    curr_power_state, en);
            close(tempFd);
        }
    }
    return res;
}

int MPLSensor::masterEnable(int en)
{
    VFUNC_LOG;

    int res = 0;

    ALOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.chip_enable, getTimestamp());
    int tempFd = open(mpu.chip_enable, O_RDWR);
    res = errno;
    if (tempFd < 0) {
        ALOGE("HAL:open of %s failed with '%s' (%d)",
             mpu.chip_enable, strerror(res), res);
        return res;
    }
    res = enable_sysfs_sensor(tempFd, en);
    return res;
}

int MPLSensor::enableGyro(int en)
{
    VFUNC_LOG;

    int res = 0;

    ALOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.gyro_fifo_enable, getTimestamp());
    int tempFd = open(mpu.gyro_fifo_enable, O_RDWR);
    res = errno;
    if (tempFd < 0) {
        ALOGE("HAL:open of %s failed with '%s' (%d)",
             mpu.gyro_fifo_enable, strerror(res), res);
        return res;
    }
    res = enable_sysfs_sensor(tempFd, en);
    ALOGE_IF(res < 0, "HAL:enable gyro failed");

    if (!en) {
        ALOGV_IF(EXTRA_VERBOSE, "HAL:MPL:inv_gyro_was_turned_off");
        inv_gyro_was_turned_off();

        /* need to also turn off the master enable */
        ALOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                en, mpu.gyro_enable, getTimestamp());
        tempFd = open(mpu.gyro_enable, O_RDWR);
        res = errno;
        if (tempFd > 0) {
            res = enable_sysfs_sensor(tempFd, en);
        } else {
            ALOGE("HAL:open of %s failed with '%s' (%d)",
                 mpu.gyro_enable, strerror(res), res);
        }
    }

    return res;
}

int MPLSensor::enableAccel(int en)
{
    VFUNC_LOG;

    int res;

    ALOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.accel_fifo_enable, getTimestamp());
    int tempFd = open(mpu.accel_fifo_enable, O_RDWR);
    res = errno;
    if (tempFd < 0) {
        ALOGE("HAL:open of %s failed with '%s' (%d)",
             mpu.accel_fifo_enable, strerror(res), res);
        return res;
    }

    res = enable_sysfs_sensor(tempFd, en);
    ALOGE_IF(res < 0, "HAL:enable accel failed");
    if (!en) {
        ALOGV_IF(EXTRA_VERBOSE, "HAL:MPL:inv_accel_was_turned_off");
        inv_accel_was_turned_off();
    }

    if (!en) {
        /* need to also turn off the master enable */
        ALOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                en, mpu.accel_enable, getTimestamp());
        tempFd = open(mpu.accel_enable, O_RDWR);
        res = errno;
        if (tempFd > 0) {
            res = enable_sysfs_sensor(tempFd, en);
        } else {
            ALOGE("HAL:open of %s failed with '%s' (%d)",
                 mpu.accel_enable, strerror(res), res);
        }
    }

    return res;
}

int MPLSensor::enableCompass(int en)
{
    VFUNC_LOG;

    if (mCompassSensor == NULL)
        return 0;

    int res = mCompassSensor->enable(ID_M, en);
    if (en == 0) {
        ALOGV_IF(EXTRA_VERBOSE, "HAL:MPL:inv_compass_was_turned_off");
        inv_compass_was_turned_off();
    }
    return res;
}

void MPLSensor::computeLocalSensorMask(int enabled_sensors)
{
    VFUNC_LOG;

    do {
        if (LA_ENABLED || GR_ENABLED || RV_ENABLED || O_ENABLED) {
            ALOGV_IF(ENG_VERBOSE, "FUSION ENABLED");
            mLocalSensorMask = ALL_MPL_SENSORS_NP;
            break;
        }

        if (!A_ENABLED && !M_ENABLED && !GY_ENABLED) {
            /* Invensense compass cal */
            ALOGV_IF(ENG_VERBOSE, "ALL DISABLED");
            mLocalSensorMask = 0;
            break;
        }

        if (GY_ENABLED) {
            ALOGV_IF(ENG_VERBOSE, "G ENABLED");
            mLocalSensorMask |= INV_THREE_AXIS_GYRO;
        } else {
            ALOGV_IF(ENG_VERBOSE, "G DISABLED");
            mLocalSensorMask &= ~INV_THREE_AXIS_GYRO;
        }

        if (A_ENABLED) {
            ALOGV_IF(ENG_VERBOSE, "A ENABLED");
            mLocalSensorMask |= INV_THREE_AXIS_ACCEL;
        } else {
            ALOGV_IF(ENG_VERBOSE, "A DISABLED");
            mLocalSensorMask &= ~INV_THREE_AXIS_ACCEL;
        }

        /* Invensense compass calibration */
        if (M_ENABLED) {
            ALOGV_IF(ENG_VERBOSE, "M ENABLED");
            mLocalSensorMask |= INV_THREE_AXIS_COMPASS;
        } else {
            ALOGV_IF(ENG_VERBOSE, "M DISABLED");
            mLocalSensorMask &= ~INV_THREE_AXIS_COMPASS;
        }
    } while (0);
}

int MPLSensor::enableSensors(unsigned long sensors, int en)
{
    VFUNC_LOG;

    inv_error_t res = -1;
    int on = 1;
    int off = 0;

    // Sequence to enable or disable a sensor
    // 1. enable Power state
    // 2. reset master enable (=0)
    // 3. enable or disable a sensor
    // 4. set master enable (=1)

    /* ensure power state is on */
    onPower(1);

    /* reset master enable */
    res = masterEnable(0);
    if (res < 0) {
        return res;
    }

    ALOGV("HAL:enableSensors - sensors: 0x%0x", (unsigned int)sensors);

    if (sensors & INV_THREE_AXIS_GYRO) {
        ALOGV("HAL:enableSensors - enable gyro");
        res = enableGyro(on);
        if (res < 0) {
            return res;
        }
    } else if ((sensors & INV_THREE_AXIS_GYRO) == 0) {
        ALOGV("HAL:enableSensors - disable gyro");
        res = enableGyro(off);
        if (res < 0) {
            return res;
        }
    }

    if (sensors & INV_THREE_AXIS_ACCEL) {
        ALOGV("HAL:enableSensors - enable accel");
        res = enableAccel(on);
        if (res < 0) {
            return res;
        }
    } else if ((sensors & INV_THREE_AXIS_ACCEL) == 0) {
        ALOGV("HAL:enableSensors - disable accel");
        res = enableAccel(off);
        if (res < 0) {
            return res;
        }
    }

    /* Invensense compass calibration */
    if (sensors & INV_THREE_AXIS_COMPASS) {
        ALOGV("HAL:enableSensors - enable compass");
        res = enableCompass(on);
        if (res < 0) {
            return res;
        }
    } else if ((sensors & INV_THREE_AXIS_COMPASS) == 0) {
        ALOGV("HAL:enableSensors - disable compass");
        res = enableCompass(off);
    }

    unsigned long mask = (INV_THREE_AXIS_GYRO | INV_THREE_AXIS_ACCEL);
    if (mCompassSensor != NULL)
        mask |= (INV_THREE_AXIS_COMPASS * mCompassSensor->isIntegrated());
    if (sensors & mask) {
        res = masterEnable(1);
        if (res < 0) {
            return res;
        }
    } else { // all sensors idle -> reduce power
        res = onPower(0);
        if (res < 0) {
            return res;
        }
        storeCalibration();
    }

    return res;
}

/* Store calibration file */
void MPLSensor::storeCalibration()
{
    if ((mHaveGoodMpuCal == true) || (mAccelAccuracy >= 2)) {
       int res = inv_store_calibration();
       if (res) {
           ALOGE("HAL:Cannot store calibration on file");
       } else {
           ALOGI("HAL:Cal file updated");
       }
    }
}

void MPLSensor::cbProcData()
{
    mNewData = 1;
    mSampleCount++;
    ALOGV_IF(EXTRA_VERBOSE, "HAL:new data");
}

/*  these handlers transform mpl data into one of the Android sensor types */
void MPLSensor::gyroHandler(sensors_event_t* s)
{
    VHANDLER_LOG;

    inv_get_sensor_type_gyroscope(s->gyro.v, &s->gyro.status, &s->timestamp);
    ALOGV_IF(HANDLER_DATA, "HAL:gy data : %+f %+f %+f - %lld",
            s->gyro.v[0], s->gyro.v[1], s->gyro.v[2], s->timestamp);
}

void MPLSensor::accelHandler(sensors_event_t* s)
{
    VHANDLER_LOG;

    inv_get_sensor_type_accelerometer(
        s->acceleration.v, &s->acceleration.status, &s->timestamp);
    ALOGV_IF(HANDLER_DATA, "HAL:ac data : %+f %+f %+f - %lld",
            s->acceleration.v[0], s->acceleration.v[1], s->acceleration.v[2],
            s->timestamp);
    mAccelAccuracy = s->acceleration.status;
}

void MPLSensor::compassHandler(sensors_event_t* s)
{
    VHANDLER_LOG;

    inv_get_sensor_type_magnetic_field(
        s->magnetic.v, &s->magnetic.status, &s->timestamp);
    ALOGV_IF(HANDLER_DATA, "HAL:mf data: %+f %+f %+f - %lld",
            s->magnetic.v[0], s->magnetic.v[1], s->magnetic.v[2], s->timestamp);
}

void MPLSensor::rvHandler(sensors_event_t* s)
{
    // rotation vector does not have an accuracy or status
    VHANDLER_LOG;

    int8_t status;
    inv_get_sensor_type_rotation_vector(s->data, &status, &s->timestamp);
    ALOGV_IF(HANDLER_DATA, "HAL:rv data: %+f %+f %+f %+f - %+lld",
            s->data[0], s->data[1], s->data[2], s->data[3], s->timestamp);
}

void MPLSensor::laHandler(sensors_event_t* s)
{
    VHANDLER_LOG;

    inv_get_sensor_type_linear_acceleration(
            s->gyro.v, &s->gyro.status, &s->timestamp);
    ALOGV_IF(HANDLER_DATA, "HAL:la data: %+f %+f %+f - %lld",
            s->gyro.v[0], s->gyro.v[1], s->gyro.v[2], s->timestamp);
}

void MPLSensor::gravHandler(sensors_event_t* s)
{
    VHANDLER_LOG;

    inv_get_sensor_type_gravity(s->gyro.v, &s->gyro.status, &s->timestamp);
    ALOGV_IF(HANDLER_DATA, "HAL:gr data: %+f %+f %+f - %lld",
            s->gyro.v[0], s->gyro.v[1], s->gyro.v[2], s->timestamp);
}

void MPLSensor::orienHandler(sensors_event_t* s)
{
    VHANDLER_LOG;

    inv_get_sensor_type_orientation(
            s->orientation.v, &s->orientation.status, &s->timestamp);
    ALOGV_IF(HANDLER_DATA, "HAL:or data: %f %f %f",
            s->orientation.v[0], s->orientation.v[1], s->orientation.v[2]);
}

int MPLSensor::enable(int32_t handle, int en)
{
    VFUNC_LOG;

    android::String8 sname;
    int what = -1;

    switch (handle) {
    case ID_A:
        what = Accelerometer;
        sname = "Accelerometer";
        break;
    case ID_M:
        what = MagneticField;
        sname = "MagneticField";
        break;
    case ID_O:
        what = Orientation;
        sname = "Orientation";
        break;
    case ID_GY:
        what = Gyro;
        sname = "Gyro";
        break;
    case ID_GR:
        what = Gravity;
        sname = "Gravity";
        break;
    case ID_RV:
        what = RotationVector;
        sname = "RotationVector";
        break;
    case ID_LA:
        what = LinearAccel;
        sname = "LinearAccel";
        break;
    default: //this takes care of all the gestures
        what = handle;
        sname = "Others";
        break;
    }

    if (uint32_t(what) >= numSensors)
        return -EINVAL;

    int newState = en ? 1 : 0;
    int err = 0;
    unsigned long sen_mask;

    ALOGV("HAL:enable - sensor %s (handle %d) %s -> %s", sname.string(), handle,
            ((mEnabled & (1 << what)) ? "en" : "dis"),
            ((uint32_t(newState) << what) ? "en" : "dis"));
    ALOGV_IF((uint32_t(newState) << what) != (mEnabled & (1 << what)),
            "HAL:%s sensor state change what=%d", sname.string(), what);

    pthread_mutex_lock(&mMplMutex);
    if ((uint32_t(newState) << what) != (mEnabled & (1 << what))) {

#if CAL_DATA_AUTO_LOAD == 1
        if (newState == 1)
        {
            inv_load_calibration();
            //set to "0" after loading calibration
            self_test_auto_load_state_write(0);
        }
#endif

        uint32_t sensor_type;
        short flags = newState;
        mEnabled &= ~(1 << what);
        mEnabled |= (uint32_t(flags) << what);
        ALOGV("HAL:handle = %d", handle);
        ALOGV("HAL:flags = %d", flags);
        computeLocalSensorMask(mEnabled);
        ALOGV("HAL:enable : mEnabled = %d", mEnabled);
        sen_mask = mLocalSensorMask & mMasterSensorMask;
        mSensorMask = sen_mask;
        ALOGV("HAL:sen_mask= 0x%0lx", sen_mask);
        enableSensors(sen_mask, flags);

        if (!newState) {
            update_delay();
        }
    }
    pthread_mutex_unlock(&mMplMutex);

#ifdef INV_PLAYBACK_DBG
    /* apparently the logging needs to be go through this sequence
       to properly flush the log file */
    inv_turn_off_data_logging();
    fclose(logfile);
    logfile = fopen("/data/playback.bin", "ab");
    if (logfile)
        inv_turn_on_data_logging(logfile);
#endif

    return err;
}

int MPLSensor::setDelay(int32_t handle, int64_t ns)
{
    VFUNC_LOG;

    android::String8 sname;
    int what = -1;

    switch (handle) {
    case ID_A:
        what = Accelerometer;
        sname = "Accelerometer";
        break;
    case ID_M:
        what = MagneticField;
        sname = "MagneticField";
        break;
    case ID_O:
        what = Orientation;
        sname = "Orientation";
        break;
    case ID_GY:
        what = Gyro;
        sname = "Gyro";
        break;
    case ID_GR:
        what = Gravity;
        sname = "Gravity";
        break;
    case ID_RV:
        what = RotationVector;
        sname = "RotationVector";
        break;
    case ID_LA:
        what = LinearAccel;
        sname = "LinearAccel";
        break;
    default: // this takes care of all the gestures
        what = handle;
        sname = "Others";
        break;
    }

    if (uint32_t(what) >= numSensors)
        return -EINVAL;

    if (ns < 0)
        return -EINVAL;

    ALOGV("setDelay : %llu ns, (%.2f Hz)", ns, 1000000000.f / ns);

    /* limit all rates to 100Hz max */
    /* TODO: Capable to run at 200Hz? */
#if FASTEST_200HZ_SUPPORT
    if (ns < 5000000LL) {
        ns = 5000000LL;
    }
#else
    if (ns < 10000000LL) {
        ns = 10000000LL;
    }
#endif

    /* store request rate to mDelays arrary for each sensor */
    mDelays[what] = ns;
    return update_delay();
}

void MPLSensor::setCompassDelay(int64_t ns)
{
    int64_t got;

    if (mCompassSensor != NULL) {
        mCompassSensor->setDelay(ID_M, ns);
        got = mCompassSensor->getDelay(ID_M);
        inv_set_compass_sample_rate(got / 1000);
    }
}

int MPLSensor::update_delay()
{
    VHANDLER_LOG;

    int res = 0;
    int64_t got;

    if (mEnabled) {
        uint64_t wanted = -1LLU;

        // Sequence to change sensor's FIFO rate
        // 1. enable Power state
        // 2. reset master enable
        // 3. Update delay
        // 4. set master enable

        // ensure power on
        onPower(1);

        // reset master enable
        masterEnable(0);

        /* search the minimum delay requested across all enabled sensors */
        for (int i = 0; i < numSensors; i++) {
            if (mEnabled & (1 << i)) {
                uint64_t ns = mDelays[i];
                wanted = wanted < ns ? wanted : ns;
            }
        }

        /* mpl rate in us in future maybe different for
           gyro vs compass vs accel */
        int rateInus = (int)wanted / 1000LL;
        int mplGyroRate = rateInus;
        int mplAccelRate = rateInus;
        int mplCompassRate = rateInus;

        ALOGV("HAL:wanted rate for all sensors : "
             "%llu ns, mpl rate: %d us, (%.2f Hz)",
             wanted, rateInus, (float)NS2HZ(wanted));

        /* set rate in MPL */
        /* compass can only do 100Hz max */
        inv_set_gyro_sample_rate(mplGyroRate);
        inv_set_accel_sample_rate(mplAccelRate);
        inv_set_compass_sample_rate(mplCompassRate);
        /* TODO: Test 200Hz */
        //inv_set_gyro_sample_rate(5000);
        ALOGV("HAL:MPL gyro sample rate: %d", mplGyroRate);
        ALOGV("HAL:MPL accel sample rate: %d", mplAccelRate);
        ALOGV("HAL:MPL compass sample rate: %d", mplCompassRate);

        unsigned long sensors = mLocalSensorMask & mMasterSensorMask;
        int tempFd = -1;

        if(sensors & INV_THREE_AXIS_GYRO) {
            ALOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %.0f > %s (%lld)",
                    (float)NS2HZ(wanted), mpu.gyro_fifo_rate, getTimestamp());
            tempFd = open(mpu.gyro_fifo_rate, O_RDWR);
            res = write_attribute_sensor(tempFd, NS2HZ(wanted));
            ALOGE_IF(res < 0, "HAL:GYRO update delay error");
        }

        if(sensors & INV_THREE_AXIS_ACCEL) {
            ALOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %.0f > %s (%lld)",
                        (float)NS2HZ(wanted), mpu.accel_fifo_rate, getTimestamp());
            tempFd = open(mpu.accel_fifo_rate, O_RDWR);
            res = write_attribute_sensor(tempFd, NS2HZ(wanted));
            ALOGE_IF(res < 0, "HAL:ACCEL update delay error");
        }

        if (sensors & INV_THREE_AXIS_COMPASS) {
            mCompassSensor->setDelay(ID_M, wanted);
            got = mCompassSensor->getDelay(ID_M);
            inv_set_compass_sample_rate(got / 1000);
        }

        unsigned long mask = (INV_THREE_AXIS_GYRO | INV_THREE_AXIS_ACCEL);
        if (mCompassSensor != NULL)
            mask |= (INV_THREE_AXIS_COMPASS * mCompassSensor->isIntegrated());
        if (sensors & mask) {
            res = masterEnable(1);
            if(res < 0) {
                return res;
            }
        } else { // all sensors idle -> reduce power
            res = onPower(0);
            if(res < 0) {
                return res;
            }
        }
    }
    return res;
}

/* use for third party accel */
int MPLSensor::readAccelEvents(sensors_event_t* data, int count)
{
    VHANDLER_LOG;

    if (count < 1)
        return -EINVAL;

    ssize_t n = mAccelInputReader.fill(accel_fd);
    if (n < 0) {
        ALOGE("HAL:missed accel events, exit");
        return n;
    }

    int numEventReceived = 0;
    input_event const* event;
    int nb, done = 0;

    while (done == 0 && count && mAccelInputReader.readEvent(&event)) {
        int type = event->type;
        if (type == EV_ABS) {
            if (event->code == EVENT_TYPE_ACCEL_X) {
                mPendingMask |= 1 << Accelerometer;
                mCachedAccelData[0] = event->value;
            } else if (event->code == EVENT_TYPE_ACCEL_Y) {
                mPendingMask |= 1 << Accelerometer;
                mCachedAccelData[1] = event->value;
            } else if (event->code == EVENT_TYPE_ACCEL_Z) {
                mPendingMask |= 1 << Accelerometer;
                mCachedAccelData[2] =event-> value;
            }
        } else if (type == EV_SYN) {
            int executed;
            done = 1;
            mPendingMask &= (~(1 << Accelerometer));
            if (mLocalSensorMask & INV_THREE_AXIS_ACCEL) {
                inv_build_accel(mCachedAccelData, 0, getTimestamp(), &executed);
                if (executed) {
                    nb = executeOnData(data, count);
                    numEventReceived += nb;
                    count -= nb;
                }
            }
        } else {
            ALOGE("HAL:AccelSensor: unknown event (type=%d, code=%d)",
                    type, event->code);
        }
        mAccelInputReader.next();
    }

    ALOGV_IF(ENG_VERBOSE, "HAL:readAccelEvents - events read=%d", numEventReceived);

    return numEventReceived;
}

/**
 *  Should be called after reading at least one of gyro
 *  compass or accel data. You should only read 1 sample of
 *  data and call this.
 *  @returns 0, if successful, error number if not.
 */
int MPLSensor::executeOnData(sensors_event_t* data, int count)
{
    VFUNC_LOG;

    int numEventReceived = 0;
    long msg;

    msg = inv_get_message_level_0(1);
    if (msg) {
        if (msg & INV_MSG_MOTION_EVENT) {
            ALOGV("HAL:**** Motion ****\n");
        }
        if (msg & INV_MSG_NO_MOTION_EVENT) {
            ALOGV("HAL:***** No Motion *****\n");
            /* after the first no motion, the gyro should be
               calibrated well */
            mGyroAccuracy = SENSOR_STATUS_ACCURACY_HIGH;
            /* if gyros are on and we got a no motion, set a flag
               indicating that the cal file can be written. */
            mHaveGoodMpuCal = true;
        }
    }

    // load up virtual sensors
    for (int i = 0; i < numSensors; i++) {
        if (mEnabled & (1 << i)) {
            CALL_MEMBER_FN(this, mHandlers[i])(mPendingEvents + i);
            if (count > 0) {
                *data++ = mPendingEvents[i];
                count--;
                numEventReceived++;
            }
        }
    }

    return numEventReceived;
}

int MPLSensor::readEvents(sensors_event_t *data, int count)
{
    VHANDLER_LOG;

    int i;
    inv_error_t res;

    if (count < 1)
        return -EINVAL;

    pthread_mutex_lock(&mMplMutex);
    ssize_t n = mGyroInputReader.fill(mpu_int_fd);
    pthread_mutex_unlock(&mMplMutex);
    if (n < 0) {
        return n;
    }

    int numEventReceived = 0;
    input_event const* event;
    int mask = 0;
    int executed;
    int nb;

    while (count && mGyroInputReader.readEvent(&event)) {
        int type = event->type;
        if (type == EV_REL) {
            switch (event->code) {
            case EVENT_TYPE_GYRO_X:
                mCachedGyroData[0] = event->value;
                mask |= 1;
                mPendingMask |= 1 << Gyro;
                break;
            case EVENT_TYPE_GYRO_Y:
                mCachedGyroData[1] = event->value;
                mask |= 1;
                mPendingMask |= 1 << Gyro;
                break;
            case EVENT_TYPE_GYRO_Z:
                mCachedGyroData[2] = event->value;
                mask |= 1;
                mPendingMask |= 1 << Gyro;
                break;
            case EVENT_TYPE_IACCEL_X:
                mCachedAccelData[0] = event->value;
                mask |= 2;
                mPendingMask |= 1 << Accelerometer;
                break;
            case EVENT_TYPE_IACCEL_Y:
                mCachedAccelData[1] = event->value;
                mask |= 2;
                mPendingMask |= 1 << Accelerometer;
                break;
            case EVENT_TYPE_IACCEL_Z:
                mCachedAccelData[2] = event->value;
                mask |= 2;
                mPendingMask |= 1 << Accelerometer;
                break;
            case EVENT_TYPE_TIMESTAMP_HI:
                // clear upper 32-bits
                mSensorTimestamp &= 0xffffffff;
                mSensorTimestamp |= ((uint64_t)event->value) << 32;
                break;
            case EVENT_TYPE_TIMESTAMP_LO:
                // clear lower 32-bits
                mSensorTimestamp &= 0xffffffff00000000ULL;
                mSensorTimestamp |= (uint64_t)((unsigned int)event->value);
                break;
            }

        } else if (type == EV_SYN) {
#if CAL_DATA_AUTO_LOAD == 1
            //polling cal data every 2 sec
            if(mSensorTimestamp - mCalDataCurrentTime >= 2000000000LL) {
                mCalDataCurrentTime = mSensorTimestamp;
                if (self_test_auto_load_state_read()==1) {
                    inv_load_calibration();
                    //set to "0" after loading calibration
                    self_test_auto_load_state_write(0);
                }
            }
#endif
            if (mPendingMask & 1 << Gyro) {
                // send down temperature every 0.5 seconds
                if(mSensorTimestamp - mTempCurrentTime >= 500000000LL) {
                    mTempCurrentTime = mSensorTimestamp;
                    long long temperature[2];
                    if(inv_read_temperature(temperature) == 0) {
                        ALOGV_IF(INPUT_DATA,
                                "HAL:inv_read_temperature = %lld, timestamp= %lld",
                                temperature[0], temperature[1]);
                        inv_build_temp(temperature[0], temperature[1]);
                    }
#ifdef TESTING
                long bias[3], temp, temp_slope[3];
                inv_get_gyro_bias(bias, &temp);
                inv_get_gyro_ts(temp_slope);

                ALOGI("T: %.3f "
                     "GB: %+13f %+13f %+13f "
                     "TS: %+13f %+13f %+13f "
                     "\n",
                     (float)temperature[0] / 65536.f,
                     (float)bias[0] / 65536.f / 16.384f,
                     (float)bias[1] / 65536.f / 16.384f,
                     (float)bias[2] / 65536.f / 16.384f,
                     temp_slope[0] / 65536.f,
                     temp_slope[1] / 65536.f,
                     temp_slope[2] / 65536.f);
#endif
                }

                if (mLocalSensorMask & INV_THREE_AXIS_GYRO) {
                    inv_build_gyro(mCachedGyroData, mSensorTimestamp,
                                   &executed);
                    ALOGV_IF(INPUT_DATA,
                            "HAL:inv_build_gyro:    %+8d %+8d %+8d - %lld, "
                            "exec=%d",
                            mCachedGyroData[0], mCachedGyroData[1],
                            mCachedGyroData[2], mSensorTimestamp, executed);
                    if (executed) {
                        nb = executeOnData(data, count);
                        numEventReceived += nb;
                        count -= nb;
                    }
                }
            }
            if (mPendingMask & 1 << Accelerometer) {
                if (mLocalSensorMask & INV_THREE_AXIS_ACCEL) {
                    inv_build_accel(mCachedAccelData, 0, mSensorTimestamp,
                                    &executed);
                    ALOGV_IF(INPUT_DATA,
                            "HAL:inv_build_accel:   %+8ld %+8ld %+8ld - %lld, "
                            "exec=%d",
                            mCachedAccelData[0], mCachedAccelData[1],
                            mCachedAccelData[2], mSensorTimestamp, executed);
                    if (executed) {
                        nb = executeOnData(data, count);
                        numEventReceived += nb;
                        count -= nb;
                    }
                }
            }
            mPendingMask &= ~((1 << Gyro));
            mPendingMask &= ~((1 << Accelerometer));
        } else {
            ALOGE("HAL:Sensor: unknown event (type=%d, code=%d)",
                 type, event->code);
        }
        mGyroInputReader.next();
    }

    return numEventReceived;
}

/* use for both MPUxxxx and third party compass */
int MPLSensor::readCompassEvents(sensors_event_t *data, int count)
{
    VHANDLER_LOG;

    if (count < 1)
        return -EINVAL;

    int numEventReceived = 0;
    int done = 0;
    int nb;

    int numNewEvents = mCompassSensor->fill();
    if (numNewEvents < 0) {
        return 0;
    }
    // if numNewEvents = 0, there could still be old samples in the buffer
    //  therefore, continue processing.

    // getSample will either return when done=1, indicating a full sample
    //  available to be push in the MPL via inv_build_compass
    //  or when InputEventCircularReader::readEvent has finished the events
    //  available without finishing the sample, in which case we exit the loop.
    while (mCompassSensor->readSample(mCachedCompassData, &mCompassTimestamp)) {
        int executed, status;
        status = 0;
        if (mCompassSensor->providesCalibration()) {
            status = mCompassSensor->getAccuracy();
            status |= INV_CALIBRATED;
        }
        if (mLocalSensorMask & INV_THREE_AXIS_COMPASS) {
            inv_build_compass(mCachedCompassData, status,
                              mCompassTimestamp, &executed);
            ALOGV_IF(INPUT_DATA, "HAL:inv_build_compass: %+8ld %+8ld %+8ld - %lld, exec=%d",
                    mCachedCompassData[0], mCachedCompassData[1],
                    mCachedCompassData[2], mCompassTimestamp, executed);
            if (executed) {
                nb = executeOnData(data, count);
                numEventReceived += nb;
                count -= nb;
            }
        }
    }

    return numEventReceived;
}

int MPLSensor::getFd() const
{
    VFUNC_LOG;
    ALOGV_IF(EXTRA_VERBOSE, "MPLSensor::getFd returning %d", mpu_int_fd);
    return mpu_int_fd;
}

int MPLSensor::getAccelFd() const
{
    VFUNC_LOG;
    ALOGV_IF(EXTRA_VERBOSE, "MPLSensor::getAccelFd returning %d", accel_fd);
    return accel_fd;
}

int MPLSensor::getCompassFd() const
{
    VFUNC_LOG;
    int fd = -1;

    if (mCompassSensor != NULL) {
        fd = mCompassSensor->getFd();
    }
    ALOGV_IF(EXTRA_VERBOSE, "MPLSensor::getCompassFd returning %d", fd);
    return fd;
}

int MPLSensor::getPollTime()
{
    VHANDLER_LOG;
    return mPollTime;
}

bool MPLSensor::hasPendingEvents() const
{
    VHANDLER_LOG;
    // if we are using the polling workaround, force the main
    // loop to check for data every time
    return (mPollTime != -1);
}

/* TODO: support resume suspend when we gain more info about them*/
void MPLSensor::sleepEvent()
{
    VFUNC_LOG;
}

void MPLSensor::wakeEvent()
{
    VFUNC_LOG;
}

int MPLSensor::inv_float_to_q16(float *fdata, long *ldata)
{
    VHANDLER_LOG;

    if (!fdata || !ldata)
        return -1;
    ldata[0] = (long)(fdata[0] * 65536.f);
    ldata[1] = (long)(fdata[1] * 65536.f);
    ldata[2] = (long)(fdata[2] * 65536.f);
    return 0;
}

int MPLSensor::inv_long_to_q16(long *fdata, long *ldata)
{
    VHANDLER_LOG;

    if (!fdata || !ldata)
        return -1;
    ldata[0] = (fdata[1] * 65536.f);
    ldata[1] = (fdata[2] * 65536.f);
    ldata[2] = (fdata[3] * 65536.f);
    return 0;
}

int MPLSensor::inv_float_to_round(float *fdata, long *ldata)
{
    VHANDLER_LOG;

    if (!fdata || !ldata)
            return -1;
    ldata[0] = (long)fdata[0];
    ldata[1] = (long)fdata[1];
    ldata[2] = (long)fdata[2];
    return 0;
}

int MPLSensor::inv_float_to_round2(float *fdata, short *ldata)
{
    VHANDLER_LOG;

    if (!fdata || !ldata)
        return -1;
    ldata[0] = (short)fdata[0];
    ldata[1] = (short)fdata[1];
    ldata[2] = (short)fdata[2];
    return 0;
}

int MPLSensor::inv_read_temperature(long long *data)
{
    VHANDLER_LOG;

    int count = 0;
    char raw_buf[40];
    short raw = 0;
    long long timestamp = 0;

    memset(raw_buf, 0, sizeof(raw_buf));
    count = read_attribute_sensor(gyro_temperature_fd, raw_buf,
                                  sizeof(raw_buf));
    if (count < 1) {
        ALOGE("HAL:error reading gyro temperature");
        return -1;
    }
    count = sscanf(raw_buf, "%hd%lld", &raw, &timestamp);
    if (count < 0) {
        return -1;
    }

    ALOGV_IF(ENG_VERBOSE,
            "HAL:temperature raw = %d, timestamp = %lld, count = %d",
            raw, timestamp, count);

    data[0] = (35 + ((raw - mTempOffset) / mTempScale)) * 65536.f;
    data[1] = timestamp;

    return 0;
}

int MPLSensor::inv_read_dmp_state(int fd)
{
    VFUNC_LOG;

    if (fd < 0)
        return -1;

    int count = 0;
    char raw_buf[10];
    short raw = 0;

    memset(raw_buf, 0, sizeof(raw_buf));
    count = read_attribute_sensor(fd, raw_buf, sizeof(raw_buf));
    if (count < 1) {
        ALOGE("HAL:error reading dmp state");
        close(fd);
        return -1;
    }

    count = sscanf(raw_buf, "%hd", &raw);
    if (count < 0) {
        ALOGE("HAL:dmp state data is invalid");
        close(fd);
        return -1;
    }

    ALOGV_IF(EXTRA_VERBOSE, "HAL:dmp state = %d, count = %d", raw, count);
    close(fd);
    return (int)raw;
}

int MPLSensor::inv_read_sensor_bias(int fd, long *data)
{
    VFUNC_LOG;

    if (fd == -1) {
        return -1;
    }

    char buf[50];
    char x[15], y[15], z[15];

    memset(buf, 0, sizeof(buf));
    int count = read_attribute_sensor(fd, buf, sizeof(buf));
    if (count < 1) {
        ALOGE("HAL:Error reading gyro bias");
        return -1;
    }

    count = sscanf(buf, "%[^','],%[^','],%[^',']", x, y, z);
    if (count) {
        /* scale appropriately for MPL */
        ALOGV_IF(ENG_VERBOSE,
                "HAL:pre-scaled bias: X:Y:Z (%ld, %ld, %ld)",
                atol(x), atol(y), atol(z));

        data[0] = (long)(atol(x) / 10000 * (1L << 16));
        data[1] = (long)(atol(y) / 10000 * (1L << 16));
        data[2] = (long)(atol(z) / 10000 * (1L << 16));

        ALOGV_IF(ENG_VERBOSE,
                "HAL:scaled bias: X:Y:Z (%ld, %ld, %ld)",
                data[0], data[1], data[2]);
    }
    return 0;
}

int MPLSensor::inv_init_sysfs_attributes(void)
{
    VFUNC_LOG;

    unsigned char i = 0;
    char sysfs_path[MAX_SYSFS_NAME_LEN], tbuf[2];
    char *sptr;
    char **dptr;
    int num;

    sysfs_names_ptr =
            (char*)malloc(sizeof(char[MAX_SYSFS_ATTRB][MAX_SYSFS_NAME_LEN]));
    sptr = sysfs_names_ptr;
    if (sptr != NULL) {
        dptr = (char**)&mpu;
        do {
            *dptr++ = sptr;
            sptr += sizeof(char[MAX_SYSFS_NAME_LEN]);
        } while (++i < MAX_SYSFS_ATTRB);
    } else {
        ALOGE("HAL:couldn't alloc mem for sysfs paths");
        return -1;
    }

    // get sysfs absolute path
    inv_get_sysfs_abs_path(sysfs_path);

    // build MPU's sysfs paths
    sprintf(mpu.chip_enable, "%s%s", sysfs_path, "/enable");
    sprintf(mpu.power_state, "%s%s", sysfs_path, "/power_state");
    sprintf(mpu.dmp_firmware, "%s%s", sysfs_path,"/dmp_firmware");
    sprintf(mpu.firmware_loaded,"%s%s", sysfs_path, "/firmware_loaded");
    sprintf(mpu.tap_on, "%s%s", sysfs_path, "/tap_on");
    sprintf(mpu.key, "%s%s", sysfs_path, "/key");
    sprintf(mpu.self_test, "%s%s", sysfs_path, "/self_test");
    sprintf(mpu.temp_scale, "%s%s", sysfs_path, "/temp_scale");
    sprintf(mpu.temp_offset, "%s%s", sysfs_path, "/temp_offset");
    sprintf(mpu.temperature, "%s%s", sysfs_path, "/temperature");
    sprintf(mpu.gyro_fifo_enable, "%s%s", sysfs_path, "/gyro_fifo_enable");
    sprintf(mpu.gyro_enable, "%s%s", sysfs_path, "/gyro_enable");
    sprintf(mpu.gyro_fifo_rate, "%s%s", sysfs_path, "/fifo_rate");
    sprintf(mpu.gyro_orient, "%s%s", sysfs_path, "/gyro_matrix");

    sprintf(mpu.accel_fifo_enable, "%s%s", sysfs_path, "/accl_fifo_enable");
    sprintf(mpu.accel_enable, "%s%s", sysfs_path, "/accl_enable");
    sprintf(mpu.accel_fifo_rate, "%s%s", sysfs_path, "/fifo_rate");
    sprintf(mpu.accel_fsr, "%s%s", sysfs_path, "/accl_fs");
    sprintf(mpu.accel_bias, "%s%s", sysfs_path, "/accl_bias");
    sprintf(mpu.accel_orient, "%s%s", sysfs_path, "/accl_matrix");

#if 0
    // test print sysfs paths
    dptr = (char**)&mpu;
    for (i = 0; i < MAX_SYSFS_ATTRB; i++) {
        ALOGE("HAL:sysfs path: %s", *dptr++);
    }
#endif
    return 0;
}

