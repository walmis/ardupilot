/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_MPU6050_H__
#define __AP_INERTIAL_SENSOR_MPU6050_H__

#include <stdint.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Progmem/AP_Progmem.h>
#include "AP_InertialSensor.h"

#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>

// enable debug to see a register dump on startup
#define MPU6050_DEBUG 0

class AP_InertialSensor_MPU6050 : public AP_InertialSensor_Backend
{
public:
	AP_InertialSensor_MPU6050(AP_InertialSensor &imu);

    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

    bool gyro_sample_available(void);
    bool accel_sample_available(void);

    /* Concrete implementation of AP_InertialSensor functions: */
    bool                update();

    float get_delta_time() const
    {
        return 0.001f;
    }

protected:
    uint16_t                    _init_sensor();

    uint8_t _gyro_instance;
    uint8_t _accel_instance;
private:
    int16_t reset_fifo(uint8_t sensors);
    void logWriteImu(Vector3f &acc, Vector3f &gyro);

    void                 _poll_data(void);
    uint8_t              _register_read( uint8_t reg );
    void                 _register_write( uint8_t reg, uint8_t val );
    void                 _register_write_check(uint8_t reg, uint8_t val);
    bool                 _hardware_init();

    AP_HAL::I2CDriver *_i2c;
    AP_HAL::Semaphore *_i2c_sem;

    volatile uint16_t _fifo_count;
    uint8_t _data[14];

    void _onSampleData();

    /*
     *  RM-MPU-6000A-00.pdf, page 33, section 4.25 lists LSB sensitivity of
     *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
     */
    static constexpr float          _gyro_scale = (0.0174532f / 16.4f);
    static constexpr float          _acc_scale = (GRAVITY_MSS / 4096.0f);

    // ensure we can't initialise twice
    bool                        _initialised;
    int16_t              _mpu6000_product_id;

    // support for updating filter at runtime
    int8_t _last_accel_filter_hz;
    int8_t _last_gyro_filter_hz;

    volatile bool _sem_missed;

    void _set_filter_register(uint8_t filter_hz);

    uint16_t _error_count;

    LowPassFilter2pVector3f _accel_filter;
    LowPassFilter2pVector3f _gyro_filter;

    Vector3f _accel_filtered;
    Vector3f _gyro_filtered;

    Vector3f _last_gyro;
    Vector3f _last_accel;
    Vector3f _last_delAng;

    Vector3f _delta_angle_accumulator;
    Vector3f _delta_velocity_accumulator;
    float _delta_velocity_dt = 0.0f;

    uint64_t _last_sample_timestamp = 0;
    uint64_t _last_sample_update_timestamp = 0;


    //i2c address
    const uint8_t _addr;
public:

#if MPU6050_DEBUG
    void						_dump_registers(void);
#endif
};

#endif // __AP_INERTIAL_SENSOR_MPU6000_H__
