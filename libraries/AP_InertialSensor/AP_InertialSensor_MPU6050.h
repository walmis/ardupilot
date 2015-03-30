/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_MPU6050_H__
#define __AP_INERTIAL_SENSOR_MPU6050_H__

#include <stdint.h>
#include <AP_HAL.h>
#include <AP_Math.h>
#include <AP_Progmem.h>
#include "AP_InertialSensor.h"

#include <Filter.h>
#include <LowPassFilter2p.h>

// enable debug to see a register dump on startup
#define MPU6050_DEBUG 0

class AP_InertialSensor_MPU6050 : public AP_InertialSensor_Backend
{
public:
	AP_InertialSensor_MPU6050(AP_InertialSensor &imu);

    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

    bool gyro_sample_available(void) { return _num_samples > 0; }
    bool accel_sample_available(void) { return _num_samples > 0; }

    /* Concrete implementation of AP_InertialSensor functions: */
    bool                update();

protected:
    uint16_t                    _init_sensor();

    uint8_t _gyro_instance;
    uint8_t _accel_instance;
private:
    int16_t reset_fifo(uint8_t sensors);
    bool configure_fifo(uint8_t sensors);

    void                 _poll_data(void);
    uint8_t              _register_read( uint8_t reg );
    void                 _register_write( uint8_t reg, uint8_t val );
    void                 _register_write_check(uint8_t reg, uint8_t val);
    bool                 _hardware_init();

    AP_HAL::I2CDriver *_i2c;
    AP_HAL::Semaphore *_i2c_sem;

    volatile uint16_t					_num_samples;

    volatile uint16_t _fifo_count;
    uint8_t _data[12];

    void _onFifoData(); //i2c callback
    void _onSampleData(); //i2c callback

    /*
     *  RM-MPU-6000A-00.pdf, page 33, section 4.25 lists LSB sensitivity of
     *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
     */
    const float          _gyro_scale = (0.0174532f / 16.4f);

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

    volatile uint8_t _fifo_reset_flag;

    //i2c address
    const uint8_t _addr;
public:

#if MPU6050_DEBUG
    void						_dump_registers(void);
#endif
};

#endif // __AP_INERTIAL_SENSOR_MPU6000_H__
