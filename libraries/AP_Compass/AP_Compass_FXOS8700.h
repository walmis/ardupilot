/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_HAL.h>
#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"

#include "Compass.h"
#include "AP_Compass_Backend.h"

class AP_Compass_FXOS8700 : public AP_Compass_Backend
{
private:
    bool                _initialised;
    bool        		read_raw(void);
    uint8_t             _base_config;
    bool                read_register(uint8_t address, uint8_t *value);
    bool                write_register(uint8_t address, uint8_t value);
    uint32_t            _retry_time; // when unhealthy the millis() value to retry at
    AP_HAL::Semaphore*  _i2c_sem;

    int16_t			    _mag_x;
    int16_t			    _mag_y;
    int16_t			    _mag_z;
    int16_t             _mag_x_accum;
    int16_t             _mag_y_accum;
    int16_t             _mag_z_accum;
    uint8_t			    _accum_count;
    uint32_t            _last_accum_time;

    uint8_t				_sample_buffer[6];

    uint8_t             _compass_instance;
public:
    AP_Compass_FXOS8700(Compass &compass) :
    	AP_Compass_Backend(compass),
		_mag_x(0),
		_mag_y(0),
		_mag_z(0),
		_mag_x_accum(0),
		_mag_y_accum(0),
		_mag_z_accum(0),
		_accum_count(0)
    {}

    bool        init(void);
    void        read(void);
    void        accumulate(void);

    static AP_Compass_Backend *detect(Compass &compass);

};

