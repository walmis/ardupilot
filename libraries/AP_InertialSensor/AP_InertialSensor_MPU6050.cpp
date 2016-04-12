/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_MPU6050.h"
#include <DataFlash/DataFlash.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// MPU6000 accelerometer scaling
#define MPU6000_ACCEL_SCALE_1G    (GRAVITY_MSS / 4096.0f)

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
#define MPU6000_DRDY_PIN 70
#elif CONFIG_HAL_BOARD == HAL_BOARD_XPCC
#define P2_6 ((2<<5)|6)
#define MPU6000_DRDY_PIN P2_6
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
#include "../AP_HAL_Linux/GPIO.h"
#define MPU6000_DRDY_PIN BBB_P8_14
#endif
#endif

#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)

// MPU 6000 registers
#define MPUREG_XG_OFFS_TC                               0x00
#define MPUREG_YG_OFFS_TC                               0x01
#define MPUREG_ZG_OFFS_TC                               0x02
#define MPUREG_X_FINE_GAIN                              0x03
#define MPUREG_Y_FINE_GAIN                              0x04
#define MPUREG_Z_FINE_GAIN                              0x05
#define MPUREG_XA_OFFS_H                                0x06    // X axis accelerometer offset (high byte)
#define MPUREG_XA_OFFS_L                                0x07    // X axis accelerometer offset (low byte)
#define MPUREG_YA_OFFS_H                                0x08    // Y axis accelerometer offset (high byte)
#define MPUREG_YA_OFFS_L                                0x09    // Y axis accelerometer offset (low byte)
#define MPUREG_ZA_OFFS_H                                0x0A    // Z axis accelerometer offset (high byte)
#define MPUREG_ZA_OFFS_L                                0x0B    // Z axis accelerometer offset (low byte)
#define MPUREG_PRODUCT_ID                               0x0C    // Product ID Register
#define MPUREG_XG_OFFS_USRH                     0x13    // X axis gyro offset (high byte)
#define MPUREG_XG_OFFS_USRL                     0x14    // X axis gyro offset (low byte)
#define MPUREG_YG_OFFS_USRH                     0x15    // Y axis gyro offset (high byte)
#define MPUREG_YG_OFFS_USRL                     0x16    // Y axis gyro offset (low byte)
#define MPUREG_ZG_OFFS_USRH                     0x17    // Z axis gyro offset (high byte)
#define MPUREG_ZG_OFFS_USRL                     0x18    // Z axis gyro offset (low byte)
#define MPUREG_SMPLRT_DIV                               0x19    // sample rate.  Fsample= 1Khz/(<this value>+1) = 200Hz
#       define MPUREG_SMPLRT_1000HZ                             0x00
#       define MPUREG_SMPLRT_500HZ                              0x01
#       define MPUREG_SMPLRT_250HZ                              0x03
#       define MPUREG_SMPLRT_200HZ                              0x04
#       define MPUREG_SMPLRT_100HZ                              0x09
#       define MPUREG_SMPLRT_50HZ                               0x13
#define MPUREG_CONFIG                                           0x1A
#define MPUREG_GYRO_CONFIG                                      0x1B
// bit definitions for MPUREG_GYRO_CONFIG
#       define BITS_GYRO_FS_250DPS                              0x00
#       define BITS_GYRO_FS_500DPS                              0x08
#       define BITS_GYRO_FS_1000DPS                             0x10
#       define BITS_GYRO_FS_2000DPS                             0x18
#       define BITS_GYRO_FS_MASK                                0x18    // only bits 3 and 4 are used for gyro full scale so use this to mask off other bits
#       define BITS_GYRO_ZGYRO_SELFTEST                 0x20
#       define BITS_GYRO_YGYRO_SELFTEST                 0x40
#       define BITS_GYRO_XGYRO_SELFTEST                 0x80
#define MPUREG_ACCEL_CONFIG                             0x1C
#define MPUREG_MOT_THR                                  0x1F    // detection threshold for Motion interrupt generation.  Motion is detected when the absolute value of any of the accelerometer measurements exceeds this
#define MPUREG_MOT_DUR                                  0x20    // duration counter threshold for Motion interrupt generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit of 1 LSB = 1 ms
#define MPUREG_ZRMOT_THR                                0x21    // detection threshold for Zero Motion interrupt generation.
#define MPUREG_ZRMOT_DUR                                0x22    // duration counter threshold for Zero Motion interrupt generation. The duration counter ticks at 16 Hz, therefore ZRMOT_DUR has a unit of 1 LSB = 64 ms.
#define MPUREG_FIFO_EN                                  0x23
#define MPUREG_INT_PIN_CFG                              0x37
#       define BIT_INT_RD_CLEAR                                 0x10    // clear the interrupt when any read occurs
#       define BIT_LATCH_INT_EN                                 0x20    // latch data ready pin 
#       define BIT_I2C_BYPASS_EN                                 0x02    // latch data ready pin
#define MPUREG_INT_ENABLE                               0x38
// bit definitions for MPUREG_INT_ENABLE
#       define BIT_RAW_RDY_EN                                   0x01
#       define BIT_DMP_INT_EN                                   0x02    // enabling this bit (DMP_INT_EN) also enables RAW_RDY_EN it seems
#       define BIT_UNKNOWN_INT_EN                               0x04
#       define BIT_I2C_MST_INT_EN                               0x08
#       define BIT_FIFO_OFLOW_EN                                0x10
#       define BIT_ZMOT_EN                                              0x20
#       define BIT_MOT_EN                                               0x40
#       define BIT_FF_EN                                                0x80
#define MPUREG_INT_STATUS                               0x3A
// bit definitions for MPUREG_INT_STATUS (same bit pattern as above because this register shows what interrupt actually fired)
#       define BIT_RAW_RDY_INT                                  0x01
#       define BIT_DMP_INT                                              0x02
#       define BIT_UNKNOWN_INT                                  0x04
#       define BIT_I2C_MST_INT                                  0x08
#       define BIT_FIFO_OFLOW_INT                               0x10
#       define BIT_ZMOT_INT                                             0x20
#       define BIT_MOT_INT                                              0x40
#       define BIT_FF_INT                                               0x80
#define MPUREG_ACCEL_XOUT_H                             0x3B
#define MPUREG_ACCEL_XOUT_L                             0x3C
#define MPUREG_ACCEL_YOUT_H                             0x3D
#define MPUREG_ACCEL_YOUT_L                             0x3E
#define MPUREG_ACCEL_ZOUT_H                             0x3F
#define MPUREG_ACCEL_ZOUT_L                             0x40
#define MPUREG_TEMP_OUT_H                               0x41
#define MPUREG_TEMP_OUT_L                               0x42
#define MPUREG_GYRO_XOUT_H                              0x43
#define MPUREG_GYRO_XOUT_L                              0x44
#define MPUREG_GYRO_YOUT_H                              0x45
#define MPUREG_GYRO_YOUT_L                              0x46
#define MPUREG_GYRO_ZOUT_H                              0x47
#define MPUREG_GYRO_ZOUT_L                              0x48
#define MPUREG_USER_CTRL                                0x6A
// bit definitions for MPUREG_USER_CTRL
#       define BIT_USER_CTRL_SIG_COND_RESET             0x01            // resets signal paths and results registers for all sensors (gyros, accel, temp)
#       define BIT_USER_CTRL_I2C_MST_RESET              0x02            // reset I2C Master (only applicable if I2C_MST_EN bit is set)
#       define BIT_USER_CTRL_FIFO_RESET                 0x04            // Reset (i.e. clear) FIFO buffer
#       define BIT_USER_CTRL_DMP_RESET                  0x08            // Reset DMP
#       define BIT_USER_CTRL_I2C_IF_DIS                 0x10            // Disable primary I2C interface and enable hal.spi->interface
#       define BIT_USER_CTRL_I2C_MST_EN                 0x20            // Enable MPU to act as the I2C Master to external slave sensors
#       define BIT_USER_CTRL_FIFO_EN                    0x40            // Enable FIFO operations
#       define BIT_USER_CTRL_DMP_EN                             0x80            // Enable DMP operations
#define MPUREG_PWR_MGMT_1                               0x6B
#       define BIT_PWR_MGMT_1_CLK_INTERNAL              0x00            // clock set to internal 8Mhz oscillator
#       define BIT_PWR_MGMT_1_CLK_XGYRO                 0x01            // PLL with X axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_YGYRO                 0x02            // PLL with Y axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_ZGYRO                 0x03            // PLL with Z axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_EXT32KHZ              0x04            // PLL with external 32.768kHz reference
#       define BIT_PWR_MGMT_1_CLK_EXT19MHZ              0x05            // PLL with external 19.2MHz reference
#       define BIT_PWR_MGMT_1_CLK_STOP                  0x07            // Stops the clock and keeps the timing generator in reset
#       define BIT_PWR_MGMT_1_TEMP_DIS                  0x08            // disable temperature sensor
#       define BIT_PWR_MGMT_1_CYCLE                             0x20            // put sensor into cycle mode.  cycles between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL
#       define BIT_PWR_MGMT_1_SLEEP                             0x40            // put sensor into low power sleep mode
#       define BIT_PWR_MGMT_1_DEVICE_RESET              0x80            // reset entire device
#define MPUREG_PWR_MGMT_2                               0x6C            // allows the user to configure the frequency of wake-ups in Accelerometer Only Low Power Mode
#define MPUREG_BANK_SEL                                 0x6D            // DMP bank selection register (used to indirectly access DMP registers)
#define MPUREG_MEM_START_ADDR                   0x6E            // DMP memory start address (used to indirectly write to dmp memory)
#define MPUREG_MEM_R_W                                  0x6F            // DMP related register
#define MPUREG_DMP_CFG_1                                0x70            // DMP related register
#define MPUREG_DMP_CFG_2                                0x71            // DMP related register
#define MPUREG_FIFO_COUNTH                              0x72
#define MPUREG_FIFO_COUNTL                              0x73
#define MPUREG_FIFO_R_W                                 0x74
#define MPUREG_WHOAMI                                   0x75


// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BITS_DLPF_CFG_256HZ_NOLPF2              0x00
#define BITS_DLPF_CFG_188HZ                             0x01
#define BITS_DLPF_CFG_98HZ                              0x02
#define BITS_DLPF_CFG_42HZ                              0x03
#define BITS_DLPF_CFG_20HZ                              0x04
#define BITS_DLPF_CFG_10HZ                              0x05
#define BITS_DLPF_CFG_5HZ                               0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF              0x07
#define BITS_DLPF_CFG_MASK                              0x07

// Product ID Description for MPU6000
// high 4 bits  low 4 bits
// Product Name	Product Revision
#define MPU6000ES_REV_C4                        0x14    // 0001			0100
#define MPU6000ES_REV_C5                        0x15    // 0001			0101
#define MPU6000ES_REV_D6                        0x16    // 0001			0110
#define MPU6000ES_REV_D7                        0x17    // 0001			0111
#define MPU6000ES_REV_D8                        0x18    // 0001			1000
#define MPU6000_REV_C4                          0x54    // 0101			0100
#define MPU6000_REV_C5                          0x55    // 0101			0101
#define MPU6000_REV_D6                          0x56    // 0101			0110
#define MPU6000_REV_D7                          0x57    // 0101			0111
#define MPU6000_REV_D8                          0x58    // 0101			1000
#define MPU6000_REV_D9                          0x59    // 0101			1001

#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_FIFO_RST        (0x04)
#define BIT_FIFO_EN         (0x40)

#define SAMPLE_RATE 1000

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))

/*
 *  RM-MPU-6000A-00.pdf, page 31, section 4.23 lists LSB sensitivity of
 *  accel as 4096 LSB/mg at scale factor of +/- 8g (AFS_SEL==2)
 *
 *  See note below about accel scaling of engineering sample MPU6k
 *  variants however
 */

AP_InertialSensor_MPU6050::AP_InertialSensor_MPU6050(AP_InertialSensor &imu) :
	AP_InertialSensor_Backend(imu),
    _i2c(NULL),
    _i2c_sem(NULL),
    _initialised(false),
    _mpu6000_product_id(AP_PRODUCT_ID_NONE),
    _error_count(0),
    _addr(0x68),
	_fifo_count(0),
    _accel_filter(SAMPLE_RATE, 15),
    _gyro_filter(SAMPLE_RATE, 15)
{
}

uint16_t AP_InertialSensor_MPU6050::_init_sensor( )
{
	if (_initialised) return _mpu6000_product_id;
    _initialised = true;

    _i2c = hal.i2c;
    _i2c_sem = _i2c->get_semaphore();

    hal.scheduler->suspend_timer_procs();

    uint8_t tries = 0;

	bool success = _hardware_init();
	if (success) {
		hal.scheduler->delay(5+2);
		if (!_i2c_sem->take(100)) {
			hal.scheduler->panic("MPU6000: Unable to get semaphore");
		}
		_i2c_sem->give();
	}
	if (tries++ > 5) {
		hal.scheduler->panic("PANIC: failed to boot MPU6000 5 times");
	}

	_gyro_instance = _imu.register_gyro();
    _accel_instance = _imu.register_accel();

    hal.scheduler->resume_timer_procs();

    _last_sample_timestamp = hal.scheduler->micros64();
    _last_sample_update_timestamp = hal.scheduler->micros64();
    // start the timer process to read samples
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_MPU6050::_poll_data, void));

#if MPU6000_DEBUG
    _dump_registers();
#endif

    return _mpu6000_product_id;
}


bool AP_InertialSensor_MPU6050::update( void )
{
	//_poll_data();

    //Vector3f accel, gyro;

    // disable timer procs for mininum time
    hal.scheduler->suspend_timer_procs();
   //accel = _accel_filtered;
    //gyro = _gyro_filtered;

    _publish_delta_velocity(_accel_instance, _delta_velocity_accumulator, _delta_velocity_dt);
    _publish_delta_angle(_gyro_instance, _delta_angle_accumulator);

    _delta_angle_accumulator.zero();
    _delta_velocity_accumulator.zero();
    _delta_velocity_dt = 0.0f;

    _publish_gyro(_gyro_instance, _gyro_filtered);
    _publish_accel(_accel_instance, _accel_filtered);

    _last_sample_update_timestamp = _last_sample_timestamp;

    hal.scheduler->resume_timer_procs();

    if (_last_accel_filter_hz != _accel_filter_cutoff()) {
        _accel_filter.set_cutoff_frequency(SAMPLE_RATE, _accel_filter_cutoff());
        _last_accel_filter_hz = _accel_filter_cutoff();
    }

    if (_last_gyro_filter_hz != _gyro_filter_cutoff()) {
        _gyro_filter.set_cutoff_frequency(SAMPLE_RATE, _gyro_filter_cutoff());
        _last_gyro_filter_hz = _gyro_filter_cutoff();
    }


    return true;
}

void AP_InertialSensor_MPU6050::logWriteImu(Vector3f &accel, Vector3f &gyro) {
	DataFlash_Class *dataflash = get_dataflash();

	if (dataflash) {
        struct log_ACCEL pkt = {
            LOG_PACKET_HEADER_INIT((uint8_t)(LOG_ACC1_MSG)),
            time_us   : hal.scheduler->micros64(),
            sample_us : _last_sample_timestamp,
            AccX      : accel.x,
            AccY      : accel.y,
            AccZ      : accel.z
        };
        dataflash->WriteBlock(&pkt, sizeof(pkt));
	}
	if (dataflash) {
		struct log_GYRO pkt = {
			LOG_PACKET_HEADER_INIT((uint8_t)(LOG_GYR1_MSG)),
			time_us   : hal.scheduler->micros64(),
			sample_us : _last_sample_timestamp,
			GyrX      : gyro.x,
			GyrY      : gyro.y,
			GyrZ      : gyro.z
		};
		dataflash->WriteBlock(&pkt, sizeof(pkt));
	}
}
/*================ HARDWARE FUNCTIONS ==================== */

void AP_InertialSensor_MPU6050::_onSampleData() {
	float dt = (hal.scheduler->micros64() - _last_sample_timestamp) / 1000000.0f;
	//float dt = 0.001;

	_last_sample_timestamp = hal.scheduler->micros64();

	/// ---- Accel ---- ///

	Vector3f accel = Vector3f(int16_val(_data, 1), int16_val(_data, 0), -int16_val(_data, 2));
	accel *= _acc_scale;

	_rotate_and_correct_accel(_accel_instance, accel);

	_imu.calc_vibration_and_clipping(_accel_instance, accel, dt);
	_accel_filtered = _accel_filter.apply(accel);

    // integrate delta velocity accumulator
    _delta_velocity_accumulator += Vector3f(accel.x, accel.y, accel.z) * dt;  // compute delta velocity
    _delta_velocity_dt += dt;

	/// ---- Gyro ---- ///
    Vector3f gyro;
    gyro = Vector3f(int16_val(_data, 5), int16_val(_data, 4), -int16_val(_data, 6));
    gyro *= _gyro_scale;
	_rotate_and_correct_gyro(_gyro_instance, gyro);
	_gyro_filtered = _gyro_filter.apply(gyro);

	logWriteImu(accel, gyro);

    // compute delta angle
    Vector3f delAng = (gyro+_last_gyro) * 0.5f * dt;

    /* compute coning correction
     * see page 26 of:
     * Tian et al (2010) Three-loop Integration of GPS and Strapdown INS with Coning and Sculling Compensation
     * Available: http://www.sage.unsw.edu.au/snap/publications/tian_etal2010b.pdf
     * see also examples/coning.py
     */
    Vector3f delConing = ((_delta_angle_accumulator+_last_delAng*(1.0f/6.0f)) % delAng) * 0.5f;

    // integrate delta angle accumulator
    // the angles and coning corrections are accumulated separately in the
    // referenced paper, but in simulation little difference was found between
    // integrating together and integrating separately (see examples/coning.py)
    _delta_angle_accumulator += delAng + delConing;


    _last_gyro = gyro;
    _last_delAng = delAng;

}

/**
 * Timer process to poll for new data from the MPU6000.
 */
void AP_InertialSensor_MPU6050::_poll_data(void)
{
	//dbgset(1);
#ifdef MPU6000_DRDY_PIN
	if(!_sem_missed && hal.gpio->read(MPU6000_DRDY_PIN) == 0) {
		return;
	}
#endif

//	if(!_i2c->readNonblocking(_addr, MPUREG_ACCEL_XOUT_H, 14, _data,
//			FUNCTOR_BIND_MEMBER(&AP_InertialSensor_MPU6050::_onSampleData, void))) {
//	}

	if(!_i2c_sem->take(1)) {
		return;
	}
	//dbgset();
	if(_i2c->readRegisters(_addr, MPUREG_ACCEL_XOUT_H, 14, _data) == 0) {
		_onSampleData();
	}

	_i2c_sem->give();
	//dbgclr();
}

uint8_t AP_InertialSensor_MPU6050::_register_read( uint8_t reg )
{
	uint8_t data = 0;
	_i2c->readRegister(_addr, reg, &data);
    return data;
}

void AP_InertialSensor_MPU6050::_register_write(uint8_t reg, uint8_t val)
{
    _i2c->writeRegister(_addr, reg, val);
}

int16_t AP_InertialSensor_MPU6050::reset_fifo(uint8_t sensors)
{
	_i2c->writeRegister(_addr, MPUREG_USER_CTRL, 0);
    _i2c->writeRegister(_addr, MPUREG_USER_CTRL, BIT_FIFO_RST);
    _i2c->writeRegister(_addr, MPUREG_USER_CTRL, BIT_FIFO_EN);

	_fifo_count = 0;

    return 0;
}

/*
  useful when debugging SPI bus errors
 */
void AP_InertialSensor_MPU6050::_register_write_check(uint8_t reg, uint8_t val)
{
    uint8_t readed;
    _register_write(reg, val);
    readed = _register_read(reg);
    if (readed != val){
	hal.console->printf_P(PSTR("Values doesn't match; written: %02x; read: %02x "), val, readed);
    }
#if MPU6000_DEBUG
    hal.console->printf_P(PSTR("Values written: %02x; readed: %02x "), val, readed);
#endif
}

/*
  set the DLPF filter frequency. Assumes caller has taken semaphore
 */
void AP_InertialSensor_MPU6050::_set_filter_register(uint8_t filter_hz)
{
    uint8_t filter;
    // choose filtering frequency
    if (filter_hz == 0) {
        filter = BITS_DLPF_CFG_256HZ_NOLPF2;
    } else if (filter_hz <= 5) {
        filter = BITS_DLPF_CFG_5HZ;
    } else if (filter_hz <= 10) {
        filter = BITS_DLPF_CFG_10HZ;
    } else if (filter_hz <= 20) {
        filter = BITS_DLPF_CFG_20HZ;
    } else if (filter_hz <= 42) {
        filter = BITS_DLPF_CFG_42HZ;
    } else if (filter_hz <= 98) {
        filter = BITS_DLPF_CFG_98HZ;
    } else {
        filter = BITS_DLPF_CFG_256HZ_NOLPF2;
    }

    _register_write(MPUREG_CONFIG, filter);
}


bool AP_InertialSensor_MPU6050::_hardware_init()
{
    if (!_i2c_sem->take(100)) {
        hal.scheduler->panic(PSTR("MPU6000: Unable to get semaphore"));
    }

    // Chip reset
    uint8_t tries;
    for (tries = 0; tries<5; tries++) {
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
        hal.scheduler->delay(100);

        // Wake up device and select GyroZ clock. Note that the
        // MPU6000 starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);
        hal.scheduler->delay(5);

        // check it has woken up
        if (_register_read(MPUREG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_ZGYRO) {
            break;
        }
#if MPU6000_DEBUG
        _dump_registers();
#endif
    }
    if (tries == 5) {
        hal.console->println_P(PSTR("Failed to boot MPU6000 5 times"));
        _i2c_sem->give();
        return false;
    }

    _register_write(MPUREG_PWR_MGMT_2, 0x00);            // only used for wake-up in accelerometer only low power mode
    hal.scheduler->delay(1);

    uint8_t default_filter;

    //Set filter to 188Hz, this causes gyro to output at 1khz instead of 8
    _register_write(MPUREG_CONFIG, 1);
    _register_write(MPUREG_SMPLRT_DIV, MPUREG_SMPLRT_1000HZ);

    hal.scheduler->delay(1);

    _register_write(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_2000DPS);  // Gyro scale 2000º/s

    hal.scheduler->delay(1);

    // read the product ID rev c has 1/2 the sensitivity of rev d
    _mpu6000_product_id = _register_read(MPUREG_PRODUCT_ID);
    //hal.console->printf("Product_ID= 0x%x\n", (unsigned) _mpu6000_product_id);

    if ((_mpu6000_product_id == MPU6000ES_REV_C4) || (_mpu6000_product_id == MPU6000ES_REV_C5) ||
        (_mpu6000_product_id == MPU6000_REV_C4)   || (_mpu6000_product_id == MPU6000_REV_C5)) {
        // Accel scale 8g (4096 LSB/g)
        // Rev C has different scaling than rev D
        _register_write(MPUREG_ACCEL_CONFIG,1<<3);
    } else {
        // Accel scale 8g (4096 LSB/g)
        _register_write(MPUREG_ACCEL_CONFIG,2<<3);
    }
    hal.scheduler->delay(1);

    // configure interrupt to fire when new data arrives
    _register_write(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);
    hal.scheduler->delay(1);

    // clear interrupt on any read, and hold the data ready pin high
    // until we clear the interrupt
    _register_write(MPUREG_INT_PIN_CFG, BIT_INT_RD_CLEAR /*| BIT_LATCH_INT_EN*/);
    _i2c_sem->give();

    hal.scheduler->delay(50);
    //configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);

    return true;
}

bool AP_InertialSensor_MPU6050::configure_fifo(uint8_t sensors)
{
    int16_t result = 0;

    /* Compass data isn't going into the FIFO. Stop trying. */
    sensors &= ~INV_XYZ_COMPASS;

    // Enable or disable the interrupts
    // set_int_enable(1);

    if (sensors) {
        _i2c->writeRegister(_addr, MPUREG_FIFO_EN, sensors);
    	if (reset_fifo(sensors)) {
            return -1;
        }
    }
    return 1;
}

#if MPU6000_DEBUG
// dump all config registers - used for debug
void AP_InertialSensor_MPU6050::_dump_registers(void)
{
    hal.console->println_P(PSTR("MPU6000 registers"));
    if (_i2c_sem->take(100)) {
        for (uint8_t reg=MPUREG_PRODUCT_ID; reg<=108; reg++) {
            uint8_t v = _register_read(reg);
            hal.console->printf_P(PSTR("%02x:%02x "), (unsigned)reg, (unsigned)v);
            if ((reg - (MPUREG_PRODUCT_ID-1)) % 16 == 0) {
                hal.console->println();
            }
        }
        hal.console->println();
        _i2c_sem->give();
    }
}
#endif

AP_InertialSensor_Backend *AP_InertialSensor_MPU6050::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_MPU6050 *sensor = new AP_InertialSensor_MPU6050(_imu);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->_init_sensor()) {
        delete sensor;
        return NULL;
    }

    return sensor;
}

bool AP_InertialSensor_MPU6050::gyro_sample_available(void) {
	return _last_sample_timestamp != _last_sample_update_timestamp;
}

bool AP_InertialSensor_MPU6050::accel_sample_available(void) {
	return _last_sample_timestamp != _last_sample_update_timestamp;
}
