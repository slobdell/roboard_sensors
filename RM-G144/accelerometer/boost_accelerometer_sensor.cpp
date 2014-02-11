#include <boost/python.hpp>
#include <math.h>
#include <roboard.h>

#define PI 3.14159265359

const unsigned char acceler_addr = 0x53;

class AccelerometerSensor
{
private:
	unsigned int d1, d2, d3, d4, d5, d6;
	int x_acceleration, y_acceleration, z_acceleration;
public:
    int get_x_acceleration(){ return x_acceleration; }
    int get_y_acceleration(){ return y_acceleration; }
    int get_z_acceleration(){ return z_acceleration; }
	AccelerometerSensor(); // ensure you sleep for 100 ms after init
	void read();
	int get_pitch();
	int get_roll();
	~AccelerometerSensor();
};

AccelerometerSensor::AccelerometerSensor(){
    roboio_SetRBVer(RB_110);
	i2c_Init(I2CMODE_STANDARD, 100000L);  // init I2C lib to 100Kbps
	i2c0_SetSpeed(I2CMODE_STANDARD, 100000L);

	i2c0master_StartN(acceler_addr, I2C_WRITE, 2);  // write 2 bytes
	i2c0master_WriteN(0x2d);  // power control register
	i2c0master_WriteN(0x28);  // link and measure mode

	i2c0master_StartN(acceler_addr, I2C_WRITE, 2);  // write 2 bytes
	i2c0master_WriteN(0x31);  // data format register
	i2c0master_WriteN(0x08);  // full resolution

	i2c0master_StartN(acceler_addr, I2C_WRITE, 2);  // write 2 bytes
	i2c0master_WriteN(0x38);  // FIFO Control register
	i2c0master_WriteN(0x00);  // bypass mode

    x_acceleration = y_acceleration = z_acceleration = 0;
}

void AccelerometerSensor::read(){
	i2c0master_StartN(acceler_addr, I2C_WRITE, 1);
	i2c0master_SetRestartN(I2C_READ, 6);
	i2c0master_WriteN(0x32);  // read from X register address 0x32
	d1 = i2c0master_ReadN();  // X LSB
	d2 = i2c0master_ReadN();  // X MSB
	d3 = i2c0master_ReadN();  // Y LSB
	d4 = i2c0master_ReadN();  // Y MSB
	d5 = i2c0master_ReadN();  // Z LSB
	d6 = i2c0master_ReadN();  // Z MSB
	x_acceleration=((d2&0x80) !=0)?(((~0)>>16)<<16) | ((d2<<8)+d1):(d2<<8)+d1;
	y_acceleration=((d4&0x80) !=0)?(((~0)>>16)<<16) | ((d4<<8)+d3):(d4<<8)+d3;
	z_acceleration=((d6&0x80) !=0)?(((~0)>>16)<<16) | ((d6<<8)+d5):(d6<<8)+d5;
}

int AccelerometerSensor::get_pitch(){
	if(y_acceleration == 0 && x_acceleration == 0 && z_acceleration == 0){
        return -999;
    }
	return 180 * atan2(y_acceleration, sqrt((float)(x_acceleration * x_acceleration+z_acceleration * z_acceleration))) / PI;
}
int AccelerometerSensor::get_roll(){
	return 180 * atan2(x_acceleration, sqrt((float)(y_acceleration * y_acceleration + z_acceleration * z_acceleration))) / PI;
}

AccelerometerSensor::~AccelerometerSensor(){
	i2c_Close();
}

using namespace boost::python;
BOOST_PYTHON_MODULE(boost_accelerometer_sensor){
    class_<AccelerometerSensor>("AccelerometerSensor")
        .def("read", &AccelerometerSensor::read)
        .def("get_pitch", &AccelerometerSensor::get_pitch)
        .def("get_roll", &AccelerometerSensor::get_roll)
        .def("x_acceleration", &AccelerometerSensor::get_x_acceleration)
        .def("y_acceleration", &AccelerometerSensor::get_y_acceleration)
        .def("z_acceleration", &AccelerometerSensor::get_z_acceleration);
}
