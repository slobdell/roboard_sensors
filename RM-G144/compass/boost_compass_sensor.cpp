#include <boost/python.hpp>
#include <math.h>
#include <roboard.h>

#define PI 3.14159265359

const unsigned char i2c_address = 0x1e;

class CompassSensor{
    private:
        int quadrant;
        int current_azimuth;
        unsigned int d1,d2,d3,d4,d5,d6;
        double x,y,z;
        double result;
        void read();
    public:
        CompassSensor(); // ensure to sleep for 100 ms after this
        int get_azimuth();
        ~CompassSensor();
};

CompassSensor::CompassSensor(){
    roboio_SetRBVer(RB_110);
	i2c_Init(I2CMODE_STANDARD, 100000L);  // init I2C lib to 100Kbps
	i2c0_SetSpeed(I2CMODE_STANDARD, 100000L);

	i2c0master_StartN(i2c_address,I2C_WRITE,2);//write 2 byte
	i2c0master_WriteN(0x02); //mode register
	i2c0master_WriteN(0x00); //continue-measureture mode
}
void CompassSensor::read(){
	i2c0master_StartN(i2c_address, I2C_WRITE, 1);
	i2c0master_SetRestartN(I2C_READ, 6);
	i2c0master_WriteN(0x03); // Read from data register (Address : 0x03)

	d1 = i2c0master_ReadN(); // X MSB
	d2 = i2c0master_ReadN(); // X LSB
	d3 = i2c0master_ReadN(); // Y MSB
	d4 = i2c0master_ReadN(); // Y LSB
	d5 = i2c0master_ReadN(); // Z MSB
	d6 = i2c0master_ReadN(); // Z LSB

	x = ((d1 & 0x80) != 0) ? (((~0) >> 16) << 16) | ((d1 << 8) + d2) : (d1 << 8) + d2;
    y = ((d3 & 0x80) != 0) ? (((~0) >> 16) << 16) | ((d3 << 8) + d4) : (d3 << 8) + d4;
    z = ((d5 & 0x80) != 0) ? (((~0) >> 16) << 16) | ((d5 << 8) + d6) : (d5 << 8) + d6;

	result = atan(y / x) * 180.0 / PI + 90;

	if(x >= 0 && y >= 0)quadrant = 1;
	if(x < 0 && y >= 0)quadrant = 2;
	if(x < 0 && y < 0)quadrant = 3;
	if(x >= 0 && y < 0)quadrant = 4;

	if(quadrant == 4 || quadrant == 1) result += 180;

	current_azimuth = result;
	if(x == 0 && y == 0 && z == 0) current_azimuth= -999;
}
int CompassSensor::get_azimuth(){
    this->read();
	return current_azimuth;
}
CompassSensor::~CompassSensor(){
	i2c_Close();
}

using namespace boost::python;
BOOST_PYTHON_MODULE(boost_compass_sensor){
    class_<CompassSensor>("CompassSensor")
        .def("get_azimuth", &CompassSensor::get_azimuth);
}
