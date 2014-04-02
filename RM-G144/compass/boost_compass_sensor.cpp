#include <boost/python.hpp>
#include <math.h>
#include <roboard.h>
#include <unistd.h>
#define PI 3.14159265359

const unsigned char i2c_address = 0x1e;

void wait_ms(unsigned long msec){
    while(msec--)usleep(1000);
}
class CompassSensor{
    private:
        int quadrant;
        int current_azimuth;
        unsigned int d1,d2,d3,d4,d5,d6;
        int x,y,z;
        double result;
        void read();
        int xyz_to_azimuth(double x, double y, double z);
    public:
        CompassSensor(); // ensure to sleep for 100 ms after this
        int get_azimuth();
        ~CompassSensor();
};

CompassSensor::CompassSensor(){
    roboio_SetRBVer(RB_110);
    if(i2c_Initialize(I2CIRQ_DISABLE) == false){
        printf("FALSE! %s\n", roboio_GetErrMsg());
    }
	i2c0_SetSpeed(I2CMODE_STANDARD, 100000L);
    wait_ms(100);

	i2c0master_StartN(i2c_address,I2C_WRITE, 2);//write 2 byte
	i2c0master_WriteN(0x02); //mode register
	i2c0master_WriteN(0x00); //continue-measureture mode
    wait_ms(100);
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

    x = ((d1 & 0xF0) > 0) ? ~(0xFFFF ^ (d1 * 256 + d2)): d1 * 256 + d2;
    y = ((d3 & 0xF0) > 0) ? ~(0xFFFF ^ (d3 * 256 + d4)): d3 * 256 + d4;
    z = ((d5 & 0xF0) > 0) ? ~(0xFFFF ^ (d5 * 256 + d6)): d5 * 256 + d6;

    current_azimuth =  xyz_to_azimuth((double) x, (double) y, (double) z);
}
int CompassSensor::xyz_to_azimuth(double x, double y, double z){
    double theta_rad = atan2(y, x);
    double theta_deg = (theta_rad/PI * 180) + (theta_rad > 0 ? 0: 360);
    return theta_deg;
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
