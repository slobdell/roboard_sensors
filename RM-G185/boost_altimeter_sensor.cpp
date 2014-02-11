#include <boost/python.hpp>
#include <math.h>
#include <roboard.h>
#include <stdio.h> // for printf

const unsigned char LPS331AP_ADDR =	(0xba>>1);
const unsigned char SHT21_ADDR	= (0x80>>1);
const unsigned char BMP180_ADDR	= (0xee>>1);

class AltimeterSensor{
    private:
        unsigned char lps331ap_state, lps331ap_press[3], lps331ap_temp[2];
        int sht21_humi[2], sht21_temp[2];
        double lps331ap_mbar, lps331ap_deg, sht21_rh, sht21_deg, height;
        int count[4];
        double offset;
        double raw_height_from_sensor;
        void read();
    public:
        AltimeterSensor();
        void zeroize_altitude();
        double get_altitude();
        ~AltimeterSensor();
};


AltimeterSensor::AltimeterSensor(){
    roboio_SetRBVer(RB_110);

	lps331ap_state = 0;
	lps331ap_press[0] = 0;
	lps331ap_press[1] = 0;
	lps331ap_press[2] = 0;

	lps331ap_temp[0] = 0;
	lps331ap_temp[1] = 0;

	count[0] = 0;
	count[1] = 0;
	count[2] = 0;
	count[3] = 0;

	i2c_Init(I2CMODE_STANDARD, 100000L);  // init I2C lib to 100Kbps
	i2c0master_StartN(SHT21_ADDR, I2C_WRITE, 2);

	// write user reg
	i2c0master_WriteN(0xe6);
	i2c0master_WriteN(0x01);

	// init LPS331AP
	// set CTRL_REG
	i2c0master_StartN(LPS331AP_ADDR, I2C_WRITE, 2);

	// set REG2
	i2c0master_WriteN(0x21);
	i2c0master_WriteN(0x00);

	i2c0master_StartN(LPS331AP_ADDR, I2C_WRITE, 2);

	// set REG1
	i2c0master_WriteN(0x20);
	i2c0master_WriteN(0xe0);

	i2c0master_StartN(LPS331AP_ADDR, I2C_WRITE, 2);

	//set REG3
	i2c0master_WriteN(0x22);
	i2c0master_WriteN(0x00);

	// set resolution mode
	i2c0master_StartN(LPS331AP_ADDR, I2C_WRITE, 2);

	i2c0master_WriteN(0x10);
	i2c0master_WriteN(0x6a);
	i2c0master_StartN(LPS331AP_ADDR, I2C_WRITE, 2);

	i2c0master_WriteN(0x25);
	i2c0master_WriteN(0x00);
	i2c0master_StartN(LPS331AP_ADDR, I2C_WRITE, 2);

	i2c0master_WriteN(0x26);
	i2c0master_WriteN(0x00);

	raw_height_from_sensor = -1;
}

void AltimeterSensor::read()
{
    //read SHT21
	i2c0master_StartN(SHT21_ADDR, I2C_WRITE,1);
	i2c0master_WriteN(0xf5);

	i2c0master_StartN(SHT21_ADDR, I2C_READ,3);
	sht21_humi[1] = i2c0master_ReadN();
	sht21_humi[0] = i2c0master_ReadN();
	// checksum
	i2c0master_ReadN();
	i2c0master_StartN(SHT21_ADDR, I2C_WRITE,1);

	i2c0master_WriteN(0xf3);
	i2c0master_StartN(SHT21_ADDR, I2C_READ,3);

	sht21_temp[1] = i2c0master_ReadN();
	sht21_temp[0] = i2c0master_ReadN();
	// checksum
	i2c0master_ReadN();

	// LPS331AP
	// read LPS331AP
	i2c_SensorRead(LPS331AP_ADDR, 0x27, &lps331ap_state,1);

	if(lps331ap_state == 0xff)
	{
		printf("LPS331AP error:%s !!\n",roboio_GetErrMsg());
	}
	// check error
	if(lps331ap_state& 0x02){
		if(i2c_SensorRead(LPS331AP_ADDR, 0x28, &lps331ap_press[0],1) == false){
			printf("LPS331AP fail to read pressure XLB (%s)!\n", roboio_GetErrMsg());
		}
		if(i2c_SensorRead(LPS331AP_ADDR, 0x29, &lps331ap_press[1],1) == false){
			printf("LPS331AP fail to read pressure LB (%s)!\n", roboio_GetErrMsg());
		}
		if(i2c_SensorRead(LPS331AP_ADDR, 0x2a, &lps331ap_press[2],1) == false){
			printf("LPS331AP fail to read pressure MSB (%s)!\n", roboio_GetErrMsg());
		}
	}

	if(lps331ap_state& 0x01){
		if(i2c_SensorRead(LPS331AP_ADDR, 0x2b, lps331ap_temp, 1) == false){
			printf("LPS331AP fail to read temp LSB (%s)!\n", roboio_GetErrMsg());
		}
		if(i2c_SensorRead(LPS331AP_ADDR, 0x2c, lps331ap_temp + 1, 1) == false){
			printf("LPS331AP fail to read temp MSB (%s)!\n", roboio_GetErrMsg());
		}
	}

	sht21_rh = (double) (((long)sht21_humi[1]<< 8) + (long)sht21_humi[0]);
	sht21_rh = sht21_rh*125/65536 - 6;

	sht21_deg = (double) (((long)sht21_temp[1]<< 8L) + (long)sht21_temp[0]);
	sht21_deg = sht21_deg*175.72/65535 - 46.85;

	lps331ap_mbar = (double) ((((long)lps331ap_press[2])<<16L) + (((long)lps331ap_press[1])<< 8L) + (long)lps331ap_press[0]);
	lps331ap_mbar = lps331ap_mbar/4096;

	/**************************************************
	PSVL = P*10^M
	M = StationHigh/(18400*(1+Temp/273))
	**************************************************/

	lps331ap_deg = (double) ((((int)lps331ap_temp[1]) << 8L) + (int)lps331ap_temp[0]);
	lps331ap_deg = 42.5 + lps331ap_deg / 480;

	height = log10(1013.25 / lps331ap_mbar) * 8400 * (1 + lps331ap_deg / 273.0);
	raw_height_from_sensor = height;
}

void AltimeterSensor::zeroize_altitude(){
	read();
	this->offset = raw_height_from_sensor;
}

double AltimeterSensor::get_altitude(){
	read();
	return raw_height_from_sensor - offset;
}

AltimeterSensor::~AltimeterSensor(){
	i2c_Close();
}

using namespace boost::python;
BOOST_PYTHON_MODULE(boost_altimeter_sensor){
    class_<AltimeterSensor>("AltimeterSensor")
        .def("zeroize_altitude", &AltimeterSensor::zeroize_altitude)
        .def("get_altitude", &AltimeterSensor::get_altitude);
}
