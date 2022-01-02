/****************************************************
 * m5stack Core2
 * ----use unit----
 * PaHUB (https://docs.m5stack.com/en/unit/pahub)
 *  I2C address : 0x71(Needs soldering)
 * ENViii (https://docs.m5stack.com/en/unit/envIII)
 *  I2C address : 0x70, 0x68(QMP6988, SHT)
 * IMU6886 (https://docs.m5stack.com/en/unit/imu)
 *  I2C address : 0x68
 * TVOC/eCO2 SGP30(https://docs.m5stack.com/en/unit/tvoc)
 *  I2C address : 0x58
 * GPS Module(NEO-M8N) (https://docs.m5stack.com/en/module/gps)
 *  TXD : GPIO16(In m5stack core2, it's connected to GPIO13)
 *  RXD : GPIO17(In m5stack core2, it's connected to GPIO14)
 * ----------------
 ****************************************************/

#include <M5Core2.h>
#include "IMU_6886.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <unistd.h>
#include <Wire.h>
#include "ClosedCube_TCA9548A.h" //https://github.com/m5stack/M5Core2/tree/master/examples/Unit/PaHUB_TCA9548A/PaHUB_TCA9548A
/**
 * https://github.com/m5stack/UNIT_ENV
 * ENV III [SHT30+QMP6988]
 * !CAUTION! In this project, rewrote some code content
 *           The changed line is commented as "//---ritu:"
 */
#include "src/UNIT_ENV/UNIT_ENV.h"
#include "src/TVOC/Adafruit_SGP30.h" //https://github.com/m5stack/M5-ProductExampleCodes/blob/master/Unit/TVOC/TVOC

#include "src/TinyGPSPlus/TinyGPS++.h" //https://github.com/mikalhart/TinyGPSPlus

// must come before #include <ArduinoJson.h>
#define ARDUINOJSON_USE_DOUBLE 1
#include <ArduinoJson.h>

#define PaHub_I2C_ADDRESS 0x71 // changed I2C address(default:0x70(same address as ENViii))

// Parameters in M5.begin()
const bool LCD_Enable = true;
const bool SD_Enable = true;
const bool Serial_Enable = true;
const bool I2C_Enable = true;

////////////////////////////////// FLAG LIST ///////////////////////////////////
//////// TEST FLAG ////////
/** true:three loop & get data, false:one loop & get data*/
const bool FLAG_THREE_MPU6886 = true;
const int NUM_MPU6886 = 3;
/** true:use gps module, false:non used gps module */
const bool USE_GPS_MODULE = false;
/** true:use Wi-Fi, false:non used Wi-Fi */
const bool USE_WiFi = true;
/** true:use SD card, false:non use SD card */
const bool USE_SD = false;
/** true:automatic stop when stopped for a long time, false:non stopping */
const bool FLAG_AUTOSTOP = false;
/** 1-cycle-time = true:CONSTANT_CYCLE, false:calculated time */
const bool FLAG_CONSTANT_CYCLE = true;
const uint32_t CONSTANT_CYCLE = 1000;
//////// SENSOR FLAG ////////
/** true:perform SGP30 related functions, false: Do not*/
bool FLAG_SGP30 = true;
//////// MODE FLAG ////////
/** true:RUN view_data(), false:do nothing , default:true*/
bool FLAG_view_data = true;
bool FLAG_test_bat_view = false;
bool FLAG_test_gps = false;
//////// opelation FLAG ////////
/** true:car is stopped for a long time, false:car is not stopped */
bool FLAG_car_stop = false;
////////////////////////////////////////////////////////////////////////////////

WiFiClient client;
// const char* ssid     = "CPSLab_ngpp";
// const char* password = "evbskis5dtir7";
// const char* ssid     = "aterm-78c7a8-g";
// const char* password = "2572709ec48e5";
const char *ssid = "A8A79546F534";
const char *password = "2211588297557";

WiFiUDP wifiUdp;
const char *pc_addr = "192.168.3.13"; //
// const char *pc_addr = "192.168.0.02";//
const int pc_port = 50007; //送信先のポート
const int my_port = 50008; //自身のポート

// 標準気圧
float standardPressure = 0;

IMU_6886 imu6886;

ClosedCube::Wired::TCA9548A tca9548a;

SHT3X sht30;
QMP6988 qmp6988;

Adafruit_SGP30 sgp;

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

typedef struct _rokujiku
{
	float accX;
	float accY;
	float accZ;
	float gyroX;
	float gyroY;
	float gyroZ;
	float pitch;
	float roll;
	float yaw;
	float temp;
} rokujiku_t;
rokujiku_t rokujiku[3];

typedef struct _enviii
{									// Arduino IDEの関数のプロトタイプ宣言がプリプロセッサ直後に挿入される仕様により、死ぬ(関数で引数として使っている)
	float cTemp;		// SHT3X
	float humidity; // SHT3X
	float pressure; // QMP6988
} enviii_t;
enviii_t enviii;
void enviii_get(enviii_t *env);
void enviii_init()
{
	if (qmp6988.init() == 0)
	{
		// M5.Lcd.printf("qmp6988:error");
		while (true)
			;
	}
	return;
}
void enviii_get(enviii_t *env)
{
	while (sht30.get() != 0)
	{ // i2c read error
		// return;
		delay(10);
	}
	env->cTemp = sht30.cTemp;
	env->humidity = sht30.humidity;
	env->pressure = qmp6988.calcPressure();
	return;
}

//===================================TVOC SGP30===================================
typedef struct _sgp30_data
{
	uint16_t tvoc;		// TVOC
	uint16_t eco2;		// eCO2
	uint16_t h2;			// raw H2
	uint16_t ethanol; // raw ethanol
} sgp30_data_t;
sgp30_data_t sgp30_data;
boolean sgp30_init()
{
	// SGP30 needs 15 seconds to initialize calibration after power on.
	uint32_t sgp30_init_time = 15000; // 15sec
	if (!sgp.begin())
	{
		return false;
	}
	while (millis() < sgp30_init_time)
		;
	return true;
}
void sgp30_get()
{
	static uint32_t sgp30_sr = 0; // sgp30 sampling rate = 1Hz
	if (sgp30_sr > millis())
	{
		return;
	}
	else
	{
		sgp30_sr = millis() + 1000;
	}

	if (sgp.IAQmeasure())
	{
		sgp30_data.tvoc = sgp.TVOC;
		sgp30_data.eco2 = sgp.eCO2;
	}
	if (sgp.IAQmeasureRaw())
	{
		sgp30_data.h2 = sgp.rawH2;
		sgp30_data.ethanol = sgp.rawEthanol;
	}

	return;
}
//================================================================================

typedef struct _gpsdata
{
	uint32_t time;
	uint32_t date;
	double lat;
	double lng;
	// double alt;//altitude
} gpsdata_t;
gpsdata_t gpsdata;
// This custom version of delay() ensures that GPS objects work properly.
static void smartDelay(unsigned long ms)
{
	unsigned long start = millis();
	do
	{
		while (gpsSerial.available())
			gps.encode(gpsSerial.read());
	} while (millis() - start < ms);
}
void gpsdata_get()
{
	smartDelay(0);
	gpsdata.time = gps.time.value() * 10; // hhmmss00 * 10 = hhmmss000
	gps_time_add(gps_time_ms(true));			//(.)sss
	// gpsdata.time += gps_time_ms(true);//(.)sss
	//// GPSモジュールからミリ秒が取得できるようになるまで上記コード(m5のmillis()を利用)で対応 ////
	/*if(gps.time.isValid()){
		gpsdata.time += gps.time.age();//(.)sss
	}*/
	gpsdata.date = gps.date.value();	// ddmmyy
	gpsdata.lat = gps.location.lat(); // dd.
	gpsdata.lng = gps.location.lng(); // ddd.

	// gpsdata.alt = 0;//altitude

	return;
}
void gps_init()
{
	// It requires the use of SoftwareSerial, and assumes that you have a 4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
	gpsSerial.begin(9600);
	gpsdata_get();
	int i = 9; // number of waits
	while (gpsdata.lat == 0.0)
	{
		if (i)
		{ // i > 0
			smartDelay(1000);
			gpsdata_get();
			M5.Lcd.printf(".");
			i--;
		}
		else
		{ // i == 0
			M5.Lcd.printf(":not find lat&lng");
			break;
		}
	}
	return;
}

void wifi_init()
{
	WiFi.begin(ssid, password);
	while (WiFi.status() != WL_CONNECTED)
	{
		delay(500);
		// M5.Lcd.print(".");
	}
	// M5.Lcd.print("\r\nWiFi connected\r\nIP address: ");
	// M5.Lcd.println(WiFi.localIP());

	wifiUdp.begin(my_port);
}
void wifi_send()
{
	wifiUdp.beginPacket(pc_addr, pc_port);
	wifiUdp.printf("%6.2f,%6.2f,%6.2f,%5.2f,%5.2f,%5.2f,%6.2f,%6.2f,%6.2f,%5.2f,%5.2f,%5.2f,%6.2f,%6.2f,%6.2f,%5.2f,%5.2f,%5.2f", rokujiku[0].gyroX, rokujiku[0].gyroY, rokujiku[0].gyroZ, rokujiku[0].accX, rokujiku[0].accY, rokujiku[0].accZ, rokujiku[1].gyroX, rokujiku[1].gyroY, rokujiku[1].gyroZ, rokujiku[1].accX, rokujiku[1].accY, rokujiku[1].accZ, rokujiku[2].gyroX, rokujiku[2].gyroY, rokujiku[2].gyroZ, rokujiku[2].accX, rokujiku[2].accY, rokujiku[2].accZ);
	// uint8_t data[] = "abc";
	// wifiUdp.write(data, sizeof(data));
	wifiUdp.endPacket();
	// M5.Lcd.print(".");
	delay(500);
}

void PaHUB_init()
{
	tca9548a.address(PaHub_I2C_ADDRESS);
}

void send_json()
{
	wifiUdp.beginPacket(pc_addr, pc_port);
	cleate_json();
	wifiUdp.endPacket();
	// delay(500);
}
void cleate_json()
{
	StaticJsonDocument<512> doc;
	// 標準気圧をくっつける

	JsonObject MPU6886 = doc.createNestedObject("MPU6886");

	JsonObject MPU6886_head = MPU6886.createNestedObject("head");
	MPU6886_head["accX"] = rokujiku[0].accX;
	MPU6886_head["accY"] = rokujiku[0].accY;
	MPU6886_head["accZ"] = rokujiku[0].accZ;
	MPU6886_head["gyroX"] = rokujiku[0].gyroX;
	MPU6886_head["gyroY"] = rokujiku[0].gyroY;
	MPU6886_head["gyroZ"] = rokujiku[0].gyroZ;

	JsonObject MPU6886_right = MPU6886.createNestedObject("right");
	MPU6886_right["accX"] = rokujiku[1].accX;
	MPU6886_right["accY"] = rokujiku[1].accY;
	MPU6886_right["accZ"] = rokujiku[1].accZ;
	MPU6886_right["gyroX"] = rokujiku[1].gyroX;
	MPU6886_right["gyroY"] = rokujiku[1].gyroY;
	MPU6886_right["gyroZ"] = rokujiku[1].gyroZ;

	JsonObject MPU6886_left = MPU6886.createNestedObject("left");
	MPU6886_left["accX"] = rokujiku[2].accX;
	MPU6886_left["accY"] = rokujiku[2].accY;
	MPU6886_left["accZ"] = rokujiku[2].accZ;
	MPU6886_left["gyroX"] = rokujiku[2].gyroX;
	MPU6886_left["gyroY"] = rokujiku[2].gyroY;
	MPU6886_left["gyroZ"] = rokujiku[2].gyroZ;

	JsonObject ENVIII = doc.createNestedObject("ENVIII");
	ENVIII["cTemp"] = enviii.cTemp;
	ENVIII["humidity"] = enviii.humidity;
	ENVIII["pressure"] = enviii.pressure;
	ENVIII["standardPressure"] = standardPressure;

	if (USE_GPS_MODULE)
	{
		JsonObject GPS = doc.createNestedObject("GPS");
		GPS["time"] = gpsdata.time;
		GPS["date"] = gpsdata.date;
		GPS["lat"] = gpsdata.lat;
		GPS["long"] = gpsdata.lng;
	}

	serializeJson(doc, wifiUdp);
}

File file;
char filename[32];
bool flg_sd_stop = false;
void setup_csvfile()
{
	String str_filename;
	if (USE_GPS_MODULE)
	{
		str_filename = "/log" + (String)gps.date.year() + (String)gps.date.month() + (String)gps.date.day() + (String)gpsdata.time + ".csv";
	}
	else
	{ // when not use GPS
		str_filename = "l.csv";
	}

	str_filename.toCharArray(filename, 32);
	file = SD.open(filename, FILE_APPEND);
	if (!file)
	{
		M5.Lcd.println("ERROR:SD FILE");
		while (1)
			;
	}

	if (USE_GPS_MODULE)
	{
		const char *tag_gpsdata[] = {"time", "date", "lat", "lng"};
		for (int i = 0; i < 4; i++)
		{
			file.print("GPS-");
			file.print(tag_gpsdata[i]);
			file.print(",");
		}
	}

	const char *tag_rokujiku[] = {"accX", "accY", "accZ", "gyroX", "gyroY", "gyroZ"};
	const char *name_mpu6886[] = {"head", "right", "left"};
	for (int i = 0; i < NUM_MPU6886; i++)
	{
		file.print("MPU6886-");
		for (int j = 0; j < 6; j++)
		{
			if (FLAG_THREE_MPU6886)
			{
				file.print(name_mpu6886[i]);
				file.print("-");
			}
			file.print(tag_rokujiku[j]);
			file.print(",");
		}
	}

	const char *tag_enviii[] = {"cTemp", "humidity", "pressure"};
	for (int i = 0; i < 3; i++)
	{
		file.print("enviii-");
		file.print(tag_enviii[i]);
		file.print(",");
	}

	if (FLAG_SGP30)
	{
		const char *tag_sgp30[] = {"TVOC", "eCO2", "H2", "ethanol"};
		for (int i = 0; i < 4; i++)
		{
			file.print("sgp30-");
			file.print(tag_sgp30[i]);
			file.print(",");
		}
	}

	file.println();
	file.close();
}
void send_SD()
{
	static int sdtime = 0;
	if (flg_sd_stop)
	{
		sdtime = 0;
		flg_sd_stop = false;
	}
	if (sdtime == 0)
		file = SD.open(filename, FILE_APPEND);
	if (!file)
	{
		M5.Lcd.println("ERROR:SD FILE");
		while (1)
			;
	}
	cleate_csvdata();
	// cleate_json();
	sdtime++;
	if (sdtime == 1000)
	{
		file.close();
		sdtime = 0;
	}
}
void cleate_csvdata()
{
	String str_rokujiku;
	for (int i = 0; i < NUM_MPU6886; i++)
	{
		str_rokujiku += String(rokujiku[i].accX, 10) + ",";
		str_rokujiku += String(rokujiku[i].accY, 10) + ",";
		str_rokujiku += String(rokujiku[i].accZ, 10) + ",";
		str_rokujiku += String(rokujiku[i].gyroX, 10) + ",";
		str_rokujiku += String(rokujiku[i].gyroY, 10) + ",";
		str_rokujiku += String(rokujiku[i].gyroZ, 10) + ",";
	}

	String str_enviii = String(enviii.cTemp, 10) + "," + String(enviii.humidity, 10) + "," + String(enviii.pressure, 10) + ",";

	String str_gps;
	if (USE_GPS_MODULE)
	{
		str_gps = (String)gpsdata.time + "," + (String)gpsdata.date + "," + String(gpsdata.lat, 10) + "," + String(gpsdata.lng, 10) + ",";
	}
	else
	{
		str_gps = "";
	}

	String str_sgp30;
	if (FLAG_SGP30)
	{
		str_sgp30 = (String)sgp30_data.tvoc + "," + (String)sgp30_data.eco2 + "," + (String)sgp30_data.h2 + "," + (String)sgp30_data.ethanol + ",";
	}
	else
	{
		str_sgp30 = "";
	}

	String str_buf = str_gps + str_rokujiku + str_enviii + str_sgp30 + '\n';

	file.write((uint8_t *)str_buf.c_str(), str_buf.length());

	// char bufbuf[512];
	// str_buf.toCharArray(bufbuf,512);
	// file.print(bufbuf);
}
void stop_SD()
{
	file.close();
	flg_sd_stop = true; // send_sdでopen/closeまでのログ書き込み回数の初期化(この関数の後のsend_sdでopenから始まる)
}

/**
 * @brief detect 3 buttons on screen
 * @return non:0, left:1, center:2, right:3
 * @note m5stck core2 ONLY!
 */
int m5_button()
{ // touched core2 button
	// y>240, x: left<109 109<center<218 218<right
	TouchPoint_t pos = M5.Touch.getPressPoint();
	if (pos.y > 240)
	{
		if (pos.x > 218)
		{
			return 3; // right button was pushed
		}
		else if (pos.x >= 109)
		{
			return 2; // center button was pushed
		}
		else if (pos.x < 109)
		{
			return 1; // left button was pushed
		}
	}
	return 0; // button was not pushed
}
void m5_pause()
{
	M5.Lcd.fillScreen(BLACK);
	M5.Lcd.setTextColor(GREEN, BLACK);
	M5.Lcd.setTextSize(2);

	delay(600);

	M5.Lcd.setCursor(0, 20);
	M5.Lcd.println(" <-- ! SHUTDOWN button !");
	M5.Lcd.setCursor(0, 42);
	M5.Lcd.println("     press for 5 seconds");
	M5.Lcd.setCursor(0, 200);
	// M5.Lcd.println("   RE START  |  BUTTON");
	M5.Lcd.println("    ! RE START BUTTON !");
	M5.Lcd.setCursor(0, 222);
	M5.Lcd.println("             @");

	M5.Lcd.setTextSize(4);
	M5.Lcd.setCursor(100, 100);
	M5.Lcd.println("stop!");

	delay(600);

	do
	{
		delay(50);
	} while (m5_button() != 2); // untill the center button is pressed

	lcd_setup(); // clean display
	delay(800);

	return;
}

uint16_t loop_counter()
{
	static uint16_t loop_count = 0;
	return loop_count++;
}

/** screen:black, text:green 2point */
void lcd_setup()
{
	M5.Lcd.fillScreen(BLACK);
	M5.Lcd.setTextColor(GREEN, BLACK);
	M5.Lcd.setTextSize(2);
}

void view_data()
{
	// lcd_setup();

	M5.Lcd.setCursor(0, 20);
	M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", rokujiku[0].gyroX, rokujiku[0].gyroY, rokujiku[0].gyroZ);
	M5.Lcd.setCursor(0, 42);
	M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", rokujiku[0].accX, rokujiku[0].accY, rokujiku[0].accZ);
	if (FLAG_THREE_MPU6886)
	{
		M5.Lcd.setCursor(0, 65);
		M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", rokujiku[1].gyroX, rokujiku[1].gyroY, rokujiku[1].gyroZ);
		M5.Lcd.setCursor(0, 87);
		M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", rokujiku[1].accX, rokujiku[1].accY, rokujiku[1].accZ);
		M5.Lcd.setCursor(0, 110);
		M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", rokujiku[2].gyroX, rokujiku[2].gyroY, rokujiku[2].gyroZ);
		M5.Lcd.setCursor(0, 132);
		M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", rokujiku[2].accX, rokujiku[2].accY, rokujiku[2].accZ);
	}

	M5.Lcd.setCursor(0, 155);
	M5.Lcd.printf(" %2.1f*C %0.1f%% %0.2fPa", enviii.cTemp, enviii.humidity, enviii.pressure);

	if (USE_GPS_MODULE)
	{
		M5.Lcd.setCursor(0, 177);
		M5.Lcd.printf("time %09d date %06d", gpsdata.time, gpsdata.date);
		M5.Lcd.setCursor(0, 200);
		M5.Lcd.printf(" %f : %f", gpsdata.lat, gpsdata.lng);
	}

	return;
}

void setup()
{
	// Initialize the M5Stack object
	M5.begin(LCD_Enable, SD_Enable, Serial_Enable, I2C_Enable);

	lcd_setup();

	M5.Lcd.setCursor(0, 20);
	M5.Lcd.printf("M5 ready");

	if (USE_SD)
	{
		if (!SD.begin())
		{
			M5.Lcd.setCursor(0, 200);
			M5.Lcd.printf("SD ERROR");
			while (1)
				;
		}
	}

	if (USE_GPS_MODULE)
	{
		M5.Lcd.setCursor(0, 42);
		gps_init();
		M5.Lcd.setCursor(0, 42);
		M5.Lcd.print(gps_time_ms(false));
		M5.Lcd.setCursor(0, 42);
		M5.Lcd.printf("GPS ready");
	}

	if (USE_WiFi)
	{
		wifi_init();
		M5.Lcd.setCursor(0, 65);
		M5.Lcd.printf("Wi-Fi ready");
	}

	Wire.begin(32, 33);
	M5.Lcd.setCursor(0, 87);
	M5.Lcd.printf("Wire ready");

	PaHUB_init();
	for (uint8_t i = 0; i < NUM_MPU6886; i++)
	{
		tca9548a.selectChannel(i);
		imu6886.Init(32, 33);
	}
	tca9548a.selectChannel(3);
	enviii_init();
	tca9548a.selectChannel(4);
	FLAG_SGP30 = sgp30_init();

	if (USE_GPS_MODULE)
	{
		gpsdata_get();
	}
	if (USE_SD)
	{
		setup_csvfile();
	}

	// 標準気圧を求める処理を描く。
	tca9548a.selectChannel(3);
	enviii_get(&enviii);
	M5.Lcd.setCursor(0, 120);
	M5.Lcd.println("Setting standard atmospheric pressure");
	float sumPressure = 0;
	time_t stopper = 10 * CLOCKS_PER_SEC + clock();
	while (stopper > clock())
	{
		sumPressure = sumPressure + enviii.pressure;
		sleep(1);
	}
	standardPressure = sumPressure / 10;
	M5.Lcd.println(standardPressure);
	M5.Lcd.setCursor(0, 190);
	M5.Lcd.println("setup:all clear!");
	M5.Lcd.println("      ! TOUCH BUTTON !");
	M5.Lcd.println("             @");
	delay(200);
	int count_ModeSelect = 0;
	while (m5_button() != 2)
	{
		if (m5_button() == 3)
		{
			count_ModeSelect++;
			if (count_ModeSelect > 250)
			{ // 5sec
				// FLAG_view_data = false;
				M5.Lcd.setCursor(0, 190);
				M5.Lcd.printf(" Mode change:     ");
				delay(200);
				mode_select();
				count_ModeSelect = 0;
			}
		}
		delay(20);
	}

	if (FLAG_test_gps)
	{ // gps状況確認
		delay(400);
		test_gps();
	}

	M5.Lcd.fillScreen(BLACK);
	delay(500);
}

void loop()
{
	// m5_time_watch(true);//---test time_watch
	// delay(500);

	for (uint8_t i = 0; i < NUM_MPU6886; i++)
	{
		tca9548a.selectChannel(i);
		imu6886.getGyroData(&rokujiku[i].gyroX, &rokujiku[i].gyroY, &rokujiku[i].gyroZ);
		imu6886.getAccelData(&rokujiku[i].accX, &rokujiku[i].accY, &rokujiku[i].accZ);
		// imu6886.getAhrsData(&rokujiku[i].itch,&rokujiku[i].roll,&rokujiku[i].yaw);
		imu6886.getTempData(&rokujiku[i].temp);
	}

	tca9548a.selectChannel(3);
	enviii_get(&enviii);

	if (FLAG_SGP30)
	{
		tca9548a.selectChannel(4);
		sgp30_get();
	}

	if (USE_GPS_MODULE)
	{
		gpsdata_get();
	}

	if (FLAG_view_data)
	{
		view_data();

		if (USE_WiFi)
		{
			M5.Lcd.setCursor(0, 222);
			M5.Lcd.println(WiFi.localIP());
		}

		M5.Lcd.setCursor(227, 222);
		M5.Lcd.printf("%-5d", loop_counter());
	}
	if (FLAG_test_bat_view)
	{
		test_bat_get();
		test_bat_view();

		M5.Lcd.setCursor(227, 222);
		M5.Lcd.printf("%-5d", loop_counter());
	}

	// wifi_send();//test code
	if (USE_WiFi)
	{
		send_json();
	}
	if (USE_SD)
	{
		send_SD();
	}

	if (m5_button() == 2)
	{
		stop_SD();
		m5_pause();
	}

	if (FLAG_AUTOSTOP)
	{
		judge_car_move();
		if (FLAG_car_stop)
		{
			stop_SD();
			sleep_car_stop();
		}
	}

	sense_cycle(); // Place this function at the end of loop()

	// M5.Lcd.setCursor(0, 222);//---test time_watch
	// M5.Lcd.printf("%d    ",m5_time_watch(false));//---test time_watch

	/*static bool flag_display = true;//true:何もせず,false:画面止める
	static bool flag_ambient = false;
	static bool flag_mouse = false;
	switch(m5_button()){
		case 0: //non
			break;
		case 1: //left
			//flag_ambient = true;
			flag_mouse = true;
			break;
		case 2: //center
			flag_display = false;
			break;
		case 3: //right
			//flag_ambient = false;
			flag_mouse = false;
			break;
	}*/

	/*while(!flag_display){
		delay(100);
		if(m5_button() == 2){
			flag_display = true;
			delay(250);
			break;
		}
	}*/

	/*//m5_time_watch(true);//---test time_watch
	//delay(500);
	//M5.Lcd.setCursor(0, 222);//---test time_watch
	/M5.Lcd.printf("%d    ",m5_time_watch(false));//---test time_watch*/
}
//================================================================================

/**
 * @brief Used only for testing
 * @param tmw_select true: set the start time , false:do nothing
 * @return millisecond = subtract now time from start time
 */
uint32_t m5_time_watch(bool tmw_select)
{
	static uint32_t st_time = 0; // start time
	// uint32_t nw_time = 0;//now time
	if (tmw_select)
	{ // set start time
		st_time = millis();
	}
	return millis() - st_time;
}

uint8_t mode_select()
{
	const uint8_t mode_num = 5;
	const uint8_t flag_num = 3;
	//[mode_num] 0:default, 1:dont view data, 2:view battery-data, 3~4:chack the gps before loop()
	//[flag_num] 0:FLAG_view_data, 1:FLAG_test_bat_view, 2:FLAG_test_gps
	const bool FLAG_MODE[mode_num][flag_num] = {{true, false, false},
																							{false, false, false},
																							{false, true, false},
																							{true, false, true},
																							{false, false, true}};

	uint8_t select = 0; // mode number

	uint32_t bt_interval = 200; // button press interval

	bool while_flag = true;
	while (while_flag)
	{
		// Pressed button
		// right-button:next mode, left-button:Previous mode, center-button:finish selection
		switch (m5_button())
		{
		case 0: // non
			break;
		case 1: // left button
			select++;
			delay(bt_interval);
			break;
		case 2: // center button
			// Get out of this while-statement
			while_flag = false;
			continue;
		case 3: // right button
			select--;
			delay(bt_interval);
			break;
		}
		// When 'select' exceeds the numbur of modes
		if (select >= mode_num)
		{
			select = 0;
		}
		M5.Lcd.setCursor(164, 190);
		M5.Lcd.print(select);
		delay(50);
	}

	M5.Lcd.printf(" ok");

	FLAG_view_data = FLAG_MODE[select][0];
	FLAG_test_bat_view = FLAG_MODE[select][1];
	FLAG_test_gps = FLAG_MODE[select][2];

	delay(bt_interval);
	return select;
}

/** ---TASK LIST---
 * [@]GPS時刻のミリ秒がGPSモジュールから取得できないので、こちらでなんとかする
 * [@]頻度の精密化
 *    	SDカードの書き込み時間短縮化の検討
 * [@]車停止中のスリープ方法
 *    	LCD・バックライト消灯、車再始動時の検出、(GPSへの給電停止)
 */

//=========================get milliseconds using millis()========================
/** @param flag_milli_getset true:milli_get(), false:milli_set() */
uint32_t gps_time_ms(bool flag_milli_getset)
{
	static uint32_t m5ms_ref = 0; // gps-millisecond reference in m5
	uint32_t now_mi_se;
	if (flag_milli_getset)
	{
		now_mi_se = milli_get(m5ms_ref);
		return now_mi_se;
	}
	else
	{
		m5ms_ref = milli_set();
		return m5ms_ref;
	}

	return 0;
}
uint32_t milli_set()
{
	uint32_t gps_milli_second = 0;

	smartDelay(100);
	// static uint32_t old_time = gps.time.value();
	/*while(!gps.time.isValid()){
		smartDelay(100);
	}*/
	uint32_t old_time = gps.time.value();
	while (gps.date.value() == 0)
	{
		smartDelay(100);
	}
	while (old_time == 0)
	{
		smartDelay(100);
		old_time = gps.time.value();
	}

	while (true)
	{
		while (gpsSerial.available())
			gps.encode(gpsSerial.read());
		if (old_time < gps.time.value())
		{ //主に1秒進んだ時
			gps_milli_second = millis() % 1000;
			break;
		}
	}

	return gps_milli_second;
}
uint32_t milli_get(uint32_t ms_ref)
{
	uint32_t now_tm = millis() - ms_ref; // now milli second
	uint32_t gps_millis = now_tm % 1000;
	/*uint32_t now_tm = millis() % 1000;//now milli second
	uint32_t gps_millis = now_tm - ms_ref;
	if(now_tm < ms_ref){
		gps_millis = ~gps_millis;
	}*/
	/*if(now_tm < ms_ref){
		gps_millis = ms_ref - now_tm;
	}else{
		gps_millis = now_tm - ms_ref;
	}*/

	static uint32_t old_gpstime = gpsdata.time;
	static uint32_t old_tm_ms = gps_millis;
	if (old_gpstime == gpsdata.time)
	{
		if (old_tm_ms > gps_millis)
		{
			// gps_millis += 1000;
			gpsdata.time += 1000;
		}
	}
	else if (old_gpstime < gpsdata.time)
	{
		if (old_tm_ms < gps_millis)
		{
			// gps_millis -= 1000;
			// gpsdata.time -= 1000;
			gps_time_sub(1000);
		}
	}
	else
	{ // if(old_gpstime > gpsdata.time)
	}
	old_gpstime = gpsdata.time;
	old_tm_ms = gps_millis;

	return gps_millis;
}
/**
 * @brief 時刻の指定時間追加関数。現状はmsのみ可。
 * @param tm_add Time to add to gpsdata.time
 * @note 分単位でのaddの時の繰り上がりについては考慮していない。
 */
void gps_time_add(uint32_t tm_add)
{
	gpsdata.time += tm_add;
	if (gpsdata.time % 100000 >= 60000)
	{												 // 60sec以上の時
		gpsdata.time += 40000; // += -60sec + 1min
		if (gpsdata.time % 10000000 >= 6000000)
		{													 // 60min以上の時
			gpsdata.time += 4000000; // += -60min + 1hour
			if (gpsdata.time >= 240000000)
			{ // 24hour以上の時
				// 24:00:00.000引いて日付を1日進める
				//日付の処理が面倒なので日付進めずにtime23:59:59.999にする
				gpsdata.time = 235959999;
			}
		}
	}
	return;
}
void gps_time_sub(uint32_t tm_sub)
{
	gpsdata.time -= tm_sub;
	if (gpsdata.time % 100000 >= 60000)
	{												 // 60sec以上の時
		gpsdata.time -= 40000; // -= -40sec
		if (gpsdata.time % 10000000 >= 6000000)
		{													 // 60min以上の時
			gpsdata.time -= 4000000; // -= -40min
			if (gpsdata.time >= 240000000)
			{ // 24hour以上の時
				//(2^(32)-1)-23:59:59.999引いて日付を1日戻す
				//日付の処理が面倒なので日付戻さずにtime00:00:00.000にする
				gpsdata.time = 000000000;
			}
		}
	}
	return;
}
//================================================================================

//===============================In-vehicle sensing===============================
/** */
void judge_car_move()
{
	const float threshold = 50;			 //ジャイロ値の止まってる判定の閾値的な(要検討)
	static uint32_t judge_timer = 0; //停止(or 再始動)した時の時間を保持　

	float gyro[3];
	gyro[0] = rokujiku[0].gyroX;
	gyro[1] = rokujiku[0].gyroY;
	gyro[2] = rokujiku[0].gyroZ;

	bool flg_judge_move = true; // true:車が走行している, false:車が停止している
	//車が動いているかどうか判定(止まっていればfalse)
	if (gyro[0] > threshold || gyro[0] < -threshold)
	{ //動いている。閾値オーバー
	}
	else if (gyro[1] > threshold || gyro[1] < -threshold)
	{ //同上
	}
	else if (gyro[1] > threshold || gyro[2] < -threshold)
	{ //同上
	}
	else
	{ //車が止まっている(どのgyro軸も閾値を超えていない場合)
		flg_judge_move = false;
	}

	//車の駐停車判定を行う
	if (flg_judge_move)
	{ //車が動いている時
		if (FLAG_car_stop)
		{ //再始動判定を行う
			if (judge_timer + (uint32_t)1000 < millis())
			{ // 1seconds
				judge_timer = millis();
				FLAG_car_stop = false;
			}
		}
		else
		{ //特に何もない
			judge_timer = millis();
		}
	}
	else
	{ // flg_judge_move==false, 車が止まっている時
		if (FLAG_car_stop)
		{ //特に何もない
			judge_timer = millis();
		}
		else
		{ //車停止判定を行う
			if (judge_timer + (uint32_t)390000 < millis())
			{ // 6.5minutes
				judge_timer = millis();
				FLAG_car_stop = true;
			}
		}
	}

	return;
}
/** aa */
void sleep_car_stop()
{
	tca9548a.selectChannel(0);
	while (FLAG_car_stop)
	{
		//本当はちゃんとスリープするべき
		//後回し
		delay(1000); //スリープ中の測定間隔的な
		imu6886.getGyroData(&rokujiku[0].gyroX, &rokujiku[0].gyroY, &rokujiku[0].gyroZ);
		judge_car_move();
	}

	return;
}
//================================================================================

//==============================generate sense cycle==============================
/** Wait until the next sensor detection cycle. */
void sense_cycle()
{
	const uint32_t sense_period = sense_cycle_period(); //[ms] = 1 / sensor_frequency[Hz]
	static uint32_t period_end_tm = 0;									// end time of next period

	while (true)
	{
		if (millis() - period_end_tm > sense_period)
		{
			period_end_tm = millis();
			break;
		}
		// delayMicroseconds(50);
	}

	return;
}
uint32_t sense_cycle_period()
{
	uint32_t s_c_period = 100;
	if (!FLAG_view_data)
	{
		s_c_period -= 50;
	}
	else if (FLAG_THREE_MPU6886)
	{ // if(FLAG_view_data && FLAG_THREE_MPU6886)
		s_c_period += 50;
	}
	if (FLAG_SGP30)
	{
		s_c_period += 50;
	}
	if (FLAG_CONSTANT_CYCLE)
	{
		return CONSTANT_CYCLE;
	}
	else
	{
		return s_c_period;
	}
}
//================================================================================

void test_gps()
{ // use FLAG_test_gps
	while (m5_button() != 2)
	{
		smartDelay(100);
		gpsdata_get();
		M5.Lcd.setCursor(0, 42);
		M5.Lcd.printf("time %09d date %06d", gpsdata.time, gpsdata.date);
		M5.Lcd.setCursor(0, 65);
		M5.Lcd.printf(" %f : %f", gpsdata.lat, gpsdata.lng);
	}
	delay(500);
	return;
}

typedef struct _battery_data
{
	float vbat;
	float ibat;
	double coulomb;
} battery_data_t;
battery_data_t battery_data;
void test_bat_setup()
{
	M5.Axp.begin(); // M5.begin()の前後に入れるべき。用途不明
}
void test_bat_get()
{
	battery_data.vbat = M5.Axp.GetBatVoltage();			// 3.5などの数値を取得
	battery_data.ibat = M5.Axp.GetBatCurrent();			// 68.7などの数値を取得
	battery_data.coulomb = M5.Axp.GetCoulombData(); // mAhが戻ってくる(0-80前後の値)
}
void test_bat_view()
{
	M5.Lcd.setCursor(0, 20);
	M5.Lcd.printf(" vbat   : %2.4f      ", battery_data.vbat);
	M5.Lcd.setCursor(0, 42);
	M5.Lcd.printf(" ibat   : %6.6f      ", battery_data.ibat);
	M5.Lcd.setCursor(0, 65);
	M5.Lcd.printf(" coulomb: %6.6f      ", battery_data.coulomb);
}
