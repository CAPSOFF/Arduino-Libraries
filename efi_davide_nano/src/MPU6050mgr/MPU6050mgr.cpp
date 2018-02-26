#ifndef MPU6050mgr_cpp
#define MPU6050mgr_cpp

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050mgr.h"
#include "../COMMmgr/COMMmgr.h" // Communication manager (for checksum calculation)
#include "../compile_options.h"

#define MPU6050_I2C_ADDRESS 0x68  // I2C address of the MPU-6050
#define MPU6050_TASK_TIME_TARGET (uint16_t)20 // Task time target, in ms
#define MPU6050_TASK_TIME_MULT (uint8_t)5 // Multiplier for polling time [the sampling time is the multiplication of this value and polling time]
#define MPU6050_SIG_FILT_CNST (int16_t)5000 // Filtering constant for newly received signals (2^15 means = 1 = no filtering)


// Copies data from one array to an other
void copy_uint8_array(uint8_t* array_src_data, uint8_t array_src_start, uint8_t* array_dst_data, uint8_t array_dst_start, uint8_t array_length){
	for (uint8_t i=0; i<array_length; i++){
		array_dst_data[array_dst_start+i] = array_src_data[array_src_start+i]; // copy start to destination byte
	}
}

// Reads multiple bytes from MPU6050
uint8_t MPU6050_read(uint8_t start, uint8_t *buffer, uint8_t size){
	Wire.beginTransmission(MPU6050_I2C_ADDRESS);
	Wire.write(start);
	Wire.endTransmission(false); // hold the I2C-bus
	Wire.requestFrom(MPU6050_I2C_ADDRESS, (int)size, true);
	uint8_t i = 0;
	while(Wire.available() && (i<size)){
		buffer[i++]=Wire.read();
	}
	if (i != size) return 1; // Did not read enough bytes
	return 0; // no error
}

// Writes multiple bytes to MPU6050
uint8_t MPU6050_write(uint8_t start, uint8_t *pData, uint8_t size){
	Wire.beginTransmission(MPU6050_I2C_ADDRESS);
	if (Wire.write(start) != 1) return 1; // write the start address
	if (Wire.write(pData, size) != 1) return 1; // write data bytes
	if (Wire.endTransmission(true) != 0) return 1; // release the I2C-bus
	return 0; // no error
}

// Writes single byte to MPU6050
uint8_t MPU6050_write_reg(uint8_t reg, uint8_t data){
	return MPU6050_write(reg, &data, 1);
}

// Initializes MPU6050
void MPU6050mgr_class::begin(){

#if (MPU6050_PRESENT == 1)
	// Initializes Wire communication (I2C)
	Wire.begin();
	Wire.beginTransmission(MPU6050_I2C_ADDRESS);
	Wire.write(0x6B);  // PWR_MGMT_1 register
	Wire.write(0);     // set to zero (wakes up the MPU-6050)
	Wire.endTransmission(true);
	
	// Clean acceleration and gyroscope
	for (uint8_t i=0; i< 6; i++){
		ag[i]=0;
	}
#endif
	
}

// Returns the address of the next buffer to be written
/*uint8_t MPU6050mgr_class::next_buffer_index(){
	uint8_t item_last_write_tmp = item_last_write + 1; // temp index of the buffer to be written
	if (item_last_write_tmp == MPU6050_BUFFERS_NUMBER) item_last_write_tmp=0; // rollover
	return item_last_write_tmp;
}*/

// Filters the signal using present value and old value
int16_t MPU6050mgr_class::filter_signal(int16_t sig_old, int16_t sig_new){
	
	int32_t acc = 1 << 14; // accumulator for MACs // load rounding constant
	acc += ((int32_t)sig_new * (int32_t)MPU6050_SIG_FILT_CNST);
	acc += ((int32_t)sig_old * ((int32_t)32768 - (int32_t)MPU6050_SIG_FILT_CNST));
	
	// saturate the result 
	if ( acc > 0x3fffffff ) {
		acc = 0x3fffffff;
	} else if ( acc < -0x40000000 ) { 
		acc = -0x40000000; 
	} 
	
	return ((int16_t)(acc >> 15)); // convert from Q30 to Q15 
}

// Reads data from MPU-6050, filters the signals, and saves the signals in the temporary buffer
uint8_t MPU6050mgr_class::read_data(){
	
	// Receives data from MPU-6050
	uint8_t IMU_buffer_recv[14]; // temporary buffer for data receive
	if (MPU6050_read(0x3B, IMU_buffer_recv, 14)) return 1; // Not OK (data could not be received correctly)
	
	// Transforms raw data into integers, and filters
	for (uint8_t i=0; i<6; i++){ // 6 signals (Acceleration and Gyroscope)
		uint8_t j = 2*i; // for Acceleration data
		if (j >= 6) j = j + 2; // for Gyroscope data, starting from index 8 on MPU packet
		int16_t int_old = ag[i]; // Old value
		int16_t int_new = (IMU_buffer_recv[j]<<8) | (IMU_buffer_recv[j+1]); // Acceleration or Gyroscope value
		ag[i] = filter_signal(int_old, int_new); // Filtered signal
	}
	copy_uint8_array(IMU_buffer_recv, 6, temperature, 0, 2); // Temperature (2 bytes) | No need to be filtered
	//copy_uint8_array(IMU_buffer_recv, 0, data_buffer[buffer_index].IMU_data, 0, 6); // Acceleration data (6 bytes)
	//copy_uint8_array(IMU_buffer_recv, 8, data_buffer[buffer_index].IMU_data, 6, 6); // Gyroscope data (6 bytes)
	
	/*ax=(MPU6050mgr_recv_buffer[i++]<<8)|(MPU6050mgr_recv_buffer[i++]);  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
	ay=(MPU6050mgr_recv_buffer[i++]<<8)|(MPU6050mgr_recv_buffer[i++]);  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	az=(MPU6050mgr_recv_buffer[i++]<<8)|(MPU6050mgr_recv_buffer[i++]);  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	temperature=(MPU6050mgr_recv_buffer[i++]<<8)|(MPU6050mgr_recv_buffer[i++]);  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	gx=(MPU6050mgr_recv_buffer[i++]<<8)|(MPU6050mgr_recv_buffer[i++]);  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	gy=(MPU6050mgr_recv_buffer[i++]<<8)|(MPU6050mgr_recv_buffer[i++]);  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	gz=(MPU6050mgr_recv_buffer[i++]<<8)|(MPU6050mgr_recv_buffer[i++]);  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)*/
	
	return 0;
	
}

// Manages the requests coming from outside (Main Loop and Yield)
void MPU6050mgr_class::manager(unsigned long time_now_ms){
	
#if (MPU6050_PRESENT == 1)

	uint32_t time_now_ms_32bit = time_now_ms; // 32 bits
	uint16_t time_now_ms_16bit = (uint16_t)(time_now_ms & 0x0000FFFF); // 16 bits

	// Checks if enough time has expired. In positive case, read data from IMU, and stores it into the buffer
	if ((time_now_ms_16bit - polling_time_last) >= MPU6050_TASK_TIME_TARGET){
		polling_time_last += MPU6050_TASK_TIME_TARGET;
		read_data(); // Reads data from MPU and filters it
		task_multiplier_cnt++;
		if (task_multiplier_cnt >= MPU6050_TASK_TIME_MULT) { // I have polled many times, I can store now
			task_multiplier_cnt = 0; // rollover
			//item_last_write = next_buffer_index(); // Officialize that the buffer is filled
			item_last_write++; // One more buffer item added
			if (item_last_write == MPU6050_BUFFERS_NUMBER) item_last_write=0; // rollover
			copy_uint8_array((uint8_t*)(&ag[0]), 0, data_buffer[item_last_write].IMU_data, 0, 12); // Copy acceleration and gyroscope
			copy_uint8_array((uint8_t*)(&time_now_ms_32bit), 0, data_buffer[item_last_write].polling_time_stamp, 0, 4); // Add time stamp
			/*for (uint8_t i=0; i<6; i++){ // 6 signals (Acceleration and Gyroscope)
				data_buffer[item_last_write].IMU_data[2*i] = ((*((uint16_t*)(&ag[i]))) & 0xFF00) >> 8; // High Byte
				data_buffer[item_last_write].IMU_data[2*i + 1] = ((*((uint16_t*)(&ag[i]))) & 0x00FF); // Low Byte
				//Serial.print(ag[i]);
				//Serial.print(",");
			}*/
			//Serial.println();
			//Serial.write(data_buffer[item_last_write].IMU_data, MPU6050_BUFFER_IMU_SIZE);
		}
	}

#endif	

}

// Flushes the buffer (so that there is no data to write)
void MPU6050mgr_class::flush_buffer(){
	item_last_read = item_last_write;
}

// Returns the number of buffer packets available to write on SD
uint8_t MPU6050mgr_class::buffer_data_available(){
	if (item_last_write >= item_last_read) return (item_last_write - item_last_read); // 0 or higher
	return (MPU6050_BUFFERS_NUMBER + item_last_write - item_last_read); // when there is a rollover
}

// Prepares the packet for SD writing, and returns 1 in case a buffer was filled
uint8_t MPU6050mgr_class::prepare_SD_packet(uint8_t* temp_data_buffer_SD){
	if (!MPU6050mgr.buffer_data_available()) return 0; // No new data available
	temp_data_buffer_SD[0]=0x49; // "I" for IMU | Byte 0
	item_last_read++; // One more item will be soon written to SD
	if (item_last_read == MPU6050_BUFFERS_NUMBER) item_last_read = 0; // rollover
	copy_uint8_array(data_buffer[item_last_read].polling_time_stamp, 0, temp_data_buffer_SD, 1, 4); // Time Stamp, in ms (32 bits) | Bytes 1-4
	copy_uint8_array(data_buffer[item_last_read].IMU_data, 0, temp_data_buffer_SD, 5, 12); // IMU data | Bytes 5-16
	copy_uint8_array(temperature, 0, temp_data_buffer_SD, 17, 2); // Temperature | Bytes 17-18
	uint16_t CK_SUM = COMM_calculate_checksum(temp_data_buffer_SD, 0, 19);
    temp_data_buffer_SD[19] = (uint8_t)(CK_SUM >> 8); // Checksum | Byte 19
    temp_data_buffer_SD[20] = (uint8_t)(CK_SUM & 0xFF); // Checksum | Byte 20
	return 1;
}

// Prepares the packet for COMM send (Serial Service protocol)
void MPU6050mgr_class::prepare_COMM_packet(uint8_t* temp_data_buffer_COMM){
	copy_uint8_array(data_buffer[item_last_write].polling_time_stamp, 0, temp_data_buffer_COMM, 0, 4); // Time Stamp, in ms (32 bits)
	copy_uint8_array(data_buffer[item_last_write].IMU_data, 0, temp_data_buffer_COMM, 4, 12); // IMU data
}

// Sends ASCII data on Serial
void MPU6050mgr_class::send_ASCII_data(){
	for (uint8_t i=0; i<6; i++){
		Serial.print(ag[i]);
		Serial.print(F(","));
	}
	Serial.write('\r');
	Serial.write('\n');
}

MPU6050mgr_class MPU6050mgr;

#endif