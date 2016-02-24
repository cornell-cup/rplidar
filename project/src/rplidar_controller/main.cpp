/*
 * Copyright (C) 2014  RoboPeak
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
 /*
  *  RoboPeak Lidar System
  *  Simple Data Grabber Demo App
  *
  *  Copyright 2009 - 2014 RoboPeak Team
  *  http://www.robopeak.com
  *
  */

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <algorithm>
#include <windows.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define CONTINUOUS 0
#define ONE_CYCLE 1
#define HISTOGRAM 2

using namespace std;
using namespace rp::standalone::rplidar;

void plot_histogram(rplidar_response_measurement_node_t * nodes, size_t count)
{
	const int BARCOUNT = 75;
	const int MAXBARHEIGHT = 20;
	const float ANGLESCALE = 360.0f / BARCOUNT;

	float histogram[BARCOUNT];
	for (int pos = 0; pos < _countof(histogram); ++pos) {
		histogram[pos] = 0.0f;
	}

	float max_val = 0;
	for (int pos = 0; pos < (int)count; ++pos) {
		int int_deg = (int)((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f / ANGLESCALE);
		if (int_deg >= BARCOUNT) int_deg = 0;
		float cachedd = histogram[int_deg];
		if (cachedd == 0.0f) {
			cachedd = nodes[pos].distance_q2 / 4.0f;
		}
		else {
			cachedd = (nodes[pos].distance_q2 / 4.0f + cachedd) / 2.0f;
		}

		if (cachedd > max_val) max_val = cachedd;
		histogram[int_deg] = cachedd;
	}

	for (int height = 0; height < MAXBARHEIGHT; ++height) {
		float threshold_h = (MAXBARHEIGHT - height - 1) * (max_val / MAXBARHEIGHT);
		for (int xpos = 0; xpos < BARCOUNT; ++xpos) {
			if (histogram[xpos] >= threshold_h) {
				putc('*', stdout);
			}
			else {
				putc(' ', stdout);
			}
		}
		printf("\n");
	}
	for (int xpos = 0; xpos < BARCOUNT; ++xpos) {
		putc('-', stdout);
	}
	printf("\n");
}

u_result capture_and_display(RPlidarDriver * drv, int mode)
{
	u_result ans;

	rplidar_response_measurement_node_t nodes[360 * 2];
	size_t   count = _countof(nodes);

	if (mode != CONTINUOUS) printf("waiting for data...\n");

	// fetch extactly one 0-360 degrees' scan
	ans = drv->grabScanData(nodes, count);
	if (IS_OK(ans) || ans == RESULT_OPERATION_TIMEOUT) {
		drv->ascendScanData(nodes, count);
		string key;
		if (mode == HISTOGRAM) {
			plot_histogram(nodes, count);

			printf("Do you want to see the data? (y/n) ");
			getline(cin, key);
			transform(key.begin(), key.end(), key.begin(), ::tolower);
		}
		if (mode != HISTOGRAM || key == "yes" || key == "y") {
			for (int pos = 0; pos < (int)count; ++pos) {
				printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
					(nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ",
					(nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
					nodes[pos].distance_q2 / 4.0f,
					nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
			}
		}
	}
	else {
		printf("error code: %x\n", ans);
	}

	return ans;
}

int get_data(RPlidarDriver * drv, int mode) {
	// take only one 360 deg scan and display the result as desired
	if (IS_FAIL(capture_and_display(drv, mode))) {
		fprintf(stderr, "Error, cannot grab scan data.\n");
		return -1;
	}

	return 0;
}

int get_device_info(RPlidarDriver * drv) {
	u_result op_result;
	rplidar_response_device_info_t devinfo;

	op_result = drv->getDeviceInfo(devinfo);

	if (IS_FAIL(op_result)) {
		if (op_result == RESULT_OPERATION_TIMEOUT) {
			// you can check the detailed failure reason
			fprintf(stderr, "Error, operation time out.\n");
		}
		else {
			fprintf(stderr, "Error, unexpected error, code: %x\n", op_result);
			// other unexpected result
		}
		return -1;
	}

	// print out the device serial number, firmware and hardware version number..
	printf("RPLIDAR S/N: ");
	for (int pos = 0; pos < 16; ++pos) {
		printf("%02X", devinfo.serialnum[pos]);
	}

	printf("\n"
		"Firmware Ver: %d.%02d\n"
		"Hardware Rev: %d\n"
		, devinfo.firmware_version >> 8
		, devinfo.firmware_version & 0xFF
		, (int)devinfo.hardware_version);

	return 0;
}

int check_device_health(RPlidarDriver * drv) {
	u_result op_result;
	rplidar_response_device_health_t healthinfo;

	op_result = drv->getHealth(healthinfo);
	if (IS_OK(op_result)) {
		printf("RPLidar health status : ");
		switch (healthinfo.status) {
		case RPLIDAR_STATUS_OK:
			printf("OK.");
			break;
		case RPLIDAR_STATUS_WARNING:
			printf("Warning.");
			break;
		case RPLIDAR_STATUS_ERROR:
			printf("Error.");
			break;
		}
		printf(" (errorcode: %d)\n", healthinfo.error_code);
	}
	else {
		fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
		return -1;
	}

	if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
		fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
		// Can reset lidar through software. Just exit for now
		//drv->reset();
		return -1;
	}

	return 0;
}

int main(int argc, const char * argv[]) {
	const char * opt_com_path = NULL;
	_u32         opt_com_baudrate = 115200;
	string		 response;
	string		 comresponse;

get_inputs:
	// read serial port from user...
	if (argc > 1) {
		opt_com_path = argv[1];
	}
	else {
		cout << "Which com port? (default : com3)\n" << flush;
		getline(cin, comresponse);
		if (comresponse.length() > 0) {
			cout << "Using " << comresponse << "\n\n" << flush;
			opt_com_path = comresponse.c_str();
		}
		else {
			#ifdef _WIN32
				// use default com port
				opt_com_path = "\\\\.\\com3";
			#else
				opt_com_path = "/dev/ttyUSB0";
			#endif

			cout << "Using default\n\n" << flush;
		}
	}

	// read baud rate from user...
	if (argc > 2) {
		opt_com_baudrate = strtoul(argv[2], NULL, 10);
	}
	else {
		cout << "What baudrate? (default : 115200)\n" << flush;
		getline(cin, response);
		if (response.length() > 0) {
			cout << "Using baudrate of " << response << "\n\n" << flush;
			opt_com_baudrate = strtoul(response.c_str(), NULL, 10);
		}
		else {
			// Default is 115200
			cout << "Using default\n\n" << flush;
		}
	}

	// create the driver instance
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

	if (!drv) {
		fprintf(stderr, "insufficent memory, exiting\n");
		exit(-2);
	}

	// try to connect
	if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
		fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n\n", opt_com_path);
		goto get_inputs;
	}

	// retrieve the device info
	if (get_device_info(drv) < 0)
		goto on_finished;

	// check the device health
	if (check_device_health(drv) < 0)
		goto on_finished;

	// start the lidar scanning
	if (IS_FAIL(drv->startScan( /* true */))) // you can force rplidar to perform scan operation regardless whether the motor is rotating
	{
		fprintf(stderr, "Error, cannot start the scan operation.\n");
		goto on_finished;
	}

	// Now do the important data grabbing stuff
	cout << "\n" << flush;
	while (1) {
		cout << "What would you like? (continuous; one cycle; histogram; exit)\n" << flush;
		while (1){
			getline(cin, response);
			transform(response.begin(), response.end(), response.begin(), ::tolower);
			if (response == "continuous"){
				while (1){
					if (get_data(drv, CONTINUOUS) < 0)
						goto on_finished;
					// Very funky toggling
					if (GetAsyncKeyState(VK_ESCAPE))
						break;
				}
				break;
			}
			else if (response == "one cycle"){
				if (get_data(drv, ONE_CYCLE) < 0)
					goto on_finished;
				break;
			}
			else if (response == "histogram"){
				if (get_data(drv, HISTOGRAM) < 0)
					goto on_finished;
				break;
			}
			else if (response == "exit"){
				goto on_finished;
			}
			else{
				cout << "Response not valid. Inputs are: continuous; one cycle; histogram\n" << flush;
			}
		}
		cout << "\n" << flush;
	}

	// Exit
on_finished:
	cout << "Press enter to exit.\n" << flush;
	int k = getchar();

	RPlidarDriver::DisposeDriver(drv);
	return 0;
}
