#pragma once
#include <GoSdk/GoSdk.h>
#include <qthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#define RECEIVE_TIMEOUT 20000000
#define INVALID_RANGE_16BIT     ((signed short)0x8000)          // gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data. 
#define DOUBLE_MAX              ((k64f)1.7976931348623157e+308) // 64-bit double - largest positive value.  
#define INVALID_RANGE_DOUBLE    ((k64f)-DOUBLE_MAX)             // floating point value to represent invalid range data.

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)	

namespace HardWare {
	class GocatorCameraHardware :public QThread
	{
		Q_OBJECT
	public:
		GocatorCameraHardware();
		~GocatorCameraHardware();
		int ConnectCamera();
		int DisConnectCamera();
		
		int GetFiledInformation();
		pcl::PointCloud<pcl::PointXYZI> GetCameraCloudXYZI();
		int SetCameraIP(std::string _ip);
	private:
		kAssembly api = kNULL;
		kStatus status;
		GoSystem system = kNULL;
		GoSensor sensor = kNULL;
		GoDataSet dataset = kNULL;
		unsigned int i, j;
		kIpAddress ipAddress;
		std::string sensor_ip = "127.0.0.1";
		pcl::PointCloud<pcl::PointXYZI> camera_cloud_XYZI;
	
	private:
		int GetOnceCameraPicture();
		void ConstructErrorInformation();

	signals:
		void GotPictureSuccessful(bool done);
		void RequestGetOncePicture();
	};
}


