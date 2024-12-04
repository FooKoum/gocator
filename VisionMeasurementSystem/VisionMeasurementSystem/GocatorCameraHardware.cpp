#include "GocatorCameraHardware.h"
typedef struct
{
    double x;   // x-coordinate in engineering units (mm) - position along laser line
    double y;   // y-coordinate in engineering units (mm) - position along the direction of travel
    double z;   // z-coordinate in engineering units (mm) - height (at the given x position)
    unsigned char intensity;
}ProfilePoint;

HardWare::GocatorCameraHardware::GocatorCameraHardware()
{
    connect(this, &HardWare::GocatorCameraHardware::RequestGetOncePicture, this, &HardWare::GocatorCameraHardware::GetOnceCameraPicture, Qt::QueuedConnection);

}

HardWare::GocatorCameraHardware::~GocatorCameraHardware()
{
    GoDestroy(system);
    GoDestroy(api);

    //printf("Press any key to continue...\n");
}

int HardWare::GocatorCameraHardware::ConnectCamera()
{
    // construct Gocator API Library
    if ((status = GoSdk_Construct(&api)) != kOK)
    {
        printf("Error: GoSdk_Construct:%d\n", status);
        return -1;
    }

    // construct GoSystem object
    if ((status = GoSystem_Construct(&system, kNULL)) != kOK)
    {
        printf("Error: GoSystem_Construct:%d\n", status);
        return -1;
    }

    // Parse IP address into address data structure
    kIpAddress_Parse(&ipAddress, sensor_ip.c_str());
    if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK)
    {
        printf("Error: GoSystem_FindSensor:%d\n", status);
        return -1;
    }

    // create connection to GoSensor object
    if ((status = GoSensor_Connect(sensor)) != kOK)
    {
        printf("Error: GoSensor_Connect:%d\n", status);
        return -1 ;
    }
    // enable sensor data channel
    if ((status = GoSystem_EnableData(system, kTRUE)) != kOK)
    {
        printf("Error: GoSensor_EnableData:%d\n", status);
        return -1;
    }
    // start Gocator sensor
    if ((status = GoSystem_Start(system)) != kOK)
    {
        printf("Error: GoSystem_Start:%d\n", status);
        return -1 ;
    }

    printf("Waiting for Whole Part data...\n\n");
	return 0;
}

int HardWare::GocatorCameraHardware::DisConnectCamera()
{
    // obtain GoSensor object by sensor IP address
    if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK)
    {
        printf("Error: GoSystem_FindSensor:%d\n", status);
        return -2;
    }
    if (GoSystem_Disconnect(system)) {
        printf("Error: GoSystem_DisconnectSensor:%d\n", status);
        return -2;
    }
    printf("Successful: GoSystem_DisconnectSensor:%d\n", status);
    return 0;
}

int HardWare::GocatorCameraHardware::GetOnceCameraPicture()
{
    if (GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK)
    {
        short int* height_map_memory = NULL;
        unsigned char* intensity_image_memory = NULL;
        ProfilePoint** surfaceBuffer = NULL;
        k32u surfaceBufferHeight = 0;

        printf("Dataset count: %u\n", GoDataSet_Count(dataset));

        // each result can have multiple data items
        // loop through all items in result message
        for (i = 0; i < GoDataSet_Count(dataset); ++i)
        {
            GoDataMsg dataObj = GoDataSet_At(dataset, i);

            switch (GoDataMsg_Type(dataObj))
            {
            case GO_DATA_MESSAGE_TYPE_STAMP:
            {
                GoStampMsg stampMsg = dataObj;
                printf("  Stamp Message batch count: %u\n\n", GoStampMsg_Count(stampMsg));

                for (j = 0; j < GoStampMsg_Count(stampMsg); j++)
                {
                    GoStamp* stamp = GoStampMsg_At(stampMsg, j);
                    printf("  Timestamp: %llu\n", stamp->timestamp);
                    printf("  Encoder position at leading edge: %lld\n", stamp->encoder);
                    printf("  Frame index: %llu\n", stamp->frameIndex);
                }
            }
            break;
            case GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE:
            {
                GoSurfaceMsg surfaceMsg = dataObj;
                unsigned int rowIdx, colIdx;

                double XResolution = NM_TO_MM(GoSurfaceMsg_XResolution(surfaceMsg));
                double YResolution = NM_TO_MM(GoSurfaceMsg_YResolution(surfaceMsg));
                double ZResolution = NM_TO_MM(GoSurfaceMsg_ZResolution(surfaceMsg));
                double XOffset = UM_TO_MM(GoSurfaceMsg_XOffset(surfaceMsg));
                double YOffset = UM_TO_MM(GoSurfaceMsg_YOffset(surfaceMsg));
                double ZOffset = UM_TO_MM(GoSurfaceMsg_ZOffset(surfaceMsg));

                printf("  Surface data width: %ld\n", GoSurfaceMsg_Width(surfaceMsg));
                printf("  Surface data length: %ld\n", GoSurfaceMsg_Length(surfaceMsg));

                //allocate memory if needed
                if (surfaceBuffer == NULL)
                {
                    surfaceBuffer = static_cast<ProfilePoint**>(malloc(GoSurfaceMsg_Length(surfaceMsg) * sizeof(ProfilePoint*)));

                    for (j = 0; j < GoSurfaceMsg_Length(surfaceMsg); j++)
                    {
                        surfaceBuffer[j] = static_cast<ProfilePoint*>(malloc(GoSurfaceMsg_Width(surfaceMsg) * sizeof(ProfilePoint)));
                    }

                    surfaceBufferHeight = GoSurfaceMsg_Length(surfaceMsg);
                }
                camera_cloud_XYZI.clear();
                for (rowIdx = 0; rowIdx < GoSurfaceMsg_Length(surfaceMsg); rowIdx++)
                {
                    k16s* data = GoSurfaceMsg_RowAt(surfaceMsg, rowIdx);
                    for (colIdx = 0; colIdx < GoSurfaceMsg_Width(surfaceMsg); colIdx++)
                    {
                        pcl::PointXYZI point;
                        // gocator transmits range data as 16-bit signed integers
                        // to translate 16-bit range data to engineering units, the calculation for each point is: 
                        //          X: XOffset + columnIndex * XResolution 
                        //          Y: YOffset + rowIndex * YResolution
                        //          Z: ZOffset + height_map[rowIndex][columnIndex] * ZResolution

                        surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * colIdx;
                        surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * rowIdx;
                        point.x= XOffset + XResolution * colIdx;
                        point.y= YOffset + YResolution * rowIdx;

                        if (data[colIdx] != INVALID_RANGE_16BIT)
                        {
                            surfaceBuffer[rowIdx][colIdx].z = ZOffset + ZResolution * data[colIdx];
                            point.z = ZOffset + ZResolution * data[colIdx];
                            //point.intensity = data[colIdx];
                            camera_cloud_XYZI.push_back(point);
                        }
                        else
                        {
                            surfaceBuffer[rowIdx][colIdx].z = INVALID_RANGE_DOUBLE;
                           // point.z = INVALID_RANGE_DOUBLE;
                        }
                    }
                }
            }
            break;
            case GO_DATA_MESSAGE_TYPE_SURFACE_POINT_CLOUD:
            {
                GoSurfacePointCloudMsg surfacePointCloudMsg = dataObj;
                unsigned int rowIdx, colIdx;

                double XResolution = NM_TO_MM(GoSurfacePointCloudMsg_XResolution(surfacePointCloudMsg));
                double YResolution = NM_TO_MM(GoSurfacePointCloudMsg_YResolution(surfacePointCloudMsg));
                double ZResolution = NM_TO_MM(GoSurfacePointCloudMsg_ZResolution(surfacePointCloudMsg));
                double XOffset = UM_TO_MM(GoSurfacePointCloudMsg_XOffset(surfacePointCloudMsg));
                double YOffset = UM_TO_MM(GoSurfacePointCloudMsg_YOffset(surfacePointCloudMsg));
                double ZOffset = UM_TO_MM(GoSurfacePointCloudMsg_ZOffset(surfacePointCloudMsg));

                printf("  Surface Point Cloud data width: %ld\n", GoSurfacePointCloudMsg_Width(surfacePointCloudMsg));
                printf("  Surface Point Cloud data length: %ld\n", GoSurfacePointCloudMsg_Length(surfacePointCloudMsg));

                //allocate memory if needed
                if (surfaceBuffer == NULL)
                {
                    surfaceBuffer = static_cast<ProfilePoint**>(malloc(GoSurfacePointCloudMsg_Length(surfacePointCloudMsg) * sizeof(ProfilePoint*)));

                    for (j = 0; j < GoSurfacePointCloudMsg_Length(surfacePointCloudMsg); j++)
                    {
                        surfaceBuffer[j] = static_cast<ProfilePoint*>(malloc(GoSurfacePointCloudMsg_Width(surfacePointCloudMsg) * sizeof(ProfilePoint)));
                    }

                    surfaceBufferHeight = GoSurfacePointCloudMsg_Length(surfacePointCloudMsg);
                }

                for (rowIdx = 0; rowIdx < GoSurfacePointCloudMsg_Length(surfacePointCloudMsg); rowIdx++)
                {
                    kPoint3d16s* data = GoSurfacePointCloudMsg_RowAt(surfacePointCloudMsg, rowIdx);

                    for (colIdx = 0; colIdx < GoSurfacePointCloudMsg_Width(surfacePointCloudMsg); colIdx++)
                    {
                        surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * data[colIdx].x;
                        surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * data[rowIdx].y;

                        if (data[colIdx].z != INVALID_RANGE_16BIT)
                        {
                            surfaceBuffer[rowIdx][colIdx].z = ZOffset + ZResolution * data[colIdx].z;
                        }
                        else
                        {
                            surfaceBuffer[rowIdx][colIdx].z = INVALID_RANGE_DOUBLE;
                        }
                    }
                }
            }
            break;
            case GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY:
            {
                GoSurfaceIntensityMsg surfaceIntMsg = dataObj;
                unsigned int rowIdx, colIdx;
                double XResolution = NM_TO_MM(GoSurfaceIntensityMsg_XResolution(surfaceIntMsg));
                double YResolution = NM_TO_MM(GoSurfaceIntensityMsg_YResolution(surfaceIntMsg));
                double XOffset = UM_TO_MM(GoSurfaceIntensityMsg_XOffset(surfaceIntMsg));
                double YOffset = UM_TO_MM(GoSurfaceIntensityMsg_YOffset(surfaceIntMsg));

                printf("  Surface intensity width: %ld\n", GoSurfaceIntensityMsg_Width(surfaceIntMsg));
                printf("  Surface intensity height: %ld\n", GoSurfaceIntensityMsg_Length(surfaceIntMsg));

                //allocate memory if needed
                if (surfaceBuffer == NULL)
                {
                    surfaceBuffer = static_cast<ProfilePoint**>(malloc(GoSurfaceIntensityMsg_Length(surfaceIntMsg) * sizeof(ProfilePoint*)));

                    for (j = 0; j < GoSurfaceIntensityMsg_Length(surfaceIntMsg); j++)
                    {
                        surfaceBuffer[j] = static_cast<ProfilePoint*>(malloc(GoSurfaceIntensityMsg_Width(surfaceIntMsg) * sizeof(ProfilePoint)));
                    }

                    surfaceBufferHeight = GoSurfaceIntensityMsg_Length(surfaceIntMsg);
                }

                for (rowIdx = 0; rowIdx < GoSurfaceIntensityMsg_Length(surfaceIntMsg); rowIdx++)
                {
                    k8u* data = GoSurfaceIntensityMsg_RowAt(surfaceIntMsg, rowIdx);

                    // gocator transmits intensity data as an 8-bit grayscale image of identical width and height as the corresponding height map
                    for (colIdx = 0; colIdx < GoSurfaceIntensityMsg_Width(surfaceIntMsg); colIdx++)
                    {
                        surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * colIdx;
                        surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * rowIdx;
                        surfaceBuffer[rowIdx][colIdx].intensity = data[colIdx];
                    }

                }
            }
            break;
            }
            //GoSurfacePointCloudMsg surfacePointCloudMsg = dataObj;
            ////GoSurfaceIntensityMsg surfaceIntMsg = dataObj;
            //unsigned int rowIdx, colIdx;
            //for (rowIdx = 0; rowIdx < GoSurfacePointCloudMsg_Length(surfacePointCloudMsg); rowIdx++)
            //{
            //    kPoint3d16s* data = GoSurfacePointCloudMsg_RowAt(surfacePointCloudMsg, rowIdx);

            //    for (colIdx = 0; colIdx < GoSurfacePointCloudMsg_Width(surfacePointCloudMsg); colIdx++)
            //    {

            //    }
            //}
       
            //// ½« ProfilePoint ×ª»»Îª pcl::PointCloud
            //for (size_t i = 0; i < rows; ++i) {
            //    for (size_t j = 0; j < cols; ++j) {
            //        ProfilePoint& p = surfaceBuffer[i][j];
            //        pcl::PointXYZI point;
            //        point.x = static_cast<float>(p.x);
            //        point.y = static_cast<float>(p.y);
            //        point.z = static_cast<float>(p.z);
            //        point.intensity = static_cast<float>(p.intensity);

            //        camera_cloud_XYZI.push_back(point);
            //    }
            //}
            //camera_cloud_XYZI.width = static_cast<uint32_t>(rows * cols);
            //camera_cloud_XYZI.height = 1;
            //camera_cloud_XYZI.is_dense = true;
        }

        GoDestroy(dataset);
        //free memory arrays
        if (surfaceBuffer)
        {
            unsigned int i;
            for (i = 0; i < surfaceBufferHeight; i++)
            {
                free(surfaceBuffer[i]);
            }
        }
        emit GotPictureSuccessful(true);

        return 0;

    }
    else
    {
        emit GotPictureSuccessful(false);
        return -2;
        printf("Error: No data received during the waiting period\n");
    }
    // stop Gocator sensor
    if ((status = GoSystem_Stop(system)) != kOK)
    {
        emit GotPictureSuccessful(false);
        printf("Error: GoSystem_Stop:%d\n", status);
        return -2;
    }
}

int HardWare::GocatorCameraHardware::GetFiledInformation()
{
    return 0;
}

pcl::PointCloud<pcl::PointXYZI> HardWare::GocatorCameraHardware::GetCameraCloudXYZI()
{
    return this->camera_cloud_XYZI;
}

int HardWare::GocatorCameraHardware::SetCameraIP(std::string _ip)
{
    this->sensor_ip = _ip;
    return 0;
}


void HardWare::GocatorCameraHardware::ConstructErrorInformation()
{
}


