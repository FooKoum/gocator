#pragma once
#include "ui_FitPlane.h"
#include "qmainwindow.h"
#include <QTextStream>
#include "pcl/point_cloud.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply/ply.h>
#include <qpushbutton.h>
#include <iostream>
#include <qfiledialog.h>
#include <qfile.h>
#include "GocatorCameraHardware.h"
#include <QMessageBox>
#include "vtkGenericOpenGLRenderWindow.h"
#include "QVTKOpenGLNativeWidget.h"
namespace MeasruementMethodForm {
	class FitPlaneForm:public QMainWindow
	{
		Q_OBJECT
	public:
		FitPlaneForm(QWidget* parent = nullptr);
		~FitPlaneForm();

		void displayPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	private:
		Ui::FitPlaneForm*ui;
		HardWare::GocatorCameraHardware* gocator_camera = nullptr;
		QVTKOpenGLNativeWidget* qvtkWidget;
		vtkSmartPointer<vtkRenderer> renderer;
	};
}


