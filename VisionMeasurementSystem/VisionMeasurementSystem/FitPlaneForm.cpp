#include "FitPlaneForm.h"
#include <vtkVertexGlyphFilter.h>
namespace MeasruementMethodForm {
	FitPlaneForm::FitPlaneForm(QWidget* parent)
		:QMainWindow(parent), ui(new Ui::FitPlaneForm)
	{
		ui->setupUi(this);
		this->setWindowTitle(QString(u8"Gocator用户界面"));
		
		QThread* gocator_camera_thread = new QThread(this);
		gocator_camera = new HardWare::GocatorCameraHardware;
		gocator_camera->moveToThread(gocator_camera_thread);
		gocator_camera_thread->start();

		//使用connect,当线程start时，运行死循环，这样不会对UI界面产生影响，如果在主线程中调用带死循环的函数，UI界面会卡死
		connect(ui->start_get_image, &QPushButton::clicked, gocator_camera, &HardWare::GocatorCameraHardware::RequestGetOncePicture);

		connect(ui->select_points_cloud, &QPushButton::clicked, this, [this]() {
			QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
				QDir::homePath(), tr("Text Files (*.ply);;All Files (*)"));
			pcl::visualization::PCLVisualizer::Ptr viewerviewer(new pcl::visualization::PCLVisualizer("3D Viewer", true));
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
			//pcl::io::loadPCDFile(fileName.toStdString(), *cloud);
			pcl::io::loadPLYFile(fileName.toStdString(), *cloud);
			this->displayPointCloud(cloud);

			double A = -0.0512;
			double B = -0.012;
			double C = 0.9986;
			double D = -1 * (0.3587 * -0.0512 + -0.012 * -42.08 + 0.9986 * -11.1403);
			Eigen::Matrix<double, 4, 1> plane_param = { A,B,C, D };

			double denominator = std::sqrt(A * A + B * B + C * C);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud, 255, 0, 0); // 红色 (RGB)
			viewerviewer->addPointCloud<pcl::PointXYZ>(cloud, color1, "sample cloud");
			pcl::PointCloud<pcl::PointXYZ> select_cloud;
			pcl::PointCloud<pcl::PointXYZ> other_cloud;
			for (const auto& point : *cloud) {
				double numerator = std::abs(A * point.x + B * point.y + C * point.z + D);
				double distance = numerator / denominator;
				if (distance < 0.1 && distance>0) {
					select_cloud.push_back(point);
				}
				else {
					other_cloud.push_back(point);
				}

			}
			//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(other_cloud.makeShared(), 0, 0, 255); // 红色 (RGB)
			//viewerviewer->addPointCloud<pcl::PointXYZ>(other_cloud.makeShared(), color3, "other cloud");
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(select_cloud.makeShared(), 0, 255, 0); // 红色 (RGB)
			viewerviewer->addPointCloud<pcl::PointXYZ>(select_cloud.makeShared(), color2, "selected cloud");

			while (!viewerviewer->wasStopped()){
				viewerviewer->spinOnce(100);
				QApplication::processEvents();
				//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
				// 可添加其他操作
			}			//// 设置背景颜色
		});
		
		connect(gocator_camera, &HardWare::GocatorCameraHardware::GotPictureSuccessful,this, [this]() {
			QDir().mkdir("Camera Cloud");
			QString  base_cloud_name= "Camera Cloud/camer_cloud";
			QString save_cloud_name = base_cloud_name;
			int count = 1;
			// 检查文件是否存在，若存在则增加后缀
			while (QFile::exists(save_cloud_name + ".ply")) {
				save_cloud_name = QString("%1_%2").arg(base_cloud_name).arg(count);
				count++;
			}
			pcl::PointCloud<pcl::PointXYZI> camer_cloud = gocator_camera->GetCameraCloudXYZI();
			pcl::io::savePLYFile(save_cloud_name.toStdString() + ".ply", camer_cloud);
		});
		
		
		

		connect(ui->connect_camera, &QPushButton::clicked, this, [this](){
			std::string sensor_ip = ui->lineEdit->text().toStdString();
			gocator_camera->SetCameraIP(sensor_ip);
			if (gocator_camera->ConnectCamera() != 0) {
				QMessageBox msgBox(this);
				msgBox.setWindowTitle("连接超时");
				msgBox.setIcon(QMessageBox::Warning);
				msgBox.setStandardButtons(QMessageBox::Ok);
			}

		});
		
		connect(ui->disconnect_camera, &QPushButton::clicked, this, [this]() {
			gocator_camera->DisConnectCamera();

		});
		// 设置默认的 OpenGL 格式
		QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

		// 创建 QVTKOpenGLNativeWidget
		qvtkWidget = new QVTKOpenGLNativeWidget(this);

		// 初始化 VTK 的渲染器
		vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
		renderer = vtkSmartPointer<vtkRenderer>::New();
		renderWindow->AddRenderer(renderer);

		// 将渲染窗口设置为 QVTKOpenGLNativeWidget 的渲染窗口
		qvtkWidget->setRenderWindow(renderWindow);

		//// 将 QVTKOpenGLNativeWidget 添加到布局或窗口中
		//QWidget* container = new QWidget();
		//QVBoxLayout* layout = new QVBoxLayout(container);
		//layout->addWidget(qvtkWidget);
		//layout->setContentsMargins(0, 0, 0, 0);
		//container->setLayout(layout);
	
		//ui->tableWidget->setCellWidget(0, 0, qvtkWidget);
		ui->gridLayout_2->addWidget(qvtkWidget);

		//ui->openGLWidget->setRenderWindow(renderWindow);
		//QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
		//QVTKOpenGLNativeWidget* qvtkWidget = new QVTKOpenGLNativeWidget();
		////初始化VTK的渲染器，平时用的比较多是vtkRenderWindow，但是在QT中要改用vtkGenericOpenGLRenderWindow，实质上与vtkRenderWindow功能一致
		//vtkGenericOpenGLRenderWindow* renderWindow = vtkGenericOpenGLRenderWindow()::New();
		////将渲染器加入到VTK窗口中。可以先写这一行，后续再将准备好的vtkRenderer加入到renderWindow中也是可以同步数据的
		//qvtkWidget->setRenderWindow(renderWindow);
		//ui->gridLayout_2->addWidget(qvtkWidget);
		////this->setLayout(displayGrid);

	}
	FitPlaneForm::~FitPlaneForm() {
		//delete ui;
	}
	void FitPlaneForm::displayPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
		//renderer->RemoveAllViewProps();

		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		for (const auto& point : *cloud) {
			points->InsertNextPoint(point.x, point.y, point.z);
		}

		vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
		polyData->SetPoints(points);

		vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
		vertexFilter->SetInputData(polyData);
		vertexFilter->Update();

		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputData(vertexFilter->GetOutput());

		vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
		actor->SetMapper(mapper);
		
		renderer->AddActor(actor);
		renderer->ResetCamera();
		qvtkWidget->renderWindow()->Render();
	}
}