#include "FitPlaneForm.h"
#include <vtkVertexGlyphFilter.h>
namespace MeasruementMethodForm {
	FitPlaneForm::FitPlaneForm(QWidget* parent)
		:QMainWindow(parent), ui(new Ui::FitPlaneForm)
	{
		ui->setupUi(this);
		this->setWindowTitle(QString(u8"Gocator�û�����"));
		
		QThread* gocator_camera_thread = new QThread(this);
		gocator_camera = new HardWare::GocatorCameraHardware;
		gocator_camera->moveToThread(gocator_camera_thread);
		gocator_camera_thread->start();

		//ʹ��connect,���߳�startʱ��������ѭ�������������UI�������Ӱ�죬��������߳��е��ô���ѭ���ĺ�����UI����Ῠ��
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
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud, 255, 0, 0); // ��ɫ (RGB)
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
			//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(other_cloud.makeShared(), 0, 0, 255); // ��ɫ (RGB)
			//viewerviewer->addPointCloud<pcl::PointXYZ>(other_cloud.makeShared(), color3, "other cloud");
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(select_cloud.makeShared(), 0, 255, 0); // ��ɫ (RGB)
			viewerviewer->addPointCloud<pcl::PointXYZ>(select_cloud.makeShared(), color2, "selected cloud");

			while (!viewerviewer->wasStopped()){
				viewerviewer->spinOnce(100);
				QApplication::processEvents();
				//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
				// �������������
			}			//// ���ñ�����ɫ
		});
		
		connect(gocator_camera, &HardWare::GocatorCameraHardware::GotPictureSuccessful,this, [this]() {
			QDir().mkdir("Camera Cloud");
			QString  base_cloud_name= "Camera Cloud/camer_cloud";
			QString save_cloud_name = base_cloud_name;
			int count = 1;
			// ����ļ��Ƿ���ڣ������������Ӻ�׺
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
				msgBox.setWindowTitle("���ӳ�ʱ");
				msgBox.setIcon(QMessageBox::Warning);
				msgBox.setStandardButtons(QMessageBox::Ok);
			}

		});
		
		connect(ui->disconnect_camera, &QPushButton::clicked, this, [this]() {
			gocator_camera->DisConnectCamera();

		});
		// ����Ĭ�ϵ� OpenGL ��ʽ
		QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

		// ���� QVTKOpenGLNativeWidget
		qvtkWidget = new QVTKOpenGLNativeWidget(this);

		// ��ʼ�� VTK ����Ⱦ��
		vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
		renderer = vtkSmartPointer<vtkRenderer>::New();
		renderWindow->AddRenderer(renderer);

		// ����Ⱦ��������Ϊ QVTKOpenGLNativeWidget ����Ⱦ����
		qvtkWidget->setRenderWindow(renderWindow);

		//// �� QVTKOpenGLNativeWidget ��ӵ����ֻ򴰿���
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
		////��ʼ��VTK����Ⱦ����ƽʱ�õıȽ϶���vtkRenderWindow��������QT��Ҫ����vtkGenericOpenGLRenderWindow��ʵ������vtkRenderWindow����һ��
		//vtkGenericOpenGLRenderWindow* renderWindow = vtkGenericOpenGLRenderWindow()::New();
		////����Ⱦ�����뵽VTK�����С�������д��һ�У������ٽ�׼���õ�vtkRenderer���뵽renderWindow��Ҳ�ǿ���ͬ�����ݵ�
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