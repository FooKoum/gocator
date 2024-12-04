#include "MainForm.h" 
#include <QTextStream>
#include "pcl/point_cloud.h"
#include "pcl/visualization/pcl_visualizer.h"
//#include <vtkRenderWindow.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply/ply.h>
namespace MeasureMainForm {
	MainForm::MainForm(QWidget* parent) :
		QMainWindow(parent), ui(new Ui::MeasureMainWindow)
	{
		
		ui->setupUi(this);
		connect(ui->divide_lib_filename, &QPushButton::clicked, this, [this]() {
			/*QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
				QDir::homePath(), tr("Text Files (*.txt);;All Files (*)"));*/
				// ѡ���ļ���

			QString dirPath = QFileDialog::getExistingDirectory(nullptr, "ѡ���ļ���");
			if (dirPath.isEmpty()) {
				//QMessageBox::information(nullptr, "��ʾ", "δѡ���ļ���");
				return 0;
			}

			QDir dir(dirPath);
			QStringList entries;
			entries = dir.entryList(QStringList() << "*.d.lib" << "*.lib", QDir::Files);

			QString dLibFileName = dirPath + "/dLibNames.txt";
			QString libFileName = dirPath + "/libNames.txt";

			QFile dLibFile(dLibFileName);
			QFile libFile(libFileName);

			// ȷ���ļ�����д��
			if (!dLibFile.open(QIODevice::WriteOnly | QIODevice::Text) ||
				!libFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
				//QMessageBox::critical(nullptr, "����", "�޷����ļ�����д��");
				return 1;
			}

			QTextStream dLibStream(&dLibFile);
			QTextStream libStream(&libFile);

			foreach(const QString & entry, entries) {
				if (entry.endsWith("d.lib")) {
					dLibStream << entry << "\n";
				}
				else if (entry.endsWith(".lib")) {
					libStream << entry << "\n";
				}
			}

			dLibFile.close();
			libFile.close();
			printf("���ļ��ɹ�");
		});
	
		connect(ui->divide_dll, &QPushButton::clicked, this, []() {
			QString dirPath = QFileDialog::getExistingDirectory(nullptr, "ѡ���ļ���");
			if (dirPath.isEmpty()) {
				return 0;
			}

			QDir dir(dirPath);
			QStringList entries;
			entries = dir.entryList(QDir::Files);

			QString debugDir = dirPath + "/debug_dll";
			QString releaseDir = dirPath + "/release_dll";

			// ���� debug �� release Ŀ¼������ǲ�����
			QDir().mkpath(debugDir);
			QDir().mkpath(releaseDir);

			foreach(const QString & entry, entries) {
				QFileInfo fileInfo(entry);
				QString srcFilePath = dir.filePath(entry);
				QString destFilePath;


				if (fileInfo.fileName().endsWith("d.dll", Qt::CaseInsensitive)) {
					destFilePath = debugDir + "/" + entry;
				}
				else if (fileInfo.fileName().endsWith(".dll", Qt::CaseInsensitive)) {
					destFilePath = releaseDir + "/" + entry;
				}

				if (!destFilePath.isEmpty()) {
					QFile srcFile(srcFilePath);
					QFile destFile(destFilePath);

					if (srcFile.copy(destFilePath)) {
						//qDebug() << "�ļ�" << entry << "�Ѹ��Ƶ�" << destFilePath;
					}
					else {
						//qDebug() << "�ļ�" << entry << "����ʧ��";
					}
				}

			}
		});
		
		connect(ui->divide_lib, &QPushButton::clicked, this, []() {
			QString dirPath = QFileDialog::getExistingDirectory(nullptr, "ѡ���ļ���");
			if (dirPath.isEmpty()) {
				return 0;
			}
			QDir dir(dirPath);
			QStringList entries;
			entries = dir.entryList(QDir::Files);

			QString debugDir = dirPath + "/debug_lib";
			QString releaseDir = dirPath + "/release_lib";

			// ���� debug �� release Ŀ¼������ǲ�����
			QDir().mkpath(debugDir);
			QDir().mkpath(releaseDir);

			foreach(const QString & entry, entries) {
				QFileInfo fileInfo(entry);
				QString srcFilePath = dir.filePath(entry);
				QString destFilePath;

				if (fileInfo.fileName().contains("-gd", Qt::CaseInsensitive)) {
					// ���� "-gd-"���ж�Ϊ Debug �汾
					destFilePath = debugDir + "/" + entry;
				}
				else {
					// ������ "-gd-"���ж�Ϊ Release �汾
					destFilePath = releaseDir + "/" + entry;
				}

				if (!destFilePath.isEmpty()) {
					QFile srcFile(srcFilePath);
					QFile destFile(destFilePath);

					if (srcFile.copy(destFilePath)) {
						//qDebug() << "�ļ�" << entry << "�Ѹ��Ƶ�" << destFilePath;
					}
					else {
						//qDebug() << "�ļ�" << entry << "����ʧ��";
					}
				}

			}
			});

		connect(ui->divide_lib_dlib, &QPushButton::clicked, this, []() {
			QString dirPath = QFileDialog::getExistingDirectory(nullptr, "ѡ���ļ���");
			if (dirPath.isEmpty()) {
				return 0;
			}

			QDir dir(dirPath);
			QStringList entries;
			entries = dir.entryList(QDir::Files);

			QString debugDir = dirPath + "/debug_lib";
			QString releaseDir = dirPath + "/release_lib";

			// ���� debug �� release Ŀ¼������ǲ�����
			QDir().mkpath(debugDir);
			QDir().mkpath(releaseDir);

			foreach(const QString & entry, entries) {
				QFileInfo fileInfo(entry);
				QString srcFilePath = dir.filePath(entry);
				QString destFilePath;

				if (fileInfo.fileName().contains("d.lib", Qt::CaseInsensitive)) {
					// ���� "-gd-"���ж�Ϊ Debug �汾
					destFilePath = debugDir + "/" + entry;
				}
				else {
					// ������ "-gd-"���ж�Ϊ Release �汾
					destFilePath = releaseDir + "/" + entry;
				}

				if (!destFilePath.isEmpty()) {
					QFile srcFile(srcFilePath);
					QFile destFile(destFilePath);

					if (srcFile.copy(destFilePath)) {
						//qDebug() << "�ļ�" << entry << "�Ѹ��Ƶ�" << destFilePath;
					}
					else {
						//qDebug() << "�ļ�" << entry << "����ʧ��";
					}
				}

			}
		});

		connect(ui->select_pc, &QPushButton::clicked, this, [this]() {
			QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
				QDir::homePath(), tr("Text Files (*.ply);;All Files (*)"));
			pcl::visualization::PCLVisualizer::Ptr viewerviewer(new pcl::visualization::PCLVisualizer("3D Viewer", true));
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
			//pcl::io::loadPCDFile(fileName.toStdString(), *cloud);
			pcl::io::loadPLYFile(fileName.toStdString(), *cloud);
			double A = -0.0512;
			double B = -0.012;
			double C = 0.9986;
			double D = -1 * (0.3587 * -0.0512 + -0.012 * -42.08 + 0.9986 * -11.1403);
			Eigen::Matrix<double, 4, 1> plane_param = { A,B,C, D};
			
			double denominator = std::sqrt(A * A + B * B + C * C);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud, 255, 0, 0); // ��ɫ (RGB)
			viewerviewer->addPointCloud<pcl::PointXYZ>(cloud, color1, "sample cloud");
			pcl::PointCloud<pcl::PointXYZ> select_cloud;
			pcl::PointCloud<pcl::PointXYZ> other_cloud;
			for (const auto& point : *cloud) {
				double numerator = std::abs(A * point.x + B * point.y + C * point.z + D);
				double distance = numerator / denominator;
				if (distance < 0.05 && distance>0) {
					select_cloud.push_back(point);
				}
				else {
					other_cloud.push_back(point);
				}

			}
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(other_cloud.makeShared(), 0, 0, 255); // ��ɫ (RGB)
			viewerviewer->addPointCloud<pcl::PointXYZ>(other_cloud.makeShared(), color3, "other cloud");
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(select_cloud.makeShared(), 0, 255, 0); // ��ɫ (RGB)
			viewerviewer->addPointCloud<pcl::PointXYZ>(select_cloud.makeShared(), color2, "selected cloud");

			while (!viewerviewer->wasStopped())
			{
				viewerviewer->spinOnce(100);
				QApplication::processEvents();
					//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
				// �������������
			}			//// ���ñ�����ɫ
			//viewer->setBackgroundColor(0, 0, 0);

			//// ��ӵ���ʾ����������ʵ���������滻��
			//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
			//cloud->width = 100;
			//cloud->height = 1;
			//cloud->is_dense = false;
			//for (size_t i = 0; i < cloud->width; ++i) {
			//	cloud->points.push_back(pcl::PointXYZ(i, i, i));
			//}
			//viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");

			//// ����Ⱦ����
			//vtkSmartPointer<vtkRenderWindow> renderWindow = vtkWidget->GetRenderWindow();
			//viewer->setupInteractor(renderWindow->GetInteractor(), renderWindow);
			//renderWindow->AddRenderer(viewer->getRendererCollection()->GetFirstRenderer());

		});

		
	}
	MainForm::~MainForm()
	{
		delete ui;	
	}
}