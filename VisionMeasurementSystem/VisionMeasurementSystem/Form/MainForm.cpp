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
				// 选择文件夹

			QString dirPath = QFileDialog::getExistingDirectory(nullptr, "选择文件夹");
			if (dirPath.isEmpty()) {
				//QMessageBox::information(nullptr, "提示", "未选择文件夹");
				return 0;
			}

			QDir dir(dirPath);
			QStringList entries;
			entries = dir.entryList(QStringList() << "*.d.lib" << "*.lib", QDir::Files);

			QString dLibFileName = dirPath + "/dLibNames.txt";
			QString libFileName = dirPath + "/libNames.txt";

			QFile dLibFile(dLibFileName);
			QFile libFile(libFileName);

			// 确保文件可以写入
			if (!dLibFile.open(QIODevice::WriteOnly | QIODevice::Text) ||
				!libFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
				//QMessageBox::critical(nullptr, "错误", "无法打开文件进行写入");
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
			printf("打开文件成功");
		});
	
		connect(ui->divide_dll, &QPushButton::clicked, this, []() {
			QString dirPath = QFileDialog::getExistingDirectory(nullptr, "选择文件夹");
			if (dirPath.isEmpty()) {
				return 0;
			}

			QDir dir(dirPath);
			QStringList entries;
			entries = dir.entryList(QDir::Files);

			QString debugDir = dirPath + "/debug_dll";
			QString releaseDir = dirPath + "/release_dll";

			// 创建 debug 和 release 目录如果它们不存在
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
						//qDebug() << "文件" << entry << "已复制到" << destFilePath;
					}
					else {
						//qDebug() << "文件" << entry << "复制失败";
					}
				}

			}
		});
		
		connect(ui->divide_lib, &QPushButton::clicked, this, []() {
			QString dirPath = QFileDialog::getExistingDirectory(nullptr, "选择文件夹");
			if (dirPath.isEmpty()) {
				return 0;
			}
			QDir dir(dirPath);
			QStringList entries;
			entries = dir.entryList(QDir::Files);

			QString debugDir = dirPath + "/debug_lib";
			QString releaseDir = dirPath + "/release_lib";

			// 创建 debug 和 release 目录如果它们不存在
			QDir().mkpath(debugDir);
			QDir().mkpath(releaseDir);

			foreach(const QString & entry, entries) {
				QFileInfo fileInfo(entry);
				QString srcFilePath = dir.filePath(entry);
				QString destFilePath;

				if (fileInfo.fileName().contains("-gd", Qt::CaseInsensitive)) {
					// 包含 "-gd-"，判定为 Debug 版本
					destFilePath = debugDir + "/" + entry;
				}
				else {
					// 不包含 "-gd-"，判定为 Release 版本
					destFilePath = releaseDir + "/" + entry;
				}

				if (!destFilePath.isEmpty()) {
					QFile srcFile(srcFilePath);
					QFile destFile(destFilePath);

					if (srcFile.copy(destFilePath)) {
						//qDebug() << "文件" << entry << "已复制到" << destFilePath;
					}
					else {
						//qDebug() << "文件" << entry << "复制失败";
					}
				}

			}
			});

		connect(ui->divide_lib_dlib, &QPushButton::clicked, this, []() {
			QString dirPath = QFileDialog::getExistingDirectory(nullptr, "选择文件夹");
			if (dirPath.isEmpty()) {
				return 0;
			}

			QDir dir(dirPath);
			QStringList entries;
			entries = dir.entryList(QDir::Files);

			QString debugDir = dirPath + "/debug_lib";
			QString releaseDir = dirPath + "/release_lib";

			// 创建 debug 和 release 目录如果它们不存在
			QDir().mkpath(debugDir);
			QDir().mkpath(releaseDir);

			foreach(const QString & entry, entries) {
				QFileInfo fileInfo(entry);
				QString srcFilePath = dir.filePath(entry);
				QString destFilePath;

				if (fileInfo.fileName().contains("d.lib", Qt::CaseInsensitive)) {
					// 包含 "-gd-"，判定为 Debug 版本
					destFilePath = debugDir + "/" + entry;
				}
				else {
					// 不包含 "-gd-"，判定为 Release 版本
					destFilePath = releaseDir + "/" + entry;
				}

				if (!destFilePath.isEmpty()) {
					QFile srcFile(srcFilePath);
					QFile destFile(destFilePath);

					if (srcFile.copy(destFilePath)) {
						//qDebug() << "文件" << entry << "已复制到" << destFilePath;
					}
					else {
						//qDebug() << "文件" << entry << "复制失败";
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
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud, 255, 0, 0); // 红色 (RGB)
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
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(other_cloud.makeShared(), 0, 0, 255); // 红色 (RGB)
			viewerviewer->addPointCloud<pcl::PointXYZ>(other_cloud.makeShared(), color3, "other cloud");
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(select_cloud.makeShared(), 0, 255, 0); // 红色 (RGB)
			viewerviewer->addPointCloud<pcl::PointXYZ>(select_cloud.makeShared(), color2, "selected cloud");

			while (!viewerviewer->wasStopped())
			{
				viewerviewer->spinOnce(100);
				QApplication::processEvents();
					//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
				// 可添加其他操作
			}			//// 设置背景颜色
			//viewer->setBackgroundColor(0, 0, 0);

			//// 添加点云示例（可用真实点云数据替换）
			//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
			//cloud->width = 100;
			//cloud->height = 1;
			//cloud->is_dense = false;
			//for (size_t i = 0; i < cloud->width; ++i) {
			//	cloud->points.push_back(pcl::PointXYZ(i, i, i));
			//}
			//viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");

			//// 绑定渲染窗口
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