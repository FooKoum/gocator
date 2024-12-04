#include <QApplication>
#include <QDebug>
#include "Form/MainForm.h"
#include "FitPlaneForm.h"
//#include "Windows.h"
#include "qwindow.h"

int main(int argc, char* argv[])
{
    // 设置 DLL 目录
    BOOL result = SetDllDirectory(L".\\libs");
    if (!result) {
        qDebug() << "SetDllDirectory failed.";
        return -1;
    }
    else {
        qDebug() << "SetDllDirectory succeeded.";
    }

    // 设置库路径
    /*QString libraryPath = QCoreApplication::applicationDirPath() + "/libs/platforms";
    QCoreApplication::addLibraryPath(libraryPath);
    qDebug() << "Library path set to:" << libraryPath;*/

    // 创建 QApplication 对象
    QApplication app(argc, argv);

    // 显示主窗口
    MeasruementMethodForm::FitPlaneForm fit_plane_form;
    fit_plane_form.show();

    // 启动事件循环
    return app.exec();
}

