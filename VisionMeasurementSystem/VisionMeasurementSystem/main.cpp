#include <QApplication>
#include <QDebug>
#include "Form/MainForm.h"
#include "FitPlaneForm.h"
//#include "Windows.h"
#include "qwindow.h"

int main(int argc, char* argv[])
{
    // ���� DLL Ŀ¼
    BOOL result = SetDllDirectory(L".\\libs");
    if (!result) {
        qDebug() << "SetDllDirectory failed.";
        return -1;
    }
    else {
        qDebug() << "SetDllDirectory succeeded.";
    }

    // ���ÿ�·��
    /*QString libraryPath = QCoreApplication::applicationDirPath() + "/libs/platforms";
    QCoreApplication::addLibraryPath(libraryPath);
    qDebug() << "Library path set to:" << libraryPath;*/

    // ���� QApplication ����
    QApplication app(argc, argv);

    // ��ʾ������
    MeasruementMethodForm::FitPlaneForm fit_plane_form;
    fit_plane_form.show();

    // �����¼�ѭ��
    return app.exec();
}

