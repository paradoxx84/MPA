#ifndef WIDGET_H
#define WIDGET_H

#define POLLTIMER 40

#define COMMAND_LENGTH 14
#define SOF 0x3B

#include <QWidget>
#include "QTimer"
#include "QtSerialPort/qserialport.h"
#include "QtSerialPort/qserialportinfo.h"
#include <fileapi.h>
#include <QThread>
#include <windows.h>
#include "SerialReader.h"
#include <QCloseEvent>
#include <QMutex>
#include <QMutexLocker>

typedef struct
{
    uint8_t Length;
    uint8_t Idx;
    uint8_t ID[256];
}Queue_t;

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

private slots:
    void on_ConnectButton_pressed();
    void Request();
    void handleSerialData(const QByteArray &data);
    void closePort();
    void on_AUTOradioButton_toggled(bool checked);

    void on_VentpushButton_pressed();

    void on_IPSRUNpushButton_pressed();

    void on_IPSStoppushButton_pressed();

    void on_IPSSETpushButton_pressed();

private:
    Ui::Widget *ui;
    QMutex m_mutex;
    HANDLE comport;
    DCB dcb;
    QSerialPortInfo* info;
    SerialReader *readerThread = nullptr;
    QByteArray m_buffer;
    QTimer *m_rxTimeoutTimer = nullptr; // Таймер сброса мусора

    QTimer* pRequestTimer;

    Queue_t ReadQueue;
    Queue_t WriteQueue;
    Queue_t* Queue;

    uint8_t RunParam;
protected:
    void closeEvent(QCloseEvent *event) override;
};
#endif // WIDGET_H
