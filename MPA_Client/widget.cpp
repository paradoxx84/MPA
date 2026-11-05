#include "widget.h"
#include "ui_widget.h"

#include <cstring>
#include <cmath>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);

    m_buffer.reserve(4096);

    dcb.DCBlength = sizeof(dcb);
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;

    info = new QSerialPortInfo();
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        ui->ComPortBox->addItem(info.portName(),info.portName());
    }
    if (ui->ComPortBox->count() > 0)
    {
        ui->ComPortBox->setCurrentIndex(0);
    }else{
        ui->ConnectButton->setEnabled(false);
    }

    pRequestTimer = new QTimer();
    pRequestTimer->setInterval(POLLTIMER);    
    connect(pRequestTimer, &QTimer::timeout, this, &Widget::Request);

    m_rxTimeoutTimer = new QTimer(this);
    m_rxTimeoutTimer->setSingleShot(true);
    m_rxTimeoutTimer->setInterval(10); // 10 миллисекунд

    // При срабатывании таймера очищаем буфер под защитой мьютекса
    connect(m_rxTimeoutTimer, &QTimer::timeout, this, [this]()
    {
        QMutexLocker locker(&m_mutex);
        if (!m_buffer.isEmpty())
        {
            m_buffer.clear();
            // Разрешено для отладки, чтобы видеть сбросы:
            // qWarning() << "Отрезок кадра удален по таймауту 10мс";
        }
    });

    WriteQueue.Length = 0;

    ReadQueue.Length = 6;
    ReadQueue.Idx = 0;
    ReadQueue.ID[0] = 0x00;
    ReadQueue.ID[1] = 0x01;
    ReadQueue.ID[2] = 0x04;
    ReadQueue.ID[3] = 0x06;
    ReadQueue.ID[4] = 0x0A;
    ReadQueue.ID[5] = 0x0C;

    Queue = &ReadQueue;
}

void Widget::closeEvent(QCloseEvent *event)
{
    if (readerThread)
    {
        readerThread->stop();

        if (!readerThread->wait(1000))
        {
            readerThread->terminate();
        }
    }

    if (comport != INVALID_HANDLE_VALUE)
    {
        CloseHandle(comport);
    }

    event->accept();
}

Widget::~Widget()
{
    delete ui;
}

void Widget::Request()
{
    uint8_t tempbuf[COMMAND_LENGTH] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    if(WriteQueue.Length > 0)
    {
        Queue = &WriteQueue;
    }else{
        Queue = &ReadQueue;
    }
    tempbuf[0] = SOF;
    tempbuf[1] = Queue->ID[Queue->Idx];
    switch(Queue->ID[Queue->Idx])
    {
        case 0x09:
        {
            uint16_t param = ui->VentspinBox->value();
            tempbuf[2] = param & 0x00FF;
            tempbuf[3] = (param & 0xFF00) >> 8;
            break;
        }
        case 0x0B:
        {

            int8_t param1 = ui->IPSSPEEDspinBox->value();
            if(ui->ReverscheckBox->isChecked())
            {
                param1 *= -1;
            }            
            uint8_t param2 = ui->IPSACCspinBox->value();
            tempbuf[2] = param1;
            tempbuf[3] = param2;
            tempbuf[4] = RunParam;
            break;
        }
    }
    Queue->Idx++;
    if(Queue->Idx >= Queue->Length)
    {
        Queue->Idx = 0;
        if(Queue == &WriteQueue)
        {
            WriteQueue.Length = 0;
            Queue = &ReadQueue;
        }
    }
    DWORD bytesWritten = 0;
    OVERLAPPED osWrite = {};

    // Создаем событие для ожидания окончания записи
    osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (osWrite.hEvent == NULL) return;

    if (!WriteFile(comport, tempbuf, COMMAND_LENGTH, &bytesWritten, &osWrite))
    {
        if (GetLastError() == ERROR_IO_PENDING)
        {
            DWORD res = WaitForSingleObject(osWrite.hEvent, 100);
            if (res == WAIT_OBJECT_0) {
                GetOverlappedResult(comport, &osWrite, &bytesWritten, FALSE);
            }
        }
    }

    // Обязательно закрываем хэндл события, чтобы не было утечки ресурсов
    CloseHandle(osWrite.hEvent);
}

void Widget::handleSerialData(const QByteArray &data)
{
    Q_UNUSED(data);
    QMutexLocker locker(&m_mutex);

    if (m_buffer.size() > 4096)
    {
        m_buffer.clear();
    }
    QMetaObject::invokeMethod(m_rxTimeoutTimer, "start", Qt::QueuedConnection);
    m_buffer.append(data);
    while (m_buffer.size() >= COMMAND_LENGTH)
    {
        if (m_buffer.at(0) != SOF)
        {
            int nextSofIndex = m_buffer.indexOf(SOF);
            if (nextSofIndex != -1)
            {
                m_buffer.remove(0, nextSofIndex);
            }else{
                m_buffer.clear();
            }
            continue;
        }

        QByteArray frame = m_buffer.left(COMMAND_LENGTH);

        switch(frame.at(1))
        {
            case 0x00:
            {
                QByteArray floatBytes = frame.mid(2, 4);
                float rawValue = 0.0f;
                std::memcpy(&rawValue, floatBytes.constData(), sizeof(float));
                QString formattedString = QString::number(rawValue, 'f', 2);
                QMetaObject::invokeMethod(ui->FWRevLabel, "setText",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, "Версия ПО " + formattedString));

                floatBytes = frame.mid(7, 4);
                uint32_t totalSeconds = 0;
                std::memcpy(&totalSeconds, floatBytes.constData(), sizeof(uint32_t));

                uint32_t hours = totalSeconds / 3600;
                uint32_t minutes = (totalSeconds % 3600) / 60;
                uint32_t seconds = totalSeconds % 60;

                QString timeString = QString("%1:%2:%3")
                                     .arg(hours, 2, 10, QChar('0'))
                                     .arg(minutes, 2, 10, QChar('0'))
                                     .arg(seconds, 2, 10, QChar('0'));

                QMetaObject::invokeMethod(ui->UptimeLabel, "setText",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, "Время работы " + timeString));

                floatBytes = frame.mid(11, 2);
                uint16_t Status = 0;
                std::memcpy(&Status, floatBytes.constData(), sizeof(Status));
                if(Status & 0x08)
                {
                    ui->ONradioButton->setChecked(true);
                }else{
                    ui->AUTOradioButton->setChecked(true);
                }
                break;
            }
            case 0x01:
            {
                QByteArray floatBytes = frame.mid(2, 4);
                float rawValue = 0.0f;
                std::memcpy(&rawValue, floatBytes.constData(), sizeof(float));
                QString formattedString = QString::number(rawValue, 'f', 2);
                QMetaObject::invokeMethod(ui->lcdNumber, "display",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, formattedString));
                break;
            }
            case 0x04:
            {
                QByteArray floatBytes = frame.mid(2, 4);
                float rawValue = 0.0f;
                std::memcpy(&rawValue, floatBytes.constData(), sizeof(float));
                QString formattedString = QString::number(rawValue, 'f', 2);
                QMetaObject::invokeMethod(ui->lcdNumber_3, "display",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, formattedString));
                break;
            }
            case 0x06:
            {
                QByteArray floatBytes = frame.mid(2, 4);
                float rawValue = 0.0f;
                std::memcpy(&rawValue, floatBytes.constData(), sizeof(float));
                QString formattedString = QString::number(rawValue, 'f', 2);
                QMetaObject::invokeMethod(ui->TemplcdNumber, "display",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, formattedString));
                break;
            }
            case 0x07:
            {
                break;
            }
            case 0x08:
            {
                break;
            }
            case 0x0A:
            {
                QByteArray floatBytes = frame.mid(2, 2);
                uint16_t Vent = 0;
                std::memcpy(&Vent, floatBytes.constData(), sizeof(Vent));
                if(Vent)
                {
                    QString formattedString = QString::number(Vent);
                    QMetaObject::invokeMethod(ui->VentStateLabel, "setText",
                                              Qt::QueuedConnection,
                                              Q_ARG(QString, "Статус: СБРОС, осталось " + formattedString + "с"));
                }else{
                    QMetaObject::invokeMethod(ui->VentStateLabel, "setText",
                                              Qt::QueuedConnection,
                                              Q_ARG(QString, "Статус: ЗАКРЫТ"));
                }
                break;
            }            
            case 0x0C:
            {
                int8_t Speed = frame.constData()[2];
                QMetaObject::invokeMethod(ui->IPSlcdNumber, "display",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, QString::number(Speed)));
                break;
            }
        }

        // Удаляем обработанный кадр из начала буфера
        m_buffer.remove(0, COMMAND_LENGTH);
        QMetaObject::invokeMethod(m_rxTimeoutTimer, "stop", Qt::QueuedConnection);
    }
}

void Widget::on_ConnectButton_pressed()
{
    QString fullPortName = ui->ComPortBox->itemText(ui->ComPortBox->currentIndex());
    if (fullPortName.startsWith("COM") && fullPortName.length() > 3)
    {
        int comNumber = fullPortName.mid(3).toInt();
        if (comNumber > 9)
        {
            fullPortName = "\\\\.\\" + fullPortName;
        }
    }
    std::wstring widePort = fullPortName.toStdWString();
    comport = CreateFile(widePort.c_str(),
                                GENERIC_READ | GENERIC_WRITE,
                                0,
                                NULL,
                                OPEN_EXISTING,
                                FILE_FLAG_OVERLAPPED, // Use overlapped I/O for async
                                //FILE_ATTRIBUTE_NORMAL,
                                NULL);

    if(comport != INVALID_HANDLE_VALUE)
    {
        dcb.DCBlength = sizeof(dcb);
        GetCommState(comport, &dcb);
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.BaudRate = 115200;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDtrControl = DTR_CONTROL_DISABLE;
        dcb.fDsrSensitivity = FALSE;
        dcb.fTXContinueOnXoff = FALSE;
        dcb.fOutX = FALSE;
        dcb.fInX = FALSE;
        dcb.fErrorChar = FALSE;
        dcb.fNull = FALSE;
        dcb.fRtsControl = RTS_CONTROL_DISABLE;
        dcb.fAbortOnError = FALSE;
        dcb.fBinary = TRUE;
        dcb.fParity = FALSE;

        if (SetCommState(comport, &dcb))
        {
            ui->ConnectButton->setEnabled(false);
            COMMTIMEOUTS timeouts;
            timeouts.ReadIntervalTimeout = MAXDWORD;
            timeouts.ReadTotalTimeoutMultiplier = 0;
            timeouts.ReadTotalTimeoutConstant = 0;
            timeouts.WriteTotalTimeoutMultiplier = 10;
            timeouts.WriteTotalTimeoutConstant = 1000;
            SetCommTimeouts(comport, &timeouts);

            Queue = &ReadQueue;
            pRequestTimer->start();

            readerThread = new SerialReader(comport, this);
            connect(readerThread, &SerialReader::dataReceived, this, &Widget::handleSerialData);
            connect(readerThread, &SerialReader::finished, readerThread, &QObject::deleteLater);
            readerThread->start(); // Запуск потока чтения
        }else{
            CloseHandle(comport);
            comport = INVALID_HANDLE_VALUE;
        }
    }
}

void Widget::closePort()
{
    if (readerThread)
    {
        readerThread->stop();
        readerThread->wait(); // Ждем безопасного завершения потока
        readerThread = nullptr;
    }

    if (comport != INVALID_HANDLE_VALUE)
    {
        CloseHandle(comport);
        comport = INVALID_HANDLE_VALUE;
    }
}


void Widget::on_AUTOradioButton_toggled(bool checked)
{
    ui->radioButton1->setEnabled(checked);
    ui->radioButton2->setEnabled(checked);
    ui->radioButton3->setEnabled(checked);
}


void Widget::on_VentpushButton_pressed()
{
    WriteQueue.Length = 1;
    WriteQueue.Idx = 0;
    WriteQueue.ID[0] = 0x09;
}

void Widget::on_IPSRUNpushButton_pressed()
{
    WriteQueue.Length = 1;
    WriteQueue.Idx = 0;
    WriteQueue.ID[0] = 0x0B;
    RunParam = 1;
}


void Widget::on_IPSStoppushButton_pressed()
{
    WriteQueue.Length = 1;
    WriteQueue.Idx = 0;
    WriteQueue.ID[0] = 0x0B;
    RunParam = 0;
}


void Widget::on_IPSSETpushButton_pressed()
{
    WriteQueue.Length = 1;
    WriteQueue.Idx = 0;
    WriteQueue.ID[0] = 0x0B;
    RunParam = 2;
}
