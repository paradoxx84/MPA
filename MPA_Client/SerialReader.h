#ifndef SERIALREADER_H
#define SERIALREADER_H


#pragma once
#include <QThread>
#include <windows.h>

class SerialReader : public QThread {
    Q_OBJECT
public:
    SerialReader(HANDLE portHandle, QObject *parent = nullptr)
        : QThread(parent), m_port(portHandle), m_running(true) {}

    void stop() {
        m_running = false;
        // Разблокируем WaitCommEvent, если он завис в ожидании
        SetCommMask(m_port, 0);
    }

signals:
    void dataReceived(const QByteArray &data);

protected:
    void run() override {
            SetCommMask(m_port, EV_RXCHAR);

            OVERLAPPED osStatus{};
            // ВАЖНО: Событие ДОЛЖНО быть с ручным сбросом (Manual Reset = TRUE),
            // это критично для корректной работы WaitCommEvent в Windows
            osStatus.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
            if (osStatus.hEvent == NULL) return;

            DWORD eventMask = 0;
            char buffer[256]; // Расширим буфер, чтобы читать пачки данных целиком
            DWORD bytesRead = 0;
            COMSTAT comStat;
            DWORD errors;

            while (m_running) {
                // 1. Сбрасываем событие перед вызовом асинхронной функции
                ResetEvent(osStatus.hEvent);

                // Вызываем асинхронный WaitCommEvent
                BOOL success = WaitCommEvent(m_port, &eventMask, &osStatus);

                if (!success) {
                    DWORD lastErr = GetLastError();
                    if (lastErr == ERROR_IO_PENDING) {
                        // Данных нет, поток засыпает здесь максимум на 100 мс
                        DWORD waitRes = WaitForSingleObject(osStatus.hEvent, 100);

                        if (waitRes == WAIT_TIMEOUT) {
                            continue; // Ничего не пришло, уходим на проверку m_running
                        }

                        if (waitRes == WAIT_OBJECT_0) {
                            // КРИТИЧЕСКИЙ ШАГ: Данные пришли в фоне. Обязательно вызываем
                            // GetOverlappedResult, чтобы ОС закрыла текущую операцию!
                            DWORD transferred = 0;
                            GetOverlappedResult(m_port, &osStatus, &transferred, FALSE);
                        } else {
                            break; // Какая-то системная ошибка
                        }
                    } else {
                        break; // Ошибка порта (например, кабель отключили)
                    }
                }

                // 2. Если мы дошли сюда — событие EV_RXCHAR гарантированно произошло!
                if (eventMask & EV_RXCHAR) {
                    ClearCommError(m_port, &errors, &comStat);
                    DWORD bytesToRead = comStat.cbInQue;

                    // Читаем всё, что успело накопиться в аппаратном буфере Windows
                    while (bytesToRead > 0 && m_running) {
                        DWORD chunk = (bytesToRead > sizeof(buffer)) ? sizeof(buffer) : bytesToRead;

                        OVERLAPPED osRead{};
                        osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

                        if (ReadFile(m_port, buffer, chunk, &bytesRead, &osRead)) {
                            if (bytesRead > 0) {
                                emit dataReceived(QByteArray(buffer, bytesRead));
                            }
                        } else if (GetLastError() == ERROR_IO_PENDING) {
                            // Если чтение ушло в фон, ждем его окончания
                            if (WaitForSingleObject(osRead.hEvent, 50) == WAIT_OBJECT_0) {
                                GetOverlappedResult(m_port, &osRead, &bytesRead, FALSE);
                                if (bytesRead > 0) {
                                    emit dataReceived(QByteArray(buffer, bytesRead));
                                }
                            }
                        }

                        CloseHandle(osRead.hEvent);

                        // Проверяем, не долетело ли еще что-то, пока мы читали
                        ClearCommError(m_port, &errors, &comStat);
                        bytesToRead = comStat.cbInQue;
                    }
                }
            }

            // Очистка ресурсов перед выходом из потока
            CloseHandle(osStatus.hEvent);
        }

private:
    HANDLE m_port;
    bool m_running;
};

#endif // SERIALREADER_H
