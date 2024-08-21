#ifndef WAVESHARESERIAL_H
#define WAVESHARESERIAL_H

#include <QSerialPort>
#include <QCanBusDevice>
#include <QThread>
#include <QTimer>
#include <QTcpSocket>
#include <QUdpSocket>

/*************/
#include <QDateTime>
/*************/

#include "canframemodel.h"
#include "canconnection.h"
#include "canconmanager.h"

namespace SERIALSTATE {
    enum WSSTATE
    {
        WS_IDLE,
        WS_GET_COMMAND,
        WS_BUILD_CAN_FRAME,
        WS_GET_CANBUS_PARAMS,
        WS_SET_SERIAL_PARAMS,
    };
}

using namespace SERIALSTATE;

class WaveshareSerial : public CANConnection
{
    Q_OBJECT

public:
    WaveshareSerial(QString portName);
    virtual ~WaveshareSerial();

protected:

    virtual void piStarted();
    virtual void piStop();
    virtual void piSetBusSettings(int pBusIdx, CANBus pBus);
    virtual bool piGetBusSettings(int pBusIdx, CANBus& pBus);
    virtual void piSetSerialSettings(int pBusIdx, CANBus bus);
    virtual void piSuspend(bool pSuspend);
    virtual bool piSendFrame(const CANFrame&);

    void disconnectDevice();
    int genChecksum(QByteArray& data, int data_start, int data_end);

public slots:
    void debugInput(QByteArray bytes);

private slots:
    void connectDevice();
    void readSerialData();
    void serialError(QSerialPort::SerialPortError err);
    void deviceConnected();

private:
    //void readSettings();
    void procRXChar(unsigned char);
    void sendCommValidation();
    void rebuildLocalTimeBasis();
    void sendToSerial(const QByteArray &bytes);
    void sendDebug(const QString debugText);

protected:
    QTimer             mTimer;
    QThread            mThread;

    QSerialPort *serial;
    int framesRapid;
    WSSTATE rx_state;
    int rx_step;
    int rx_count;
    int rx_expected_count;
    CANFrame buildFrame;
    qint64 buildTimestamp;
    quint32 buildId;
    QByteArray buildData;
    int can0Baud, can1Baud, swcanBaud, lin1Baud, lin2Baud;
    bool can0Enabled, can1Enabled, swcanEnabled, lin1Enabled, lin2Enabled;
    bool can0ListenOnly, can1ListenOnly, swcanListenOnly;
};

#endif // WAVESHARESERIAL_H
