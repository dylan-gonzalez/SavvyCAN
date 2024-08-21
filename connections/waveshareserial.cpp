#include <QObject>
#include <QDebug>
#include <QCanBusFrame>
#include <QSerialPortInfo>
#include <QSettings>
#include <QStringBuilder>
#include <QtNetwork>

#include "waveshareserial.h"

WaveshareSerial::WaveshareSerial(QString portName) :
    CANConnection(portName, "waveshare", CANCon::WAVESHARE_SERIAL, 0, 0, false, 0, 3, 4000, true),
    mTimer(this) /*NB: set this as parent of timer to manage it from working thread */
{
    sendDebug("WaveshareSerial()");

    serial = nullptr;
    rx_state = WS_IDLE;
    rx_step = 0;

    //readSettings();
}

WaveshareSerial::~WaveshareSerial()
{
    stop();
    sendDebug("~WaveshareSerial()");
}

void WaveshareSerial::sendDebug(const QString debugText)
{
    qDebug() << debugText;
    debugOutput(debugText);
}

void WaveshareSerial::sendToSerial(const QByteArray &bytes)
{
    sendDebug("sendToSerial()");
    if (serial == nullptr)
    {
        sendDebug("Attempt to write to serial port when it has not been initialized!");
        return;
    }

    if (serial && !serial->isOpen())
    {
        sendDebug("Attempt to write to serial port when it is not open!");
        return;
    }

    QString buildDebug;
    buildDebug = "Write to serial -> ";
    foreach (int byt, bytes) {
        byt = (unsigned char)byt;
        buildDebug = buildDebug % QString::number(byt, 16) % " ";
    }
    sendDebug(buildDebug);

    if (serial) serial->write(bytes);
}

void WaveshareSerial::piStarted()
{
    sendDebug("piStarted()");
    connectDevice();
}


void WaveshareSerial::piSuspend(bool pSuspend)
{
    sendDebug("piSuspend()");
    /* update capSuspended */
    setCapSuspended(pSuspend);

    /* flush queue if we are suspended */
    if(isCapSuspended())
        getQueue().flush();
}


void WaveshareSerial::piStop()
{
    sendDebug("piStop()");
    mTimer.stop();
    disconnectDevice();
}

bool WaveshareSerial::piGetBusSettings(int pBusIdx, CANBus& pBus)
{
    sendDebug("piGetBusSettings()");
    return getBusConfig(pBusIdx, pBus);
}

void WaveshareSerial::piSetSerialSettings(int pBusIdx, CANBus bus)
{
    sendDebug("piSetSerialSettings()");
    /* sanity checks */
    if( (pBusIdx < 0) || pBusIdx >= getNumBuses())
        return;

    qDebug() << "About to update serial baudrate on Waveshare";

    /*
     * Serial Baud Rates
     * 0x00 = 2,000,000
     * 0x01 = 1,228,800
     * 0x02 = 115,200
     * 0x03 = 38,400
     * 0x04 = 19,200
     * 0x05 = 9,600
     */
    if (pBusIdx == 0) {
        /* update baud rates */
        QByteArray buffer;
        buffer.fill(0,20);
        sendDebug("Got signal to update serial bauds");
        buffer[0] = (char)0xAA; //start of a frame over serial
        buffer[1] = (char)0x55; //start of a command over serial
        buffer[2] = (char)0x06; // Update Serial Baudrate
        buffer[3] = (char)0x00; // Serial Baud Rate: 0x00 = 2000000
        buffer[4] = (char)0x00; // Spare (not used)
        buffer[5] = (char)0x00; // Spare (not used)
        buffer[6] = (char)0x00; // Spare (not used)
        buffer[7] = (char)0x00; // Spare (not used)
        buffer[8] = (char)0x00; // Spare (not used)
        buffer[9] = (char)0x00; // Spare (not used)
        buffer[10] = (char)0x00; // Spare (not used)
        buffer[11] = (char)0x00; // Spare (not used)
        buffer[12] = (char)0x00; // Spare (not used)
        buffer[13] = (char)0x00; // Spare (not used)
        buffer[14] = (char)0x00; // Spare (not used)
        buffer[15] = (char)0x00; // Spare (not used)
        buffer[16] = (char)0x00; // Spare (not used)
        buffer[17] = (char)0x00; // Spare (not used)
        buffer[18] = (char)0x00; // Spare (not used)
        buffer[19] = genChecksum(buffer, 2, 19); // Checksum (sum of lower 8 bits from Type to Checksum)
        sendToSerial(buffer);
    }
}

void WaveshareSerial::piSetBusSettings(int pBusIdx, CANBus bus)
{
    sendDebug("piSetBusSettings()");
    /* sanity checks */
    if( (pBusIdx < 0) || pBusIdx >= getNumBuses())
        return;

    /* copy bus config */
    setBusConfig(pBusIdx, bus);

    qDebug() << "About to update bus " << pBusIdx << " on Waveshare";
    if (pBusIdx == 0)
    {
        can0Baud = bus.getSpeed();
        can0Baud |= 0x80000000;
        if (bus.isActive())
        {
            can0Baud |= 0x40000000;
            can0Enabled = true;
        }
        else can0Enabled = false;

        if (bus.isListenOnly())
        {
            can0Baud |= 0x20000000;
            can0ListenOnly = true;
        }
        else can0ListenOnly = false;
    }

    if (pBusIdx == 0) {
        /* update baud rates */
        QByteArray buffer;
        buffer.fill(0,20);
        sendDebug("Got signal to update bauds. 0: " + QString::number((can0Baud & 0xFFFFFFF)));
        buffer[0] = (char)0xAA; //start of a frame over serial
        buffer[1] = (char)0x55; //start of a command over serial
        buffer[2] = (char)0x12; // Type: Variable length protocol
        buffer[3] = (char)0x01; // CAN Baud Rate: 0x01 = 1Mbps
        buffer[4] = (char)0x01; // Standard frame
        buffer[5] = (char)0x00; // Filter 1 (not used)
        buffer[6] = (char)0x00; // Filter 2 (not used)
        buffer[7] = (char)0x00; // Filter 3 (not used)
        buffer[8] = (char)0x00; // Filter 4 (not used)
        buffer[9] = (char)0x00; // Filter Mask 1 (not used)
        buffer[10] = (char)0x00; // Filter Mask 2 (not used)
        buffer[11] = (char)0x00; // Filter Mask 3 (not used)
        buffer[12] = (char)0x00; // Filter Mask 4 (not used)
        buffer[13] = (char)0x01; // CAN mode: 0x01 = Loopback
        //buffer[13] = (char)0x00; // CAN mode: 0x00 = Normal
        buffer[14] = (char)0x01; // Automatic retransmission: 0x01 = Disabled
        buffer[15] = (char)0x00; // Spare (not used)
        buffer[16] = (char)0x00; // Spare (not used)
        buffer[17] = (char)0x00; // Spare (not used)
        buffer[18] = (char)0x00; // Spare (not used)
        buffer[19] = genChecksum(buffer, 2, 19); // Checksum (sum of lower 8 bits from Type to Checksum)
        sendToSerial(buffer);
    }
}

int WaveshareSerial::genChecksum(QByteArray& data, int data_start, int data_end)
{
    int i, checksum;

    sendDebug("genChecksum()");
    checksum = 0;
    for(i = data_start; i < data_end; i++) {
        checksum += data[i];
    }

    return checksum & 0xFF;
}

bool WaveshareSerial::piSendFrame(const CANFrame& frame)
{
    sendDebug("piSendFrame()");
    QByteArray buffer;
    int c;
    quint32 ID;
    int dataFrameLen = 0;

    //qDebug() << "Sending out Waveshare frame with id " << frame.ID << " on bus " << frame.bus;

    framesRapid++;

    if (serial == nullptr) return false;
    if (serial && !serial->isOpen()) return false;
    //if (!isConnected) return false;

    // Doesn't make sense to send an error frame
    // to an adapter
    if (frame.frameId() & 0x20000000) {
        return true;
    }
    ID = frame.frameId();
    if (frame.hasExtendedFrameFormat()) ID |= 1u << 31;

    buffer.fill(0,15);
    // Byte 0: Packet start
    buffer[dataFrameLen++] = (char)0xAA; //start of a frame over serial
    // Byte 1: CAN Bus Data Frame Information
    buffer[dataFrameLen] = (char)0xC0; // Frame type always starts with 0xC0
    if(frame.hasExtendedFrameFormat())
    {
        buffer[dataFrameLen] = (char)(buffer[dataFrameLen]) | (char)0x20; // Extended frame set bit 5

    }
    else
    {
        buffer[dataFrameLen] = (char)(buffer[dataFrameLen]) & (char)0xDF; // Extended frame clear bit 5
    }
    if(frame.frameType() == QCanBusFrame::RemoteRequestFrame)
    {
        buffer[dataFrameLen] = (char)(buffer[dataFrameLen]) | (char)0x10; // Remote frame set bit 4

    }
    else
    {
        buffer[dataFrameLen] = (char)(buffer[dataFrameLen]) & (char)0xEF; // Extended frame clear bit 4

    }
    buffer[dataFrameLen] = (char)(buffer[dataFrameLen]) | (char)(frame.payload().length() & 0xF); // Data frame length bits 0-3

    dataFrameLen++;

    // Byte 2-3: ID
    buffer[dataFrameLen++] = (char)(ID & 0xFF); //four bytes of ID LSB first
    buffer[dataFrameLen++] = (char)(ID >> 8);
    if(frame.hasExtendedFrameFormat())
    {
        // Byte 4-5: Extended ID
        buffer[dataFrameLen++] = (char)(ID >> 16);
        buffer[dataFrameLen++] = (char)(ID >> 24);
    }

    // Byte 4-11 (or 6-13)
    for (c = 0; c < frame.payload().length(); c++)
    {
        buffer[dataFrameLen + c] = frame.payload()[c];
    }
    dataFrameLen += c;
    buffer[dataFrameLen] = (char)0x55; // End Frame
    buffer.resize(dataFrameLen+1);

    sendToSerial(buffer);

    return true;
}

/****************************************************************/

/*void WaveshareSerial::readSettings()
{
    sendDebug("readSettings()");
    QSettings settings;

    if (settings.value("Main/ValidateComm", true).toBool())
    {
        //doValidation = true;
        doValidation = false;
    }
    else doValidation = false;
}*/

void WaveshareSerial::connectDevice()
{
    sendDebug("connectDevice()");
    QSettings settings;

    /* disconnect device */
    if(serial)
        disconnectDevice();

    /* open new device */
    sendDebug("Serial connection to a Waveshare device");
    serial = new QSerialPort(QSerialPortInfo(getPort()));
    if(!serial) {
        sendDebug("can't open serial port " + getPort());
        return;
    }
    sendDebug("Created Serial Port Object");

    /* connect reading event */
    connect(serial, SIGNAL(readyRead()), this, SLOT(readSerialData()));
    connect(serial, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(serialError(QSerialPort::SerialPortError)));

    /* configure */
    serial->setBaudRate(2000000); // Default value is 2000000
    serial->setDataBits(serial->Data8);
    //serial->setStopBits(serial->TwoStop); // I think this is actually correct, still works with OneStop...
    serial->setStopBits(serial->OneStop);

    sendDebug("Trying Standard Serial Mode");
    if (!serial->open(QIODevice::ReadWrite))
    {
        sendDebug("Error returned during port opening: " + serial->errorString());
    }
    else
    {
        deviceConnected();
    }
}

void WaveshareSerial::deviceConnected()
{
        sendDebug("deviceConnected()");
        sendDebug("Connecting to Waveshare Device!");
        QByteArray output;
        output.fill(0,20);
        sendDebug("Setting initial sane values...");
        output[0] = (char)0xAA; //start of a frame over serial
        output[1] = (char)0x55; //start of a command over serial
        output[2] = (char)0x12; // Type: Variable length protocol
        output[3] = (char)0x01; // CAN Baud Rate: 0x01 = 1Mbps
        output[4] = (char)0x01; // Standard frame
        output[5] = (char)0x00; // Filter 1 (not used)
        output[6] = (char)0x00; // Filter 2 (not used)
        output[7] = (char)0x00; // Filter 3 (not used)
        output[8] = (char)0x00; // Filter 4 (not used)
        output[9] = (char)0x00; // Filter Mask 1 (not used)
        output[10] = (char)0x00; // Filter Mask 2 (not used)
        output[11] = (char)0x00; // Filter Mask 3 (not used)
        output[12] = (char)0x00; // Filter Mask 4 (not used)
        output[13] = (char)0x01; // CAN mode: 0x01 = Loopback
        //output[13] = (char)0x00; // CAN mode: 0x00 = silent
        output[14] = (char)0x01; // Automatic retransmission: 0x01 = Disabled
        output[15] = (char)0x00; // Spare (not used)
        output[16] = (char)0x00; // Spare (not used)
        output[17] = (char)0x00; // Spare (not used)
        output[18] = (char)0x00; // Spare (not used)
        output[19] = genChecksum(output, 2, 19); // Checksum (sum of lower 8 bits from Type to Checksum)

        mNumBuses = 1;

        sendToSerial(output);

        setStatus(CANCon::CONNECTED);
        CANConStatus stats;
        stats.conStatus = getStatus();
        stats.numHardwareBuses = mNumBuses;
        emit status(stats);
}

void WaveshareSerial::disconnectDevice() {
        sendDebug("disconnectDevice()");
        if (serial != nullptr)
        {
            if (serial->isOpen())
            {
                //serial->clear();
                serial->close();
            }
            serial->disconnect(); //disconnect all signals
            delete serial;
            serial = nullptr;
        }

        setStatus(CANCon::NOT_CONNECTED);
        CANConStatus stats;
        stats.conStatus = getStatus();
        stats.numHardwareBuses = mNumBuses;
        emit status(stats);
}

void WaveshareSerial::serialError(QSerialPort::SerialPortError err)
{
        sendDebug("serialError()");
        QString errMessage;
        bool killConnection = false;
        switch (err)
        {
        case QSerialPort::NoError:
            return;
        case QSerialPort::DeviceNotFoundError:
            errMessage = "Device not found error on serial";
            killConnection = true;
            piStop();
            break;
        case QSerialPort::PermissionError:
            errMessage =  "Permission error on serial port";
            killConnection = true;
            piStop();
            break;
        case QSerialPort::OpenError:
            errMessage =  "Open error on serial port";
            killConnection = true;
            piStop();
            break;
#if QT_VERSION <= QT_VERSION_CHECK( 6, 0, 0 )
        case QSerialPort::ParityError:
            errMessage = "Parity error on serial port";
            break;
        case QSerialPort::FramingError:
            errMessage = "Framing error on serial port";
            break;
        case QSerialPort::BreakConditionError:
            errMessage = "Break error on serial port";
            break;
#endif
        case QSerialPort::WriteError:
            errMessage = "Write error on serial port";
            piStop();
            break;
        case QSerialPort::ReadError:
            errMessage = "Read error on serial port";
            piStop();
            break;
        case QSerialPort::ResourceError:
            errMessage = "Serial port seems to have disappeared.";
            killConnection = true;
            piStop();
            break;
        case QSerialPort::UnsupportedOperationError:
            errMessage = "Unsupported operation on serial port";
            killConnection = true;
            break;
        case QSerialPort::UnknownError:
            errMessage = "Beats me what happened to the serial port.";
            killConnection = true;
            piStop();
            break;
        case QSerialPort::TimeoutError:
            errMessage = "Timeout error on serial port";
            killConnection = true;
            break;
        case QSerialPort::NotOpenError:
            errMessage = "The serial port isn't open";
            killConnection = true;
            piStop();
            break;
        }
        /*
    if (serial)
    {
        serial->clearError();
        serial->flush();
        serial->close();
    }*/
        if (errMessage.length() > 1)
        {
            sendDebug(errMessage);
        }
        if (killConnection)
        {
            qDebug() << "Shooting the serial object in the head. It deserves it.";
            disconnectDevice();
        }
}

void WaveshareSerial::readSerialData()
{
        sendDebug("readSerialData()");
        QByteArray data;
        unsigned char c;
        QString debugBuild;

        if (serial) data = serial->readAll();

        sendDebug("Got data from serial. Len = " % QString::number(data.length()));
        for (int i = 0; i < data.length(); i++)
        {
            c = data.at(i);
            //qDebug() << c << "    " << QString::number(c, 16) << "     " << QString(c);
            debugBuild = debugBuild % QString::number(c, 16).rightJustified(2,'0') % " ";
            procRXChar(c);
        }
        debugOutput(debugBuild);
        //qDebug() << debugBuild;
}

//Debugging data sent from connection window. Inject it into Comm traffic.
void WaveshareSerial::debugInput(QByteArray bytes) {
        sendDebug("debugInput()");
        sendToSerial(bytes);
}

void WaveshareSerial::procRXChar(unsigned char c)
{
        qDebug() << "procRXChar() c: 0x" + QString::number((int)c,16).rightJustified(2,'0');
        rx_count += 1;
        QByteArray output;
        bool getCommandHandled = false;

        switch (rx_state)
        {
        case WS_IDLE:
            if (c == 0xAA)
            {
                rx_state = WS_GET_COMMAND;
                rx_count = 1;
                qDebug() << "procRXChar() 0xAA switching to GET COMMAND";
            }
            break;
        case WS_GET_COMMAND:
            qDebug() << "procRXChar() GET COMMAND c & 0xF0: " + QString::number((int)c & 0xF0,16).rightJustified(2,'0');
            switch (c)
            {
                case 0x55:
                    // Expecting either a 20 byte command or a 20 byte fixed length transmission
                    qDebug() << "procRXChar() 0x55 staying on WS_GET_COMMAND";
                    rx_state = WS_GET_COMMAND; // No change
                    rx_expected_count = 20;
                    getCommandHandled = true;
                    break;
                case 0x04:
                    qDebug() << "procRXChar() 0x04 switching to GET CANBUS PARAMS";
                    rx_state = WS_GET_CANBUS_PARAMS;
                    rx_expected_count = 20;
                    break;
                case 0x06:
                    qDebug() << "procRXChar() 0x06 switching to SET SERIAL PARAMS";
                    rx_state = WS_SET_SERIAL_PARAMS;
                    rx_expected_count = 20;
                    break;
            }
            if(getCommandHandled)
            {
                    break;
            }

            switch (c & 0xF0)
            {
                case 0xC0: //receiving a can frame standard length
                    qDebug() << "procRXChar() 0xCx switching to BUILD CAN FRAME";
                    rx_state = WS_BUILD_CAN_FRAME;
                    rx_step = 0;
                    buildData.resize(c & 0xF);
                    buildFrame.setExtendedFrameFormat(false);
                    buildFrame.bus = 0;
                    break;
                case 0xE0: // receiving a can frame extended length
                    qDebug() << "procRXChar() 0xEx switching to BUILD CAN FRAME";
                    rx_state = WS_BUILD_CAN_FRAME;
                    rx_step = 0;
                    buildData.resize(c & 0xF);
                    buildFrame.setExtendedFrameFormat(true);
                    buildFrame.bus = 0;
                    break;
            }
            break;
        case WS_BUILD_CAN_FRAME:
            /*if(c == 0x55)
            {
                qDebug() << "procRXChar() WS_BUILD_CAN_FRAME 0x55 end of frame received, switching to IDLE";
                rx_state = WS_IDLE;
                break;
            }*/
            switch (rx_step)
            {
            case 0:
                qDebug() << "procRXChar() WS_BUILD_CAN_FRAME step 0";
                buildTimestamp = QDateTime::currentMSecsSinceEpoch() * 1000l;
                buildFrame.setTimeStamp(QCanBusFrame::TimeStamp(0, buildTimestamp));
                buildId = c;
                break;
            case 1:
                qDebug() << "procRXChar() WS_BUILD_CAN_FRAME step 1";
                buildId |= c << 8;
                if(!buildFrame.hasExtendedFrameFormat())
                {
                    buildFrame.setFrameId(buildId);
                }
                break;
            case 2:
                qDebug() << "procRXChar() WS_BUILD_CAN_FRAME step 2";
                if(buildFrame.hasExtendedFrameFormat())
                {
                    buildId |= c << 16;
                    break;
                }
                else
                {
                    //fallthrough
                }
            case 3:
                qDebug() << "procRXChar() WS_BUILD_CAN_FRAME step 3";
                if(buildFrame.hasExtendedFrameFormat())
                {
                    buildId |= c << 24;
                    buildFrame.setFrameId(buildId);
                    break;
                }
                else
                {
                    // fallthrough
                }
            default:
                qDebug() << "procRXChar() WS_BUILD_CAN_FRAME default";
                qDebug() << "procRXChar() WS_BUILD_CAN_FRAME rx_step: " + QString::number(rx_step) + ", buildData.length: " + QString::number(buildData.length()) + ", extendedFrame: " + QString::number(buildFrame.hasExtendedFrameFormat() ? 2 : 0);
                if (rx_step < buildData.length() + 2 + (buildFrame.hasExtendedFrameFormat() ? 2 : 0))
                {
                    qDebug() << "procRXChar() WS_BUILD_CAN_FRAME rx_step < buildData.length + 2 + 0";
                    buildData[rx_step - (2 + (buildFrame.hasExtendedFrameFormat() ? 2 : 0))] = c;
                    if (rx_step == buildData.length() + 2 + (buildFrame.hasExtendedFrameFormat() ? 2 : 0) - 1) //it's the last data byte so immediately process the frame
                    {
                        qDebug() << "procRXChar() WS_BUILD_CAN_FRAME rx_step == buildData.length + 2 + 0 - 1";
                        rx_state = WS_IDLE;
                        rx_step = 0;
                        buildFrame.isReceived = true;
                        buildFrame.setPayload(buildData);
                        buildFrame.setFrameType(QCanBusFrame::FrameType::DataFrame);
                        if (!isCapSuspended())
                        {
                            /* get frame from queue */
                            CANFrame* frame_p = getQueue().get();
                            if(frame_p) {
                                //qDebug() << "GVRET got frame on bus " << frame_p->bus;
                                /* copy frame */
                                *frame_p = buildFrame;
                                checkTargettedFrame(buildFrame);
                                /* enqueue frame */
                                getQueue().queue();
                            }
                            else
                                qDebug() << "can't get a frame, ERROR";
                        }
                    }
                }
                else //should never get here! But, just in case, reset the comm
                {
                    rx_state = WS_IDLE;
                    rx_step = 0;
                }
                break;
            }
            rx_step++;
            break;
        case SERIALSTATE::WS_GET_CANBUS_PARAMS:
        case SERIALSTATE::WS_SET_SERIAL_PARAMS:
            break;
        }

        if(rx_state != SERIALSTATE::WS_IDLE && rx_count == rx_expected_count)
        {
            qDebug() << "procRXChar() WS_GET_COMMAND rx_count == rx_expected_count, resetting to IDLE";
            rx_state = WS_IDLE;
            rx_step = 0;
            rx_count = 0;
        }
}
