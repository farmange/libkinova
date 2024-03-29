#include "kinovadrv.h"
#include "logger.h"
#include "Kinova.API.USBCommLayerUbuntu.h"

#include <iostream>
#include <sstream>
#include <sys/ioctl.h>
#include <linux/serial.h>
using namespace std;

namespace KinovaApi
{

    CommLayer::CommLayer()
    {
        init_flag_ = false;
        return;
    }

    CommLayer::CommStatus_t CommLayer::commLayerClose()
    {
        // print(DEBUG) << "Close the RS485 port..." << endl;
        LOG_DEBUG("Close the RS485 port...");
        if (init_flag_ == false)
        {
            LOG_ERROR("Device not initialize...");
            return COMM_INIT_ERROR;
        }
#ifndef DEBUG_STUB_MSG
        close(serial_port_);
#endif
        return COMM_OK;
    }

    CommLayer::CommStatus_t CommLayer::commLayerInit(std::string device, speed_t baudrate)
    {
        device_ = device;
        baudrate_ = baudrate;
        init_flag_ = false;
        LOG_DEBUG_STREAM("Initialize RS485 interface on " << device_.c_str() << "...");
#ifndef DEBUG_STUB_MSG
        serial_port_ = open(device_.c_str(), O_RDWR);
        if (serial_port_ < 0)
        {
            LOG_DEBUG_STREAM("Error " << errno << " from open: " << strerror(errno));
            return COMM_INIT_ERROR;
        }
        // Read in existing settings, and handle any error
        if (tcgetattr(serial_port_, &tty_settings_) != 0)
        {
            LOG_DEBUG_STREAM("Error " << errno << " from tcgetattr: " << strerror(errno));

            return COMM_INIT_ERROR;
        }

        tty_settings_.c_cflag |= PARENB; // Set parity bit
        // tty_settings_.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
        tty_settings_.c_cflag |= CSTOPB; // tow stop byte
        // tty_settings_.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
        tty_settings_.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
        tty_settings_.c_cflag |= CS8;            // 8 bits per byte (most common)
        tty_settings_.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
        tty_settings_.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty_settings_.c_cflag &= ~(CBAUD | CBAUDEX); // TESTTTTTTTT
        tty_settings_.c_cflag |= B115200;            // TESTTTTTTTT

        tty_settings_.c_lflag &= ~ICANON;
        tty_settings_.c_lflag &= ~ECHO;                                                        // Disable echo
        tty_settings_.c_lflag &= ~ECHOE;                                                       // Disable erasure
        tty_settings_.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
        tty_settings_.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
        tty_settings_.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
        tty_settings_.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes
        tty_settings_.c_oflag &= ~OPOST;                                                       // Prevent special interpretation of output bytes (e.g. newline chars)
        tty_settings_.c_oflag &= ~ONLCR;                                                       // Prevent conversion of newline to carriage return/line feed

        // tty_settings_.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty_settings_.c_cc[VTIME] = 0; // Non blocking read
        tty_settings_.c_cc[VMIN] = 0;

        // Set in/out baud rate
        if (cfsetispeed(&tty_settings_, baudrate_) != 0)
        {
            // print(ERROR, "Baud rate setting (cfsetispeed) failed");
            // print(DEBUG) << "Baud rate setting (cfsetispeed) failed" << endl;
            LOG_DEBUG("Baud rate setting (cfsetispeed) failed");
        }
        if (cfsetospeed(&tty_settings_, baudrate_) != 0)
        {
            // print(ERROR, "Baud rate setting (cfsetospeed) failed");
            LOG_DEBUG("Baud rate setting (cfsetospeed) failed");
        }

        // Save tty_settings_ settings, also checking for error
        if (tcsetattr(serial_port_, TCSANOW, &tty_settings_) != 0)
        {
            LOG_DEBUG_STREAM("Error " << errno + " from tcsetattr: " << strerror(errno));
            return COMM_INIT_ERROR;
        }
#endif
        init_flag_ = true;

        return COMM_OK;
    }

    CommLayer::CommStatus_t CommLayer::commLayerRead(message_t *message_buffer, const int nb_msg_to_read, int &nb_msg_read)
    {
        if (init_flag_ == false)
        {
            return COMM_INIT_ERROR;
        }
        LOG_DEBUG("CommLayerRead");
        memset(message_buffer, '\0', nb_msg_to_read * sizeof(message_t));
#ifdef DEBUG_STUB_MSG
        int bytes_read = nb_msg_to_read * sizeof(message_t);
#else
        int bytes_read = read(serial_port_, message_buffer, nb_msg_to_read * sizeof(message_t));
#endif
        if (bytes_read <= 0)
        {
            return COMM_READ_ERROR;
        }
        nb_msg_read = bytes_read / sizeof(message_t);
        LOG_DEBUG_STREAM("Read " << nb_msg_read << " message (" << bytes_read << " bytes)");
        LOG_HEXDUMP(Logger::DEBUG, "message_buffer", message_buffer, nb_msg_read * sizeof(message_t));
        return COMM_OK;
    }

    CommLayer::CommStatus_t CommLayer::commLayerWrite(message_t *message_buffer, const int nb_msg_to_send, int &nb_msg_sent)
    {
        // print(DEBUG) << "CommLayerWrite" << endl;
        if (init_flag_ == false)
        {
            return COMM_INIT_ERROR;
        }
        LOG_DEBUG("CommLayerWrite");
#ifdef DEBUG_STUB_MSG
        int bytes_sent = nb_msg_to_send * sizeof(message_t);
        memset(message_buffer, '\0', bytes_sent);
#else
        int bytes_sent = write(serial_port_, message_buffer, nb_msg_to_send * sizeof(message_t));
#endif
        if (bytes_sent <= 0)
        {
            return COMM_WRITE_ERROR;
        }
        nb_msg_sent = bytes_sent / sizeof(message_t);
        LOG_DEBUG_STREAM("Write " << nb_msg_sent << " message (" << bytes_sent << " bytes)");
        LOG_HEXDUMP(Logger::DEBUG, "message_buffer", message_buffer, nb_msg_sent * sizeof(message_t));
        return COMM_OK;
    }

    APILayer::APILayer()
    {
    }

    APILayer::ApiStatus_t APILayer::init(const std::string device, const bool &debug_log)
    {
        api_mutex_.lock();

        LOG_DEBUG_STREAM("Initialize APILayer instance... (debug logging : " << std::boolalpha << debug_log << ") ...");
        if (debug_log == false)
        {
            Logger::instance().setLevel(Logger::ERROR);
        }

        LOG_DEBUG_STREAM("Initialize CommLayer...");
        if (comm_.commLayerInit(device.c_str(), B115200) != CommLayer::COMM_OK)
        {
            return API_INIT_ERROR;
        }
        api_mutex_.unlock();

        return API_OK;
    }

    APILayer::ApiStatus_t APILayer::readWrite_(CommLayer::message_t *writeMessage, CommLayer::message_t *readMessage, const int &expectedResponseMsg)
    {
        int nb_msg_sent = 0;
        int nb_msg_read = 0;

        for (int i = 0; i < MAX_WRITE_RETRY; i++)
        {
            bool transmission_ok = false;
            comm_.commLayerWrite(writeMessage, 1, nb_msg_sent);
            if (nb_msg_sent != 1)
            {
                /* Try to send message again */
                usleep(2);
                LOG_DEBUG_STREAM("Fail to write message (nb_msg_sent=" << nb_msg_sent << ")");
                continue;
            }

            usleep(50);

            for (int j = 0; j < MAX_READ_RETRY; j++)
            {
                comm_.commLayerRead(readMessage, expectedResponseMsg, nb_msg_read);
                if (nb_msg_read != expectedResponseMsg)
                {
                    /* Try to receive message again */
                    usleep(2);
                    LOG_DEBUG_STREAM("Fail to read message (nb_msg_read=" << nb_msg_read << ")");
                    continue;
                }

                LOG_DEBUG_STREAM("Message successfully write and reply read");
                transmission_ok = true;
                break;
            }
            if (transmission_ok)
            {
                break;
            }
        }
        if (nb_msg_sent != 1)
        {
            return API_WRITE_ERROR;
        }
        if (nb_msg_read != expectedResponseMsg)
        {
            return API_READ_ERROR;
        }

        return API_OK;
    }

    const int APILayer::getExpectedReply_(uint16_t command)
    {
        switch (command)
        {
        case RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES:
            return 2;
            break;

        case RS485_MSG_GET_ACTUALPOSITION:
        case RS485_MSG_FEEDTHROUGH:
        case RS485_MSG_GET_POSITION_COMMAND:
        case RS485_MSG_GET_DEVICE_INFO:
        case RS485_MSG_GET_CODE_VERSION:
        case RS485_MSG_GET_TEMPERATURE:
        case RS485_MSG_SET_TEMPERATURE:
        case RS485_GET_ENCODER_STATUSSES:
        case RS485_MSG_SET_ADDRESS:
        case RS485_MSG_CLEAR_FAULT_FLAG:
        case RS485_MSG_STAR_ASSERV:
        case RS485_MSG_STOP_ASSERV:
        case RS485_MSG_POSITION_MAX_MIN:
        case RS485_MSG_KP_GAIN:
        case RS485_MSG_KI_KD_GAIN:
        case RS485_MSG_PROGRAM_JOINT_ZERO:
        case RS485_SET_PID_FILTERS:
        case RS485_SET_ZERO_TORQUESENSOR:
        case RS485_SET_GAIN_TORQUESENSOR:
        case RS485_SET_CONTROL_WITH_ENCODER:
        case RS485_SET_PID_ADVANCED_PARAMETERS:
        case RS485_MSG_SPEED_ACCEL_MAX:
        case RS485_MSG_CURRENT_TORQUE_MAX:
            return 1;
            break;

        default:
            return 0;
            break;
        }
    }

    APILayer::ApiStatus_t
    APILayer::deviceInitialisation(const uint16_t &jointAddress, float &jointPosition)
    {
        api_mutex_.lock();

        ApiStatus_t status;
        CommLayer::message_t msgWrite, msgRead;
        msgWrite.Command = RS485_MSG_SET_ADDRESS;
        msgWrite.SourceAddress = 0x00;
        msgWrite.DestinationAddress = jointAddress;
        msgWrite.DataLong[0] = jointAddress;
        msgWrite.DataLong[1] = 0;
        msgWrite.DataLong[2] = 0;
        msgWrite.DataLong[3] = 0;

        status = readWrite_(&msgWrite, &msgRead, getExpectedReply_(RS485_MSG_SET_ADDRESS));
        if (status != API_OK)
        {
            api_mutex_.unlock();
            return status;
        }
        jointPosition = msgRead.DataFloat[1];
        api_mutex_.unlock();

        return API_OK;
    }

    APILayer::ApiStatus_t
    APILayer::clearError(const uint16_t &jointAddress)
    {
        api_mutex_.lock();

        ApiStatus_t status;
        CommLayer::message_t msgWrite, msgRead;
        msgWrite.Command = RS485_MSG_CLEAR_FAULT_FLAG;
        msgWrite.SourceAddress = 0x00;
        msgWrite.DestinationAddress = jointAddress;
        msgWrite.DataLong[0] = 0x00;
        msgWrite.DataLong[1] = 0x00;
        msgWrite.DataLong[2] = 0;
        msgWrite.DataLong[3] = 0;

        status = readWrite_(&msgWrite, &msgRead, getExpectedReply_(RS485_MSG_CLEAR_FAULT_FLAG));
        if (status != API_OK)
        {
            api_mutex_.unlock();
            return status;
        }
        api_mutex_.unlock();

        return API_OK;
    }

    APILayer::ApiStatus_t
    APILayer::startMotorControl(const uint16_t &jointAddress)
    {
        api_mutex_.lock();

        ApiStatus_t status;
        CommLayer::message_t msgWrite, msgRead;
        msgWrite.Command = RS485_MSG_STAR_ASSERV;
        msgWrite.SourceAddress = 0x00;
        msgWrite.DestinationAddress = jointAddress;
        msgWrite.DataLong[0] = jointAddress;
        msgWrite.DataLong[1] = RS485_MSG_STAR_ASSERV;
        msgWrite.DataLong[2] = 0;
        msgWrite.DataLong[3] = 0;

        status = readWrite_(&msgWrite, &msgRead, getExpectedReply_(RS485_MSG_STAR_ASSERV));
        if (status != API_OK)
        {
            api_mutex_.unlock();
            return status;
        }
        api_mutex_.unlock();

        return API_OK;
    }

    APILayer::ApiStatus_t
    APILayer::stopMotorControl(const uint16_t &jointAddress)
    {
        api_mutex_.lock();

        ApiStatus_t status;
        CommLayer::message_t msgWrite, msgRead;
        msgWrite.Command = RS485_MSG_STOP_ASSERV;
        msgWrite.SourceAddress = 0x00;
        msgWrite.DestinationAddress = jointAddress;
        msgWrite.DataLong[0] = jointAddress;
        msgWrite.DataLong[1] = RS485_MSG_STOP_ASSERV;
        msgWrite.DataLong[2] = 0;
        msgWrite.DataLong[3] = 0;

        status = readWrite_(&msgWrite, &msgRead, getExpectedReply_(RS485_MSG_STOP_ASSERV));
        if (status != API_OK)
        {
            api_mutex_.unlock();
            return status;
        }
        api_mutex_.unlock();

        return API_OK;
    }

    APILayer::ApiStatus_t
    APILayer::getActualPosition(const uint16_t &jointAddress, float &jointCurrent, float &jointPositionHall, float &jointSpeed, float &jointTorque)
    {
        api_mutex_.lock();

        ApiStatus_t status;
        CommLayer::message_t msgWrite, msgRead;
        msgWrite.Command = RS485_MSG_GET_ACTUALPOSITION;
        msgWrite.SourceAddress = 0x00;
        msgWrite.DestinationAddress = jointAddress;
        msgWrite.DataLong[0] = 0;
        msgWrite.DataLong[1] = 0;
        msgWrite.DataLong[2] = 0;
        msgWrite.DataLong[3] = 0;

        status = readWrite_(&msgWrite, &msgRead, getExpectedReply_(RS485_MSG_GET_ACTUALPOSITION));
        if (status != API_OK)
        {
            api_mutex_.unlock();
            return status;
        }
        jointCurrent = msgRead.DataFloat[0];
        jointPositionHall = msgRead.DataFloat[1];
        jointSpeed = msgRead.DataFloat[2];
        jointTorque = msgRead.DataFloat[3];
        api_mutex_.unlock();

        return API_OK;
    }

    APILayer::ApiStatus_t
    APILayer::setCommandAllValue(const uint16_t &jointAddress, const float &jointCommand, float &jointCurrent, float &jointPositionHall,
                                 float &jointSpeed, float &jointTorque, float &jointPMW, float &jointPositionOptical,
                                 short &jointAccelX, short &jointAccelY, short &jointAccelZ, short &jointTemp)
    {
        api_mutex_.lock();

        ApiStatus_t status;
        CommLayer::message_t msgWrite, msgRead[2];
        msgWrite.Command = RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES;
        msgWrite.SourceAddress = 0x00;
        msgWrite.DestinationAddress = jointAddress;
        msgWrite.DataFloat[0] = jointCommand;
        msgWrite.DataFloat[1] = jointCommand;
        msgWrite.DataLong[2] = 0;
        msgWrite.DataLong[3] = 0;

        status = readWrite_(&msgWrite, &msgRead[0], getExpectedReply_(RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES));
        if (status != API_OK)
        {
            api_mutex_.unlock();
            return status;
        }
        jointCurrent = msgRead[0].DataFloat[0];
        jointPositionHall = msgRead[0].DataFloat[1];
        jointSpeed = msgRead[0].DataFloat[2];
        jointTorque = msgRead[0].DataFloat[3];

        jointPMW = msgRead[1].DataFloat[0];
        jointPositionOptical = msgRead[1].DataFloat[1];
        jointAccelX = msgRead[1].DataShort[4];
        jointAccelY = msgRead[1].DataShort[5];
        jointAccelZ = msgRead[1].DataShort[6];
        jointTemp = msgRead[1].DataShort[7];
        api_mutex_.unlock();

        return API_OK;
    }

    APILayer::ApiStatus_t
    APILayer::setPositionCommand(const uint16_t &jointAddress, const float &jointCommand, float &jointCurrent, float &jointPositionHall,
                                 float &jointSpeed, float &jointTorque)
    {
        api_mutex_.lock();

        ApiStatus_t status;
        CommLayer::message_t msgWrite, msgRead[2];
        msgWrite.Command = RS485_MSG_GET_POSITION_COMMAND;
        msgWrite.SourceAddress = 0x00;
        msgWrite.DestinationAddress = jointAddress;
        msgWrite.DataFloat[0] = jointCommand;
        msgWrite.DataFloat[1] = jointCommand;
        msgWrite.DataLong[2] = 0;
        msgWrite.DataLong[3] = 0;

        status = readWrite_(&msgWrite, &msgRead[0], getExpectedReply_(RS485_MSG_GET_POSITION_COMMAND));
        if (status != API_OK)
        {
            api_mutex_.unlock();
            return status;
        }
        jointCurrent = msgRead[0].DataFloat[0];
        jointPositionHall = msgRead[0].DataFloat[1];
        jointSpeed = msgRead[0].DataFloat[2];
        jointTorque = msgRead[0].DataFloat[3];

        api_mutex_.unlock();

        return API_OK;
    }

    APILayer::ApiStatus_t APILayer::getPosition()
    {
        api_mutex_.lock();
        api_mutex_.unlock();

        return API_OK;
    }

} // namespace KinovaApi

// a statically declared instance of our derived factory class
ApiFactory Factory;
