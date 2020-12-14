#include "kinovadrv.h"
#include "logger.h"

#include <stdio.h>
#include <getopt.h>
#include <iostream>
#include <sstream>

using namespace KinovaApi;

/* Program to communicate with usb to serial bridge */
void print_usage()
{
    printf("Usage: test -d device\n");
}

int main(int argc, char *argv[])
{
    int opt = 0;
    std::string device = "";
    Logger::logging_level_t log_level = Logger::INFO;
    //Specifying the expected options
    //The two options l and b expect numbers as argument
    static struct option long_options[] = {
        {"dev", required_argument, 0, 'd'},
        {"verbose", required_argument, 0, 'v'},
        {0, 0, 0, 0}};

    int long_index = 0;
    while ((opt = getopt_long(argc, argv, "d:v:",
                              long_options, &long_index)) != -1)
    {

        switch (opt)
        {
        case 'd':
            device = optarg;
            break;
        case 'v':
            log_level = Logger::logging_level_t(atoi(optarg));
            break;
        default:
            print_usage();
            exit(EXIT_FAILURE);
        }
    }
    Logger::instance().setLevel(log_level);

    // return 0;
    if (device == "")
    {
        print_usage();
        exit(EXIT_FAILURE);
    }

    CommLayer rs485;
    if (rs485.commLayerInit(device, B115200) != 0)
    {
        return 0;
    }

    CommLayer::message_t data_to_send;
    int nb_message = 1;
    float JointCommandComm = 0;
    int count = 0;

    while (1)
    {
        data_to_send.Command = 0x0; // DeviceInitialisation 0x01//GetActualPosition
        data_to_send.SourceAddress = 0x00;
        data_to_send.DestinationAddress = 0x11; //17
        data_to_send.DataLong[0] = 0x11;
        data_to_send.DataLong[1] = 0;
        data_to_send.DataLong[2] = 0.0;
        data_to_send.DataLong[3] = 0.0;
        rs485.commLayerWrite(&data_to_send, 1, nb_message);
        usleep(100);

        CommLayer::message_t data_to_read;
        memset(&data_to_read, '\0', sizeof(CommLayer::message_t));
        int nb_msg_read = 0;
        rs485.commLayerRead(&data_to_read, 1, nb_msg_read);
        if (nb_msg_read == 1)
        {
            /* message received */
            JointCommandComm = data_to_read.DataFloat[1];
            LOG_INFO_STREAM("initial position optical = " << JointCommandComm);
            break;
        }

        usleep(100);
    }
    usleep(1000000);

    while (count < 10)
    {
        LOG_INFO("========================================");

        JointCommandComm += 60 * (0.0025);
        LOG_INFO_STREAM("JointCommandComm = " << JointCommandComm);
        data_to_send.Command = 0x14; // DeviceInitialisation 0x01//GetActualPosition
        data_to_send.SourceAddress = 0x00;
        data_to_send.DestinationAddress = 0x11; //17
        data_to_send.DataFloat[0] = JointCommandComm;
        data_to_send.DataFloat[1] = JointCommandComm;
        data_to_send.DataLong[2] = 0.0;
        data_to_send.DataLong[3] = 0.0;
        rs485.commLayerWrite(&data_to_send, 1, nb_message);

        usleep(1000);
        LOG_INFO("   -------------------------");

        CommLayer::message_t data_to_read[2];
        memset(&data_to_read[0], '\0', 2 * sizeof(CommLayer::message_t));
        int nb_msg_read = 0;
        rs485.commLayerRead(&data_to_read[0], 1, nb_msg_read);
        if (nb_msg_read == 1)
        {
            /* message received */
            LOG_INFO_STREAM("current = " << data_to_read[0].DataFloat[0]);
            LOG_INFO_STREAM("position halls = " << data_to_read[0].DataFloat[1]);
            LOG_INFO_STREAM("speed = " << data_to_read[0].DataFloat[2]);
            LOG_INFO_STREAM("torque = " << data_to_read[0].DataFloat[3]);
        }
        usleep(100);
        LOG_INFO("   -------------------------");
        nb_msg_read = 0;
        rs485.commLayerRead(&data_to_read[0], 1, nb_msg_read);
        if (nb_msg_read == 1)
        {
            /* message received */
            LOG_INFO_STREAM("pwm = " << data_to_read[0].DataFloat[0]);
            LOG_INFO_STREAM("position optical = " << data_to_read[0].DataFloat[1]);
            LOG_INFO_STREAM("accelX = " << data_to_read[0].DataShort[4]);
            LOG_INFO_STREAM("accelY = " << data_to_read[0].DataShort[5]);
            LOG_INFO_STREAM("accelZ = " << data_to_read[0].DataShort[6]);
            LOG_INFO_STREAM("temp = " << data_to_read[0].DataShort[7]);
        }

        usleep(9000);
        count++;
    }

    rs485.commLayerClose();

    usleep(1500000);
    APILayer api;
    api.init();
    float position;
    APILayer::ApiStatus_t status = api.deviceInitialisation(0x11, position);
    if (status == APILayer::API_OK)
    {
        LOG_INFO_STREAM("DeviceInitialisation position: " << position);
    }
    else
    {
        LOG_INFO_STREAM("DeviceInitialisation error: " << status);
    }

    uint16_t jointAddress = 0x11;
    float jointCommand = position;

    float jointCurrent = 0;
    float jointPositionHall = 0;
    float jointSpeed = 0;
    float jointTorque = 0;
    float jointPMW = 0;
    float jointPositionOptical = 0;
    short jointAccelX = 0;
    short jointAccelY = 0;
    short jointAccelZ = 0;
    short jointTemp = 0;
    count = 0;
    while (count < 10)
    {
        jointCommand -= 60 * (0.0025);

        api.setCommandAllValue(jointAddress, jointCommand, jointCurrent, jointPositionHall,
                               jointSpeed, jointTorque, jointPMW, jointPositionOptical,
                               jointAccelX, jointAccelY, jointAccelZ, jointTemp);
        LOG_INFO_STREAM("jointAddress :" << jointAddress);
        LOG_INFO_STREAM("jointCommand :" << jointCommand);
        LOG_INFO_STREAM("jointCurrent :" << jointCurrent);
        LOG_INFO_STREAM("jointPositionHall :" << jointPositionHall);
        LOG_INFO_STREAM("jointSpeed :" << jointSpeed);
        LOG_INFO_STREAM("jointTorque :" << jointTorque);
        LOG_INFO_STREAM("jointPMW :" << jointPMW);
        LOG_INFO_STREAM("jointPositionOptical :" << jointPositionOptical);
        LOG_INFO_STREAM("jointAccelX :" << jointAccelX);
        LOG_INFO_STREAM("jointAccelY :" << jointAccelY);
        LOG_INFO_STREAM("jointAccelZ :" << jointAccelZ);
        LOG_INFO_STREAM("jointTemp :" << jointTemp);
        usleep(9000);
        count++;
    }

    KinovaApi::APILayer *api_ptr = create();
    api_ptr->init();
    return 0;
}