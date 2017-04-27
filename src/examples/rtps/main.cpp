#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <termios.h>

#include <microcdr/microCdr.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>

#include "UART_node.h"
#include "topic-id.h"

void *uart_reader_thread(void *data);
int _rtps_main(int argc, char *argv[]);
int subscribe_topic(const orb_id_t topic);

void handle_vehicle_status(int fd, UART_node &uart);
void handle_sensor_combined(int fd, UART_node &uart);
void handle_vehicle_command_ack(int fd, UART_node &uart);

void handle_vehicle_command(struct reader_data *reader_data, char *buffer);

extern "C" __EXPORT int rtps_main(int argc, char *argv[]);

static int _started = 0;

struct reader_data {
    UART_node &uart_node;
    orb_advert_t vehicle_command_pub; //TODO make it a list?
};

void handle_vehicle_command(struct reader_data *reader_data, char *buffer)
{
    struct vehicle_command_s vehicle_command_data;

    deserialize_vehicle_command(&vehicle_command_data, buffer);
    orb_publish(ORB_ID(vehicle_command), reader_data->vehicle_command_pub, &vehicle_command_data);
}

void *uart_reader_thread(void *data)
{
    struct reader_data *reader_data = (struct reader_data *)data;
    char buffer[255];

    struct pollfd fds[1] = {};
    fds[0].fd = reader_data->uart_node.get_fd();
    fds[0].events = POLLIN;
    uint8_t len;

    char topic_ID = 255;
    while (true) {
        if (poll(&fds[0], 1, 1000) > 0) {
            uint8_t seq;
            if ((len = reader_data->uart_node.readFromUART(&topic_ID, &seq, buffer)) != 0) {
                if (topic_ID == VehicleCommandId) {
                    handle_vehicle_command(reader_data, buffer);
                }
            }
        }
    }

    return nullptr;
}

void handle_sensor_combined(int fd, UART_node &uart)
{
    char buffer[MICROCDR_INIT_BUF_LENGTH];
    struct sensor_combined_s data;
    uint32_t length = 0;

    /* copy raw data into local buffer */
    orb_copy(ORB_ID(sensor_combined), fd, &data);

    serialize_sensor_combined(&data, buffer, &length);

    uart.writeToUART((char)SensorCombinedId, buffer, length);
}

void handle_vehicle_status(int fd, UART_node &uart)
{
    char buffer[MICROCDR_INIT_BUF_LENGTH];
    struct vehicle_status_s data;
    uint32_t length = 0;

    /* copy raw data into local buffer */
    orb_copy(ORB_ID(vehicle_status), fd, &data);

    serialize_vehicle_status(&data, buffer, &length);

    uart.writeToUART((char)VehicleStatusId, buffer, length);
}

void handle_vehicle_command_ack(int fd, UART_node &uart)
{
    char buffer[MICROCDR_INIT_BUF_LENGTH];
    struct vehicle_command_ack_s data;
    uint32_t length = 0;

    /* copy raw data into local buffer */
    orb_copy(ORB_ID(vehicle_command_ack), fd, &data);

    serialize_vehicle_command_ack(&data, buffer, &length);

    uart.writeToUART((char)VehicleCommandAckId, buffer, length);
}

int subscribe_topic(const orb_id_t topic)
{
    int fd = orb_subscribe(topic);
    orb_set_interval(fd, 500);
    return fd;
}

int _rtps_main(int argc, char *argv[])
{
    UART_node uart;

    uint32_t baudrate = strtoul(argv[3], nullptr, 10);

    uart.init_uart(argv[2], baudrate);
    pthread_t _reader_thread{};

    int fds[3];

    /* subcribe to topics */
    fds[0] = subscribe_topic(ORB_ID(vehicle_status));
    fds[1] = subscribe_topic(ORB_ID(sensor_combined));
    fds[2] = subscribe_topic(ORB_ID(vehicle_command_ack));

    /* advertise topics */
    struct vehicle_command_s vehicle_command_data { };
    orb_advert_t vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &vehicle_command_data);

    /* Set up and start receiver thread */
    pthread_attr_t reader_attr;

    struct sched_param param;
    (void)pthread_attr_getschedparam(&reader_attr, &param);
    param.sched_priority = SCHED_PRIORITY_MAX - 80;
    (void)pthread_attr_setschedparam(&reader_attr, &param);

    pthread_attr_init(&reader_attr);
    pthread_attr_setstacksize(&reader_attr, PX4_STACK_ADJUSTED(2100));

    struct reader_data reader_data {
        .uart_node = uart,
        .vehicle_command_pub = vehicle_command_pub
    };
    pthread_create(&_reader_thread, &reader_attr, uart_reader_thread, &reader_data);

    /* Sender loop */
    _started++;
    for (;;)
    {
        bool updated;

        // TODO make this more genereric
        orb_check(fds[0], &updated);
        if (updated)
            handle_vehicle_status(fds[0], uart);

        orb_check(fds[1], &updated);
        if (updated)
            handle_sensor_combined(fds[1], uart);

        orb_check(fds[2], &updated);
        if (updated)
            handle_vehicle_command_ack(fds[2], uart);

        // Sleep instead of poll as we'd have to sleep anyway
        // to give time to low prio threads
        usleep(500000);
    }
    pthread_join(_reader_thread, nullptr);
    uart.close_uart();

    PX4_INFO("exiting");
    fflush(stdout);
    return OK;
}

int rtps_main(int argc, char *argv[]) {
    char buf[] = "rtps_pro";

    px4_task_spawn_cmd(buf,
            SCHED_DEFAULT,
            SCHED_PRIORITY_DEFAULT,
            2500,
            (px4_main_t) _rtps_main,
            (char *const *)argv);

    // TODO have a maximum time waiting
    while (!_started) {
        ::usleep(500);
    }

    return OK;

}
