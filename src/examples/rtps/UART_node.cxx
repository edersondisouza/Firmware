// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string>
#include <errno.h>

//#include <iostream>

#include "UART_node.h"

#define DEFAULT_UART "/dev/ttyACM0"

UART_node::UART_node(): m_uart_filestream(0)
{

}

UART_node::~UART_node()
{
    close_uart();
}

int UART_node::init_uart(const char * uart_name, uint32_t baudrate)
{

    // Open a serial port
    m_uart_filestream = open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (m_uart_filestream < 0)
    {
        printf("failed to open uart: %s\n", uart_name);
        return 1;
    }

    // Try to set baud rate
    struct termios uart_config;
    int termios_state;
    // Back up the original uart configuration to restore it after exit
    if ((termios_state = tcgetattr(m_uart_filestream, &uart_config)) < 0)
    {
        printf("ERR GET CONF %s: %d\n", uart_name, termios_state);
        close(m_uart_filestream);
        return -1;
    }

    // Clear ONLCR flag (which appends a CR for every LF)
    uart_config.c_oflag &= ~ONLCR;

    // USB serial is indicated by /dev/ttyACM0
    if (strcmp(uart_name, "/dev/ttyACM0") != 0 && strcmp(uart_name, "/dev/ttyACM1") != 0)
    {
        // Set baud rate
        if (cfsetispeed(&uart_config, baudrate) < 0 || cfsetospeed(&uart_config, baudrate) < 0)
        {
            printf("ERR SET BAUD %s: %d\n", uart_name, termios_state);
            close(m_uart_filestream);
            return -1;
        }
    }

    if ((termios_state = tcsetattr(m_uart_filestream, TCSANOW, &uart_config)) < 0)
    {
        printf("ERR SET CONF %s\n", uart_name);
        close(m_uart_filestream);
        return -1;
    }

    return m_uart_filestream;
}


uint8_t UART_node::close_uart()
{
    printf("Close UART\n");
    close(m_uart_filestream);
    return 0;
}

void UART_node::_read()
{
    int size;
    size = ::read(m_uart_filestream, _buf + _buf_size, sizeof(_buf) - _buf_size);

    printf("Read %d bytes", size);
    if (size > 0)
        _buf_size += size;
}

uint8_t UART_node::readFromUART(char* topic_ID, uint8_t *seq, char buffer[])
{
    size_t start = 0;
    uint8_t len;
    if (m_uart_filestream == -1) return 2;

    _read();

    if (_buf_size < 6) // starting ">>>" + topic + seq + len
        return 0;

    // look for starting ">>>"
    while (start < (_buf_size - 3) && strncmp(_buf + start, ">>>", 3) != 0) {
        start++;
    }
    if (start > (_buf_size - 3)) {
        // no starting marker, time to flush buffer
        _buf_size = 0;
        return 0;
    }

    // Discard everything up to start
    if (start > 0) {
        memmove(_buf, _buf + start, _buf_size - start);
        _buf_size -= start;
    }

    len = _buf[5];
    if (len > _buf_size)
        return 0; // we don't have a complete msg yet

    // Found a whole message, send it and remove from local _buf
    *topic_ID = _buf[3];
    *seq = _buf[4];
    memmove(buffer, _buf + 6, len);

    _buf_size -= len + 6;
    memmove(_buf, _buf + 6 + len, _buf_size);

    return len;
}

uint8_t UART_node::writeToUART(const char topic_ID, char buffer[], uint32_t length)
{
    if (m_uart_filestream == -1) return 2;

    static uint8_t seq = 0;
    static const char pre[] = ">>>";
    uint8_t len = length;

    write(m_uart_filestream, pre, 3);
    write(m_uart_filestream, &topic_ID, 1);    // topic_ID
    write(m_uart_filestream, &seq, 1);    // seq
    write(m_uart_filestream, &len, 1);
    write(m_uart_filestream, buffer, length);

    seq++;

    return 0;
}
