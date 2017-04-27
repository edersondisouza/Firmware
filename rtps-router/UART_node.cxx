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
#include <string>
#include <errno.h>

#include <asm/termbits.h>
#include <sys/ioctl.h>

#include <iostream>

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
    struct termios2 tc;
    int fd;

    fd = ::open(uart_name, O_RDWR|O_NONBLOCK|O_CLOEXEC|O_NOCTTY);
    if (fd < 0) {
        printf("Could not open %s (%m)", uart_name);
        return -1;
    }

    bzero(&tc, sizeof(tc));

    if (ioctl(fd, TCGETS2, &tc) == -1) {
        printf("Could not get termios2 (%m)");
        goto fail;
    }

    tc.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    tc.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
    tc.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG | TOSTOP);
    tc.c_cflag &= ~(CSIZE | PARENB | CBAUD | CRTSCTS);
    tc.c_cflag |= CS8 | BOTHER;

    tc.c_cc[VMIN] = 0;
    tc.c_cc[VTIME] = 0;
    tc.c_ispeed = baudrate;
    tc.c_ospeed = baudrate;

    if (ioctl(fd, TCSETS2, &tc) == -1) {
        printf("Could not set terminal attributes (%m)");
        goto fail;
    }

    if (ioctl(fd, TCFLSH, TCIOFLUSH) == -1) {
        printf("Could not flush terminal (%m)");
        goto fail;
    }

    printf("Open UART [%d] %s:%u *", fd, uart_name, baudrate);

    m_uart_filestream = fd;
    return m_uart_filestream;

fail:
    ::close(fd);
    fd = -1;
    return -1;
}


uint8_t UART_node::close_uart()
{
    printf("Close UART\n");
    close(m_uart_filestream);
    return 0;
}

void UART_node::_read()
{
    int r = ::read(m_uart_filestream, _buf + _buf_size, sizeof(_buf) - _buf_size);
    if (r < 0) {
        std::cerr << "Error reading UART " << errno << std::endl;
        return;
    }

    std::cout << "Read " << r << " bytes" << std::endl;
    _buf_size += r;
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
