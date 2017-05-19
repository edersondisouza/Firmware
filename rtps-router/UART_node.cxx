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

using namespace std;

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

    printf("Open UART [%d] %s:%u *\n", fd, uart_name, baudrate);

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

void UART_node::read()
{
    /* Discard whole buffer if it's filled beyond a threshold,
     * This should prevent buffer being filled by garbage that
     * no reader (MAVLink or RTPS) can understand.
     *
     * TODO: a better approach would be checking if both reader
     * start understanding messages beyond a certain buffer size,
     * meaning that everything before is garbage.
     */
    if (_buf_size > BUFFER_THRESHOLD) {
        _buf_size = 0;
    }

    int r = ::read(m_uart_filestream, _buf + _buf_size, sizeof(_buf) - _buf_size);
    if (r < 0) {
        std::cerr << "Error reading UART " << errno << std::endl;
        return;
    }

    // std::cout << "Read " << r << " bytes. Former buffer size: " << _buf_size << ": now " << _buf_size + r << std::endl;
    _buf_size += r;
}

int UART_node::parseMavlinkFromUART(char buffer[], size_t buflen)
{
    size_t start = 0;
    uint8_t len;
    int i;

    if (m_uart_filestream == -1) return 2;

    if (_buf_size < 3)
        return 0;

    // Search for a mavlink packet on buffer to send it
    i = 0;
    while (i < (_buf_size - 3)
            && _buf[i] != 253
            && _buf[i] != 254)
        i++;


    // We need at least the first three bytes to get packet len
    if (i == _buf_size - 3) {
        return 0;
    }

    uint16_t packet_len;
    if (_buf[i] == 253) {
        uint8_t payload_len = _buf[i + 1];
        uint8_t incompat_flags = _buf[i + 2];
        packet_len = payload_len + 12;

        if (incompat_flags & 0x1) { //signing
            packet_len += 13;
        }
    } else {
        packet_len = _buf[i + 1] + 8;
    }

    // cerr << "mavlink parse. message len " << (int)packet_len << " on: " << i << endl;

    // packet is bigger than what we've read, better luck next time
    if (i + packet_len > _buf_size) {
        return 0;
    }

    // buffer should be big enough to hold a mavlink packet
    if (packet_len > buflen) {
        return -EMSGSIZE;
    }

    // Found a whole message, send it and remove from local _buf
    memmove(buffer, _buf + i, packet_len);
    memmove(_buf + i, _buf + i + packet_len, sizeof(_buf) - i - packet_len);
    _buf_size -= packet_len;

    return packet_len;
}

int UART_node::parseRTPSfromUART(char* topic_ID, uint8_t *seq, char buffer[])
{
    size_t start = 0;
    uint8_t len;
    if (m_uart_filestream == -1) return 2;

    if (_buf_size < 6) // starting ">>>" + topic + seq + len
        return 0;

    // look for starting ">>>"
    while (start <= (_buf_size - 6) && memcmp(_buf + start, ">>>", 3) != 0) {
        start++;
    }
    if (start >= (_buf_size - 6)) {
        return 0;
    }

    len = _buf[start + 5];
    if (start + len > _buf_size)
        return 0; // we don't have a complete msg yet

    // cerr << "rtps parse. message len " << (int)len << " on: " << start << endl;

    // Found a whole message, send it and remove from local _buf
    *topic_ID = _buf[start + 3];
    *seq = _buf[start + 4];
    memmove(buffer, _buf + start + 6, len);

    memmove(_buf + start, _buf + start + 6 + len, sizeof(_buf) - start - 6 - len);
    _buf_size -= len + 6;

    return len;
}

int UART_node::writeRTPStoUART(const char topic_ID, char buffer[], uint32_t length)
{
    if (m_uart_filestream == -1) return -EINVAL;

    static uint8_t seq = 0;
    static char pre[] = ">>>TSL"; // ugly hack in place
    uint8_t len = length;
    int r;

    pre[3] = topic_ID;
    pre[4] = seq;
    pre[5] = len;
    r = write(m_uart_filestream, pre, 6);
    if (r < 0)
        return r;
    r = write(m_uart_filestream, buffer, length);
    if (r < 0)
        return r;

    seq++;

    return length;
}

int UART_node::writeMavlinkToUART(char buffer[], uint32_t length)
{
    if (m_uart_filestream == -1) return -EINVAL;

    return write(m_uart_filestream, buffer, length);
}
