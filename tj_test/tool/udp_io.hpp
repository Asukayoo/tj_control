#pragma once

#include <arpa/inet.h>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

// 非阻塞 UDP 接收
class UdpReceiver {
public:
    bool Open(int port) {
        Close();
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) {
            return false;
        }
        int yes = 1;
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(static_cast<uint16_t>(port));
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            Close();
            return false;
        }
        const int flags = ::fcntl(fd_, F_GETFL, 0);
        if (flags >= 0) {
            ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
        }
        return true;
    }

    ssize_t Recv(void* buf, size_t len) {
        if (fd_ < 0) {
            return -1;
        }
        return ::recvfrom(fd_, buf, len, 0, nullptr, nullptr);
    }

    void Close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    ~UdpReceiver() { Close(); }

private:
    int fd_ = -1;
};

// UDP 发送
class UdpSender {
public:
    bool Open() {
        Close();
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        return fd_ >= 0;
    }

    bool SetDestination(const char* host, int port) {
        if (fd_ < 0 || host == nullptr) {
            return false;
        }
        std::memset(&dest_, 0, sizeof(dest_));
        dest_.sin_family = AF_INET;
        dest_.sin_port = htons(static_cast<uint16_t>(port));
        if (::inet_pton(AF_INET, host, &dest_.sin_addr) != 1) {
            return false;
        }
        has_dest_ = true;
        return true;
    }

    ssize_t Send(const void* buf, size_t len) {
        if (fd_ < 0 || !has_dest_) {
            return -1;
        }
        return ::sendto(fd_, buf, len, 0, reinterpret_cast<sockaddr*>(&dest_),
                        sizeof(dest_));
    }

    void Close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        has_dest_ = false;
    }

    ~UdpSender() { Close(); }

private:
    int fd_ = -1;
    sockaddr_in dest_{};
    bool has_dest_ = false;
};
