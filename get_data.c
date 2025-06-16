#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

int main() {
    int serial_fd;
    struct termios tty;
    
    serial_fd = open("/dev/ttyCH343USB0", O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        perror("无法打开串口设备");
        return 1;
    }
    
    // 获取当前串口配置
    if (tcgetattr(serial_fd, &tty) != 0) {
        perror("获取串口配置失败");
        close(serial_fd);
        return 1;
    }
    
    // 设置波特率921600
    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);
    
    // 8位数据位，无校验，1位停止位
    tty.c_cflag &= ~PARENB;  // 禁用奇偶校验
    tty.c_cflag &= ~CSTOPB;  // 1位停止位
    tty.c_cflag &= ~CSIZE;   // 清除数据位设置
    tty.c_cflag |= CS8;      // 8位数据位
    
    // 禁用硬件流控
    tty.c_cflag &= ~CRTSCTS;
    
    // 启用接收
    tty.c_cflag |= CREAD | CLOCAL;
    
    // 禁用软件流控
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    // 原始输入模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;  // 原始输出模式
    
    // 设置超时 - 等待至少1个字符，每字符最多等待100ms
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    
    // 应用设置
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        perror("设置串口参数失败");
        close(serial_fd);
        return 1;
    }
    
    printf("开始读取16进制数据 (按Ctrl+C停止)...\n");
    
    // 读取数据
    unsigned char buffer[512];
    while (1) {
        int n = read(serial_fd, buffer, sizeof(buffer));
        if (n <= 26)
        {
            puts("");
        }
            

        if (n > 0)
        {
            for (int i = 0 ; i < n; i ++) {
                printf("%02x ", buffer[i]);
            }
        }
        else {
            printf("error");
            break;
        }
    }
    
    close(serial_fd);
    return 0;
}
