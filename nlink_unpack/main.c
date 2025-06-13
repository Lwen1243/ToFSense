#include "nlink_linktrack_nodeframe0.h"
#include "nlink_linktrack_nodeframe1.h"
#include "nlink_tofsense_frame0.h"
#include "nlink_tofsensem_frame0.h"
#include "nlink_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>


int main()
{
    int serial_fd;
    struct termios tty;
    
    serial_fd = open("/dev/ttyCH343USB0", O_RDWR | O_NOCTTY);
    
}