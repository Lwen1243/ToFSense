#include <boost/asio.hpp>
#include <iostream>
#include <iomanip>

using namespace boost::asio;

int main() {
    try {
        io_service io;
        serial_port serial(io, "/dev/ttyCH343USB0"); 
        
        // 设置串口参数
        serial.set_option(serial_port_base::baud_rate(921600));
        serial.set_option(serial_port_base::character_size(8));
        serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
        serial.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
        
        std::cout << "开始读取16进制数据 (按Ctrl+C停止)..." << std::endl;
        
        while (true) {
            unsigned char buffer[256];
            size_t len = serial.read_some(boost::asio::buffer(buffer));
            
            std::cout << "接收 " << len << " 字节: ";
            for (size_t i = 0; i < len; ++i) {
                std::cout << std::hex << " " <<  
                          static_cast<int>(buffer[i]) << " ";
            }
            std::cout << std::endl;
        }
    } catch (std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
