import serial
import serial.tools.list_ports

<<<<<<< HEAD
def read_serial(port_name, baudrate=9600, timeout=1):
=======
def read_serial(port_name, baudrate=921600, timeout=1):
>>>>>>> update
    try:
        with serial.Serial(port_name, baudrate, timeout=timeout) as ser:
            while True:
                try:
                    data = ser.read(ser.in_waiting or 1)
                    if data:
<<<<<<< HEAD
                        hex_str = ''.join([f"{byte:02X}" for byte in data])
                        if hex_str == '57':
                            break
=======
                        hex_str = ' '.join([f"{byte:02X}" for byte in data])

                        print(hex_str)
>>>>>>> update
                except KeyboardInterrupt:
                    print('stopping')
                    break
    except serial.SerialException as e:
        print(f"串口错误：{e}")
    except Exception as e:
        print(f"发生错误：{e}")

if __name__ == "__main__":
    selected_port = '/dev/ttyCH343USB0'
    baudrate = int(921600)
    read_serial(selected_port, baudrate)
