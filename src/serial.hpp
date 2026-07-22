#include <libserial/SerialPort.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <unistd.h>

using namespace LibSerial ;

class serial{
private:
    //constexpr const char* const SERIAL_PORT_1 = "/dev/ttyACM0" ;
    char data_byte; // Char variable to store data coming from the serial port.
    //char string[20]={0};
    SerialPort serial_port;
public:
    serial(char port[]="/dev/ttyACM0"){
        try
        {
            usleep(1000) ;
            this->serial_port.Open(port);
        }
        catch (const OpenFailed&)
        {
            std::cerr << "The serial port did not open correctly." << std::endl ;
        }
        // Set the baud rate of the serial port.
        this->serial_port.SetBaudRate(BaudRate::BAUD_115200) ;
        // Set the number of data bits.
        this->serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8) ;
        // Turn off hardware flow control.
        this->serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
        // Disable parity.
        this->serial_port.SetParity(Parity::PARITY_NONE) ;
        // Set the number of stop bits.
        this->serial_port.SetStopBits(StopBits::STOP_BITS_1) ;
    }
    char* read_string(){
        char string[20]={0};
        int i=0;

        while(!serial_port.IsDataAvailable()){
            usleep(10) ;
        }
        size_t ms_timeout = 250 ;
        // Read one byte from the serial port and print it to the terminal.
        try{
            // Read a single byte of data from the serial port.
            do{
                serial_port.ReadByte(data_byte, ms_timeout) ;
                string[i]=data_byte;
                i++;

            }while(i<20 && data_byte!='\0');
                i=0;
                return string;

            // Show the user what is being read from the serial port.
            //std::cout << data_byte << std::flush ;
        }

        catch (const ReadTimeout&)
        {
            //std::cerr << "\nThe ReadByte() call has timed out." << std::endl ;
        }

        // Wait a brief period for more data to arrive.
        usleep(1000) ;

        // DataBuffer read_buffer ;

        // try
        // {
        //     // Read as many bytes as are available during the timeout period.
        //     serial_port.Read(read_buffer, 0, ms_timeout) ;
        // }
        // catch (const ReadTimeout&)
        // {
        //     for (size_t i = 0 ; i < read_buffer.size() ; i++)
        //     {
        //         std::cout << read_buffer.at(i) << std::flush;
        //     }

        //     //std::cerr << "The Read() call timed out waiting for additional data." << std::endl ;
        // }
        return string;
    }
    
};
