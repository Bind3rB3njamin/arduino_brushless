#include <boost/asio.hpp>
#include <boost/regex.hpp>

class Serial
{
public:
    Serial(std::string port, unsigned int baud_rate)
    : io(), serial(io,port)
    {
		serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
	    serial.set_option(boost::asio::serial_port_base::character_size( 8 ));
	    serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none ));
	    serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one ));
    }
  
    void writeString(char *data, int length)
    {
        boost::asio::write(serial,boost::asio::buffer(data, length));
    }

    std::string readLine()
    {
		using namespace boost;
	
		asio::streambuf bf;
		size_t recBytes = asio::read_until(serial, bf, boost::regex("\n"));
		const char *data = asio::buffer_cast<const char*> (bf.data());

		return data;
    }

	void close()
	{
		serial.close();
	}

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
	std::string lineHelper;
};
