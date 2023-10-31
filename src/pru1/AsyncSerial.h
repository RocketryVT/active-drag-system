//
// Created by Gregory Wainer on 10/30/2023.
// Sourced from: https://gist.github.com/kaliatech/427d57cb1a8e9a8815894413be337cf9
// Archive: https://web.archive.org/web/20130825102715/http://www.webalice.it/fede.tft/serial_port/serial_port.html
// StackOverFlow: https://stackoverflow.com/questions/2807055/reading-from-serial-port-with-boost-asio
//

#include <boost/asio.hpp>

class AsyncSerial: private boost::asio::noncopyable
{
public:
    AsyncSerial();

    AsyncSerial(const std::string& devname, unsigned int baud_rate,
                boost::asio::serial_port_base::parity opt_parity=
                boost::asio::serial_port_base::parity(
                        boost::asio::serial_port_base::parity::none),
                boost::asio::serial_port_base::character_size opt_csize=
                boost::asio::serial_port_base::character_size(8),
                boost::asio::serial_port_base::flow_control opt_flow=
                boost::asio::serial_port_base::flow_control(
                        boost::asio::serial_port_base::flow_control::none),
                boost::asio::serial_port_base::stop_bits opt_stop=
                boost::asio::serial_port_base::stop_bits(
                        boost::asio::serial_port_base::stop_bits::one));

    void open(const std::string& devname, unsigned int baud_rate,
              boost::asio::serial_port_base::parity opt_parity=
              boost::asio::serial_port_base::parity(
                      boost::asio::serial_port_base::parity::none),
              boost::asio::serial_port_base::character_size opt_csize=
              boost::asio::serial_port_base::character_size(8),
              boost::asio::serial_port_base::flow_control opt_flow=
              boost::asio::serial_port_base::flow_control(
                      boost::asio::serial_port_base::flow_control::none),
              boost::asio::serial_port_base::stop_bits opt_stop=
              boost::asio::serial_port_base::stop_bits(
                      boost::asio::serial_port_base::stop_bits::one));

    bool isOpen() const;

    bool errorStatus() const;

    void close();

    void write(const char *data, size_t size);

    void write(const std::vector<char>& data);

    void writeString(const std::string& s);

    virtual ~AsyncSerial()=0;
};

class BufferedAsyncSerial: public AsyncSerial
{
public:
    BufferedAsyncSerial();

    BufferedAsyncSerial(const std::string& devname, unsigned int baud_rate,
                        boost::asio::serial_port_base::parity opt_parity=
                        boost::asio::serial_port_base::parity(
                                boost::asio::serial_port_base::parity::none),
                        boost::asio::serial_port_base::character_size opt_csize=
                        boost::asio::serial_port_base::character_size(8),
                        boost::asio::serial_port_base::flow_control opt_flow=
                        boost::asio::serial_port_base::flow_control(
                                boost::asio::serial_port_base::flow_control::none),
                        boost::asio::serial_port_base::stop_bits opt_stop=
                        boost::asio::serial_port_base::stop_bits(
                                boost::asio::serial_port_base::stop_bits::one));

    size_t read(char *data, size_t size);

    std::vector<char> read();

    std::string readString();

    std::string readStringUntil(const std::string delim="\n");

    virtual ~BufferedAsyncSerial();
};

class CallbackAsyncSerial: public AsyncSerial
{
public:
    CallbackAsyncSerial();

    CallbackAsyncSerial(const std::string& devname, unsigned int baud_rate,
                        boost::asio::serial_port_base::parity opt_parity=
                        boost::asio::serial_port_base::parity(
                                boost::asio::serial_port_base::parity::none),
                        boost::asio::serial_port_base::character_size opt_csize=
                        boost::asio::serial_port_base::character_size(8),
                        boost::asio::serial_port_base::flow_control opt_flow=
                        boost::asio::serial_port_base::flow_control(
                                boost::asio::serial_port_base::flow_control::none),
                        boost::asio::serial_port_base::stop_bits opt_stop=
                        boost::asio::serial_port_base::stop_bits(
                                boost::asio::serial_port_base::stop_bits::one));

    void setCallback(const
                     boost::function<void (const char*, size_t)>& callback);

    void clearCallback();

    virtual ~CallbackAsyncSerial();
};