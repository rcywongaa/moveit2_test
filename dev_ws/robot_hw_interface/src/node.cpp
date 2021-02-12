#include "rclcpp/rclcpp.hpp"

#include "Connection.hpp"

class PrintingConnection : public Connection
{
  virtual int open() override
  {
    return 0;
  }

  virtual int close() override
  {
    return 0;
  }

  virtual int send(std::vector<unsigned char>& data) override
  {
    return 0;
  }

  virtual int receive(std::vector<unsigned char>& data) override
  {
    return 0;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::shutdown();
}
