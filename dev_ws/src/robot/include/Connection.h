#include <vector>
class Connection
{
    public:
        Connection(){}
        virtual ~Connection(){}

        virtual int open() = 0;
        virtual int close() = 0;

        virtual int send(std::vector<unsigned char> &data) = 0;
        //send data to the robot. use explicit pointer conversion

        virtual int receive(std::vector<unsigned char> &data) = 0;
        //receive state of the robot. record to data. use explicit pointer conversion
};
