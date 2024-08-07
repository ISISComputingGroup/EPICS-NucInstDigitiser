#include <string>
#include <iostream>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <flatbuffers/flatbuffers.h>
#include "dat2_digitizer_analog_trace_v2_generated.h"
#include "dev2_digitizer_event_v2_generated.h"

#include <zmq.hpp>

int main(int argc, char* argv[])
{
    zmq::context_t ctx{1};
    zmq::socket_t socket(ctx, zmq::socket_type::pull);
    std::string addr = std::string("tcp://") + argv[1] + ":5556";
    try {
        socket.connect(addr.c_str());
    }
    catch(const std::exception& ex)
    {
        std::cerr << "Unable to connect to " << addr << " - " << ex.what() << std::endl;
        return 1;    
    }
    std::cerr << "Connected to " << addr << std::endl;
    while(true) {
    zmq::message_t reply{};
    socket.recv(reply, zmq::recv_flags::none);
    auto msg = GetDigitizerAnalogTraceMessage(reply.data());
    auto channels = msg->channels();
    for(int i=0; i<channels->size(); ++i) {
        std::cerr << "chan " << (channels->Get(i)->channel()) << std::endl;
        std::cerr << "n voltages " << channels->Get(i)->voltage()->size() << std::endl;
        for(int j=0; j<1000; ++j)
        {
            std::cerr << channels->Get(i)->voltage()->Get(j) << " ";
        }
        std::cerr << std::endl;
    }
    std::cerr << "timestamp " << msg->metadata()->timestamp() << std::endl;
    }
    return 0;
}


//fb = DigitizerAnalogTraceMessage.GetRootAsDigitizerAnalogTraceMessage(message)
//        for i in range(0, fb.ChannelsLength()):
//            print(fb.Channels(i).VoltageAsNumpy())

