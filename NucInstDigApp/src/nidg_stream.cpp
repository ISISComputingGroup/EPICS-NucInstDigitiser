#include <string>
#include <iostream>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <flatbuffers/flatbuffers.h>
#include "dat1_digitizer_analog_trace_v1_generated.h"
#include "dev1_digitizer_event_v1_generated.h"

#include <zmq.hpp>

int main(int argc, char* argv[])
{
    zmq::context_t ctx{1};
    zmq::socket_t socket(ctx, zmq::socket_type::pull);
    socket.connect(argv[1]);
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
    std::cerr << "timestamp " << msg->status()->timestamp() << std::endl;
    }
    return 0;
}


//fb = DigitizerAnalogTraceMessage.GetRootAsDigitizerAnalogTraceMessage(message)
//        for i in range(0, fb.ChannelsLength()):
//            print(fb.Channels(i).VoltageAsNumpy())

