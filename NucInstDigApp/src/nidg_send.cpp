#include <string>
#include <iostream>
#include <exception>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>

#include <zmq.hpp>

int main(int argc, char* argv[])
{
    try {
    zmq::context_t ctx{1};
    zmq::socket_t socket(ctx, zmq::socket_type::req);
    std::string type = argv[1];
    std::string addr = std::string("tcp://") + argv[2] + ":5557";
    try {
        socket.connect(addr.c_str());
    }
    catch(const std::exception& ex)
    {
        std::cerr << "Unable to connect to " << addr << " - " << ex.what() << std::endl;
        return 1;    
    }
    std::cerr << "Connected to " << addr << std::endl;
    rapidjson::Document doc_send;
    rapidjson::Value arg3((argc > 3 ? argv[3] : ""), doc_send.GetAllocator());
    rapidjson::Value arg4;
    rapidjson::Value arg5((argc > 5 ? argv[5] : ""), doc_send.GetAllocator());
    doc_send.SetObject();
    if (type == "execute_cmd")
    {
        doc_send.AddMember("command", "execute_cmd", doc_send.GetAllocator());
        doc_send.AddMember("name", arg3, doc_send.GetAllocator());
        arg4.SetString((argc > 4 ? argv[4] : ""), doc_send.GetAllocator());
        doc_send.AddMember("args", arg4, doc_send.GetAllocator());
    }
    else if (type == "get_parameter")
    {
        doc_send.AddMember("command", "get_parameter", doc_send.GetAllocator());
        doc_send.AddMember("name", arg3, doc_send.GetAllocator());
        arg4.SetInt(atol(argc > 4 ? argv[4] : "0"));
        doc_send.AddMember("idx", arg4, doc_send.GetAllocator());
    }
    else if (type == "set_parameter")
    {
        doc_send.AddMember("command", "set_parameter", doc_send.GetAllocator());
        doc_send.AddMember("name", arg3, doc_send.GetAllocator());
        arg4.SetInt(atol(argc > 4 ? argv[4] : "0"));
        doc_send.AddMember("idx", arg4, doc_send.GetAllocator());
        doc_send.AddMember("value", arg5, doc_send.GetAllocator());
    }
    else if (type == "read_data")
    {
        doc_send.AddMember("command", "execute_read_command", doc_send.GetAllocator());
        doc_send.AddMember("name", arg3, doc_send.GetAllocator());
        arg4.SetString((argc > 4 ? argv[4] : ""), doc_send.GetAllocator());
        doc_send.AddMember("args", arg4, doc_send.GetAllocator());
    }
    else
    {
        std::cerr << "unknown command" << std::endl;
    }

    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    doc_send.Accept(writer);
    std::string sendstr = sb.GetString();
    std::cout << "Sending " << sendstr << std::endl;
    socket.send(zmq::buffer(sendstr), zmq::send_flags::none);
    zmq::message_t reply{};
    socket.recv(reply, zmq::recv_flags::none);
    std::cout << "Received " << reply.to_string() << std::endl;
    rapidjson::Document doc_recv;
    doc_recv.Parse(reply.to_string().c_str());
    return 0;
    }
    catch(const std::exception& ex)
    {
        std::cerr << "Exception: " << ex.what() << std::endl;
        return 1;    
    }
}

