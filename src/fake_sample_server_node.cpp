#include <ros/ros.h>

#include "apc_msgs/Sample.h"
#include "apc_vision/TakeSample.h"
#include "apc_vision/GetSamples.h"

#include "util.h"

using namespace apc_vision;
using namespace apc_msgs;

class FakeSampleServer {
private:

    bool take_sample_cb(TakeSample::Request& request, TakeSample::Response& response) {
        if(request.command == "reset") {
            samples[request.bin].clear();
            response.status = TakeSample::Response::SUCCESS;
            return true;
        } else if(request.command == "sample") {
            if(it == files.end()) {
                response.status = TakeSample::Response::NO_DATA;
                return true;
            }

            Sample sample;
            std::string filename = vision_dir + "/samples/" + *it;
            it++;
            if(!deserialize_msg(sample, filename)) {
                response.status = TakeSample::Response::NO_DATA;
                return true;
            }

            samples[request.bin].push_back(sample);
            response.status = TakeSample::Response::SUCCESS;
            return true;
        } else {
            response.status = TakeSample::Response::INVALID_COMMAND;
            return true;
        }
    }
public:
    FakeSampleServer(ros::NodeHandle nh, std::string vision_dir, std::vector<std::string> files) :
        vision_dir(vision_dir), files(files), it(this->files.begin()),
        take_sample_server(nh.advertiseService("take_sample", &FakeSampleServer::take_sample_cb, this)),
        get_samples_server(nh.advertiseService("get_samples", &FakeSampleServer::get_samples_cb, this))
    {}
private:
    std::string vision_dir;
    std::map<std::string, std::vector<apc_msgs::Sample> > samples;
    std::vector<std::string> files;
    std::vector<std::string>::iterator it;
    ros::ServiceServer take_sample_server;
    ros::ServiceServer get_samples_server;

    bool get_samples_cb(GetSamples::Request& request, GetSamples::Response& response) {
        response.status = GetSamples::Response::SUCCESS;
        response.samples = samples[request.bin];
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_sample_server_node");
    ros::NodeHandle nh;

    std::string vision_dir;
    nh.getParam("sample_server_node/vision_dir", vision_dir);

    std::vector<std::string> files;
    nh.getParam("sample_server_node/files", files);

    FakeSampleServer server(nh, vision_dir, files);

    ros::spin();

    return 0;
}
