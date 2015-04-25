#include <ros/ros.h>
#include <motoman_moveit/move_group_server.h>
#include <apc_vision/SampleVision.h>
#include <apc_vision/ProcessVision.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv) {

    if(argc < 3) {
        std::cout << "Needs one argument (which object?)" << std::endl;
        return -1;
    }

    ros::init(argc, argv, "move_there_move_back");
    ros::NodeHandle node_handle;  
    ros::AsyncSpinner spinner(4);
    spinner.start();
                    
    ros::service::waitForService("sample_vision");
    std::cout << "SAMPLE STARTED!" << std::endl;
    ros::service::waitForService("process_vision");
    std::cout << "PROCESS STARTED!" << std::endl;
    ros::service::waitForService("move_group_service");
    std::cout << "SERVICES STARTED!" << std::endl;
    for (int i = 0; i < 15; i++) { 
        sleep(1.0);
        std::cout << i << " seconds" << std::endl;
    }
    std::cout << "-------DONE SLEEPING----" << std::endl;

    geometry_msgs::Pose pose1;
    pose1.position.x = 0.5083;
    pose1.position.y = 0.15821;
    pose1.position.z = 1.61;
    pose1.orientation.x = 0.52684;
    pose1.orientation.y = -0.34443;
    pose1.orientation.z = -0.54901;
    pose1.orientation.w = 0.5499;
    
    geometry_msgs::Pose pose2;
    pose2.position.x = 0.52377;
    pose2.position.y = 0.32783;
    pose2.position.z = 1.7247;
    pose2.orientation.x = 0.36779;
    pose2.orientation.y = -0.34908;
    pose2.orientation.z = -0.6687;
    pose2.orientation.w =  0.54379;

    geometry_msgs::Pose pose3;
    pose3.position.x = 0.50602;
    pose3.position.y = 0.29665;
    pose3.position.z = 1.6084;
    pose3.orientation.x = -0.4916;
    pose3.orientation.y = 0.39284;
    pose3.orientation.z = 0.59876;
    pose3.orientation.w = -0.49547;

    geometry_msgs::Pose pose4;
    pose4.position.x = 0.51712;
    pose4.position.y = 0.4292;
    pose4.position.z = 1.6063;
    pose4.orientation.x = -0.40749;
    pose4.orientation.y = 0.475429;
    pose4.orientation.z = 0.680853;
    pose4.orientation.w = -0.378845;

    std::string bin = "A";

    // Clear the vision buffer
    apc_vision::SampleVision reset;
    reset.request.command = "reset";
    ros::service::call("sample_vision", reset);

    {
        motoman_moveit::move_group_server move;
        apc_vision::SampleVision sample;

        sample.request.command = bin;

        geometry_msgs::PoseStamped p;
        p.pose = pose3;

        move.request.pose = p;
        move.request.arm = "arm_left";
        move.request.tolerance = true;

        ros::service::call("move_group_service", move);
        if(!move.response.success) {
            std::cout << "move_group_service returned failure" << std::endl;
            return -1;
        }
        ros::service::call("sample_vision", sample);
        ros::service::call("sample_vision", sample);
    }

    {
        motoman_moveit::move_group_server move;
        apc_vision::SampleVision sample;

        sample.request.command = bin;

        geometry_msgs::PoseStamped p;
        p.pose = pose1;

        move.request.pose = p;
        move.request.arm = "arm_left";
        move.request.tolerance = true;

        ros::service::call("move_group_service", move);
        if(!move.response.success) {
            std::cout << "move_group_service returned failure" << std::endl;
            return -1;
        }
        ros::service::call("sample_vision", sample);
        ros::service::call("sample_vision", sample);
    }

    {
        motoman_moveit::move_group_server move;
        apc_vision::SampleVision sample;

        sample.request.command = bin;

        geometry_msgs::PoseStamped p;
        p.pose = pose2;

        move.request.pose = p;
        move.request.arm = "arm_left";
        move.request.tolerance = true;

        ros::service::call("move_group_service", move);
        if(!move.response.success) {
            std::cout << "move_group_service returned failure" << std::endl;
            return -1;
        }
        ros::service::call("sample_vision", sample);
        ros::service::call("sample_vision", sample);
    }

    {
        motoman_moveit::move_group_server move;
        apc_vision::SampleVision sample;

        sample.request.command = bin;

        geometry_msgs::PoseStamped p;
        p.pose = pose4;

        move.request.pose = p;
        move.request.arm = "arm_left";
        move.request.tolerance = true;

        ros::service::call("move_group_service", move);
        if(!move.response.success) {
            std::cout << "move_group_service returned failure" << std::endl;
            return -1;
        }
        ros::service::call("sample_vision", sample);
        ros::service::call("sample_vision", sample);
    }
	
    apc_vision::ProcessVision process;
    process.request.object1 = argv[1];
	process.request.object2 = argv[2];

    process.request.bin = bin;

    ros::service::call("process_vision", process);

    std::cout << "DONE!" << std::endl;
    return 0;
}
