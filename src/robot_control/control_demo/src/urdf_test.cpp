#include <boost/date_time.hpp>
#include <ros/ros.h>
#include <limits>
#include <urdf/model.h>


int main(int argc, char **argv) {
	ros::init(argc, argv, "urdf_test_demo");
    ros::NodeHandle node_handle("~");

    urdf::Model robot_model;
    std::string xml_string;

    node_handle.searchParam("robot_description",xml_string);
    robot_model.initString(xml_string);

	std::map<std::string, boost::shared_ptr<urdf::Joint>>::iterator it;

    for(it = robot_model.joints_.begin(); it!= robot_model.joints_.end(); ++it)
        std::cout<<it->first<<std::endl;
}
