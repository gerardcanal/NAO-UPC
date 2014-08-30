// ROS
#include "ros/ros.h"
// CURRENT PROJECT
#include "shopping_list/checkObjects.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "shopping_list_checkObjects_client");

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<shopping_list::checkObjects>("/shopping_list/checkObjects");

  shopping_list::checkObjects srv;
  if (client.call(srv))
  {
	int num_objects = srv.response.num_objects;

    ROS_INFO("Found: %d objects", num_objects);

    for (int i = 0; i < num_objects; ++i)
        ROS_INFO("Object with ID %d",  srv.response.shopping_list[i]);

  }
  else
  {
    ROS_ERROR("Failed to call service checkObjects");
    return 1;
  }

  return 0;
}
