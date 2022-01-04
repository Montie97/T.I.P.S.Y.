#include "ros/ros.h"
#include <iostream>
#include <knowrob_tutorial/Coordinates.h>
#include <knowrob_tutorial/DrinkOrder.h>
#include <std_msgs/String.h>
#include <string>

int main(int argc, char **argv) {
    ros::init(argc, argv, "table_to_coordinates");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<knowrob_tutorial::DrinkOrder>("drink_to_table");
    
    knowrob_tutorial::DrinkOrder srv;
    knowrob_tutorial::Coordinates coordinate;
    knowrob_tutorial::DrinkOrder::Request req;
    knowrob_tutorial::DrinkOrder::Response res;

    std::cout << "Enter desired drink: "; 
    std::cin >> req.name_drink ;
    client.waitForExistence();
    client.call(req,res);
    
    if (client.call(srv)){
        ROS_INFO("Table: %s", res.table.c_str());
        std::string table_number = res.table.c_str();
        if (table_number == "1") {
            coordinate.x = -0.68;
            coordinate.y = -2.17;
            coordinate.Y = 3.14;
        }

        if (table_number == "2") {
            coordinate.x = -0.68;
            coordinate.y = -1.03;
            coordinate.Y = 3.14;
        }

        if (table_number == "3") {
            coordinate.x = -0.68;
            coordinate.y = 1.17;
            coordinate.Y = 3.14;
        }

        if (table_number == "4") {
            coordinate.x = -0.68;
            coordinate.y = 2.13;
            coordinate.Y = 3.14;
        }
    }
    else{
       ROS_ERROR("Failed to call service ");
       return 1;
    }

  ros::Publisher coordinates_pub = n.advertise<knowrob_tutorial::Coordinates>("coordinates_topic", 10);
  ros::Rate loop_rate(1);

  while (ros::ok()) {
    coordinates_pub.publish(coordinate);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}