#include "ros/ros.h"
#include <iostream>
#include <std_msgs/String.h>
#include <string>
#include <tipsy/Coordinates.h>
#include <tipsy/DrinkOrder.h>

#define PUBLISH_ITERATIONS 5

int main(int argc, char **argv) {
  ros::init(argc, argv, "table_to_coordinates");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);
  ros::ServiceClient client =
      n.serviceClient<tipsy::DrinkOrder>("drink_to_table");

  tipsy::DrinkOrder srv;
  tipsy::Coordinates coordinate;
  tipsy::DrinkOrder::Request req;
  tipsy::DrinkOrder::Response res;

  int cont_published = 0;

  std::cout << "Enter desired drink: ";
  std::cin >> req.name_drink;
  client.waitForExistence();
  client.call(req, res);

  if (client.call(srv)) {
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
    coordinate.timestamp = ros::Time::now();
  } else {
    ROS_ERROR("Failed to call service ");
    return 1;
  }

  ros::Publisher coordinates_pub =
      n.advertise<tipsy::Coordinates>("coordinates_topic", 1);

  while (cont_published < PUBLISH_ITERATIONS) {
    ROS_INFO("Publishing coordinates with timestamp %d", coordinate.timestamp);
    coordinates_pub.publish(coordinate);
    cont_published++;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}