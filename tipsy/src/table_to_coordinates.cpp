#include "ros/ros.h"
#include <iostream>
#include <std_msgs/String.h>
#include <string>
#include <tipsy/Coordinates.h>
#include <tipsy/DrinkOrder.h>

#define PUBLISH_ITERATIONS 5

#define TABLE_1_X -0.68
#define TABLE_1_Y -2.17
#define TABLE_2_X -0.68
#define TABLE_2_Y -1.03
#define TABLE_3_X -0.68
#define TABLE_3_Y 1.17
#define TABLE_4_X -0.68
#define TABLE_4_Y 2.13

int main(int argc, char **argv) {
    ros::init(argc, argv, "table_to_coordinates");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    ros::ServiceClient client =
        n.serviceClient<tipsy::DrinkOrder>("drink_to_table");
    ros::Publisher coordinates_pub =
            n.advertise<tipsy::Coordinates>("coordinates_topic", 1);

    tipsy::DrinkOrder srv;
    tipsy::Coordinates coordinate;
    tipsy::DrinkOrder::Request req;
    tipsy::DrinkOrder::Response res;

    int cont_published = 0;

    if(strcmp(argv[1], "serving") == 0){
        std::cout << "Enter desired drink: ";
        std::cin >> req.name_drink;
        client.waitForExistence();
        client.call(req, res);

        if (client.call(srv)) {
            ROS_INFO("Table: %s", res.table.c_str());
            std::string table_number = res.table.c_str();
            if (table_number == "1") {
                coordinate.x = TABLE_1_X;
                coordinate.y = TABLE_1_Y;
            }

            if (table_number == "2") {
                coordinate.x = TABLE_2_X;
                coordinate.y = TABLE_2_Y;
            }

            if (table_number == "3") {
                coordinate.x = TABLE_3_X;
                coordinate.y = TABLE_3_Y;
            }

            if (table_number == "4") {
                coordinate.x = TABLE_4_X;
                coordinate.y = TABLE_4_Y;
            }
            coordinate.timestamp = ros::Time::now();
            coordinate.serving = true;
        } else {
            ROS_ERROR("Failed to call service ");
            return 1;
        }

        while (cont_published < PUBLISH_ITERATIONS) {
            ROS_INFO("Publishing coordinates");
            coordinates_pub.publish(coordinate);
            cont_published++;
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    else if(strcmp(argv[1], "learning") == 0){

        // Table 1
        coordinate.x = TABLE_1_X;
        coordinate.y = TABLE_1_Y;
        coordinate.timestamp = ros::Time::now();
        coordinate.serving = false;

        while (cont_published < PUBLISH_ITERATIONS) {
            ROS_INFO("Publishing coordinates for table 1");
            coordinates_pub.publish(coordinate);
            cont_published++;
            ros::spinOnce();
            loop_rate.sleep();
        }
        cont_published = 0;

        // Table 2
        coordinate.y = TABLE_2_Y;
        coordinate.timestamp = ros::Time::now();

        while (cont_published < PUBLISH_ITERATIONS) {
            ROS_INFO("Publishing coordinates for table 2");
            coordinates_pub.publish(coordinate);
            cont_published++;
            ros::spinOnce();
            loop_rate.sleep();
        }
        cont_published = 0;

        // Table 3
        coordinate.y = TABLE_3_Y;
        coordinate.timestamp = ros::Time::now();

        while (cont_published < PUBLISH_ITERATIONS) {
            ROS_INFO("Publishing coordinates for table 3");
            coordinates_pub.publish(coordinate);
            cont_published++;
            ros::spinOnce();
            loop_rate.sleep();
        }
        cont_published = 0;

        // Table 4
        coordinate.y = TABLE_4_Y;
        coordinate.timestamp = ros::Time::now();

        while (cont_published < PUBLISH_ITERATIONS) {
            ROS_INFO("Publishing coordinates for table 4");
            coordinates_pub.publish(coordinate);
            cont_published++;
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    else
        ROS_ERROR("Please select a valid mode [serving/learning]\n");

    return 0;
}