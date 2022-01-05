#include "ros/ros.h"
#include <iostream>
#include <tipsy/DrinkOrder.h>
#include <rosprolog/rosprolog_client/PrologClient.h>
#include <std_msgs/String.h>
#include <string>

using namespace std;

bool drinkOrder(tipsy::DrinkOrder::Request &req,
                  tipsy::DrinkOrder::Response &resp) {
  PrologClient pl = PrologClient("/rosprolog", true);

  PrologQuery bdgs = pl.query("rdf_has(drinkOntology:'" + req.name_drink +
                              "', drinkOntology:onTable, Table_number)");

    string table_number;

    for (PrologQuery::iterator it = bdgs.begin(); it != bdgs.end(); it++) {
        PrologBindings bdg = *it;
        cout << "Found solution: " << (bool)(it == bdgs.end()) << endl;
        cout << "Table_number = " << bdg["Table_number"] << endl;
        table_number = bdg["Table_number"].toString()[55];
  }

  resp.table = table_number;
  return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drink_to_table");
    ros::NodeHandle n;

    ros::ServiceServer service =
        n.advertiseService("drink_to_table", &drinkOrder);
    ros::spin();
         
  return 0;
}