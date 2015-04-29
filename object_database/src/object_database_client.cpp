#include "ros/ros.h"
#include <cstdlib>
#include <fstream>
#include <algorithm>
#include "object_database/DatabaseRetrievalService.h"
#include <unordered_map>
#include <vector>

#include "jsoncpp/json.h"
#include "ros/package.h"

using namespace std;

/*
 * Hash map for storing the BinContents from the JSON file
 * Key: Bin Name
 * Value: List of Objects
 */
unordered_map<string, vector<object_database::Object>> BinContents;

/*
 * Ordered map for storing the WorkOrder from the JSON file
 * Key: Bin Name
 * Value: Object
 */
map<string, object_database::Object> WorkOrder;

/*
 * \fn void readJSONFile()
 * \brief Initializes the hashmaps with JSON file names
 * \param JsonFileName a string defining the name of the JSON file
 * \return void
 */
void readJSONFile(string *JsonFileName)
{
	Json::Value root;   // will contain the root value after parsing.
	Json::Reader reader;
	std::string ROSPackagePath = ros::package::getPath("object_database")+"/src/" + *JsonFileName;
	std::ifstream test(ROSPackagePath, std::ifstream::binary);
	bool parsingSuccessful = reader.parse( test, root, false );
	if ( !parsingSuccessful )
	{
		// report to the user the failure and their locations in the document.
		std::cout  << reader.getFormatedErrorMessages()
			   << "\n";
	}

	const Json::Value binContents = root["bin_contents"];
	const Json::Value workOrder = root["work_order"];
//	cout<<binContents.size()<<endl;
	Json::Value::Members binNames = binContents.getMemberNames();
	// Parse the BinContents
	for(int i = 0; i < binNames.size(); i++)
	{
		string binName = binNames[i];

		Json::Value itemArray = binContents[binName];
        vector<object_database::Object> items;

		for(int j = 0; j < itemArray.size(); j++)
		{
			string objectName(itemArray[j].asString());
			// Convert name of the objects to lower case for standardization purposes
			std::transform(objectName.begin(), objectName.end(), objectName.begin(), ::tolower);

            object_database::Object object;
            object.object_name = objectName;
            items.push_back(object);
		}
		BinContents[binName] = items;
	}
	// Parse the WorkOrder
	for(int i = 0; i < workOrder.size(); i++)
	{
		Json::Value item = workOrder[i];
		string binName(item.get("bin", "UTF-8").asString());

		string itemName(item.get("item", "UTF-8").asString());
		std::transform(itemName.begin(), itemName.end(), itemName.begin(), ::tolower);

        object_database::Object object;
        object.object_name = itemName;
        WorkOrder[binName] = object;
	}
    cout<<"\nWork Order: \n";
    for (auto& item: WorkOrder)
    {
        cout<<item.first<<"\n";
    }
    cout<<"\nBin Contents: \n";
    for (auto& item: BinContents)
    {
        cout<<item.first<<"\n";
    }

}

/*
 * \fn void getObjects()
 * \brief Requests the Database server for the objects
 * \return void
 */
void getObjects()
{
    // Create a node handle
    ros::NodeHandle n;

    // Create a service client
    ros::ServiceClient client = n.serviceClient<object_database::DatabaseRetrievalService>("object_database_server");

    object_database::DatabaseRetrievalService srv;

    string obj_name = "";

    int i = 0;
    // Retrieve the items in the WorkOrder from the database server
    for (auto& item: WorkOrder)
    {
        object_database::Object object = item.second;

        srv.request.listObject.listObjects.push_back(object);
        cout<<object<<" "<<item.first<<endl;
        i++;
    }

    if(client.call(srv))
    {
        ROS_INFO("Request for objects sent. Here is the answer:");
        for(auto&item:srv.response.listObjectWithPosition.listObjects)
        {
            int ID = item.id;
            ROS_INFO("Found object with id: '%d'", ID);
        }

    }
    else
    {
        ROS_ERROR("Failed to call service object_database_server");
        exit(1);
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_database_client");

	if (argc != 2)
	{
		ROS_INFO("usage: provide JSON file name");
		return 1;
	}

	// Read the JSON file
	string JsonFileName(argv[1]);
	readJSONFile(&JsonFileName);

	// Get the objects from the database
    getObjects();

	return 0;
}
