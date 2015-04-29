#include <iostream>
#include <string>
#include <sys/types.h>
#include <dirent.h>
#include <string>
#include <errno.h>
#include <algorithm>
#include <unordered_map>

#include "ros/ros.h"
#include "object_database/DatabaseRetrievalService.h"
#include "ros/package.h"

#include <vector>

using namespace std;

/*
 * Hash map for retrieving the value as an instance of an object given its key name as string
 */
unordered_map<string, object_database::Object> DatabaseMap;

/*
 * \fn bool initDatabase()
 * \brief Initializes the database with its properties that reside in a folder/directory
 * \param strDirectory a string defining the path to the directory where each object's properties are stored
 * \return success as a boolean
 */
bool initDatabase(string strDirectory)
{
	 DIR *dp;
	 struct dirent *dirp;

	 int id = 1;

	 // Open the directory
	 if((dp = opendir(strDirectory.c_str())) != NULL)
	 {
		 // Read the sub-directories
		 while((dirp = readdir(dp)) != NULL)
		 {
			 string strTempFilename = string(dirp->d_name);
			 if(strcmp(strTempFilename.c_str(), ".") != 0 && strcmp(strTempFilename.c_str(), "..") != 0)
			 {
				 // Extract name of the sub-directories (objects) as lower case for standardization purposes
				 string objectName(dirp->d_name);
				 std::transform(objectName.begin(), objectName.end(), objectName.begin(), ::tolower);

				 //Initialize the database with the name and instance of the object
                 object_database::Object object;
                 object.object_name = objectName;
                 object.id = id++;
                 DatabaseMap[objectName] = object;
//				 cout<<objectName<<"\n";
			 }
		 }
		 return true;
	 }
	 else
	 {
		 ROS_INFO("cannot open directory. %s \n", strDirectory.c_str());
		 return false;
	 }
}

/*
 * \fn bool findObject(object_database::DatabaseRetrievalService::Request &req,
 *			     		object_database::DatabaseRetrievalService::Response &res)
 * \brief Provides the service to find the requested object within the database
 * \param req DatabaseRetrievalService defined in the service as a request
 * \param res DatabaseRetrievalService defined in the service as a response
 * \return success as a boolean
 * TODO: Add complex properties of the objects to be returned as a response
 */
bool findObjectPosition (object_database::DatabaseRetrievalService::Request &req,
		object_database::DatabaseRetrievalService::Response &res)
{
    for (auto &item:req.listObject.listObjects)
    {
        unordered_map<string, object_database::Object>::const_iterator got = DatabaseMap.find (item.object_name);
        cout<<"\nObject To Be Found: "<<item.object_name <<endl;
        // If object not found
        if ( got == DatabaseMap.end() )
        {
            ROS_INFO("Not found");
            return false;
        }
        // Object found
        else
        {
            object_database::Object object = (got->second);
            // return response as ID of the object
            res.listObjectWithPosition.listObjects.push_back(object);
        }
    }
    return true;
}

int main(int argc, char **argv)
{
	// Initialize the ROS node
	ros::init(argc, argv, "object_database_server");
	ros::NodeHandle n;

	std::string strDirectory = ros::package::getPath("object_database") + "/src/" + "objects";
	//	 cout<<strDirectory<<"\n";

	// Initialize the database
	initDatabase(strDirectory);

	// Advertize the service
    ros::ServiceServer service = n.advertiseService("object_database_server", findObjectPosition);
	ROS_INFO("Ready to find objects.");

	ros::spin();

	return 0;
}
