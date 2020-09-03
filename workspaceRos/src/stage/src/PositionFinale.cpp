#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <stdlib.h> 
#include <cstdlib>
#include <ctime>


using namespace std;

geometry_msgs::Pose poseRob1;


void chatterCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
	poseRob1.position.x = msg->position.x;
	poseRob1.position.y = msg->position.y;
	poseRob1.position.z = msg->position.z;

	poseRob1.orientation.x = msg->orientation.x;
	poseRob1.orientation.y = msg->orientation.y;
	poseRob1.orientation.z = msg->orientation.z;
	poseRob1.orientation.w = msg->orientation.w;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "PositionFinale");

	ros::NodeHandle n;

	geometry_msgs::Pose poseRob;

	ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("posBruite", 1000);

	ros::Subscriber pose_robot = n.subscribe("pose", 1000, chatterCallback);

	srand(time(NULL));

	ros::Rate loop_rate(21);

	while (ros::ok()){

		poseRob1.position.x = poseRob1.position.x + ((double)rand() / (double)RAND_MAX)*0.0002-0.0001;
		poseRob1.position.y = poseRob1.position.y + ((double)rand() / (double)RAND_MAX)*0.0002-0.0001;
		poseRob1.position.z = poseRob1.position.z + ((double)rand() / (double)RAND_MAX)*0.0002-0.0001;

		poseRob1.orientation.x = poseRob1.orientation.x +((double)rand() / (double)RAND_MAX)*0.00000000002-0.00000000001;
		poseRob1.orientation.y = poseRob1.orientation.y +((double)rand() / (double)RAND_MAX)*0.00000002-0.00000001;
		poseRob1.orientation.z = poseRob1.orientation.z +((double)rand() / (double)RAND_MAX)*0.0000002-0.0000001;
		poseRob1.orientation.w = poseRob1.orientation.w +((double)rand() / (double)RAND_MAX)*0.002-0.001;

		pose_pub.publish(poseRob1);
		ros::spinOnce();

	}

}

