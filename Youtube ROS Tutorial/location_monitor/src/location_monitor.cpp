#include <vector>
#include <string>
#include <math.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "location_monitor/LandmarkDistance.h"

using std::vector;
using std::string;
using location_monitor::LandmarkDistance;

class Landmark{
  public:
    Landmark(string name, double x, double y)
			: name(name), x(x), y(y) {}
		string name;
		double x;
		double y;
};

class LandmarkMonitor{
	public:
		LandmarkMonitor(const ros::Publisher& landmark_pub): landmarks_(), landmark_pub_(landmark_pub) {
			InitLandmarks();
		}
	
		void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
			double x = msg->pose.pose.position.x;
  		double y = msg->pose.pose.position.y;
			LandmarkDistance closest_obj = FindClosest(x, y);
//  		ROS_INFO("Closest is : %s, distance is : %f", closest_obj.name.c_str(), closest_obj.distance);
			landmark_pub_.publish(closest_obj);
			if(closest_obj.distance <= 0.5){
				ROS_INFO("I am near the %s", closest_obj.name.c_str());
			}
		}

	private:
		vector<Landmark> landmarks_;
		ros::Publisher landmark_pub_;

		LandmarkDistance FindClosest(double robo_x, double robo_y){
			LandmarkDistance result;
			result.distance = -1;
			
			for (int i=0;i<landmarks_.size();++i){
				const Landmark& landmark = landmarks_[i];
				double xd = landmark.x - robo_x;
				double yd = landmark.y - robo_y;
				double distance = sqrt(xd*xd + yd*yd);

				if (result.distance < 0 || distance < result.distance){
					result.name = landmark.name;
					result.distance = distance;
				}
			}
			return result;
		}		

		void InitLandmarks(){
			landmarks_.push_back(Landmark("Cube",0.02,-0.28));
			landmarks_.push_back(Landmark("Dumpster",0.05,-1.77));
			landmarks_.push_back(Landmark("Cylinder",-1.98,-1.89));
			landmarks_.push_back(Landmark("Barrier",-2.59,-0.83));
			landmarks_.push_back(Landmark("Bookshelf",-0.66,1.68));
		}
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "location_monitor");
  ros::NodeHandle n;
	ros::Publisher landmark_pub = n.advertise<LandmarkDistance>("closest_landmark", 10);
  LandmarkMonitor monitor(landmark_pub);
  
  ros::Subscriber sub = n.subscribe("odom", 10, &LandmarkMonitor::OdomCallback, &monitor);
  ros::spin();

  return 0;
}
