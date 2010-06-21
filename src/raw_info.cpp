#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <stdlib.h>
#include <stdio.h>

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
  printf("%d x %d (step %d), %s\n",
         msg->width, msg->height, msg->step, msg->encoding.c_str());
  exit(0);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "raw_info", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  if (nh.resolveName("image") == "/image") {
    ROS_WARN("raw_info: image has not been remapped! Typical command-line usage:\n"
             "\t$ ./raw_info image:=<image topic> [transport]");
  }

  std::string transport = (argc > 1) ? argv[1] : "raw";
  image_transport::Subscriber sub_;
  image_transport::ImageTransport it(nh);

  std::string topic = nh.resolveName("image");
  sub_ = it.subscribe(topic, 1, &image_cb, transport);

  ros::spin();

  return 0;
}
