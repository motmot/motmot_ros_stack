/*

Copyright (c) 2004-2010, California Institute of Technology.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <cam_iface.h>

#define _check_error() {                                                \
    int _check_error_err;                                               \
    _check_error_err = cam_iface_have_error();                          \
    if (_check_error_err != 0) {                                        \
                                                                        \
      fprintf(stderr,"%s:%d %s\n", __FILE__,__LINE__,cam_iface_get_error_string()); \
      exit(1);                                                          \
    }                                                                   \
  }                                                                     \


// forward declaration
class CameraNode {
public:
  CameraNode(int argc, char** argv);
  int run();
private:
  CamContext *cc;
  int step;
  std::string encoding;
  int width, height;
  sensor_msgs::CameraInfo cam_info;
  image_transport::CameraPublisher publisher;
  camera_info_manager::CameraInfoManager *cam_info_manager;
};

CameraNode::CameraNode(int argc, char** argv) {
  int num_buffers;

  ros::init(argc, argv, "camiface_ros_capture");

  if (ros::this_node::getNamespace() == "/") {
    fprintf(stderr,"[camiface_ros_capture] Started in the global namespace! This is probably wrong. Start camiface_ros_capture "
             "in the camera namespace.\nExample command-line usage:\n"
             "\t$ ROS_NAMESPACE=my_camera rosrun camiface_ros camiface_ros_capture");
  }

  cam_iface_startup_with_version_check();
  _check_error();

  printf("using driver %s\n",cam_iface_get_driver_name());
  int ncams = cam_iface_get_num_cameras();
  _check_error();

  if (ncams<1) {

    printf("no cameras found, will now exit\n");

    cam_iface_shutdown();
    _check_error();

    exit(1);
  }
  _check_error();

  int device_number = -1;

  printf("%d camera(s) found.\n",ncams);
  for (int i=0; i<ncams; i++) {
    Camwire_id cam_info_struct;
    cam_iface_get_camera_info(i, &cam_info_struct);
    if (cam_iface_have_error()==CAM_IFACE_CAMERA_NOT_AVAILABLE_ERROR) {
      printf("  camera %d: (not available)\n",i);
      cam_iface_clear_error();
      continue;
    }
    device_number = i;
    _check_error();
    printf("  camera %d:\n",i);
    printf("    vendor: %s\n",cam_info_struct.vendor);
    printf("    model: %s\n",cam_info_struct.model);
    printf("    chip: %s\n",cam_info_struct.chip);
  }

  if (device_number == -1) {
    fprintf(stderr,"No cameras available.\n");
    exit(1);
  }

  printf("choosing camera %d\n",device_number);

  int num_modes;
  cam_iface_get_num_modes(device_number, &num_modes);
  _check_error();

  printf("%d mode(s) available:\n",num_modes);

  int mode_number = 0;

  for (int i=0; i<num_modes; i++) {
    char mode_string[255];
    cam_iface_get_mode_string(device_number,i,mode_string,255);
    if (strstr(mode_string,"FORMAT7_0")!=NULL) {
      if (strstr(mode_string,"MONO8")!=NULL) {
        // pick this mode
        mode_number = i;
      }
    }
    printf("  %d: %s\n",i,mode_string);
  }

  printf("Choosing mode %d\n",mode_number);

  num_buffers = 5;
  cam_iface_constructor_func_t new_CamContext = cam_iface_get_constructor_func(device_number);
  cc = new_CamContext(device_number,num_buffers,mode_number);
  _check_error();

  int left, top;
  CamContext_get_frame_roi(cc, &left, &top, &width, &height);
  _check_error();

  CamContext_get_num_framebuffers(cc,&num_buffers);
  printf("allocated %d buffers\n",num_buffers);

  int num_props;
  CamContext_get_num_camera_properties(cc,&num_props);
  _check_error();

  printf("%d camera properties: \n",num_props);

  for (int i=0; i<num_props; i++) {
    CameraPropertyInfo cam_props;
    CamContext_get_camera_property_info(cc,i,&cam_props);
    _check_error();

    if (strcmp(cam_props.name,"white balance")==0) {
      fprintf(stderr,"WARNING: ignoring white balance property\n");
      continue;
    }

    printf("  %s: ",cam_props.name);
    fflush(stdout);

    if (cam_props.is_present) {
      if (cam_props.available) {
	if (cam_props.absolute_capable) {
	  if (cam_props.absolute_control_mode) {
	    printf("(absolute capable, on) " );
	  } else {
	    printf("(absolute capable, off) " );
	  }
	  fflush(stdout);
	}
	if (cam_props.readout_capable) {
	  if (cam_props.has_manual_mode) {
            long prop_value;
            int prop_auto;
	    CamContext_get_camera_property(cc,i,&prop_value,&prop_auto);
	    _check_error();
	    printf("%ld\n",prop_value);
	  } else {
	    /* Firefly2 temperature won't be read out. */
	    printf("no manual mode, won't read out. Original value: %ld\n",cam_props.original_value);
	  }
	} else {
	  printf("not readout capable");
	}
      } else {
	printf("present, but not available\n");
      }
    } else {
      printf("not present\n");
    }
  }

  int buffer_size;
  CamContext_get_buffer_size(cc,&buffer_size);
  _check_error();

  if (buffer_size == 0) {
    fprintf(stderr,"buffer size was 0 in %s, line %d\n",__FILE__,__LINE__);
    exit(1);
  }

  ros::NodeHandle n;
  cam_info_manager = new camera_info_manager::CameraInfoManager(n);

  if (!cam_info_manager->setCameraName(ros::this_node::getNamespace())) {
    fprintf(stderr,"namespace %s not valid for camera_info_manager\n",ros::this_node::getNamespace().c_str());
  }

  // topic is "image_raw", with queue size of 1

  // image transport interfaces
  image_transport::ImageTransport *transport = new image_transport::ImageTransport(n);
  publisher = transport->advertiseCamera("image_raw", 1);

  CamContext_start_camera(cc);
  _check_error();

  int num_trigger_modes;
  CamContext_get_num_trigger_modes( cc, &num_trigger_modes );
  _check_error();

  printf("trigger modes:\n");
  for (int i =0; i<num_trigger_modes; i++) {
    char mode_string[255];
    CamContext_get_trigger_mode_string( cc, i, mode_string, 255 );
    printf("  %d: %s\n",i,mode_string);
  }
  printf("\n");

  switch (cc->coding) {
  case CAM_IFACE_MONO8_BAYER_BGGR:
    step = width;
    encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
    break;
  case CAM_IFACE_MONO8_BAYER_RGGB:
    step = width;
    encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
    break;
  case CAM_IFACE_MONO8_BAYER_GRBG:
    step = width;
    encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
    break;
  case CAM_IFACE_MONO8_BAYER_GBRG:
    step = width;
    encoding = sensor_msgs::image_encodings::BAYER_GBRG8;
    break;
  case CAM_IFACE_MONO8:
    step = width;
    encoding = sensor_msgs::image_encodings::MONO8;
    break;
  case CAM_IFACE_RGB8:
    step = width*3;
    encoding = sensor_msgs::image_encodings::RGB8;
    break;
  default:
    printf("do not know encoding for this format\n");
    exit(1);
  }
}

int CameraNode::run() {
  printf("will now run forever. press Ctrl-C to interrupt\n");

  while (ros::ok())
  {

    std::vector<uint8_t> data(step*height);
    //CamContext_grab_next_frame_blocking(cc,&data[0],0.2); // timeout after 200 msec
    CamContext_grab_next_frame_blocking(cc,&data[0],-1.0f); // never timeout
    int errnum = cam_iface_have_error();
    if (errnum == CAM_IFACE_FRAME_TIMEOUT) {
      cam_iface_clear_error();
      continue; // wait again
    }
    if (errnum == CAM_IFACE_FRAME_DATA_MISSING_ERROR) {
      cam_iface_clear_error();
    } else if (errnum == CAM_IFACE_FRAME_INTERRUPTED_SYSCALL) {
      cam_iface_clear_error();
    } else if (errnum == CAM_IFACE_FRAME_DATA_CORRUPT_ERROR) {
      cam_iface_clear_error();
    } else {
      _check_error();

      double timestamp;
      CamContext_get_last_timestamp(cc,&timestamp);
      _check_error();

      unsigned long framenumber;
      CamContext_get_last_framenumber(cc,&framenumber);
      _check_error();


      sensor_msgs::Image msg;

      msg.header.seq = framenumber;
      msg.header.stamp = ros::Time(timestamp);
      msg.header.frame_id = "0";

      msg.height = height;
      msg.width = width;
      msg.encoding = encoding;
      msg.step = step;
      msg.data = data;

      // get current CameraInfo data
      cam_info = cam_info_manager->getCameraInfo();
      cam_info.header.stamp = msg.header.stamp;
      cam_info.header.seq = msg.header.seq;
      cam_info.header.frame_id = msg.header.frame_id;

      publisher.publish(msg, cam_info);
    }
    ros::spinOnce();
  }
  return 0;
}

int main(int argc, char** argv)
{
  CameraNode* cn = new CameraNode(argc,argv);
  return cn->run();
}
