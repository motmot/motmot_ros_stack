/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
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

std::string make_safe_name(std::string instr) {
  std::string outstr;
  const char* valid="abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890_";
  std::string mychar;
  for (std::string::const_iterator i=instr.begin();
       i < instr.end(); i++ ) {
    mychar = *i;
    if ( mychar.find_first_of(valid)!=std::string::npos) {
      outstr += mychar;
    } else {
      outstr += "_";
    }
  }
  return outstr;
}

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
    bool _host_timestamp;
    int _device_number;
};

CameraNode::CameraNode(int argc, char** argv) :
    _host_timestamp(false),
    _device_number(-1)
{
    int num_buffers;

    ros::init(argc, argv, "camnode", ros::init_options::AnonymousName);
    ros::NodeHandle _node;

    ros::param::get ("host_timestamp", _host_timestamp);
    if (_host_timestamp)
        ROS_INFO("Host timestamps ON");

    /*
    if the user supplies a device_guid or number (or puts it in the parameter server
    under the path of this node, use that. For example
        $ rosrun camiface_ros camnode _device_guid:=Prosilica-02-2020C-06732
    */

    int param_device_number = -1;
    ros::param::get (ros::this_node::getName() + "/device_number", param_device_number);

    std::string param_device_guid;
    ros::param::get (ros::this_node::getName() + "/device_guid", param_device_guid);

    int param_device_guid_int = -1;
    ros::param::get (ros::this_node::getName() + "/device_guid", param_device_guid_int);

    std::string param_device_trigger;
    ros::param::get (ros::this_node::getName() + "/device_trigger", param_device_trigger);

    if (param_device_guid_int != -1 && param_device_guid.empty()) {
        ROS_WARN("stupid ros weakly typed parameter server - converting guid to string");
        std::stringstream out;
        out << param_device_guid_int;
        param_device_guid = out.str();
    }

    cam_iface_startup_with_version_check();
    _check_error();

    ROS_DEBUG("using driver %s",cam_iface_get_driver_name());

    int ncams = cam_iface_get_num_cameras();
    _check_error();

    if (ncams<1) {
        ROS_WARN("no cameras found, will now exit");
        cam_iface_shutdown();
        _check_error();
        exit(1);
    }

    std::vector<std::string> safe_names;
    ROS_INFO("%d camera(s) found",ncams);
    for (int i=0; i<ncams; i++) {
        Camwire_id cam_info_struct;
        cam_iface_get_camera_info(i, &cam_info_struct);
        if (cam_iface_have_error()==CAM_IFACE_CAMERA_NOT_AVAILABLE_ERROR) {
            ROS_WARN("camera %d: (not available)",i);
            cam_iface_clear_error();
            continue;
        }

        _check_error();
        ROS_INFO("camera %d guid: %s",i,cam_info_struct.chip);
        std::string sn = make_safe_name(cam_info_struct.chip);
        safe_names.push_back( sn );
        ROS_DEBUG("camera safe name: %s",sn.c_str());

        if ((param_device_number != -1) && (param_device_number == i)) {
            _device_number = i;
            ROS_INFO("using user supplied device_number");
        } else if (param_device_guid.length() && (cam_info_struct.chip == param_device_guid)) {
            _device_number = i;
            ROS_INFO("using user supplied device_guid");
        }
    }

    if (safe_names.empty()) {
        ROS_WARN("No cameras available");
        exit(1);
    } else if (_device_number == -1 && ((param_device_number != -1) || param_device_guid.length())) {
        ROS_WARN("Selected camera not found");
        exit(1);
    } else {
        Camwire_id cam_info_struct;
        /* choose the first camera */
        _device_number = 0;
        cam_iface_get_camera_info(_device_number, &cam_info_struct);
        _check_error();
        ROS_INFO("choosing camera %d (%s %s guid:%s)",
                    _device_number,
                    cam_info_struct.vendor, cam_info_struct.model, cam_info_struct.chip);
    }

    int num_modes;
    cam_iface_get_num_modes(_device_number, &num_modes);
    _check_error();

    ROS_DEBUG("%d mode(s) available:\n",num_modes);

    int mode_number = 0;
    for (int i=0; i<num_modes; i++) {
        char mode_string[255];
        cam_iface_get_mode_string(_device_number,i,mode_string,255);
        if (strstr(mode_string,"FORMAT7_0")!=NULL) {
            if (strstr(mode_string,"MONO8")!=NULL) {
                // pick this mode
                mode_number = i;
            }
        }
        ROS_DEBUG("  %d: %s",i,mode_string);
    }

    ROS_INFO("choosing mode %d",mode_number);

    num_buffers = 5;
    cam_iface_constructor_func_t new_CamContext = cam_iface_get_constructor_func(_device_number);
    cc = new_CamContext(_device_number,num_buffers,mode_number);
    _check_error();

    int left, top;
    CamContext_get_frame_roi(cc, &left, &top, &width, &height);
    _check_error();

    CamContext_get_num_framebuffers(cc,&num_buffers);
    ROS_DEBUG("allocated %d buffers",num_buffers);

    int num_props;
    CamContext_get_num_camera_properties(cc,&num_props);
    _check_error();

    ROS_DEBUG("%d camera properties:",num_props);

    for (int i=0; i<num_props; i++) {
        CameraPropertyInfo cam_props;
        CamContext_get_camera_property_info(cc,i,&cam_props);
        _check_error();

        if (strcmp(cam_props.name,"white balance")==0) {
            ROS_WARN("WARNING: ignoring white balance property");
            continue;
        }

        ROS_DEBUG("  %s: ",cam_props.name);

        if (cam_props.is_present) {
            if (cam_props.available) {
                if (cam_props.absolute_capable) {
                    if (cam_props.absolute_control_mode) {
                        ROS_DEBUG("(absolute capable, on) " );
                    } else {
                        ROS_DEBUG("(absolute capable, off) " );
                    }

                }
                if (cam_props.readout_capable) {
                    if (cam_props.has_manual_mode) {
                        long prop_value;
                        int prop_auto;
                        CamContext_get_camera_property(cc,i,&prop_value,&prop_auto);
                        _check_error();
                        ROS_DEBUG("%ld",prop_value);
                    } else {
                        /* Firefly2 temperature won't be read out. */
                        ROS_DEBUG("no manual mode, won't read out. Original value: %ld",cam_props.original_value);
                    }
                } else {
                        ROS_DEBUG("not readout capable");
                }
            } else {
                ROS_DEBUG("present, but not available");
            }
        } else {
            ROS_DEBUG("not present");
        }
    }

    cam_info_manager = new camera_info_manager::CameraInfoManager(_node);
//    if (!cam_info_manager->setCameraName(safe_names.at(_device_number))) {
//        ROS_WARN("ROS name %s not valid for camera_info_manager\n",ros::this_node::getName().c_str());
//    }

    // topic is "image_raw", with queue size of 1
    // image transport interfaces
    image_transport::ImageTransport *transport = new image_transport::ImageTransport(_node);
    publisher = transport->advertiseCamera(ros::this_node::getName() + "/image_raw", 1);

    int num_trigger_modes;
    CamContext_get_num_trigger_modes( cc, &num_trigger_modes );
    _check_error();

    ROS_DEBUG("trigger modes:");

    char mode[255];
    int trigger_mode_number = -1;
    for (int i =0; i<num_trigger_modes; i++) {
        CamContext_get_trigger_mode_string( cc, i, mode, 255 );
        ROS_DEBUG("  %s (#%d)", mode, i);

        std::string mode_string(mode);
        if (param_device_trigger == mode_string)
            trigger_mode_number = i;
    }

    if (trigger_mode_number != -1) {
        CamContext_get_trigger_mode_string( cc, trigger_mode_number, mode, 255 );
        _check_error();
        ROS_INFO("choosing trigger mode %d (%s)", trigger_mode_number, mode);
        CamContext_set_trigger_mode_number( cc, trigger_mode_number );
        _check_error();
    }

    CamContext_start_camera(cc);
    _check_error();

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
            ROS_FATAL("do not know encoding for this format");
            exit(1);
    }
}

int CameraNode::run()
{
    bool got_frame = false;
    while (ros::ok()) {
        std::vector<uint8_t> data(step*height);

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

            if (!got_frame) {
                ROS_INFO("recieving images");
                got_frame = true;
            }

            sensor_msgs::Image msg;
            if (_host_timestamp) {
                msg.header.stamp = ros::Time::now();
            } else {
                double timestamp;
                CamContext_get_last_timestamp(cc,&timestamp);
                _check_error();
                msg.header.stamp = ros::Time(timestamp);
            }

            unsigned long framenumber;
            CamContext_get_last_framenumber(cc,&framenumber);
            _check_error();

            msg.header.seq = framenumber;
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
            cam_info.height = height;
            cam_info.width = width;

            publisher.publish(msg, cam_info);
        }

        ros::spinOnce();
    }

    CamContext_stop_camera(cc);
    cam_iface_shutdown();
    return 0;
}

int main(int argc, char** argv)
{
    CameraNode* cn = new CameraNode(argc,argv);
    return cn->run();
}
