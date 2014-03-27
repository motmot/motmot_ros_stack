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

#include <map>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <dynamic_reconfigure/server.h>
#include <camiface_ros/camera_configConfig.h>

#include <cam_iface.h>

typedef dynamic_reconfigure::Server<camiface_ros::camera_configConfig> DynamicReconfigureCameraConfig;

const std::string PROPERTY_NAME_SHUTTER("shutter");
const std::string PROPERTY_NAME_GAIN("gain");

// the level bitmasks are defined in the .cfg file
#define CFG_MASK_TRIGGER 0x01
#define CFG_MASK_SHUTTER 0x02
#define CFG_MASK_GAIN    0x04

#define _check_error() {                                                \
    int _check_error_err;                                               \
    _check_error_err = cam_iface_have_error();                          \
    if (_check_error_err != 0) {                                        \
      ROS_FATAL("%s:%d %s\n", __FILE__,__LINE__,cam_iface_get_error_string()); \
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
    CameraNode(ros::NodeHandle &node_priv, int argc, char** argv);
    int run();
    void config_callback(camiface_ros::camera_configConfig &config, uint32_t level);
private:
    CamContext *cc;
    ros::NodeHandle &_node_priv;
    std::map<std::string, int> _trigger_modes;
    std::map<std::string, int> _property_numbers;
    int _verbose;
    bool _got_frame;
    int step;
    std::string encoding;
    int width, height;
    sensor_msgs::CameraInfo cam_info;
    image_transport::CameraPublisher _pub_image;
    ros::Publisher _pub_rate;
    camera_info_manager::CameraInfoManager *cam_info_manager;
    int _host_timestamp;
    int _device_number;
    std::string _interface_name;
};

CameraNode::CameraNode(ros::NodeHandle &node_priv, int argc, char** argv) :
    _node_priv(node_priv),
    _verbose(0),
    _got_frame(false),
    _host_timestamp(0),
    _device_number(-1),
    _interface_name("")
{
    node_priv.param("host_timestamp", _host_timestamp, _host_timestamp);
    if (_host_timestamp)
        ROS_INFO("host timestamps ON");
    else
        ROS_INFO("host timestamps OFF");

    _node_priv.getParam("verbose", _verbose);

    /*
    if the user supplies a devicqe_guid or number (or puts it in the parameter server
    under the path of this node, use that. For example
        $ rosrun camiface_ros camnode _device_guid:=Prosilica-02-2020C-06732
    */

    int param_device_number = -1;
    _node_priv.getParam("device_number", param_device_number);

    std::string param_device_guid;
    _node_priv.getParam("device_guid", param_device_guid);
    int param_device_guid_int = -1;
    _node_priv.getParam("device_guid", param_device_guid_int);
    if (param_device_guid_int != -1 && param_device_guid.empty()) {
        ROS_WARN("stupid ros weakly typed parameter server - converting guid to string");
        std::stringstream out;
        out << param_device_guid_int;
        param_device_guid = out.str();
    }

    int param_device_mode = -1;
    _node_priv.getParam("device_mode", param_device_mode);

    _node_priv.getParam("interface_name", _interface_name);

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

    if (_verbose)
        ROS_INFO("%d camera(s) found",ncams);

    std::vector<std::string> safe_names;
    for (int i=0; i<ncams; i++) {
        Camwire_id cam_info_struct;
        cam_iface_get_camera_info(i, &cam_info_struct);
        if (cam_iface_have_error()==CAM_IFACE_CAMERA_NOT_AVAILABLE_ERROR) {
            ROS_WARN("camera %d: (not available)",i);
            cam_iface_clear_error();
            continue;
        }

        _check_error();
        if (_verbose)
            ROS_INFO("camera %d guid: %s",i,cam_info_struct.chip);
        std::string sn = make_safe_name(cam_info_struct.chip);
        safe_names.push_back( sn );
        ROS_DEBUG("camera safe name: %s",sn.c_str());

        if ((param_device_number != -1) && (param_device_number == i)) {
            _device_number = i;
            ROS_INFO("using user supplied device_number %d", param_device_number);
        } else if (param_device_guid.length() && (cam_info_struct.chip == param_device_guid)) {
            _device_number = i;
            ROS_INFO("using user supplied device_guid %s", param_device_guid.c_str());
        }
    }

    if (safe_names.empty()) {
        ROS_WARN("No cameras available");
        exit(1);
    } else if (_device_number == -1 && ((param_device_number != -1) || param_device_guid.length())) {
        ROS_WARN("Selected camera %s (%d) not found", param_device_guid.c_str(), param_device_number);
        exit(1);
    } else {
        Camwire_id cam_info_struct;

        if (_device_number == -1) {
            ROS_INFO("No explicitly selected camera");
            _device_number = 0; /* choose first camera if the user didn't specify */
        }

        cam_iface_get_camera_info(_device_number, &cam_info_struct);
        _check_error();
        ROS_INFO("choosing camera %d (%s guid:%s) on %s interface",
                    _device_number,
                    cam_info_struct.vendor, cam_info_struct.chip,
                    _interface_name.length() ? _interface_name.c_str() : "any");
    }

    int mode_number = 0;
    if (param_device_mode != -1) {
        ROS_INFO("user specified mode");
        mode_number = param_device_mode;
    } else {
        int num_modes;
        ROS_INFO("attempting to auto choose mode");
        cam_iface_get_num_modes(_device_number, &num_modes);
        _check_error();
        if (_verbose)
            ROS_INFO("%d mode(s) available:",num_modes);
        for (int i=0; i<num_modes; i++) {
            char mode_string[255];
            cam_iface_get_mode_string(_device_number,i,mode_string,255);
            if (strstr(mode_string,"FORMAT7_0")!=NULL) {
                if (strstr(mode_string,"MONO8")!=NULL) {
                    // pick this mode
                    mode_number = i;
                }
            }
            if (_verbose)
                ROS_INFO("  %d: %s",i,mode_string);
        }
    }
    ROS_INFO("chose mode %d", mode_number);

    cam_iface_constructor_func_t new_CamContext = cam_iface_get_constructor_func(_device_number);
    cc = new_CamContext(
            _device_number,
            5 /*num buffers*/,
            mode_number,
            _interface_name.length() ? _interface_name.c_str() : NULL);
    _check_error();

    int left, top;
    CamContext_get_frame_roi(cc, &left, &top, &width, &height);
    _check_error();

    // camera nodes must support at least shutter and gain
    int num_props;
    CamContext_get_num_camera_properties(cc,&num_props);
    _check_error();
    ROS_DEBUG("%d camera properties:",num_props);
    for (int i=0; i<num_props; i++) {
        CameraPropertyInfo cam_props;
        CamContext_get_camera_property_info(cc,i,&cam_props);
        _check_error();

        if (cam_props.is_present) {
            if (cam_props.available) {
                _property_numbers[std::string(cam_props.name)] = i;
                if (_verbose)
                    ROS_DEBUG("  %s (#%d): ",cam_props.name, i);
            }
        }
    }

    ROS_INFO("starting camera info manager for: %s", safe_names.at(_device_number).c_str());
    cam_info_manager = new camera_info_manager::CameraInfoManager(_node_priv, safe_names.at(_device_number));
    image_transport::ImageTransport *transport = new image_transport::ImageTransport(_node_priv);
    _pub_image = transport->advertiseCamera(_node_priv.resolveName("image_raw"), 1);

    _pub_rate = _node_priv.advertise<std_msgs::Float32>("framerate", 5);

    int num_trigger_modes;
    CamContext_get_num_trigger_modes( cc, &num_trigger_modes );
    _check_error();

    if (_verbose)
        ROS_INFO("%d trigger source(s):", num_trigger_modes);
    char mode[255];
    for (int i=0; i<num_trigger_modes; i++) {
        CamContext_get_trigger_mode_string( cc, i, mode, 255 );
        _trigger_modes[std::string(mode)] = i;
        if (_verbose)
            ROS_INFO("  %s (#%d)", mode, i);

    }

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
        case CAM_IFACE_YUV422:
            step = width;
            encoding = sensor_msgs::image_encodings::YUV422;
            break;
        default:
            ROS_FATAL("do not know encoding for this format: %x", cc->coding);
            exit(1);
    }

    CamContext_start_camera(cc);
    _check_error();
}

void CameraNode::config_callback(camiface_ros::camera_configConfig &config, uint32_t level)
{
    if (level & CFG_MASK_TRIGGER) {
        if (_trigger_modes.count(config.trigger)) {
            int i = _trigger_modes[config.trigger];
            ROS_INFO("setting trigger %s (#%d)", config.trigger.c_str(), i);
            CamContext_set_trigger_mode_number(cc, i);
            _check_error();
        }
    }

    if (level & CFG_MASK_SHUTTER) {
        if (_property_numbers.count(PROPERTY_NAME_SHUTTER)) {
            int i = _property_numbers[PROPERTY_NAME_SHUTTER];
            long usec = config.shutter;
            CamContext_set_camera_property(cc, i, usec, usec < 0);
            _check_error();
            ROS_INFO("setting shutter -> %ldus", usec);

        }
    }

    if (level & CFG_MASK_GAIN) {
        if (_property_numbers.count(PROPERTY_NAME_GAIN)) {
            int i = _property_numbers[PROPERTY_NAME_GAIN];
            long gain = config.gain;
            CamContext_set_camera_property(cc, i, gain, gain < 0);
            _check_error();
            ROS_INFO("setting gain -> %li", gain);
        }
    }
}

int CameraNode::run()
{
    ros::WallTime t_prev, t_now;
    int dt_avg;

    // reset timing information
    dt_avg = 0;
    t_prev = ros::WallTime::now();

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

            if (!_got_frame) {
                ROS_INFO("receiving images");
                _got_frame = true;
            }

            if(dt_avg++ == 9) {
                ros::WallTime t_now = ros::WallTime::now();
                ros::WallDuration dur = t_now - t_prev;
                t_prev = t_now;

                std_msgs::Float32 rate;
                rate.data = 10 / dur.toSec();
                _pub_rate.publish(rate);

                dt_avg = 0;
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

            _pub_image.publish(msg, cam_info);
        }

        ros::spinOnce();
    }

    CamContext_stop_camera(cc);
    cam_iface_shutdown();
    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camnode", ros::init_options::AnonymousName);
    ros::NodeHandle node;
    ros::NodeHandle node_priv("~");
    DynamicReconfigureCameraConfig config_srv;
    DynamicReconfigureCameraConfig::CallbackType cb;
    CameraNode* cn = new CameraNode(node_priv, argc,argv);
    cb = boost::bind(&CameraNode::config_callback, cn, _1, _2);
    config_srv.setCallback(cb);
    return cn->run();
}
