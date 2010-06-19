#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
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

int main(int argc, char** argv)
{
  CamContext *cc;
  int num_buffers;

  ros::init(argc, argv, "camiface_ros_capture");


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


  printf("%d camera(s) found.\n",ncams);
  for (int i=0; i<ncams; i++) {
    Camwire_id cam_info_struct;
    cam_iface_get_camera_info(i, &cam_info_struct);
    printf("  camera %d:\n",i);
    printf("    vendor: %s\n",cam_info_struct.vendor);
    printf("    model: %s\n",cam_info_struct.model);
    printf("    chip: %s\n",cam_info_struct.chip);
  }

  int device_number = ncams-1;

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
  int width, height;
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

  // topic is "image_raw", with queue size of 5
  ros::Publisher publisher = n.advertise<sensor_msgs::Image>("image_raw", 5);

  CamContext_start_camera(cc);
  _check_error();

  printf("will now run forever. press Ctrl-C to interrupt\n");

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

  int step;
  std::string encoding;
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

      sensor_msgs::Image msg;
      msg.height = height;
      msg.width = width;
      msg.encoding = encoding;
      msg.step = step;
      msg.data = data;

      publisher.publish(msg);
    }
    ros::spinOnce();
  }
}
