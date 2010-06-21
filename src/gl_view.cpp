/*

Copyright (c) 2004-2010, California Institute of Technology.
Copyright (c) 2009, António Ramires Fernandes.
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
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#define USE_GLEW 1
#ifdef USE_GLEW
#  include <GL/glew.h>
#endif
#if defined(__APPLE__)
#  include <OpenGL/gl.h>
#  include <GLUT/glut.h>
#else
#  include <GL/gl.h>
#  include <GL/glut.h>
#endif

typedef enum MyCameraPixelCodings {   CAM_IFACE_UNKNOWN=0,
  CAM_IFACE_MONO8, /* pure monochrome (no Bayer) */
  CAM_IFACE_RGB8,
  CAM_IFACE_MONO16,
  CAM_IFACE_MONO8_BAYER_BGGR, /* BGGR Bayer coding */
  CAM_IFACE_MONO8_BAYER_RGGB, /* RGGB Bayer coding */
  CAM_IFACE_MONO8_BAYER_GRBG, /* GRBG Bayer coding */
  CAM_IFACE_MONO8_BAYER_GBRG  /* GBRG Bayer coding */ } MyCameraPixelCodings;


// globals (these are bad...)
bool did_first_frame = false;
int stride, width, height;
int use_pbo, use_shaders;
GLuint pbo;
size_t PBO_stride;
int tex_width, tex_height;
GLuint textureId;
double buf_wf, buf_hf;
GLint gl_data_format;
#ifdef USE_GLEW
GLhandleARB glsl_program;
#endif

static std::map<std::string, MyCameraPixelCodings> static_coding_map;

// forward declarations
void initialize_gl_texture(std::string encoding);
void upload_image_data_to_opengl(const unsigned char* raw_image_data,
                                 std::string encoding);
void setShaders();

void init_coding_map()
{
  static_coding_map[sensor_msgs::image_encodings::MONO8] = CAM_IFACE_MONO8;
  static_coding_map[sensor_msgs::image_encodings::MONO16] = CAM_IFACE_MONO16;
  static_coding_map[sensor_msgs::image_encodings::RGB8] = CAM_IFACE_RGB8;
  static_coding_map[sensor_msgs::image_encodings::BAYER_BGGR8] = CAM_IFACE_MONO8_BAYER_BGGR;
  static_coding_map[sensor_msgs::image_encodings::BAYER_RGGB8] = CAM_IFACE_MONO8_BAYER_RGGB;
  static_coding_map[sensor_msgs::image_encodings::BAYER_GRBG8] = CAM_IFACE_MONO8_BAYER_GRBG;
  static_coding_map[sensor_msgs::image_encodings::BAYER_GBRG8] = CAM_IFACE_MONO8_BAYER_GBRG;
}

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
  if (!did_first_frame) {
    fprintf(stdout,"Got first frame, setting up\n");

    // initialize everything...

    did_first_frame = true;

    stride = msg->step;
    width = msg->width; // XXX FIXME TODO: check when data != MONO8 or Bayer
    height = msg->height;

    initialize_gl_texture(msg->encoding);

    if (use_pbo) {
      glGenBuffers(1, &pbo);
      glBindBuffer(GL_PIXEL_UNPACK_BUFFER_ARB, pbo);
      glBufferData(GL_PIXEL_UNPACK_BUFFER_ARB,
                   PBO_stride*tex_height, 0, GL_STREAM_DRAW);
    }

    glEnable(GL_TEXTURE_2D);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

  }

  upload_image_data_to_opengl(&msg->data[0], msg->encoding);
  glutPostRedisplay(); /* trigger display redraw */
}

#define do_copy() {                                       \
  rowstart = dest;                                        \
  for (i=0; i<height; i++) {                              \
    memcpy(rowstart, src + (stride*i), width );           \
    rowstart += dest_stride;                              \
  }                                                       \
}

const unsigned char* convert_pixels(const unsigned char* src,
                     std::string src_coding,
                     size_t dest_stride,
                     unsigned char* dest, int force_copy) {
  static int gave_error=0;
  const unsigned char *src_ptr;
  static int attempted_to_start_glsl_program=0;
  GLubyte* rowstart;
  int i,j;
  int copy_required;
  GLint firstRed;

  copy_required = force_copy || (dest_stride!=stride);
  src_ptr = src;

  switch (static_coding_map[src_coding]) {
  case CAM_IFACE_MONO8:
    switch (gl_data_format) {
    case GL_LUMINANCE:
      if (copy_required) {
        do_copy();
        return dest;
      }
      return src; /* no conversion necessary*/
      break;
    default:
      fprintf(stderr,"ERROR: will not convert MONO8 image to non-luminance\n");
      exit(1);
      break;
    }
    break;
  case CAM_IFACE_RGB8:
    switch (gl_data_format) {
    case GL_RGB:
      if (copy_required) {
        // update data directly on the mapped buffer
        GLubyte* rowstart = dest;
        for (i=0; i<height; i++) {
          memcpy(rowstart, src_ptr, width*3 );
          rowstart += dest_stride;
          src_ptr += stride;
        }
        return dest;
      } else {
        return src; /* no conversion necessary*/
      }
      break;
    case GL_RGBA:
      if (copy_required) {
        // update data directly on the mapped buffer
        GLubyte* rowstart = dest;
        int j;
        for (i=0; i<height; i++) {
          for (j=0; j<width; j++) {
            rowstart[j*4] = src_ptr[j*3];
            rowstart[j*4+1] = src_ptr[j*3+1];
            rowstart[j*4+2] = src_ptr[j*3+2];
            rowstart[j*4+3] = 255;
          }
          rowstart += dest_stride;
          src_ptr += stride;
        }
        return dest;
      } else {
        return src; /* no conversion necessary*/
      }
      break;
    default:
      fprintf(stderr,"ERROR: invalid conversion at line %d\n",__LINE__);
      exit(1);
      break;
    }
    break;
#ifdef USE_GLEW
  case CAM_IFACE_MONO8_BAYER_BGGR:
  case CAM_IFACE_MONO8_BAYER_RGGB:
  case CAM_IFACE_MONO8_BAYER_GRBG:
  case CAM_IFACE_MONO8_BAYER_GBRG:


    //FIXME: add switch (gl_data_format)
    if (!attempted_to_start_glsl_program) {
      setShaders();
      if (use_shaders) {

        firstRed = glGetUniformLocation(glsl_program,"firstRed");
        switch(static_coding_map[src_coding]) {
        case CAM_IFACE_MONO8_BAYER_BGGR:
          glUniform2f(firstRed,0,0);
          break;
        case CAM_IFACE_MONO8_BAYER_RGGB:
          glUniform2f(firstRed,1,1);
          break;
        case CAM_IFACE_MONO8_BAYER_GRBG:
          glUniform2f(firstRed,0,1);
          break;
        case CAM_IFACE_MONO8_BAYER_GBRG:
        default:
          glUniform2f(firstRed,1,0);
          break;
        }
      } else {
        fprintf(stderr,"ERROR: Failed to start GLSL Bayer program\n");
      }
      attempted_to_start_glsl_program=1;
    }
    do_copy();
    return dest;
    break;
#endif
  case CAM_IFACE_MONO16:
    switch (gl_data_format) {
    case GL_LUMINANCE:
      rowstart = dest;
      for (i=0; i<height; i++) {
        for (j=0; j<width; j++) {
          rowstart[j] = (src + (stride*i))[j*2];
        }
        rowstart += dest_stride;
      }
      return dest;
      break;
    default:
      fprintf(stderr,"ERROR: invalid conversion at line %d\n",__LINE__);
      exit(1);
      break;
    }
  default:
    if (!gave_error) {
      fprintf(stderr,"ERROR: unsupported pixel coding %s\n",src_coding.c_str());
      gave_error=1;
    }
    if (copy_required) {
      do_copy();
      return dest;
    } else {
      return src; /* no conversion necessary*/
    }
    break;
  }
}

double next_power_of_2(double f) {
  return pow(2.0,ceil(log(f)/log(2.0)));
}

void initialize_gl_texture(std::string encoding) {
  int bytes_per_pixel;
  char *buffer;

  if (use_pbo) {
    // align
    PBO_stride = ((unsigned int)width/32)*32;
    if (PBO_stride<(unsigned int)width) PBO_stride+=32;
    tex_width = PBO_stride;
    tex_height = height;

  } else {
    PBO_stride = width;
    tex_width = (int)next_power_of_2(stride);
    tex_height = (int)next_power_of_2(height);
  }

  buf_wf = ((double)(width))/((double)tex_width);
  buf_hf = ((double)(height))/((double)tex_height);

  printf("for %dx%d image, allocating %dx%d texture (fractions: %.2f, %.2f)\n",
         width,height,tex_width,tex_height,buf_wf,buf_hf);

  if (static_coding_map[encoding]==CAM_IFACE_RGB8){
    bytes_per_pixel=4;
    gl_data_format = GL_RGBA;

    // This gives only grayscale images ATI fglrx Ubuntu Jaunty, but
    // transfers less data:

    //bytes_per_pixel=3;
    // gl_data_format = GL_RGB;

  } else if (static_coding_map[encoding]==CAM_IFACE_MONO16) {
    bytes_per_pixel=1;
    gl_data_format = GL_LUMINANCE;
  } else {
    bytes_per_pixel=1;
    gl_data_format = GL_LUMINANCE;
  }
  PBO_stride = PBO_stride*bytes_per_pixel; // FIXME this pads the rows more than necessary

  if (use_pbo) {
    printf("image stride: %d, PBO stride: %d\n",stride,(int)PBO_stride);
  }

  buffer = (char*)malloc( tex_height*PBO_stride );
  if (!buffer) {
    fprintf(stderr,"ERROR: failed to allocate buffer\n");
    exit(1);
  }

  int ncams=1;
  glGenTextures(ncams, &textureId);

  glBindTexture(GL_TEXTURE_2D, textureId);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D, // target
               0, // mipmap level
               GL_RGBA, // internal format
               tex_width, tex_height,
               0, // border
               gl_data_format, // format
               GL_UNSIGNED_BYTE, // type
               buffer);
  free(buffer);
  glBindTexture(GL_TEXTURE_2D, 0);
}

void display_pixels(void) {
  int i;
  int ncols,nrows,col_idx,row_idx;
  float wfrac, hfrac, lowx, highx, lowy, highy;
  nrows = 1;
  ncols = 1;
  wfrac = 2.0f/ncols;
  hfrac = 2.0f/nrows;

  i=0;

  if (i < ncols) {
    row_idx = 0;
  } else {
    row_idx = 1;
  }

  col_idx = i % ncols;
  lowx = col_idx*wfrac-1.0f;
  highx = (col_idx+1)*wfrac-1.0f;
  lowy = row_idx*hfrac-1.0f;
  highy = (row_idx+1)*hfrac-1.0f;

  //  printf("i %d\n col_idx %d, row_idx %d, lowx %f, highx %f, lowy %f, highy %f\n",
  // i,col_idx, row_idx, lowx, highx, lowy, highy );

  //glClear(GL_COLOR_BUFFER_BIT);
    glBindTexture(GL_TEXTURE_2D, textureId);
    glColor4f(1, 1, 1, 1);
    glBegin(GL_QUADS);

    glNormal3d(0, 0, 1);

    glTexCoord2f(0,0);
    glVertex3f(lowx,highy,0);

    glTexCoord2f(0,buf_hf);
    glVertex3f(lowx,lowy,0);

    glTexCoord2f(buf_wf,buf_hf);
    glVertex3f(highx,lowy,0);

    glTexCoord2f(buf_wf,0);
    glVertex3f(highx,highy,0);

    glEnd();

    glutSwapBuffers();
}

char *textFileRead(const char *fn) {


        FILE *fp;
        char *content = NULL;

        int count;
        fp = fopen(fn, "rt");
	if (fp == NULL) {
	  return NULL;
	}

	fseek(fp, 0, SEEK_END);
        count = ftell(fp);

        fseek(fp, 0, SEEK_SET);

        if (fn != NULL) {

                if (fp != NULL) {


                        if (count > 0) {
                                content = (char *)malloc(sizeof(char) * (count+1));
                                count = fread(content,sizeof(char),count,fp);
                                content[count] = '\0';
                        }
                        fclose(fp);
                }
        }
        return content;
}

#ifdef USE_GLEW
        void printShaderInfoLog(GLuint obj)
        {
            int infologLength = 0;
            int charsWritten  = 0;
            char *infoLog;

                glGetShaderiv(obj, GL_INFO_LOG_LENGTH,&infologLength);

            if (infologLength > 0)
            {
                infoLog = (char *)malloc(infologLength);
                glGetShaderInfoLog(obj, infologLength, &charsWritten, infoLog);
                        printf("%s\n",infoLog);
                free(infoLog);
            }
        }

        void printProgramInfoLog(GLuint obj)
        {
            int infologLength = 0;
            int charsWritten  = 0;
            char *infoLog;

                glGetProgramiv(obj, GL_INFO_LOG_LENGTH,&infologLength);

            if (infologLength > 0)
            {
                infoLog = (char *)malloc(infologLength);
                glGetProgramInfoLog(obj, infologLength, &charsWritten, infoLog);
                        printf("%s\n",infoLog);
                free(infoLog);
            }
        }

void setShaders() {

  const char * vv;
  const char * ff;
                char *vs,*fs;
                GLint status;
                GLhandleARB vertex_program,fragment_program;
                GLint sourceSize, shader_texture_source;

                vertex_program = glCreateShader(GL_VERTEX_SHADER);
                fragment_program = glCreateShader(GL_FRAGMENT_SHADER);

                vs = textFileRead("demosaic.vrt");
                if (vs==NULL) {
                  fprintf(stderr,"ERROR: failed to read vertex shader %s\n",
			  "demosaic.vrt");
                  use_shaders = 0;
                  return;
                }
                fs = textFileRead("demosaic.frg");
                if (fs==NULL) {
                  fprintf(stderr,"ERROR: failed to read fragment shader %s\n",
			  "demosaic.frg");
                  use_shaders = 0;
                  return;
                }

                vv = vs;
                ff = fs;

                glShaderSource(vertex_program, 1, &vv,NULL);
                glShaderSource(fragment_program, 1, &ff,NULL);

                free(vs);free(fs);

                glCompileShader(vertex_program);
                glGetShaderiv(vertex_program,GL_COMPILE_STATUS,&status);
                if (status!=GL_TRUE) {
                  fprintf(stderr,
                          "ERROR: GLSL vertex shader compile error, disabling shaders\n");
                  use_shaders = 0;
                  return;
                }

                glCompileShader(fragment_program);
                glGetShaderiv(fragment_program,GL_COMPILE_STATUS,&status);
                if (status!=GL_TRUE) {
                  fprintf(stderr,
                          "ERROR: GLSL fragment shader compile error, disabling shaders\n");
                  use_shaders = 0;
                  return;
                }

                //printShaderInfoLog(vertex_program);
                //printShaderInfoLog(fragment_program);

                glsl_program = glCreateProgram();
                glAttachShader(glsl_program,vertex_program);
                glAttachShader(glsl_program,fragment_program);
                glLinkProgram(glsl_program);

                //printProgramInfoLog(glsl_program);

                glGetProgramiv(glsl_program,GL_LINK_STATUS,&status);
                if (status!=GL_TRUE) {
                  fprintf(stderr,"ERROR: GLSL link error, disabling shaders\n");
                  use_shaders = 0;
                  return;
                }

                glUseProgram(glsl_program);


                sourceSize = glGetUniformLocation(glsl_program,"sourceSize");
                shader_texture_source = glGetUniformLocation(glsl_program,"source");

                glUniform4f(sourceSize,
                            PBO_stride,height,
                            1.0/PBO_stride,1.0/height);
                glUniform1i(shader_texture_source, 0);
}
#endif  /* ifdef USE_GLEW */

int main(int argc, char** argv) {
  init_coding_map();

  glutInit(&argc, argv);
  ros::init(argc, argv, "raw_view", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  if (nh.resolveName("image") == "/image") {
    ROS_WARN("raw_view: image has not been remapped! Typical command-line usage:\n"
             "\t$ ./raw_view image:=<image topic> [transport]");
  }

  std::string transport = (argc > 1) ? argv[1] : "raw";
  image_transport::Subscriber sub_;
  image_transport::ImageTransport it(nh);

  std::string topic = nh.resolveName("image");
  sub_ = it.subscribe(topic, 1, &image_cb, transport);

  glutInitWindowPosition(-1,-1);
  glutInitWindowSize(640, 480);
  glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE );
  glutCreateWindow(nh.resolveName("image").c_str());

  use_shaders=0;

#ifdef USE_GLEW
  glewInit();
  if (glewIsSupported("GL_VERSION_2_0 "
                      "GL_ARB_pixel_buffer_object")) {
    use_pbo=1;

    if (GLEW_ARB_vertex_shader && GLEW_ARB_fragment_shader) {
      printf("GLSL shaders present\n");
      use_shaders=1;
    }

  } else {
    printf("GLEW available, but no pixel buffer support -- not using PBO\n");
    use_pbo=0;
  }
#else
  printf("GLEW not available -- not using PBO\n");
  use_pbo=0;
#endif

  glutDisplayFunc(display_pixels); // set the display callback
  glutIdleFunc(ros::spinOnce); // set the idle callback

  glutMainLoop();
  return 0;
}

/* Send the data to OpenGL. Use the fastest possible method. */

void upload_image_data_to_opengl(const unsigned char* raw_image_data,
                                 std::string encoding) {
  const unsigned char * gl_image_data;
  static unsigned char* show_pixels=NULL;
  GLubyte* ptr;

  if (use_pbo) {
#ifdef USE_GLEW
    glBindTexture(GL_TEXTURE_2D, textureId);
    glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, pbo);

    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, tex_width, tex_height, gl_data_format, GL_UNSIGNED_BYTE, 0);
    glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, pbo);
    glBufferDataARB(GL_PIXEL_UNPACK_BUFFER_ARB, PBO_stride*tex_height, 0, GL_STREAM_DRAW_ARB);
    ptr = (GLubyte*)glMapBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, GL_WRITE_ONLY_ARB);
    if(ptr) {
      convert_pixels(raw_image_data, encoding, PBO_stride, ptr, 1);
      glUnmapBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB); // release pointer to mapping buffer
    }
    glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);
#endif  /* ifdef USE_GLEW */
  } else {

    if (show_pixels==NULL) {
      /* allocate memory */
      show_pixels = (unsigned char *)malloc( PBO_stride*height );
      if (show_pixels==NULL) {
        fprintf(stderr,"couldn't allocate memory in %s, line %d\n",__FILE__,__LINE__);
        exit(1);
      }
    }

    gl_image_data = convert_pixels(raw_image_data, encoding, PBO_stride, show_pixels, 0);

    glBindTexture(GL_TEXTURE_2D, textureId);
    glTexSubImage2D(GL_TEXTURE_2D, /* target */
                    0, /* mipmap level */
                    0, /* x offset */
                    0, /* y offset */
                    width,
                    height,
                    gl_data_format, /* data format */
                    GL_UNSIGNED_BYTE, /* data type */
                    gl_image_data);

  }
}
