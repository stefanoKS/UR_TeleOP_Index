#include <librealsense2/rs.hpp>
#include <iostream>
#include <sstream>
#include <cstring>
#include <cstdint>
#include <algorithm>

// SDK-only child process: librealsense 2.56.5 embeds Fast DDS symbols that
// conflict with Humble's Fast DDS. Never load both libraries into one process.
void intrinsics(std::ostream& out,const rs2_intrinsics& intr) {
  const bool undistorted=std::all_of(intr.coeffs,intr.coeffs+5,[](float v){return v==0.f;});
  const char* model=undistorted || intr.model==RS2_DISTORTION_NONE || intr.model==RS2_DISTORTION_BROWN_CONRADY
    ? "plumb_bob" : intr.model==RS2_DISTORTION_INVERSE_BROWN_CONRADY ? "inverse_brown_conrady" : nullptr;
  if(!model) throw std::runtime_error("Unsupported camera distortion model");
  // Preserve inverse Brown calibration explicitly; never mislabel its coefficients
  // as forward Brown. Raw images remain unrectified; SDK deprojection supports it.
  out << "{\"model\":\""<<model<<"\",\"fx\":"<<intr.fx<<",\"fy\":"<<intr.fy<<",\"ppx\":"<<intr.ppx<<",\"ppy\":"<<intr.ppy<<",\"d\":[";
  for(int i=0;i<5;++i) out<<(i?",":"")<<intr.coeffs[i];
  out<<"]}";
}
int main(int argc,char** argv) {
  try {
    if(argc!=2) throw std::runtime_error("Expected D405 serial");
    rs2::pipeline pipeline; rs2::config config;
    config.enable_device(argv[1]);
    config.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_RGB8,15);
    config.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,15);
    auto profile=pipeline.start(config);
    if(std::string(profile.get_device().get_info(RS2_CAMERA_INFO_NAME)).find("D405")==std::string::npos)
      throw std::runtime_error("Selected device is not a D405");
    auto color_intr=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    auto depth_intr=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
    const float units=profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
    std::cerr<<"D405 "<<argv[1]<<" 640x480@15; depth units="<<units<<" m; arm mount uncalibrated\n";
    while(std::cout) {
      auto frames=pipeline.wait_for_frames(3000);
      auto rgb=frames.get_color_frame(); auto depth=frames.get_depth_frame();
      if(!rgb || !depth) continue;
      std::ostringstream info; info.precision(12);
      info<<"{\"width\":"<<rgb.get_width()<<",\"height\":"<<rgb.get_height()
          <<",\"color_stride\":"<<rgb.get_stride_in_bytes()<<",\"depth_stride\":"<<depth.get_stride_in_bytes()
          <<",\"depth_units\":"<<units<<",\"color\":";
      intrinsics(info,color_intr); info<<",\"depth\":"; intrinsics(info,depth_intr); info<<"}";
      const auto metadata=info.str();
      // Local pipe protocol: magic, three uint32 lengths, JSON, RGB8, Z16.
      const uint32_t lengths[3]={static_cast<uint32_t>(metadata.size()),
        static_cast<uint32_t>(rgb.get_data_size()),static_cast<uint32_t>(depth.get_data_size())};
      std::cout.write("D405",4);
      std::cout.write(reinterpret_cast<const char*>(lengths),sizeof(lengths));
      std::cout.write(metadata.data(),metadata.size());
      std::cout.write(static_cast<const char*>(rgb.get_data()),lengths[1]);
      std::cout.write(static_cast<const char*>(depth.get_data()),lengths[2]);
      std::cout.flush();
    }
    pipeline.stop();
  } catch(const std::exception& error) {std::cerr<<error.what()<<std::endl;return 1;}
}
