#ifndef MP4_2_ROSBAG_HPP__
#define MP4_2_ROSBAG_HPP__

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <filesystem>
#include <memory>
#include <vector>
#include <string>

class MP4_2_ROSBAG : public rclcpp::Node
{
public:
  MP4_2_ROSBAG(std::string mp4_file);

  ~MP4_2_ROSBAG() {
    writer_->close();
    cap_->release();
  }

private:
  void read_and_convert(std::string file);
  void find_mp4_files(const std::filesystem::path& path, std::vector<std::filesystem::path>& mp4_files);

  void refresh_bagstorage(std::string file_name) {
    writer_.reset(new rosbag2_cpp::Writer());
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = output_path_.string() + "/" + file_name;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    writer_->open(storage_options, converter_options);
  }

  constexpr static size_t FPS = 60;
  std::filesystem::path path_{};
  std::filesystem::path output_path_{"./output"};
  std::string output_name_{"output"};

  std::shared_ptr<rosbag2_cpp::Writer> writer_{};
  std::shared_ptr<cv::VideoCapture> cap_{};
  std::vector<std::filesystem::path> mp4_files_{};
};

#endif // MP4_2_ROSBAG_HPP__