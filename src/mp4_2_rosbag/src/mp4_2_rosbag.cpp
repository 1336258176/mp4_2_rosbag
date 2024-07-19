#include "mp4_2_rosbag/mp4_2_rosbag.hpp"

MP4_2_ROSBAG::MP4_2_ROSBAG(std::string mp4_file) : rclcpp::Node("mp4_2_rosbag") {
  RCLCPP_INFO(this->get_logger(), "initializing...");
  path_.assign(mp4_file);
  path_.make_preferred();
  output_path_.make_preferred();

  if (path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No input file provided");
    return;
  }

  if (!std::filesystem::exists(output_path_)) {
    std::filesystem::create_directory(output_path_);
  }

  RCLCPP_INFO(this->get_logger(), "Start reading and converting...");
  if (!std::filesystem::is_directory(path_)) {
    refresh_bagstorage(output_name_);
    read_and_convert(path_.string());
  } else {
    find_mp4_files(path_, mp4_files_);
    for (const auto& file : mp4_files_) {
      refresh_bagstorage(file.stem().string());
      read_and_convert((path_ / file).string());
    }
  }
  RCLCPP_INFO(this->get_logger(), "Conversion complete");
}

void MP4_2_ROSBAG::find_mp4_files(const std::filesystem::path& path,
                                  std::vector<std::filesystem::path>& mp4_files) {
  try {
    for (const auto& entry : std::filesystem::directory_iterator(path)) {
      if (entry.is_regular_file()) {
        mp4_files.push_back(entry.path().filename().string());
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error reading directory: %s", e.what());
  }
}

void MP4_2_ROSBAG::read_and_convert(std::string file) {
  cap_ = std::make_shared<cv::VideoCapture>(file);
  if (!cap_->isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open the video file");
    return;
  }

  cv::Mat frame;
  while (cap_->read(frame)) {
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    try {
      writer_->write(*msg, "/camera/front/capture", this->now());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error converting frame: %s", e.what());
      continue;
    }
    cv::waitKey(1000.0 / FPS);
  }
}

int main(int argc, char* argv[]) {
  if (argc < 2 || std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
    std::clog << "Usage: mp4_2_rosbag <mp4_files>" << std::endl;
    std::clog << "\nbrief: Convert video files to ros2 bag.(Not just MP4)\n"
              << std::endl;
    std::clog << "example: mp4_2_rosbag /path/to/video_file" << std::endl;
    std::clog << "         mp4_2_rosbag /path/to/directory" << std::endl;
    return 0;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin_some(std::make_shared<MP4_2_ROSBAG>(argv[1]));
  rclcpp::shutdown();
  return 0;
}