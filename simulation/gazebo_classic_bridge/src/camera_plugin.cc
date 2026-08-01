#include <cerrno>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/Sensor.hh>

namespace gazebo
{
namespace
{
void WriteLe16(std::ostream &stream, const std::uint16_t value)
{
  const char bytes[2] = {
    static_cast<char>(value & 0xffu),
    static_cast<char>((value >> 8u) & 0xffu),
  };
  stream.write(bytes, 2);
}

void WriteLe32(std::ostream &stream, const std::uint32_t value)
{
  const char bytes[4] = {
    static_cast<char>(value & 0xffu),
    static_cast<char>((value >> 8u) & 0xffu),
    static_cast<char>((value >> 16u) & 0xffu),
    static_cast<char>((value >> 24u) & 0xffu),
  };
  stream.write(bytes, 4);
}
}  // namespace

class CbfRoverCameraPlugin final : public SensorPlugin
{
public:
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override
  {
    sensor_ = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
    if (!sensor_)
    {
      throw std::runtime_error(
        "cbf_rover camera plugin requires a CameraSensor"
      );
    }
    if (!sdf->HasElement("output_path"))
    {
      throw std::runtime_error(
        "cbf_rover camera plugin requires output_path"
      );
    }
    output_path_ = sdf->Get<std::string>("output_path");
    if (output_path_.empty())
    {
      throw std::runtime_error("camera output_path must not be empty");
    }
    temporary_path_ = output_path_ + ".tmp";
    if (sdf->HasElement("queue_directory"))
    {
      queue_directory_ = sdf->Get<std::string>("queue_directory");
      if (queue_directory_.empty())
      {
        throw std::runtime_error("camera queue_directory must not be empty");
      }
      queue_enabled_ = true;
    }
    camera_ = sensor_->Camera();
    if (!camera_)
    {
      throw std::runtime_error("CameraSensor has no rendering camera");
    }
    connection_ = camera_->ConnectNewImageFrame(
      std::bind(
        &CbfRoverCameraPlugin::OnNewFrame,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4,
        std::placeholders::_5
      )
    );
    sensor_->SetActive(true);
  }

private:
  void OnNewFrame(
    const unsigned char *image,
    const unsigned int width,
    const unsigned int height,
    const unsigned int depth,
    const std::string &format
  )
  {
    if (image == nullptr || width == 0u || height == 0u || depth != 3u)
    {
      return;
    }
    const bool rgb = format == "R8G8B8";
    const bool bgr = format == "B8G8R8";
    if (!rgb && !bgr)
    {
      if (!unsupported_format_reported_)
      {
        gzerr << "Unsupported cbf_rover camera image format: "
              << format << "\n";
        unsupported_format_reported_ = true;
      }
      return;
    }

    std::lock_guard<std::mutex> guard(write_mutex_);
    const common::Time capture_time = sensor_->LastMeasurementTime();
    const std::uint64_t next_frame_index = frame_index_ + 1u;
    std::ostringstream metadata_stream;
    metadata_stream << "\nCBF_ROVER_CAMERA_FRAME_V1 "
                    << capture_time.sec << " "
                    << capture_time.nsec << " "
                    << next_frame_index << "\n";
    const std::string metadata = metadata_stream.str();
    const std::uint32_t row_bytes = width * 3u;
    const std::uint32_t row_stride = (row_bytes + 3u) & ~3u;
    const std::uint32_t pixel_bytes = row_stride * height;
    const std::uint32_t file_bytes =
      54u + pixel_bytes + static_cast<std::uint32_t>(metadata.size());
    const std::string queue_frame_path =
      queue_enabled_ ? QueueFramePath(next_frame_index) : std::string();
    const std::string frame_temporary_path =
      queue_enabled_ ? queue_frame_path + ".tmp" : temporary_path_;
    std::ofstream stream(
      frame_temporary_path,
      std::ios::binary | std::ios::out | std::ios::trunc
    );
    if (!stream)
    {
      gzerr << "Cannot open camera frame temporary file: "
            << frame_temporary_path << "\n";
      return;
    }

    stream.put('B');
    stream.put('M');
    WriteLe32(stream, file_bytes);
    WriteLe16(stream, 0u);
    WriteLe16(stream, 0u);
    WriteLe32(stream, 54u);
    WriteLe32(stream, 40u);
    WriteLe32(stream, width);
    WriteLe32(stream, height);
    WriteLe16(stream, 1u);
    WriteLe16(stream, 24u);
    WriteLe32(stream, 0u);
    WriteLe32(stream, pixel_bytes);
    WriteLe32(stream, 2835u);
    WriteLe32(stream, 2835u);
    WriteLe32(stream, 0u);
    WriteLe32(stream, 0u);

    const std::vector<char> padding(row_stride - row_bytes, 0);
    for (unsigned int output_y = 0u; output_y < height; ++output_y)
    {
      const unsigned int source_y = height - 1u - output_y;
      const unsigned char *row = image + source_y * width * depth;
      for (unsigned int x = 0u; x < width; ++x)
      {
        const unsigned char *pixel = row + x * depth;
        if (rgb)
        {
          stream.put(static_cast<char>(pixel[2]));
          stream.put(static_cast<char>(pixel[1]));
          stream.put(static_cast<char>(pixel[0]));
        }
        else
        {
          stream.write(reinterpret_cast<const char *>(pixel), 3);
        }
      }
      if (!padding.empty())
      {
        stream.write(padding.data(), padding.size());
      }
    }
    stream.write(metadata.data(), metadata.size());
    stream.close();
    if (!stream)
    {
      gzerr << "Failed while writing camera frame: "
            << frame_temporary_path << "\n";
      RemoveBestEffort(frame_temporary_path);
      return;
    }
    if (queue_enabled_)
    {
      if (!PublishQueuedFrame(frame_temporary_path, queue_frame_path))
      {
        return;
      }
    }
    else if (std::rename(temporary_path_.c_str(), output_path_.c_str()) != 0)
    {
      const int error_number = errno;
      RemoveBestEffort(temporary_path_);
      gzerr << "Cannot atomically publish camera frame: "
            << output_path_ << ": " << std::strerror(error_number) << "\n";
      return;
    }
    frame_index_ = next_frame_index;
  }

  std::string QueueFramePath(const std::uint64_t frame_index) const
  {
    std::ostringstream path;
    path << queue_directory_;
    if (queue_directory_.back() != '/')
    {
      path << '/';
    }
    path << "frame_" << std::setfill('0') << std::setw(9)
         << static_cast<unsigned long long>(frame_index) << ".bmp";
    return path.str();
  }

  void RemoveBestEffort(const std::string &path) const
  {
    if (std::remove(path.c_str()) != 0 && errno != ENOENT)
    {
      const int error_number = errno;
      gzerr << "Cannot remove camera temporary artifact: " << path
            << ": " << std::strerror(error_number) << "\n";
    }
  }

  bool PublishQueuedFrame(
    const std::string &frame_temporary_path,
    const std::string &queue_frame_path
  )
  {
    if (std::remove(temporary_path_.c_str()) != 0 && errno != ENOENT)
    {
      const int error_number = errno;
      RemoveBestEffort(frame_temporary_path);
      gzerr << "Cannot clear stale latest-camera temporary file: "
            << temporary_path_ << ": " << std::strerror(error_number)
            << "\n";
      return false;
    }
    if (::link(frame_temporary_path.c_str(), temporary_path_.c_str()) != 0)
    {
      const int error_number = errno;
      RemoveBestEffort(frame_temporary_path);
      gzerr << "Cannot hard-link camera frame to latest temporary file: "
            << frame_temporary_path << " -> " << temporary_path_ << ": "
            << std::strerror(error_number) << "\n";
      return false;
    }
    if (std::rename(temporary_path_.c_str(), output_path_.c_str()) != 0)
    {
      const int error_number = errno;
      RemoveBestEffort(temporary_path_);
      RemoveBestEffort(frame_temporary_path);
      gzerr << "Cannot atomically publish latest queued camera frame: "
            << output_path_ << ": " << std::strerror(error_number) << "\n";
      return false;
    }
    if (
      std::rename(
        frame_temporary_path.c_str(),
        queue_frame_path.c_str()
      ) != 0
    )
    {
      const int error_number = errno;
      RemoveBestEffort(frame_temporary_path);
      gzerr << "Cannot atomically publish queued camera frame: "
            << queue_frame_path << ": " << std::strerror(error_number)
            << "\n";
      return false;
    }
    return true;
  }

  sensors::CameraSensorPtr sensor_;
  rendering::CameraPtr camera_;
  event::ConnectionPtr connection_;
  std::string output_path_;
  std::string temporary_path_;
  std::string queue_directory_;
  std::mutex write_mutex_;
  std::uint64_t frame_index_{0u};
  bool queue_enabled_{false};
  bool unsupported_format_reported_{false};
};

GZ_REGISTER_SENSOR_PLUGIN(CbfRoverCameraPlugin)
}  // namespace gazebo
