#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <vector>
#include <limits>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <fstream>




cv::Mat frame_to_mat_color(const rs2::frame& frame) {
    rs2::video_frame video_frame = frame.as<rs2::video_frame>();
    const int width = video_frame.get_width();
    const int height = video_frame.get_height();
    const int bpp = video_frame.get_bytes_per_pixel();
    return cv::Mat(cv::Size(width, height), CV_8UC(bpp), (void*)video_frame.get_data(), cv::Mat::AUTO_STEP);
}
cv::Mat frame_to_mat(const rs2::frame& frame) {
    rs2::video_frame video_frame = frame.as<rs2::video_frame>();
    const int width = video_frame.get_width();
    const int height = video_frame.get_height();
    const int bpp = video_frame.get_bytes_per_pixel();
    return cv::Mat(cv::Size(width, height), bpp, (void*)video_frame.get_data(), cv::Mat::AUTO_STEP);
}

//time_stamp_type - 需要获取的时间戳的级别，0表示秒级时间戳，1表示毫秒级时间戳，2表示微秒级时间戳，3表示纳秒级时间戳
std::string GetCurrentTimeStamp(int time_stamp_type = 0)
{
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

	std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
	std::tm* now_tm = std::localtime(&now_time_t);

	char buffer[128];
	strftime(buffer, sizeof(buffer), "%F %T", now_tm);

	std::ostringstream ss;
	ss.fill('0');

	std::chrono::milliseconds ms;
	std::chrono::microseconds cs;
	std::chrono::nanoseconds ns;
	
	switch (time_stamp_type)
	{
	case 0:
		ss << buffer;
		break;
	case 1:
		ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
		ss << buffer << ":" << ms.count();
		break;
	case 2:
		ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
		cs = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
		ss << buffer << ":" << ms.count() << ":" << cs.count() % 1000;
		break;
	case 3:
		ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
		cs = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
		ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()) % 1000000000;
		ss << buffer << ":" << ms.count() << ":" << cs.count() % 1000 << ":" << ns.count() % 1000;
		break;
	default:
		ss << buffer;
		break;
	}

	return ss.str();
}

//定义一个结构体，代表 linux里目录分层符号 ‘/’，再后面std：：find_if 查找符号的函数中会用到
struct pathseperator
{
    pathseperator ()
    { }

    bool
    operator () (char ch) const
    {
        return ch == '/';
    }
};

//检查路径是否已创建
int checkFolderExist(std::string const & name)
{
    if (access(name.c_str (), 0))
        return -1;
    return 0;
}

int checkandmkdir(std::string path_input)
{
    std::string sep = "/";
    //第一个参数为需要创建的全路径
   std::string target = std::string(path_input);
   
   //通过查找 / 的方式将每层路径拆开 
    std::vector<std::string> container;
    std::string::const_iterator const end = target.end ();
    std::string::const_iterator it = target.begin ();
    pathseperator pathsep = pathseperator ();
    while (it != end)
    {
        std::string::const_iterator sep = std::find_if (it, end, pathsep);
        container.push_back (std::string (it, sep));
        it = sep;
        if (it != end)
            ++it;
    }

	//将拆解的路径重新一层一层的拼接，并调用系统函数mkdir创建
    std::string path;
    for(int i=1; i<container.size(); i++) {
        path += sep;
        path += container[i];
        if(checkFolderExist(path) == 0) {
            continue;
        }
        int res = mkdir(path.c_str (), 0777);
    }
    return 0;
}

int checkandcreattxt(std::string path_input)
{
    if(checkFolderExist(path_input)==0)
    {
        return 0;
    }
    else
    {

    }
    return 1;
}




int main()
{
   
    // Create a single context for managing all the realsense devices.
    rs2::context ctx;
    std::vector<std::thread> threads;
   
    std::size_t dev_id = 0;
    std::string data_path = "/home/merlincs/hwsyn/Data";
    for (auto&& dev : ctx.query_devices())
    {
        threads.emplace_back([dev, dev_id,data_path](){

            std::cout << "Device (Thread) ID: " << dev_id << std::endl;
            std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            std::cout << "Camera with serial number: " << serial << std::endl;

            auto advanced_dev = dev.as<rs400::advanced_mode>();
            auto advanced_sensors = advanced_dev.query_sensors();

            bool depth_found = false;
            bool color_found = false;
            bool infrared1_found = false;
            bool infrared2_found = false;
            rs2::sensor depth_sensor;
            rs2::sensor color_sensor;
            rs2::sensor infrared1_sensor;
            rs2::sensor infrared2_sensor;


            for (auto&& sensor : advanced_sensors) {
                std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
                std::cout << module_name << std::endl;

                if (module_name == "Stereo Module") {
                    depth_sensor = sensor;
                    depth_found = true;
                } else if (module_name == "RGB Camera") {
                    color_sensor = sensor;
                    color_found = true;
                }else if (module_name == "RGB Camera") {

                }
            }

            if (!(depth_found && color_found)) {
                std::cout << "Unable to find both stereo and color modules" <<
                    std::endl;
            }
            
            depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
            depth_sensor.set_option(RS2_OPTION_EXPOSURE, 8500); // microseconds
            depth_sensor.set_option(RS2_OPTION_GAIN, 16);
            depth_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 1);
            depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);

            // RGB sync doesn't work, need to use depth as master.
                std::cout << "Setting " << dev_id << " to slave!" << std::endl;
                depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 4);

            color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
            color_sensor.set_option(RS2_OPTION_EXPOSURE, 100); // 1/10 ms (10)
            color_sensor.set_option(RS2_OPTION_GAIN, 64);
            color_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 1);

            rs2::pipeline pipe;
            rs2::config cfg;
            cfg.enable_device(serial);
            cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_BGR8, 30);
            cfg.enable_stream(RS2_STREAM_INFRARED, 1, 848, 480, RS2_FORMAT_Y8, 30);
            cfg.enable_stream(RS2_STREAM_INFRARED, 2, 848, 480, RS2_FORMAT_Y8, 30);
            cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
            cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
            cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

            rs2::pipeline_profile profile = pipe.start(cfg);

            double last_time = 0;
            int lastnum=9999;
            int frame_counter=0;

            for (int frame_count = 0; ; ++frame_count) {

                rs2::frameset frames = pipe.wait_for_frames();
                std::string timestamp_now = GetCurrentTimeStamp(1);
                rs2_vector accel_sample;
                rs2_vector gyro_sample;


                // Get imu data
                if (rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL))
                {
                    accel_sample = accel_frame.get_motion_data();
                }
                if (rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO))
                {
                    gyro_sample = gyro_frame.get_motion_data();
                }


                rs2::frame color_frame = frames.get_color_frame();
                rs2::frame left_frame = frames.get_infrared_frame(1);
                rs2::frame right_frame = frames.get_infrared_frame(2);
                rs2::frame depth_frame = frames.get_depth_frame();
                frame_counter = depth_frame.get_frame_number();
                if(lastnum==frame_counter)
                continue;

                std::cout << frames.size() << " frames from " << serial << ": ";
                std::cout.precision(std::numeric_limits<double>::max_digits10);

                std::cout << "Drift: " << depth_frame.get_timestamp() - last_time << ", ";
                last_time = depth_frame.get_timestamp();

                std::cout << "frame numbers: " << depth_frame.get_frame_number() << ", ";
                lastnum=depth_frame.get_frame_number();

                for (const rs2::frame& f : {(rs2::frame)frames, depth_frame, color_frame}) {
                    switch(f.get_frame_timestamp_domain()) {
                        case (RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK):
                            std::cout << "Hardware Clock ";
                            break;
                        case (RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME):
                            std::cout << "System Time ";
                            break;
                        default:
                            std::cout << "Unknown ";
                            break;
                    }
                    std::cout << "TS: " << std::scientific << f.get_timestamp() 
                        << "(" << f.get_frame_number() << "), ";
                }
                std::cout << std::endl;

                if(checkandcreattxt(data_path + "/d435i_" + std::to_string(dev_id) + "imu.txt")){
                    std::ofstream dataFile;
	                dataFile.open(data_path + "/d435i_" + std::to_string(dev_id) + "imu.txt", std::ofstream::app);
                    std::fstream file(data_path + "/d435i_" + std::to_string(dev_id) + "imu.txt", std::ios::out); // 打开文件
                    file << std::to_string(frame_counter) + "_" << timestamp_now << " Accel:" << accel_sample.x << "," << accel_sample.y << "," << accel_sample.z << ";Gyro:" << gyro_sample.x << "," << gyro_sample.y << "," << gyro_sample.z << std::endl; // 将新文本写入文件
                }
                else{
                    std::fstream file(data_path + "/d435i_" + std::to_string(dev_id) + "imu.txt", std::ios::in | std::ios::out | std::ios::app); // 打开文件
                    if (file.is_open()) { // 检查是否成功打开
                    file << std::to_string(frame_counter) + "_" << timestamp_now << " Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << ";Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl; // 将新文本写入文件
                    file.close();} // 关闭文件流
                    else{
                    std::cout << "Failed to modify file." << std::endl;
                    }
                }

                //save pictures
                cv::Mat color_mat = frame_to_mat_color(color_frame);
                cv::Mat depth_mat = frame_to_mat(depth_frame);
                cv::Mat left_mat = frame_to_mat(left_frame);
                cv::Mat right_mat = frame_to_mat(right_frame);
                std::string color_filepath = data_path + "/d435i_" + std::to_string(dev_id) + "/color";
                std::string depth_filepath = data_path + "/d435i_" + std::to_string(dev_id) + "/depth";
                std::string left_filepath = data_path + "/d435i_" + std::to_string(dev_id) + "/left";
                std::string right_filepath = data_path + "/d435i_" + std::to_string(dev_id) + "/right";
                checkandmkdir(color_filepath);
                checkandmkdir(depth_filepath);
                checkandmkdir(left_filepath);
                checkandmkdir(right_filepath);
                std::string color_filename = color_filepath + "/color_image" + std::to_string(frame_counter) + "_" + timestamp_now + ".png";
                std::string depth_filename = depth_filepath + "/depth_image" + std::to_string(frame_counter) + "_" + timestamp_now + ".png";
                std::string left_filename = left_filepath + "/left_image" + std::to_string(frame_counter) + "_" + timestamp_now + ".png";
                std::string right_filename = right_filepath + "/right_image" + std::to_string(frame_counter) + "_" + timestamp_now + ".png";
                cv::imwrite(color_filename, color_mat);
                cv::imwrite(depth_filename, depth_mat);
                cv::imwrite(left_filename, left_mat);
                cv::imwrite(right_filename, right_mat);
            }
        });

        dev_id++;
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

	for (auto& t : threads) t.join(); // Must join / detach all threads
}