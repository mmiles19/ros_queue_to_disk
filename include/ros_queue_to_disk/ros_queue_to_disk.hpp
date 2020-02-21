#ifndef ROS_QUEUE_TO_DISK_H
#define ROS_QUEUE_TO_DISK_H

#include <ros/ros.h>
#include <fstream>
// #include <filesystem>
#include <experimental/filesystem>
#include <boost/thread.hpp>
#include <cerrno>
#include <queue>
#include <ctime>
#include <string>
// #include <mutex>

#ifdef DEBUG_TEST_IMAGE
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#endif

namespace fs = std::experimental::filesystem;

// void demo_perms(fs::perms p)
// {
//     std::cout << ((p & fs::perms::owner_read) != fs::perms::none ? "r" : "-")
//               << ((p & fs::perms::owner_write) != fs::perms::none ? "w" : "-")
//               << ((p & fs::perms::owner_exec) != fs::perms::none ? "x" : "-")
//               << ((p & fs::perms::group_read) != fs::perms::none ? "r" : "-")
//               << ((p & fs::perms::group_write) != fs::perms::none ? "w" : "-")
//               << ((p & fs::perms::group_exec) != fs::perms::none ? "x" : "-")
//               << ((p & fs::perms::others_read) != fs::perms::none ? "r" : "-")
//               << ((p & fs::perms::others_write) != fs::perms::none ? "w" : "-")
//               << ((p & fs::perms::others_exec) != fs::perms::none ? "x" : "-")
//               << '\n';
// }

typedef std::chrono::high_resolution_clock Time;

namespace ros_queue_to_disk {
// using namespace fs = std::filesystem;

template <typename T>
class ServiceClient
{
    private:
    ros::NodeHandle* _nh;
    std::string _srv_name;
    std::string _queue_dir;
    std::fstream _fstream;
    ros::ServiceClient _client;
    boost::thread* _dequeue_thread;
    // uint _queue_size;
    uint _queue_idx_incr = 0;
    uint _max_queue_size; // queue length (num elements)
    ulong _max_queue_dir_size; // in bytes
    ulong _current_queue_dir_size = 0;
    bool _comm_status_is_connected;
    uint _comm_status_count;
    // std::mutex _file_mutex;
    std::queue<std::string> _queued_files;
    // std::time_t _timer;
    // float _timer;
    Time::time_point _timer;
    const float _timer_limit;

    void setCommStatus(bool status)
    {
        if(_comm_status_is_connected != status){ _comm_status_count = 0; }
        _comm_status_count++;
        _comm_status_is_connected = status;
    }
    std::string calcFileName(T item)
    {
        uint idx = _queue_idx_incr++;
        char format_char[std::to_string(_max_queue_size).length()+1];
        sprintf(format_char,"%0*u",std::to_string(_max_queue_size).length(),idx);
        std::string filename = std::string(ros::service_traits::md5sum(item)) + "_" + std::string(format_char);
        return filename;
    }
    std::string calcFilePath(T item)
    {
        std::string filepath = _queue_dir + calcFileName(item);
        return filepath;
    }
    bool queueToDisk(T item)
    {
        if (_queued_files.size() >= _max_queue_size) 
        { 
            ROS_WARN("Queue full. Aborting queueToDisk.");
            return false; 
        }
        if ( (_current_queue_dir_size+sizeof(item)) > _max_queue_dir_size) 
        { 
            ROS_WARN("Queue directory full. Aborting queueToDisk.");
            return false; 
        }
       
        std::string filepath = calcFilePath(item);
        uint32_t serial_size = ros::serialization::serializationLength(item.request);
        char* buffer = new char[serial_size];
        ros::serialization::OStream stream(reinterpret_cast<uint8_t*>(buffer), serial_size);
        ros::serialization::serialize(stream, item.request);
        ROS_INFO("Queuing %s. Size %d.", filepath.c_str(), sizeof(item));
        try
        {
            // std::lock_guard<std::mutex> lock(_file_mutex);
            // fs::permissions(filepath,
            //     fs::perms::owner_all | fs::perms::group_all | fs::perms::others_all);
            // demo_perms(fs::status(filepath).permissions());
            _fstream.open(filepath, std::fstream::out | std::fstream::binary);
            
            if(!_fstream)
            {
                ROS_ERROR("Failed to open file: %s.", std::strerror(errno));
                return false;
            }
            _fstream.write(buffer,serial_size);
            _fstream.close();
        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << '\n';
            _fstream.close();
            // std::lock_guard<std::mutex> unlock(_file_mutex);
            return false;
        }
        // std::lock_guard<std::mutex> unlock(_file_mutex);
        // _queue_size++;
        _queued_files.push(filepath);
        _current_queue_dir_size += sizeof(item);
        return true;
    }
    bool dequeueFromDisk(T* item, std::string filepath)
    {
        // auto& file = fs::directory_iterator(_queue_dir), begin;
        fs::directory_entry file;
        // if (!filename.empty())
        // {
        //     file.assign(fs::path(_queue_dir + filename));
        //     // for (const auto& file_temp : fs::directory_iterator(_queue_dir)) 
        //     // {
        //     //     if(file_temp.path().filename()==filename){ file = file_temp; break; }
        //     // }
        // }
        // else
        // {
        //     std::string filepath = calcFilePath(*item, _queue_size-1);
            file.assign(filepath);
        // }
        // uint32_t serial_size = ros::serialization::serializationLength(item->request);

        uint32_t serial_size = fs::file_size(file.path());
        char* buffer = new char[serial_size];
        // file.seekg(0, std::fstream::beg);

        try
        {
            // std::lock_guard<std::mutex> lock(_file_mutex);
            _fstream.open(file.path().string(), std::fstream::in);
            _fstream.read(buffer,serial_size);
            _fstream.close();
            // fs::remove(file);
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            _fstream.close();
            // std::lock_guard<std::mutex> unlock(_file_mutex);
            return false;   
        }
        // std::lock_guard<std::mutex> unlock(_file_mutex);
        ros::serialization::IStream stream(reinterpret_cast<uint8_t*>(buffer), serial_size);
        ros::serialization::deserialize(stream, item->request);
        // ROS_INFO("Dequeuing %s", file.path().string().c_str());
        // _queue_size--;
        // _queued_files.pop();

#ifdef DEBUG_TEST_IMAGE
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(item->request.test_img, item->request.test_img.encoding);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return 0;
        }
        cv::namedWindow( "Dequeued image", cv::WINDOW_AUTOSIZE );
        cv::imshow( "Dequeued image", cv_ptr->image );
        cv::waitKey(0);
#endif 

        bool success = _client.call(*item);
        if(success)
        {
            setCommStatus(true);
            ROS_INFO("Dequeuing %s. Size %d.", _queued_files.front().c_str(), sizeof(item));
            fs::remove(_queued_files.front());
            _queued_files.pop();
            _current_queue_dir_size -= sizeof(item);
        }
        else
        {
            setCommStatus(false);
            ROS_INFO("Attempt to dequeue %s failed.",_queued_files.front().c_str());
        }

        return success;
    }

    void dequeueThreadFunc()
    {
        // _timer = std::clock();
        // time(&_timer);
        _timer = Time::now();
        while(ros::ok())
        {
            if((_comm_status_is_connected || timerHasExpired()/*difftime(std::clock(),_timer)/CLOCKS_PER_SEC>_timer_limit*/) && _queued_files.size()>0)
            {
                _timer = Time::now();
                // _timer = std::clock();
                // time(&_timer);
                T srv;
                // if (!dequeueFromDisk(&srv,_queued_files.front())){ continue; }
                dequeueFromDisk(&srv,_queued_files.front());
                // call(srv);
                // usleep(100);
            }
            usleep(10e3);
        }
    }

    bool timerHasExpired()
    {   
        // std::time_t now = std::time(NULL);
        // std::time_t diff_time = difftime(now,_timer);
        // bool has_expired = diff_time > _timer_limit;
        // return has_expired;

        Time::time_point now = Time::now();
        // std::chrono::duration<float> diff = now - _timer;
        std::chrono::duration<double> diff = std::chrono::duration_cast<std::chrono::duration<double>>(now - _timer);
        float fdiff = diff.count();
        return fdiff>_timer_limit;
    }

    public:
    ServiceClient() :
        _max_queue_size(-1),
        _max_queue_dir_size(2000000000000),
        _timer_limit(5.0)
    {}
    ServiceClient(ros::NodeHandle* nh, std::string srv_name, uint max_queue_size = -1, ulong max_queue_dir_size = 2000000000000, std::string queue_dir = expand_environment_variables("${HOME}/.queue_to_disk/"), float timer_limit = 5.0) :
        _nh(nh),
        _srv_name(srv_name),
        _max_queue_size(max_queue_size),
        _max_queue_dir_size(max_queue_dir_size),
        _queue_dir(queue_dir),
        _timer_limit(timer_limit)
    {
        if(fs::exists(_queue_dir))
        {
            if(!fs::remove_all(_queue_dir))
            {
                ROS_ERROR("Failed to delete contents of existing directory.");
            }
        }
        if(!fs::create_directories(_queue_dir))
        {
            ROS_ERROR("Failed to create temporary directory.");
        }
        _client = _nh->serviceClient<T>(_srv_name, false);
        _dequeue_thread = new boost::thread(std::bind(&ServiceClient::dequeueThreadFunc,this));
    }
    ~ServiceClient()
    {
        // kill dequeue thread
        if(_dequeue_thread) {
            _dequeue_thread->join();
        }
        delete _dequeue_thread;
        _dequeue_thread = 0;

        // delete temporary directory
        if(fs::exists(_queue_dir))
        {
            if(!fs::remove_all(_queue_dir))
            {
                ROS_ERROR("Failed to delete contents of temporary directory.");
            }
        }
    }
    bool call(T item)
    {
        bool success = _client.call(item);
        if(success) // check if service call succeeded 
        {
            setCommStatus(true);
            return true;
        }
        else // service call failed, need to queue
        {
            setCommStatus(false);
            queueToDisk(item);
            return false;
        }
    }
    bool hasComms(){ return _comm_status_is_connected; }
    static std::string expand_environment_variables( const std::string &s ) {
        if( s.find( "${" ) == std::string::npos ) return s;

        std::string pre  = s.substr( 0, s.find( "${" ) );
        std::string post = s.substr( s.find( "${" ) + 2 );

        if( post.find( '}' ) == std::string::npos ) return s;

        std::string variable = post.substr( 0, post.find( '}' ) );
        std::string value    = "";

        post = post.substr( post.find( '}' ) + 1 );

        const auto *v = getenv( variable.c_str() );
        if( v != NULL ) value = std::string( v );

        return expand_environment_variables( pre + value + post );
    }

};

} // namespace ros_queue_to_disk

#endif // ROS_QUEUE_TO_DISK_H