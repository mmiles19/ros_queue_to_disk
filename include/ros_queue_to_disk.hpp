#ifndef ROS_QUEUE_TO_DISK_H
#define ROS_QUEUE_TO_DISK_H

#include <ros/ros.h>
#include <fstream>
#include <filesystem>
#include <boost/thread.hpp>
#include <cerrno>
// #include <mutex>

namespace fs = std::filesystem;

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
    uint _queue_size;
    uint _max_queue_size;
    bool _comm_status_is_connected;
    uint _comm_status_count;
    // std::mutex _file_mutex;

    void setCommStatus(bool status)
    {
        if(_comm_status_is_connected != status){ _comm_status_count = 0; }
        _comm_status_count++;
        _comm_status_is_connected = status;
    }
    std::string calcFileName(T item, uint8_t idx)
    {
        char format_char[std::to_string(_max_queue_size).length()+1];
        sprintf(format_char,"%0*u",std::to_string(_max_queue_size).length(),idx);
        std::string filename = std::string(ros::service_traits::md5sum(item)) + "_" + std::string(format_char);
        return filename;
    }
    std::string calcFilePath(T item, uint8_t idx)
    {
        std::string filepath = _queue_dir + calcFileName(item, idx);
        return filepath;
    }
    bool queueToDisk(T item)
    {
        if(_queue_size >= _max_queue_size){ return false; }
       
        std::string filepath = calcFilePath(item, _queue_size);
        uint32_t serial_size = ros::serialization::serializationLength(item.request);
        char* buffer = new char[serial_size];
        ros::serialization::OStream stream(reinterpret_cast<uint8_t*>(buffer), serial_size);
        ros::serialization::serialize(stream, item.request);
        ROS_INFO("Queuing %s", filepath.c_str());
        try
        {
            // std::lock_guard<std::mutex> lock(_file_mutex);
            // fs::permissions(filepath,
            //     fs::perms::owner_all | fs::perms::group_all | fs::perms::others_all);
            // demo_perms(fs::status(filepath).permissions());
            _fstream.open(filepath, std::fstream::out | std::fstream::binary);
            
            if(!_fstream)
            {
                ROS_ERROR("Failed to open file: %s", std::strerror(errno));
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
        _queue_size++;
        return true;
    }
    bool dequeueFromDisk(T* item, std::string filename = "")
    {
        // auto& file = fs::directory_iterator(_queue_dir), begin;
        fs::directory_entry file;
        if (!filename.empty())
        {
            file.assign(fs::path(_queue_dir + filename));
            // for (const auto& file_temp : fs::directory_iterator(_queue_dir)) 
            // {
            //     if(file_temp.path().filename()==filename){ file = file_temp; break; }
            // }
        }
        else
        {
            std::string filepath = calcFilePath(*item, _queue_size-1);
            file.assign(filepath);
        }
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
            fs::remove(file);
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
        ROS_INFO("Dequeuing %s", file.path().string().c_str());
        _queue_size--;
        return true;
    }

    public:
    ServiceClient(ros::NodeHandle* nh, std::string srv_name, uint max_queue_size = -1, std::string queue_dir = "/home/mike/.queue_to_disk/") :
        _nh(nh),
        _srv_name(srv_name),
        _queue_dir(queue_dir),
        _max_queue_size(max_queue_size),
        _dequeue_thread(0),
        _queue_size(0)
    {
        if(fs::exists(_queue_dir))
        {
            if(!fs::remove_all(_queue_dir))
            {
                ROS_ERROR("Failed to delete contents of existing directory");
            }
        }
        if(!fs::create_directories(_queue_dir))
        {
            ROS_ERROR("Failed to create directory");
        }
        _client = _nh->serviceClient<T>(_srv_name, false);
        _dequeue_thread = new boost::thread(std::bind(&ServiceClient::dequeueThreadFunc,this));
    }
    ~ServiceClient()
    {
        if(fs::exists(_queue_dir))
        {
            if(!fs::remove_all(_queue_dir))
            {
                ROS_ERROR("Failed to delete contents of existing directory");
            }
        }

        if(_dequeue_thread) {
            _dequeue_thread->join();
        }
        delete _dequeue_thread;
        _dequeue_thread = 0;
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
    void dequeueThreadFunc()
    {
        while(true)
        {
            while(_comm_status_is_connected && _queue_size>0)
            {
                T srv;
                if (!dequeueFromDisk(&srv)){ continue; }
                call(srv);
                usleep(100);
            }
            usleep(10e3);
        }
    }

};

} // namespace ros_queue_to_disk

#endif // ROS_QUEUE_TO_DISK_H