// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "../include/framos_realsense_node_factory.h"
#include "../include/base_realsense_node.h"
#include "../include/t265_realsense_node.h"
#include <iostream>
#include <map>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <thread>
#include <sys/time.h>
#include <regex>
#include "yaml-cpp/yaml.h"
#include <ros/package.h>

using namespace realsense2_framos_camera;

#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;

PLUGINLIB_EXPORT_CLASS(realsense2_framos_camera::FramosRealSenseNodeFactory, nodelet::Nodelet)

FramosRealSenseNodeFactory::FramosRealSenseNodeFactory():
    _is_alive(true)
{
    ROS_INFO("RealSense ROS for FRAMOS v%s", REALSENSE_ROS_VERSION_STR);
    ROS_INFO("Running with LibRealSense for FRAMOS v%s", RS2_API_VERSION_STR);

    auto severity = rs2_log_severity::RS2_LOG_SEVERITY_WARN;
    tryGetLogSeverity(severity);
    if (rs2_log_severity::RS2_LOG_SEVERITY_DEBUG == severity)
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    rs2::log_to_console(severity);
}

FramosRealSenseNodeFactory::~FramosRealSenseNodeFactory()
{
    _is_alive = false;
    if (_query_thread.joinable())
    {
        _query_thread.join();
    }

    for (int i = 0; i < _device_list.size(); i++)
    {
        if (_device_list[i]->device)
        {
            _device_list[i]->realSenseNode.reset();
        }
    }
}

std::string FramosRealSenseNodeFactory::parse_usb_port(std::string line)
{
    std::string port_id;
    std::regex self_regex("(?:[^ ]+/usb[0-9]+[0-9./-]*/){0,1}([0-9.-]+)(:){0,1}[^ ]*", std::regex_constants::ECMAScript);
    std::smatch base_match;
    bool found = std::regex_match(line, base_match, self_regex);
    if (found)
    {
        port_id = base_match[1].str();
        if (base_match[2].str().size() == 0)    //This is libuvc string. Remove counter is exists.
        {
            std::regex end_regex = std::regex(".+(-[0-9]+$)", std::regex_constants::ECMAScript);
            bool found_end = std::regex_match(port_id, base_match, end_regex);
            if (found_end)
            {
                port_id = port_id.substr(0, port_id.size() - base_match[1].str().size());
            }
        }
    }
    return port_id;
}

void FramosRealSenseNodeFactory::loadParameters()
{
    std::string multicam_config_file;
    std::string path = ros::package::getPath("realsense2_framos_camera");
    ros::NodeHandle privateNh = getPrivateNodeHandle();

    privateNh.param("multicam_config_file", multicam_config_file, std::string(path+"/config/multicam_config_file.yaml"));
    YAML::Node config = YAML::LoadFile(multicam_config_file);

    for(YAML::iterator it=config.begin(); it!=config.end(); ++it)
    {
        YAML::Node child = it->second;

        auto cd = new camera_device();

        cd->serial = child["serial_no"].as<std::string>();
        cd->ip_address = child["ip_address"].as<std::string>();
        cd->usb_port_id = child["usb_port_id"].as<std::string>();
        cd->device_type = child["device_type"].as<std::string>();
        cd->rosbag_filename = child["rosbag_filename"].as<std::string>();
        cd->camera_name = it->first.as<std::string>();
        cd->is_found = false;
        cd->is_started = false;
        cd->initial_reset = child["initial_reset"].as<bool>();

        _device_list.push_back(cd);
    }
}

void FramosRealSenseNodeFactory::publishStaticTransforms()
{
       std::vector<geometry_msgs::TransformStamped> static_tf_msgs;

       for (int i = 0; i < _device_list.size(); i++)
       {
               if (_device_list[i]->is_started)
               {
                       auto transforms = _device_list[i]->realSenseNode->getStaticTransforms();
                       static_tf_msgs.insert(std::end(static_tf_msgs), std::begin(transforms), std::end(transforms));
               }
       }

       _static_tf_broadcaster.sendTransform(static_tf_msgs);
}

void FramosRealSenseNodeFactory::getDeviceFromBag()
{
    for (int i = 0; i < _device_list.size(); i++)
    {
        if (!_device_list[i]->rosbag_filename.empty() && !_device_list[i]->is_found)
        {
            std::string private_nh_namespace, nh_namespace;

            ros::NodeHandle temp_nh = getNodeHandle();
            ros::NodeHandle temp_privateNh = getPrivateNodeHandle();

            private_nh_namespace = temp_privateNh.getNamespace() + "/" + _device_list[i]->camera_name;
            nh_namespace = temp_nh.getNamespace() + "/" + _device_list[i]->camera_name;

            ros::NodeHandle pnh(private_nh_namespace.c_str());
            ros::NodeHandle nh(nh_namespace.c_str());

            ROS_INFO_STREAM("publish topics from rosbag file: " << _device_list[i]->rosbag_filename.c_str());
            auto pipe = std::make_shared<rs2::pipeline>();
            rs2::config cfg;
            cfg.enable_device_from_file(_device_list[i]->rosbag_filename.c_str(), false);
            cfg.enable_all_streams();
            pipe->start(cfg); //File will be opened in read mode at this point
            _device_list[i]->device = pipe->get_active_profile().get_device();
            _device_list[i]->serial = _device_list[i]->device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            _device_list[i]->is_found = true;
        }
    }
}

void FramosRealSenseNodeFactory::getDevice(rs2::device_list list)
{
    bool found = false;

    if (0 == list.size())
    {
        ROS_WARN("No RealSense devices were found!");
    }
    else
    {
        ROS_INFO_STREAM(" ");

        for (auto&& dev : list)
        {   
            std::string ip;

            bool found_ip = false;
            bool already_on_list = false;

            auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            found_ip = dev.supports(RS2_CAMERA_INFO_IP_ADDRESS);
            if (found_ip) ip = dev.get_info(RS2_CAMERA_INFO_IP_ADDRESS);

            for (int i = 0; i < _device_list.size(); i++)
            {
                if ((_device_list[i]->serial.compare(sn) == 0 || (_device_list[i]->ip_address.compare(ip) == 0 && found_ip)) && _device_list[i]->is_found) 
                {
                    already_on_list = true;
                    break;
                }
            }

            if (!already_on_list)   
            {

                ROS_INFO_STREAM("Device with serial number " << sn << " was found."<<std::endl);

                if (found_ip) ROS_INFO_STREAM("Device with ip address " << ip << " was found."<<std::endl);
                std::string pn = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
                std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
                ROS_INFO_STREAM("Device with physical ID " << pn << " was found.");
                std::vector<std::string> results;
                ROS_INFO_STREAM("Device with name " << name << " was found.");
                std::string port_id = parse_usb_port(pn);   

                for (int i = 0; i < _device_list.size(); i++)
                {
                    if (!_device_list[i]->is_found)
                    {
                        if (port_id.empty() && !found_ip)
                        {
                            std::stringstream msg;
                            msg << "Error extracting usb port from device with physical ID: " << pn << std::endl << "Please report on github issue at https://github.com/IntelRealSense/realsense-ros";
                            if (_device_list[i]->usb_port_id.empty())
                            {
                                ROS_WARN_STREAM(msg.str());
                            }
                            else
                            {
                                ROS_ERROR_STREAM(msg.str());
                                ROS_ERROR_STREAM("Please use serial number instead of usb port.");
                            }
                        }
                        else if (!port_id.empty() && !found_ip)
                        {
                            ROS_INFO_STREAM("Device with port number " << port_id << " was found.");                    
                        }

                        bool found_device_type(true);
                        if (!_device_list[i]->device_type.empty())
                        {
                            std::smatch match_results;
                            std::regex device_type_regex(_device_list[i]->device_type.c_str(), std::regex::icase);
                            found_device_type = std::regex_search(name, match_results, device_type_regex);
                        }

                        if ((_device_list[i]->ip_address.empty() || (_device_list[i]->ip_address.compare(ip) == 0 && found_ip)) && (_device_list[i]->serial.empty() || _device_list[i]->serial.compare(sn) == 0) && (_device_list[i]->usb_port_id.empty() || port_id == _device_list[i]->usb_port_id) && found_device_type)
                        {
                            _device_list[i]->device = dev;
                            _device_list[i]->serial = sn;
                            _device_list[i]->ip_address = ip;
                            _device_list[i]->is_found = true;

                            bool remove_tm2_handle(_device_list[i]->device && RS_T265_PID != std::stoi(_device_list[i]->device.get_info(RS2_CAMERA_INFO_PRODUCT_ID), 0, 16));
                            if (remove_tm2_handle)
                            {
                                _ctx.unload_tracking_module();
                            }

                            if (_device_list[i]->initial_reset)
                            {
                                _device_list[i]->initial_reset = false;
                                try
                                {
                                    ROS_INFO("Resetting device...");
                                    _device_list[i]->device.hardware_reset();
                                    _device_list[i]->device = rs2::device();
                                    
                                }
                                catch(const std::exception& ex)
                                {
                                    ROS_WARN_STREAM("An exception has been thrown: " << ex.what());
                                }
                            }

                            break;
                        }
                    }
                }
            }
        }

        for (int i = 0; i < _device_list.size(); i++)
        {
            if (!_device_list[i]->is_found) 
            {
                // T265 could be caught by another node.
                std::string msg ("The requested device with ");
                bool add_and(false);
                if (!_device_list[i]->serial.empty())
                {
                    msg += "serial number " + _device_list[i]->serial;
                    add_and = true;
                }
                if (!_device_list[i]->ip_address.empty())
                {
                    msg += "ip address " + _device_list[i]->ip_address;
                    add_and = true;
                }
                if (!_device_list[i]->usb_port_id.empty())
                {
                    if (add_and)
                    {
                        msg += " and ";
                    }
                    msg += "usb port id " + _device_list[i]->usb_port_id;
                    add_and = true;
                }
                if (!_device_list[i]->device_type.empty())
                {
                    if (add_and)
                    {
                        msg += " and ";
                    }
                    msg += "device name containing " + _device_list[i]->device_type;
                }
                msg += " is NOT found. Will Try again.";
                ROS_WARN_STREAM(msg);
            }
        }
    }       
}

void FramosRealSenseNodeFactory::change_device_callback(rs2::event_information& info)
{
    for (int i = 0; i < _device_list.size(); i++ )
    {
        if (info.was_removed(_device_list[i]->device))
        {
            ROS_WARN("%s The device has been disconnected!", _device_list[i]->camera_name.c_str());
            _device_list[i]->realSenseNode.reset(nullptr);
            _device_list[i]->device = rs2::device();
            _device_list[i]->is_started = false;
            _device_list[i]->is_found = false;
        }
    }

    rs2::device_list new_devices = info.get_new_devices();
    if (new_devices.size() > 0)
    {
        ROS_INFO("Checking new devices...");
        getDevice(new_devices);

        for (int i = 0; i < _device_list.size(); i++)
        {
            if (_device_list[i]->is_found && !_device_list[i]->is_started)
            {
                _device_list[i]->is_started = true;
                StartDevice(_device_list[i]);
            }
        }
    }
}

void FramosRealSenseNodeFactory::onInit()
{
    try
    {
#ifdef BPDEBUG
        std::cout << "Attach to Process: " << getpid() << std::endl;
        std::cout << "Press <ENTER> key to continue." << std::endl;
        std::cin.get();
#endif
        loadParameters();
        /////////////////////////////////////////////////////
    
        std::function<void(rs2::event_information&)> change_device_callback_function = [this](rs2::event_information& info){change_device_callback(info);};
        _ctx.set_devices_changed_callback(change_device_callback_function);

        _query_thread = std::thread([=]()
                    {
                        std::chrono::milliseconds timespan(6000);
                        volatile bool devices_found = false;

                        while (_is_alive && !devices_found)
                        {
                            bool devices_flag = true;
                            getDevice(_ctx.query_devices());
                            getDeviceFromBag();

                            for (int i = 0; i < _device_list.size(); i++)
                            {
                                devices_flag &= _device_list[i]->is_found;

                                if (_device_list[i]->is_found && !_device_list[i]->is_started)
                                {
                                    _device_list[i]->is_started = true;
                                    StartDevice(_device_list[i]);
                                    publishStaticTransforms();
                                }
                            }

                            devices_found = devices_flag;

                            if (!devices_found) std::this_thread::sleep_for(timespan);
                        }
                    });
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
        exit(1);
    }
    catch(...)
    {
        ROS_ERROR_STREAM("Unknown exception has occured!");
        exit(1);
    }
}

void FramosRealSenseNodeFactory::StartDevice(camera_device *device)
{
    if (device->realSenseNode) device->realSenseNode.reset();

    std::string private_nh_namespace, nh_namespace;
    ros::NodeHandle temp_nh = getNodeHandle();
    ros::NodeHandle temp_privateNh = getPrivateNodeHandle();

    private_nh_namespace = temp_privateNh.getNamespace() + "/" + device->camera_name;
    nh_namespace = temp_nh.getNamespace() + "/" + device->camera_name;
    ros::NodeHandle pnh(private_nh_namespace.c_str());
    ros::NodeHandle nh(nh_namespace.c_str());
    
    std::string pid_str(device->device.get_info(RS2_CAMERA_INFO_PRODUCT_ID));
    uint16_t pid = std::stoi(pid_str, 0, 16);
    switch(pid)
    {
    case SR300_PID:
    case SR300v2_PID:
    case RS400_PID:
    case RS405_PID:
    case RS410_PID:
    case RS460_PID:
    case RS415_PID:
    case RS420_PID:
    case RS420_MM_PID:
    case RS430_PID:
    case RS430_MM_PID:
    case RS430_MM_RGB_PID:
    case RS435_RGB_PID:
    case RS435i_RGB_PID:
    case RS_USB2_PID:
    case RS_L515_PID:
        device->realSenseNode = std::unique_ptr<BaseRealSenseNode>(new BaseRealSenseNode(nh, pnh, device->device, device->serial));
        break;
    case RS_T265_PID:
        device->realSenseNode = std::unique_ptr<T265RealsenseNode>(new T265RealsenseNode(nh, pnh, device->device, device->serial));
        break;
    default:
        ROS_FATAL_STREAM("Unsupported device!" << " Product ID: 0x" << pid_str);
        ros::shutdown();
        exit(1);
    }
    assert(device->realSenseNode);
    device->realSenseNode->publishTopics();
}

void FramosRealSenseNodeFactory::tryGetLogSeverity(rs2_log_severity& severity) const
{
    static const char* severity_var_name = "LRS_LOG_LEVEL";
    auto content = getenv(severity_var_name);

    if (content)
    {
        std::string content_str(content);
        std::transform(content_str.begin(), content_str.end(), content_str.begin(), ::toupper);

        for (uint32_t i = 0; i < RS2_LOG_SEVERITY_COUNT; i++)
        {
            auto current = std::string(rs2_log_severity_to_string((rs2_log_severity)i));
            std::transform(current.begin(), current.end(), current.begin(), ::toupper);
            if (content_str == current)
            {
                severity = (rs2_log_severity)i;
                break;
            }
        }
    }
}
