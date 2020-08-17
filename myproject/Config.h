//
// Created by cacacadaxia on 2020/8/13.
//

#ifndef VO1_CONFIG_H
#define VO1_CONFIG_H

#include "Common.h"

class Config{
private:
    cv::FileStorage file_;
    static std::shared_ptr<Config> config_;

public:
    static void setParament(const std::string & filename){
        if (config_ == nullptr) {
            config_ = shared_ptr<Config>(new Config);
        }
        config_->file_ = cv::FileStorage(filename, cv::FileStorage::READ);
    }
    template<typename T>
    static T get(const string &name){
        return T(config_->file_[name]);
    }
};

std::shared_ptr<Config> Config::config_ = nullptr;

#endif //VO1_CONFIG_H
