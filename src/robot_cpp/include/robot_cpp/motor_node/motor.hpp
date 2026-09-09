#pragma once

#include <string>

class Motor
{
public:
    Motor(const std::string& name);
    void forward();
    void backward();
    void stop();

    void setName(const std::string& name) { name_ = name; }
    const std::string getName() const { return name_; }

private:
    std::string name_;
};