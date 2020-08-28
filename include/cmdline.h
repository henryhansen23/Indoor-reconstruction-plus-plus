#pragma once

#include <string>

class CmdLine
{
public:
    CmdLine( int argc, char** argv );

    const std::string& getDataDir() const { return _data_dir; }
    const std::string& getICPType() const { return _icp; }
    bool        getVisualize() const { return _visualize; }

private:
    std::string _data_dir;
    bool        _visualize;
    std::string _icp;
};

