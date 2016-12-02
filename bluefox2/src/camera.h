#pragma once

#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <cassert>

#include <apps/Common/exampleHelper.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

class BluefoxCamera
{
  public:
    BluefoxCamera(mvIMPACT::acquire::Device *_device);

    bool ready();

    void loop();

    std::vector<unsigned char> getImg();
    ~BluefoxCamera();
    int imageHeight, imageWidth;

    mvIMPACT::acquire::Device *device;
    mvIMPACT::acquire::FunctionInterface fi;
    mvIMPACT::acquire::SettingsBlueFOX s;

    std::thread t;

    std::mutex barrier;
    std::queue<std::vector<unsigned char>> data_q;
};

class BluefoxManager
{
  public:
    BluefoxManager();
    int getImgCnt();
    bool ready();
    std::vector<unsigned char> getImg();

  private:
    mvIMPACT::acquire::DeviceManager devMgr;
    std::vector<BluefoxCamera *> bluefoxCameras;
};
