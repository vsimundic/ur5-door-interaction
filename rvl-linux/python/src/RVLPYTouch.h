#pragma once
// #include "RVLCore2.h"
#include "Touch.h"
#include "Visualizer.h"
#include <pybind11/numpy.h>
#include <string>
#include <vector>

namespace RVL
{
    class PYTouch
    {
    public:
        PYTouch();
        ~PYTouch();

        void create(std::string cfgFileName);
        void clear();
        void create_scene(float sx, float sy, float sz, float rx, float ry, float a, float b, float c, float qDeg);
        void create_simple_tool(float a, float b, float c, float d, float h, float *t = nullptr);
        void set_capturing_flange_pose(pybind11::array T_E_0);
        void correct_real_experiment(pybind11::array T_Ek_E, pybind11::array V, pybind11::array T_A_E, pybind11::array T_E_0);

        RVL::Touch touch;
        int memSize;
        int mem0Size;
        RVL::Visualizer visualizer;
        RVL::Array<RVL::MOTION::TouchData> touches;
        std::vector<RVL::MOTION::Contact> contacts;
        Pose3D pose_E_0; // pose of flange when capturing
    };
} // namespace RVL