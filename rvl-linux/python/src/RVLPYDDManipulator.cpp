#include "RVLCore2.h"
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLVTK.h"
#include "Util.h"
#include "SE3Grid.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "RVLRecognitionCommon.h"
#include "PSGMCommon.h"
#include "CTISet.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#include "PSGM.h"
#include "VN.h"
#include "RRT.h"
#include "DDManipulator.h"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

using namespace RVL;
using namespace py::literals;

////////////////////////////////////////////////////////////////////
//
//     PYDDManipulator
//
////////////////////////////////////////////////////////////////////

class PYDDManipulator
{
public:
	PYDDManipulator();
	~PYDDManipulator();
	void create(
		std::string cfgFileName);
	void set_memory_storage_size(
		int mem0Size,
		int memSize);
	void clear();
	py::array path2(py::array T_G_S_init);
	py::array approach_path(py::array T_G_S_contact);
	void load_tool_model(std::string toolModelDir);
	void set_environment_state(double state);
	void load_feasible_tool_contact_poses(std::string contactPosesFileName);
	void set_furniture_pose(py::array T_F_S);
	void set_robot_pose(py::array T_0_S);
	void set_door_model_params(
		py::array T_A_S,
		float sx,
		float sy,
		float sz,
		float rx,
		float ry,
		float opening_direction,
		float static_side_width,
		float moving_to_static_part_distance);
	py::array get_T_DD_S();
	py::array get_T_F_S();

public:
	DDManipulator manipulator;
	int memSize;
	int mem0Size;
	Visualizer visualizer;
	MOTION::DisplayCallbackData* pVisualizationData;
};

PYDDManipulator::PYDDManipulator()
{
	memSize = 1000000000;
	mem0Size = 1000000000;
}

PYDDManipulator::~PYDDManipulator()
{
	clear();
}

void PYDDManipulator::create(
	std::string cfgFileName)
{
	manipulator.pMem0 = new CRVLMem;
	manipulator.pMem0->Create(mem0Size);
	manipulator.pMem = new CRVLMem;
	manipulator.pMem->Create(memSize);
	char *cfgFileName_ = (char *)(cfgFileName.data());
	manipulator.Create(cfgFileName_);
	visualizer.Create();
	manipulator.InitVisualizer(&visualizer);
}

void PYDDManipulator::set_memory_storage_size(
		int mem0SizeIn,
		int memSizeIn)
{
	memSize = memSizeIn;
	mem0Size = mem0SizeIn;
}

void PYDDManipulator::clear()
{
	delete manipulator.pMem0;
	delete manipulator.pMem;
}

py::array PYDDManipulator::path2(py::array T_G_S_init)
{
	double *T_G_S_init_ = (double *)T_G_S_init.request().ptr;	
	Pose3D pose_G_S_init;
	RVLHTRANSFMXDECOMP(T_G_S_init_, pose_G_S_init.R, pose_G_S_init.t);
	FILE *fpDebug = fopen("pose_G_S_init-2.txt", "w");
    float T_G_S_init__[16];
    RVLHTRANSFMX(pose_G_S_init.R, pose_G_S_init.t, T_G_S_init__);
    PrintMatrix(fpDebug, T_G_S_init__, 4, 4);
    fclose(fpDebug);
	Array<Pose3D> poses_G_0;
	manipulator.Path2(&pose_G_S_init, poses_G_0);
	auto T_G_0 = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		3,
		{poses_G_0.n, 4, 4},
		{4 * 4 * sizeof(float), 4 * sizeof(float), sizeof(float)}
		));
	float *T_G_0_ = (float *)T_G_0.request().ptr;
	int iPose;
	Pose3D *pPose_G_0 = poses_G_0.Element;
	for(iPose = 0; iPose < poses_G_0.n; iPose++, T_G_0_ += 16, pPose_G_0++)
		RVLHTRANSFMX(pPose_G_0->R, pPose_G_0->t, T_G_0_);
	delete[] poses_G_0.Element;

	return T_G_0;
}

py::array PYDDManipulator::approach_path(py::array T_G_S_contact)
{
	double *T_G_S_contact_ = (double *)T_G_S_contact.request().ptr;
	Pose3D pose_G_S_contact;
	RVLHTRANSFMXDECOMP(T_G_S_contact_, pose_G_S_contact.R, pose_G_S_contact.t);
	Array<Pose3D> poses_G_0_via;
	Pose3D viaPtPosesMem[2];
    poses_G_0_via.Element = viaPtPosesMem;	
	manipulator.ApproachPath(&pose_G_S_contact, poses_G_0_via);
	auto T_G_0_via = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		3,
		{poses_G_0_via.n, 4, 4},
		{4 * 4 * sizeof(float), 4 * sizeof(float), sizeof(float)}
		));
	float *T_G_0_via_ = (float *)T_G_0_via.request().ptr;
	int iPose;
	Pose3D *pPose_G_0 = poses_G_0_via.Element;
	for(iPose = 0; iPose < poses_G_0_via.n; iPose++, T_G_0_via_ += 16, pPose_G_0++)
		RVLHTRANSFMX(pPose_G_0->R, pPose_G_0->t, T_G_0_via_);

	return T_G_0_via;
}

void PYDDManipulator::load_tool_model(std::string toolModelDir)
{
	manipulator.LoadToolModel(toolModelDir);
}

void PYDDManipulator::set_environment_state(double state)
{
	manipulator.SetEnvironmentState(state);
	//printf("Environment state: %f\n", manipulator.dd_state_angle);
}

void PYDDManipulator::load_feasible_tool_contact_poses(std::string contactPosesFileName)
{
	manipulator.LoadFeasibleToolContactPoses(contactPosesFileName);
}

void PYDDManipulator::set_furniture_pose(py::array T_F_S)
{
	double *T_F_S_ = (double *)T_F_S.request().ptr;
	RVLHTRANSFMXDECOMP(T_F_S_, manipulator.pose_F_S.R, manipulator.pose_F_S.t);
	// printf("manipulator.pose_F_S:\n");
	// for(int i = 0; i < 3; i++)
	// {
	// 	for(int j = 0; j < 3; j++)
	// 		printf("%f ", manipulator.pose_F_S.R[j+3*i]);
	// 	printf("%f\n", manipulator.pose_F_S.t[i]);
	// }
}

void PYDDManipulator::set_robot_pose(py::array T_0_S)
{
	double *T_0_S_ = (double *)T_0_S.request().ptr;
	RVLHTRANSFMXDECOMP(T_0_S_, manipulator.robot.pose_0_W.R, manipulator.robot.pose_0_W.t);	
}

void PYDDManipulator::set_door_model_params(
	py::array T_A_S,
	float sx,
	float sy,
	float sz,
	float rx,
	float ry,
	float opening_direction,
	float static_side_width,
	float moving_to_static_part_distance)
{
	double *T_A_S_ = (double *)T_A_S.request().ptr;
	Pose3D pose_A_S;
	RVLHTRANSFMXDECOMP(T_A_S_, pose_A_S.R, pose_A_S.t);
	manipulator.SetDoorModelParams(pose_A_S, sx, sy, sz, rx, ry, opening_direction, static_side_width, moving_to_static_part_distance);
}

py::array PYDDManipulator::get_T_DD_S()
{
	auto T_DD_S = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{4, 4},
		{4 * sizeof(float), sizeof(float)}
		));
	float *T_DD_S_ = (float *)T_DD_S.request().ptr;
	RVLHTRANSFMX(manipulator.pose_DD_S.R, manipulator.pose_DD_S.t, T_DD_S_);

	return T_DD_S;
}

py::array PYDDManipulator::get_T_F_S()
{
	auto T_F_S = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{4, 4},
		{4 * sizeof(float), sizeof(float)}
		));
	float *T_F_S_ = (float *)T_F_S.request().ptr;
	RVLHTRANSFMX(manipulator.pose_F_S.R, manipulator.pose_F_S.t, T_F_S_);

	return T_F_S;
}

////////////////////////////////////////////////////////////////////
//
//     RVL PYDDManipulator Wrapper
//
////////////////////////////////////////////////////////////////////

PYBIND11_MODULE(RVLPYDDManipulator, m) 
{
	m.doc() = "RVL PYDDManipulator wrapper";
	
	py::class_<PYDDManipulator>(m, "PYDDManipulator")
		.def(py::init<>())
		.def("create", &PYDDManipulator::create)
		.def("set_memory_storage_size", &PYDDManipulator::set_memory_storage_size)
		.def("clear", &PYDDManipulator::clear)
		.def("path2", &PYDDManipulator::path2)
		.def("approach_path", &PYDDManipulator::approach_path)
		.def("load_tool_model", &PYDDManipulator::load_tool_model)
		.def("set_environment_state", &PYDDManipulator::set_environment_state)
		.def("load_feasible_tool_contact_poses", &PYDDManipulator::load_feasible_tool_contact_poses)
		.def("set_furniture_pose", &PYDDManipulator::set_furniture_pose)
		.def("set_robot_pose", &PYDDManipulator::set_robot_pose)
		.def("set_door_model_params", &PYDDManipulator::set_door_model_params)
		.def("get_T_DD_S", &PYDDManipulator::get_T_DD_S)
		.def("get_T_F_S", &PYDDManipulator::get_T_F_S);
}
