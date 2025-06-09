#pragma once

#define RVLMOTION_TOUCH_NUM_PARAMS 15
#define RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE 0
#define RVLMOTION_TOUCH_CONTACT_TYPE_EDGE_EDGE 1
#define RVLMOTION_TOUCH_CONTACT_TYPE_PLANE_POINT 2
#define RVLMOTION_TOUCH_NUM_REF_PTS 6

namespace RVL
{
	struct SolidVertex
	{
		float P[3];
		bool bConvex;
		int iEdge;
	};

	struct SolidEdge
	{
		int iVertex;
		int iFace;
		int iNext;
		int iPrev;
		int iTwin;
		float len;
	};

	struct SolidFace
	{
		float N[3];
		float d;
		int iEdge;
		float area;
	};

	class Solid
	{
	public:
		Solid();
		virtual ~Solid();
		void Clear();
		void Add(Solid *pSolid);
		void Union();
		void Create(Array<Array<int>> faces_);
		void Move(
			Solid *pSolidSrc,
			Pose3D *pMove);
		void Copy(Solid *pSolidSrc);
		void ComputeFaceParams();
		Solid *CreateBox(
			float *size,
			Pose3D *pPose);
		bool Intersect(
			float *P1,
			float *P2,
			float *s,
			float *V,
			float &l);
		bool Intersect(
			Solid *pSolid,
			Array<Pair<Vector3<float>, Vector3<float>>> *pIntersectionsWithOtherEdges,
			Array<Pair<Vector3<float>, Vector3<float>>> *pIntersectionsWithThisEdges = NULL);
		bool IntersectWithConvex(
			Solid *pSolid,
			Array<Pair<Vector3<float>, Vector3<float>>> *pIntersectionsWithOtherEdges,
			Array<Pair<Vector3<float>, Vector3<float>>> *pIntersectionsWithThisEdges = NULL);
		float FreeMove(
			float *V,
			Solid *pObstacle);
		std::vector<vtkSmartPointer<vtkActor>> Visualize(
			Visualizer *pVisualizer,
			uchar *color);

	public:
		Array<SolidVertex> vertices;
		Array<SolidEdge> edges;
		Array<SolidFace> faces;
		std::vector<Solid *> solids;
		Box<float> bbox;
		Visualizer *pVisualizer;
	};

	namespace MOTION
	{
		struct TouchModel
		{
			Pose3D pose_C_E;
			Camera camera;
			float K[9];
			float kappa;
			float kappazn;
			float TCP_E[3];
			VN *pVNEnv;
			float *d;
			MOTION::Plane *plane;
			Vector3<float> *vertex;
			Pose3D pose_A_E;
			float PAxis_E[3];
		};

		struct TouchSample
		{
			float P[3];
			int iAxis;
			float direction;
		};

		struct Contact
		{
			int iTouch;
			int iToolFeature;
			Pair<int, int> iEnvFeature;
			uchar type;
		};

		struct TouchData
		{
			Pose3D pose;
			float V[3];
			MOTION::Contact contact;
			int iFirstContact;
			int nContacts;
		};

		struct PlanarSurface
		{
			MOTION::Plane plane;
			Array<int> VNFeatures;
			Array<Pair<int, int>> solidFaces;
		};

		struct Vertex
		{
			float P[3];
			Array<Pair<int, int>> solidVertices;
		};

		struct Tool
		{
			// Array<Vector3<float>> vertices;
			// Array2D<float> vertices;
			// Array2D<int> faces;
			Solid solid;
			float TCP[3];
			VN *pVN;
			Array<RECOG::VN_::ModelCluster *> VNMClusters;
			float *d;
		};

		struct TouchPoint
		{
			int iPanel;
			int iFace;
			int iEdge;
			float u;
			float v;
		};

		struct TouchLMError
		{
			float ravg;
			float gmax;
			float Ex;
			float E;
		};
	}

	class Touch
	{
	public:
		Touch();
		virtual ~Touch();
		void Create(char *cfgFileName);
		void CreateParamList();
		void Clear();
		void LM(
			float *x0,
			Array<MOTION::TouchData> touches,
			float *x,
			MOTION::TouchLMError *pErr = NULL);
		float SDF(
			MOTION::TouchData *pTouchData,
			VN *pVNEnv,
			float *d);
		Pair<float, float> Error(
			MOTION::TouchData *pTouchData,
			bool bToolPositioned = false,
			bool bPointTool = false);
		void Sample(
			int nSamples,
			float rayDist,
			Array<MOTION::TouchSample> &samples);
		void CreateEnvironmentModelTemplete(
			MOTION::TouchModel *pModel,
			int nPanels,
			Array<RECOG::PSGM_::Plane> CT,
			Pair<float, float> betaInterval,
			Pose3D *pPose_Arot_A = NULL);
		void DeleteTouchModel(MOTION::TouchModel *pModel);
		void CreateScene(
			float sx,
			float sy,
			float sz,
			float rx,
			float ry,
			float a,
			float b,
			float c,
			float qDeg);
		void CreateSimpleTool(
			float a,
			float b,
			float c,
			float d,
			float h,
			float *t = NULL);
		void Simulation();
		void SimulateVisionWithError(float *x);
		void Correct(
			MOTION::TouchModel *pModelSrc,
			float *x,
			MOTION::TouchModel *pModelTgt);
		void CorrectVertex(
			float *P0,
			float *a_,
			float b_,
			float *D,
			Pose3D *pPose_C_E_0,
			Pose3D *pPose_C_E_x,
			float *Px);
		void CorrectPlane(
			MOTION::Plane *pPlaneSrc,
			float *a,
			float b,
			float *M,
			float *V,
			MOTION::Plane *pPlaneTgt);
		void Contacts(
			MOTION::TouchData *pTouch_E,
			std::vector<MOTION::Contact> &contacts,
			bool bVisualization = false);
		bool Correction(
			float *xInit,
			Array<MOTION::TouchData> touches_E,
			std::vector<MOTION::Contact> &contacts,
			float *x);
		void Reconstruct(
			float *P_E,
			MOTION::TouchModel *pModel_x,
			float *P_E_x,
			float *P_C_x,
			float *invKxIn = NULL);
		void SetVerticesAndPlanes(
			Array<MOTION::PlanarSurface> surfaces_,
			Array<MOTION::Vertex> vertices_,
			Pose3D pose,
			MOTION::TouchModel *pModel);
		void UpdateVerticesAndPlanes(MOTION::TouchModel *pModel);
		void UpdateDoorAxis(MOTION::TouchModel *pModel0);
		void UpdateDoorOrientation(MOTION::TouchModel *pModel);
		void UpdateEnvironmentModel(
			MOTION::TouchModel *pModel0,
			float *x,
			MOTION::TouchModel *pModelx);
		void UpdateEnvironmentVNModel(
			MOTION::TouchModel *pModel0,
			float *x,
			MOTION::TouchModel *pModelx);
		bool Intersection(
			float *N1,
			float *N2,
			float *N3,
			float *N4);
		void RndTouchPoint(MOTION::TouchPoint &touchPt);
		void RefPts();
		void PlanTouch(
			MOTION::TouchPoint touchPt,
			Pose3D &initPose,
			Array<Vector3<float>> &path,
			float *PTgt);
		void SimulateMove(
			Pose3D initPose,
			Array<Vector3<float>> path,
			Pose3D &finalPose,
			float *V,
			int &iLastSegment);
		void SceneBBox(
			MOTION::TouchModel *pModel,
			Box<float> *pBBox);
		void InitVisualizer(
			Visualizer *pVisualizerIn,
			char *cfgFileName);
		void SetVisualizeOptimization(bool bVisualizeOptimization);


		// Simundic
		void CopyTouchModel(const RVL::MOTION::TouchModel& src, RVL::MOTION::TouchModel& dst);
		void RealExpCorrect(Array<MOTION::TouchData> &touches, 
			std::vector<MOTION::Contact> &contacts,
			Pose3D pose_Ek_E,
			float* V,
			Pose3D pose_A_E,
			Pose3D pose_E_0,
			Pose3D pose_0_S,
			Pose3D pose_C_W_,
			Pose3D pose_C_W_gt_,
			Pose3D pose_C_E);
		MOTION::TouchModel getModelX() const
		{
			return model_x;
		}
		void setModelGTParams(Pose3D pose_A_E, Pose3D pose_C_E, float kappa, float kappazn, float *K, float* TCP_E);
		void setModelEParams(Pose3D pose_A_E, Pose3D pose_C_E, float kappa, float kappazn, float *K, float* TCP_E);
		void loadTransfMatrixFromNPY(std::string fileName, Pose3D &pose);
		void loadVectorFromNPY(std::string fileName, float *vec, int size);
		void CreateSceneLeftAxis(
			float sx,
			float sy,
			float sz,
			float rx,
			float ry,
			float a,
			float b,
			float c,
			float qDeg);	
		private:
		void Constants();
		void AuxParams(
			MOTION::TouchModel *pModel0,
			MOTION::TouchModel *pModelx,
			float gz,
			float hz,
			float *a,
			float &b,
			float *a_,
			float &b_,
			float *D,
			float *invD,
			float *M,
			float *V);
		void VisionVertex(
			float *PSrc,
			float *a,
			float b,
			float *invD,
			Pose3D pose_C_E,
			Pose3D pose_C_E_e,
			float *PTgt);
		void VisionPlane(
			MOTION::Plane *pPlaneSrc,
			float *a_,
			float b_,
			float *D,
			Pose3D *pPose_C_E,
			Pose3D *pPose_C_E_e,
			MOTION::Plane *pPlaneTgt);
		void TestPlaneMapping(MOTION::TouchModel *pModel_x);
		void TestVertexPlaneConsistency();

	public:
		CRVLMem *pMem0;
		CRVLParameterList paramList;
		float kappa;
		float zn;
		Camera camera;
		float alpha;
		float beta;
		int nSamples;
		float contactAngleThr;
		float maxReconstructionError; // m
		bool bRefPtConstraints;
		float toolTiltDeg;
		int simulationSeed;
		int nSimulationTouches;
		bool bDoor;

		// Simundic
		Pose3D pose_A_E;
		float x_real[RVLMOTION_TOUCH_NUM_PARAMS];
		Solid envSolidGT;

	private:
		MOTION::DisplayCallbackData *pVisualizationData;
		Array<RECOG::VN_::ModelCluster *> VNMClusters;
		Array<int> rndVal;
		float stdgxyk;
		float stdgzk;
		float stdhxyk;
		float stdhzk;
		float stdphi;
		float stds;
		float stdc;
		float stdl[4];
		float stdgz;
		float stdhz;
		float stdphirad;
		float varx[RVLMOTION_TOUCH_NUM_PARAMS];
		float csContactAngleThr;
		int iRndVal;
		Pose3D pose_C_W;
		Pose3D pose_E_W;
		Pose3D pose_W_E;
		MOTION::TouchModel model_gt;
		MOTION::TouchModel model_e;
		MOTION::TouchModel model_x;
		float xInc[RVLMOTION_TOUCH_NUM_PARAMS];
		float *SDFBuff;
		Array<MOTION::PlanarSurface> surfaces;
		uchar *surfaceMem;
		Array<MOTION::Vertex> vertices;
		Pair<int, int> *vertexMem;
		MOTION::Tool tool;
		Solid toolMoved;
		Pose3D toolPose;
		Pose3D doorPose;
		Solid envSolid;
		Solid envSolidx;
		int *solidVNAssoc;
		Solid envSolid_E;
		Array<MOTION::TouchData> refPts;
		int doorRefSurfaceIdx;
	};
}
