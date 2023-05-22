#pragma once

namespace RVL
{
	namespace RECOG
	{
		namespace BM
		{
			struct Particle
			{
				bool bExists;
				float c[3];
				float r;
				int iPix;
			};

			struct DisplayCallbackData
			{
				Visualizer* pVisualizer;
				bool bOwnVisualizer;
				CRVLParameterList paramList;
				Mesh* pMesh;
			};
		}
	}

	class BranchMatcher
	{
	public:
		BranchMatcher();
		virtual ~BranchMatcher();
		void Create(char* cfgFileName);
		void Clear();
		void CreateParamList();
		void Detect(Array2D<short int> depthImage);
		void InitVisualizer(Visualizer* pVisualizer = NULL);

	public:
		CRVLMem* pMem0;
		CRVLParameterList paramList;
		Camera camera;
		RECOG::BM::DisplayCallbackData* pVisualizationData;
		std::string cfgFileName;
	};
}

