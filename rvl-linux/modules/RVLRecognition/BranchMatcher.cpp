#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
//#include "ReconstructionEval.h"
#include "SurfelGraph.h"
//#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "RVLRecognitionCommon.h"
#include "RVLBuffer.h"
#include "RVLPtrChain.h"
#include "RVLMPtrChain.h"
#include "Rect.h"
#include "RVLEDT.h"
#include "MSTree.h"
#include "BranchMatcher.h"

using namespace RVL;

BranchMatcher::BranchMatcher()
{
	pVisualizationData = NULL;
}

BranchMatcher::~BranchMatcher()
{
	Clear();
}

void BranchMatcher::Create(char* cfgFileNameIn)
{
	cfgFileName = cfgFileNameIn;
	CreateParamList();
	paramList.LoadParams(cfgFileNameIn);
}

void BranchMatcher::Clear()
{
	if (pVisualizationData)
	{
		if (pVisualizationData->bOwnVisualizer)
			delete pVisualizationData->pVisualizer;
	}
}

void BranchMatcher::CreateParamList()
{

}

void BranchMatcher::Detect(Array2D<short int> depthImage)
{
	// Parameters.

	int depthDiscontinuityThr = 20;		// mm
	float maxParticleDist = 0.2f;		// m
	float particleDistResolution = 0.001f;

	// Constants.

	int nPix = depthImage.w * depthImage.h;

	// Depth discontinuity map.

	bool *bDepthDiscontinuity = new bool[nPix];
	memset(bDepthDiscontinuity, 0, nPix * sizeof(bool));
	int neighborhood4[4];
	neighborhood4[0] = 1; neighborhood4[1] = -depthImage.w; neighborhood4[2] = -1; neighborhood4[3] = depthImage.w;
	int iPix, iPix_;
	int u, v;
	int iNeighbor;
	ushort d, d_;
	int dd;
	int absdd;
	uchar* visPix;
	for (v = 1; v < depthImage.h - 1; v++)
		for (u = 1; u < depthImage.w - 1; u++)
		{
			iPix = u + v * depthImage.w;
			d = depthImage.Element[iPix];
			if (d == 0)
				continue;
			for (iNeighbor = 0; iNeighbor < 4; iNeighbor++)
			{				
				iPix_ = iPix + neighborhood4[iNeighbor];
				d_ = depthImage.Element[iPix_];
				if (d_ == 0)
				{
					bDepthDiscontinuity[iPix] = true;
					break;
				}
				dd = d_ - d;
				if (dd > depthDiscontinuityThr)
				{
					bDepthDiscontinuity[iPix] = true;
					break;
				}
			}
		}

	// Depth discontinuity map visualization.

	//cv::Mat depthDiscontinuityDisplay;
	//depthDiscontinuityDisplay.create(depthImage.h, depthImage.w, CV_8UC1);
	//visPix = depthDiscontinuityDisplay.data;
	//for (iPix = 0; iPix < nPix; iPix++, visPix++)
	//	*visPix = (bDepthDiscontinuity[iPix] ? 255 : 0);
	//cv::imshow("depth discontinuity map", depthDiscontinuityDisplay);
	//cv::waitKey();

	// EDT.

	CRVLEDT EDT;
	EDT.m_Flags |= RVLEDT_FLAG_EDGE_CONTOURS;
	RVLEDT_PIX_ARRAY EDTImage;
	EDTImage.Width = depthImage.w;
	EDTImage.Height = depthImage.h;
	EDTImage.pPix = new RVLEDT_PIX[nPix];
	RVLEDT_BUCKET_ENTRY* EDTBucketMem = new RVLEDT_BUCKET_ENTRY[4 * nPix];
	WORD iBucket;
	for (iBucket = 0; iBucket < 4; iBucket++)
		EDT.m_BucketPtrArray[iBucket] = EDTBucketMem + iBucket * nPix;
	EDT.Border(&EDTImage);
	EDT.CreateRTSqrtLUT();
	CRVLBuffer EDTBuff;
	EDTBuff.DataBuff = new void* [2 * nPix];
	EDTBuff.m_bOwnData = FALSE;
	EDT.m_maxd2 = depthImage.w * depthImage.w + depthImage.h * depthImage.h;
	CRVLMem mem;
	mem.Create(nPix * sizeof(RVLPTRCHAIN_ELEMENT));
	CRVLMPtrChain boundary;
	boundary.m_pMem = &mem;
	RVLEDT_PIX* pEDTPix;
	for (v = 1; v < depthImage.h - 1; v++)
		for (u = 1; u < depthImage.w - 1; u++)
		{
			iPix = u + v * depthImage.w;
			if (depthImage.Element[iPix] > 0 && bDepthDiscontinuity[iPix])
			{
				EDTImage.pPix[iPix].d2 = 0;
				boundary.Add((void*)(EDTImage.pPix + iPix));
			}
			else if(depthImage.Element[iPix] == 0)
				EDTImage.pPix[iPix].d2 = 0;
		}
	EDT.Apply(&boundary, NULL, &EDTImage, &EDTBuff);
	delete[] EDTBucketMem;
	delete[] EDTBuff.DataBuff;

	// Visualize EDT.

	//cv::Mat EDTDisplayImageHSV(depthImage.h, depthImage.w, CV_8UC3);
	//visPix = EDTDisplayImageHSV.data;
	//pEDTPix = EDTImage.pPix;
	//int maxd2 = 140;
	//for (iPix = 0; iPix < nPix; iPix++, visPix += 3, pEDTPix++)
	//	if (pEDTPix->d2 == 0 && !bDepthDiscontinuity[iPix])
	//	{
	//		RVLNULL3VECTOR(visPix);
	//	}
	//	else
	//	{
	//		visPix[0] = (pEDTPix->d2 <= maxd2 ? pEDTPix->d2+1 : maxd2);
	//		visPix[1] = visPix[2] = 255;
	//	}
	//cv::Mat EDTDisplayImageBGR(depthImage.h, depthImage.w, CV_8UC3);
	//cv::cvtColor(EDTDisplayImageHSV, EDTDisplayImageBGR, CV_HSV2RGB);
	//cv::imshow("EDT", EDTDisplayImageBGR);
	//cv::waitKey(); 

	// Particles.

	int neighborhood8[8][2];
	neighborhood8[0][0] = 1; neighborhood8[0][1] = 0;
	neighborhood8[1][0] = 1; neighborhood8[1][1] = -1;
	neighborhood8[2][0] = 0; neighborhood8[2][1] = -1;
	neighborhood8[3][0] = -1; neighborhood8[3][1] = -1;
	neighborhood8[4][0] = -1; neighborhood8[4][1] = 0;
	neighborhood8[5][0] = -1; neighborhood8[5][1] = 1;
	neighborhood8[6][0] = 0; neighborhood8[6][1] = 1;
	neighborhood8[7][0] = 1; neighborhood8[7][1] = 1;
	int neighborhood8_[8];
	float fNeighborhood8[8][2];
	for (iNeighbor = 0; iNeighbor < 8; iNeighbor++)
	{
		neighborhood8_[iNeighbor] = neighborhood8[iNeighbor][0] + neighborhood8[iNeighbor][1] * depthImage.w;
		fNeighborhood8[iNeighbor][0] = (float)(neighborhood8[iNeighbor][0]);
		fNeighborhood8[iNeighbor][1] = (float)(neighborhood8[iNeighbor][1]);
	}
	int *particleMap = new int[nPix];	
	memset(particleMap, 0xff, nPix * sizeof(int));
	bool* bNMS = new bool[nPix];
	memset(bNMS, 0, nPix * sizeof(bool));
	RVLEDT_PIX* pEDTPix_;
	int nCorePixels;
	float fnCorePixels;
	float du, dv;
	float cImg[2];
	float rImg;
	float z;
	float fTmp;
	float V3Tmp[3];
	Rect<int> neighborhood;
	Rect<int> imageRect;
	imageRect.minx = 0;
	imageRect.maxx = depthImage.w - 1;
	imageRect.miny = 0; 
	imageRect.maxy = depthImage.h - 1;
	int rNeighborhood, rNeighborhood2;
	int u_, v_, du_, dv_;
	RECOG::BM::Particle* particleMem = new RECOG::BM::Particle[nPix];
	RECOG::BM::Particle* pParticle = particleMem;
	for (v = 1; v < depthImage.h - 1; v++)
		for (u = 1; u < depthImage.w - 1; u++)
		{
			iPix = u + v * depthImage.w;
			if (bNMS[iPix])
				continue;
			pEDTPix = EDTImage.pPix + iPix;
			if (pEDTPix->d2 > EDT.m_maxd2)
				continue;
			if (pEDTPix->d2 == 0 && !bDepthDiscontinuity[iPix])
				continue;
			for (iNeighbor = 0; iNeighbor < 8; iNeighbor++)
			{
				pEDTPix_ = EDTImage.pPix + iPix + neighborhood8_[iNeighbor];
				if (pEDTPix_->d2 > EDT.m_maxd2)
					continue;
				if (pEDTPix_->d2 > pEDTPix->d2)
					break;
			}
			if (iNeighbor < 8)
				continue;
			pParticle->bExists = true;
			pParticle->iPix = iPix;
			particleMap[iPix] = pParticle - particleMem;
			cImg[0] = cImg[1] = 0.0f;
			nCorePixels = 1;
			for (iNeighbor = 0; iNeighbor < 8; iNeighbor++)
			{
				pEDTPix_ = EDTImage.pPix + iPix + neighborhood8_[iNeighbor];
				if (pEDTPix_->d2 == pEDTPix->d2)
				{
					cImg[0] += fNeighborhood8[iNeighbor][0];
					cImg[1] += fNeighborhood8[iNeighbor][1];
					nCorePixels++;
				}
			}
			fnCorePixels = (float)nCorePixels;
			cImg[0] /= fnCorePixels; cImg[1] /= fnCorePixels;
			du = (float)(pEDTPix->dx) + cImg[0];
			dv = (float)(pEDTPix->dz) + cImg[1];
			rImg = sqrt(du * du + dv * dv) + 0.5f;
			z = 0.001f * (float)(depthImage.Element[iPix]);
			pParticle->r = rImg * z / (camera.fu - rImg);
			pParticle->c[0] = z * ((float)u - camera.uc) / camera.fu;
			pParticle->c[1] = z * ((float)v - camera.vc) / camera.fv;
			pParticle->c[2] = z;
			fTmp = pParticle->r / sqrt(RVLDOTPRODUCT3(pParticle->c, pParticle->c));
			RVLSCALE3VECTOR(pParticle->c, fTmp, V3Tmp);
			RVLSUM3VECTORS(pParticle->c, V3Tmp, pParticle->c);
			pParticle++;
			rNeighborhood = (int)ceil(rImg) + 1;
			rNeighborhood2 = rNeighborhood * rNeighborhood;
			neighborhood.minx = u - rNeighborhood;
			neighborhood.maxx = u + rNeighborhood;
			neighborhood.miny = v - rNeighborhood;
			neighborhood.maxy = v + rNeighborhood;
			CropRect<int>(neighborhood, imageRect);
			for(v_ = neighborhood.miny; v_ <= neighborhood.maxy; v_++)
				for (u_ = neighborhood.minx; u_ <= neighborhood.maxx; u_++)
				{
					du_ = u_ - u;
					dv_ = v_ - v;
					if (du_ * du_ + dv_ * dv_ <= rNeighborhood2)
					{
						iPix_ = u_ + v_ * depthImage.w;
						bNMS[iPix_] = true;
					}
				}
		}
	Array<RECOG::BM::Particle> particles;
	particles.n = pParticle - particleMem;
	particles.Element = new RECOG::BM::Particle[particles.n];
	memcpy(particles.Element, particleMem, particles.n * sizeof(RECOG::BM::Particle));
	delete[] particleMem;
	delete[] EDTImage.pPix;
	delete[] bDepthDiscontinuity;
	delete[] particleMap;
	delete[] bNMS;
	int iParticle;

	// Visualize particle map.

	//cv::Mat particleMapDisplay;
	//particleMapDisplay.create(depthImage.h, depthImage.w, CV_8UC3);
	//DisplayDisparityMap(depthImage, (unsigned char*)(particleMapDisplay.data), true, RVLRGB_DEPTH_FORMAT_1MM);
	//for (iParticle = 0; iParticle < particles.n; iParticle++)
	//{
	//	pParticle = particles.Element + iParticle;
	//	visPix = particleMapDisplay.data + 3 * pParticle->iPix;
	//	RVLSET3VECTOR(visPix, 0, 255, 0);
	//}
	//cv::imshow("Particle map", particleMapDisplay);
	//cv::waitKey();

	// Edges.

	Array<GRAPH::MSTreeEdge> edges;
	edges.Element = new GRAPH::MSTreeEdge[particles.n * particles.n];
	edges.n = 0;
	GRAPH::MSTreeEdge* pEdge;
	int iParticle_;
	RECOG::BM::Particle* pParticle_;
	float dP[3];
	//int* sortKey = new int[particles.n * (particles.n + 1) / 2];
	for (iParticle = 0; iParticle < particles.n; iParticle++)
	{
		pParticle = particles.Element + iParticle;
		for (iParticle_ = 0; iParticle_ < iParticle; iParticle_++)
		{
			pParticle_ = particles.Element + iParticle_;
			pEdge = edges.Element + edges.n;
			pEdge->iVertex[0] = iParticle;
			pEdge->iVertex[1] = iParticle_;
			RVLDIF3VECTORS(pParticle_->c, pParticle->c, dP);
			pEdge->cost = sqrt(RVLDOTPRODUCT3(dP, dP));
			if (pEdge->cost > maxParticleDist)
				continue;
			//sortKey[edges.n] = (int)floor(pEdge->cost / particleDistResolution);
			edges.n++;
		}
	}
	//int* sortIdx = new int[particles.n * (particles.n + 1) / 2];
	//QuickSort(sortKey, sortIdx, edges.n);
	//delete[] sortKey;
	//delete[] sortIdx;

	// Minimum spanning tree.

	MSTree MST;
	MST.Init(particles.n);
	MST.Create(edges);

	// Visualize MST.

	Array<Point> visPts;
	visPts.Element = new Point[particles.n];
	visPts.n = particles.n;
	for (iParticle = 0; iParticle < particles.n; iParticle++)
	{
		pParticle = particles.Element + iParticle;
		RVLCOPY3VECTOR(pParticle->c, visPts.Element[iParticle].P);
	}
	uchar darkGreen[] = {0, 128, 0};
	pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(visPts, darkGreen, 6);
	Array<Pair<int, int>> visEdges;
	visEdges.Element = new Pair<int, int>[particles.n - 1];
	visEdges.n = 0;
	GRAPH::Node* pMSTNode;
	GRAPH::EdgePtr<GRAPH::Edge>* pMSTEdgePtr;
	Pair<int, int>* pVisEdge;
	for (iParticle = 0; iParticle < particles.n; iParticle++)
	{
		pMSTNode = MST.T.NodeArray.Element + iParticle;
		pMSTEdgePtr = pMSTNode->EdgeList.pFirst;
		while (pMSTEdgePtr)
		{
			iParticle_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pMSTEdgePtr);
			if (iParticle_ > iParticle)
			{
				pVisEdge = visEdges.Element + visEdges.n;
				pVisEdge->a = iParticle;
				pVisEdge->b = iParticle_;
				visEdges.n++;
			}
			pMSTEdgePtr = pMSTEdgePtr->pNext;
		}
	}
	uchar blue[] = {0, 0, 255};
	pVisualizationData->pVisualizer->DisplayLines(visPts, visEdges, blue);
	pVisualizationData->pVisualizer->Run();
	delete[] visEdges.Element;
	delete[] visPts.Element;

	//

	delete[] particles.Element;
	delete[] edges.Element;
}

void BranchMatcher::InitVisualizer(Visualizer* pVisualizerIn)
{
	if (pVisualizationData == NULL)
		pVisualizationData = new RECOG::BM::DisplayCallbackData;
	if (pVisualizerIn)
	{
		pVisualizationData->pVisualizer = pVisualizerIn;
		pVisualizationData->bOwnVisualizer = false;
	}
	else
	{
		pVisualizationData->pVisualizer = new Visualizer;
		pVisualizationData->bOwnVisualizer = true;
	}
	pVisualizationData->paramList.m_pMem = pMem0;
	RVLPARAM_DATA* pParamData;
	pVisualizationData->paramList.Init();
	//pParamData = pVisualizationData->paramList.AddParam("DDD.visualizeSurfels", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizeSurfels));
	pVisualizationData->paramList.LoadParams((char*)(cfgFileName.data()));
}

