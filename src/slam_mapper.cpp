/*
 * slam_mapper
 * Copyright (c) 2018, Steve Macenski
 * Copyright (c) 2019, Samsung Research America
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#include "slam_toolbox/slam_mapper.hpp"

namespace mapper_utils
{

/*****************************************************************************/
SMapper::SMapper() : Mapper()
/*****************************************************************************/
{
  dataset_ = std::make_unique<karto::Dataset>();
}

/*****************************************************************************/
SMapper::~SMapper()
/*****************************************************************************/
{
  dataset_.reset();
}

/*****************************************************************************/
karto::Dataset* SMapper::getDataset()
/*****************************************************************************/
{
  return dataset_.get();
}

/*****************************************************************************/
void SMapper::setDataset(karto::Dataset* dataset)
/*****************************************************************************/
{
  dataset_.reset(dataset);
}

/*****************************************************************************/
karto::OccupancyGrid* SMapper::getOccupancyGrid(const double& resolution)
/*****************************************************************************/
{
  karto::OccupancyGrid* occ_grid = nullptr;
  return karto::OccupancyGrid::CreateFromScans(GetAllProcessedScans(),
    resolution);
}

/*****************************************************************************/
tf2::Transform SMapper::toTfPose(const karto::Pose2& pose) const
/*****************************************************************************/
{
  tf2::Transform new_pose;
  new_pose.setOrigin(tf2::Vector3(pose.GetX(), pose.GetY(), 0.));
  tf2::Quaternion q;
  q.setRPY(0., 0., pose.GetHeading());
  new_pose.setRotation(q);
  return new_pose;
};

/*****************************************************************************/
karto::Pose2 SMapper::toKartoPose(const tf2::Transform& pose) const
/*****************************************************************************/
{
  karto::Pose2 transformed_pose;
  transformed_pose.SetX(pose.getOrigin().x());
  transformed_pose.SetY(pose.getOrigin().y());
  transformed_pose.SetHeading(tf2::getYaw(pose.getRotation()));
  return transformed_pose;
};

void SMapper::configure(const ros::NodeHandle& nh)
{
  bool use_scan_matching;
  if(nh.getParam("use_scan_matching", use_scan_matching))
  {
    setParamUseScanMatching(use_scan_matching);
  }
  
  bool use_scan_barycenter;
  if(nh.getParam("use_scan_barycenter", use_scan_barycenter))
  {
    setParamUseScanBarycenter(use_scan_barycenter);
  }

  double minimum_travel_distance = 0.5;
  if(nh.getParam("minimum_travel_distance", minimum_travel_distance))
  {
    setParamMinimumTravelDistance(minimum_travel_distance);
  }

  double minimum_travel_heading;
  if(nh.getParam("minimum_travel_heading", minimum_travel_heading))
  {
    setParamMinimumTravelHeading(minimum_travel_heading);
  }

  int scan_buffer_size;
  if(nh.getParam("scan_buffer_size", scan_buffer_size))
  {
    setParamScanBufferSize(scan_buffer_size);
  }

  double scan_buffer_maximum_scan_distance;
  if(nh.getParam("scan_buffer_maximum_scan_distance",
    scan_buffer_maximum_scan_distance))
  {
    setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);
  }

  double link_match_minimum_response_fine;
  if(nh.getParam("link_match_minimum_response_fine",
    link_match_minimum_response_fine))
  {
    setParamLinkMatchMinimumResponseFine(link_match_minimum_response_fine);
  }

  double link_scan_maximum_distance;
  if(nh.getParam("link_scan_maximum_distance", link_scan_maximum_distance))
  {
    setParamLinkScanMaximumDistance(link_scan_maximum_distance);
  }

  double loop_search_maximum_distance;
  if(nh.getParam("loop_search_maximum_distance", loop_search_maximum_distance))
  {
    setParamLoopSearchMaximumDistance(loop_search_maximum_distance);
  }

  bool do_loop_closing;
  if(nh.getParam("do_loop_closing", do_loop_closing))
  {
    setParamDoLoopClosing(do_loop_closing);
  }

  int loop_match_minimum_chain_size;
  if(nh.getParam("loop_match_minimum_chain_size",
    loop_match_minimum_chain_size))
  {
    setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);
  }

  double loop_match_maximum_variance_coarse;
  if(nh.getParam("loop_match_maximum_variance_coarse",
    loop_match_maximum_variance_coarse))
  {
    setParamLoopMatchMaximumVarianceCoarse(loop_match_maximum_variance_coarse);
  }

  double loop_match_minimum_response_coarse;
  if(nh.getParam("loop_match_minimum_response_coarse",
    loop_match_minimum_response_coarse))
  {
    setParamLoopMatchMinimumResponseCoarse(loop_match_minimum_response_coarse);
  }

  double loop_match_minimum_response_fine;
  if(nh.getParam("loop_match_minimum_response_fine",
    loop_match_minimum_response_fine))
  {
    setParamLoopMatchMinimumResponseFine(loop_match_minimum_response_fine);
  }

  // Setting Correlation Parameters
  double correlation_search_space_dimension;
  if(nh.getParam("correlation_search_space_dimension",
    correlation_search_space_dimension))
  {
    setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);
  }

  double correlation_search_space_resolution;
  if(nh.getParam("correlation_search_space_resolution",
    correlation_search_space_resolution))
  {
    setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);
  }

  double correlation_search_space_smear_deviation;
  if(nh.getParam("correlation_search_space_smear_deviation",
    correlation_search_space_smear_deviation))
  {
    setParamCorrelationSearchSpaceSmearDeviation(
      correlation_search_space_smear_deviation);
  }

  // Setting Correlation Parameters, Loop Closure Parameters
  double loop_search_space_dimension;
  if(nh.getParam("loop_search_space_dimension", loop_search_space_dimension))
  {
    setParamLoopSearchSpaceDimension(loop_search_space_dimension);
  }

  double loop_search_space_resolution;
  if(nh.getParam("loop_search_space_resolution", loop_search_space_resolution))
  {
    setParamLoopSearchSpaceResolution(loop_search_space_resolution);
  }

  double loop_search_space_smear_deviation;
  if(nh.getParam("loop_search_space_smear_deviation",
    loop_search_space_smear_deviation))
  {
    setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);
  }

  // Setting Scan Matcher Parameters
  double distance_variance_penalty;
  if(nh.getParam("distance_variance_penalty", distance_variance_penalty))
  {
    setParamDistanceVariancePenalty(distance_variance_penalty);
  }

  double angle_variance_penalty;
  if(nh.getParam("angle_variance_penalty", angle_variance_penalty))
  {
    setParamAngleVariancePenalty(angle_variance_penalty);
  }

  double fine_search_angle_offset;
  if(nh.getParam("fine_search_angle_offset", fine_search_angle_offset))
  {
    setParamFineSearchAngleOffset(fine_search_angle_offset);
  }

  double coarse_search_angle_offset;
  if(nh.getParam("coarse_search_angle_offset", coarse_search_angle_offset))
  {
    setParamCoarseSearchAngleOffset(coarse_search_angle_offset);
  }

  double coarse_angle_resolution;
  if(nh.getParam("coarse_angle_resolution", coarse_angle_resolution))
  {
    setParamCoarseAngleResolution(coarse_angle_resolution);
  }

  double minimum_angle_penalty;
  if(nh.getParam("minimum_angle_penalty", minimum_angle_penalty))
  {
    setParamMinimumAnglePenalty(minimum_angle_penalty);
  }

  double minimum_distance_penalty;
  if(nh.getParam("minimum_distance_penalty", minimum_distance_penalty))
  {
    setParamMinimumDistancePenalty(minimum_distance_penalty);
  }

  bool use_response_expansion;
  if(nh.getParam("use_response_expansion", use_response_expansion))
  {
    setParamUseResponseExpansion(use_response_expansion);
  }
  return;
}

/*****************************************************************************/
kt_bool SMapper::ProcessAtDock(LocalizedRangeScan* pScan)
/*****************************************************************************/
{
  // Special case of processing against node where node is the starting point
  return ProcessAgainstNode(pScan, 0);
}

/*****************************************************************************/
kt_bool SMapper::ProcessAgainstNode(LocalizedRangeScan* pScan, 
  const int& nodeId)
/*****************************************************************************/
{
  if (pScan != NULL)
  {
    karto::LaserRangeFinder* pLaserRangeFinder = pScan->GetLaserRangeFinder();

    // validate scan
    if (pLaserRangeFinder == NULL || pScan == NULL || 
      pLaserRangeFinder->Validate(pScan) == false)
    {
      return false;
    }

    if (m_Initialized == false)
    {
      // initialize mapper with range threshold from device
      Initialize(pLaserRangeFinder->GetRangeThreshold());
    }

    // If we're matching against a node from an older mapping session
    // lets get the first scan as the last scan and populate running scans
    // with the first few from that run as well.
    LocalizedRangeScan* pLastScan =
    m_pMapperSensorManager->GetScan(pScan->GetSensorName(), nodeId);
    m_pMapperSensorManager->ClearRunningScans(pScan->GetSensorName());
    m_pMapperSensorManager->AddRunningScan(pLastScan);
    m_pMapperSensorManager->SetLastScan(pLastScan);

    Matrix3 covariance;
    covariance.SetToIdentity();

    // correct scan (if not first scan)
    if (m_pUseScanMatching->GetValue() && pLastScan != NULL)
    {
      Pose2 bestPose;
      m_pSequentialScanMatcher->MatchScan(pScan,
          m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()),
          bestPose,
          covariance);
      pScan->SetSensorPose(bestPose);
    }

    pScan->SetOdometricPose(pScan->GetCorrectedPose());

    // add scan to buffer and assign id
    m_pMapperSensorManager->AddScan(pScan);

    if (m_pUseScanMatching->GetValue())
    {
      // add to graph
      m_pGraph->AddVertex(pScan);
      m_pGraph->AddEdges(pScan, covariance);

      m_pMapperSensorManager->AddRunningScan(pScan);

      if (m_pDoLoopClosing->GetValue())
      {
        std::vector<Name> deviceNames =
        m_pMapperSensorManager->GetSensorNames();
        const_forEach(std::vector<Name>, &deviceNames)
        {
          m_pGraph->TryCloseLoop(pScan, *iter);
        }
      }
    }

    m_pMapperSensorManager->SetLastScan(pScan);

    return true;
  }

  return false;
}

/*****************************************************************************/
kt_bool SMapper::ProcessAgainstNodesNearBy(LocalizedRangeScan* pScan)
/*****************************************************************************/
{
  if (pScan != NULL)
  {
    karto::LaserRangeFinder* pLaserRangeFinder = pScan->GetLaserRangeFinder();

    // validate scan
    if (pLaserRangeFinder == NULL || pScan == NULL ||
      pLaserRangeFinder->Validate(pScan) == false)
    {
      return false;
    }

    if (m_Initialized == false)
    {
      // initialize mapper with range threshold from device
      Initialize(pLaserRangeFinder->GetRangeThreshold());
    }

    Vertex<LocalizedRangeScan>* closetVertex =m_pGraph->FindNearByScan(
      pScan->GetSensorName(), pScan->GetOdometricPose());
    LocalizedRangeScan* pLastScan = NULL;
    if (closetVertex)
    {
      pLastScan = m_pMapperSensorManager->GetScan(pScan->GetSensorName(),
        closetVertex->GetObject()->GetUniqueId());
      m_pMapperSensorManager->ClearRunningScans(pScan->GetSensorName());
      m_pMapperSensorManager->AddRunningScan(pLastScan);
      m_pMapperSensorManager->SetLastScan(pLastScan);
    }

    Matrix3 covariance;
    covariance.SetToIdentity();

    // correct scan (if not first scan)
    if (m_pUseScanMatching->GetValue() && pLastScan != NULL)
    {
      Pose2 bestPose;
      m_pSequentialScanMatcher->MatchScan(pScan,
          m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()),
          bestPose,
          covariance);
      pScan->SetSensorPose(bestPose);
    }

    pScan->SetOdometricPose(pScan->GetCorrectedPose());

    // add scan to buffer and assign id
    m_pMapperSensorManager->AddScan(pScan);

    if (m_pUseScanMatching->GetValue())
    {
      // add to graph
      m_pGraph->AddVertex(pScan);
      m_pGraph->AddEdges(pScan, covariance);

      m_pMapperSensorManager->AddRunningScan(pScan);

      if (m_pDoLoopClosing->GetValue())
      {
        std::vector<Name> deviceNames =
          m_pMapperSensorManager->GetSensorNames();
        const_forEach(std::vector<Name>, &deviceNames)
        {
          m_pGraph->TryCloseLoop(pScan, *iter);
        }
      }
    }

    m_pMapperSensorManager->SetLastScan(pScan);

    return true;
  }

  return false;
}

/*****************************************************************************/
kt_bool SMapper::ProcessLocalization(LocalizedRangeScan* pScan)
/*****************************************************************************/
{
  if (pScan == NULL)
  {
    return false;
  }

  karto::LaserRangeFinder* pLaserRangeFinder = pScan->GetLaserRangeFinder();

  // validate scan
  if (pLaserRangeFinder == NULL || pScan == NULL ||
    pLaserRangeFinder->Validate(pScan) == false)
  {
    return false;
  }

  if (m_Initialized == false)
  {
    // initialize mapper with range threshold from device
    Initialize(pLaserRangeFinder->GetRangeThreshold());
  }

  // get last scan
  LocalizedRangeScan* pLastScan = m_pMapperSensorManager->GetLastScan(
    pScan->GetSensorName());

  // update scans corrected pose based on last correction
  if (pLastScan != NULL)
  {
    Transform lastTransform(pLastScan->GetOdometricPose(),
      pLastScan->GetCorrectedPose());
    pScan->SetCorrectedPose(lastTransform.TransformPose(
      pScan->GetOdometricPose()));
  }

  // test if scan is outside minimum boundary 
  // or if heading is larger then minimum heading
  if (!HasMovedEnough(pScan, pLastScan))
  {
    return false;
  }

  Matrix3 covariance;
  covariance.SetToIdentity();

  // correct scan (if not first scan)
  if (m_pUseScanMatching->GetValue() && pLastScan != NULL)
  {
    Pose2 bestPose;
    m_pSequentialScanMatcher->MatchScan(pScan,
        m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()),
        bestPose,
        covariance);
    pScan->SetSensorPose(bestPose);
  }

  // add scan to buffer and assign id
  m_pMapperSensorManager->AddScan(pScan);

  Vertex<LocalizedRangeScan>* scan_vertex = NULL;
  if (m_pUseScanMatching->GetValue())
  {
    // add to graph
    scan_vertex = m_pGraph->AddVertex(pScan);
    m_pGraph->AddEdges(pScan, covariance);

    m_pMapperSensorManager->AddRunningScan(pScan);
    
    if (m_pDoLoopClosing->GetValue())
    {
      std::vector<Name> deviceNames = m_pMapperSensorManager->GetSensorNames();
      const_forEach(std::vector<Name>, &deviceNames)
      {
        m_pGraph->TryCloseLoop(pScan, *iter);
      }
    }
  }

  m_pMapperSensorManager->SetLastScan(pScan);

  // generate the info to store and later decay, outside of dataset
  if (m_LocalizationScanVertices.size() > getParamScanBufferSize())
  {
    LocalizationScanVertex& oldLSV = m_LocalizationScanVertices.front();

    // 1) delete edges in adjacent vertices, graph, and optimizer
    std::vector<Vertex<LocalizedRangeScan>*> adjVerts =
      oldLSV.vertex->GetAdjacentVertices();
    for (int i = 0; i != adjVerts.size(); i++)
    {
      std::vector<Edge<LocalizedRangeScan>*> adjEdges = adjVerts[i]->GetEdges();
      bool found = false;
      for (int j=0; j!=adjEdges.size(); j++)
      {
        if (adjEdges[j]->GetTarget() == oldLSV.vertex ||
          adjEdges[j]->GetSource() == oldLSV.vertex)
        {
          adjVerts[i]->RemoveEdge(j);
          m_pScanOptimizer->RemoveConstraint(
            adjEdges[j]->GetSource()->GetObject()->GetUniqueId(),
            adjEdges[j]->GetTarget()->GetObject()->GetUniqueId()); 
          std::vector<Edge<LocalizedRangeScan>*> edges = m_pGraph->GetEdges();
          std::vector<Edge<LocalizedRangeScan>*>::iterator edgeGraphIt =
            std::find(edges.begin(), edges.end(), adjEdges[j]);

          if (edgeGraphIt == edges.end())
          {
            std::cout << "Edge not found in graph to remove!" << std::endl;
            continue;
          }

          int posEdge = edgeGraphIt - edges.begin();
          m_pGraph->RemoveEdge(posEdge); // remove from graph
          delete *edgeGraphIt; // free hat!
          *edgeGraphIt = NULL;
          found = true;
        }
      }
      if (!found)
      {
        std::cout << "Failed to find any edge in adj. vertex" <<
          " with a matching vertex to current!" << std::endl;
      }
    }

    // 2) delete vertex from optimizer
    m_pScanOptimizer->RemoveNode(oldLSV.vertex->GetObject()->GetUniqueId());

    // 3) delete from vertex map
    std::map<Name, std::vector<Vertex<LocalizedRangeScan>*> > 
      vertexMap = m_pGraph->GetVertices();
    std::vector<Vertex<LocalizedRangeScan>*> graphVertices =
      vertexMap[pScan->GetSensorName()];
    std::vector<Vertex<LocalizedRangeScan>*>::iterator
      vertexGraphIt = std::find(graphVertices.begin(),
      graphVertices.end(), oldLSV.vertex);
    if (vertexGraphIt != graphVertices.end())
    {
      // right now just sets to NULL, vector map will 
      // scale in size but just contain a bunch of null pointers
      int posVert = vertexGraphIt - graphVertices.begin();
      m_pGraph->RemoveVertex(pScan->GetSensorName(), posVert);
    }
    else
    {
      std::cout << "Vertex not found in graph to remove!" << std::endl;
    }

    // 4) delete node and scans
    // free hat!
    // No need to delete from m_scans as those pointers will be freed memory
    if (oldLSV.vertex)
    {
      delete oldLSV.vertex;
      oldLSV.vertex = NULL;
    }
    
    m_pMapperSensorManager->RemoveScan(oldLSV.scan);
    if (oldLSV.scan)
    {
      delete oldLSV.scan;
      oldLSV.scan = NULL;
    }

    m_LocalizationScanVertices.pop();
  }

  LocalizationScanVertex lsv;
  lsv.scan = pScan;
  lsv.vertex = scan_vertex;
  m_LocalizationScanVertices.push(lsv);

  return true;
}

/*****************************************************************************/
void SMapper::Reset()
/*****************************************************************************/
{
  Mapper::Reset();
  while (!m_LocalizationScanVertices.empty())
  {
    m_LocalizationScanVertices.pop();
  }
  return;
}

} // end namespace
