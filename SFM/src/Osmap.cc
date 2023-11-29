/**
* This file is part of OSMAP.
*
* Copyright (C) 2018-2019 Alejandro Silvestri <alejandrosilvestri at gmail>
* For more information see <https://github.com/AlejandroSilvestri/osmap>
*
* OSMAP is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OSMAP is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OSMAP. If not, see <http://www.gnu.org/licenses/>.
*/

#include <fstream>
#include <iostream>
#include <assert.h>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include "Osmap.h"

// Option check macro
#define OPTION(OP) if(options[OP]) headerFile << #OP;

// Log variable
#define LOGV(VAR) if(verbose) cout << #VAR << ": " << VAR << endl;

using namespace std;
using namespace cv;

namespace ORB_SLAM2{

Osmap::Osmap(System &_system):
	map(static_cast<OsmapMap&>(*_system.map_)),
	keyFrameDatabase(*_system.keyframe_database_),
	system(_system),
	currentFrame(_system.tracker_->current_frame_)
{
#ifndef OSMAP_DUMMY_MAP

	/* Every new MapPoint require a dummy pRefKF in its constructor, copying the following parameters:
	 *
	 * - mnFirstKFid(pRefKF->id_)
	 * - mnFirstFrame(pRefKF->mnFrameId)
	 * - reference_keyframe_(pRefKF)
	 *
	 * A fake keyframe construction requires a Frame with
	 *
	 *  - mTcw_ (must be provided, error if not)
	 *  - Grid (already exists)
	 */
	Frame dummyFrame;
	dummyFrame.Tcw_ = Eigen::Matrix4d::Identity();
	pRefKF = new KeyFrame(dummyFrame, &map, &keyFrameDatabase);

#endif
};


void Osmap::mapSave(const string givenFilename, bool pauseThreads){
	// Stop threads
	if(pauseThreads){
		system.local_mapper_->RequestStop();
		while(!system.local_mapper_->isStopped()) usleep(1000);
	}

	// Strip out .yaml if present
	string baseFilename, filename, pathDirectory;
	parsePath(givenFilename, &filename, &pathDirectory);
	if(pathDirectory != "")
		chdir(pathDirectory.c_str());

	int length = filename.length();
	if(length>5 && filename.substr(length-5) == ".yaml")
	  baseFilename = filename.substr(0, length-5);
	else
	  baseFilename = filename;

	// Map depuration
	if(!options[NO_DEPURATION])
		depurate();


	// Actual saving
	filename = baseFilename + ".yaml";

	// Open YAML file for write, it will be the last file to close.
	// FileStorage https://docs.opencv.org/3.1.0/da/d56/classcv_1_1FileStorage.html
	FileStorage headerFile(filename, FileStorage::WRITE);
	if(!headerFile.isOpened()){
	// Is this necessary?
	 cerr << "Couldn't create file " << baseFilename << ".yaml, map not saved." << endl;
	 return;
	}

	// MapPoints
	if(!options[NO_MAPPOINTS_FILE]){
	  // Order mappoints by id_
	  getMapPointsFromMap();

	  // New file
	  filename = baseFilename + ".mappoints";

	  // Serialize
	  cout << "Saving " << filename << endl;
	  headerFile << "mappointsFile" << filename;
	  headerFile << "nMappoints" << MapPointsSave(filename);
	}

	// K: grab camera calibration matrices.  Will be saved to yaml file later.
	if(!options[K_IN_KEYFRAME]) getVectorKFromKeyframes();

	// KeyFrames
	if(!options[NO_KEYFRAMES_FILE]){
	  getKeyFramesFromMap();

	  // New file
	  filename = baseFilename + ".keyframes";

	  // Serialize
	  cout << "Saving " << filename << endl;
	  headerFile << "keyframesFile" << filename;
	  headerFile << "nKeyframes" << KeyFramesSave(filename);
	}

	// Features
	if(!options[NO_FEATURES_FILE]){
	  filename = baseFilename + ".features";
	  cout << "Saving " << filename << endl;
	  headerFile << "featuresFile" << filename;
	  headerFile << "nFeatures" << featuresSave(filename);
	}


	// Save options, as an int
	headerFile << "Options" << (int) options.to_ulong();
	// Options
	if(options.any()){
	headerFile << "Options descriptions" << "[:";
	OPTION(NO_LOOPS)
	OPTION(NO_FEATURES_DESCRIPTORS)
	OPTION(K_IN_KEYFRAME)
	OPTION(ONLY_MAPPOINTS_FEATURES)
	OPTION(FEATURES_FILE_DELIMITED)
	OPTION(FEATURES_FILE_NOT_DELIMITED)
	OPTION(NO_MAPPOINTS_FILE)
	OPTION(NO_KEYFRAMES_FILE)
	OPTION(NO_FEATURES_FILE)
	headerFile << "]";
	}


	// K: camera calibration matrices, save to yaml at the end of file.
	if(!options[K_IN_KEYFRAME]){
	// Save K matrices in header file yaml
	headerFile << "cameraMatrices" << "[";
	for(auto pK:vectorK)
	   headerFile << "{:"  << "fx" << pK->at<float>(0,0) << "fy" << pK->at<float>(1,1) << "cx" << pK->at<float>(0,2) << "cy" << pK->at<float>(1,2) << "}";
	headerFile << "]";
	}

	// Save yaml file
	headerFile.release();

	// Clear temporary vectors
	clearVectors();

	// if(pauseThreads)
	//   system.viewer_->Release();
}

void Osmap::mapLoad(string yamlFilename, bool noSetBad, bool pauseThreads){
#ifndef OSMAP_DUMMY_MAP
	LOGV(system.tracker_->state_)
	// Initialize currentFrame via calling GrabImageMonocular just in case, with a dummy image.
	if(system.tracker_->state_ == ORB_SLAM2::Tracking::NO_IMAGES_YET){
		Mat m = Mat::zeros(100, 100, CV_8U);
		system.tracker_->GrabImageMonocular(m, 0.0);
	}
#endif

	if(pauseThreads){
		// Reset thr tracker to clean the map
		system.local_mapper_->Release();	// Release local mapper just in case it's stopped, because if it is stopped it can't be reset
		system.tracker_->Reset();
		// Here the system is reset, state is NO_IMAGE_YET

		// Stop LocalMapping and Viewer
		system.local_mapper_->RequestStop();
		// system.viewer_	    ->RequestStop();
		while(!system.local_mapper_->isStopped()) usleep(1000);
		// while(!system.viewer_     ->isStopped()) usleep(1000);
	}

#if !defined OSMAP_DUMMY_MAP && !defined OS1
	if(system.tracker_->relative_frame_poses_.empty()){
		// Add dummy point to trajectory recorder to avoid errors.  The point is in the origin of the map's reference system.
		system.tracker_->relative_frame_poses_.push_back(Eigen::Matrix4d::Identity());
		system.tracker_->reference_keyframes_.push_back(nullptr);
		system.tracker_->frame_times_.push_back(0.0);
		system.tracker_->do_lostes_.push_back(true);
	}
#endif

	LOGV(system.local_mapper_->isStopped())
	// LOGV(system.viewer_     ->isStopped())

	string filename;
	int intOptions;

	// Open YAML
	cv::FileStorage headerFile(yamlFilename, cv::FileStorage::READ);

	// Options
	headerFile["Options"] >> intOptions;
	options = intOptions;

	// K
	if(!options[K_IN_KEYFRAME]){
		vectorK.clear();
		FileNode cameraMatrices = headerFile["cameraMatrices"];
		FileNodeIterator it = cameraMatrices.begin(), it_end = cameraMatrices.end();
		for( ; it != it_end; ++it){
			Mat *k = new Mat();
			*k = Mat::eye(3,3,CV_32F);
			k->at<float>(0,0) = (*it)["fx"];
			k->at<float>(1,1) = (*it)["fy"];
			k->at<float>(0,2) = (*it)["cx"];
			k->at<float>(1,2) = (*it)["cy"];
			vectorK.push_back(k);
		}
	}


	// Change directory
	string pathDirectory;
	parsePath(yamlFilename, NULL, &pathDirectory);
	if(pathDirectory != "")
		chdir(pathDirectory.c_str());


	// MapPoints
	vectorMapPoints.clear();
	if(!options[NO_MAPPOINTS_FILE]){
		headerFile["mappointsFile"] >> filename;
		MapPointsLoad(filename);
	}


	// KeyFrames
	vectorKeyFrames.clear();
	if(!options[NO_KEYFRAMES_FILE]){
		headerFile["keyframesFile"] >> filename;
		KeyFramesLoad(filename);
	}

	// Features
	if(!options[NO_FEATURES_FILE]){
		headerFile["featuresFile"] >> filename;
		cout << "Loading features from " << filename << " ..." << endl;
		featuresLoad(filename);
	}

	// Close yaml file
	headerFile.release();

	// Rebuild
	rebuild(noSetBad);

	// Copy to map
	setMapPointsToMap();
	setKeyFramesToMap();

	// Release temporary vectors
	clearVectors();

#ifndef OSMAP_DUMMY_MAP
// Lost state, the system must relocalize itself in the just loaded map.
	system.tracker_->state_ = ORB_SLAM2::Tracking::LOST;
#endif

	if(pauseThreads){
		// // Resume threads

		// // Reactivate viewer.  Do not reactivate localMapper because the system resumes in "only tracking" mode immediatly after loading.
		// system.viewer_->Release();

		// // Tracking do this when going to LOST state.
		// // Invoked after viewer.Release() because of mutex.
		// system.frame_drawer_->Update(system.tracker_);
	}
}

int Osmap::MapPointsSave(string filename){
	ofstream file;
	file.open(filename, std::ofstream::binary);

	// Serialize
	SerializedMappointArray serializedMappointArray;
	int nMP = serialize(vectorMapPoints, serializedMappointArray);

	// Closing
	if (!serializedMappointArray.SerializeToOstream(&file))
		// Signals the error
		nMP = -1;
	file.close();

	return nMP;
}

int Osmap::MapPointsLoad(string filename){
	ifstream file;
	file.open(filename, ifstream::binary);

	SerializedMappointArray serializedMappointArray;
	serializedMappointArray.ParseFromIstream(&file);
	int nMP = deserialize(serializedMappointArray, vectorMapPoints);
	cout << "Mappoints loaded: " << nMP << endl;

	file.close();
	return nMP;
}

int Osmap::KeyFramesSave(string filename){
	ofstream file;
	file.open(filename, std::ofstream::binary);

	// Serialize
	SerializedKeyframeArray serializedKeyFrameArray;
	int nKF = serialize(vectorKeyFrames, serializedKeyFrameArray);

	// Closing
	if (!serializedKeyFrameArray.SerializeToOstream(&file))
		// Signals the error
		nKF = -1;
	file.close();

	return nKF;
}

int Osmap::KeyFramesLoad(string filename){
	ifstream file;
	file.open(filename, ifstream::binary);
#ifndef OSMAP_DUMMY_MAP
	if(!(int)currentFrame.Tcw_.size())	// if map is no initialized, currentFrame has no pose, a pose is needed to create keyframes.
		currentFrame.Tcw_ = Eigen::Matrix4d::Identity();
#endif
	SerializedKeyframeArray serializedKeyFrameArray;
	serializedKeyFrameArray.ParseFromIstream(&file);
	int nKF = deserialize(serializedKeyFrameArray, vectorKeyFrames);
	cout << "Keyframes loaded: "
		<< nKF << endl;
	file.close();
	return nKF;
}

int Osmap::featuresSave(string filename){
	int nFeatures = 0;
	ofstream file;

	file.open(filename, ofstream::binary);
	if(
		options[FEATURES_FILE_DELIMITED] ||
		(!options[FEATURES_FILE_NOT_DELIMITED] && countFeatures() > FEATURES_MESSAGE_LIMIT)
	){
		// Saving with delimited ad hoc file format
		// Loop serializing blocks of no more than FEATURES_MESSAGE_LIMIT features, using Kendon Varda's function

		options.set(FEATURES_FILE_DELIMITED);

		// This Protocol Buffers stream must be deleted before closing file.  It happens automatically at }.
		::google::protobuf::io::OstreamOutputStream protocolbuffersStream(&file);
		vector<OsmapKeyFrame*> vectorBlock;
		vectorBlock.reserve(FEATURES_MESSAGE_LIMIT/30);

		auto it = vectorKeyFrames.begin();
		while(it != vectorKeyFrames.end()){
			unsigned int n = (*it)->N_;
			vectorBlock.clear();
			do{
				vectorBlock.push_back(*it);
				++it;
				if(it == vectorKeyFrames.end()) break;
				KeyFrame *KF = *it;
				n += KF->N_;
			} while(n <= FEATURES_MESSAGE_LIMIT);

			SerializedKeyframeFeaturesArray serializedKeyframeFeaturesArray;
			nFeatures += serialize(vectorBlock, serializedKeyframeFeaturesArray);
			writeDelimitedTo(serializedKeyframeFeaturesArray, &protocolbuffersStream);
		}
	}else{
		options.set(FEATURES_FILE_NOT_DELIMITED);
		SerializedKeyframeFeaturesArray serializedKeyframeFeaturesArray;
		nFeatures = serialize(vectorKeyFrames, serializedKeyframeFeaturesArray);
		if (!serializedKeyframeFeaturesArray.SerializeToOstream(&file)){
			cerr << "Error while serializing features file without delimitation." << endl;
			nFeatures = -1;
		}
	}
	file.close();

	return nFeatures;
}

int Osmap::featuresLoad(string filename){
	int nFeatures = 0;
	ifstream file;
	file.open(filename, ifstream::binary);
	auto *googleStream = new ::google::protobuf::io::IstreamInputStream(&file);
	SerializedKeyframeFeaturesArray serializedKeyframeFeaturesArray;
	if(options[FEATURES_FILE_DELIMITED]){
		while(true)
			if(readDelimitedFrom(googleStream, &serializedKeyframeFeaturesArray)){
				nFeatures += deserialize(serializedKeyframeFeaturesArray);
				cout << "Features deserialized in loop: "
					 << nFeatures << endl;
			}
			else
				break;
	} else {
		// Not delimited, pure Protocol Buffers
		serializedKeyframeFeaturesArray.ParseFromIstream(&file);
		nFeatures = deserialize(serializedKeyframeFeaturesArray);
	  }
	cout << "Features loaded: " << nFeatures << endl;
	file.close();
	return nFeatures;
}

void Osmap::getMapPointsFromMap(){
	  vectorMapPoints.clear();
	  vectorMapPoints.reserve(map.map_points_.size());
	  std::transform(map.map_points_.begin(), map.map_points_.end(), std::back_inserter(vectorMapPoints), [](MapPoint *pMP)->OsmapMapPoint*{return static_cast<OsmapMapPoint*>(pMP);});
	  sort(vectorMapPoints.begin(), vectorMapPoints.end(), [](const MapPoint* a, const MapPoint* b){return a->id_ < b->id_;});
}

void Osmap::setMapPointsToMap(){
	map.map_points_.clear();
	copy(vectorMapPoints.begin(), vectorMapPoints.end(), inserter(map.map_points_, map.map_points_.end()));
}

void Osmap::getKeyFramesFromMap(){
	// Order keyframes by id_
	vectorKeyFrames.clear();
	vectorKeyFrames.reserve(map.keyframes_.size());
	std::transform(map.keyframes_.begin(), map.keyframes_.end(), std::back_inserter(vectorKeyFrames), [](KeyFrame *pKF)->OsmapKeyFrame*{return static_cast<OsmapKeyFrame*>(pKF);});
	sort(vectorKeyFrames.begin(), vectorKeyFrames.end(), [](const KeyFrame *a, const KeyFrame *b){return a->id_ < b->id_;});
}

void Osmap::setKeyFramesToMap(){
	map.keyframes_.clear();
	copy(vectorKeyFrames.begin(), vectorKeyFrames.end(), inserter(map.keyframes_, map.keyframes_.end()));
}



void Osmap::clearVectors(){
	keyframeid2vectorkIdx.clear();
	vectorKeyFrames.clear();
	vectorMapPoints.clear();
	vectorK.clear();
}

void Osmap::parsePath(const string &path, string *filename, string *pathDirectory){
	size_t pos = path.find_last_of('/');
	if(std::string::npos == pos)
		// No directory separator, file is assumed.
		pos = 0;
	else
		// Last directory separator (/) will be in pathDirectory, not in filename.
		pos++;

	if(pathDirectory)
		*pathDirectory = path.substr(0, pos);
	if(filename)
		*filename = path.substr(pos);
	return;
}


void Osmap::depurate(){
	// First erase MapPoint from KeyFrames, and then erase KeyFrames from MapPoints.

	// NULL out bad MapPoints in KeyFrame::map_points_
	for(auto pKF: map.keyframes_){
		// NULL out bad MapPoints and warns if not in map.  Usually doesn't find anything.
		auto pOKF = static_cast<OsmapKeyFrame*>(pKF);
		auto &pMPs = pOKF->map_points_;
		for(int i=pMPs.size(); --i>=0;){
			auto pOMP = static_cast<OsmapMapPoint *>(pMPs[i]);

			if(!pOMP) continue;	// Ignore if NULL

			if(pOMP->is_bad_){
				// If MapPoint is bad, NULL it in keyframe's observations.
				cerr << "depurate(): Nullifying bad MapPoint " << pOMP->id_ << " in KeyFrame " << pOKF->id_ << endl;
				pMPs[i] = NULL;
			} else if(!map.map_points_.count(pOMP) && !options[NO_APPEND_FOUND_MAPPOINTS]){
				// If MapPoint is not in map, append it to the map
				map.map_points_.insert(pOMP);
				cout << "depurate(): APPEND_FOUND_MAPPOINTS: MapPoint " << pOMP->id_ << " added to map. ";
			}
		}
	}
}

void Osmap::rebuild(bool noSetBad){
	/*
	 * On every KeyFrame:
	 * - Builds the map database
	 * - UpdateConnections to rebuild covisibility graph
	 * - MapPoint::AddObservation on each point to rebuild MapPoint:observations_ y MapPoint:mObs
	 */
	cout << "Rebuilding map:" << endl;
	keyFrameDatabase.clear();

	if(noSetBad)
		options.set(NO_SET_BAD);

	log("Processing", vectorKeyFrames.size(), "keyframes");
	for(auto *pKF : vectorKeyFrames){
		LOGV(pKF);
		LOGV(pKF->id_);

		pKF->do_not_erase_ = !pKF->loop_edges_.empty();
		LOGV(pKF->do_not_erase_);

		// Build BoW vectors
		pKF->ComputeBoW();
		log("BoW computed");

		// Build many pose matrices
		pKF->SetPose(pKF->Tcw_);
		log("Pose set");

		/*
		 * Rebuilding grid.
		 * Code from Frame::AssignFeaturesToGrid()
		 */
		std::vector<std::size_t> grid[pKF->grid_cols_][pKF->grid_rows_];
		int nReserve = 0.5f*pKF->N_/(pKF->grid_cols_*pKF->grid_rows_);
		for(int i=0; i<pKF->grid_cols_;i++)
			for (int j=0; j<pKF->grid_rows_;j++)
				grid[i][j].reserve(nReserve);
		log("Grid built");

		for(int i=0;i<pKF->N_;i++){
			const cv::KeyPoint &kp = pKF->undistort_keypoints_[i];
			int posX = round((kp.pt.x-pKF->min_x_)*pKF->grid_element_width_inv_);
			int posY = round((kp.pt.y-pKF->min_y_)*pKF->grid_element_height_inv_);

			//Keypoint's coordinates are undistorted, which could cause to go out of the image
			if(!(posX<0 || posX>=pKF->grid_cols_ || posY<0 || posY>=pKF->grid_rows_))
				grid[posX][posY].push_back(i);
		}
		log("Grid full");

		pKF->grid_.resize(pKF->grid_cols_);
		for(int i=0; i < pKF->grid_cols_;i++){
			pKF->grid_[i].resize(pKF->grid_rows_);
			for(int j=0; j < pKF->grid_rows_; j++)
				pKF->grid_[i][j] = grid[i][j];
		}
		log("Grid fitted");

		// Append keyframe to the database
		keyFrameDatabase.add(pKF);

		// Rebuild MapPoints obvervations
		size_t n = pKF->map_points_.size();
		for(size_t i=0; i<n; i++){
			MapPoint *pMP = pKF->map_points_[i];
			if(pMP)
				pMP->AddObservation(pKF, i);
		}
		log("Observations rebuilt");

		// Calling UpdateConnections in id_ order rebuilds the covisibility graph and the spanning tree.
		pKF->UpdateConnections();
	}

	// Last KeyFrame's id
	map.max_keyframe_id_ = vectorKeyFrames.back()->id_;

	// Next KeyFrame id
	KeyFrame::next_id_ = map.max_keyframe_id_ + 1;

	// Retry on isolated keyframes
	for(auto *pKF : vectorKeyFrames)
		if(pKF->connected_keyframe_weights_.empty()){
			log("Isolated keyframe pKF:", pKF);
			pKF->UpdateConnections();
			if(!options[NO_SET_BAD] && pKF->connected_keyframe_weights_.empty() && pKF->id_){
				// If this keyframe is isolated (and it isn't keyframe zero), erase it.
				cerr << "Isolated keyframe " << pKF->id_ << " set bad." << endl;
				pKF->SetBadFlag();
			}
		}



	/*
	 * Check and fix the spanning tree created with UpdateConnections.
	 * Rebuilds the spanning tree asigning a parent_ to every orphan KeyFrame without, except that with id 0.
 	 * It ends when every KeyFrame has a parent.
	 */

	// keyframe_origins_ should be empty at this point, and must contain only one element, the first keyframe.
	map.keyframe_origins_.clear();
	map.keyframe_origins_.push_back(*vectorKeyFrames.begin());

	// Number of parents assigned in each iteration and in total.  Usually 0.
	int nParents = -1, nParentsTotal = 0;
	log("Rebuilding spanning tree.");
	while(nParents){
		nParents = 0;
		for(auto pKF: vectorKeyFrames)
			if(!pKF->parent_ && pKF->id_)	// Process all keyframes without parent, exccept id 0
				for(auto *pConnectedKF : pKF->ordered_connected_keyframes_){
					auto poConnectedKF = static_cast<OsmapKeyFrame*>(pConnectedKF);
					if(poConnectedKF->parent_ || poConnectedKF->id_ == 0){	// Parent found: not orphan or id 0
						nParents++;
						pKF->ChangeParent(pConnectedKF);
						break;
					}
				}
		nParentsTotal += nParents;
		log("Parents assigned in this loop:", nParents);
	}
	log("Parents assigned in total:", nParentsTotal);

	/*
	 * On every MapPoint:
	 * - Rebuilds reference_keyframe_ as the first observation, which should be the KeyFrame with the lowest id
	 * - Rebuilds many properties with UpdateNormalAndDepth()
	 */
	log("Processing", vectorMapPoints.size(), "mappoints.");
	for(OsmapMapPoint *pMP : vectorMapPoints){
		LOGV(pMP)
		LOGV(pMP->id_)
		// Rebuilds reference_keyframe_.  Requires observations_.
		if(!options[NO_SET_BAD] && pMP->id_ && pMP->observations_.empty()){
			cerr << "MP " << pMP->id_ << " without observations." << "  Set bad." << endl;
			pMP->SetBadFlag();
			continue;
		}

		// Asumes the first observation in mappoint has the lowest id_.  Processed keyframes in id_ order ensures this.
		auto pair = (*pMP->observations_.begin());
		pMP->reference_keyframe_ = pair.first;

		/* UpdateNormalAndDepth() requires prior rebuilding of reference_keyframe_, and rebuilds:
		 * - mNormalVector
		 * - mfMinDistance
		 * - mfMaxDistance
		 */
		pMP->UpdateNormalAndDepth();
	}
	MapPoint::next_id_ = vectorMapPoints.back()->id_ + 1;
}

void Osmap::getVectorKFromKeyframes(){
  vectorK.clear();
  keyframeid2vectorkIdx.resize(KeyFrame::next_id_);	// Assume map is not ill formed so next_id_ is ok, thus no keyframe's id is bigger than this.
  fill(keyframeid2vectorkIdx.begin(), keyframeid2vectorkIdx.end(), 0);	// Fill with index 0 to prevent segfault from unknown bugs.

  if(vectorKeyFrames.empty())
	  getKeyFramesFromMap();

  //for(auto &pKF:map.keyframes_){
  for(auto pKF: vectorKeyFrames){
    // Test if K can be found in vectorK.  If new, add it to the end of vectorK.
    //Mat &K = const_cast<cv::Mat &> (pKF->K_);
    const Mat &K = pKF->K_;

    // Will be the index of K in vectorK
    unsigned int i;
    for(i=0; i<vectorK.size(); i++){
      const Mat &vK = *vectorK[i];

      // Tests: break if found

      // Quick test
      if(K.data == vK.data) break;

      // Slow test, compare each element
/*
      if(
        K.at<float>(0,0) == vK.at<float>(0,0) &&
        K.at<float>(1,1) == vK.at<float>(1,1) &&
        K.at<float>(0,2) == vK.at<float>(0,2) &&
        K.at<float>(1,2) == vK.at<float>(1,2)
      ) break;
*/
#define DELTA 0.1
      if(
        abs(K.at<float>(0,0) - vK.at<float>(0,0)) < DELTA &&
        abs(K.at<float>(1,1) - vK.at<float>(1,1)) < DELTA &&
        abs(K.at<float>(0,2) - vK.at<float>(0,2)) < DELTA &&
        abs(K.at<float>(1,2) - vK.at<float>(1,2)) < DELTA
      ) break;
    }

    // if not found, push
    if(i>=vectorK.size()){
      // add new K
      vectorK.push_back(&K);
    }

    // i is the vectorK index for this keyframe
    keyframeid2vectorkIdx[ pKF->id_ ] = i;
  }
}

int Osmap::countFeatures(){
	int n=0;
	for(auto pKP : vectorKeyFrames)
		n += pKP->N_;

	return n;
}


// Utilities
MapPoint *Osmap::getMapPoint(unsigned int id){
  //for(auto pMP : map.map_points_)
  for(auto pMP : vectorMapPoints)
    if(pMP->id_ == id)
    	return pMP;

  // Not found
  return NULL;
}

OsmapKeyFrame *Osmap::getKeyFrame(unsigned int id){
  //for(auto it = map.keyframes_.begin(); it != map.keyframes_.end(); ++it)
  for(auto pKF : vectorKeyFrames)
	if(pKF->id_ == id)
	  return pKF;

  // If not found
  return NULL;
}




// K matrix ================================================================================================
void Osmap::serialize(const Mat &k, SerializedK *serializedK){
  serializedK->set_fx(k.at<float>(0,0));
  serializedK->set_fy(k.at<float>(1,1));
  serializedK->set_cx(k.at<float>(0,2));
  serializedK->set_cy(k.at<float>(1,2));
}

void Osmap::deserialize(const SerializedK &serializedK, Mat &m){
  m = Mat::eye(3,3,CV_32F);
  m.at<float>(0,0) = serializedK.fx();
  m.at<float>(1,1) = serializedK.fy();
  m.at<float>(0,2) = serializedK.cx();
  m.at<float>(1,2) = serializedK.cy();
}

void Osmap::serialize(const vector<Mat*> &vK, SerializedKArray &serializedKArray){

}

void Osmap::deserialize(const SerializedKArray &serializedKArray, vector<Mat*> &vK){

}



// Descriptor ================================================================================================
void Osmap::serialize(const Mat &m, SerializedDescriptor *serializedDescriptor){
  assert(m.rows == 1 && m.cols == 32);
  for(unsigned int i = 0; i<8; i++)
	serializedDescriptor->add_block(((unsigned int*)m.data)[i]);
}

void Osmap::deserialize(const SerializedDescriptor &serializedDescriptor, Mat &m){
  assert(serializedDescriptor.block_size() == 8);
  m = Mat(1, 32, CV_8UC1);
  for(unsigned int i = 0; i<8; i++)
	((unsigned int*)m.data)[i] = serializedDescriptor.block(i);
}

// Pose ================================================================================================
void Osmap::serialize(const Mat &m, SerializedPose *serializedPose){
  float *pElement = (float*) m.data;
  for(unsigned int i = 0; i<12; i++)
    serializedPose->add_element(pElement[i]);
}

void Osmap::serialize(const Eigen::Matrix4d& m, SerializedPose *serializedPose) {
	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++)
			serializedPose->add_element((float)m(i,j));
}

void Osmap::deserialize(const SerializedPose &serializedPose, Mat &m){
  assert(serializedPose.element_size() == 12);
  m = Mat::eye(4,4,CV_32F);
  float *pElement = (float*) m.data;
  for(unsigned int i = 0; i<12; i++)
	pElement[i] = serializedPose.element(i);
}

void Osmap::deserialize(const SerializedPose &serializedPose, Eigen::Matrix4d& m){
  assert(serializedPose.element_size() == 12);
  m = Eigen::Matrix4d::Zero();
	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++)
			m(i,j) = serializedPose.element(i*4+j);
}


// Position ================================================================================================
void Osmap::serialize(const Mat &m, SerializedPosition *serializedPosition){
  serializedPosition->set_x(m.at<float>(0,0));
  serializedPosition->set_y(m.at<float>(1,0));
  serializedPosition->set_z(m.at<float>(2,0));
}

void Osmap::serialize(const Eigen::Vector3d &m, SerializedPosition *serializedPosition){
  serializedPosition->set_x((float)m(0));
  serializedPosition->set_y((float)m(1));
  serializedPosition->set_z((float)m(2));
}

void Osmap::deserialize(const SerializedPosition &serializedPosition, Mat &m){
  m = Mat(3,1,CV_32F);
  m.at<float>(0,0) = serializedPosition.x();
  m.at<float>(1,0) = serializedPosition.y();
  m.at<float>(2,0) = serializedPosition.z();
}

void Osmap::deserialize(const SerializedPosition &serializedPosition, Eigen::Vector3d &m){
  m = Eigen::Vector3d::Zero();
  m(0) = serializedPosition.x();
  m(1) = serializedPosition.y();
  m(2) = serializedPosition.z();
}

// KeyPoint ================================================================================================
void Osmap::serialize(const KeyPoint &kp, SerializedKeypoint *serializedKeypoint){
  serializedKeypoint->set_ptx(kp.pt.x);
  serializedKeypoint->set_pty(kp.pt.y);
  serializedKeypoint->set_octave(kp.octave);
  serializedKeypoint->set_angle(kp.angle);
}

void Osmap::deserialize(const SerializedKeypoint &serializedKeypoint, KeyPoint &kp){
  kp.pt.x   = serializedKeypoint.ptx();
  kp.pt.y   = serializedKeypoint.pty();
  kp.octave = serializedKeypoint.octave();
  kp.angle  = serializedKeypoint.angle();
}



// MapPoint ================================================================================================
void Osmap::serialize(const OsmapMapPoint &mappoint, SerializedMappoint *serializedMappoint){
  serializedMappoint->set_id(mappoint.id_);
  serialize(mappoint.world_pose_, serializedMappoint->mutable_position());
  serializedMappoint->set_visible(mappoint.n_visible_);
  serializedMappoint->set_found(mappoint.n_found_);
  //if(options[NO_FEATURES_DESCRIPTORS])	// This is the only descriptor to serialize	** This line is disable to force mappoint descriptor serialization, while it's not being reconstructed in rebuild. **
    serialize(mappoint.descriptor_, serializedMappoint->mutable_briefdescriptor());
}

OsmapMapPoint *Osmap::deserialize(const SerializedMappoint &serializedMappoint){
  OsmapMapPoint *pMappoint = new OsmapMapPoint(this);

  pMappoint->id_        = serializedMappoint.id();
  pMappoint->n_visible_   = serializedMappoint.visible();
  pMappoint->n_found_     = serializedMappoint.found();
  if(serializedMappoint.has_briefdescriptor()) deserialize(serializedMappoint.briefdescriptor(), pMappoint->descriptor_);
  if(serializedMappoint.has_position())        deserialize(serializedMappoint.position(),        pMappoint->world_pose_  );

  return pMappoint;
}

int Osmap::serialize(const vector<OsmapMapPoint*>& vectorMP, SerializedMappointArray &serializedMappointArray){
  for(auto pMP : vectorMP)
    serialize(*pMP, serializedMappointArray.add_mappoint());

  return vectorMP.size();
}


int Osmap::deserialize(const SerializedMappointArray &serializedMappointArray, vector<OsmapMapPoint*>& vectorMapPoints){
  int i, n = serializedMappointArray.mappoint_size();
  for(i=0; i<n; i++)
	vectorMapPoints.push_back(deserialize(serializedMappointArray.mappoint(i)));

  return i;
}


// KeyFrame ================================================================================================
void Osmap::serialize(const OsmapKeyFrame &keyframe, SerializedKeyframe *serializedKeyframe){
  serializedKeyframe->set_id(keyframe.id_);
  serialize(keyframe.Tcw_, serializedKeyframe->mutable_pose());
  serializedKeyframe->set_timestamp(keyframe.timestamp_);
  if(options[K_IN_KEYFRAME])
	serialize(keyframe.K_, serializedKeyframe->mutable_kmatrix());
  else
	serializedKeyframe->set_kindex(keyframeid2vectorkIdx[keyframe.id_]);
  if(!keyframe.loop_edges_.empty())
	for(auto loopKF : keyframe.loop_edges_)
		// Only serialize id of keyframes already serialized, to easy deserialization.
		if(keyframe.id_ > loopKF->id_)
			serializedKeyframe->add_loopedgesids(loopKF->id_);
}

OsmapKeyFrame *Osmap::deserialize(const SerializedKeyframe &serializedKeyframe){
	OsmapKeyFrame *pKeyframe = new OsmapKeyFrame(this);

  pKeyframe->id_ = serializedKeyframe.id();
  const_cast<double&>(pKeyframe->timestamp_) = serializedKeyframe.timestamp();

  if(serializedKeyframe.has_pose())
	  deserialize(serializedKeyframe.pose(), pKeyframe->Tcw_);

  if(serializedKeyframe.has_kmatrix())
	  // serialized with K_IN_KEYFRAME option, doesn't use K list in yaml
	  deserialize(serializedKeyframe.kmatrix(), const_cast<cv::Mat&>(pKeyframe->K_));
  else
	  // serialized with default no K_IN_KEYFRAME option, K list in yaml
	  const_cast<cv::Mat&>(pKeyframe->K_) = *vectorK[serializedKeyframe.kindex()];

  if(serializedKeyframe.loopedgesids_size()){
	// Only ids of keyframes already deserialized and present on vectorKeyFrames
	for(int i=0; i<serializedKeyframe.loopedgesids_size(); i++){
	  unsigned int loopEdgeId = serializedKeyframe.loopedgesids(i);
	  OsmapKeyFrame *loopEdgeKF = getKeyFrame(loopEdgeId);
	  loopEdgeKF->loop_edges_.insert(pKeyframe);
	  pKeyframe->loop_edges_.insert(loopEdgeKF);
	}
  }

  return pKeyframe;
}

int Osmap::serialize(const vector<OsmapKeyFrame*>& vectorKF, SerializedKeyframeArray &serializedKeyframeArray){
  for(auto pKF: vectorKF)
    serialize(*pKF, serializedKeyframeArray.add_keyframe());

  return vectorKF.size();
}



int Osmap::deserialize(const SerializedKeyframeArray &serializedKeyframeArray, vector<OsmapKeyFrame*>& vectorKeyFrames){
  int i, n = serializedKeyframeArray.keyframe_size();
  for(i=0; i<n; i++)
	  vectorKeyFrames.push_back(deserialize(serializedKeyframeArray.keyframe(i)));

  return i;
}


// Feature ================================================================================================
void Osmap::serialize(const OsmapKeyFrame &keyframe, SerializedKeyframeFeatures *serializedKeyframeFeatures){
  serializedKeyframeFeatures->set_keyframe_id(keyframe.id_);
  for(int i=0; i<keyframe.N_; i++){
	if(!options[ONLY_MAPPOINTS_FEATURES] || keyframe.map_points_[i]){	// If chosen to only save mappoints features, check if there is a mappoint.
		SerializedFeature &serializedFeature = *serializedKeyframeFeatures->add_feature();

		// KeyPoint
		serialize(keyframe.undistort_keypoints_[i], serializedFeature.mutable_keypoint());

		// If there is a MapPoint, serialize it
		if(keyframe.map_points_[i])
		  serializedFeature.set_mappoint_id(keyframe.map_points_[i]->id_);

		// Serialize descriptor but skip if chosen to not do so.
		if(!options[NO_FEATURES_DESCRIPTORS])	//
		  serialize(keyframe.descriptors_.row(i), serializedFeature.mutable_briefdescriptor());
	}
  }
}

OsmapKeyFrame *Osmap::deserialize(const SerializedKeyframeFeatures &serializedKeyframeFeatures){
  unsigned int KFid = serializedKeyframeFeatures.keyframe_id();
  OsmapKeyFrame *pKF = getKeyFrame(KFid);
  if(pKF){
	  int n = serializedKeyframeFeatures.feature_size();
	  const_cast<int&>(pKF->N_) = n;
	  const_cast<std::vector<cv::KeyPoint>&>(pKF->undistort_keypoints_).resize(n);
	  pKF->map_points_.resize(n);
	  const_cast<cv::Mat&>(pKF->descriptors_) = Mat(n, 32, CV_8UC1);	// n descriptors

// ORB-SLAM2 needs to have set mvuRight and depthes_ even though they are not used in monocular.  DUMMY_MAP and OS1 don't have these properties.
#if !defined OSMAP_DUMMY_MAP && !defined OS1
	  const_cast<std::vector<float>&>(pKF->mvuRight) = vector<float>(n,-1.0f);
	  const_cast<std::vector<float>&>(pKF->depthes_) = vector<float>(n,-1.0f);
#endif
	  for(int i=0; i<n; i++){
		const SerializedFeature &feature = serializedKeyframeFeatures.feature(i);
		if(feature.mappoint_id())		  pKF->map_points_[i] = getMapPoint(feature.mappoint_id());
		if(feature.has_keypoint())    	  deserialize(feature.keypoint(), const_cast<cv::KeyPoint&>(pKF->undistort_keypoints_[i]));
		if(feature.has_briefdescriptor()){
			Mat descriptor;
			deserialize(feature.briefdescriptor(), descriptor);
			descriptor.copyTo(pKF->descriptors_.row(i));
		}
	  }
  } else {
	  cerr << "KeyFrame id "<< KFid << "not found while deserializing features: skipped.  Inconsistence between keyframes and features serialization files." << endl;
  }
  return pKF;
}


int Osmap::serialize(const vector<OsmapKeyFrame*> &vectorKF, SerializedKeyframeFeaturesArray &serializedKeyframeFeaturesArray){
  unsigned int nFeatures = 0;
  for(auto pKF:vectorKF){
    serialize(*pKF, serializedKeyframeFeaturesArray.add_feature());
    nFeatures += pKF->N_;
  }

  return nFeatures;
}


int Osmap::deserialize(const SerializedKeyframeFeaturesArray &serializedKeyframeFeaturesArray){
  int nFeatures = 0, i, n = serializedKeyframeFeaturesArray.feature_size();
  for(i=0; i<n; i++){
    KeyFrame *pKF=deserialize(serializedKeyframeFeaturesArray.feature(i));
	if(pKF)
		nFeatures += pKF->N_;
  }

  return nFeatures;
}



// Kendon Varda's code to serialize many messages in one file, from https://stackoverflow.com/questions/2340730/are-there-c-equivalents-for-the-protocol-buffers-delimited-i-o-functions-in-ja
// Returns false if error, true if ok.
bool Osmap::writeDelimitedTo(
    const google::protobuf::MessageLite& message,
    google::protobuf::io::ZeroCopyOutputStream* rawOutput
){
  // We create a new coded stream for each message.  Don't worry, this is fast.
  google::protobuf::io::CodedOutputStream output(rawOutput);

  // Write the size.
  const int size = message.ByteSize();
  output.WriteVarint32(size);
  uint8_t* buffer = output.GetDirectBufferForNBytesAndAdvance(size);
  if (buffer != NULL) {
    // Optimization:  The message fits in one buffer, so use the faster
    // direct-to-array serialization path.
    message.SerializeWithCachedSizesToArray(buffer);
  } else {
    // Slightly-slower path when the message is multiple buffers.
    message.SerializeWithCachedSizes(&output);
    if (output.HadError()){
      cerr << "Error in writeDelimitedTo." << endl;
      return false;
    }
  }
  return true;
}

bool Osmap::readDelimitedFrom(
    google::protobuf::io::ZeroCopyInputStream* rawInput,
    google::protobuf::MessageLite* message
){
  // We create a new coded stream for each message.  Don't worry, this is fast,
  // and it makes sure the 64MB total size limit is imposed per-message rather
  // than on the whole stream.  (See the CodedInputStream interface for more
  // info on this limit.)
  google::protobuf::io::CodedInputStream input(rawInput);

  // Read the size.
  uint32_t size;
  if (!input.ReadVarint32(&size)) return false;

  // Tell the stream not to read beyond that size.
  google::protobuf::io::CodedInputStream::Limit limit =
      input.PushLimit(size);

  // Parse the message.
  if (!message->MergeFromCodedStream(&input)) return false;
  if (!input.ConsumedEntireMessage()) return false;

  // Release the limit.
  input.PopLimit(limit);

  return true;
};


/*
 * Orbslam adapter.  Class wrappers.
 */
#ifndef OSMAP_DUMMY_MAP

OsmapMapPoint::OsmapMapPoint(Osmap *osmap):
	MapPoint(Eigen::Vector3d::Zero(), osmap->pRefKF, &osmap->map)
{};

OsmapKeyFrame::OsmapKeyFrame(Osmap *osmap):
	KeyFrame(osmap->currentFrame, &osmap->map, &osmap->keyFrameDatabase)
{};

#else

OsmapMapPoint::OsmapMapPoint(Osmap *osmap):
	MapPoint(osmap)
{};

OsmapKeyFrame::OsmapKeyFrame(Osmap *osmap):
	KeyFrame(osmap)
{};

#endif


}	// namespace ORB_SLAM2
