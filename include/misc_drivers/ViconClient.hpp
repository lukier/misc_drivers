/**
 * ****************************************************************************
 * Copyright (c) 2016, Robert Lukierski.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * ****************************************************************************
 * Vicon Client Wrapper.
 * ****************************************************************************
 */
#ifndef VICON_CLIENT_HPP
#define VICON_CLIENT_HPP

#include <cstdint>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <map>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <sophus/se3.hpp>

#ifdef MISC_DRIVERS_HAVE_CEREAL
#include <cereal/cereal.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>
#endif // MISC_DRIVERS_HAVE_CEREAL

#ifdef MISC_DRIVERS_HAVE_CAMERA_MODELS
#include <CameraModels/PinholeDisparityDistorted.hpp>
#endif // MISC_DRIVERS_HAVE_CAMERA_MODELS

namespace drivers
{
 
namespace sensor
{

class ViconPacket
{
public:
    struct Subject
    {
        struct Segment
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            
            std::string               Name;
            std::string               ParentName;
            std::vector<std::string>  Children;
            
            Eigen::Vector3d           StaticTranslation;            
            Eigen::Vector3d           StaticEulerXYZ;
            Eigen::Quaternion<double> StaticQuaternion;
            
            Eigen::Vector3d           GlobalTranslation;
            bool                      GlobalTranslationOccluded;
            Eigen::Vector3d           GlobalEulerXYZ;
            bool                      GlobalEulerXYZOccluded;
            Eigen::Quaternion<double> GlobalQuaternion;
            bool                      GlobalQuaternionOccluded;
            
            Eigen::Vector3d           LocalTranslation;
            bool                      LocalTranslationOccluded;
            Eigen::Vector3d           LocalEulerXYZ;
            bool                      LocalEulerXYZOccluded;
            Eigen::Quaternion<double> LocalQuaternion;
            bool                      LocalQuaternionOccluded;
            
#ifdef MISC_DRIVERS_HAVE_CEREAL
            template<class Archive>
            void serialize(Archive& archive)
            {
                archive(CEREAL_NVP(Name));
                archive(CEREAL_NVP(ParentName));
                archive(CEREAL_NVP(Children));
                
                archive(cereal::make_nvp("StaticTranslation", StaticTranslation));
                archive(cereal::make_nvp("StaticEuler", StaticEulerXYZ));
                archive(cereal::make_nvp("StaticQuaternion", StaticQuaternion));
                
                archive(cereal::make_nvp("GlobalTranslation", GlobalTranslation));
                archive(CEREAL_NVP(GlobalTranslationOccluded));
                archive(cereal::make_nvp("GlobalEuler", GlobalEulerXYZ));
                archive(CEREAL_NVP(GlobalEulerXYZOccluded));                
                archive(cereal::make_nvp("GlobalQuaternion", GlobalQuaternion));
                archive(CEREAL_NVP(GlobalQuaternionOccluded));
                
                archive(cereal::make_nvp("LocalTranslation", LocalTranslation));               
                archive(CEREAL_NVP(LocalTranslationOccluded));                
                archive(cereal::make_nvp("LocalEuler", LocalEulerXYZ));
                archive(CEREAL_NVP(LocalEulerXYZOccluded));                
                archive(cereal::make_nvp("LocalQuaternion", LocalQuaternion));
                archive(CEREAL_NVP(LocalQuaternionOccluded));
            }
#endif // MISC_DRIVERS_HAVE_CEREAL
        };
        
        struct Marker
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            
            std::string              Name;
            std::string              ParentName;
            Eigen::Vector3d          GlobalTranslation;
            bool                     Occluded;
            
#ifdef MISC_DRIVERS_HAVE_CEREAL
            template<class Archive>
            void serialize(Archive& archive)
            {
                archive(CEREAL_NVP(Name));
                archive(CEREAL_NVP(ParentName));
                archive(cereal::make_nvp("GlobalTranslation", GlobalTranslation));
                archive(CEREAL_NVP(Occluded));
            }
#endif // MISC_DRIVERS_HAVE_CEREAL
        };
        
        std::string                                                             Name;
        std::string                                                             RootSegment;
        std::vector<Segment, Eigen::aligned_allocator<Segment>>                 Segments;
        std::vector<Marker, Eigen::aligned_allocator<Marker>>                   Markers;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> UnlabelledMarkers;
        
#ifdef MISC_DRIVERS_HAVE_CEREAL
        template<class Archive>
        void serialize(Archive& archive)
        {
            archive(CEREAL_NVP(Name));
            archive(CEREAL_NVP(RootSegment));
            archive(CEREAL_NVP(Segments));
            archive(CEREAL_NVP(Markers));
            archive(CEREAL_NVP(UnlabelledMarkers));
        }
#endif // MISC_DRIVERS_HAVE_CEREAL
    };
    
    struct Camera
    {
        struct Centroid
        {
            double X;
            double Y;
            double Radius;
            
#ifdef MISC_DRIVERS_HAVE_CEREAL
            template<class Archive>
            void serialize(Archive& archive)
            {
                archive(CEREAL_NVP(X));
                archive(CEREAL_NVP(Y));
                archive(CEREAL_NVP(Radius));
            }
#endif // MISC_DRIVERS_HAVE_CEREAL
        };
        
        std::string              Name;
        std::vector<Centroid>    Centroids;
        
#ifdef MISC_DRIVERS_HAVE_CEREAL
        template<class Archive>
        void serialize(Archive& archive)
        {
            archive(CEREAL_NVP(Name));
            archive(CEREAL_NVP(Centroids));
        }
#endif // MISC_DRIVERS_HAVE_CEREAL
    };
    
    uint64_t                     FrameNumber;
    double                       Latency;
    double                       FramerateHz;
    std::vector<Subject>         Subjects;
    std::vector<Camera>          Centroids;
    uint64_t                     PCTimestamp;
    uint64_t                     PCTimestampEnd;
    
#ifdef MISC_DRIVERS_HAVE_CEREAL
    template<class Archive>
    void serialize(Archive& archive)
    {
        archive(CEREAL_NVP(FrameNumber));
        archive(CEREAL_NVP(Latency));
        archive(CEREAL_NVP(FramerateHz));
        archive(CEREAL_NVP(Subjects));
        archive(CEREAL_NVP(Centroids));
        archive(CEREAL_NVP(PCTimestamp));
        archive(CEREAL_NVP(PCTimestampEnd));
    }
#endif // MISC_DRIVERS_HAVE_CEREAL
};

template<typename T>
struct ViconCompactObject
{
    typedef Sophus::SE3<T>  PoseT;
    typedef Eigen::Matrix<T,3,1> PointT;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PoseT Pose;
    std::vector<PointT, Eigen::aligned_allocator<PointT>> Markers;
    std::vector<std::string>                              MarkerNames;
    
#ifdef MISC_DRIVERS_HAVE_CEREAL
    template<class Archive>
    void serialize(Archive& archive)
    {
        archive(cereal::make_nvp("GlobalPose", Pose));
        archive(CEREAL_NVP(Markers));
        archive(CEREAL_NVP(MarkerNames));
    }
#endif // MISC_DRIVERS_HAVE_CEREAL
};

/**
 * Main receiving class.
 */
class ViconClient
{
public:
    struct ViconPimpl;
    
    ViconClient(bool rc = false);
    virtual ~ViconClient();
    
    void open(const std::string& hostname);
    void close();
    bool isOpened();
    
    std::string getVersion();
    
    bool waitForFrame(ViconPacket& packet);
private:
    std::unique_ptr<ViconPimpl> pimpl;
    bool read_centroids;
};

// TODO FIXME these derived should be elsewhere perhaps
template<typename T>
struct ViconCompactPacket
{
    typedef ViconCompactObject<T> ObjectT;
    
    uint64_t                                     FrameNumber;
    double                                       Latency;
    double                                       FramerateHz;
    uint64_t                                     PCTimestamp;
    uint64_t                                     PCTimestampEnd;
    std::map<std::string, ObjectT>               Objects;
    
    ViconCompactPacket() = default;
    ViconCompactPacket(const ViconCompactPacket& src) = default;
    ViconCompactPacket(ViconCompactPacket&& src) = default;
    ViconCompactPacket& operator=(const ViconCompactPacket& other) = default;
    ViconCompactPacket& operator=(ViconCompactPacket&& other) = default;
    
    ViconCompactPacket(const ViconPacket& src)
    {
        convert(src);
    }
    
    void convert(const ViconPacket& src)
    {
        FrameNumber = src.FrameNumber;
        Latency = src.Latency;
        FramerateHz = src.FramerateHz;
        PCTimestamp = src.PCTimestamp;
        PCTimestampEnd = src.PCTimestampEnd;
        
        Objects.clear();
        
        for(const auto& sub : src.Subjects)
        {
            if((sub.Segments[0].GlobalQuaternionOccluded == false) && (sub.Segments[0].GlobalTranslationOccluded == false))
            {
                Sophus::SE3d pose_d(Sophus::SO3d(sub.Segments[0].GlobalQuaternion), sub.Segments[0].GlobalTranslation);
                Objects[sub.Name].Pose = pose_d.cast<T>();
                for(const auto& m : sub.UnlabelledMarkers)
                {
                    Objects[sub.Name].Markers.push_back(m.cast<T>());
                    Objects[sub.Name].MarkerNames.push_back("");
                }
                for(const auto& m : sub.Markers)
                {
                    if(!m.Occluded)
                    {    
                        Objects[sub.Name].Markers.push_back(m.GlobalTranslation.cast<T>());
                        Objects[sub.Name].MarkerNames.push_back(m.Name);
                    }
                }
            }
        }
    }
    
    #ifdef MISC_DRIVERS_HAVE_CEREAL
    template<class Archive>
    void serialize(Archive& archive)
    {
        archive(CEREAL_NVP(FrameNumber));
        archive(CEREAL_NVP(Latency));
        archive(CEREAL_NVP(FramerateHz));
        archive(CEREAL_NVP(PCTimestamp));
        archive(CEREAL_NVP(PCTimestampEnd));
        archive(CEREAL_NVP(Objects));
    }
    #endif // MISC_DRIVERS_HAVE_CEREAL
};

template<typename CameraT, typename Scalar>
struct ViconCameraWithPose
{
    typedef Sophus::SE3<Scalar> PoseT;
    typedef CameraT ModelT;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    int CameraID;
    PoseT Pose;
    CameraT Model;
    
#ifdef MISC_DRIVERS_HAVE_CEREAL
    template <typename Archive>
    void serialize(Archive & archive)
    { 
        archive(cereal::make_nvp("CameraID", CameraID));
        archive(cereal::make_nvp("Pose", Pose));
        archive(cereal::make_nvp("Model", Model));
    }
#endif // MISC_DRIVERS_HAVE_CEREAL
};

#ifdef MISC_DRIVERS_HAVE_CAMERA_MODELS
typedef ViconCameraWithPose<cammod::PinholeDisparityDistorted<double>, double> NamedViconCameraT;
bool ViconCalibrationFileParse(const std::string& fn, std::vector<NamedViconCameraT>& out_cams);
#endif // MISC_DRIVERS_HAVE_CAMERA_MODELS
    
}

}

#endif // VICON_CLIENT_HPP
