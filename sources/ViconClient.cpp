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
#include <misc_drivers/ViconClient.hpp>

#include <map>
#include <chrono>

#include <Client.h>

#ifdef MISC_DRIVERS_HAVE_TINYXML
#include <tinyxml2.h>
#endif // MISC_DRIVERS_HAVE_TINYXML

struct drivers::sensor::ViconClient::ViconPimpl
{
    ViconPimpl() 
    { 
        
    }
    
    ViconDataStreamSDK::CPP::Client cli;
};

drivers::sensor::ViconClient::ViconClient(bool rc) : pimpl(new ViconPimpl()), read_centroids(rc)
{

}

drivers::sensor::ViconClient::~ViconClient()
{

}

void drivers::sensor::ViconClient::open(const std::string& hostname)
{
    if(isOpened()) { return; }
    
    if(pimpl->cli.Connect(hostname).Result != ViconDataStreamSDK::CPP::Result::Success)
    {
        throw std::runtime_error("Cannot connect");
    }
    
    // Enable some different data types
    pimpl->cli.EnableSegmentData();
    pimpl->cli.EnableMarkerData();
    pimpl->cli.EnableUnlabeledMarkerData();
    pimpl->cli.EnableDeviceData();
    if(read_centroids)
    {
        pimpl->cli.EnableCentroidData();
    }
    
    // Set the streaming mode
    pimpl->cli.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);
  
//#if 0
    // Set the global up axis - default 
    ViconDataStreamSDK::CPP::Output_SetAxisMapping res = 
    pimpl->cli.SetAxisMapping(ViconDataStreamSDK::CPP::Direction::Forward, 
                              ViconDataStreamSDK::CPP::Direction::Left, 
                              ViconDataStreamSDK::CPP::Direction::Up ); // Z-up
//#endif
#if 0
    // Set the global up axis - opengl
    ViconDataStreamSDK::CPP::Output_SetAxisMapping res =
    pimpl->cli.SetAxisMapping(ViconDataStreamSDK::CPP::Direction::Forward, 
                              ViconDataStreamSDK::CPP::Direction::Up, 
                              ViconDataStreamSDK::CPP::Direction::Right ); // Y-up
#endif

    if(res.Result != ViconDataStreamSDK::CPP::Result::Success)
    {
        throw std::runtime_error("Cannot Set Coordinate System");
    }
}

void drivers::sensor::ViconClient::close()
{
    if(!isOpened()) { return; }
    
    pimpl->cli.DisableSegmentData();
    pimpl->cli.DisableMarkerData();
    pimpl->cli.DisableUnlabeledMarkerData();
    pimpl->cli.DisableDeviceData();
    if(read_centroids)
    {
        pimpl->cli.DisableCentroidData();
    }
    
    // Disconnect and dispose
    pimpl->cli.Disconnect();
}

bool drivers::sensor::ViconClient::isOpened()
{
    return pimpl->cli.IsConnected().Connected;
}

std::string drivers::sensor::ViconClient::getVersion()
{
    std::stringstream ss;
    ss << pimpl->cli.GetVersion().Major << "." << pimpl->cli.GetVersion().Minor << "." << pimpl->cli.GetVersion().Point;
    return ss.str();
}

bool drivers::sensor::ViconClient::waitForFrame(drivers::sensor::ViconPacket& packet)
{
    if(pimpl->cli.GetFrame().Result != ViconDataStreamSDK::CPP::Result::Success)
    {
        return false;
    }
    
    packet.PCTimestamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    
    packet.FrameNumber = pimpl->cli.GetFrameNumber().FrameNumber;
    packet.Latency = pimpl->cli.GetLatencyTotal().Total;
    packet.FramerateHz = pimpl->cli.GetFrameRate().FrameRateHz;
    
    unsigned int scount = pimpl->cli.GetSubjectCount().SubjectCount;
    packet.Subjects.reserve(scount);
    packet.Subjects.clear();
    
    for(unsigned int sidx = 0 ; sidx < scount ; ++sidx)
    {
        ViconPacket::Subject su;
        
        su.Name = pimpl->cli.GetSubjectName(sidx).SubjectName;
        su.RootSegment = pimpl->cli.GetSubjectRootSegmentName(su.Name).SegmentName;
        
        unsigned int segcnt = pimpl->cli.GetSegmentCount(su.Name).SegmentCount;
        su.Segments.reserve(segcnt);
        su.Segments.clear();
        
        for(unsigned int sgidx = 0 ; sgidx < segcnt ; ++sgidx)
        {
            ViconPacket::Subject::Segment sg;
            
            sg.Name = pimpl->cli.GetSegmentName(su.Name, sgidx).SegmentName;
            sg.ParentName = pimpl->cli.GetSegmentParentName(su.Name, sg.Name).SegmentName;
            unsigned int childcnt = pimpl->cli.GetSegmentChildCount(su.Name, sg.Name).SegmentCount;
            for(unsigned int childidx = 0 ; childidx < childcnt ; ++childidx)
            {
                sg.Children.push_back(pimpl->cli.GetSegmentChildName(su.Name, sg.Name, childidx).SegmentName);
            }
            
            // -- static
            
            auto tmp1 = pimpl->cli.GetSegmentStaticTranslation(su.Name, sg.Name);
            sg.StaticTranslation << tmp1.Translation[0] , tmp1.Translation[1] , tmp1.Translation[2];
            
            auto tmp2 = pimpl->cli.GetSegmentStaticRotationEulerXYZ(su.Name, sg.Name);
            sg.StaticEulerXYZ << tmp2.Rotation[0] , tmp2.Rotation[1] , tmp2.Rotation[2];
            
            auto tmp3 = pimpl->cli.GetSegmentStaticRotationQuaternion(su.Name, sg.Name);
            sg.StaticQuaternion = Eigen::Quaternion<double>(tmp3.Rotation[3], tmp3.Rotation[0], tmp3.Rotation[1], tmp3.Rotation[2]); // W, X, Y, Z
            
            // -- local
            
            auto tmp4 = pimpl->cli.GetSegmentLocalTranslation(su.Name, sg.Name);
            sg.LocalTranslation << tmp4.Translation[0] , tmp4.Translation[1] , tmp4.Translation[2];
            sg.LocalTranslationOccluded = tmp4.Occluded;
            
            auto tmp5 = pimpl->cli.GetSegmentLocalRotationEulerXYZ(su.Name, sg.Name);
            sg.LocalEulerXYZ << tmp5.Rotation[0] , tmp5.Rotation[1] , tmp5.Rotation[2];
            sg.LocalEulerXYZOccluded = tmp5.Occluded;
            
            auto tmp6 = pimpl->cli.GetSegmentLocalRotationQuaternion(su.Name, sg.Name);
            sg.LocalQuaternion = Eigen::Quaternion<double>(tmp6.Rotation[3], tmp6.Rotation[0], tmp6.Rotation[1], tmp6.Rotation[2]); // W, X, Y, Z
            sg.LocalQuaternionOccluded = tmp6.Occluded;
            
            // -- global
            
            auto tmp7 = pimpl->cli.GetSegmentGlobalTranslation(su.Name, sg.Name);
            sg.GlobalTranslation << tmp7.Translation[0] , tmp7.Translation[1] , tmp7.Translation[2];
            sg.GlobalTranslationOccluded = tmp7.Occluded;
            
            auto tmp8 = pimpl->cli.GetSegmentGlobalRotationEulerXYZ(su.Name, sg.Name);
            sg.GlobalEulerXYZ << tmp8.Rotation[0] , tmp8.Rotation[1] , tmp8.Rotation[2];
            sg.GlobalEulerXYZOccluded = tmp8.Occluded;
            
            auto tmp9 = pimpl->cli.GetSegmentGlobalRotationQuaternion(su.Name, sg.Name);
            sg.GlobalQuaternion = Eigen::Quaternion<double>(tmp9.Rotation[3], tmp9.Rotation[0], tmp9.Rotation[1], tmp9.Rotation[2]); // W, X, Y, Z
            sg.GlobalQuaternionOccluded = tmp9.Occluded;
            
            su.Segments.push_back(sg);
        }
        
        unsigned int markcnt = pimpl->cli.GetMarkerCount(su.Name).MarkerCount;
        su.Markers.reserve(markcnt);
        su.Markers.clear();
        
        for(unsigned int midx = 0 ; midx < markcnt ; ++midx)
        {
            ViconPacket::Subject::Marker mrk;
            
            mrk.Name = pimpl->cli.GetMarkerName(su.Name, midx).MarkerName;
            mrk.ParentName = pimpl->cli.GetMarkerParentName(su.Name, mrk.Name).SegmentName;
            auto tmp = pimpl->cli.GetMarkerGlobalTranslation(su.Name, mrk.Name);
            mrk.GlobalTranslation << tmp.Translation[0] , tmp.Translation[1] , tmp.Translation[2];
            mrk.Occluded = tmp.Occluded;
            
            su.Markers.push_back(mrk);
        }
        
        unsigned int unmarkcnt = pimpl->cli.GetUnlabeledMarkerCount().MarkerCount;
        su.UnlabelledMarkers.reserve(unmarkcnt);
        su.UnlabelledMarkers.clear();
        
        for(unsigned int midx = 0 ; midx < unmarkcnt ; ++midx)
        {
            auto tmp = pimpl->cli.GetUnlabeledMarkerGlobalTranslation(midx);
            Eigen::Vector3d pos(tmp.Translation[0], tmp.Translation[1], tmp.Translation[2]);
            su.UnlabelledMarkers.push_back(pos);
        }
        
        packet.Subjects.push_back(su);
    }
    
    packet.PCTimestampEnd = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    
    return true;
}

#ifdef MISC_DRIVERS_HAVE_TINYXML

bool drivers::sensor::ViconCalibrationFileParse(const std::string& fn, std::vector< drivers::sensor::NamedViconCameraT >& out_cams)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError err = doc.LoadFile(fn.c_str());
    if(err != tinyxml2::XMLError::XML_SUCCESS)
    {
        return false;
    }
    
    tinyxml2::XMLElement* root = doc.FirstChildElement( "Cameras" );
    for (tinyxml2::XMLElement* camera = root->FirstChildElement(); camera != NULL; camera = camera->NextSiblingElement())
    {
        int gotvars = 0;
        
        double sens_x = 1024.0, sens_y = 1024.0;
        int cam_id = 0;
        double focal = 700.0;
        double orient_x = 0.0, orient_y = 0.0, orient_z = 0.0, orient_w = 0.0;
        double pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;
        double princ_u = 0.0, princ_v = 0.0;
        double radial_k1 = 0.0, radial_k2 = 0.0;
        
        if(camera->Attribute("SENSOR_SIZE") != nullptr)
        {
            std::stringstream ss(std::string(camera->Attribute("SENSOR_SIZE")));
            ss >> sens_x >> sens_y;
            ++gotvars;
        }
        
        if(camera->Attribute("USERID") != nullptr)
        {
            std::stringstream ss(std::string(camera->Attribute("USERID")));
            ss >> cam_id;
            ++cam_id;
            ++gotvars;
        }
        
        if(camera->FirstChildElement( "KeyFrames" ) != nullptr) 
        {
            tinyxml2::XMLElement* params = camera->FirstChildElement( "KeyFrames" )->FirstChildElement( "KeyFrame" );
            
            if(params != nullptr) 
            {
                if(params->Attribute("FOCAL_LENGTH") != nullptr)
                {
                    std::stringstream ss(std::string(params->Attribute("FOCAL_LENGTH")));
                    ss >> focal;
                    ++gotvars;
                }
                
                if(params->Attribute("ORIENTATION") != nullptr)
                {
                    std::stringstream ss(std::string(params->Attribute("ORIENTATION")));
                    ss >> orient_x >> orient_y >> orient_z >> orient_w;
                    ++gotvars;
                }
                
                if(params->Attribute("POSITION") != nullptr)
                {
                    std::stringstream ss(std::string(params->Attribute("POSITION")));
                    ss >> pos_x >> pos_y >> pos_z;
                    ++gotvars;
                }
                
                if(params->Attribute("PRINCIPAL_POINT") != nullptr)
                {
                    std::stringstream ss(std::string(params->Attribute("PRINCIPAL_POINT")));
                    ss >> princ_u >> princ_v;
                    ++gotvars;
                }
                
                if(params->Attribute("VICON_RADIAL") != nullptr)
                {
                    std::stringstream ss(std::string(params->Attribute("VICON_RADIAL")));
                    ss >> radial_k1 >> radial_k2;
                    ++gotvars;
                }
            }
        }
        
        if(gotvars == 7)
        {
            NamedViconCameraT camout;
            camout.CameraID = cam_id;
            camout.Model = NamedViconCameraT::ModelT(focal, focal, princ_u, princ_v, radial_k1, radial_k2, 0.0, 0.0, 0.0, sens_x, sens_y);
            camout.Pose = NamedViconCameraT::PoseT(Eigen::Quaternion<NamedViconCameraT::PoseT::Scalar>( orient_w , orient_x , orient_y , orient_z ), 
                                                   Eigen::Matrix<NamedViconCameraT::PoseT::Scalar,3,1>(pos_x, pos_y, pos_z));
            out_cams.push_back(camout);
        }
    }
    
    return true;
}

#endif // MISC_DRIVERS_HAVE_TINYXML
