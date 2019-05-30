/*
 * This file is part of VO
 * created by ZhaoXinwen
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef MAPPOINT_H
#define MAPPOINT_H
#include <myslam/common_include.h>
namespace myslam
{
  class Frame;
class MapPoint 
{
public:
    typedef shared_ptr<MapPoint> Ptr;
    // 该场景点的id号
    unsigned long      id_; 
    static unsigned long factory_id_;
    bool        good_; 
    // 该场景点的三维位置
    Vector3d    pos_; 
    // 
    Vector3d    norm_;      
    // 描述子
    Mat         descriptor_; 
    // 
    list<Frame*>    observed_frames_;
    int         matched_times_;
    int         visible_times_;  
    MapPoint();
    MapPoint( unsigned long id, 
        const Vector3d& position, 
        const Vector3d& norm, 
        Frame* frame=nullptr, 
        const Mat& descriptor=Mat()  );
    inline cv::Point3f getPositionCV() const 
    {
        return cv::Point3f( pos_(0,0), pos_(1,0), pos_(2,0) );
    }
    static MapPoint::Ptr createMapPoint();
    static MapPoint::Ptr createMapPoint( 
        const Vector3d& pos_world, 
        const Vector3d& norm_,
        const Mat& descriptor,
        Frame* frame );
    
    
};
}
#endif