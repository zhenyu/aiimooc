
#include "aligner.h"

#ifndef _ZHENYU_SHA_AIIMOOC_PAIR_ICP_ALIANER_H
#define _ZHENYU_SHA_AIIMOOC_PAIR_ICP_ALIANER_H
class PairICPAligner : public Aligner
{
public:
  virtual void align(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample) override;
};
#endif