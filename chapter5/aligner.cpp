#include <iostream>

#include "aligner.h"
#include "pair_icp_aligner.h"
#include "ndt_aligner.h"

Aligner *Aligner::get_aligner(const std::string &name)
{
    if (name == ICP_NAME)
    {
        return new PairICPAligner();
    }
    else if (name == NDT_NAME)
    {
        return new NDTAligner();
    }
    //std::cout<<"nothing found"<<std::endl;
    return nullptr;
}