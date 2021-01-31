#include <iostream>
#include "aligner.h"
#include "pair_icp_aligner.h"

Aligner* Aligner::get_aligner(const std::string& name) {
    if(name == ICP_NAME) {
        return new PairICPAligner();
    }
    //std::cout<<"nothing found"<<std::endl;
    return nullptr;
}