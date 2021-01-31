#include "aligner.h"
#include "pair_icp_aligner.h"
Aligner* Aligner::get_aligner(const std::string& name) {
    //TODO NDT
    return new PairICPAligner();
}