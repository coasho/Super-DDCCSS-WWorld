#pragma once
#include "contracts/world_stream/FusionFrameV1.h"
#include <string>
namespace fsim::ui {
void drawFusionV1(const ipc::FusionRecordV1& record,bool available,const std::string& name);
}
