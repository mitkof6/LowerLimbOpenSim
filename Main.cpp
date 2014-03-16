#include <OpenSim\OpenSim.h>
//#include <Simbody.h>
#include <OpenSim\Tools\InverseDynamicsTool.h>

#include "Constants.h"

using namespace OpenSim;
using namespace SimTK;

/**
* Performs scale, inverse kinematics and inverse dynamics
* based on xml settings
*/
int main(){
	
	try {

		//scale model
		ScaleTool scale(SCALE_SETTING);
		Model model = *scale.createModel();

		/*Model model(MODEL_PATH);
		MarkerSet markerSet(MARKERS_PATH);
		markerSet.connectMarkersToModel(model);
		model.updMarkerSet() = markerSet;*/
		
		//inverse kinematics
		InverseKinematicsTool ik(IK_SETTING);
		ik.setModel(model);
		ik.run();

		//inverse dynamics
		InverseDynamicsTool id(ID_SETTINGS);
		id.setModel(model);
		id.run();
		
	}catch (const std::exception& ex){
        std::cout << ex.what() << std::endl;
		std::cin.get();
        return 1;
    }catch (...){
        std::cout << "Unercognized Error" << std::endl;
		std::cin.get();
        return 1;
    }

    
	std::cin.get();
	return 0;
}
