#include <OpenSim\OpenSim.h>
//#include <Simbody.h>
#include <OpenSim\Tools\InverseDynamicsTool.h>
#include <OpenSim\Simulation\SimbodyEngine\RollingOnSurfaceConstraint.h>
#include "ini\INIReader.h"
#include <string>
#include <iostream>

using namespace OpenSim;
using namespace std;

void addContactParameters(OpenSim::Model &model);
void addRollConstrains(OpenSim::Model &model);


/**
* Performs scale, inverse kinematics and inverse dynamics
* based on xml settings
*/
int main(){
	
	try {

		INIReader ini("settings.ini");
		string SCALE_SETTING = ini.Get("path", "SCALE_SETTING", "");
		//scale model
		ScaleTool scale(SCALE_SETTING);
		Model model = *scale.createModel();

		addContactParameters(model);
		//addRollConstrains(model);

		/*Model model(MODEL_PATH);
		MarkerSet markerSet(MARKERS_PATH);
		markerSet.connectMarkersToModel(model);
		model.updMarkerSet() = markerSet;*/
		
		//inverse kinematics
		//string IK_SETTING = ini.Get("path", "IK_SETTING", "");
		//InverseKinematicsTool ik(IK_SETTING);
		//ik.setModel(model);
		//ik.run();
	
		////inverse dynamics
		//string ID_SETTINGS = ini.Get("path", "ID_SETTINGS", "");
		//InverseDynamicsTool id(ID_SETTINGS);
		//id.setModel(model);
		//id.run();

		model.print("mod.osim");
		
	}catch (const std::exception& ex){
        std::cout << ex.what() << std::endl;
		std::cin.get();
        return 1;
    }catch (...){
        std::cout << "Unercognized Error" << std::endl;
		std::cin.get();
        return 1;
    }

    std::cout << "Finished" << std::endl;
	std::cin.get();
	return 0;
}

void addRollConstrains(OpenSim::Model &model){

	RollingOnSurfaceConstraint *rollRight = new RollingOnSurfaceConstraint();
	rollRight->setName("right_foot_contact");
	rollRight->setRollingBodyByName("calcn_r");
	rollRight->setSurfaceBodyByName("ground");
	rollRight->connectToModel(model);
	model.addConstraint(rollRight);

	RollingOnSurfaceConstraint *rollLeft = new RollingOnSurfaceConstraint();
	rollLeft->setName("left_foot_contact");
	rollLeft->setRollingBodyByName("calcn_l");
	rollRight->setSurfaceBodyByName("ground");
	rollLeft->connectToModel(model);
	model.addConstraint(rollLeft);
	
}


void addContactParameters(OpenSim::Model &model){
	INIReader ini("settings.ini");

	//get model body
	Body &ground = model.getGroundBody();
	BodySet bodySet = model.updBodySet();
	Body &calcn_r = bodySet.get(ini.Get("contact", "CALCN_R", "calcn_r"));
	Body &toes_r = bodySet.get(ini.Get("contact", "TOES_R", "toes_r"));
	Body &calcn_l = bodySet.get(ini.Get("contact", "CALCN_L", "calcn_l"));
	Body &toes_l = bodySet.get(ini.Get("contact", "TOES_L", "toes_l"));

	//crete ground reaction plate body
	Body *plate = new Body("groundReactionPlate", 0.001, 
		SimTK::Vec3(0),	SimTK::Inertia(0.001, 0.001, 0.001));
	SimTK::Vec3 locationInParent(0), orientationInParent(0), 
		locationInBody(0, ini.GetReal("contact", "PLATE_Y_OFFSET", 0), 0), orientationInBody(0);
	WeldJoint *groundPlate = new WeldJoint("groundPlate", ground, 
		locationInParent, orientationInParent, *plate, locationInBody, orientationInBody);
	model.addBody(plate);

	//contact platform plate
	ContactHalfSpace *grf = new ContactHalfSpace(SimTK::Vec3(0), 
		SimTK::Vec3(0, 0, -SimTK::Pi/2), *plate, "grf");
	grf->setDisplayPreference(4);
	model.addContactGeometry(grf);
	
	//contact right leg
	ContactSphere *heel_r = new ContactSphere(0.03, 
		SimTK::Vec3(0.01, 0.01, -0.005), calcn_r, "heel_r");
	model.addContactGeometry(heel_r);
	/*ContactSphere *big_toe_r = new ContactSphere(0.02, 
		SimTK::Vec3(-0.005, 0.005, -0.03), toes_r, "big_toe_r");
	model.addContactGeometry(big_toe_r);
	ContactSphere *big_toe2_r = new ContactSphere(0.015, 
		SimTK::Vec3(0.05, 0.005, 0.03), toes_r, "big_toe2_r");
	model.addContactGeometry(big_toe2_r);
	ContactSphere *small_toe_r = new ContactSphere(0.015, 
		SimTK::Vec3(-0.04, 0.005, 0.04), toes_r, "small_toe_r");
	model.addContactGeometry(small_toe_r);*/

	//contact left leg
	ContactSphere *heel_l = new ContactSphere(0.03, 
		SimTK::Vec3(0.01, 0.01, -0.005), calcn_l, "heel_l");
	model.addContactGeometry(heel_l);
	/*ContactSphere *big_toe_l = new ContactSphere(0.02, 
		SimTK::Vec3(-0.005, 0.005, 0.03), toes_l, "big_toe_l");
	model.addContactGeometry(big_toe_l);
	ContactSphere *big_toe2_l = new ContactSphere(0.015, 
		SimTK::Vec3(0.05, 0.005, -0.03), toes_l, "big_toe2_l");
	model.addContactGeometry(big_toe2_l);
	ContactSphere *small_toe_l = new ContactSphere(0.015, 
		SimTK::Vec3(-0.04, 0.005, -0.04), toes_l, "small_toe_l");
	model.addContactGeometry(small_toe_l);*/

	double stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction;
	stiffness = ini.GetReal("contact", "CONTACT_STIFFNESS", 1e8);
	dissipation = ini.GetReal("contact", "CONTACT_DISSIPATION", 0.5);
	staticFriction = ini.GetReal("contact", "CONTACT_STATIC_FRICTION", 0.9);
	dynamicFriction = ini.GetReal("contact", "CONTACT_DYNAMIC_FRICTION", 0.9);
	viscousFriction = ini.GetReal("contact", "CONTACT_VISCOUS_FRICTION", 0.6);

	//hunt crossley force for right foot
	HuntCrossleyForce::ContactParameters *rightFootParam = 
		new HuntCrossleyForce::ContactParameters(stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction);
	rightFootParam->append_geometry("grf");
	rightFootParam->append_geometry("heel_r");
	/*rightFootParam->append_geometry("big_toe_r");
	rightFootParam->append_geometry("big_toe2_r");
	rightFootParam->append_geometry("small_toe_r");*/
	HuntCrossleyForce *foot_rForce = new HuntCrossleyForce(rightFootParam);
	foot_rForce->setName("foot_r");
	model.addForce(foot_rForce);

	//hunt crossley force for left foot
	HuntCrossleyForce::ContactParameters *leftFootParam = 
		new HuntCrossleyForce::ContactParameters(stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction);
	leftFootParam->append_geometry("grf");
	leftFootParam->append_geometry("heel_l");
	/*leftFootParam->append_geometry("big_toe_l");
	leftFootParam->append_geometry("big_toe2_l");
	leftFootParam->append_geometry("small_toe_l");*/
	HuntCrossleyForce *foot_lForce = new HuntCrossleyForce(leftFootParam);
	foot_lForce->setName("foot_l");
	model.addForce(foot_lForce);
	
}