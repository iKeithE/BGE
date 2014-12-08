#include "Assignment.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"



using namespace BGE;
Assignment::Assignment()
{
}


Assignment::~Assignment()
{
}

bool Assignment::Initialise()
{
	//Call physics for the camera and the ground from the physicsFactory
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	//Set the gravity
	setGravity(glm::vec3(0, 0, 0));

	//Create a sphere for the body of the spider
	//shared_ptr<PhysicsController> bodySphere = physicsFactory->CreateSphere(3, glm::vec3(0, 0, 0), glm::quat());
	shared_ptr<PhysicsController> bodyCube = physicsFactory->CreateBox(6, 2, 4, glm::vec3(0, 3, 0), glm::quat());


	//Create front right leg
	shared_ptr<PhysicsController> frontRightLeg = physicsFactory->CreateBox(3.0f, 1.0f, 1.0f,  glm::vec3(3.0f, 4, -3.5f), glm::angleAxis(45.0f, glm::vec3(0, 0, 1)));
	//frontLegLeft->transform->diffuse = (glm::vec3(0, 0, 0));

	btHingeConstraint * frontShoulderRight = new btHingeConstraint(*bodyCube->rigidBody, *frontRightLeg->rigidBody, btVector3(2, 0, -2.5), btVector3(1.5f, 0, 1), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	dynamicsWorld->addConstraint(frontShoulderRight);

	//Create front right calf
	shared_ptr<PhysicsController> frontRightCalf = physicsFactory->CreateBox(3.0f, 1.0f, 1.0f, glm::vec3(6.5f, 4, -3.5f), glm::angleAxis(-45.0f, glm::vec3(0, 0, 1)));
	
	
	
	btFixedConstraint * frontRightKnee = new btFixedConstraint(*frontRightLeg->rigidBody, *frontRightCalf->rigidBody, 



	//Make body black
	//bodySphere->transform->diffuse = (glm::vec3(0,0,0));

	//Create front two legs - there will need a hinge between the 2 joints of each leg
	//																			r  , h,  position           , angle offset degrees,     axis to rotate on
	/*shared_ptr<PhysicsController> frontLegLeft = physicsFactory->CreateCylinder(0.2f, 5, glm::vec3(2.0f, 0.0f, 2.0f), glm::angleAxis(-135.0f, glm::vec3(1, 0, 0)));
	frontLegLeft->transform->diffuse = (glm::vec3(0, 0, 0));*/

	////Join leg to body using a hinge																				sphere attach pos		leg attach pos
	//btHingeConstraint * frontShoulderLeft = new btHingeConstraint(*bodySphere->rigidBody, *frontLegLeft->rigidBody, btVector3(-0.5f, 1, 3), btVector3(-2.5f, 0, 0), btVector3(0, 0, 0), btVector3(0, 1, 0), true);

	////Below works from body
	//btHingeConstraint * frontShoulderLeft = new btHingeConstraint(*bodySphere->rigidBody, *frontLegLeft->rigidBody, btVector3(2, 4, 3), btVector3(0, 3, 0), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	//dynamicsWorld->addConstraint(frontShoulderLeft);




	//If the Initialise wasn't called, return false, if it was , true.
	if (!Game::Initialise()) {
		return false;
	}
	return true;

}

void BGE::Assignment::Update()
{
	Game::Update();
}

