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
#include <ctime>

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
	setGravity(glm::vec3(0, -9.8, 0));


	//Move sun according to time
	time_t currentTime;
	struct tm *localTime;

	time(&currentTime);                   // Get the current time
	localTime = localtime(&currentTime);  // Convert the current time to the local time

	int Hour = localTime->tm_hour;
	int Min = localTime->tm_min;
	int Sec = localTime->tm_sec;

	if (Hour > 0 && Hour <= 6)
	{
		//Create the sun sphere - Morning 6am
		shared_ptr<PhysicsController> sun = physicsFactory->CreateSphere(50, glm::vec3(0, 0, -4000), glm::quat(), true, true);
		sun->transform->diffuse = glm::vec3(1, 1, 0);
		sun->transform->ambient = glm::vec3(1.0f, 1.0f, 0.0f);
	}else
		if (Hour > 6 && Hour <= 12)
		{
		//Create the sun sphere - Noon 12pm
		shared_ptr<PhysicsController> sun = physicsFactory->CreateSphere(50, glm::vec3(0, 4000, 0), glm::quat(), true, true);
		sun->transform->diffuse = glm::vec3(1, 1, 0);
		sun->transform->ambient = glm::vec3(1.0f, 1.0f, 0.0f);
		}
		else
			if (Hour > 12 && Hour <= 18)
			{
		//Create the sun sphere - Evening - 6pm
		shared_ptr<PhysicsController> sun = physicsFactory->CreateSphere(50, glm::vec3(0, 600, 4000), glm::quat(), true, true);
		sun->transform->diffuse = glm::vec3(1, 1, 0);
		sun->transform->ambient = glm::vec3(1.0f, 0.5f, 0.0f);
			}
			else
				if (Hour > 18)
				{
		//Create the moon sphere - Night 
		shared_ptr<PhysicsController> sun = physicsFactory->CreateSphere(50, glm::vec3(0, 2000, 0), glm::quat(), true, true);
		sun->transform->diffuse = glm::vec3(1, 1, 0);
		sun->transform->ambient = glm::vec3(0.7f, 0.7f, 0.7f);
				}








	//Declare sizes of rigid bodies
	float body_width = 15;
	float body_heigth = 2;
	float body_length = 4;
	float body_mass = 30;
	glm::vec3 position = glm::vec3(0, 3, 0);

	float leg_width = 1;
	float leg_heigth = 1;
	float leg_length = 3;

	float wheel_radius = 2.0f;
	float wheel_width = 1.0f;
	float wheel_offset = 3.0f;
	float wheel_mass = 5.0f;

	float shoulder_velocity = 10;
	float shoulder_scaler = -80;


	//Create a box for the body of the  thing
	shared_ptr<PhysicsController> bodyCube = physicsFactory->CreateBox(body_width, body_heigth, body_length, position, glm::quat());
	bodyCube->transform->diffuse = glm::vec3(0,0,0);
	//Set bodys Inertia
	btVector3 boxInertia(0, 0, 0);
	bodyCube->shape->calculateLocalInertia(body_mass, boxInertia);
	bodyCube->rigidBody->setMassProps(body_mass, boxInertia);

	//Create the seat at the back
	shared_ptr<PhysicsController> seat_back = physicsFactory->CreateBox(1, 6, 4, glm::vec3(4, 1.5f, 0), glm::quat());
	seat_back->transform->diffuse = glm::vec3(0.54f, 0, 0);
	btTransform seat_backTRAN, bodyTRAN;
	seat_backTRAN.setIdentity();
	bodyTRAN.setIdentity();

	seat_backTRAN.setOrigin(btVector3(0, 0, 0));
	bodyTRAN.setOrigin(btVector3(-6.9, 3.4f, 0));

	//Fix back to body
	btFixedConstraint * spine = new btFixedConstraint(*bodyCube->rigidBody, *seat_back->rigidBody, bodyTRAN, seat_backTRAN);
	dynamicsWorld->addConstraint(spine);

	//Create head
	shared_ptr<PhysicsController> head = physicsFactory->CreateFromModel("monkey", glm::vec3(1, 0, 0), glm::quat());
	////Attach head to front of body - HERE

	btTransform headTRAN;
	headTRAN.setIdentity();
	headTRAN.setOrigin(btVector3(0, 0, 0));
	bodyTRAN.setOrigin(btVector3(6.9, 3.4f, 0));
	//

	btFixedConstraint *neck = new btFixedConstraint(*bodyCube->rigidBody, *head->rigidBody, bodyTRAN, headTRAN);
	dynamicsWorld->addConstraint(neck);



	//Create front right leg
	shared_ptr<PhysicsController> frontRightLeg = physicsFactory->CreateBox(leg_length, leg_heigth, leg_width, glm::vec3(4.0f, 4, -3.5f), glm::angleAxis(45.0f, glm::vec3(0, 0, 1)));
	//Change colour
	frontRightLeg->transform->diffuse = glm::vec3(0.54f, 0, 0);
	//Create joint from body to leg
	btHingeConstraint * frontShoulderRight = new btHingeConstraint(*bodyCube->rigidBody, *frontRightLeg->rigidBody, btVector3(4, 0, -2.5), btVector3(1.5f, 0, 1), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	//Eanble the constraint
	dynamicsWorld->addConstraint(frontShoulderRight);


	if (bodyCube->transform->position.y < 5)
	{
		//Give power to the shoulder in the right direction
		frontShoulderRight->enableAngularMotor(true, shoulder_scaler, shoulder_velocity);
	}else
		frontShoulderRight->enableAngularMotor(false, shoulder_scaler, shoulder_velocity);

	//Create front right calf (bottom part of the leg
	shared_ptr<PhysicsController> frontRightCalf = physicsFactory->CreateBox(leg_length, leg_heigth, leg_width, glm::vec3(6.5f, 4, -3.5f), glm::angleAxis(-45.0f, glm::vec3(0, 0, 1)));
	frontRightCalf->transform->diffuse = glm::vec3(0.54f, 0, 0);
	//Setup transforms for right legs and calfs
	btTransform frontRightLegTRAN, frontRightCalfTRAN;
	//Set the identities of the right leg and calf
	frontRightLegTRAN.setIdentity();
	frontRightCalfTRAN.setIdentity();
	//Set the origin for the right leg and calf
	frontRightLegTRAN.setOrigin(btVector3(-2, 0, 0));
	frontRightCalfTRAN.setOrigin(btVector3(2, 0, 0));
	//Set the rotation of the leg and calf
	frontRightLegTRAN.setRotation(GLToBtQuat(glm::angleAxis(-45.0f, glm::vec3(0, 0, 1))));
	frontRightCalfTRAN.setRotation(GLToBtQuat(glm::angleAxis(45.0f, glm::vec3(0, 0, 1))));
	//Create the joint that will not allow movement between the calf and leg
	btFixedConstraint * frontRightKnee = new btFixedConstraint(*frontRightLeg->rigidBody, *frontRightCalf->rigidBody, frontRightLegTRAN, frontRightCalfTRAN);
	//Enable the constraint
	dynamicsWorld->addConstraint(frontRightKnee);

	//Create Left leg
	shared_ptr<PhysicsController> frontLeftLeg = physicsFactory->CreateBox(leg_length, leg_heigth, leg_width, glm::vec3(4.0f, 4, 3.5f), glm::angleAxis(45.0f, glm::vec3(0, 0, 1)));
	frontLeftLeg->transform->diffuse = glm::vec3(0.54f, 0, 0);
	//Left Shoulder
	btHingeConstraint * frontShoulderLeft = new btHingeConstraint(*bodyCube->rigidBody, *frontLeftLeg->rigidBody, btVector3(4, 0, 2.5), btVector3(1.5f, 0, -1), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	dynamicsWorld->addConstraint(frontShoulderLeft);
	
	if (bodyCube->transform->position.y < 5)
	{
		frontShoulderLeft->enableAngularMotor(true, shoulder_scaler, shoulder_velocity);
	}else
		frontShoulderLeft->enableAngularMotor(false, shoulder_scaler, shoulder_velocity);

	//Create Left Calf
	shared_ptr<PhysicsController> frontLeftCalf = physicsFactory->CreateBox(leg_length, leg_heigth, leg_width, glm::vec3(6.5f, 4, 3.5f), glm::angleAxis(-45.0f, glm::vec3(0, 0, 1)));
	frontLeftCalf->transform->diffuse = glm::vec3(0.54f, 0, 0);
	btTransform frontLeftLegTRAN, frontLeftCalfTRAN;

	frontLeftLegTRAN.setIdentity();
	frontLeftCalfTRAN.setIdentity();
	frontLeftLegTRAN.setOrigin(btVector3(-2, 0, 0));
	frontLeftCalfTRAN.setOrigin(btVector3(2, 0, 0));
	frontLeftLegTRAN.setRotation(GLToBtQuat(glm::angleAxis(-45.0f, glm::vec3(0, 0, 1))));
	frontLeftCalfTRAN.setRotation(GLToBtQuat(glm::angleAxis(45.0f, glm::vec3(0, 0, 1))));
	//Left Knee
	btFixedConstraint * frontLeftKnee = new btFixedConstraint(*frontLeftLeg->rigidBody, *frontLeftCalf->rigidBody, frontLeftLegTRAN, frontLeftCalfTRAN);
	dynamicsWorld->addConstraint(frontLeftKnee);


	//Create wheels for the back here
	shared_ptr<PhysicsController> wheel;
	glm::quat q = glm::angleAxis(90.0f, glm::vec3(1, 0, 0));
	glm::vec3 offset;
	btHingeConstraint * hinge;

	//Back wheels
	offset = glm::vec3(-(body_width / 2 - wheel_radius), 0, -(body_length / 2 + wheel_offset));
	wheel = physicsFactory->CreateCylinder(wheel_radius, wheel_width, position + offset, q);
	hinge = new btHingeConstraint(*bodyCube->rigidBody, *wheel->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge);
	wheel->transform->diffuse = glm::vec3(1, 1, 1);
	
	offset = glm::vec3(-(body_width / 2 - wheel_radius), 0, +(body_length / 2 + wheel_offset));
	wheel = physicsFactory->CreateCylinder(wheel_radius, wheel_width, glm::vec3(position.x + (body_width / 2) - wheel_radius, position.y, position.z - (body_length / 2) - wheel_width), q);
	hinge = new btHingeConstraint(*bodyCube->rigidBody, *wheel->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge);
	wheel->transform->diffuse = glm::vec3(1, 1, 1);

	//Set wheels Inertia
	btVector3 wheelInertia(0, 0, 0);
	wheel->shape->calculateLocalInertia(wheel_mass, wheelInertia);
	wheel->rigidBody->setMassProps(wheel_mass, wheelInertia);

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

