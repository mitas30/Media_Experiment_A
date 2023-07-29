#pragma once

#include <btBulletDynamicsCommon.h>

//プレイヤー、筆、ペン、色源、障害物
enum CollisionGroup {
	RX_COL_GROUND = 0b000001,
	RX_COL_PLAYER = 0b000010,
	RX_COL_PENCIL = 0b000100,
	RX_COL_SOURCE = 0b001000,
	RX_COL_OBST   = 0b010000,
	RX_COL_CP     = 0b100000,
	RX_COL_ALL = 0b111111
};

btRigidBody* CreateRigidBody(double mass, const btTransform& init_trans, btCollisionShape* shape, btDynamicsWorld* world = 0, int* index = 0, int group=RX_COL_OBST, int mask=RX_COL_ALL, bool inertia = false,btVector3 &setinertia=btVector3(0,0,0));
btRigidBody* AddBoxBodies(double mass, btVector3& translation, btVector3& size, btQuaternion& qrot = btQuaternion(0, 0, 0, 1), int index = 2, double friction = 0.95, double restitution = 0.3);
void CreateBrush(btVector3 initpos);
btRigidBody* AddGround(btVector3& pos,btVector3& size,double friction = 0.80, double restitution = 0.3);
btRigidBody* AddWall(btVector3& position, btVector3& size, int index2 = 2, btQuaternion& qrot = btQuaternion(btVector3(1, 0, 0), 0));
void AddPointConstraintToSpace(btRigidBody* A,btVector3 pivot);
void AddHingeToSpace(btRigidBody* A);
void RotateObstacle(btRigidBody* obstacle, btScalar torque); 
void RespawnPlayer(void);
void RespawnBrush(void);
void AddCheckFlag(btVector3 &pos,int stage,btVector3& wind_velo);
void CreateCharacter(btVector3 p_respawn);
void MoveSkyBox(void);
void AddMovingSkyBox(double mass, btVector3& translation, btVector3& size, btVector3& min_move, btVector3& max_move);