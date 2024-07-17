//includeはcppにまとめよう
#include <random>
#include <iostream>
#include "make_stage.hpp"
#include "make_gimmick.hpp"
#include "utils.h"

using namespace std;

#define M_PI 3.1415926535

extern btRigidBody* rotate_rigid;
extern btSoftRigidDynamicsWorld* g_dynamicsworld;
extern btAlignedObjectArray<btCollisionShape*>	g_collisionshapes;
extern btRigidBody* cp_rigid;
btVector3 p_respawn;
btVector3 b_respawn;
btVector3 min_viable_area;
btVector3 max_viable_area;
btScalar max_stage_x = 20;
btScalar max_stage_z = 20;
btScalar delta = 1.2;
btScalar half_wall_height = 5;
btScalar half_wall_width = 10;

void MakeStage1(void)
{
	min_viable_area = btVector3(-(max_stage_x + delta), -delta,-delta-2);
	max_viable_area = btVector3(delta, half_wall_height * 2, (max_stage_z + delta));
	//Tutorial 1/4　ステージごとにp_respawnを変更する
	AddGround(btVector3(-max_stage_x/2,- 0.3, max_stage_z/2-1), btVector3(max_stage_x/2, 0.3, max_stage_z/2+1));
	btVector3 translations[5] = { btVector3(-max_stage_x,half_wall_height,max_stage_z/2), btVector3(0,half_wall_height,max_stage_z/2), btVector3(-max_stage_x/2,half_wall_height,max_stage_z),btVector3(-max_stage_x / 6, half_wall_height, 0), btVector3(-max_stage_x / 6 * 5, half_wall_height, 0) };
	btVector3 sizes[5] = { btVector3(0.1,half_wall_height,half_wall_width), btVector3(0.1,half_wall_height,half_wall_width) , btVector3(half_wall_width,half_wall_height,0.1),btVector3(max_stage_x / 6,half_wall_height, 0.1),btVector3(max_stage_x / 6,half_wall_height,0.1)};
	btQuaternion rotations[5];
	int colors[5] = { 7,8,97,10,10};
	for (int i = 0; i < 5; i++)
		AddWall(translations[i], sizes[i], colors[i]); 
	btVector3 position = btVector3(-max_stage_x/2,half_wall_height, 0);
	btVector3 size = btVector3(max_stage_x / 6-0.1, half_wall_height, 0.1);
	btQuaternion rotation = btQuaternion(btVector3(1, 0, 0), 0);
	rotate_rigid=AddBoxBodies(200, position, size, rotation,2);
	AddHingeToSpace(rotate_rigid);
	RotateObstacle(rotate_rigid,10000);
	p_respawn = btVector3(-max_stage_x / 2, half_wall_height / 5, 4 * max_stage_z / 5);
	b_respawn = btVector3(-max_stage_x / 2, 1, 6 * max_stage_z / 10);
	CreateCharacter(p_respawn);
	CreateBrush(b_respawn);
	AddCheckFlag(btVector3(-15,0.5,-1),1, btVector3(20.0, 0.0, -20.0));
	random_device rd;
	mt19937 gen(rd());
	uniform_real_distribution<> dis(-19, 19);
	for (int i = 1; i <= 10; ++i) {
		double x = dis(gen);
		while (x >= -1) {
			x = dis(gen);
		}
		double z = dis(gen);
		while (z <= 4) {
			z = dis(gen);
		}
		AddBoxBodies(1, btVector3(x, -0.2, z), btVector3(0.4, 0.4, 0.4), btQuaternion(0, 0, 0, 1), i);
	}
}

void MakeStage2(void)
{
	max_stage_z = 14;
	delta = 2.4;
	min_viable_area = btVector3(-(max_stage_x + delta), -delta, -(max_stage_z+delta));
	max_viable_area = btVector3(max_stage_x + delta, half_wall_height * 2,max_stage_z + delta);
	AddGround(btVector3(-15, -0.3, -1), btVector3(5, 0.3, 1));
	AddGround(btVector3(-15, -0.3, -8), btVector3(5, 0.3,6));
	AddGround(btVector3(-7, -0.7, -10), btVector3(3.0, 0.1, 4.0));
	AddGround(btVector3(-1.5,-0.7, -10), btVector3(1.5, 0.1, 4.0));
	AddGround(btVector3(-1, -0.3, 13), btVector3(3, 0.3, 1));
	AddGround(btVector3(-11, -0.7, 13), btVector3(7.0, 0.1, 1.0));
	AddGround(btVector3(-19, -0.7, 13), btVector3(1.0, 0.1, 1.0));
	AddWall(btVector3(-20.3, half_wall_height, -1), btVector3(0.3, half_wall_height, 1),4);
	AddWall(btVector3(-20.3, half_wall_height, -4), btVector3(0.3, half_wall_height, 2), 8);
	AddWall(btVector3(-20.3, half_wall_height, -8), btVector3(0.3, half_wall_height, 2), 7);
	AddWall(btVector3(-20.3, half_wall_height, -12), btVector3(0.3, half_wall_height, 2), 5);
	AddWall(btVector3(-9.7, half_wall_height, -3), btVector3(0.3, half_wall_height, 3), 9);
	AddWall(btVector3(2.3, half_wall_height, -7), btVector3(0.3, half_wall_height, 7), 3);
	AddWall(btVector3(2.3, half_wall_height,3.5), btVector3(0.3, half_wall_height, 3.5), 9);
	AddWall(btVector3(2.3, half_wall_height,10.5), btVector3(0.3, half_wall_height, 3.5), 3);
	//ジャンプの高さはだいたい0.8
	AddBoxBodies(100, btVector3(-12.5, 0.9, -4), btVector3(0.5,0.9,0.5));
	AddBoxBodies(100, btVector3(-17.5, 2.25, -12), btVector3(2.5,2.25,2.5),btQuaternion(0,0,0,1),7);
	AddBoxBodies(100, btVector3(-17.5, 1.125, -4), btVector3(0.5,1.125, 0.4));
	AddBoxBodies(100, btVector3(-12.5,0.36, -12), btVector3(0.5,1.8, 0.5), btQuaternion(0, 0, 0, 1),8);
	AddBoxBodies(1000, btVector3(-7,-0.3, -10), btVector3(3.0,0.3,4.0));
	AddBoxBodies(1000, btVector3(-1.5,0.9, -10), btVector3(1.5,1.5, 4.0));
	AddBoxBodies(1000, btVector3(0.0,1.0,13), btVector3(2.0,1.0,1.0));
	AddBoxBodies(0, btVector3(-3.0, 1.0, 13), btVector3(sqrt(2.0)-0.1,0.1, 1.0),btQuaternion(btVector3(0,0,1),btRadians(45.0)));
	AddBoxBodies(1000, btVector3(-11, -0.3, 13), btVector3(7.0, 0.3, 1.0));
	AddBoxBodies(1000, btVector3(-19, -0.3, 13), btVector3(1.0, 0.3, 1.0), btQuaternion(0, 0, 0, 1),3);
	AddBoxBodies(0, btVector3(-11,1.2, 13), btVector3(1.0, 0.3, 1.0), btQuaternion(0, 0, 0, 1), 2);
	AddBoxBodies(0, btVector3(-15,0.9, 13), btVector3(1.0, 0.1, 1.0), btQuaternion(0, 0, 0, 1), 2);
	p_respawn = btVector3(-15, 0.5, -1);
	b_respawn = btVector3(-15, 1.5, -1);
	AddMovingSkyBox(10, btVector3(1.0, 2.3, -10), btVector3(4, 0.1, 0.5), btVector3(0, 0, -14), btVector3(0, 0, 10));
	AddBoxBodies(0, btVector3(-11,half_wall_height-0.8, 14.3), btVector3(7.0, half_wall_height, 0.3), btQuaternion(0,0,0,1),1);
	AddBoxBodies(0, btVector3(-11,half_wall_height-0.8, 11.7), btVector3(7.0, half_wall_height, 0.3), btQuaternion(0, 0, 0, 1), 1);
	CreateCharacter(p_respawn);
	CreateBrush(b_respawn);
	AddCheckFlag(btVector3(-19.5, 1.0, 13),2,btVector3(-20.0,0.0,-20.0));
}

void MakeStage3(void)
{
	max_stage_z = 14;
	delta = 2.4;
	min_viable_area = btVector3(-(max_stage_x + delta), -delta, -(max_stage_z + delta));
	max_viable_area = btVector3(max_stage_x + delta, half_wall_height * 2, max_stage_z + delta);
	p_respawn = btVector3(-19.5,1.0,13);
	b_respawn = btVector3(-19.5,1.0,13);
	CreateCharacter(p_respawn);
	CreateBrush(b_respawn);
	AddGround(btVector3(-max_stage_x / 2, -0.3, max_stage_z / 2 - 1), btVector3(max_stage_x / 2, 0.3, max_stage_z / 2 + 1));
}

