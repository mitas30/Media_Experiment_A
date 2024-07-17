#include "make_gimmick.hpp"
#include "utils.h"
#include "rx_obj.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"

extern btSoftRigidDynamicsWorld* g_dynamicsworld;
extern btAlignedObjectArray<btCollisionShape*>	g_collisionshapes;
extern btSoftBodyWorldInfo g_softBodyWorldInfo;
btRigidBody* player_rigid[8]{};
btRigidBody* brush_rigid;
extern btScalar half_wall_height;
extern btScalar half_wall_width;
extern btScalar max_stage_x;
extern btScalar max_stage_z;
extern btVector3 p_respawn;
extern btVector3 b_respawn;
btRigidBody* cp_rigid;
btRigidBody* move_rigid;

extern btScalar fric_coef;
extern btScalar mass_coef;
extern btScalar restitution_coef;
extern btScalar expand_coef;
extern btScalar reduce_coef;
extern btScalar incr_inertia_coef;
extern btScalar decr_inertia_coef;
btVector3 move_min_move;
btVector3 move_max_move;


#define INF 10000000

/*!
* Bullet剛体(btRigidBody)の作成
* @param[in] mass 質量
* @param[in] init_tras 初期位置・姿勢
* @param[in] shape 形状
* @return 作成したbtRigidBody
*/
btRigidBody* CreateRigidBody(double mass, const btTransform& init_trans, btCollisionShape* shape, btDynamicsWorld* world, int* index, int group, int mask, bool inertia, btVector3& setinertia)
{
	// 質量が0ならば静的な(static)オブジェクトとして設定，
	bool isDynamic = (mass != 0.0);

	btVector3 calc_inertia(0, 0, 0);
	if (isDynamic && !inertia)
	{
		if (setinertia.length() == 0)
		{
			shape->calculateLocalInertia(mass, calc_inertia);
			if (*index == 9)
				calc_inertia *= incr_inertia_coef;
			else if (*index == 10)
				calc_inertia *= decr_inertia_coef;
		}
		else
			calc_inertia = setinertia;
	}
	else
		calc_inertia = btVector3(INF, INF, INF);

	btDefaultMotionState* motion_state = new btDefaultMotionState(init_trans);

	btRigidBody::btRigidBodyConstructionInfo rb_info(mass, motion_state, shape, calc_inertia);

	btRigidBody* body = new btRigidBody(rb_info);

	body->setUserIndex(*index);

	if (mass <= 1e-10) {
		// Kinematicオブジェクトとして設定(stepSimulationしても運動の計算を行わないようにする)
		body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
		// 常にスリープ状態にする
		body->setActivationState(DISABLE_DEACTIVATION);
	}

	if (world) {
		world->addRigidBody(body, group, mask);
	}

	return body;
}

/*!
*　box_shapeの追加 普通のギミック、変更可能な床など
*/
btRigidBody* AddBoxBodies(double mass, btVector3& translation, btVector3& size, btQuaternion& qrot, int index, double friction, double restitution)
{
	btRigidBody* body1;
	btCollisionShape* box_shape;
	//Transformは位置と姿勢
	btTransform trans;	// 剛体オブジェクトの位置姿勢を格納する変数(行列)
	trans.setIdentity();// 位置姿勢行列の初期化
	// 形状設定
	if (index == 7)
		box_shape = new btBoxShape(size * expand_coef);
	else if (index == 8)
		box_shape = new btBoxShape(size * reduce_coef);
	else
		box_shape = new btBoxShape(size);
	g_collisionshapes.push_back(box_shape); // 最後に破棄(delete)するために形状データを格納しておく

	trans.setOrigin(translation);
	trans.setRotation(qrot);	// 四元数を行列に変換して姿勢行列に掛け合わせる

	// 剛体オブジェクト生成
	if (index == 4)
		body1 = CreateRigidBody(mass * mass_coef, trans, box_shape, g_dynamicsworld, &index, RX_COL_OBST, RX_COL_ALL);
	else
		body1 = CreateRigidBody(mass, trans, box_shape, g_dynamicsworld, &index, RX_COL_OBST, RX_COL_ALL);
	//プロパティ設定
	if (index == 3)
		body1->setFriction(friction * fric_coef);
	else
		body1->setFriction(friction);
	if (index == 5)
		body1->setRestitution(restitution * restitution_coef);
	else
		body1->setRestitution(restitution);
	return body1;
}


void CreateBrush(btVector3 initpos)
{
	//変数定義
	double mass = 0.05;
	int brush_index = 98;

	btCompoundShape* brushShape = new btCompoundShape();
	btTransform transform;
	//毛を作成
	btScalar cone_r = 0.2;
	btScalar cone_h = 1.0;
	transform.setIdentity();
	transform.setOrigin(btVector3(0, 0, -0.2)); // move head up
	btConeShape* hairShape = new btConeShapeZ(cone_r, cone_h);
	brushShape->addChildShape(transform, hairShape);
	//持ち手を作成
	btVector3* cylinderInfo = new btVector3(0.2, 0.5, 0.2);
	transform.setIdentity();
	transform.setOrigin(btVector3(0, 0, 0.5));
	btCylinderShape* handShape = new btCylinderShapeZ(*cylinderInfo);
	brushShape->addChildShape(transform, handShape);

	// 剛体オブジェクト生成
	transform.setOrigin(initpos);
	transform.setRotation(btQuaternion(btVector3(1, 0, 0), 0));
	btVector3 inertia(100, 1000, 100);
	btDefaultMotionState* motion_state = new btDefaultMotionState(transform);
	btRigidBody::btRigidBodyConstructionInfo rb_info(mass, motion_state, brushShape, inertia);

	brush_rigid = new btRigidBody(rb_info);

	brush_rigid->setUserIndex(brush_index);
	brush_rigid->setUserIndex2(2);
	if (g_dynamicsworld) {
		g_dynamicsworld->addRigidBody(brush_rigid, RX_COL_PENCIL, RX_COL_ALL & (~RX_COL_PLAYER) & (~RX_COL_CP));
	}
	// すり抜け防止用Swept sphereの設定(CCD:Continuous Collision Detection)
	brush_rigid->setCcdMotionThreshold(0.2);
	brush_rigid->setCcdSweptSphereRadius(0.05 * 0.2);

}

void SetConstraint(btGeneric6DofConstraint* con)
{
	con->setLinearUpperLimit(btVector3(0, 0, 0));
	con->setAngularUpperLimit(btVector3(0, 0, 0));
	g_dynamicsworld->addConstraint(con);
}

//Z方向のみ Timer関数に搭載 move_rigidはグローバル変数に
//Constraintで縛ればよさそう
void MoveSkyBox(void)
{
	btScalar force_coef = 100;
	bool force_dir = true;
	btVector3 rigid_origin = move_rigid->getCenterOfMassTransform().getOrigin();
	if (rigid_origin.z() < move_min_move.z())
		force_dir = true;
	else if (move_max_move.x() < rigid_origin.z())
		force_dir = false;
	if (force_dir)
		move_rigid->applyCentralForce(force_coef * btVector3(0, 0, 1));
	else
		move_rigid->applyCentralForce(force_coef * btVector3(0, 0, -1));
}

//初期作成は白(2)のみ
void AddMovingSkyBox(double mass, btVector3& translation, btVector3& size, btVector3& min_move, btVector3& max_move)
{
	move_min_move = min_move;
	move_max_move = max_move;
	btCollisionShape* box_shape;
	//Transformは位置と姿勢
	btTransform trans;	// 剛体オブジェクトの位置姿勢を格納する変数(行列)
	trans.setIdentity();// 位置姿勢行列の初期化
	box_shape = new btBoxShape(size);
	g_collisionshapes.push_back(box_shape); // 最後に破棄(delete)するために形状データを格納しておく

	trans.setOrigin(translation);
	trans.setRotation(btQuaternion(btVector3(0,1,0),btRadians(90)));
	int index = 2;
	move_rigid = CreateRigidBody(mass, trans, box_shape, g_dynamicsworld, &index, RX_COL_OBST, RX_COL_ALL);
	trans.setIdentity();
	btGeneric6DofConstraint* slider = new btGeneric6DofConstraint(*move_rigid,trans,true);
	slider->setLinearLowerLimit(btVector3(-100, 0,0));
	slider->setLinearUpperLimit(btVector3(100, 0,0));
	slider->setAngularLowerLimit(btVector3(-RX_PI / 3, -RX_PI / 3, -RX_PI / 3));
	slider->setAngularLowerLimit(btVector3(RX_PI / 3, RX_PI / 3, RX_PI / 3));
	g_dynamicsworld->addConstraint(slider);
	move_rigid->setFriction(0.95);
	move_rigid->setRestitution(0.3);
}



void CreateCharacter(btVector3 p_respawn)
{
	double mass = 10;
	int index = 0;
	btTransform trans;
	btQuaternion qrot = btQuaternion(btVector3(1, 0, 0), 0);

	struct PartInfo {
		btScalar radius;
		btScalar height;
		btVector3 offset;
		btQuaternion rotation;
	};

	PartInfo partsInfo[] = {
		{0.1, 0.3, btVector3(0, 0, 0), btQuaternion(btVector3(1, 0, 0), 0)},  // Body
		{0.15, 0, btVector3(0, 0.3, 0), btQuaternion(btVector3(1, 0, 0), 0)},  // Head
		{0.05, 0.4, btVector3(-0.05, -0.3, 0), btQuaternion(btVector3(1, 0, 0), 0)},  // Left Leg
		{0.05, 0.4, btVector3(0.05, -0.3, 0), btQuaternion(btVector3(1, 0, 0), 0)},  // Right Leg
		{0.05, 0.18, btVector3(-0.11, 0.05, 0), btQuaternion(0, 0, RX_PI * 2 / 3)},  // Left Shoulder
		{0.05, 0.18, btVector3(0.11, 0.05, 0), btQuaternion(0, 0, -RX_PI * 2 / 3)},  // Right Shoulder
		{0.05, 0.3, btVector3(-0.15, -0.05, 0), btQuaternion(btVector3(1, 0, 0), 0)},  // Left Hand
		{0.05, 0.3, btVector3(0.15, -0.05, 0), btQuaternion(btVector3(1, 0, 0), 0)}  // Right Hand
	};

	btRigidBody* body;
	btVector3 offset_base;
	btQuaternion rotation_base;
	for (int i = 0; i < 8; i++) {
		auto& info = partsInfo[i];
		auto shape = info.height > 0 ? static_cast<btCollisionShape*>(new btCapsuleShape(info.radius, info.height)) : new btSphereShape(info.radius);
		trans.setIdentity();
		trans.setOrigin(p_respawn + info.offset);
		trans.setRotation(info.rotation);
		body = CreateRigidBody(mass, trans, shape, g_dynamicsworld, &index, RX_COL_PLAYER, RX_COL_ALL & (~RX_COL_CP) & (~RX_COL_PENCIL) & (~RX_COL_PLAYER), 1);
		body->setRestitution(0.4);
		body->setFriction(0.8);
		if (i == 0) {
			player_rigid[0] = body;
			offset_base = info.offset;
			rotation_base = info.rotation;
		}
		else {
			player_rigid[i] = body;
			btTransform trans_a, trans_b;
			trans_a.setIdentity();
			trans_b.setIdentity();
			trans_a.setOrigin(info.offset);
			trans_b.setOrigin(offset_base);
			trans_a.setRotation(info.rotation);
			trans_b.setRotation(rotation_base);
			btGeneric6DofConstraint* constraint = new btGeneric6DofConstraint(*player_rigid[0], *body, trans_a, trans_b, true);
			if (i == 7)
			{
				constraint->setLinearUpperLimit(btVector3(0, 0, 0));
				g_dynamicsworld->addConstraint(constraint);
			}
			else
				SetConstraint(constraint);
		}
		mass = 0.5; //1/20
	}
}

//変更不可な黒色(97の割当)
btRigidBody* AddGround(btVector3& pos, btVector3& size, double friction, double restitution)
{
	btTransform trans;	// 剛体オブジェクトの位置姿勢を格納する変数(行列)
	trans.setIdentity();// 位置姿勢行列の初期化
	const btScalar CUBE_HALF_EXTENTS = 0.2;	// 立方体の変の長さの半分(中心から辺までの距離)
	// 形状設定
	btCollisionShape* ground = new btBoxShape(size);
	g_collisionshapes.push_back(ground); // 最後に破棄(delete)するために形状データを格納しておく

	// 初期位置・姿勢
	btQuaternion qrot(0, 0, 0, 1);
	trans.setIdentity();// 位置姿勢行列の初期化
	trans.setOrigin(pos);
	trans.setRotation(qrot);	// 四元数を行列に変換して姿勢行列に掛け合わせる

	// 剛体オブジェクト生成
	int index = 97;
	btRigidBody* body1 = CreateRigidBody(0.0, trans, ground, g_dynamicsworld, &index, RX_COL_GROUND, RX_COL_ALL);
	body1->setFriction(friction);
	body1->setRestitution(restitution);

	// すり抜け防止用Swept sphereの設定(CCD:Continuous Collision Detection)
	body1->setCcdMotionThreshold(CUBE_HALF_EXTENTS);
	body1->setCcdSweptSphereRadius(0.05 * CUBE_HALF_EXTENTS);
	return body1;
}

//色源の1つ(indexは99)
btRigidBody* AddWall(btVector3& position, btVector3& size, int index2, btQuaternion& qrot)
{
	btTransform trans;	// 剛体オブジェクトの位置姿勢を格納する変数(行列)
	trans.setIdentity();// 位置姿勢行列の初期化
	// 衝突 planeは、collisionと見た目でどちらも回転があるので注意
	btCollisionShape* wall = new btBoxShape(size);
	g_collisionshapes.push_back(wall); // 最後に破棄(delete)するために形状データを格納しておく

	// 初期位置・姿勢(見た目)
	trans.setIdentity();// 位置姿勢行列の初期化
	trans.setOrigin(position);
	trans.setRotation(qrot);

	// 剛体オブジェクト生成
	int index = 99;
	btRigidBody* body1 = CreateRigidBody(0.0, trans, wall, g_dynamicsworld, &index, RX_COL_SOURCE, RX_COL_ALL);
	body1->setUserIndex2(index2);
	return body1;
}

void AddPointConstraintToSpace(btRigidBody* A, btVector3 pivot)
{
	btPoint2PointConstraint* p_constraint = new btPoint2PointConstraint(*A, pivot);
	g_dynamicsworld->addConstraint(p_constraint);
}

void AddHingeToSpace(btRigidBody* A)
{
	btHingeConstraint* y_rotate = new btHingeConstraint(*A, btVector3(0, 0, 0), btVector3(0, 1, 0));
	g_dynamicsworld->addConstraint(y_rotate);
}

void RotateObstacle(btRigidBody* obstacle, btScalar torque)
{
	obstacle->applyTorqueImpulse(btVector3(0, torque, 0));
}

void RespawnPlayer(void)
{
	btTransform trans;
	for (int i = 0; i < 8; i++)
	{
		player_rigid[i]->setAngularVelocity(btVector3(0, 0, 0));
		player_rigid[i]->setLinearVelocity(btVector3(0, 0, 0));
		trans.setIdentity();
		trans.setOrigin(p_respawn);
		if (i == 4 || i == 5)
			trans.setRotation(btQuaternion(0, 0, -1 * (6 - i) * RX_PI * 2 / 3));
		else
			trans.setRotation(btQuaternion(btVector3(1, 0, 0), 0));
		player_rigid[i]->setCenterOfMassTransform(trans);
	}
}

void RespawnBrush(void)
{
	btTransform trans;
	brush_rigid->setAngularVelocity(btVector3(0, 0, 0));
	brush_rigid->setLinearVelocity(btVector3(0, 0, 0));
	trans.setIdentity();
	trans.setOrigin(b_respawn);
	trans.setRotation(btQuaternion(btVector3(1, 0, 0), 0));
	brush_rigid->setCenterOfMassTransform(trans);
}

//flagは静止するように flagとrodの位置を揃えること
void AddCheckFlag(btVector3& pos, int stage,btVector3& wind_velo)
{
	btScalar x = pos.x();
	btScalar y1 = pos.y() + 1.0;
	btScalar y2 = pos.y() + 0.5;
	btScalar z1 = pos.z() - 0.5;
	btScalar z2 = pos.z();
	int res = 5;
	btSoftBody* cloth = btSoftBodyHelpers::CreatePatch(g_softBodyWorldInfo,
		btVector3(x, y1, z1),    // 四隅の座標 00
		btVector3(x, y1, z2),    // 四隅の座標 10
		btVector3(x, y2, z1),    // 四隅の座標 01
		btVector3(x, y2, z2),    // 四隅の座標 11
		res, res, // 分割数(2方向)
		0, // 四隅の固定フラグ(1,2,4,8)
		true); // 斜め方向のばねのON/OFF
	cloth->getCollisionShape()->setMargin(0.01);
	cloth->setTotalMass(0.02); // 全体の質量
	cloth->m_materials[0]->m_kLST = 0.5;
	// Lift, Drag Forceのための係数[0,∞]
	cloth->m_cfg.kLF = 0.05;
	cloth->m_cfg.kDG = 0.01;

	cloth->m_cfg.piterations = 2; // ばねによる位置修正の最大反復回数
	cloth->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSidedLiftDrag;

	cloth->setWindVelocity(wind_velo);
	g_dynamicsworld->addSoftBody(cloth);
	btScalar mass = 0.0;
	btTransform trans;
	btVector3 rod_size = btVector3(0.05, 1.0, 0.05);
	trans.setIdentity();
	trans.setOrigin(pos);
	btCollisionShape* rod_shape = new btCylinderShape(rod_size);
	g_collisionshapes.push_back(rod_shape);
	int index = 96;
	cp_rigid = CreateRigidBody(mass, trans, rod_shape, g_dynamicsworld, &index, RX_COL_CP, RX_COL_ALL & (~RX_COL_PLAYER));
	cp_rigid->setUserIndex2(stage);
	for (int i = 1; i <= res; i++)
		cloth->appendAnchor(res * i - 1, cp_rigid);
}