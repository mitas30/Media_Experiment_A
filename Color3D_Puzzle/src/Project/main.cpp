/*
* 豆知識集
* コールバック関数とは、Unityのイベント関数のこと
* 動かせる物体は軽くすること
* 白ギミックは反発係数を小さめにすること。
*/
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glu32.lib")
#pragma comment(lib, "glew32.lib")
#pragma comment(lib, "glfw3dll.lib")

#ifdef _DEBUG
#pragma comment (lib, "LinearMath_vs2010_x64_debug.lib")
#pragma comment (lib, "BulletCollision_vs2010_x64_debug.lib")
#pragma comment (lib, "BulletDynamics_vs2010_x64_debug.lib")
#pragma comment (lib, "BulletSoftBody_vs2010_x64_debug.lib")
#else
#pragma comment (lib, "LinearMath_vs2010_x64_release.lib")
#pragma comment (lib, "BulletCollision_vs2010_x64_release.lib")
#pragma comment (lib, "BulletDynamics_vs2010_x64_release.lib")
#pragma comment (lib, "BulletSoftBody_vs2010_x64_release.lib")
#endif

#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

#define GL_SILENCE_DEPRECATION	// mac環境でgluを使っている場合の非推奨warningの抑制


//-----------------------------------------------------------------------------
// Include Files
//-----------------------------------------------------------------------------
#include "utils.h"
// 必要なヘッダーファイル
#include <GLFW/glfw3.h>
#include "make_stage.hpp"
#include "make_gimmick.hpp"
// ImGUI
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

#include <time.h>

using namespace std;

#define SPECIAL_INDEX 98
#define R_MAX 10
//-----------------------------------------------------------------------------
// 定数・グローバル変数
//-----------------------------------------------------------------------------
const glm::vec4 LIGHT0_POS(0.5f, 4.0f, 1.5f, 0.0f);
const glm::vec4 LIGHT1_POS(-1.0f, -10.0f, -1.0f, 0.0f);
const glm::vec4 LIGHT_AMBI(0.3f, 0.3f, 0.3f, 1.0f);
const glm::vec4 LIGHT_DIFF(1.0f, 1.0f, 1.0f, 1.0f);
const glm::vec4 LIGHT_SPEC(1.0f, 1.0f, 1.0f, 1.0f);
const GLfloat FOV = 45.0f;

// グローバル変数
int g_winw = 1024;							//!< 描画ウィンドウの幅
int g_winh = 1024;							//!< 描画ウィンドウの高さ
rxTrackball g_view;							//!< 視点移動用トラックボール
float g_bgcolor[3] = { 1, 1, 1 };			//!< 背景色
bool g_animation_on = false;				//!< アニメーションON/OFF

// シャドウマッピング
ShadowMap g_shadowmap;
int g_shadowmap_res = 1024;

// 物理シミュレーション関連定数/変数
float g_dt = 0.02;	//!< 時間ステップ幅
// Bullet
btSoftRigidDynamicsWorld* g_dynamicsworld;	//!< Bulletワールド
btAlignedObjectArray<btCollisionShape*>	g_collisionshapes;		//!< 剛体オブジェクトの形状を格納する動的配列
btSoftBodyWorldInfo g_softBodyWorldInfo;

//物体のグローバル変数
extern btRigidBody* player_rigid[8];
extern btRigidBody* brush_rigid;
btTypedConstraint* pickConstraint = nullptr;
btRigidBody* rotate_rigid;
extern btRigidBody* move_rigid;

// マウスピック
btVector3 g_pickpos;
btRigidBody* g_pickbody = 0;
btPoint2PointConstraint* g_pickconstraint = 0;
btSoftBody::Node* g_picknode = 0;
double g_pickdist = 0.0;

//プロパティ類
clock_t touch_time = -1000;
btScalar leg_ang_speed=3.0;

//ステージ管理(trueになったときに破棄と生成)
extern btVector3 min_viable_area;
extern btVector3 max_viable_area;
extern btVector3 move_min_move;
extern btVector3 move_max_move;

//プレイヤー固有
btScalar max_speed = 30.0;
btScalar force_coef = 100;
btScalar lr_force_coef = 0.7;
btVector3 jump_impulse = btVector3(0, 45, 0);
bool w_flag = false, a_flag = false, s_flag = false, d_flag = false, release_flag = false;
bool left_pressing = false;
int clear_stage = 0;
int current_stage = 1;

//色変化用係数
btScalar fric_coef = 0.02;
btScalar mass_coef = 0.01;
btScalar restitution_coef = 12;
btScalar expand_coef = 1.2;
btScalar reduce_coef = 0.2;
btScalar incr_inertia_coef = 10000;
btScalar decr_inertia_coef = 0.01;

//-----------------------------------------------------------------------------
// 関数の前方宣言
//-----------------------------------------------------------------------------
void MoveCameraWithRigid(rxTrackball& view, btRigidBody* body);
void Mouse(GLFWwindow* window, int button, int action, int mods);
void reset(void);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void Keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);
btRigidBody* SwitchColorChange(btRigidBody* n_SpecialBody, btRigidBody* n_OtherBody);
btRigidBody* RigidMakedWhite(btRigidBody* n_SpecialBody, btRigidBody* n_OtherBody);
btSoftBody* AddSoftBodies2D(int& res);
btRigidBody* GetClosestObj(float maxRad);
btVector3 LocalToWorldTrans(btRigidBody* local_body, btVector3 local_vector);
void InitBullet(void);
void CleanBullet(void);

//筆(index=98)が衝突するたびに呼ばれるコールバッククラス
class MyContactResultCallback : public btCollisionWorld::ContactResultCallback
{
public:
	bool collision = false;

	virtual btScalar addSingleResult(btManifoldPoint& cp,
		const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
		const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
	{
		// Get the rigidbodies involved in the collision
		const btRigidBody* body0 = btRigidBody::upcast(colObj0Wrap->getCollisionObject());
		const btRigidBody* body1 = btRigidBody::upcast(colObj1Wrap->getCollisionObject());
		collision = true;
		//specialが
		const btRigidBody* specialBody = (body0->getUserIndex() == SPECIAL_INDEX) ? body0 : body1;
		const btRigidBody* otherBody = (body0->getUserIndex() == SPECIAL_INDEX) ? body1 : body0;
		time_t temp_t = clock() - touch_time;
		//0はプレイヤー、1は黒、99は色源
		//一定時間が経過+触った先が変えれるなら...
		int user_index = otherBody->getUserIndex();
		if (user_index != 0 && user_index != 1 && (temp_t > 1000))
		{
			// Remove const modifier
			btRigidBody* n_OtherBody = const_cast<btRigidBody*>(otherBody);
			btRigidBody* n_SpecialBody = const_cast<btRigidBody*>(specialBody);

			//触った先が色源の場合
			if (user_index == 99)
			{
				if (otherBody->getUserIndex2() == 97)
					n_SpecialBody->setUserIndex2(2);
				else
					n_SpecialBody->setUserIndex2(n_OtherBody->getUserIndex2());
			}
			else if (user_index == 97)
			{
				n_SpecialBody->setUserIndex2(2);
			}
			//checkpoint+ステージ破壊と作成
			else if (user_index == 96)
			{
				int index2=n_OtherBody->getUserIndex2();
				switch (index2)
				{
				case 1:
					clear_stage = 1;
					current_stage = 2;
					break;
				case 2:
					clear_stage = 2;
					current_stage = 3;
					break;
				case 3:
					clear_stage = 3;
					break;
				default:
					cout << "Undefined stage number." << endl;
					break;
				}
			}
			// 筆のIndex2が2(色吸収モード)のとき
			else if (n_SpecialBody->getUserIndex2() == 2)
			{
				n_OtherBody=RigidMakedWhite(n_SpecialBody, n_OtherBody);
				n_SpecialBody->setUserIndex2(n_OtherBody->getUserIndex());
				n_OtherBody->setUserIndex(2);
			}
			//筆のIndex2が塗りモードの場合(else ifで競合回避)
			else if (3 <= n_SpecialBody->getUserIndex2() && n_SpecialBody->getUserIndex2() <= 10)
			{
				if (user_index == 2)
				{;
					n_OtherBody = SwitchColorChange(n_SpecialBody, n_OtherBody);
					n_OtherBody->setUserIndex(n_SpecialBody->getUserIndex2());
					n_SpecialBody->setUserIndex2(2);
				}
				//一回白に戻す必要性がある(性質の重ね塗り対策)
				else
				{
					n_OtherBody = RigidMakedWhite(n_SpecialBody, n_OtherBody);
					n_OtherBody = SwitchColorChange(n_SpecialBody, n_OtherBody);
					n_OtherBody->setUserIndex(n_SpecialBody->getUserIndex2());
					n_SpecialBody->setUserIndex2(2);
				}
			}
			else
			{
				cout << "Unkown Color Detected." << endl;
			}
			touch_time = clock();
		}
		return 0;
	}
};

void InitBullet(void)
{
	int res = 0;
	// 衝突検出方法の選択(デフォルトを選択)
	btDefaultCollisionConfiguration* config = new btSoftBodyRigidBodyCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(config);

	// ブロードフェーズ法の設定(Dynamic AABB tree method)
	btDbvtBroadphase* broadphase = new btDbvtBroadphase();

	// 拘束(剛体間リンク)のソルバ設定
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	// Bulletのワールド作成
	g_dynamicsworld = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, config, 0);
	// 重力加速度の設定(OpenGLに合わせてy軸方向を上下方向にする)
	g_dynamicsworld->setGravity(btVector3(0, -9.8, 0));

	// btSoftBodyWorldInfoの初期化・設定
	g_softBodyWorldInfo.m_dispatcher = dispatcher;
	g_softBodyWorldInfo.m_broadphase = broadphase;
	g_softBodyWorldInfo.m_sparsesdf.Initialize();
	g_softBodyWorldInfo.m_gravity.setValue(0, -9.8, 0);
	g_softBodyWorldInfo.air_density = 1.204;
	g_softBodyWorldInfo.m_sparsesdf.Reset();
	btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
}

/*!
* 設定したBulletの剛体オブジェクト，ワールドの破棄
*/
void CleanBullet(void)
{
	// 衝突オブジェクトの破棄
	for (int i = g_dynamicsworld->getNumCollisionObjects() - 1; i >= 0; --i) {
		btCollisionObject* obj = g_dynamicsworld->getCollisionObjectArray()[i];

		// オブジェクトがRigid Bodyの場合の破棄
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState()) {
			delete body->getMotionState();
		}

		// オブジェクトがSoft Bodyの場合の破棄
		btSoftBody* softBody = btSoftBody::upcast(obj);
		if (softBody) {
			static_cast<btSoftRigidDynamicsWorld*>(g_dynamicsworld)->removeSoftBody(softBody);
		}
		else {
			static_cast<btSoftRigidDynamicsWorld*>(g_dynamicsworld)->removeCollisionObject(obj);
		}

		g_dynamicsworld->removeCollisionObject(obj);
		delete obj;
	}

	// 形状の破棄
	for (int j = 0; j < (int)g_collisionshapes.size(); ++j) {
		btCollisionShape* shape = g_collisionshapes[j];
		g_collisionshapes[j] = 0;
		delete shape;
	}
	g_collisionshapes.clear();

	// ワールド破棄
	delete g_dynamicsworld->getBroadphase();
	delete g_dynamicsworld;
}
//-----------------------------------------------------------------------------
// アプリケーション制御関数
//-----------------------------------------------------------------------------
/*!
* アニメーションN/OFF
* @param[in] on trueでON, falseでOFF
*/
bool switchanimation(int on)
{
	g_animation_on = (on == -1) ? !g_animation_on : (on ? true : false);
	return g_animation_on;
}

/*!
* 現在の画面描画を画像ファイルとして保存(連番)
* @param[in] stp 現在のステップ数(ファイル名として使用)
*/
void savedisplay(const int& stp)
{
	static int nsave = 1;
	string fn = CreateFileName("img_", ".bmp", (stp == -1 ? nsave++ : stp), 5);
	saveFrameBuffer(fn, g_winw, g_winh);
	std::cout << "saved the screen image to " << fn << std::endl;
}
/*!
* 視点の初期化
*/
void resetview(void)
{
	double q[4] = { 1, 0, 0, 0 };
	g_view.SetQuaternion(q);
	g_view.SetRotation(20.0, 1.0, 0.0, 0.0);
	g_view.SetScaling(-7.0);
	g_view.SetTranslation(0.0, -2.0);
}
/*!
* シミュレーションのリセット
*/
void reset(void)
{
	CleanBullet();
	InitBullet();
	switch (current_stage)
	{
	case 1:
		MakeStage1();
		break;
	case 2:
		MakeStage2();
		break;
	case 3:
		MakeStage3();
		break;
	default:
		cout << "That is exception stage" << endl;
		break;
	}
}

/*!
* 初期化関数
*/
void Init(void)
{
	// OpenGLのバージョンチェック
	cout << "OpenGL version: " << glGetString(GL_VERSION) << endl;
	cout << "GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
	cout << "Vendor: " << glGetString(GL_VENDOR) << endl;
	cout << "Renderer: " << glGetString(GL_RENDERER) << endl;

	// GLEWの初期化
	GLenum err = glewInit();
	if (err != GLEW_OK) cout << "GLEW Error : " << glewGetErrorString(err) << endl;

	// 描画系フラグ設定(アンチエイリアス,デプステスト,隠面除去,法線計算,点描画)
	glEnable(GL_MULTISAMPLE);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glEnable(GL_AUTO_NORMAL);
	glEnable(GL_NORMALIZE);
	glEnable(GL_POINT_SMOOTH);

	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_POSITION, glm::value_ptr(LIGHT0_POS));
	glLightfv(GL_LIGHT0, GL_DIFFUSE, glm::value_ptr(LIGHT_DIFF));
	glLightfv(GL_LIGHT0, GL_SPECULAR, glm::value_ptr(LIGHT_SPEC));
	glLightfv(GL_LIGHT0, GL_AMBIENT, glm::value_ptr(LIGHT_AMBI));

	// 視点初期化
	resetview();

	// シャドウマップ初期化
	g_shadowmap.InitShadow(g_shadowmap_res, g_shadowmap_res);

	// Bullet初期化
	InitBullet();
	MakeStage1();

	switchanimation(1);
}

//-----------------------------------------------------------------------------
// OpenGL/GLFWコールバック(特定の条件で呼ばれる)関数
//-----------------------------------------------------------------------------
/*!
* Bulletのオブジェクトの描画シーン描画
* @param[in] x クラスのメンバ関数(static)を渡すときに用いるポインタ(グローバル関数の場合は使わないので0でOK)
*/
void DrawBulletObjects(void* x = 0)
{
	static const GLfloat spec[] = { 0.4, 0.4, 0.4, 1.0 };	// 鏡面反射色
	static const GLfloat ambi[] = { 0.3, 0.3, 0.3, 1.0 };	// 環境光
	static const GLfloat main_white[] = { 0.808, 0.78, 0.808, 1.0 };	// メインオブジェクト:(0)
	static const GLfloat black[] = { 0.0, 0.0, 0.0, 1.0 };	// 黒(1)
	static const GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };	// 白(2)
	static const GLfloat cyan[] = { 0.0, 1.0, 1.0, 1.0 };	// (3)
	static const GLfloat clear_yellow[] = { 1.0, 1.0, 0.0, 1.0 };	//(4) 
	static const GLfloat deep_green[] = { 0.0, 0.5, 0.0, 1.0 };	// (5)
	static const GLfloat pink[] = { 1.0, 0.75,0.8, 1.0 };	// (6)　弾性体
	static const GLfloat red[] = { 1.0, 0.0, 0.0, 1.0 };	// (7)　大きさ↑
	static const GLfloat blue[] = { 0.0, 0.0, 1.0, 1.0 };	// (8)　大きさ↓
	static const GLfloat gray[] = { 0.5, 0.5, 0.5, 1.0 };	// (9)　慣性モーメント+
	static const GLfloat orange[] = { 1.0, 0.65, 0.0, 1.0 };	// (10)　慣性モーメント-
	static const GLfloat shine_black[] = { 0.2,0.2,0.2 }; //index=97


	glDisable(GL_COLOR_MATERIAL);

	// 光源/材質設定
	glLightfv(GL_LIGHT0, GL_POSITION, glm::value_ptr(LIGHT0_POS));
	glMaterialfv(GL_FRONT, GL_SPECULAR, spec);
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambi);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0f);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.0, 0.0, 1.0);

	glEnable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);

	if (g_dynamicsworld) {
		btScalar m[16];
		btMatrix3x3	rot;
		rot.setIdentity();

		// Bulletワールドから剛体オブジェクト情報を取得してOpenGLで描画
		const int n = g_dynamicsworld->getNumCollisionObjects();	// オブジェクト数の取得
		for (int i = 0; i < n; ++i) {

			// btCollisionObject → btRigidBodyへのキャストで剛体オブジェクトを取得
			btCollisionObject* obj = g_dynamicsworld->getCollisionObjectArray()[i];

			// 形状取得
			btCollisionShape* shape = obj->getCollisionShape();
			int shapetype = shape->getShapeType();

			if (shapetype == SOFTBODY_SHAPE_PROXYTYPE) {
				btSoftBody* body = btSoftBody::upcast(obj);

				glMaterialfv(GL_FRONT, GL_DIFFUSE, pink);

				// draw a softbody
				DrawBulletSoftBody(body);
			}
			else {
				btRigidBody* body = btRigidBody::upcast(obj);
				if (body && body->getMotionState()) {
					// btRigidBodyからMotion Stateを取得して，OpenGLの変換行列として位置・姿勢情報を得る
					btDefaultMotionState* ms = (btDefaultMotionState*)body->getMotionState();
					ms->m_graphicsWorldTrans.getOpenGLMatrix(m);
					rot = ms->m_graphicsWorldTrans.getBasis();
				}
				else {
					obj->getWorldTransform().getOpenGLMatrix(m);
					rot = obj->getWorldTransform().getBasis();
				}

				if (body) {
					switch (body->getUserIndex())
					{
						//持てるものとプレイヤー
					case 0:
					case 96:
						glMaterialfv(GL_FRONT, GL_DIFFUSE, main_white);
						//色源または筆
					case 98:
					case 99:
						switch (body->getUserIndex2()) {
						case 2:
							glMaterialfv(GL_FRONT, GL_DIFFUSE, white);
							break;
						case 3:
							glMaterialfv(GL_FRONT, GL_DIFFUSE, cyan);
							break;
						case 4:
							glMaterialfv(GL_FRONT, GL_DIFFUSE, clear_yellow);
							break;
						case 5:
							glMaterialfv(GL_FRONT, GL_DIFFUSE, deep_green);
							break;
						case 6:
							glMaterialfv(GL_FRONT, GL_DIFFUSE, pink);
							break;
						case 7:
							glMaterialfv(GL_FRONT, GL_DIFFUSE, red);
							break;
						case 8:
							glMaterialfv(GL_FRONT, GL_DIFFUSE, blue);
							break;
						case 9:
							glMaterialfv(GL_FRONT, GL_DIFFUSE, gray);
							break;
						case 10:
							glMaterialfv(GL_FRONT, GL_DIFFUSE, orange);
							break;
						case 97:
							glMaterialfv(GL_FRONT, GL_DIFFUSE, shine_black);
							break;
						}
						break;
					case 1:
						glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
						break;
					case 2:
						glMaterialfv(GL_FRONT, GL_DIFFUSE, white);
						break;
					case 3:
						glMaterialfv(GL_FRONT, GL_DIFFUSE, cyan);
						break;
					case 4:
						glMaterialfv(GL_FRONT, GL_DIFFUSE, clear_yellow);
						break;
					case 5:
						glMaterialfv(GL_FRONT, GL_DIFFUSE, deep_green);
						break;
					case 6:
						glMaterialfv(GL_FRONT, GL_DIFFUSE, pink);
						break;
					case 7:
						glMaterialfv(GL_FRONT, GL_DIFFUSE, red);
						break;
					case 8:
						glMaterialfv(GL_FRONT, GL_DIFFUSE, blue);
						break;
					case 9:
						glMaterialfv(GL_FRONT, GL_DIFFUSE, gray);
						break;
					case 10:
						glMaterialfv(GL_FRONT, GL_DIFFUSE, orange);
						break;
					case 97:
						glMaterialfv(GL_FRONT, GL_DIFFUSE, shine_black);
						break;
					default:
						glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
						break;
					}
				}

				btVector3 world_min, world_max;
				g_dynamicsworld->getBroadphase()->getBroadphaseAabb(world_min, world_max);

				glPushMatrix();
#ifdef BT_USE_DOUBLE_PRECISION
				glMultMatrixd(m);
#else
				glMultMatrixf(m);
#endif

				// 形状描画
				DrawBulletShape(shape, world_min, world_max);

				glPopMatrix();
			}
		}
	}

}

void Display(void)
{
	// ビューポート,透視変換行列,モデルビュー変換行列の設定
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glm::mat4 mp = glm::perspective(FOV, (float)g_winw / g_winh, 0.2f, 1000.0f);
	glMultMatrixf(glm::value_ptr(mp));
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// 描画バッファのクリア
	glClearColor((GLfloat)g_bgcolor[0], (GLfloat)g_bgcolor[1], (GLfloat)g_bgcolor[2], 1.0f);
	glClearDepth(1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();

	// マウスによる回転・平行移動の適用
	g_view.Apply();

	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glDisable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
	// シャドウマップを使って影付きでオブジェクト描画
	glm::vec3 light_pos(LIGHT0_POS[0], LIGHT0_POS[1], LIGHT0_POS[2]);
	ShadowMap::Frustum light = CalFrustum(80, 0.02, 20.0, g_shadowmap_res, g_shadowmap_res, light_pos, glm::vec3(0.0, -1.0, 0.0), glm::vec3(0.0, 1.0, 0.0));
	g_shadowmap.RenderSceneWithShadow(light, DrawBulletObjects, 0);

	glPopMatrix();
}

void Timer(void)
{
	if (g_animation_on) {
		if (g_dynamicsworld) {
			// シミュレーションを1ステップ進める
			g_dynamicsworld->stepSimulation(g_dt, 1);
			MoveCameraWithRigid(g_view, player_rigid[0]); //カメラを後ろに配置
		}
		//移動処理
		if (w_flag || a_flag || s_flag || d_flag)
		{
			btVector3 force = btVector3(0, 0, 0);
			btVector3 front_back_force = LocalToWorldTrans(player_rigid[0], btVector3(0, 0, 1)).normalize();
			btVector3 left_right_force = LocalToWorldTrans(player_rigid[0], btVector3(1, 0, 0)).normalize()*lr_force_coef;
			if (w_flag)
				force -= front_back_force;
			if (s_flag)
				force += front_back_force;
			if (a_flag)
				force -= left_right_force;
			if (d_flag)
				force += left_right_force;
			if (force.length() != 0)
			{
				if (force.length() > front_back_force.length())
					force.normalize();
				player_rigid[0]->applyCentralForce(force * force_coef);
				for(int i=1;i<8;i++)
					player_rigid[i]->applyCentralForce(force * force_coef/20);
			}
		}
		//場外判定
		if (player_rigid)
		{
			btVector3 chara_pos = player_rigid[0]->getCenterOfMassPosition();

			if (chara_pos.x() < min_viable_area.x() || chara_pos.y() < min_viable_area.y() || chara_pos.z() < min_viable_area.z() ||
				max_viable_area.x() < chara_pos.x() || max_viable_area.y() < chara_pos.y() || max_viable_area.z() < chara_pos.z())
				RespawnPlayer();
		}
		if (brush_rigid)
		{
			btVector3 brush_pos = brush_rigid->getCenterOfMassPosition();
			if (brush_pos.x() < min_viable_area.x() || brush_pos.y() < min_viable_area.y() || brush_pos.z() < min_viable_area.z() ||
				max_viable_area.x() < brush_pos.x() || max_viable_area.y() < brush_pos.y() || max_viable_area.z() < brush_pos.z())
				RespawnBrush();
		}
		if (move_rigid)
			MoveSkyBox();
	}
}

/*!
* リサイズイベント処理関数
* @param[in] window コールバック関数を呼んだウィンドウハンドル
* @param[in] w キャンバス幅(ピクセル数)
* @param[in] h キャンバス高さ(ピクセル数)
*/
void Resize(GLFWwindow* window, int w, int h)
{
	g_winw = w; g_winh = h;
	g_view.SetRegion(w, h);
	glViewport(0, 0, g_winw, g_winh);
}

/*!
* ImGUI(GUI部分)のウィジット配置 デバッグに使おう
*  - ImGUI/imgui_demo.cppを参考に ( https://github.com/ocornut/imgui#demo )
* @param[in] window コールバック関数を呼んだウィンドウハンドル
*/
void SetImGUI(GLFWwindow* window)
{
	ImGui::Text("simulation:");
	if (ImGui::Button("start/stop")) { switchanimation(-1); } ImGui::SameLine();
	if (ImGui::Button("run a step")) { g_animation_on = true; Timer(); g_animation_on = false; }
	if (ImGui::Button("reset")) { reset(); }
	ImGui::Separator();
	ImGui::InputFloat("dt", &(g_dt), 0.001f, 0.01f, "%.3f");
	ImGui::Separator();
	if (ImGui::Button("reset viewpos")) { resetview(); }
	if (ImGui::Button("save screenshot")) { savedisplay(-1); }
	if (ImGui::Button("quit")) { glfwSetWindowShouldClose(window, GL_TRUE); }
	ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Goal");
}

void Clean()
{
	CleanBullet();
}

/*!
 * メインルーチン
 * @param[in] argc コマンドライン引数の数
 * @param[in] argv コマンドライン引数
 */
int main(int argc, char* argv[])
{
	if (!glfwInit()) return 1;
	glfwSetErrorCallback(glfw_error_callback);

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
#ifdef __APPLE__
	glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_FALSE);
#endif

	// Create window
	GLFWwindow* window = glfwCreateWindow(g_winw, g_winh, "OpenGL Application", NULL, NULL);
	if (window == NULL) return 1;

	// Set glfw window as current OpenGL rendering context
	glfwMakeContextCurrent(window);
	glewExperimental = GL_TRUE;
	glfwSwapInterval(0); // Disable vsync

	// Initilization for OpenGL
	Init();

	// Setup callback functions (callbackは全部ここっぽい)
	glfwSetMouseButtonCallback(window, Mouse);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetKeyCallback(window, Keyboard);
	glfwSetFramebufferSizeCallback(window, Resize);
	Resize(window, g_winw, g_winh);

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsClassic();

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL2_Init();

	// Settings for timer
	float cur_time = 0.0f, last_time = 0.0f, elapsed_time = 0.0f;
	glfwSetTime(0.0);	// Initialize the glfw timer

	// Main loop
	while (!glfwWindowShouldClose(window))
	{
		// Poll and handle events (inputs, window resize, etc.)
		glfwPollEvents();

		// 描画
		Display();

		// Timer
		cur_time = glfwGetTime();
		elapsed_time = cur_time - last_time;
		if (elapsed_time >= g_dt) {
			Timer();
			last_time = glfwGetTime();
		}
		MyContactResultCallback callback;
		//衝突している回数だけ呼ばれる
		if (brush_rigid)
		{
			g_dynamicsworld->contactTest(brush_rigid, callback);
		}
		if (clear_stage!=0)
		{
			switch (clear_stage)
			{
			case 1:
				CleanBullet();
				InitBullet();
				MakeStage2();
				break;
			case 2:
				CleanBullet();
				InitBullet();
				move_rigid = nullptr;
				MakeStage3();
				break;
			case 3:
				cout << "Congrats." << endl;
			default:
				break;
			}
			clear_stage = 0;
		}

		// Start the ImGui frame
		ImGui_ImplOpenGL2_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		// GUI
		ImGui::Begin("ImGui Window");
		ImGui::Text("Framerate: %.3f ms/frame (%.1f fps)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::Separator();
		SetImGUI(window);
		ImGui::End();

		// Rendering of the ImGUI frame in opengl canvas
		ImGui::Render();
		ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	// Cleanup
	Clean();
	ImGui_ImplOpenGL2_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(window);
	glfwTerminate();


	return 0;
}

/*!
* マウスイベント処理関数
* @param[in] window コールバック関数を呼んだウィンドウハンドル
* @param[in] button マウスボタン(GLFW_MOUSE_BUTTON_LEFT,GLFW_MOUSE_BUTTON_MIDDLE,GLFW_MOUSE_BUTTON_RIGHT)
* @param[in] action マウスボタンの状態(GLFW_PRESS, GLFW_RELEASE)
* @param[in] mods 修飾キー(CTRL,SHIFT,ALT) -> https://www.glfw.org/docs/latest/group__mods.html
*/
void Mouse(GLFWwindow* window, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
	{
		left_pressing = true;
		//プレイヤーから最も近いオブジェクトを探します
		btRigidBody* pickedObject = GetClosestObj(5.0);
		if (pickedObject == nullptr)
		{
			std::cout << "nothing\n";
		}
		if (pickedObject)
		{
			btQuaternion a(0, 0, 0, 1), b(0, 0, 0, 1);
			if (!player_rigid[0])
			{
				cout << "null playerRigid" << endl;
				return;
			}

			btTransform frame_in_a(a, btVector3(0,0, -0.1)), frame_in_b(b, btVector3(0, 0, 0.6));
			//Aのほうが重いのでA=true ワールド座標での指定を行うこと
			btGeneric6DofConstraint* player2pencilcon = new btGeneric6DofConstraint(*player_rigid[7], *pickedObject, frame_in_a, frame_in_b, true);
			// 全ての軸で移動を無効化
			for (int i = 0; i < 3; i++) {
				player2pencilcon->setLimit(i, 0, 0);
			}
			player2pencilcon->setAngularLowerLimit(btVector3(0,0, -RX_PI / 12));
			player2pencilcon->setAngularUpperLimit(btVector3(0, 0,RX_PI / 12));
			// 制約を追加
			g_dynamicsworld->addConstraint(player2pencilcon, true);
			pickConstraint = player2pencilcon;
		}
	}
	else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
	{
		left_pressing = false;
		// マウスのボタンが離されたときに、"ピック"操作を終了
		if (pickConstraint)
		{
			g_dynamicsworld->removeConstraint(pickConstraint);
			delete pickConstraint;
			pickConstraint = nullptr;
		}
	}
}

//xしか使用しない
double lastX = 0;
bool firstMouse = true;
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	//可変パラメータ 1で1cmの移動につき100度回転
	double rotatespeed = 1.0;
	if (firstMouse) // 初めてのマウス入力の場合
	{
		lastX = xpos;
		firstMouse = false;
	}

	// マウスの移動量を計算
	double xoffset = lastX - xpos;
	lastX = xpos;

	// マウスの移動量に基づいてキャラクターを回転
	if (player_rigid[0]) {
		btTransform trans;
		trans.setIdentity();
		for (int i = 0; i < 8; i++)
		{
			player_rigid[i]->getMotionState()->getWorldTransform(trans);
			btQuaternion rotation;
			if(i==4||i==5)
				rotation = btQuaternion(btVector3(1, 0, 0), btScalar(xoffset * rotatespeed * RX_PI / 180.0));
			else
				rotation = btQuaternion(btVector3(0, 1, 0), btScalar(xoffset * rotatespeed * RX_PI / 180.0));
			// 現在の姿勢と新たな回転を合成
			btQuaternion newRotation = trans.getRotation() * rotation;
			trans.setRotation(newRotation);

			// 新たな姿勢を設定
			player_rigid[i]->setWorldTransform(trans);
		}
	}
}

/*!
* キーボードイベント処理関数
* @param[in] window コールバック関数を呼んだウィンドウハンドル
* @param[in] key キーの種類 -> https://www.glfw.org/docs/latest/group__keys.html
* @param[in] scancode キーのスキャンコード(プラットフォーム依存)
* @param[in] action アクション(GLFW_PRESS:キーを押す, GLFW_RELEASE:キーを離す，GLFW_REPEAT:キーリピート機能時)
* !Attention REPEATは、「押している間」ではないので注意
* @param[in] mods 修飾キー(CTRL,SHIFT,ALT) -> https://www.glfw.org/docs/latest/group__mods.html
*/
void Keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (ImGui::GetIO().WantCaptureKeyboard)  // ImGUIウィンドウ上でのキーボードイベント時
		return;

	if (action == GLFW_PRESS) {
		switch (key) {
		case GLFW_KEY_ESCAPE:	// ESC,Qキーでアプリケーション終了
		case GLFW_KEY_Q:
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;
		case GLFW_KEY_ENTER:
			switchanimation(-1);
			break;
		case GLFW_KEY_R: // Rキーでシーン(シミュレーション)リセット
			reset();
			break;
		case GLFW_KEY_SPACE: // ジャンプ
			player_rigid[0]->applyCentralImpulse(jump_impulse);
			for(int i=1;i<8;i++)
				player_rigid[i] ->applyCentralImpulse(jump_impulse/20);
			break;
		case GLFW_KEY_W: // 前進
			w_flag = true;
			break;
		case GLFW_KEY_A: // 左移動
			a_flag = true;
			break;
		case GLFW_KEY_S: // 後退
			s_flag = true;
			break;
		case GLFW_KEY_D: // 右移動
			d_flag = true;
			break;
		default:
			break;
		}
	}
	else if (action == GLFW_RELEASE)
	{
		release_flag = true;
		switch (key)
		{
		case GLFW_KEY_W: // 前進
			w_flag = false;
			break;
		case GLFW_KEY_A: // 左移動
			a_flag = false;
			break;
		case GLFW_KEY_S: // 後退
			s_flag = false;
			break;
		case GLFW_KEY_D: // 右移動
			d_flag = false;
			break;
		default:
			break;
		}
	}
}

//-----------------------------------------------------------------------------
// 子関数
//-----------------------------------------------------------------------------
// 
//物体を色で染める
//!BUG constraint繋いでたりするとやばい。constraint側がメモリリーク起こすので、デモプレイではギミックを使わないように。
btRigidBody* SwitchColorChange(btRigidBody* n_SpecialBody, btRigidBody* n_OtherBody)
{
	btScalar b_mass;
	if (n_OtherBody->getInvMass())
		b_mass=1 / n_OtherBody->getInvMass();
	else
		b_mass = 0;
	btVector3 b_inertia = n_OtherBody->getLocalInertia();
	btVector3 b_scaling = n_OtherBody->getCollisionShape()->getLocalScaling();
	int o_index;
	btCollisionShape* o_shape{};
	btTransform o_trans;
	btMotionState* motion_state_a{};
	btRigidBody* delete_body = n_OtherBody;
	btRigidBody* return_body{};
	btVector3 b_lin_velo;
	btVector3 b_ang_velo;
	btScalar o_friction;
	btScalar o_restitution;

	switch (n_SpecialBody->getUserIndex2()) {
	case 3:
		n_OtherBody->setFriction(n_OtherBody->getFriction() * fric_coef);
		return_body = n_OtherBody;
		break;
	case 4:
		n_OtherBody->setMassProps(b_mass * mass_coef, b_inertia);
		return_body = n_OtherBody;
		break;
	case 5:
		n_OtherBody->setRestitution(n_OtherBody->getRestitution() * restitution_coef);
		return_body = n_OtherBody;
		break;
	case 6:
		break;
	case 7:
		b_lin_velo = n_OtherBody->getLinearVelocity();
		b_ang_velo = n_OtherBody->getAngularVelocity();
		o_index = n_OtherBody->getUserIndex();
		o_trans = n_OtherBody->getWorldTransform();
		o_shape = n_OtherBody->getCollisionShape();
		o_friction = n_OtherBody->getFriction();
		o_restitution = n_OtherBody->getRestitution();
		o_shape->setLocalScaling(b_scaling * expand_coef);
		g_dynamicsworld->removeRigidBody(n_OtherBody);
		motion_state_a = n_OtherBody->getMotionState();
		delete motion_state_a;
		delete delete_body;
		return_body=CreateRigidBody(b_mass,o_trans, o_shape, g_dynamicsworld,&o_index,RX_COL_OBST,RX_COL_ALL,false,b_inertia);
		return_body->setLinearVelocity(b_lin_velo);
		return_body->setAngularVelocity(b_ang_velo);
		return_body->setFriction(o_friction);
		return_body->setRestitution(o_restitution);
		break;
	case 8:
		b_lin_velo = n_OtherBody->getLinearVelocity();
		b_ang_velo = n_OtherBody->getAngularVelocity();
		o_index = n_OtherBody->getUserIndex();
		o_trans = n_OtherBody->getWorldTransform();
		o_shape = n_OtherBody->getCollisionShape();
		o_friction = n_OtherBody->getFriction();
		o_restitution = n_OtherBody->getRestitution();
		o_shape->setLocalScaling(b_scaling * reduce_coef);
		g_dynamicsworld->removeRigidBody(n_OtherBody);
		motion_state_a = n_OtherBody->getMotionState();
		delete motion_state_a;
		delete delete_body;
		return_body = CreateRigidBody(b_mass, o_trans, o_shape, g_dynamicsworld,&o_index, RX_COL_OBST, RX_COL_ALL, false, b_inertia);
		return_body->setLinearVelocity(b_lin_velo);
		return_body->setAngularVelocity(b_ang_velo);
		return_body->setFriction(o_friction);
		return_body->setRestitution(o_restitution);
		break;
	case 9:
		n_OtherBody->setMassProps(b_mass, b_inertia * incr_inertia_coef);
		return_body = n_OtherBody;
		break;
	case 10:
		n_OtherBody->setMassProps(b_mass, b_inertia * decr_inertia_coef);
		return_body = n_OtherBody;
		break;
	default:
		cout << "error\n";
		break;
	}
	return return_body;
}

//物体を白に戻す ただし、性質のみ変化
//!BUG constraint繋いでたりするとやばい。
btRigidBody* RigidMakedWhite(btRigidBody* n_SpecialBody, btRigidBody* n_OtherBody)
{
	btScalar b_mass;
	if (n_OtherBody->getInvMass())
		b_mass = 1 / n_OtherBody->getInvMass();
	else
		b_mass = 0;
	btVector3 b_inertia = n_OtherBody->getLocalInertia();
	btVector3 b_scaling = n_OtherBody->getCollisionShape()->getLocalScaling();
	int o_index;
	btCollisionShape* o_shape{};
	btTransform o_trans;
	btMotionState* motion_state_a{};
	btRigidBody* delete_body = n_OtherBody;
	btRigidBody* return_body{};
	btVector3 b_lin_velo;
	btVector3 b_ang_velo;
	btScalar o_friction;
	btScalar o_restitution;

	switch (n_OtherBody->getUserIndex()) {
		//2は何も起きない 2,2の特殊パターン
	case 2:
		return_body = n_OtherBody;
		break;
	case 3:
		n_OtherBody->setFriction(n_OtherBody->getFriction() / fric_coef);
		return_body = n_OtherBody;
		break;
	case 4:
		n_OtherBody->setMassProps(b_mass / mass_coef, b_inertia);
		return_body = n_OtherBody;
		break;
	case 5:
		n_OtherBody->setRestitution(n_OtherBody->getRestitution() / restitution_coef);
		return_body = n_OtherBody;
		break;
	case 6:
		return_body = n_OtherBody;
		break;
	case 7:
		b_lin_velo = n_OtherBody->getLinearVelocity();
		b_ang_velo = n_OtherBody->getAngularVelocity();
		o_index = n_OtherBody->getUserIndex();
		o_trans = n_OtherBody->getWorldTransform();
		o_shape = n_OtherBody->getCollisionShape();
		o_friction = n_OtherBody->getFriction();
		o_restitution = n_OtherBody->getRestitution();
		o_shape->setLocalScaling(b_scaling / expand_coef);
		g_dynamicsworld->removeRigidBody(n_OtherBody);
		motion_state_a = n_OtherBody->getMotionState();
		delete motion_state_a;
		delete delete_body;
		return_body = CreateRigidBody(b_mass, o_trans, o_shape, g_dynamicsworld, &o_index, RX_COL_OBST, RX_COL_ALL, false, b_inertia);
		return_body->setLinearVelocity(b_lin_velo);
		return_body->setAngularVelocity(b_ang_velo);
		return_body->setFriction(o_friction);
		return_body->setRestitution(o_restitution);
		break;
	case 8:
		b_lin_velo = n_OtherBody->getLinearVelocity();
		b_ang_velo = n_OtherBody->getAngularVelocity();
		o_index = n_OtherBody->getUserIndex();
		o_trans = n_OtherBody->getWorldTransform();
		o_shape = n_OtherBody->getCollisionShape();
		o_friction = n_OtherBody->getFriction();
		o_restitution = n_OtherBody->getRestitution();
		o_shape->setLocalScaling(b_scaling / reduce_coef);
		g_dynamicsworld->removeRigidBody(n_OtherBody);
		motion_state_a = n_OtherBody->getMotionState();
		delete motion_state_a;
		delete delete_body;
		return_body = CreateRigidBody(b_mass, o_trans, o_shape, g_dynamicsworld, &o_index, RX_COL_OBST, RX_COL_ALL, false, b_inertia);
		return_body->setLinearVelocity(b_lin_velo);
		return_body->setAngularVelocity(b_ang_velo);
		return_body->setFriction(o_friction);
		return_body->setRestitution(o_restitution);
		break;
	case 9:
		n_OtherBody->setMassProps(b_mass, b_inertia / incr_inertia_coef);
		return_body = n_OtherBody;
		break;
	case 10:
		n_OtherBody->setMassProps(b_mass, b_inertia / decr_inertia_coef);
		return_body = n_OtherBody;
		break;
	default:
		cout << "error\n";
		break;
	}
	return return_body;
}

btSoftBody* AddSoftBodies2D(int& res)
{
	btScalar sl = 1.0;
	btScalar y = 3;
	res = 10;
	btSoftBody* cloth = btSoftBodyHelpers::CreatePatch(
		g_softBodyWorldInfo,
		btVector3(-sl, y, -sl),
		btVector3(-sl, y, sl),
		btVector3(sl, y, -sl),
		btVector3(sl, y, sl),
		res, res, 0, true);
	cloth->getCollisionShape()->setMargin(0.01);
	cloth->setTotalMass(2);
	cloth->m_materials[0]->m_kLST = 0.5;
	cloth->m_cfg.kLF = 0.05;
	cloth->m_cfg.kDG = 0.01;
	cloth->m_cfg.piterations = 2;
	cloth->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSidedLiftDrag;
	cloth->setWindVelocity(btVector3(4.0, 0.0, 0.0));
	g_dynamicsworld->addSoftBody(cloth);
	return cloth;
}

void MoveCameraWithRigid(rxTrackball& view, btRigidBody* body)
{
	btVector3 r = btVector3(0, 1, 4); // 視点の設定

	// 剛体の位置と姿勢(四元数)
	btTransform trans;
	body->getMotionState()->getWorldTransform(trans);
	btVector3 p = trans.getOrigin();
	btQuaternion q = trans.getRotation();
	q = q.inverse();

	// 視点変更用のトラックボールの回転中心が常に原点にあるので，
	// 先に回転させてから，平行移動を行う

	// 視線方向を回転
	double qd[4];
	qd[0] = q[3]; qd[1] = q[0]; qd[2] = q[1]; qd[3] = q[2];
	view.SetQuaternion(qd);

	// 剛体の後ろに視点を移動
	btVector3 epos = quatRotate(q, p) + r;
	view.SetTranslation(-epos[0], -epos[1]);
	view.SetScaling(-epos[2]);
}

//筆(index=98)を引き寄せる
btRigidBody* GetClosestObj(float maxRad) {
	const btCollisionObjectArray& objArray = g_dynamicsworld->getCollisionObjectArray();
	btRigidBody* closestObject = nullptr;
	float minDist = maxRad;

	for (int i = 0; i < objArray.size(); ++i) {
		btRigidBody* body = btRigidBody::upcast(objArray[i]);
		if (body&&body->getUserIndex() == SPECIAL_INDEX) {
			btVector3 diff = player_rigid[0]->getWorldTransform().getOrigin() - body->getWorldTransform().getOrigin();
			float dist = diff.length();
			if (dist < minDist) {
				minDist = dist;
				closestObject = body;
			}
		}
	}

	return minDist <= maxRad ? closestObject : nullptr;
}


btVector3 LocalToWorldTrans(btRigidBody* local_body, btVector3 local_vector)
{
	btTransform trans = local_body->getWorldTransform();
	btQuaternion rot = trans.getRotation().normalize();
	btVector3 world_vector = quatRotate(rot, local_vector);
	return world_vector;
}
