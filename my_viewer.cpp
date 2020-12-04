# include "my_viewer.h"

# include <sigogl/ui_button.h>
# include <sigogl/ui_radio_button.h>
# include <sig/sn_primitive.h>
# include <sig/sn_transform.h>
# include <sig/sn_manipulator.h>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
# include <sigogl/ws_run.h>
float bridge_left_angle = 0.0f;
float bridge_right_angle = 0.0f;
float gate_left_angle = 0.0f;
float currentWheelAngle = 0.0f;
float currentFrameAngle = 0.0f;
float currentPedalAngle = 0.0f;
float currentRollAngle = 0.0f;
float currentRotX = 0.0f;
float currentRotY = 0.0f;
float currentRotZ = 0.0f;
float wheelRotSpeed = 1.0f;
float speedX = 0.1f;
float speedZ = 0.0f;
float offset = 0.0;
SnModel* river[8];
SnModel* dock[10];
SnModel* wall[36];
SnModel* ship;
//Front end of bike: front wheel, handlebars
SnManipulator* front_wheel_manip = new SnManipulator;
GsMat front_wheel_transl, front_wheel_rotat, front_wheel_start;

SnManipulator* hb_left_lower_manip = new SnManipulator;
GsMat hb_left_lower_transl, hb_left_lower_rotat, hb_left_lower_start;

SnManipulator* hb_left_upper_manip = new SnManipulator;
GsMat hb_left_upper_transl, hb_left_upper_rotat, hb_left_upper_start;

SnManipulator* hb_right_upper_manip = new SnManipulator;
GsMat hb_right_upper_transl, hb_right_upper_rotat, hb_right_upper_start;

SnManipulator* hb_right_lower_manip = new SnManipulator;
GsMat hb_right_lower_transl, hb_right_lower_rotat, hb_right_lower_start;

SnManipulator* hb_joint_manip = new SnManipulator;
GsMat hb_joint_transl, hb_joint_rotat, hb_joint_start;

SnManipulator* front_bar_manip = new SnManipulator;
GsMat front_bar_transl, front_bar_rotat, front_bar_start;

//Frame of bike:
SnManipulator* frame_top_manip = new SnManipulator;
GsMat frame_top_transl, frame_top_rotat, frame_top_start;

SnManipulator* frame_left_manip = new SnManipulator;
GsMat frame_left_transl, frame_left_rotat, frame_left_start;

SnManipulator* frame_right_manip = new SnManipulator;
GsMat frame_right_transl, frame_right_rotat, frame_right_start;

SnManipulator* rear_left_up_manip = new SnManipulator;
GsMat rear_left_up_transl, rear_left_up_rotat, rear_left_up_start;

SnManipulator* rear_left_low_manip = new SnManipulator;
GsMat rear_left_low_transl, rear_left_low_rotat, rear_left_low_start;

SnManipulator* rear_right_up_manip = new SnManipulator;
GsMat rear_right_up_transl, rear_right_up_rotat, rear_right_up_start;

SnManipulator* rear_right_low_manip = new SnManipulator;
GsMat rear_right_low_transl, rear_right_low_rotat, rear_right_low_start;

SnManipulator* frame_bot_manip = new SnManipulator;
GsMat frame_bot_transl, frame_bot_rotat, frame_bot_start;

SnManipulator* frame_cross_manip = new SnManipulator;
GsMat frame_cross_transl, frame_cross_rotat, frame_cross_start;

SnManipulator* rear_wheel_manip = new SnManipulator;
GsMat rear_wheel_transl, rear_wheel_rotat, rear_wheel_start;

SnManipulator* seat_manip = new SnManipulator;
GsMat seat_transl, seat_rotat, seat_start;

SnManipulator* pedals_cross_manip = new SnManipulator;
GsMat pedals_cross_transl, pedals_cross_rotat, pedals_cross_start;

SnManipulator* left_vert_manip = new SnManipulator;
GsMat left_vert_transl, left_vert_rotat, left_vert_start;

SnManipulator* right_vert_manip = new SnManipulator;
GsMat right_vert_transl, right_vert_rotat, right_vert_start;

SnManipulator* left_horiz_manip = new SnManipulator;
GsMat left_horiz_transl, left_horiz_rotat, left_horiz_start;

SnManipulator* right_horiz_manip = new SnManipulator;
GsMat right_horiz_transl, right_horiz_rotat, right_horiz_start;

SnManipulator* left_pedal_manip = new SnManipulator;
GsMat left_pedal_transl, left_pedal_rotat, left_pedal_start;

SnManipulator* right_pedal_manip = new SnManipulator;
GsMat right_pedal_transl, right_pedal_rotat, right_pedal_start;


SnManipulator* base_left_manip = new SnManipulator;
GsMat base_left_transl, base_left_rotat, base_left_start;

SnManipulator* base_right_manip = new SnManipulator;
GsMat base_right_transl, base_right_rotat, base_right_start;

SnManipulator* bridge_left_joint_manip = new SnManipulator;
GsMat bridge_left_joint_transl, bridge_left_joint_rotat, bridge_left_joint_start;

SnManipulator* bridge_left_joint2_manip = new SnManipulator;
GsMat bridge_left_joint2_transl, bridge_left_joint2_rotat, bridge_left_joint2_start;

SnManipulator* bridge_left_seg1_manip = new SnManipulator;
GsMat bridge_left_seg1_transl, bridge_left_seg1_rotat, bridge_left_seg1_start;

SnManipulator* bridge_left_seg2_manip = new SnManipulator;
GsMat bridge_left_seg2_transl, bridge_left_seg2_rotat, bridge_left_seg2_start;

SnManipulator* bridge_right_joint_manip = new SnManipulator;
GsMat bridge_right_joint_transl, bridge_right_joint_rotat, bridge_right_joint_start;

SnManipulator* bridge_right_joint2_manip = new SnManipulator;
GsMat bridge_right_joint2_transl, bridge_right_joint2_rotat, bridge_right_joint2_start;

SnManipulator* bridge_right_seg1_manip = new SnManipulator;
GsMat bridge_right_seg1_transl, bridge_right_seg1_rotat, bridge_right_seg1_start;

SnManipulator* bridge_right_seg2_manip = new SnManipulator;
GsMat bridge_right_seg2_transl, bridge_right_seg2_rotat, bridge_right_seg2_start;

SnManipulator* sign_base_right_manip = new SnManipulator;
GsMat sign_base_right_transl, sign_base_right_rotat, sign_base_right_start;

SnManipulator* sign_right_manip = new SnManipulator;
GsMat sign_right_transl, sign_right_rotat, sign_right_start;

SnManipulator* sign_right_j_manip = new SnManipulator;
GsMat sign_right_j_transl, sign_right_j_rotat, sign_right_j_start;

SnManipulator* sign_base_left_manip = new SnManipulator;
GsMat sign_base_left_transl, sign_base_left_rotat, sign_base_left_start;

SnManipulator* sign_left_j_manip = new SnManipulator;
GsMat sign_left_j_transl, sign_left_j_rotat, sign_left_j_start;

SnManipulator* sign_left_manip = new SnManipulator;
GsMat sign_left_transl, sign_left_rotat, sign_left_start;

MyViewer::MyViewer(int x, int y, int w, int h, const char* l) : WsViewer(x, y, w, h, l)
{
	_nbut = 0;
	_animating = false;
	bridgeIsDown = true;
	build_ui();
	background(GsColor::darkgray);
	build_frame();
	build_base();
	build_floor();
	build_walls();
	build_bridge();
	build_bike();
	shadow_ship_gr = new SnGroup;
	shadow_ship_gr->separator(true);
	shadow_ship_gr->add(ship_shadow = new SnTransform);
	shadow_ship_gr->add(ship_gr);
	rootg()->add(shadow_ship_gr);

	computeShadow();
}
void MyViewer::build_ui()
{
	UiPanel* p;
	UiManager* uim = WsWindow::uim();
	p = uim->add_panel("", UiPanel::HorizLeft);
	p->add(new UiButton("Animate", EvAnimate));
	p->top()->separate();
	light().position.z = 20.0f;
	light().position.y = 20.0f;
	light().ambient = GsColor::lightblue;
	light().specular = GsColor::blue;
}

void MyViewer::run_animation()
{
	if (_animating) return; // prevent recursive calls
	_animating = true;

	// access the transformations to use
	// (pointers below could have been saved during scene construction for easier access)
	double frdt = 1.0 / 60.0; // delta time to reach given number of frames per second
	double t = 0, lt = 0, t0 = gs_time(), t_up= 0;
	do // run for a while:
	{
		while (t - lt < frdt) { ws_check(); t = gs_time() - t0; } // wait until it is time for next frame
		lt = gs_time() - t0;
		GsMat& m = ship_transf->get();
		GsMat trans;
		trans.translation(GsVec(0.0f, 0.0f, 0.5f));
		m.mult(m, trans);

		gate_left_angle += 0.5f;
		if (gate_left_angle >= 90.0f) {
			gate_left_angle = 90.0f;
		}
	
	
		if (lt > 3.0f && lt < 15.0f) {
			bridge_left_angle -= 0.25f;
			bridgeIsDown = false;
			if (bridge_left_angle <= -90.0f) {
				bridge_left_angle = -90.0f;
			}
		}
		else if (lt >= 15.0f) {
			bridge_left_angle += 0.25f;

			if (bridge_left_angle >= 0.0f) {
				bridge_left_angle = 0.0f;
				bridgeIsDown = true;
			}
			if (bridgeIsDown) {
				gate_left_angle -= 1.0f;
				speedX -= 0.5f;
				currentPedalAngle += 20.0f;
			}
			if (gate_left_angle <= 0.0f) {
				gate_left_angle = 0.0f;
			}

		}
		
		pedals_cross_rotat.rotz(GS_TORAD(currentPedalAngle));
		pedals_cross_manip->initial_mat(pedals_cross_start * pedals_cross_rotat);
		frame_top_transl.translation(speedX, 0, 0);
		frame_top_manip->initial_mat(frame_top_start * frame_top_transl);

		sign_right_j_rotat.rotz(GS_TORAD(gate_left_angle));
		sign_right_j_manip->initial_mat(sign_right_j_start * sign_right_j_rotat);
		bridge_left_joint_rotat.rotz(GS_TORAD(bridge_left_angle));
		bridge_left_joint_manip->initial_mat(bridge_left_joint_start * bridge_left_joint_rotat);
		bridge_right_joint_rotat.rotz(GS_TORAD(-bridge_left_angle));
		bridge_right_joint_manip->initial_mat(bridge_right_joint_start * bridge_right_joint_rotat);
		sign_left_j_rotat.rotz(GS_TORAD(-gate_left_angle));
		sign_left_j_manip->initial_mat(sign_left_j_start * sign_left_j_rotat);
		computeShadow();
		render(); // notify it needs redraw
		ws_check(); // redraw now

	} while (_animating);
	_animating = false;
}
void MyViewer::computeShadow() {

	GsVec q = GsVec(20.0f, 30.0f, 4.0f);

	GsMat s(
		1.0f, (-q.x / q.y), 0.0f, 0.0f,

		0.0f, 0.0f,         0.0f, 0.0f,

		0.0f, (-q.z / q.y), 1.0f, 0.0f,

		0.0f, 0.0f,          0.0f, 1.0f);

	GsMat tr, rot,sc;
	sc.scaling(0.15f);
	ship_shadow->get().mult(ship_shadow->get(), sc);
	tr.translation(GsVec(0, -offset, 0));
	ship_shadow->get().mult(tr, s);
	rot.roty(GS_TORAD(-90.0f));
	ship_shadow->get().mult(rot, s);

}
void MyViewer::build_frame() {
	ship_gr = new SnGroup;
	ship = new SnModel();
	ship->model()->load_obj("../model/SimpleRedShip.obj");
	ship_gr->separator(true);
	ship_gr->add(ship_transf = new SnTransform);
	ship_gr->add(ship);
	ship_transf->get().scaling(0.5f);
	ship->model()->translate(GsVec(0, 0, -350.0f));
	rootg()->add_group(ship_gr);
}

void MyViewer::build_walls() {
	wall_gr = new SnGroup;
	float startPosX = 140.0f;
	float startPosZ = 38.0f;
	wall_gr->add(wall_transf = new SnTransform);
	for (int i = 0; i < 9; i++) {
		wall[i] = new SnModel();
		wall[i]->model()->load_obj("../model/stonewall.obj");
		if (i == 0) {
			wall[i]->model()->translate(GsVec(startPosX, 12.5f, startPosZ));
		}
		else {
			wall[i]->model()->translate(GsVec(startPosX -= 14.0f, 12.5f, startPosZ));
		}
		wall_gr->add(wall[i]);
	}
	startPosX -= 37.5;
	for (int i = 9; i < 18; i++) {
		wall[i] = new SnModel();
		wall[i]->model()->load_obj("../model/stonewall.obj");
		if (i == 0) {
			wall[i]->model()->translate(GsVec(startPosX, 12.5f, startPosZ));
		}
		else {
			wall[i]->model()->translate(GsVec(startPosX -= 14.0f, 12.5f, startPosZ));
		}
		wall_gr->add(wall[i]);
	}
	startPosX = 155.0f;
	for (int i = 18; i < 27; i++) {
		wall[i] = new SnModel();
		wall[i]->model()->load_obj("../model/stonewall.obj");
		if (i == 0) {
			wall[i]->model()->translate(GsVec(startPosX, 12.5f, -startPosZ));
		}
		else {
			wall[i]->model()->translate(GsVec(startPosX -= 14.0f, 12.5f, -startPosZ));
		}
		wall_gr->add(wall[i]);
	}
	startPosX -= 44.0;
	for (int i = 27; i < 36; i++) {
		wall[i] = new SnModel();
		wall[i]->model()->load_obj("../model/stonewall.obj");
		if (i == 0) {
			wall[i]->model()->translate(GsVec(startPosX, 12.5f, -startPosZ));
		}
		else {
			wall[i]->model()->translate(GsVec(startPosX -= 14.0f, 12.5f, -startPosZ));
		}
		wall_gr->add(wall[i]);
	}
	
	rootg()->add_group(wall_gr);
	GsMat trans, rotat;
	for (int i = 0; i < 36; i++) {
		////Code Provided by the Professor in the PA5 Pdf.
		GsModel& m = *wall[i]->model();
		//To reuse G to recompute texture.

		m.G.init();
		GsModel::Group& g = *m.G.push();
		g.fi = 0;
		g.fn = m.F.size();
		g.dmap = new GsModel::Texture;
		g.dmap->fname.set("../textures/brick.jpg");
		m.M.push().init();

		int nv = m.V.size();
		m.T.size(nv);
		m.set_mode(GsModel::Hybrid, GsModel::PerGroupMtl);
		m.textured = true;
	}
		wall_transf->get().scaling(2.0f);
		rotat.roty(GS_TORAD(90.0f));
		//trans.translation(GsVec(-38.0f, 12.5f, -140.0f));
		wall_transf->get().mult(wall_transf->get(), rotat);
}

void MyViewer::build_base() {
	GsModel* base_left = new GsModel;
	SnModel* base_left_model = new SnModel(base_left);
	SnGroup* base_gr = new SnGroup;
	base_left_manip->visible(false);
	base_gr->add(base_left_model);
	base_gr->top<SnModel>()->color(GsColor::gray);
	base_left_manip->child(base_gr);
	base_gr->add(base_right_manip);
	base_left_rotat.rotx(0);
	base_left_transl.translation(0.0f, 0.0f, 0.0f);
	base_left_start = base_left_rotat * base_left_transl;
	base_left_manip->initial_mat(base_left_start);
	base_left->make_box(GsBox(GsPnt(75.0f, 0.0f, -300.0f), GsPnt(150.0f, 25.0f, 300.0f)));
	rootg()->add(base_left_manip);

	GsModel* base_right = new GsModel;
	SnModel* base_right_model = new SnModel(base_right);
	SnGroup* base_gr_r = new SnGroup;
	base_right_manip->visible(false);
	base_gr_r->add(base_right_model);
	base_gr_r->top<SnModel>()->color(GsColor::gray);
	base_right_manip->child(base_gr_r);
	base_right_rotat.rotx(0);
	base_right_transl.translation(-225.0f, 0.0f, 0.0f);
	base_right_start = base_right_rotat * base_right_transl;
	base_right_manip->initial_mat(base_right_start);
	base_right->make_box(GsBox(GsPnt(75.0f, 0.0f, -300.0f), GsPnt(150.0f, 25.0f, 300.0f)));
}

void MyViewer::build_bridge() {
	GsModel* bridge_left_j = new GsModel;
	SnModel* bridge_left_j_model = new SnModel(bridge_left_j);
	SnGroup* bridge_gr_left = new SnGroup;
	bridge_left_joint_manip->visible(false);
	bridge_gr_left->add(bridge_left_j_model);
	bridge_gr_left->top<SnModel>()->color(GsColor::black);
	bridge_left_joint_manip->child(bridge_gr_left);
	bridge_gr_left->add(bridge_left_seg1_manip);

	bridge_left_joint_rotat.roty(GS_TORAD(90.0f));
	bridge_left_joint_transl.translation(37.5f, 12.5f, 0.0f);
	bridge_left_joint_start = bridge_left_joint_rotat * bridge_left_joint_transl;
	bridge_left_joint_manip->initial_mat(bridge_left_joint_start);
	bridge_left_j->make_capsule(GsPnt(0.0f, 0.0f, -15.0f), GsPnt(0.0f, 0.0f, 15.0f), 1.0f, 1.0f, 100, true);
	rootg()->add(bridge_left_joint_manip);

	GsModel* bridge_left_seg1 = new GsModel;
	SnModel* bridge_left_seg1_model = new SnModel(bridge_left_seg1);
	SnGroup* bridge_gr_left_seg1 = new SnGroup;
	bridge_left_seg1_manip->visible(false);
	bridge_gr_left_seg1->add(bridge_left_seg1_model);
	bridge_gr_left_seg1->top<SnModel>()->color(GsColor::brown);
	bridge_left_seg1_manip->child(bridge_gr_left_seg1);
	bridge_left_seg1_rotat.rotx(0);
	bridge_left_seg1_transl.translation(0.0f, 0.0f, 0.0f);
	bridge_left_seg1_start = bridge_left_seg1_rotat * bridge_left_seg1_transl;
	bridge_left_seg1_manip->initial_mat(bridge_left_seg1_start);
	bridge_left_seg1->make_box(GsBox(GsPnt(-37.5f, -5.0f, -15.0f), GsPnt(0.0f, 0.0f, 15.0f)));

	GsModel* bridge_right_j = new GsModel;
	SnModel* bridge_right_j_model = new SnModel(bridge_right_j);
	SnGroup* bridge_gr_right = new SnGroup;
	bridge_right_joint_manip->visible(false);
	bridge_gr_right->add(bridge_right_j_model);
	bridge_gr_right->top<SnModel>()->color(GsColor::black);
	bridge_right_joint_manip->child(bridge_gr_right);
	bridge_gr_right->add(bridge_right_seg1_manip);

	bridge_right_joint_rotat.roty(GS_TORAD(90.0f));
	bridge_right_joint_transl.translation(-37.5f, 12.5f, 0.0f);
	bridge_right_joint_start = bridge_right_joint_rotat * bridge_right_joint_transl;
	bridge_right_joint_manip->initial_mat(bridge_right_joint_start);
	bridge_right_j->make_capsule(GsPnt(0.0f, 0.0f, -15.0f), GsPnt(0.0f, 0.0f, 15.0f), 1.0f, 1.0f, 100, true);
	rootg()->add(bridge_right_joint_manip);

	GsModel* bridge_right_seg1 = new GsModel;
	SnModel* bridge_right_seg1_model = new SnModel(bridge_right_seg1);
	SnGroup* bridge_gr_right_seg1 = new SnGroup;
	bridge_right_seg1_manip->visible(false);
	bridge_gr_right_seg1->add(bridge_right_seg1_model);
	bridge_gr_right_seg1->top<SnModel>()->color(GsColor::brown);
	bridge_right_seg1_manip->child(bridge_gr_right_seg1);
	bridge_right_seg1_rotat.rotx(0);
	bridge_right_seg1_transl.translation(0.0f, 0.0f, 0.0f);
	bridge_right_seg1_start = bridge_right_seg1_rotat * bridge_right_seg1_transl;
	bridge_right_seg1_manip->initial_mat(bridge_right_seg1_start);
	bridge_right_seg1->make_box(GsBox(GsPnt(0.0f, -5.0f, -15.0f), GsPnt(37.5f, 0.0f, 15.0f)));

	GsModel* sign_base_right = new GsModel;
	SnModel* sign_base_right_model = new SnModel(sign_base_right);
	SnGroup* sign_right_gr = new SnGroup;
	sign_base_right_manip->visible(false);
	sign_right_gr->add(sign_base_right_model);
	sign_right_gr->top<SnModel>()->color(GsColor::black);
	sign_base_right_manip->child(sign_right_gr);
	//->add(sign_right_j_manip);
	sign_base_right_rotat.roty(GS_TORAD(0.0f));
	sign_base_right_transl.translation(0.0f, 0.0f, 0.0f);
	sign_base_right_start = sign_base_right_rotat * sign_base_right_transl;
	sign_base_right_manip->initial_mat(sign_base_right_start);
	sign_base_right->make_box(GsBox(GsPnt(15.5f, 10.0f, 37.5f), GsPnt(20.5f, 20.0f, 40.5f)));
	rootg()->add(sign_base_right_manip);

	GsModel* sign_base_right_j = new GsModel;
	SnModel* sign_base_right_j_model = new SnModel(sign_base_right_j);
	SnGroup* sign_j_gr = new SnGroup;
	sign_right_j_manip->visible(false);
	sign_j_gr->add(sign_base_right_j_model);
	sign_j_gr->top<SnModel>()->color(GsColor::black);
	sign_right_j_manip->child(sign_j_gr);
	sign_j_gr->add(sign_right_manip);

	sign_right_j_rotat.roty(GS_TORAD(0.0f));
	//sign_right_j_transl.translation(-37.5f, 17.5f, 0.0f);
	sign_right_j_transl.translation(16.0f, 18.0f, 39.0f);
	sign_right_j_start = sign_right_j_rotat * sign_right_j_transl;
	sign_right_j_manip->initial_mat(sign_right_j_start);
	sign_base_right_j->make_capsule(GsPnt(0.0f, 0.0f, 0.0f), GsPnt(0.0f, 0.0f, 1.0f), 0.5f, 0.5f, 100, true);
	rootg()->add(sign_right_j_manip);
	
	GsModel* sign_right = new GsModel;
	SnModel* sign_right_model = new SnModel(sign_right);
	SnGroup* sign_gr = new SnGroup;
	sign_right_manip->visible(false);
	sign_gr->add(sign_right_model);
	sign_gr->top<SnModel>()->color(GsColor::white);
	sign_right_manip->child(sign_gr);
	sign_right_rotat.roty(GS_TORAD(90.0f));
	sign_right_transl.translation(0.0f, -2.5f, -0.0f);
	sign_right_start = sign_right_rotat * sign_right_transl;
	sign_right_manip->initial_mat(sign_right_start);
	sign_right->make_box(GsBox(GsPnt(0.0f, 0.0f, 0.0f), GsPnt(1.5f, 30.0f, 3.0f)));

	GsModel* sign_base_left = new GsModel;
	SnModel* sign_base_left_model = new SnModel(sign_base_left);
	SnGroup* sign_left_gr = new SnGroup;
	sign_base_left_manip->visible(false);
	sign_left_gr->add(sign_base_left_model);
	sign_left_gr->top<SnModel>()->color(GsColor::black);
	sign_base_left_manip->child(sign_left_gr);
	//sign_left_gr->add(sign_left_j_manip);
	sign_base_left_rotat.roty(GS_TORAD(0.0f));
	sign_base_left_transl.translation(0.0f, 0.0f, 0.0f);
	sign_base_left_start = sign_base_left_rotat * sign_base_left_transl;
	sign_base_left_manip->initial_mat(sign_base_left_start);
	sign_base_left->make_box(GsBox(GsPnt(-20.5f, 10.0f, -40.5f), GsPnt(-15.5f, 20.0f, -37.5f)));
	rootg()->add(sign_base_left_manip);
	
	GsModel* sign_left_j = new GsModel;
	SnModel* sign_left_j_model = new SnModel(sign_left_j);
	SnGroup* sign_j_grleft = new SnGroup;
	sign_left_j_manip->visible(false);
	sign_j_grleft->add(sign_left_j_model);
	sign_j_grleft->top<SnModel>()->color(GsColor::white);
	sign_left_j_manip->child(sign_j_grleft);
	sign_j_grleft->add(sign_left_manip);

	sign_left_j_rotat.roty(GS_TORAD(0.0f));
	//sign_right_j_transl.translation(-37.5f, 17.5f, 0.0f);
	sign_left_j_transl.translation(-16.0f, 18.0f, -39.0f);
	sign_left_j_start = sign_left_j_rotat * sign_left_j_transl;
	sign_left_j_manip->initial_mat(sign_left_j_start);
	sign_left_j->make_capsule(GsPnt(0.0f, 0.0f, 0.0f), GsPnt(0.0f, 0.0f, 1.0f), 0.5f, 0.5f, 100, true);
	rootg()->add(sign_left_j_manip);

	
	GsModel* sign_left = new GsModel;
	SnModel* sign_left_model = new SnModel(sign_left);
	SnGroup* sign_grleft = new SnGroup;
	sign_left_manip->visible(false);
	sign_grleft->add(sign_right_model);
	sign_grleft->top<SnModel>()->color(GsColor::white);
	sign_left_manip->child(sign_gr);
	sign_left_rotat.roty(GS_TORAD(90.0f));
	sign_left_transl.translation(0.0f, -2.5f, -2.5f);
	sign_left_start = sign_left_rotat * sign_left_transl;
	sign_left_manip->initial_mat(sign_left_start);
	sign_left->make_box(GsBox(GsPnt(0.0f, 0.0f, 0.0f), GsPnt(1.5f, 30.0f, 3.0f)));
	

}
void MyViewer::build_floor() {
	floor = new SnGroup;
	for (int i = 0; i < 8; i++) {
		river[i] = new SnModel();
	}
	for (int i = 0; i < 10; i++) {
		dock[i] = new SnModel();
	}
	float width = 50.0f;
	float height = width * 3.0f;
	float widthD = 75.0f;
	float heightD = 75.0f;
	for (int i = 0; i < 8; i++) {
		GsModel& m = *river[i]->model();

		//Vertices
		m.V.push() = GsVec(-height, 0, -width);
		m.V.push() = GsVec(-height, 0, width );
		m.V.push() = GsVec(height, 0, -width);
		m.V.push() = GsVec(-height, 0, width);
		m.V.push() = GsVec(height, 0, width);
		m.V.push() = GsVec(height, 0, -width);
		//Faces
		m.F.push() = GsModel::Face(0, 1, 2);
		m.F.push() = GsModel::Face(2, 3, 4);

		//Normals
		m.N.push() = normalize(GsVec(0, 1, 0));
		m.N.push() = normalize(GsVec(0, 1, 0));
		m.N.push() = normalize(GsVec(0, 1, 0));
		m.N.push() = normalize(GsVec(0, 1, 0));
		m.N.push() = normalize(GsVec(0, 1, 0));
		m.N.push() = normalize(GsVec(0, 1, 0));

		//Textures, code given by Professor
		m.T.push() = GsVec2(0.0f, 1.0f);
		m.T.push() = GsVec2(0.0f, 0.0f);
		m.T.push() = GsVec2(1.0f, 1.0f);
		m.T.push() = GsVec2(0.0f, 0.0f);
		m.T.push() = GsVec2(1.0f, 0.0f);
		m.T.push() = GsVec2(1.0f, 1.0f);

		GsModel::Group& g = *m.G.push();

		g.fi = 0;
		g.fn = m.F.size();
		g.dmap = new GsModel::Texture;
		g.dmap->fname.set("../textures/water.jpg");
		m.M.push().init();
		m.set_mode(GsModel::Smooth, GsModel::PerGroupMtl);
		m.textured = true;
	}
	for (int i = 0; i < 10; i++) {
		GsModel& d = *dock[i]->model();
		//Vertices
		d.V.push() = GsVec(-heightD, 25.8f, -widthD);
		d.V.push() = GsVec(-heightD, 25.8f, widthD);
		d.V.push() = GsVec(heightD, 25.8f, -widthD);
		d.V.push() = GsVec(-heightD, 25.8f, widthD);
		d.V.push() = GsVec(heightD, 25.8f, widthD);
		d.V.push() = GsVec(heightD, 25.8f, -widthD);
		//Faces
		d.F.push() = GsModel::Face(0, 1, 2);
		d.F.push() = GsModel::Face(2, 3, 4);

		//Normals
		d.N.push() = normalize(GsVec(0, 1, 0));
		d.N.push() = normalize(GsVec(0, 1, 0));
		d.N.push() = normalize(GsVec(0, 1, 0));
		d.N.push() = normalize(GsVec(0, 1, 0));
		d.N.push() = normalize(GsVec(0, 1, 0));
		d.N.push() = normalize(GsVec(0, 1, 0));

		//Textures, code given by Professor
		d.T.push() = GsVec2(0.0f, 1.0f);
		d.T.push() = GsVec2(0.0f, 0.0f);
		d.T.push() = GsVec2(1.0f, 1.0f);
		d.T.push() = GsVec2(0.0f, 0.0f);
		d.T.push() = GsVec2(1.0f, 0.0f);
		d.T.push() = GsVec2(1.0f, 1.0f);

		GsModel::Group& g = *d.G.push();

		g.fi = 0;
		g.fn = d.F.size();
		g.dmap = new GsModel::Texture;
		g.dmap->fname.set("../textures/wood.jpg");
		d.M.push().init();
		d.set_mode(GsModel::Smooth, GsModel::PerGroupMtl);
		d.textured = true;
	}
	floor->add(floor_transf = new SnTransform);
	float startPos = -5.0f * width;
	for (int i = 0; i < 6; i++) {

		if (i == 0) {
			river[i]->model()->translate(GsVec( 0, 0, startPos));
		}
		else {
			river[i]->model()->translate(GsVec(0, 0, startPos+= (width * 2.0f)));
		}
		floor->add(river[i]);
	}
	float startPosX = 150.0f;
	float startPosZ = -225.0f;

	for (int i = 0; i < 4; i++) {
		if (i == 0) {
			dock[i]->model()->translate(GsVec(startPosX, 0.0f, startPosZ));
		}
		else {
			dock[i]->model()->translate(GsVec(startPosX, 0.0f, startPosZ += (widthD * 2.0f)));
		}

		floor->add(dock[i]);
	}
	startPosX = 150.0f;
	startPosZ = -225.0f;
	for (int i = 4; i < 8; i++) {
		if (i == 4) {
			dock[i]->model()->translate(GsVec(-startPosX, 0.0f, startPosZ));
		}
		else {
			dock[i]->model()->translate(GsVec(-startPosX, 0.0f, startPosZ += (widthD * 2.0f)));
		}

		floor->add(dock[i]);
	}

	rootg()->add_group(floor);
}
void MyViewer::build_scene() {
}
void MyViewer::build_bike() {
	GsModel* frame_top = new GsModel;
	SnModel* frame_top_snmodel = new SnModel(frame_top);
	SnGroup* frame_top_group = new SnGroup;
	frame_top_group->add(frame_top_snmodel);
	frame_top_group->top<SnModel>()->color(GsColor::black);

	//Adding components to frame-top anchor
	frame_top_manip->child(frame_top_group);
	frame_top_group->add(pedals_cross_manip);
	frame_top_group->add(frame_left_manip);
	frame_top_group->add(front_bar_manip);
	frame_top_group->add(rear_left_up_manip);
	frame_top_group->add(rear_right_up_manip);
	frame_top_group->add(rear_left_low_manip);
	frame_top_group->add(rear_right_low_manip);
	frame_top_group->add(frame_cross_manip);
	frame_top_group->add(rear_wheel_manip);
	frame_top_group->add(seat_manip);

	//frame_top placement
	frame_top_rotat.roty(GS_TORAD(-90.0f));
	frame_top_transl.translation(60.0f, 14.0f, 0);
	frame_top_start = frame_top_rotat * frame_top_transl;
	frame_top_manip->initial_mat(frame_top_start);
	frame_top->make_capsule(GsPnt(0.0f, 2.75f, 0), GsPnt(5.75f, 2.75f, 0), 0.2f, 0.2f, 100, true);
	frame_top_manip->visible(false);
	rootg()->add(frame_top_manip);

	//Frame_left placement
	GsModel* frame_left = new GsModel;
	SnModel* frame_left_snmodel = new SnModel(frame_left);
	SnGroup* frame_left_grp = new SnGroup;
	frame_left_manip->visible(false);
	frame_left_grp->add(frame_left_snmodel);
	frame_left_grp->top<SnModel>()->color(GsColor::black);
	frame_left_manip->child(frame_left_grp);
	frame_left_rotat.rotz(0);
	frame_left_transl.translation(0.0f, 2.75f, 0);
	frame_left_start = frame_left_rotat * frame_left_transl;
	frame_left_manip->initial_mat(frame_left_start);
	frame_left->make_capsule(GsPnt(0, 0, 0), GsPnt(2.5f, -3.0f, 0), 0.2f, 0.2f, 100, true);

	//Rear Left Upper frame placement
	GsModel* rear_left_up = new GsModel;
	SnModel* rear_left_up_snmodel = new SnModel(rear_left_up);
	SnGroup* rear_left_up_grp = new SnGroup;
	rear_left_up_manip->visible(false);
	rear_left_up_grp->add(rear_left_up_snmodel);
	rear_left_up_grp->top<SnModel>()->color(GsColor::black);
	rear_left_up_manip->child(rear_left_up_grp);
	rear_left_up_rotat.rotz(0);
	rear_left_up_transl.translation(5.75f, 2.75f, 0);
	rear_left_up_start = rear_left_up_rotat * rear_left_up_transl;
	rear_left_up_manip->initial_mat(rear_left_up_start);
	rear_left_up->make_capsule(GsPnt(0, 0, 0), GsPnt(2.5f, -3.0f, 0.45f), 0.15f, 0.15f, 100, true);

	//Rear Right Upper frame placement
	GsModel* rear_right_up = new GsModel;
	SnModel* rear_right_up_snmodel = new SnModel(rear_right_up);
	SnGroup* rear_right_up_grp = new SnGroup;
	rear_right_up_manip->visible(false);
	rear_right_up_grp->add(rear_right_up_snmodel);
	rear_right_up_grp->top<SnModel>()->color(GsColor::black);
	rear_right_up_manip->child(rear_right_up_grp);
	rear_right_up_rotat.rotz(0);
	rear_right_up_transl.translation(5.75f, 2.75f, 0);
	rear_right_up_start = rear_left_up_rotat * rear_left_up_transl;
	rear_right_up_manip->initial_mat(rear_right_up_start);
	rear_right_up->make_capsule(GsPnt(0, 0, 0), GsPnt(2.5f, -3.0f, -0.45f), 0.15f, 0.15f, 100, true);

	//Rear left lower frame piece placement
	GsModel* rear_left_low = new GsModel;
	SnModel* rear_left_low_snmodel = new SnModel(rear_left_low);
	SnGroup* rear_left_low_grp = new SnGroup;
	rear_left_low_manip->visible(false);
	rear_left_low_grp->add(rear_left_low_snmodel);
	rear_left_low_grp->top<SnModel>()->color(GsColor::black);
	rear_left_low_manip->child(rear_left_low_grp);
	rear_left_low_rotat.rotz(0);
	rear_left_low_transl.translation(2.5f, -0.325f, 0);
	rear_left_low_start = rear_left_low_rotat * rear_left_low_transl;
	rear_left_low_manip->initial_mat(rear_left_low_start);
	rear_left_low->make_capsule(GsPnt(0, 0, 0), GsPnt(5.75f, 0, 0.45f), 0.15f, 0.15f, 100, true);

	//Rear right lower frame piece placement
	GsModel* rear_right_low = new GsModel;
	SnModel* rear_right_low_snmodel = new SnModel(rear_right_low);
	SnGroup* rear_right_low_grp = new SnGroup;
	rear_right_low_manip->visible(false);
	rear_right_low_grp->add(rear_right_low_snmodel);
	rear_right_low_grp->top<SnModel>()->color(GsColor::black);
	rear_right_low_manip->child(rear_right_low_grp);
	rear_right_low_rotat.rotz(0);
	rear_right_low_transl.translation(2.5f, -0.325f, 0);
	rear_right_low_start = rear_right_low_rotat * rear_right_low_transl;
	rear_right_low_manip->initial_mat(rear_right_low_start);
	rear_right_low->make_capsule(GsPnt(0, 0, 0), GsPnt(5.75f, 0, -0.45f), 0.15f, 0.15f, 100, true);

	//Frame_cross placement
	GsModel* frame_cross = new GsModel;
	SnModel* frame_cross_snmodel = new SnModel(frame_cross);
	SnGroup* frame_cross_grp = new SnGroup;
	frame_cross_manip->visible(false);
	frame_cross_grp->add(frame_cross_snmodel);
	frame_cross_grp->top<SnModel>()->color(GsColor::black);
	frame_cross_manip->child(frame_cross_grp);
	frame_cross_rotat.rotz(0);
	frame_cross_transl.translation(2.5f, 0.0f, 0);
	frame_cross_start = frame_cross_rotat * frame_cross_transl;
	frame_cross_manip->initial_mat(frame_cross_start);
	frame_cross->make_capsule(GsPnt(0, 0, 0), GsPnt(3.25f, 2.75f, 0), 0.2125f, 0.2125f, 100, true);

	//Rear wheel placement
	GsModel* rear_wheel = new GsModel;
	SnModel* rear_wheel_snmodel = new SnModel(rear_wheel);
	SnGroup* rear_wheel_grp = new SnGroup;
	rear_wheel_manip->visible(false);
	rear_wheel_grp->add(rear_wheel_snmodel);
	rear_wheel_grp->top<SnModel>()->color(GsColor::magenta);
	rear_wheel_manip->child(rear_wheel_grp);
	rear_wheel_rotat.rotx(GS_TORAD(90.0f));
	rear_wheel_transl.translation(8.0f, 0, 0);
	rear_wheel_start = rear_wheel_rotat * rear_wheel_transl;
	rear_wheel_manip->initial_mat(rear_wheel_start);
	rear_wheel->make_cylinder(GsPnt(0, -0.125f, 0.0f), GsPnt(0, 0.125f, 0.0f), 1.25f, 1.25f, 100, true);

	//Pedals
	GsModel* pedals_cross = new GsModel;
	SnModel* pedals_cross_snmodel = new SnModel(pedals_cross);
	SnGroup* pedals_cross_group = new SnGroup;
	pedals_cross_group->add(pedals_cross_snmodel);
	pedals_cross_group->top<SnModel>()->color(GsColor::black);
	pedals_cross_manip->child(pedals_cross_group);
	pedals_cross_group->add(left_vert_manip);
	pedals_cross_group->add(right_vert_manip);

	//Pedal cross placement
	pedals_cross_rotat.roty(GS_TORAD(0.0f));
	pedals_cross_transl.translation(2.5f, -0.3f, 0.0f);
	pedals_cross_start = pedals_cross_rotat * pedals_cross_transl;
	pedals_cross_manip->initial_mat(pedals_cross_start);
	pedals_cross->make_capsule(GsPnt(0.0f, 0.0f, 0.375f), GsPnt(0.0f, 0.0f, -0.375f), 0.075f, 0.075f, 100, true);
	pedals_cross_manip->visible(false);

	//left vertical pedal strut
	GsModel* left_vert = new GsModel;
	SnModel* left_vert_snmodel = new SnModel(left_vert);
	SnGroup* left_vert_grp = new SnGroup;
	left_vert_manip->visible(false);
	left_vert_grp->add(left_vert_snmodel);
	left_vert_grp->top<SnModel>()->color(GsColor::white);
	left_vert_manip->child(left_vert_grp);
	left_vert_grp->add(left_horiz_manip);
	left_vert_rotat.rotz(0);
	left_vert_transl.translation(0.0f, 0.0f, 0.375f);
	left_vert_start = left_vert_rotat * left_vert_transl;
	left_vert_manip->initial_mat(left_vert_start);
	left_vert->make_capsule(GsPnt(0, 0, 0), GsPnt(0.0f, 0.5f, 0), 0.075f, 0.075f, 100, true);

	//left horizontal pedal strut
	GsModel* left_horiz = new GsModel;
	SnModel* left_horiz_snmodel = new SnModel(left_horiz);
	SnGroup* left_horiz_grp = new SnGroup;
	left_horiz_manip->visible(false);
	left_horiz_grp->add(left_horiz_snmodel);
	left_horiz_grp->top<SnModel>()->color(GsColor::white);
	left_horiz_manip->child(left_horiz_grp);
	left_horiz_grp->add(left_pedal_manip);
	left_horiz_rotat.rotz(0);
	left_horiz_transl.translation(0.0f, 0.5f, 0.0f);
	left_horiz_start = left_horiz_rotat * left_horiz_transl;
	left_horiz_manip->initial_mat(left_horiz_start);
	left_horiz->make_cylinder(GsPnt(0, 0, 0), GsPnt(0.0f, 0.0f, 0.25f), 0.075f, 0.075f, 100, true);

	//Left pedal
	GsModel* left_pedal = new GsModel;
	SnModel* left_pedal_snmodel = new SnModel(left_pedal);
	SnGroup* left_pedal_grp = new SnGroup;
	left_pedal_manip->visible(false);
	left_pedal_grp->add(left_pedal_snmodel);
	left_pedal_grp->top<SnModel>()->color(GsColor::magenta);
	left_pedal_manip->child(left_pedal_grp);
	left_pedal_rotat.rotx(0);
	left_pedal_transl.translation(-0.2f, -0.1f, 0.25f);
	left_pedal_start = left_pedal_rotat * left_pedal_transl;
	left_pedal_manip->initial_mat(left_pedal_start);
	left_pedal->make_box(GsBox(GsPnt(0.0f, 0.0f, 0.0f), GsPnt(0.375f, 0.225f, 0.5f)));

	//right vertical pedal strut
	GsModel* right_vert = new GsModel;
	SnModel* right_vert_snmodel = new SnModel(right_vert);
	SnGroup* right_vert_grp = new SnGroup;
	right_vert_manip->visible(false);
	right_vert_grp->add(right_vert_snmodel);
	right_vert_grp->top<SnModel>()->color(GsColor::magenta);
	right_vert_manip->child(right_vert_grp);
	right_vert_grp->add(right_horiz_manip);
	right_vert_rotat.rotz(0);
	right_vert_transl.translation(0.0f, -0.5f, -0.375f);
	right_vert_start = right_vert_rotat * right_vert_transl;
	right_vert_manip->initial_mat(right_vert_start);
	right_vert->make_capsule(GsPnt(0, 0, 0), GsPnt(0.0f, 0.5f, 0), 0.075f, 0.075f, 100, true);

	//right horizontal pedal strut
	GsModel* right_horiz = new GsModel;
	SnModel* right_horiz_snmodel = new SnModel(right_horiz);
	SnGroup* right_horiz_grp = new SnGroup;
	right_horiz_manip->visible(false);
	right_horiz_grp->add(right_horiz_snmodel);
	right_horiz_grp->top<SnModel>()->color(GsColor::white);
	right_horiz_manip->child(right_horiz_grp);
	right_horiz_grp->add(right_pedal_manip);
	right_horiz_rotat.rotz(0);
	right_horiz_transl.translation(0.0f, 0.0f, -0.25f);
	right_horiz_start = right_horiz_rotat * right_horiz_transl;
	right_horiz_manip->initial_mat(right_horiz_start);
	right_horiz->make_cylinder(GsPnt(0, 0, 0), GsPnt(0.0f, 0.0f, 0.25f), 0.075f, 0.075f, 100, true);

	//Right pedal
	GsModel* right_pedal = new GsModel;
	SnModel* right_pedal_snmodel = new SnModel(right_pedal);
	SnGroup* right_pedal_grp = new SnGroup;
	right_pedal_manip->visible(false);
	right_pedal_grp->add(right_pedal_snmodel);
	right_pedal_grp->top<SnModel>()->color(GsColor::magenta);
	right_pedal_manip->child(right_pedal_grp);
	right_pedal_rotat.rotx(0);
	right_pedal_transl.translation(-0.2f, -0.1f, -0.5f);
	right_pedal_start = right_pedal_rotat * right_pedal_transl;
	right_pedal_manip->initial_mat(right_pedal_start);
	right_pedal->make_box(GsBox(GsPnt(0.0f, 0.0f, 0.0f), GsPnt(0.375f, 0.225f, 0.5f)));

	//Seat placement
	GsModel* seat = new GsModel;
	SnModel* seat_snmodel = new SnModel(seat);
	SnGroup* seat_grp = new SnGroup;
	seat_manip->visible(false);
	seat_grp->add(seat_snmodel);
	seat_grp->top<SnModel>()->color(GsColor::white);
	seat_manip->child(seat_grp);
	seat_rotat.rotx(0);
	seat_transl.translation(5.0f, 3.125f, 0);
	seat_start = seat_rotat * seat_transl;
	seat_manip->initial_mat(seat_start);
	seat->make_box(GsBox(GsPnt(-0.5f, -0.25f, -0.25f), GsPnt(0.75f, 0, 0.25f)));


	///////////////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
	GsModel* front_bar = new GsModel;
	SnModel* front_bar_snmodel = new SnModel(front_bar);
	SnGroup* front_end_group = new SnGroup;
	front_end_group->add(front_bar_snmodel);
	front_end_group->top<SnModel>()->color(GsColor::black);
	//Adding components to front bar anchor
	front_bar_manip->child(front_end_group);
	front_end_group->add(front_wheel_manip);
	front_end_group->add(hb_joint_manip);

	//front_bar placement
	front_bar_rotat.rotz(GS_TORAD(0.0f));
	front_bar_transl.translation(0, 0.0f, 0);
	front_bar_start = front_bar_rotat * front_bar_transl;
	front_bar_manip->initial_mat(front_bar_start);
	front_bar->make_capsule(GsPnt(0, 0, 0), GsPnt(0.0f, 3.75f, 0), 0.25f, 0.25f, 100, true);
	front_bar_manip->visible(false);
	//	rootg()->add(front_bar_manip);

		//front_wheel placement
	GsModel* front_wheel = new GsModel;
	SnModel* front_wheel_snmodel = new SnModel(front_wheel);
	SnGroup* front_wheel_grp = new SnGroup;
	front_wheel_grp->add(front_wheel_snmodel);
	front_wheel_grp->top<SnModel>()->color(GsColor::magenta);
	front_wheel_manip->child(front_wheel_grp);
	front_wheel_rotat.rotx(GS_TORAD(90.0f));
	front_wheel_transl.translation(0, 0, 0);
	front_wheel_start = front_wheel_rotat * front_wheel_transl;
	front_wheel_manip->initial_mat(front_wheel_start);
	front_wheel->make_cylinder(GsPnt(0, -0.125f, 0.0f), GsPnt(0, 0.125f, 0.0f), 1.25f, 1.25f, 100, true);
	front_wheel_manip->visible(false);

	//handlebar joint placement
	GsModel* hb_joint = new GsModel;
	SnModel* hb_joint_snmodel = new SnModel(hb_joint);
	SnGroup* hb_joint_grp = new SnGroup;
	hb_joint_grp->add(hb_joint_snmodel);
	hb_joint_grp->top<SnModel>()->color(GsColor::black);
	hb_joint_manip->child(hb_joint_grp);
	hb_joint_grp->add(hb_left_lower_manip);
	hb_joint_grp->add(hb_right_lower_manip);
	hb_joint_transl.translation(0.0f, 3.75f, 0);
	hb_joint_rotat.rotz(0);
	hb_joint_start = hb_joint_rotat * hb_joint_transl;
	hb_joint_manip->initial_mat(hb_joint_start);
	hb_joint->make_sphere(GsPnt(0.0f, 0.0f, 0.0f), 0.2625f, 100, true);
	hb_joint_manip->visible(false);

	//lower left handlebar placement
	GsModel* hb_left_lower = new GsModel;
	SnModel* hb_left_lower_snmodel = new SnModel(hb_left_lower);
	SnGroup* hb_left_lower_grp = new SnGroup;
	hb_left_lower_manip->visible(false);
	hb_left_lower_grp->add(hb_left_lower_snmodel);
	hb_left_lower_grp->top<SnModel>()->color(GsColor::black);
	hb_left_lower_manip->child(hb_left_lower_grp);
	hb_left_lower_grp->add(hb_left_upper_manip);
	hb_left_lower_rotat.rotz(0);
	hb_left_lower_transl.translation(0.0f, 0.0f, 0);
	hb_left_lower_start = hb_left_lower_rotat * hb_left_lower_transl;
	hb_left_lower_manip->initial_mat(hb_left_lower_start);
	hb_left_lower->make_capsule(GsPnt(0, 0, 0), GsPnt(0.25f, 0.125f, 0.75f), 0.125f, 0.125f, 100, true);

	//lower right handlebar placement
	GsModel* hb_right_lower = new GsModel;
	SnModel* hb_right_lower_snmodel = new SnModel(hb_right_lower);
	SnGroup* hb_right_lower_grp = new SnGroup;
	hb_right_lower_manip->visible(false);
	hb_right_lower_grp->add(hb_right_lower_snmodel);
	hb_right_lower_grp->top<SnModel>()->color(GsColor::black);
	hb_right_lower_manip->child(hb_right_lower_grp);
	hb_right_lower_grp->add(hb_right_upper_manip);
	hb_right_lower_rotat.rotz(0);
	hb_right_lower_transl.translation(0.0f, 0.0f, 0);
	hb_right_lower_start = hb_right_lower_rotat * hb_right_lower_transl;
	hb_right_lower_manip->initial_mat(hb_right_lower_start);
	hb_right_lower->make_capsule(GsPnt(0, 0, 0), GsPnt(0.25f, 0.125f, -0.75f), 0.125f, 0.125f, 100, true);

	//upper right handlebar placement
	GsModel* hb_right_upper = new GsModel;
	SnModel* hb_right_upper_snmodel = new SnModel(hb_right_upper);
	SnGroup* hb_right_upper_grp = new SnGroup;
	hb_right_upper_manip->visible(false);
	hb_right_upper_grp->add(hb_right_upper_snmodel);
	hb_right_upper_grp->top<SnModel>()->color(GsColor::magenta);
	hb_right_upper_manip->child(hb_right_upper_grp);
	hb_right_upper_rotat.rotz(0);
	hb_right_upper_transl.translation(0.25f, 0.125f, 0.75f);
	hb_right_upper_start = hb_right_upper_rotat * hb_right_upper_transl;
	hb_right_upper_manip->initial_mat(hb_right_upper_start);
	hb_right_upper->make_capsule(GsPnt(0, 0, 0), GsPnt(0.625f, 0.0f, 0.5f), 0.125f, 0.125f, 100, true);

	//upper left hb placement
	GsModel* hb_left_upper = new GsModel;
	SnModel* hb_left_upper_snmodel = new SnModel(hb_left_upper);
	SnGroup* hb_left_upper_grp = new SnGroup;
	hb_left_upper_manip->visible(false);
	hb_left_upper_grp->add(hb_left_upper_snmodel);
	hb_left_upper_grp->top<SnModel>()->color(GsColor::magenta);
	hb_left_upper_manip->child(hb_left_upper_grp);
	hb_left_upper_rotat.rotz(0);
	hb_left_upper_transl.translation(0.25f, 0.125f, -0.75f);
	hb_left_upper_start = hb_left_upper_rotat * hb_left_upper_transl;
	hb_left_upper_manip->initial_mat(hb_left_upper_start);
	hb_left_upper->make_capsule(GsPnt(0, 0, 0), GsPnt(0.625f, 0.0f, -0.5f), 0.125f, 0.125f, 100, true);
}


int MyViewer::handle_keyboard(const GsEvent& e)
{
	int ret = WsViewer::handle_keyboard(e); // 1st let system check events
	if (ret) return ret;

	switch (e.key)
	{
	case GsEvent::KeyEsc: { gs_exit(); return 1; }

	case 'q': {
		bridge_left_angle -= 5.0f;
		bridge_right_angle += 5.0f;
		if (bridge_left_angle <= -90.0f) {
			bridge_left_angle = -90.0f;
		}
		bridge_left_joint_rotat.rotz(GS_TORAD(bridge_left_angle));
		bridge_left_joint_manip->initial_mat(bridge_left_joint_start * bridge_left_joint_rotat);
		
		bridge_right_joint_rotat.rotz(GS_TORAD(-bridge_left_angle));
		bridge_right_joint_manip->initial_mat(bridge_right_joint_start * bridge_right_joint_rotat);
		redraw();
		break;
	}
	case 'a': {
		bridge_left_angle += 5.0f;
		if (bridge_left_angle >= 0.0f) {
			bridge_left_angle = 0.0f;
		}
		bridge_left_joint_rotat.rotz(GS_TORAD(bridge_left_angle));
		bridge_left_joint_manip->initial_mat(bridge_left_joint_start * bridge_left_joint_rotat);
		bridge_right_joint_rotat.rotz(GS_TORAD(-bridge_left_angle));
		bridge_right_joint_manip->initial_mat(bridge_right_joint_start * bridge_right_joint_rotat);
		redraw();
		break;
	}
	case 'w': {
		GsMat& m = ship_transf->get();
		GsMat trans;
		trans.translation(GsVec(0.0f, 0.0f, 5.0f));
		m.mult(m, trans);
		redraw();
		break;
	}
	case 'e': {
		gate_left_angle += 5.0f;
		if (gate_left_angle >= 90.0f) {
			gate_left_angle = 90.0f;
		}
		sign_right_j_rotat.rotz(GS_TORAD(gate_left_angle));
		sign_right_j_manip->initial_mat(sign_right_j_start * sign_right_j_rotat);

		sign_left_j_rotat.rotz(GS_TORAD(-gate_left_angle));
		sign_left_j_manip->initial_mat(sign_left_j_start * sign_left_j_rotat);
		redraw();
		break;
	}
	case 'd': {
		gate_left_angle -= 5.0f;
		if (gate_left_angle <= 0.0f) {
			gate_left_angle = 0.0f;
		}
		sign_right_j_rotat.rotz(GS_TORAD(gate_left_angle));
		sign_right_j_manip->initial_mat(sign_right_j_start * sign_right_j_rotat);

		sign_left_j_rotat.rotz(GS_TORAD(-gate_left_angle));
		sign_left_j_manip->initial_mat(sign_left_j_start * sign_left_j_rotat);
		redraw();
		break;
	}

	case 's': {
		GsMat& m = ship_transf->get();
		GsMat trans;
		trans.translation(GsVec(0.0f, 0.0f, -5.0f));
		m.mult(m, trans);
		redraw();
		break;
	}
	case GsEvent::KeyLeft: {
		currentWheelAngle += 5.0f;
		if (currentWheelAngle >= 45.0f) {
			currentWheelAngle = 45.0f;
		}
		front_bar_rotat.roty(GS_TORAD(currentWheelAngle));
		front_bar_manip->initial_mat(front_bar_start * front_bar_rotat);
		render();
		break;
	}

	case GsEvent::KeyRight: {

		currentWheelAngle -= 5.0f;
		if (currentWheelAngle <= -45.0f) {
			currentWheelAngle = -45.0f;
		}
		front_bar_rotat.roty(GS_TORAD(currentWheelAngle));
		front_bar_manip->initial_mat(front_bar_start * front_bar_rotat);
		redraw();
		break;
	}

	case GsEvent::KeyUp: {
		speedX += 0.5f;
		if (currentWheelAngle != -0) {
			speedZ += 0.5f;
		}
		else speedZ = 0.0f;
		frame_top_rotat.roty(GS_TORAD(0.5f * (currentWheelAngle)));
		//gs_show_console();
		//std::cout << currentWheelAngle << std::endl;
		//currentFrameAngle = 0.5f * (currentWheelAngle - currentFrameAngle);
		currentPedalAngle += 20.0f;
		pedals_cross_rotat.rotz(GS_TORAD(currentPedalAngle));
		pedals_cross_manip->initial_mat(pedals_cross_start* pedals_cross_rotat);
		frame_top_start = frame_top_start * frame_top_rotat;
		frame_top_transl.translation(-(speedX), 0, 0);
		frame_top_manip->initial_mat(frame_top_start * frame_top_transl);
		render();
		break;
	}

	case GsEvent::KeyDown: {
		speedX += 0.5f;
		if (currentWheelAngle != 0) {
			speedZ += 0.5f;
		}
		else speedZ = 0.0f;
		frame_top_rotat.roty(GS_TORAD(0.5f * (currentWheelAngle - currentFrameAngle)));
		currentFrameAngle = 0.5f * (currentWheelAngle - currentFrameAngle);
		frame_top_start = frame_top_start * frame_top_rotat;
		frame_top_transl.translation((speedX), 0, 0);
		frame_top_manip->initial_mat(frame_top_start * frame_top_transl);
		render();
		break;
	}
	case ' ': {
		camera().eye.x = 200.0f;
		camera().eye.y = 150.0f;
		camera().eye.z = 100.0f;
		camera().center.x = 0.0f;
		camera().center.y = 0.0f;
		camera().center.z = 0.0f;
		camera().up.x = 0.0f;
		camera().up.y = 30.0f;
		camera().up.z = 0.0f;
		render();
	}
	}
	return 0;
}
int MyViewer::uievent(int e)
{
	switch (e)
	{
	case EvAnimate: if (_animating)_animating = false; else run_animation(); return 1;
	}
	return WsViewer::uievent(e);
}
