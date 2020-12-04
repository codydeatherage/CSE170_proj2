# pragma once

# include <sig/sn_poly_editor.h>
# include <sig/sn_lines2.h>

# pragma once

# include <sig/sn_poly_editor.h>
# include <sig/sn_lines2.h>
# include <sig/sn_primitive.h>
# include <sigogl/ui_button.h>
# include <sigogl/ws_viewer.h>

// Viewer for this example:
class MyViewer : public WsViewer
{
protected:
	enum MenuEv{EvAnimate};
	UiCheckButton* _nbut;
	SnGroup* floor, *ship_gr, *wall_gr, *light_gr, *shadow_ship_gr;
	SnTransform* ship_transf, *floor_transf, * wall_transf, *ship_shadow;
	bool _animating, bridgeIsDown;
public:
	MyViewer(int x, int y, int w, int h, const char* l);
	void add_model(SnShape* s, GsVec p);
	void build_frame();
	void build_base();
	void build_floor();
	void build_scene();
	void build_walls();
	void build_bridge();
	void build_bike();
	void build_ui();
	void computeShadow();
	virtual int uievent(int e) override;
	void run_animation();
	//void update_shadow();
	virtual int handle_keyboard(const GsEvent& e) override;
};
