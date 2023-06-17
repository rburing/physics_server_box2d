#ifndef BOX2D_AREA_H
#define BOX2D_AREA_H

#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <godot_cpp/variant/variant.hpp>

#include "box2d_collision_object.h"
#include "box2d_space.h"

using namespace godot;

class Box2DArea : public Box2DCollisionObject {
	bool monitorable = false;
	Callable monitor_callback;
	Callable area_monitor_callback;
public:
	// Physics Server
	void set_monitorable(bool monitorable);
	void set_monitor_callback(const Callable &callback);
	void set_area_monitor_callback(const Callable &callback);

	void set_transform(const Transform2D &p_transform) override;

	void set_space(Box2DSpace *p_space) override;

	Box2DArea();
	~Box2DArea();
};

#endif // BOX2D_AREA_H
