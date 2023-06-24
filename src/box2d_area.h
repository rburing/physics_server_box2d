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
	int32_t gravity_override_mode = 0;
	double gravity = 10;
	b2Vec2 gravity_vector = b2Vec2(0,-1);
	int32_t linear_damp_mode = 0;
	double linear_damp;
	int32_t angular_damp_mode = 0;
	double angular_damp;
public:
	// Physics Server
	void set_monitorable(bool monitorable);
	void set_monitor_callback(const Callable &callback);
	void set_area_monitor_callback(const Callable &callback);

	void set_transform(const Transform2D &p_transform) override;

	void set_space(Box2DSpace *p_space) override;

	void set_gravity_override_mode(int32_t p_value);
	void set_gravity(double p_value);
	void set_gravity_vector(Vector2 p_value);
	void set_linear_damp_override_mode(int32_t p_value);
	void set_angular_damp_override_mode(int32_t p_value);

	int32_t get_gravity_override_mode() const;
	double get_gravity() const;
	Vector2 get_gravity_vector() const;
	int32_t get_linear_damp_override_mode() const;
	int32_t get_angular_damp_override_mode() const;

	Box2DArea();
	~Box2DArea();
};

#endif // BOX2D_AREA_H
