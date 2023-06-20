#ifndef BOX2D_JOINT_H
#define BOX2D_JOINT_H

#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector2.hpp>

#include <box2d/b2_joint.h>

using namespace godot;

class Box2DBody;

class Box2DJoint {
	RID self;
	bool disable_collisions = false;
	double pin_softness = 0;
	double damped_spring_rest_length = 0;
	double damped_spring_stiffness = 0;
	double damped_spring_damping = 0;
	Box2DBody *body_a = nullptr;
	Box2DBody *body_b = nullptr;

protected:
	bool configured = false;
	PhysicsServer2D::JointType type;

public:
	_FORCE_INLINE_ PhysicsServer2D::JointType get_type() const { return type; }

	_FORCE_INLINE_ void set_self(const RID &p_self) { self = p_self; }
	_FORCE_INLINE_ RID get_self() const { return self; }

	_FORCE_INLINE_ bool is_configured() const { return configured; }
	Box2DBody *get_body_a();
	Box2DBody *get_body_b();
	void set_data(const Variant &p_data);
	Variant get_data() const;
	void clear();
	void set_disable_collisions(bool disable_collisions);
	bool get_disable_collisions();
	void make_pin(const Vector2 &p_anchor, Box2DBody *p_body_a, Box2DBody *p_body_b);
	void make_groove(const Vector2 &p_a_groove1, const Vector2 &p_a_groove2, const Vector2 &p_b_anchor, Box2DBody *p_body_a, Box2DBody *p_body_b);
	void make_damped_spring(const Vector2 &p_anchor_a, const Vector2 &p_anchor_b, Box2DBody *p_body_a, Box2DBody *p_body_b);
	void set_pin_softness(double p_softness);
	double get_pin_softness();
	void set_damped_spring_rest_length(double p_damped_spring_rest_length);
	double get_damped_spring_rest_length();
	void set_damped_spring_stiffness(double p_damped_spring_stiffness);
	double get_damped_spring_stiffness();
	void set_damped_spring_damping(double p_damped_spring_damping);
	double get_damped_spring_damping();
	Box2DJoint() {}
	virtual ~Box2DJoint(){};
};

#endif // BOX2D_JOINT_H
