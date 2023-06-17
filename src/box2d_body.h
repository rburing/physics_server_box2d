#ifndef BOX2D_BODY_H
#define BOX2D_BODY_H

#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/templates/hash_set.hpp>
#include <godot_cpp/templates/vector.hpp>

#include "box2d_collision_object.h"
#include "box2d_space.h"
#include "box2d_joint.h"

using namespace godot;

class Box2DDirectBodyState;

class Box2DBody : public Box2DCollisionObject {
	PhysicsServer2D::BodyMode mode = PhysicsServer2D::BODY_MODE_RIGID;

	HashSet<Box2DBody *> collision_exception;
	SelfList<Box2DBody> active_list;
	SelfList<Box2DBody> direct_state_query_list;

	bool active = true;
	bool can_sleep = true;
	PhysicsServer2D::CCDMode collision_mode = PhysicsServer2D::CCD_MODE_DISABLED;

	Transform2D new_transform;

	Callable body_state_callback;

	Box2DDirectBodyState *direct_state = nullptr;
	HashSet<Box2DJoint *> joints;
	double priority;
	int32 max_contacts_reported;

	double gravity_scale;
public:
	// Physics Server
	void set_max_contacts_reported(int32 p_max_contacts_reported);
	void set_priority(double p_priority);
	void set_gravity_scale(double p_gravity_scale);
	void set_linear_damp(double p_linear_damp);
	void set_angular_damp(double p_angular_damp);

	int32 get_max_contacts_reported();
	double get_priority();
	double get_gravity_scale();
	double get_linear_damp();
	double get_angular_damp();


	void set_space(Box2DSpace *p_space) override;

	void add_collision_exception(Box2DBody * excepted_body);
	void remove_collision_exception(Box2DBody * excepted_body);
	TypedArray<RID> get_collision_exception();

	void set_state_sync_callback(const Callable &p_callable);

	Box2DDirectBodyState *get_direct_state();

	void set_linear_velocity(const Vector2 &p_linear_velocity) override;
	Vector2 get_linear_velocity() const override;

	void set_angular_velocity(real_t p_angular_velocity);
	double get_angular_velocity() const override;

	void set_active(bool p_active);
	bool is_active() const;

	void wakeup();

	void set_mode(PhysicsServer2D::BodyMode p_mode);
	PhysicsServer2D::BodyMode get_mode() const;

	void set_state(PhysicsServer2D::BodyState p_state, const Variant &p_variant);
	Variant get_state(PhysicsServer2D::BodyState p_state) const;

	void set_continuous_collision_detection_mode(PhysicsServer2D::CCDMode mode);
	PhysicsServer2D::CCDMode get_continuous_collision_detection_mode() const;
	
	void add_joint(Box2DJoint *p_joint);
	void remove_joint(Box2DJoint *p_joint);

	void after_step();
	void call_queries();

	Box2DBody();
	~Box2DBody();
};

#endif // BOX2D_BODY_H
