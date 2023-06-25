#include "box2d_area.h"
#include "box2d_body.h"

// Physics Server
void Box2DArea::set_monitorable(bool p_monitorable) {
	monitorable = p_monitorable;
}
void Box2DArea::set_monitor_callback(const Callable &p_callback) {
	monitor_callback = p_callback;
}
void Box2DArea::set_area_monitor_callback(const Callable &p_callback) {
	area_monitor_callback = p_callback;
}

void Box2DArea::set_transform(const Transform2D &p_transform) {
	// TODO: add to moved list?

	_set_transform(p_transform);
	// _set_inv_transform(p_transform.affine_inverse());
}

void Box2DArea::set_space(Box2DSpace *p_space) {
	// TODO: remove from monitor query list, remove from moved list?

	//monitored_bodies.clear();
	//monitored_areas.clear();

	_set_space(p_space);
}
void Box2DArea::set_priority(real_t p_priority) {
	priority = p_priority;
	for (Box2DCollisionObject *body : bodies) {
		body->recalculate_total_gravity();
		body->recalculate_total_angular_damp();
		body->recalculate_total_linear_damp();
	}
}

void Box2DArea::set_gravity_override_mode(PhysicsServer2D::AreaSpaceOverrideMode p_value) {
	gravity_override_mode = p_value;
	for (Box2DCollisionObject *body : bodies) {
		body->recalculate_total_gravity();
	}
}
void Box2DArea::set_gravity(real_t p_value) {
	gravity = p_value / 100.0f;
	for (Box2DCollisionObject *body : bodies) {
		body->recalculate_total_gravity();
	}
}
void Box2DArea::set_gravity_vector(Vector2 p_value) {
	gravity_vector = b2Vec2(p_value.x, p_value.y);
	for (Box2DCollisionObject *body : bodies) {
		body->recalculate_total_gravity();
	}
}
void Box2DArea::set_linear_damp_override_mode(PhysicsServer2D::AreaSpaceOverrideMode p_value) {
	linear_damp_override_mode = p_value;
	for (Box2DCollisionObject *body : bodies) {
		body->recalculate_total_linear_damp();
	}
}
void Box2DArea::set_angular_damp_override_mode(PhysicsServer2D::AreaSpaceOverrideMode p_value) {
	angular_damp_override_mode = p_value;
	for (Box2DCollisionObject *body : bodies) {
		body->recalculate_total_angular_damp();
	}
}

void Box2DArea::set_linear_damp(real_t p_linear_damp) {
	linear_damp = p_linear_damp;
	for (Box2DCollisionObject *body : bodies) {
		body->recalculate_total_linear_damp();
	}
}
void Box2DArea::set_angular_damp(real_t p_angular_damp) {
	angular_damp = p_angular_damp;
	for (Box2DCollisionObject *body : bodies) {
		body->recalculate_total_angular_damp();
	}
}

PhysicsServer2D::AreaSpaceOverrideMode Box2DArea::get_gravity_override_mode() const {
	return gravity_override_mode;
}
double Box2DArea::get_gravity() const {
	return gravity * 100.0f;
}
b2Vec2 Box2DArea::get_b2_gravity() const {
	return gravity * gravity_vector;
}
Vector2 Box2DArea::get_gravity_vector() const {
	return Vector2(gravity_vector.x, gravity_vector.y);
}
PhysicsServer2D::AreaSpaceOverrideMode Box2DArea::get_linear_damp_override_mode() const {
	return linear_damp_override_mode;
}
PhysicsServer2D::AreaSpaceOverrideMode Box2DArea::get_angular_damp_override_mode() const {
	return angular_damp_override_mode;
}
void Box2DArea::add_body(Box2DCollisionObject *p_body) {
	bodies.append(p_body);
}
void Box2DArea::remove_body(Box2DCollisionObject *p_body) {
	bodies.erase(p_body);
}

Box2DArea::Box2DArea() :
		Box2DCollisionObject(TYPE_AREA) {
	linear_damp = 0.1;
	angular_damp = 1;
	//_set_static(true); //areas are not active by default
}

Box2DArea::~Box2DArea() {
	for (Box2DCollisionObject *body : bodies) {
		if (body) {
			body->remove_area(this);
		}
	}
}