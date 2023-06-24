#include "box2d_area.h"

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

Box2DArea::Box2DArea() :
		Box2DCollisionObject(TYPE_AREA) {
	//_set_static(true); //areas are not active by default
}

Box2DArea::~Box2DArea() {
}

void Box2DArea::set_gravity_override_mode(int32_t p_value) {
	gravity_override_mode = p_value;
}
void Box2DArea::set_gravity(double p_value) {
	gravity = p_value / 100.0f;
}
void Box2DArea::set_gravity_vector(Vector2 p_value) {
	gravity_vector = b2Vec2(p_value.x, p_value.y);
}
void Box2DArea::set_linear_damp_override_mode(int32_t p_value) {
	linear_damp_mode = p_value;
}
void Box2DArea::set_angular_damp_override_mode(int32_t p_value) {
	angular_damp_mode = p_value;
}

int32_t Box2DArea::get_gravity_override_mode() const {
	return gravity_override_mode;
}
double Box2DArea::get_gravity() const {
	return gravity * 100.0f;
}
Vector2 Box2DArea::get_gravity_vector() const {
	return Vector2(gravity_vector.x, gravity_vector.y);
}
int32_t Box2DArea::get_linear_damp_override_mode() const {
	return linear_damp_mode;
}
int32_t Box2DArea::get_angular_damp_override_mode() const {
	return angular_damp_mode;
}