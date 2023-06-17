#include "box2d_collision_object.h"
#include "b2_user_settings.h"
#include "box2d_direct_space_state.h"
#include "box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_fixture.h>

// Direct Body API

void Box2DCollisionObject::reset_mass_properties() {
	mass_data.mass = 0;
	mass_data.center = b2Vec2();
	mass_data.I = 0;
	body->ResetMassData();
}

double Box2DCollisionObject::get_mass() {
	return mass_data.mass; // no need to convert
}
double Box2DCollisionObject::get_inertia() {
	return mass_data.I; // no need to convert
}
Vector2 Box2DCollisionObject::get_center_of_mass() {
	Vector2 center;
	box2d_to_godot(mass_data.center, center);
	return center;
}

void Box2DCollisionObject::set_mass(double p_mass) {
	mass_data.mass = p_mass;
	if (body) {
		body->SetMassData(&mass_data);
	}
}
void Box2DCollisionObject::set_inertia(double p_inertia) {
	mass_data.I = p_inertia;
	if (body) {
		body->SetMassData(&mass_data);
	}
}
void Box2DCollisionObject::set_center_of_mass(Vector2 p_center_of_mass) {
	godot_to_box2d(p_center_of_mass, mass_data.center);
	if (body) {
		body->SetMassData(&mass_data);
	}
}

Vector2 Box2DCollisionObject::get_total_gravity() const {
	return Vector2();
}

double Box2DCollisionObject::get_total_linear_damp() const {
	double linear_damping;
	box2d_to_godot(body_def->linearDamping, linear_damping);
	return linear_damping;
}

double Box2DCollisionObject::get_total_angular_damp() const {
	double linear_damping;
	box2d_to_godot(body_def->linearDamping, linear_damping);
	return linear_damping;
}

Vector2 Box2DCollisionObject::get_center_of_mass() const {
	return Vector2();
}

Vector2 Box2DCollisionObject::get_center_of_mass_local() const {
	Vector2 center_local;
	box2d_to_godot(mass_data.center, center_local);
	return center_local;
}

double Box2DCollisionObject::get_inverse_mass() const {
	if (mass_data.mass == 0) {
		return 0;
	}
	return 1.0 / mass_data.mass;
}
double Box2DCollisionObject::get_inverse_inertia() const {
	if (mass_data.I == 0) {
		return 0;
	}
	return 1.0 / mass_data.I;
}
void Box2DCollisionObject::set_linear_velocity(const Vector2 &velocity) {
	godot_to_box2d(velocity, body_def->linearVelocity);
}
Vector2 Box2DCollisionObject::get_linear_velocity() const {
	Vector2 velocity;
	if (body) {
		box2d_to_godot(body->GetLinearVelocity(), velocity);
	}
	box2d_to_godot(body_def->linearVelocity, velocity);
	return velocity;
}
void Box2DCollisionObject::set_angular_velocity(double velocity) {
	godot_to_box2d(velocity, body_def->angularVelocity);
}
double Box2DCollisionObject::get_angular_velocity() const {
	double velocity;
	box2d_to_godot(body_def->angularVelocity, velocity);
	return velocity;
}
void Box2DCollisionObject::set_transform(const Transform2D &transform) {
	_set_transform(transform);
}
Transform2D Box2DCollisionObject::get_transform() const {
	if (body) {
		Vector2 position;
		box2d_to_godot(body->GetPosition(), position);
		return Transform2D(body->GetAngle(), position);
	} else {
		Vector2 position;
		box2d_to_godot(body_def->position, position);
		return Transform2D(body_def->angle, position);
	}
}
Vector2 Box2DCollisionObject::get_velocity_at_local_position(const Vector2 &local_position) const {
	return Vector2();
}
void Box2DCollisionObject::apply_central_impulse(const Vector2 &impulse) {
}
void Box2DCollisionObject::apply_impulse(const Vector2 &impulse, const Vector2 &position) {
}
void Box2DCollisionObject::apply_torque_impulse(double impulse) {
}
void Box2DCollisionObject::apply_central_force(const Vector2 &force) {
}
void Box2DCollisionObject::apply_force(const Vector2 &force, const Vector2 &position) {
}
void Box2DCollisionObject::apply_torque(double torque) {
}
void Box2DCollisionObject::add_constant_central_force(const Vector2 &force) {
	constant_force += force;
	// TODO set position to center
	//constant_force_position = position;
}
void Box2DCollisionObject::add_constant_force(const Vector2 &force, const Vector2 &position) {
	constant_force += force;
	constant_force_position = position;
}
void Box2DCollisionObject::add_constant_torque(double torque) {
	constant_torque += torque;
}
void Box2DCollisionObject::set_constant_force(const Vector2 &force) {
	constant_force = force;
}
Vector2 Box2DCollisionObject::get_constant_force() const {
	return constant_force;
}
void Box2DCollisionObject::set_constant_torque(double torque) {
	constant_torque = torque;
}
double Box2DCollisionObject::get_constant_torque() const {
	return constant_torque;
}
void Box2DCollisionObject::set_sleep_state(bool enabled) {
}
bool Box2DCollisionObject::is_sleeping() const {
	return false;
}
int32_t Box2DCollisionObject::get_contact_count() const {
	return false;
}
Vector2 Box2DCollisionObject::get_contact_local_position(int32_t contact_idx) const {
	return Vector2();
}
Vector2 Box2DCollisionObject::get_contact_local_normal(int32_t contact_idx) const {
	return Vector2();
}
int32_t Box2DCollisionObject::get_contact_local_shape(int32_t contact_idx) const {
	return 0;
}
RID Box2DCollisionObject::get_contact_collider(int32_t contact_idx) const {
	return RID();
}
Vector2 Box2DCollisionObject::get_contact_collider_position(int32_t contact_idx) const {
	return Vector2();
}
uint64_t Box2DCollisionObject::get_contact_collider_id(int32_t contact_idx) const {
	return 0;
}
Object *Box2DCollisionObject::get_contact_collider_object(int32_t contact_idx) const {
	return nullptr;
}
int32_t Box2DCollisionObject::get_contact_collider_shape(int32_t contact_idx) const {
	return 0;
}
Vector2 Box2DCollisionObject::get_contact_collider_velocity_at_position(int32_t contact_idx) const {
	return Vector2();
}
Vector2 Box2DCollisionObject::get_contact_impulse(int32_t contact_idx) const {
	return Vector2();
}
double Box2DCollisionObject::get_step() const {
	Box2DSpace *space = get_space();
	if (!space) {
		return 0;
	}
	return space->get_step();
}
void Box2DCollisionObject::integrate_forces() {
}

PhysicsDirectSpaceState2D *Box2DCollisionObject::get_space_state() {
	if (!direct_space) {
		direct_space = memnew(Box2DDirectSpaceState);
		direct_space->space = space;
	}
	return direct_space;
}

// Physics Server

void Box2DCollisionObject::set_collision_layer(uint32_t layer) {
	collision_layer = layer;
}

uint32_t Box2DCollisionObject::get_collision_layer() const {
	return collision_layer;
}
void Box2DCollisionObject::set_collision_mask(uint32_t layer) {
	collision_mask = layer;
}

uint32_t Box2DCollisionObject::get_collision_mask() const {
	return collision_mask;
}

void Box2DCollisionObject::set_pickable(bool p_pickable) {
	pickable = p_pickable;
}

void Box2DCollisionObject::set_object_instance_id(const ObjectID &p_instance_id) { object_instance_id = p_instance_id; }
ObjectID Box2DCollisionObject::get_object_instance_id() const { return object_instance_id; }

void Box2DCollisionObject::set_canvas_instance_id(const ObjectID &p_instance_id) { canvas_instance_id = p_instance_id; }
ObjectID Box2DCollisionObject::get_canvas_instance_id() const { return canvas_instance_id; }

Box2DSpace *Box2DCollisionObject::get_space() const { return space; }

void Box2DCollisionObject::add_shape(Box2DShape *p_shape, const Transform2D &p_transform, bool p_disabled) {
	Shape s;
	s.shape = p_shape;
	s.xform = p_transform;
	s.disabled = p_disabled;
	shapes.push_back(s);

	// TODO (queue) update
}

void Box2DCollisionObject::set_shape(int p_index, Box2DShape *p_shape) {
	ERR_FAIL_INDEX(p_index, shapes.size());
	//shapes[p_index].shape->remove_owner(this);
	shapes.write[p_index].shape = p_shape;

	// TODO: (queue) update
}

void Box2DCollisionObject::set_shape_transform(int p_index, const Transform2D &p_transform) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	shapes.write[p_index].xform = p_transform;

	// TODO: (queue) update
}

void Box2DCollisionObject::set_shape_disabled(int p_index, bool p_disabled) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	Shape &shape = shapes.write[p_index];
	if (shape.disabled == p_disabled) {
		return;
	}

	shape.disabled = p_disabled;

	if (!space) {
		return;
	}

	for (int j = 0; j < shape.fixtures.size(); j++) {
		body->DestroyFixture(shape.fixtures[j]);
		shape.fixtures.write[j] = nullptr;
	}
	shape.fixtures.clear();

	// TODO: (queue) update
}

void Box2DCollisionObject::set_shape_as_one_way_collision(int p_index, bool enable) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	Shape &shape = shapes.write[p_index];
	if (shape.one_way_collision == enable) {
		return;
	}

	shape.one_way_collision = enable;

	_update_shapes();
}

void Box2DCollisionObject::remove_shape(Box2DShape *p_shape) {
	//remove a shape, all the times it appears
	for (int i = 0; i < shapes.size(); i++) {
		if (shapes[i].shape == p_shape) {
			remove_shape(i);
			i--;
		}
	}
}

void Box2DCollisionObject::remove_shape(int p_index) {
	//remove anything from shape to be erased to end, so subindices don't change
	ERR_FAIL_INDEX(p_index, shapes.size());
	for (int i = p_index; i < shapes.size(); i++) {
		Shape &shape = shapes.write[i];
		for (int j = 0; j < shape.fixtures.size(); j++) {
			// should never get here with a null owner
			body->DestroyFixture(shape.fixtures[j]);
			shape.fixtures.write[j] = nullptr;
		}
		shape.fixtures.clear();
	}
	shapes.remove_at(p_index);

	// TODO: (queue) update
}

void Box2DCollisionObject::_set_space(Box2DSpace *p_space) {
	if (space) {
		// NOTE: Remember the transform by copying it from the b2Body to the b2BodyDef.
		body_def->position = body->GetPosition();
		body_def->angle = body->GetAngle();

		for (int i = 0; i < shapes.size(); i++) {
			Shape &shape = shapes.write[i];
			for (int j = 0; j < shape.fixtures.size(); j++) {
				body->DestroyFixture(shape.fixtures[j]);
				shape.fixtures.write[j] = nullptr;
			}
			shape.fixtures.clear();
		}
		space->remove_object(this);
	}
	space = p_space;
	if (space) {
		space->add_object(this);
		_update_shapes();
	}
}

int Box2DCollisionObject::get_shape_count() const { return shapes.size(); }
Box2DShape *Box2DCollisionObject::get_shape(int p_index) const {
	CRASH_BAD_INDEX(p_index, shapes.size());
	return shapes[p_index].shape;
}

const Transform2D &Box2DCollisionObject::get_shape_transform(int p_index) const {
	CRASH_BAD_INDEX(p_index, shapes.size());
	return shapes[p_index].xform;
}
// MISC

void Box2DCollisionObject::_update_shapes() {
	if (!space) {
		return;
	}

	for (int i = 0; i < shapes.size(); i++) {
		Shape &s = shapes.write[i];
		if (s.disabled) {
			continue;
		}

		//not quite correct, should compute the next matrix..
		//Transform2D xform = transform * s.xform;

		if (s.fixtures.is_empty()) {
			int box2d_shape_count = s.shape->get_b2Shape_count();
			s.fixtures.resize(box2d_shape_count);
			for (int j = 0; j < box2d_shape_count; j++) {
				b2FixtureDef fixture_def;
				fixture_def.shape = s.shape->get_transformed_b2Shape(j, s.xform, s.one_way_collision);
				fixture_def.density = 1.0f;
				fixture_def.isSensor = type == Type::TYPE_AREA;
				fixture_def.userData.shape_idx = i;
				fixture_def.userData.box2d_fixture_idx = j;
				s.fixtures.write[j] = body->CreateFixture(&fixture_def);
			}
		}

		//space->get_broadphase()->move(s.bpid, shape_aabb);
	}
}

Box2DCollisionObject::Box2DCollisionObject(Type p_type) {
	type = p_type;
	body_def = memnew(b2BodyDef);
	body_def->userData.collision_object = this;
}

Box2DCollisionObject::~Box2DCollisionObject() {
	memdelete(body_def);
}

Box2DCollisionObject::Type Box2DCollisionObject::get_type() const { return type; }

void Box2DCollisionObject::set_self(const RID &p_self) { self = p_self; }
RID Box2DCollisionObject::get_self() const { return self; }

b2BodyDef *Box2DCollisionObject::get_b2BodyDef() { return body_def; }
void Box2DCollisionObject::set_b2BodyDef(b2BodyDef *p_body_def) { body_def = p_body_def; }
b2Body *Box2DCollisionObject::get_b2Body() { return body; }
void Box2DCollisionObject::set_b2Body(b2Body *p_body) { body = p_body; }



void Box2DCollisionObject::_set_transform(const Transform2D &p_transform, bool p_update_shapes) {
	if (body) {
		Vector2 pos = p_transform.get_origin();
		b2Vec2 box2d_pos;
		godot_to_box2d(pos, box2d_pos);
		body->SetTransform(box2d_pos, p_transform.get_rotation());
	} else {
		godot_to_box2d(p_transform.get_origin(), body_def->position);
		body_def->angle = p_transform.get_rotation();
	}
	if (p_update_shapes) {
		_update_shapes();
	}
}