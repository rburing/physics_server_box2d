#include "box2d_collision_object.h"
#include "b2_user_settings.h"
#include "box2d_direct_space_state.h"
#include "box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_contact.h>
#include <box2d/b2_fixture.h>

// Direct Body API

void Box2DCollisionObject::reset_mass_properties() {
	mass_data.mass = 1;
	mass_data.I = 0;
	mass_data.center = b2Vec2();
	if (body) {
		body->SetMassData(&mass_data);
	}
}

void Box2DCollisionObject::set_linear_damp(double p_linear_damp) {
	godot_to_box2d(p_linear_damp, body_def->linearDamping);
	if (body) {
		body->SetLinearDamping(body_def->linearDamping);
	}
}
void Box2DCollisionObject::set_angular_damp(double p_angular_damp) {
	godot_to_box2d(p_angular_damp, body_def->angularDamping);
	if (body) {
		body->SetAngularDamping(body_def->angularDamping);
	}
}
double Box2DCollisionObject::get_linear_damp() const{
	return box2d_to_godot(body_def->linearDamping);
}
double Box2DCollisionObject::get_angular_damp() const{
	return box2d_to_godot(body_def->angularDamping);
}

void Box2DCollisionObject::set_priority(double p_priority) {
	priority = p_priority;
}

double Box2DCollisionObject::get_priority() const{
	return priority;
}

double Box2DCollisionObject::get_mass() const{
	return mass_data.mass; // no need to convert
}
double Box2DCollisionObject::get_inertia() const{
	return mass_data.I; // no need to convert
}

void Box2DCollisionObject::set_bounce(double p_bounce) {
	bounce = p_bounce;
	if (body) {
		_clear_fixtures();
		_update_shapes();
	}
}
void Box2DCollisionObject::set_friction(double p_friction) {
	friction = p_friction;
	if (body) {
		_clear_fixtures();
		_update_shapes();
	}
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

double Box2DCollisionObject::get_bounce() const{
	return bounce;
}
double Box2DCollisionObject::get_friction() const{
	return friction;
}

Vector2 Box2DCollisionObject::get_total_gravity() const {
	return Vector2(); // get gravity from area
}

double Box2DCollisionObject::get_total_linear_damp() const {
	return box2d_to_godot(body_def->linearDamping);
}

double Box2DCollisionObject::get_total_angular_damp() const {
	return box2d_to_godot(body_def->linearDamping);
}

Vector2 Box2DCollisionObject::get_center_of_mass() const {
	if (body) {
		return box2d_to_godot(mass_data.center + body->GetPosition());
	} else {
		return box2d_to_godot(mass_data.center + body_def->position);
	}
}

Vector2 Box2DCollisionObject::get_center_of_mass_local() const {
	return box2d_to_godot(mass_data.center);
}

double Box2DCollisionObject::get_inverse_mass() const {
	if (mass_data.mass <= 0) {
		return 0;
	}
	return 1.0 / mass_data.mass;
}
double Box2DCollisionObject::get_inverse_inertia() const {
	if (mass_data.I <= 0) {
		return 0;
	}
	return 1.0 / mass_data.I;
}
void Box2DCollisionObject::set_linear_velocity(const Vector2 &p_linear_velocity) {
	b2Vec2 box2d_linear_velocity = godot_to_box2d(p_linear_velocity);
	if (body) {
		body->SetLinearVelocity(box2d_linear_velocity);
	}
}

Vector2 Box2DCollisionObject::get_linear_velocity() const {
	if (body) {
		return box2d_to_godot(body->GetLinearVelocity());
	}
	return Vector2();
}
void Box2DCollisionObject::set_angular_velocity(double p_velocity) {
	float angularVelocity = godot_to_box2d(p_velocity);
	if (body) {
		body->SetAngularVelocity(angularVelocity);
	}
}
double Box2DCollisionObject::get_angular_velocity() const {
	if (body) {
		return box2d_to_godot(body->GetAngularVelocity());
	}
	return 0;
}
void Box2DCollisionObject::set_transform(const Transform2D &transform) {
	_set_transform(transform);
}
Transform2D Box2DCollisionObject::get_transform() const {
	if (body) {
		return Transform2D(body->GetAngle(), box2d_to_godot(body->GetPosition()));
	} else {
		return Transform2D(body_def->angle, box2d_to_godot(body_def->position));
	}
}
Vector2 Box2DCollisionObject::get_velocity_at_local_position(const Vector2 &p_local_position) const {
	if (body) {
		b2Vec2 velocity = body->GetLinearVelocityFromLocalPoint(godot_to_box2d(p_local_position));
		return box2d_to_godot(velocity);
	}
	return Vector2();
}
void Box2DCollisionObject::apply_central_impulse(const Vector2 &impulse) {
	if (body) {
		body->ApplyLinearImpulseToCenter(godot_to_box2d(impulse), true);
	}
}
void Box2DCollisionObject::apply_impulse(const Vector2 &impulse, const Vector2 &position) {
	if (body) {
		body->ApplyLinearImpulse(godot_to_box2d(impulse), godot_to_box2d(position), true);
	}
}
void Box2DCollisionObject::apply_torque_impulse(double impulse) {
	if (body) {
		body->ApplyTorque(godot_to_box2d(impulse), true);
	}
}
void Box2DCollisionObject::apply_central_force(const Vector2 &force) {
	if (body) {
		body->ApplyForceToCenter(godot_to_box2d(force), true);
	}
}
void Box2DCollisionObject::apply_force(const Vector2 &force, const Vector2 &position) {
	if (body) {
		body->ApplyForce(godot_to_box2d(force), godot_to_box2d(position), true);
	}
}
void Box2DCollisionObject::apply_torque(double torque) {
	if (body) {
		body->ApplyTorque(godot_to_box2d(torque), true);
	}
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
	if (body) {
		body->SetAwake(!enabled);
	}
}
bool Box2DCollisionObject::is_sleeping() const {
	return !body->IsAwake();
}
int32_t Box2DCollisionObject::get_contact_count() const {
	if (!body) {
		return 0;
	}
	b2ContactEdge *contacts = body->GetContactList();
	int32 contacts_count = 0;
	while (contacts) {
		contacts_count += contacts->contact->GetManifold()->pointCount;
		contacts = contacts->next;
	}
	return contacts_count;
}
Vector2 Box2DCollisionObject::get_contact_local_position(int32_t contact_idx) const {
	if (!body) {
		return Vector2();
	}
	b2ContactEdge *contacts = body->GetContactList();
	int32 contacts_count = 0;
	while (contacts) {
		contacts_count += contacts->contact->GetManifold()->pointCount;
		if (contacts_count >= contact_idx) {
			return box2d_to_godot(contacts->contact->GetManifold()->points[contacts_count - contact_idx].localPoint);
		}
		contacts = contacts->next;
	}
	return Vector2();
}
Vector2 Box2DCollisionObject::get_contact_local_normal(int32_t contact_idx) const {
	if (!body) {
		return Vector2();
	}
	b2ContactEdge *contacts = body->GetContactList();
	int32 contacts_count = 0;
	while (contacts) {
		contacts_count += contacts->contact->GetManifold()->pointCount;
		if (contacts_count >= contact_idx) {
			return box2d_to_godot(contacts->contact->GetManifold()->localNormal);
		}
		contacts = contacts->next;
	}
	return Vector2();
}
int32_t Box2DCollisionObject::get_contact_local_shape(int32_t contact_idx) const {
	if (!body) {
		return 0;
	}
	b2ContactEdge *contacts = body->GetContactList();
	int32 contacts_count = 0;
	while (contacts) {
		contacts_count += contacts->contact->GetManifold()->pointCount;
		if (contacts_count >= contact_idx) {
			return contacts->contact->GetFixtureA()->GetUserData().shape_idx;
		}
		contacts = contacts->next;
	}
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
	if (body) {
		_clear_fixtures();
		_update_shapes();
	}
}

uint32_t Box2DCollisionObject::get_collision_layer() const {
	return collision_layer;
}
void Box2DCollisionObject::set_collision_mask(uint32_t layer) {
	collision_mask = layer;
	if (body) {
		_clear_fixtures();
		_update_shapes();
	}
}

uint32_t Box2DCollisionObject::get_collision_mask() const {
	return collision_mask;
}

void Box2DCollisionObject::set_pickable(bool p_pickable) {
	pickable = p_pickable;
	if (body) {
		_clear_fixtures();
		_update_shapes();
	}
}

void Box2DCollisionObject::set_object_instance_id(const ObjectID &p_instance_id) {
	object_instance_id = p_instance_id;
	if (body) {
		_clear_fixtures();
		_update_shapes();
	}
}
ObjectID Box2DCollisionObject::get_object_instance_id() const { return object_instance_id; }

void Box2DCollisionObject::set_canvas_instance_id(const ObjectID &p_instance_id) {
	canvas_instance_id = p_instance_id;
	if (body) {
		_clear_fixtures();
		_update_shapes();
	}
}
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
		if (body) {
			body->DestroyFixture(shape.fixtures[j]);
		}
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
			if (body) {
				body->DestroyFixture(shape.fixtures[j]);
			}
			shape.fixtures.write[j] = nullptr;
		}
		shape.fixtures.clear();
	}
	shapes.remove_at(p_index);

	// TODO: (queue) update
}

void Box2DCollisionObject::_clear_fixtures() {
	for (int i = 0; i < shapes.size(); i++) {
		Shape &shape = shapes.write[i];
		for (int j = 0; j < shape.fixtures.size(); j++) {
			if (body) {
				body->DestroyFixture(shape.fixtures[j]);
			}
			shape.fixtures.write[j] = nullptr;
		}
		shape.fixtures.clear();
	}
}

void Box2DCollisionObject::_set_space(Box2DSpace *p_space) {
	if (space) {
		// NOTE: Remember the transform by copying it from the b2Body to the b2BodyDef.
		if (body) {
			body_def->position = body->GetPosition();
			body_def->angle = body->GetAngle();
		}

		_clear_fixtures();
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
	if (!space || !body) {
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
				fixture_def.filter.maskBits = collision_mask;
				fixture_def.filter.categoryBits = collision_layer;
				fixture_def.friction = friction;
				fixture_def.restitution = bounce;
				fixture_def.isSensor = type == Type::TYPE_AREA;
				fixture_def.userData.shape_idx = i;
				fixture_def.userData.box2d_fixture_idx = j;
				fixture_def.userData.type = s.shape->get_type();
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
	mass_data.mass = 1;
	mass_data.I = 0;
	mass_data.center = b2Vec2();
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
void Box2DCollisionObject::set_b2Body(b2Body *p_body) {
	body = p_body;
	if (body) {
		body->SetMassData(&mass_data);
	}
}

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