#include "box2d_body.h"
#include "box2d_direct_body_state.h"
#include "box2d_type_conversions.h"

bool Box2DBody::is_active() const { return active; }

// Physics Server

void Box2DBody::set_max_contacts_reported(int32 p_max_contacts_reported) {
	max_contacts_reported = p_max_contacts_reported;
}

int32 Box2DBody::get_max_contacts_reported() {
	return max_contacts_reported;
}

void Box2DBody::set_priority(double p_priority) {
	priority = p_priority;
}

double Box2DBody::get_priority() {
	return priority;
}

void Box2DBody::set_gravity_scale(double p_gravity_scale) {
	p_gravity_scale = body_def->gravityScale;
	if (body) {
		body->SetGravityScale(body_def->gravityScale);
	}
}
void Box2DBody::set_linear_damp(double p_linear_damp) {
	godot_to_box2d(p_linear_damp, body_def->linearDamping);
	if (body) {
		body->SetLinearDamping(body_def->linearDamping);
	}
}
void Box2DBody::set_angular_damp(double p_angular_damp) {
	godot_to_box2d(p_angular_damp, body_def->angularDamping);
	if (body) {
		body->SetAngularDamping(body_def->angularDamping);
	}
}

double Box2DBody::get_gravity_scale() {
	return body_def->gravityScale; // no need to convert
}
double Box2DBody::get_linear_damp() {
	double linear_damp;
	box2d_to_godot(body_def->linearDamping, linear_damp);
	return body_def->linearDamping;
}
double Box2DBody::get_angular_damp() {
	double angular_damp;
	box2d_to_godot(body_def->angularDamping, angular_damp);
	return angular_damp;
}

void Box2DBody::wakeup() {
	if ((!get_space()) || mode == PhysicsServer2D::BODY_MODE_STATIC || mode == PhysicsServer2D::BODY_MODE_KINEMATIC) {
		return;
	}
	set_active(true);
}

void Box2DBody::set_state_sync_callback(const Callable &p_callable) {
	body_state_callback = p_callable;
}

Box2DDirectBodyState *Box2DBody::get_direct_state() {
	if (!direct_state) {
		direct_state = memnew(Box2DDirectBodyState);
		direct_state->body = this;
	}
	return direct_state;
}

void Box2DBody::set_linear_velocity(const Vector2 &p_linear_velocity) {
	b2Vec2 box2d_linear_velocity;
	godot_to_box2d(p_linear_velocity, box2d_linear_velocity);
	if (body) {
		body->SetLinearVelocity(box2d_linear_velocity);
	} else {
		body_def->linearVelocity = box2d_linear_velocity;
	}
}

Vector2 Box2DBody::get_linear_velocity() const {
	b2Vec2 box2d_linear_velocity = body->GetLinearVelocity();
	Vector2 linear_velocity;
	box2d_to_godot(box2d_linear_velocity, linear_velocity);
	return linear_velocity;
}

void Box2DBody::set_angular_velocity(real_t p_angular_velocity) {
	if (body) {
		body->SetAngularVelocity(p_angular_velocity);
	} else {
		body_def->angularVelocity = p_angular_velocity;
	}
}

double Box2DBody::get_angular_velocity() const {
	return body->GetAngularVelocity();
}

void Box2DBody::set_active(bool p_active) {
	if (active == p_active) {
		return;
	}

	active = p_active;

	if (active) {
		if (mode == PhysicsServer2D::BODY_MODE_STATIC) {
			// Static bodies can't be active.
			active = false;
		} else if (get_space()) {
			if (body) {
				body->SetAwake(true);
			}
			get_space()->body_add_to_active_list(&active_list);
		}
	} else if (get_space()) {
		if (body) {
			body->SetAwake(false);
		}
		get_space()->body_remove_from_active_list(&active_list);
	}
}

void Box2DBody::set_mode(PhysicsServer2D::BodyMode p_mode) {
	PhysicsServer2D::BodyMode prev = mode;
	mode = p_mode;

	switch (p_mode) {
		case PhysicsServer2D::BODY_MODE_STATIC: {
			// TODO: other stuff
			body_def->type = b2_staticBody;
			set_active(false);
		} break;
		case PhysicsServer2D::BODY_MODE_KINEMATIC: {
			// TODO: other stuff
			body_def->type = b2_kinematicBody;
			set_active(true); // TODO: consider contacts
		} break;
		case PhysicsServer2D::BODY_MODE_RIGID:
		case PhysicsServer2D::BODY_MODE_RIGID_LINEAR: {
			// TODO: (inverse) mass calculation?
			//_set_static(false);
			body_def->type = b2_dynamicBody;
			set_active(true);
		} break;
	}
}

PhysicsServer2D::BodyMode Box2DBody::get_mode() const {
	return mode;
}

void Box2DBody::set_state(PhysicsServer2D::BodyState p_state, const Variant &p_variant) {
	switch (p_state) {
		case PhysicsServer2D::BODY_STATE_TRANSFORM: {
			if (mode == PhysicsServer2D::BODY_MODE_KINEMATIC) {
				// TODO
			} else if (mode == PhysicsServer2D::BODY_MODE_STATIC) {
				_set_transform(p_variant);
				//_set_inv_transform(get_transform().affine_inverse());
				//wakeup_neighbours();
			} else { // rigid body
				Transform2D t = p_variant;
				t.orthonormalize();
				new_transform = get_transform(); // used as old to compute motion
				if (t == new_transform) {
					break;
				}
				_set_transform(t);
				//_set_inv_transform(get_transform().inverse());
				//_update_transform_dependent();
			}
			wakeup();
		} break;
		case PhysicsServer2D::BODY_STATE_LINEAR_VELOCITY: {
			Vector2 linear_velocity = p_variant;
			set_linear_velocity(linear_velocity);
			wakeup();
		} break;
		case PhysicsServer2D::BODY_STATE_ANGULAR_VELOCITY: {
			float angular_velocity = p_variant;
			set_angular_velocity(angular_velocity);
			wakeup();
		} break;
		case PhysicsServer2D::BODY_STATE_SLEEPING: {
			if (mode == PhysicsServer2D::BODY_MODE_STATIC || mode == PhysicsServer2D::BODY_MODE_KINEMATIC) {
				break;
			}
			bool do_sleep = p_variant;
			if (do_sleep) {
				set_linear_velocity(Vector2());
				set_angular_velocity(0);
				set_active(false);
			} else {
				if (mode != PhysicsServer2D::BODY_MODE_STATIC) {
					set_active(true);
				}
			}
		} break;
		case PhysicsServer2D::BODY_STATE_CAN_SLEEP: {
			can_sleep = p_variant;
			if (mode >= PhysicsServer2D::BODY_MODE_RIGID && !active && !can_sleep) {
				set_active(true);
			}
		} break;
	}
}

Variant Box2DBody::get_state(PhysicsServer2D::BodyState p_state) const {
	switch (p_state) {
		case PhysicsServer2D::BODY_STATE_TRANSFORM: {
			return get_transform();
		} break;
		case PhysicsServer2D::BODY_STATE_LINEAR_VELOCITY: {
			return get_linear_velocity();
		} break;
		case PhysicsServer2D::BODY_STATE_ANGULAR_VELOCITY: {
			return get_angular_velocity();
		} break;
		case PhysicsServer2D::BODY_STATE_SLEEPING: {
			return !is_active();
		}
		case PhysicsServer2D::BODY_STATE_CAN_SLEEP: {
			return can_sleep;
		}
	}
	return Variant();
}

void Box2DBody::set_space(Box2DSpace *p_space) {
	if (get_space()) {
		// TODO: clean up more
		if (active_list.in_list()) {
			get_space()->body_remove_from_active_list(&active_list);
		}
		if (direct_state_query_list.in_list()) {
			get_space()->body_remove_from_state_query_list(&direct_state_query_list);
		}
	}

	_set_space(p_space);

	if (get_space()) {
		// TODO: do more
		if (body) {
			body->SetAwake(active);
		}
		if (active) {
			get_space()->body_add_to_active_list(&active_list);
		}
	}
}

void Box2DBody::after_step() {
	if (body_state_callback.is_valid()) {
		get_space()->body_add_to_state_query_list(&direct_state_query_list);
	}
}

void Box2DBody::call_queries() {
	Variant direct_state = get_direct_state();
	if (body_state_callback.is_valid()) {
		body_state_callback.callv(Array::make(direct_state));
	}
}

void Box2DBody::set_continuous_collision_detection_mode(PhysicsServer2D::CCDMode p_mode) {
	collision_mode = p_mode;
	switch (collision_mode) {
		case PhysicsServer2D::CCD_MODE_DISABLED:
			break;
		case PhysicsServer2D::CCD_MODE_CAST_RAY:
			break; // bullet
		case PhysicsServer2D::CCD_MODE_CAST_SHAPE:
			break; // bullet
	}
}
PhysicsServer2D::CCDMode Box2DBody::get_continuous_collision_detection_mode() const {
	return collision_mode;
}

void Box2DBody::add_collision_exception(Box2DBody *excepted_body) {
	collision_exception.insert(excepted_body);
}
void Box2DBody::remove_collision_exception(Box2DBody *excepted_body) {
	collision_exception.erase(excepted_body);
}
TypedArray<RID> Box2DBody::get_collision_exception() {
	TypedArray<RID> array;
	for (Box2DBody *E : collision_exception) {
		array.append(E->get_self());
	}
	return array;
}

void Box2DBody::add_joint(Box2DJoint *p_joint) {
	joints.insert(p_joint);
}
void Box2DBody::remove_joint(Box2DJoint *p_joint) {
	joints.erase(p_joint);
}

Box2DBody::Box2DBody() :
		Box2DCollisionObject(TYPE_BODY),
		active_list(this),
		direct_state_query_list(this) {
}

Box2DBody::~Box2DBody() {
	if (direct_state) {
		memdelete(direct_state);
	}
}
